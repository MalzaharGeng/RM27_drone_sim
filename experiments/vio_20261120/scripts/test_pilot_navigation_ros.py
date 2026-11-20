"""Real ROS/DDS mock transport tests, isolated domain 44; no PX4 connection."""
import json
import math
import os
import time
import unittest
from types import SimpleNamespace

os.environ["ROS_DOMAIN_ID"] = "44"
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import (VehicleStatus, VehicleLocalPosition, ManualControlSetpoint,
                          ManualControlSwitches, VehicleLandDetected, VehicleCommand,
                          TrajectorySetpoint, OffboardControlMode)
from std_msgs.msg import String
import pilot_navigation_bt as bt
from test_pilot_authority import Fixture

bt.ABORT_FILE = "/tmp/pilot_mock_unused_abort_" + str(os.getpid())
PREFIX = "/pilot_test"


class TaskGeometryTest(unittest.TestCase):
    def test_hold_transition_does_not_jump_setpoint_to_target(self):
        args = bt.parse_args([])
        state = SimpleNamespace(args=args, yaw=0., home=(0., 0., -1.), pos=(10., 0., -1.),
                                sp=(9., 0., -1.), vel=(0., 0., 0.), last_tick=0., phase="NAVIGATE")
        tree = bt.build_tree(args)
        for i in range(1, 110):
            now = i*.05
            previous = state.sp
            tree.tick(state, now)
            self.assertLessEqual(math.dist(previous, state.sp), args.speed*.05+1e-10)
            state.last_tick = now
        self.assertEqual(state.phase, "HOLD")

    def test_ten_metres_from_nonzero_anchor_and_heading(self):
        args = bt.parse_args([])
        state = SimpleNamespace(args=args, yaw=math.pi/2, home=(12., -7., -1.5),
                                pos=(12., -7., -1.5), sp=(12., -7., -1.5),
                                target=None, vel=(0., 0., 0.), last_tick=0., phase="NAVIGATE")
        tree = bt.build_tree(args)
        for i in range(1, 900):
            now = i*.05
            previous = state.sp
            tree.tick(state, now)
            self.assertLessEqual(math.dist(previous, state.sp), .25*.05+1e-10)
            self.assertAlmostEqual(state.sp[2], -1.5)
            state.pos = state.sp
            state.last_tick = now
        self.assertEqual(state.phase, "HOLD")
        self.assertAlmostEqual(state.target[0], 12.)
        self.assertAlmostEqual(state.target[1], 3.)
        self.assertAlmostEqual(math.dist(state.target, state.home), 10.)

    def test_arrival_requires_continuous_low_speed(self):
        args = bt.parse_args([])
        state = SimpleNamespace(args=args, yaw=0., home=(0., 0., -1.), pos=(10., 0., -1.),
                                sp=(10., 0., -1.), vel=(0., 0., 0.), last_tick=1.)
        action = bt.GotoPoint(10., 0.)
        self.assertEqual(action.tick(state, 1.), bt.RUNNING)
        state.vel = (.3, 0., 0.)
        self.assertEqual(action.tick(state, 2.), bt.RUNNING)
        state.vel = (0., 0., 0.)
        self.assertEqual(action.tick(state, 3.), bt.RUNNING)
        self.assertEqual(action.tick(state, 4.9), bt.RUNNING)
        self.assertEqual(action.tick(state, 5.1), bt.SUCCESS)


class TransportTest(unittest.TestCase):
    def test_boot_wait_has_no_ten_minute_timeout(self):
        self.node.args.pilot_timeout = 0
        self.node.started_at = time.monotonic()-86400
        self.node.on_tick()
        self.assertFalse(self.node.done)
        self.assertFalse(self.setpoints)

    def test_three_position_boot_sequence_and_takeover(self):
        from pilot_authority import PilotAuthority
        self.node.authority = PilotAuthority(three_position=True)
        self.physical_slot = 6
        self.mode = 1
        self.wait(lambda: self.node.authority.manual_slots.get(1) == 6)
        self.assertFalse(self.setpoints)
        self.physical_slot = 3
        self.mode = 2
        self.wait(lambda: self.node.authority.manual_slots.get(2) == 3)
        self.wait(lambda: self.node.offboard_ready)
        self.assertFalse(self.setpoints)
        self.physical_slot = 1
        self.mode = 14
        self.wait(lambda: bool(self.setpoints))
        self.assertTrue(self.node.authority.started)
        self.physical_slot = 6
        self.wait(lambda: self.node.done)
        self.assert_quiet()

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.args = bt.parse_args(["--topic-prefix", PREFIX])
        self.node = bt.SquareBT(self.args)
        self.peer = rclpy.create_node("pilot_authority_test_peer")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.peer)
        self.mode, self.selected, self.switch_stream, self.health_stream = 2, False, True, True
        self.health = Fixture().payloads["health"]
        self.health["timestamp_us"] = 0
        self.ocm, self.setpoints, self.commands = [], [], []
        self.statuses, self.events = [], []
        self.real_cadence = False
        self.sent = {}
        self.rc_sample = 0
        self.physical_slot = None
        self.last_mode = self.last_selected = None
        self.peer.create_subscription(String, PREFIX+"/navigation/task_status",
                                      lambda m: self.statuses.append(json.loads(m.data)), 1)
        self.peer.create_subscription(String, PREFIX+"/navigation/task_event",
                                      lambda m: self.events.append(json.loads(m.data)), 10)
        self.pubs = {}
        for cls, name in ((VehicleStatus, "vehicle_status"), (VehicleLocalPosition, "vehicle_local_position"),
                          (ManualControlSetpoint, "manual_control_setpoint"), (ManualControlSwitches, "manual_control_switches"),
                          (VehicleLandDetected, "vehicle_land_detected")):
            self.pubs[cls] = self.peer.create_publisher(cls, self.topic("out/"+name, cls), qos_profile_sensor_data)
        self.health_pub = self.peer.create_publisher(String, PREFIX+"/vio/navigation_health", 1)
        for cls, name, received in ((TrajectorySetpoint, "trajectory_setpoint", self.setpoints),
                                     (OffboardControlMode, "offboard_control_mode", self.ocm),
                                     (VehicleCommand, "vehicle_command", self.commands)):
            self.peer.create_subscription(cls, self.topic("in/"+name, cls), received.append, qos_profile_sensor_data)
        self.timer = self.peer.create_timer(.03, self.emit)

    @staticmethod
    def topic(base, cls):
        v = getattr(cls, "MESSAGE_VERSION", 0)
        return PREFIX+"/fmu/"+base+(f"_v{v}" if v else "")

    def emit(self):
        now = time.time_ns()//1000
        mono = time.monotonic()
        def due(key, interval):
            if not self.real_cadence or mono-self.sent.get(key, 0) >= interval:
                self.sent[key] = mono
                return True
            return False
        if due("status", .5) or self.mode != self.last_mode:
            self.last_mode = self.mode
            self.pubs[VehicleStatus].publish(VehicleStatus(timestamp=now, arming_state=2,
            nav_state=self.mode, nav_state_user_intention=self.mode, failsafe=False))
        if due("local", .1):
            self.pubs[VehicleLocalPosition].publish(VehicleLocalPosition(timestamp=now, timestamp_sample=now,
            x=2., y=3., z=-1., heading=math.pi/2, xy_valid=True, z_valid=True, v_xy_valid=True,
            v_z_valid=True, heading_good_for_control=True))
        if now-self.rc_sample >= 350000:
            self.rc_sample = now
        if due("rc", .2):
            self.pubs[ManualControlSetpoint].publish(ManualControlSetpoint(timestamp=now,
                timestamp_sample=self.rc_sample, valid=True, data_source=1))
        if self.switch_stream and (due("switches", 1.) or self.selected != self.last_selected):
            self.last_selected = self.selected
            self.pubs[ManualControlSwitches].publish(ManualControlSwitches(timestamp=now, timestamp_sample=now,
                offboard_switch=1 if self.selected else 3,
                mode_slot=self.physical_slot if self.physical_slot is not None else 2))
        if due("land", 1.):
            self.pubs[VehicleLandDetected].publish(VehicleLandDetected(timestamp=now, landed=False, freefall=False))
        self.health["timestamp_us"] = now
        if self.health_stream and due("health", .2):
            self.health_pub.publish(String(data=json.dumps(self.health)))

    def spin(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.01)

    def wait(self, predicate, seconds=3.):
        end = time.monotonic()+seconds
        while not predicate() and time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.01)
        self.assertTrue(predicate(), self.node.authority.reason)

    def start(self):
        self.wait(lambda: self.statuses and self.statuses[-1]["offboard_ready"], seconds=5.)
        self.assertFalse(self.setpoints, "trajectory before physical Offboard")
        self.mode, self.selected = 14, True
        self.wait(lambda: len(self.setpoints) >= 3)

    def test_real_cadence_and_repeated_rc_samples_continue_task(self):
        self.real_cadence = True
        self.start()
        count = len(self.setpoints)
        self.spin(3.)
        self.assertFalse(self.node.done, self.node.authority.reason)
        self.assertGreater(len(self.setpoints), count+40)
        self.selected = False
        self.wait(lambda: self.node.done)
        self.assert_quiet()

    def test_early_selector_requires_cycle_after_prestream_ready(self):
        self.wait(lambda: len(self.ocm) >= 2)
        self.selected = True
        self.spin(1.4)
        self.assertFalse(self.node.authority.started)
        self.assertFalse(self.setpoints)
        self.assertFalse(self.node.offboard_ready)
        self.selected = False
        self.start()

    def test_arrival_emits_completion_once_and_keeps_hold_active(self):
        self.node.tree = bt.Sequence([bt.GotoPoint(.05, 0.), bt.Hold()])
        self.start()
        self.wait(lambda: any(p["task_completed"] for p in self.statuses))
        self.assertFalse(self.node.done)
        self.assertEqual(self.node.phase, "HOLD")
        before = len(self.setpoints)
        self.spin(.4)
        self.assertGreater(len(self.setpoints), before)
        self.assertEqual(sum(p["event"] == "task_completed" for p in self.events), 1)
        self.assertTrue(self.statuses[-1]["control_active"])

    def test_exact_rc_publication_duplicate_does_not_revoke(self):
        self.start()
        m = ManualControlSetpoint(timestamp=self.node.authority.source["rc"],
            timestamp_sample=self.node.authority.samples["rc"], valid=True, data_source=1)
        self.node.on_rc(m)
        self.assertFalse(self.node.done)

    def assert_quiet(self):
        self.spin(.15)  # drain DDS packets already published before cancellation
        count = len(self.setpoints), len(self.ocm)
        self.node.publish_stream()  # direct call cannot bypass final boundary
        self.spin(.4)
        self.assertEqual(count, (len(self.setpoints), len(self.ocm)))
        self.assertFalse(self.commands)

    def test_selector_takeover_stops_without_mode_or_land_commands(self):
        self.start()
        self.assertAlmostEqual(self.node.target[0], 2., places=6)
        self.assertAlmostEqual(self.node.target[1], 13., places=6)
        self.assertEqual(self.node.target[2], -1.)
        self.selected = False  # physical selector changes before commander publishes mode
        self.wait(lambda: self.node.done)
        self.assertIsNone(self.node.sp)
        self.assert_quiet()
        self.mode, self.selected = 14, True
        self.spin(.3)
        self.assert_quiet()

    def test_actual_altitude_takeover(self):
        self.start()
        self.mode = 1
        self.wait(lambda: self.node.done)
        self.assert_quiet()

    def test_actual_position_takeover(self):
        self.start()
        self.mode = 2
        self.wait(lambda: self.node.done)
        self.assert_quiet()

    def test_health_publisher_loss_stops_old_setpoints(self):
        self.start()
        self.health_stream = False
        self.wait(lambda: self.node.done)
        self.assertEqual(self.node.authority.reason, "stale_health")
        self.assert_quiet()

    def test_relocalization_reference_change_revokes(self):
        self.start()
        self.health["reference_epoch"] += 1
        self.wait(lambda: self.node.done)
        self.assertEqual(self.node.authority.reason, "reference_changed")
        self.assert_quiet()

    def test_switch_stream_missing_cannot_enable_control(self):
        self.switch_stream = False
        self.spin(1.)
        self.mode, self.selected = 14, True
        self.spin(.4)
        self.assertFalse(self.setpoints)
        self.assertFalse(self.ocm)
        self.assertFalse(self.commands)

    def test_dry_run_never_publishes_even_with_all_inputs(self):
        self.node.args.dry_run = True
        self.spin(.8)
        self.mode, self.selected = 14, True
        self.spin(.5)
        self.assertFalse(self.setpoints or self.ocm or self.commands)

    def tearDown(self):
        self.executor.remove_node(self.node)
        self.executor.remove_node(self.peer)
        self.executor.shutdown()
        self.node.destroy_node()
        self.peer.destroy_node()


if __name__ == "__main__":
    unittest.main(verbosity=2)
