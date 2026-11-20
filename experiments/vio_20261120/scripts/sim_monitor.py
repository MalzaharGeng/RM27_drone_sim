#!/usr/bin/env python3
"""Time-aligned recorder for the pilot-navigation SITL evaluation.

Records PX4 mode/state, EKF local position, gz truth pose, BT phase/reason,
trajectory setpoints and OffboardControlMode counts, plus any
/fmu/in/vehicle_command messages (expected: none from the behaviour tree).
"""
import argparse
import csv
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage
from px4_msgs.msg import (VehicleStatus, VehicleLocalPosition, ManualControlSwitches,
                          VehicleLandDetected, EstimatorStatusFlags,
                          OffboardControlMode, TrajectorySetpoint, VehicleCommand)
from std_msgs.msg import String

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=1)


class Monitor(Node):
    def __init__(self, args):
        super().__init__("pilot_sim_monitor")
        self.args = args
        self.start = time.monotonic()
        self.status = self.local = self.switches = self.land = self.flags = None
        self.bt = {}
        self.events = []
        self.sp = None
        self.sp_rx = None
        self.ocm = self.sp_count = self.cmd_count = 0
        self.commands = []
        self.truth = None
        self.truth_origin = args.origin
        p = args.topic_prefix
        for cls, name, fn in (
                (VehicleStatus, "vehicle_status", self.on_status),
                (VehicleLocalPosition, "vehicle_local_position", self.on_local),
                (ManualControlSwitches, "manual_control_switches", self.on_switches),
                (VehicleLandDetected, "vehicle_land_detected", self.on_land),
                (EstimatorStatusFlags, "estimator_status_flags", self.on_flags),
                (OffboardControlMode, "offboard_control_mode", self.on_ocm),
                (TrajectorySetpoint, "trajectory_setpoint", self.on_sp),
                (VehicleCommand, "vehicle_command", self.on_command)):
            version = getattr(cls, "MESSAGE_VERSION", 0)
            topic = p + "/fmu/" + ("out/" if cls not in (OffboardControlMode, TrajectorySetpoint, VehicleCommand) else "in/") + name
            if version:
                topic += f"_v{version}"
            self.create_subscription(cls, topic, fn, QOS)
        self.create_subscription(String, p + "/navigation/task_status", self.on_bt, 1)
        self.create_subscription(String, p + "/navigation/task_event", self.on_event, 10)
        self.create_subscription(TFMessage, args.tf_topic, self.on_tf, QOS)
        self.csv = open(args.log, "w", newline="")
        self.writer = csv.writer(self.csv)
        self.writer.writerow([
            "wall_us", "mono_s", "px4_nav", "px4_intended", "armed", "failsafe", "landed",
            "x", "y", "z", "vx", "vy", "vz", "heading",
            "truth_n", "truth_e", "truth_d", "err_n", "err_e", "err_d",
            "sp_n", "sp_e", "sp_d", "target_n", "target_e", "target_d",
            "bt_phase", "bt_reason", "offboard_ready", "prestream_s", "task_completed",
            "control_active", "health_ready", "health_reason", "ev_pos", "ev_hgt",
            "rc_slot", "ocm_total", "sp_total", "cmd_total", "sp_age_s"])
        self.timer = self.create_timer(0.1, self.record)
        self.last_print = 0.0
        self.get_logger().info("monitor: truth=" + args.tf_topic + " log=" + args.log)

    def on_status(self, m):
        self.status = m

    def on_local(self, m):
        self.local = m

    def on_switches(self, m):
        self.switches = m

    def on_land(self, m):
        self.land = m

    def on_flags(self, m):
        self.flags = m

    def on_ocm(self, m):
        self.ocm += 1

    def on_sp(self, m):
        self.sp = (float(m.position[0]), float(m.position[1]), float(m.position[2]))
        self.sp_rx = time.monotonic()
        self.sp_count += 1

    def on_command(self, m):
        self.cmd_count += 1
        self.commands.append(dict(wall_us=time.time_ns() // 1000,
                                  command=int(m.command), param1=float(m.param1),
                                  source_system=int(m.source_system),
                                  source_component=int(m.source_component)))

    def on_bt(self, m):
        try:
            self.bt = json.loads(m.data)
        except (ValueError, TypeError):
            pass

    def on_event(self, m):
        try:
            p = json.loads(m.data)
        except (ValueError, TypeError):
            return
        self.events.append(dict(wall_us=time.time_ns() // 1000, **p))
        self.get_logger().info("BT EVENT " + json.dumps(p, ensure_ascii=False))

    def on_tf(self, msg):
        for tr in msg.transforms:
            if self.args.model in tr.child_frame_id.lstrip("/"):
                if tr.child_frame_id.rstrip("/").endswith("base_link") or self.truth is None:
                    t = tr.transform.translation
                    self.truth = (t.x, t.y, t.z)

    def record(self):
        now_wall = time.time_ns() // 1000
        now_mono = time.monotonic()
        st, lo = self.status, self.local
        truth = self.truth
        tn = te = td = float("nan")
        if truth is not None:
            tn = truth[1] - self.truth_origin[1]
            te = truth[0] - self.truth_origin[0]
            td = -(truth[2] - self.truth_origin[2])
        x = y = z = vx = vy = vz = hdg = float("nan")
        err = [float("nan")] * 3
        if lo is not None:
            x, y, z = float(lo.x), float(lo.y), float(lo.z)
            vx, vy, vz = float(lo.vx), float(lo.vy), float(lo.vz)
            hdg = float(lo.heading)
            err = [x - tn, y - te, z - td]
        tgt = self.bt.get("target") or [float("nan")] * 3
        sp = self.sp or (float("nan"),) * 3
        self.writer.writerow([
            now_wall, now_mono,
            st.nav_state if st else "", st.nav_state_user_intention if st else "",
            st.arming_state == 2 if st else "", st.failsafe if st else "",
            self.land.landed if self.land else "",
            x, y, z, vx, vy, vz, hdg,
            tn, te, td, *err,
            *sp, *tgt,
            self.bt.get("phase", ""), self.bt.get("reason", ""),
            self.bt.get("offboard_ready", ""), self.bt.get("prestream_seconds", ""),
            self.bt.get("task_completed", ""), self.bt.get("control_active", ""),
            (self.bt.get("ages", {}).get("health") is not None and
             self.bt.get("reason") != "vio_unready"),
            self.bt.get("reason", ""),
            self.flags.cs_ev_pos if self.flags else "", self.flags.cs_ev_hgt if self.flags else "",
            self.switches.mode_slot if self.switches else "",
            self.ocm, self.sp_count, self.cmd_count,
            (now_mono - self.sp_rx) if self.sp_rx else ""])
        self.csv.flush()
        if now_mono - self.last_print >= 5.0:
            self.last_print = now_mono
            self.get_logger().info(
                "t=%.1f mode=%s/%s armed=%s pos=(%.2f,%.2f,%.2f) truth=(%.2f,%.2f,%.2f) "
                "err=(%.2f,%.2f,%.2f) bt=%s/%s ready=%s done=%s ocm=%d sp=%d cmd=%d"
                % (now_mono - self.start,
                   st.nav_state if st else "?", st.nav_state_user_intention if st else "?",
                   st.arming_state == 2 if st else "?",
                   x, y, z, tn, te, td, *err,
                   self.bt.get("phase", "?"), self.bt.get("reason", "?"),
                   self.bt.get("offboard_ready", "?"), self.bt.get("task_completed", "?"),
                   self.ocm, self.sp_count, self.cmd_count))

    def close(self):
        self.csv.close()
        if self.commands:
            with open(self.args.log + ".commands.json", "w") as f:
                json.dump(self.commands, f, indent=2)
        with open(self.args.log + ".events.json", "w") as f:
            json.dump(self.events, f, indent=2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", required=True)
    ap.add_argument("--tf-topic", default="/world/rm27_world/dynamic_pose/info")
    ap.add_argument("--model", default="Drone-Harrier")
    ap.add_argument("--origin", nargs=3, type=float, default=[5.0, 6.0, 0.5])
    ap.add_argument("--topic-prefix", default="")
    args = ap.parse_args()
    rclpy.init()
    n = Monitor(args)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.close()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
