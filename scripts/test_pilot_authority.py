"""No ROS or aircraft required: python3 -m unittest test_pilot_authority -v."""
import copy
import math
import unittest
from pilot_authority import PilotAuthority, TOKEN_KEYS


class Fixture:
    def __init__(self, slot=0):
        self.gate = PilotAuthority(slot)
        self.now, self.wall = 10., 1800000000000000
        self.payloads = dict(
            status=dict(armed=True, nav_state=2, intended_mode=2, failsafe=False),
            rc=dict(valid=True, data_source=1),
            switches=dict(mode_slot=2, offboard_switch=3),
            local=dict(valid=True, position=(2., 3., -1.), velocity=(0., 0., 0.), heading=0., resets=(0, 0, 0)),
            health=dict(schema_version=1, navigation_ready=True, airborne_fault=False,
                        requires_disarm=False, armed=True, instance_id="first", reset_counter=0,
                        reference_epoch=1, clock_epoch=0, px4_xy_reset_counter=0,
                        px4_z_reset_counter=0, px4_heading_reset_counter=0),
            land=dict(landed=False, freefall=False))
        self.feed()

    def feed(self, dt=.05, skip=()):
        self.now += dt
        self.wall += round(dt*1e6)
        for name, p in self.payloads.items():
            if name not in skip:
                self.gate.ingest(name, self.wall, self.now, self.wall, copy.deepcopy(p))

    def start(self):
        self.payloads["switches"].update(offboard_switch=1, mode_slot=self.gate.offboard_slot or 2)
        self.payloads["status"].update(nav_state=14, intended_mode=14)
        self.feed()
        assert self.gate.acquire(self.now), self.gate.reason
        return self


class AuthorityTest(unittest.TestCase):
    @staticmethod
    def detent(f, mode, slot, ticks=28):
        f.payloads["status"].update(nav_state=mode, intended_mode=mode)
        f.payloads["switches"].update(mode_slot=slot, offboard_switch=0)
        for _ in range(ticks):
            f.feed()
            f.gate.learn_manual_selector(f.now)

    def test_three_position_learns_normal_and_reversed_rc_slots(self):
        for altitude, position, offboard in ((1, 4, 6), (6, 3, 1)):
            f = Fixture()
            f.gate = PilotAuthority(three_position=True)
            self.detent(f, 1, altitude)
            self.assertEqual(f.gate.selector_configuration_reason(), "observe_position_detent")
            self.assertFalse(f.gate.acquire(f.now))
            self.detent(f, 2, position)
            self.assertEqual(f.gate.manual_slots, {1: altitude, 2: position})
            self.assertTrue(f.gate.prestream(f.now))
            f.payloads["status"].update(nav_state=14, intended_mode=14)
            f.payloads["switches"]["mode_slot"] = offboard
            f.feed()
            self.assertTrue(f.gate.acquire(f.now), f.gate.reason)
            self.assertTrue(f.gate.authorized(f.now))
            f.payloads["switches"]["mode_slot"] = altitude
            f.feed()
            self.assertFalse(f.gate.authorized(f.now))
            self.assertEqual(f.gate.reason, "pilot_takeover")

    def test_mode_command_without_physical_third_detent_does_not_authorize(self):
        f = Fixture()
        f.gate = PilotAuthority(three_position=True)
        self.detent(f, 1, 1)
        self.detent(f, 2, 4)
        f.payloads["status"].update(nav_state=14, intended_mode=14)
        f.feed()
        self.assertFalse(f.gate.acquire(f.now))

    def test_delayed_old_mode_cannot_learn_the_next_slot(self):
        f = Fixture()
        f.gate = PilotAuthority(three_position=True)
        self.detent(f, 1, 1)
        self.detent(f, 1, 4, ticks=10)  # slot moves before commander updates its status
        self.detent(f, 2, 4)
        self.assertEqual(f.gate.manual_slots, {1: 1, 2: 4})
        self.assertFalse(f.gate.selector_fault)

    def test_third_detent_before_offboard_takes_effect_is_not_a_mapping_fault(self):
        """实机拨向 Offboard 档时 nav_state 尚未切换，不得判为档位映射漂移。

        三档开关的通道值先变，PX4 在 offboard 控制流建立前保持原 nav_state，
        于是「已学过的手动模式 + 第三档位」会稳定存在数秒。这是正常拨档，
        不是遥控器配置被改动。
        """
        f = Fixture()
        f.gate = PilotAuthority(three_position=True)
        self.detent(f, 1, 1)
        self.detent(f, 2, 4)
        self.detent(f, 1, 6, ticks=60)  # 3 s：拨到第三档，PX4 仍保持 Altitude
        self.detent(f, 2, 6, ticks=60)
        self.assertFalse(f.gate.selector_fault)
        self.assertEqual(f.gate.manual_slots, {1: 1, 2: 4})
        self.assertEqual(f.gate.selector_configuration_reason(), "")

    def test_ambiguous_or_changed_manual_mapping_blocks_authorization(self):
        # 冲突要持续超过 3 s 才判故障（拨档过渡期 PX4 可能晚几秒才改 nav_state），
        # 所以这里观察 13 s。
        for mode, slot, reason in ((2, 1, "rc_manual_slots_not_distinct"), (1, 4, "rc_mode_slot_changed")):
            f = Fixture()
            f.gate = PilotAuthority(three_position=True)
            self.detent(f, 1, 1)
            self.detent(f, mode, slot, ticks=260)
            self.assertEqual(f.gate.selector_configuration_reason(), reason)
            self.assertFalse(f.gate.prestream(f.now))

    def test_local_initial_alignment_does_not_latch_a_nonexistent_task(self):
        f = Fixture()
        f.payloads["local"].update(position=(12., 3., -1.), heading=math.pi/2)
        f.feed()
        self.assertFalse(f.gate.local_fault)
        f.start()
        f.payloads["local"]["position"] = (14., 3., -1.)
        f.feed()
        self.assertFalse(f.gate.authorized(f.now))

    def test_manual_position_and_altitude_have_no_authority(self):
        f = Fixture()
        for mode in (1, 2):
            f.payloads["status"].update(nav_state=mode, intended_mode=mode)
            f.feed()
            self.assertFalse(f.gate.acquire(f.now))
            self.assertTrue(f.gate.prestream(f.now))
        f.start()
        self.assertTrue(f.gate.authorized(f.now))

    def test_offboard_from_gcs_without_physical_selector_is_rejected(self):
        f = Fixture()
        f.payloads["status"].update(nav_state=14, intended_mode=14)
        f.feed()
        self.assertFalse(f.gate.acquire(f.now))

    def test_must_observe_manual_and_selector_off_before_acquisition(self):
        f = Fixture().start()
        gate = PilotAuthority()
        for name, p in f.payloads.items():
            gate.ingest(name, f.wall, f.now, f.wall, p)
        self.assertFalse(gate.acquire(f.now))
        self.assertFalse(gate.prestream(f.now))

    def test_selector_off_revokes_before_px4_mode_changes(self):
        f = Fixture().start()
        f.payloads["switches"]["offboard_switch"] = 3
        f.feed()
        self.assertFalse(f.gate.authorized(f.now))
        self.assertEqual(f.gate.reason, "pilot_takeover")
        f.payloads["switches"]["offboard_switch"] = 1
        f.feed()
        self.assertFalse(f.gate.acquire(f.now))
        self.assertFalse(f.gate.authorized(f.now))
        self.assertFalse(f.gate.prestream(f.now))

    def test_actual_or_intended_mode_exit_revokes(self):
        for field in ("nav_state", "intended_mode"):
            for mode in (1, 2, 18):
                with self.subTest(field=field, mode=mode):
                    f = Fixture().start()
                    f.payloads["status"][field] = mode
                    f.feed()
                    self.assertFalse(f.gate.authorized(f.now))

    def test_no_stale_stream_can_keep_permission(self):
        for stream, ttl in PilotAuthority.LIMITS.items():
            with self.subTest(stream=stream):
                f = Fixture().start()
                for _ in range(math.ceil(ttl/.05) + 2):
                    f.feed(skip=(stream,))
                self.assertFalse(f.gate.authorized(f.now))
                self.assertEqual(f.gate.reason, "stale_" + stream)

    def test_replay_and_late_source_do_not_refresh_receipt(self):
        for delta in (-10000, -2000000, 100000):
            f = Fixture().start()
            before = f.gate.received["rc"]
            self.assertFalse(f.gate.ingest("rc", f.wall+delta, f.now+.1, f.wall,
                                         dict(valid=True, data_source=1)))
            self.assertEqual(f.gate.received["rc"], before)
            self.assertFalse(f.gate.authorized(f.now+.1))

    def test_identical_publication_is_ignored_without_invalidating_or_renewing(self):
        f = Fixture().start()
        before = f.gate.received["rc"]
        self.assertFalse(f.gate.ingest("rc", f.wall, f.now+.1, f.wall+100000, f.payloads["rc"]))
        self.assertEqual(f.gate.received["rc"], before)
        self.assertTrue(f.gate.authorized(f.now+.1))
        self.assertFalse(f.gate.authorized(f.now+f.gate.LIMITS["rc"]+.01))

    def test_duplicate_publication_cannot_hide_changed_payload(self):
        f = Fixture().start()
        changed = dict(f.payloads["rc"], valid=False)
        self.assertFalse(f.gate.ingest("rc", f.wall, f.now+.01, f.wall+10000, changed))
        self.assertFalse(f.gate.authorized(f.now+.01))

    def test_real_px4_cadence_constant_sticks_and_single_packet_loss(self):
        f = Fixture().start()
        sample = f.wall
        # 60 simulated seconds at 20Hz evaluation: status 2Hz, RC 5Hz,
        # switches/land 1Hz, local 10Hz, real bridge health 5Hz.
        periods = dict(status=10, rc=4, switches=20, local=2, health=4, land=20)
        # Drop one status and RC publication, without extending the lease twice.
        for tick in range(1, 1201):
            now, wall = f.now+tick*.05, f.wall+tick*50000
            if tick % 7 == 0:
                sample = wall  # unchanged RC input sample is periodically refreshed by rc_update
            for stream, period in periods.items():
                if tick % period or (stream, tick) in (("status", 100), ("rc", 104)):
                    continue
                self.assertTrue(f.gate.ingest(stream, wall, now, wall, f.payloads[stream],
                                sample_us=sample if stream == "rc" else wall), (stream, f.gate.reason))
            self.assertTrue(f.gate.authorized(now), (tick, f.gate.reason))

    def test_fresh_publication_cannot_extend_frozen_rc_sample(self):
        f = Fixture().start()
        sample = f.wall
        # 跟随 SAMPLE_LIMITS 配置，避免把预算写死在测试里
        ticks = int(math.ceil(PilotAuthority.SAMPLE_LIMITS["rc"] / .05)) + 3
        for i in range(1, ticks):
            now, wall = f.now+i*.05, f.wall+i*50000
            for stream in f.payloads:
                f.gate.ingest(stream, wall, now, wall, f.payloads[stream],
                              sample_us=sample if stream == "rc" else wall)
            if i <= 10:
                self.assertTrue(f.gate.authorized(now))
        self.assertFalse(f.gate.authorized(now))

    def test_fault_or_takeover_with_unchanged_sample_is_immediate(self):
        for stream, change in (("rc", dict(valid=False)), ("switches", dict(offboard_switch=3))):
            f = Fixture().start()
            sample = f.wall
            f.payloads[stream].update(change)
            self.assertTrue(f.gate.ingest(stream, sample+10000, f.now+.01, sample+10000,
                                         f.payloads[stream], sample_us=sample))
            self.assertFalse(f.gate.authorized(f.now+.01))

    def test_source_age_counts_towards_remaining_lease(self):
        f = Fixture().start()
        publication = f.wall+100000
        receipt = publication+600000  # RC publication already 0.6s old at receipt
        self.assertTrue(f.gate.ingest("rc", publication, f.now+.7, receipt,
                                     f.payloads["rc"], sample_us=publication))
        # publication 已 0.6s 老；按当前 rc 预算算出应判 stale 的时刻
        stale_at = f.now + .7 + (PilotAuthority.LIMITS["rc"] - .6) + .01
        self.assertEqual(f.gate.freshness_reason("rc", stale_at), "stale_rc")

    def test_sample_timestamp_regression_and_future_sample_are_rejected(self):
        # 采样时戳倒退 + 载荷完全一致 = 同一采样样本的重复投递：幂等忽略，
        # 租约保留但不续租（received/source/samples 均不前进）。
        f = Fixture().start()
        before = (f.gate.source["rc"], f.gate.samples["rc"], f.gate.received["rc"])
        self.assertFalse(f.gate.ingest("rc", f.wall+10000, f.now+.01, f.wall+10000,
                                       f.payloads["rc"], sample_us=f.wall-1))
        self.assertTrue(f.gate.authorized(f.now+.01))
        self.assertEqual((f.gate.source["rc"], f.gate.samples["rc"],
                          f.gate.received["rc"]), before)
        self.assertEqual(f.gate.sample_repeat_ignored.get("rc"), 1)
        # 载荷不同的倒退样本属于乱序/冲突旧样本：仍撤销租约。
        f = Fixture().start()
        changed = dict(f.payloads["rc"], data_source=0)
        self.assertFalse(f.gate.ingest("rc", f.wall+10000, f.now+.01, f.wall+10000,
                                       changed, sample_us=f.wall-1))
        self.assertFalse(f.gate.authorized(f.now+.01))
        # 未来采样时戳仍然拒绝。
        f = Fixture().start()
        self.assertFalse(f.gate.ingest("rc", f.wall+10000, f.now+.01, f.wall+10000,
                                       f.payloads["rc"], sample_us=f.wall+200000))
        self.assertFalse(f.gate.authorized(f.now+.01))

    def test_reference_identity_changes_cancel_task(self):
        for key in TOKEN_KEYS:
            with self.subTest(key=key):
                f = Fixture().start()
                f.payloads["health"][key] = "second" if key == "instance_id" else 2
                f.feed()
                self.assertFalse(f.gate.authorized(f.now))

    def test_faults_and_overrides(self):
        cases = [("status", "armed", False), ("status", "failsafe", True),
                 ("rc", "valid", False), ("rc", "data_source", 2),
                 ("land", "landed", True), ("land", "freefall", True),
                 ("local", "valid", False), ("health", "navigation_ready", False),
                 ("health", "schema_version", 2), ("health", "airborne_fault", True),
                 ("health", "requires_disarm", True)]
        cases += [("switches", k, 1) for k in
                  ("return_switch", "loiter_switch", "kill_switch", "termination_switch")]
        for stream, key, value in cases:
            with self.subTest(stream=stream, key=key):
                f = Fixture().start()
                f.payloads[stream][key] = value
                f.feed()
                self.assertFalse(f.gate.authorized(f.now))

    def test_nonfinite_and_position_or_yaw_jump(self):
        for change in (dict(position=(math.nan, 0., 0.)), dict(heading=math.inf),
                       dict(position=(3., 3., -1.)), dict(heading=math.pi/2)):
            with self.subTest(change=change):
                f = Fixture().start()
                f.payloads["local"].update(change)
                f.feed()
                self.assertFalse(f.gate.authorized(f.now))

    def test_smooth_motion_and_yaw_wrap(self):
        f = Fixture()
        f.payloads["local"]["heading"] = math.pi-.02
        f.feed(dt=.35)  # establish a new comparison window before starting
        f.start()
        for i in range(20):
            f.payloads["local"].update(position=(2.+i*.002, 3., -1.), velocity=(.04, 0., 0.), heading=-math.pi+.02)
            f.feed()
            self.assertTrue(f.gate.authorized(f.now), f.gate.reason)

    def test_configured_mode_slot(self):
        f = Fixture(slot=6).start()
        f.payloads["switches"]["mode_slot"] = 2
        f.feed()
        self.assertFalse(f.gate.authorized(f.now))
        with self.assertRaises(ValueError):
            PilotAuthority(7)


if __name__ == "__main__":
    unittest.main()
