#!/usr/bin/env python3
"""[NP-01] State-sequence replay of the TAKEOFF branch.

Uses the SAME advance_takeoff() that sim_pilot.py calls, so this is an
integration-level check of the state machine, not only of a boolean helper.
"""
import unittest

from takeoff_altitude_policy import (DEFAULT_CONFIG, HeightSample,
                                     advance_takeoff)

ALT = 1.0
BASE = 100.0


def sample(z, t, *, vz=0.0, z_valid=True, vz_valid=True,
           xy_valid=False, vxy_valid=False, zrc=1, vzrc=1, dz=0.0,
           age=0.05):
    return HeightSample(z=z, vz=vz, z_valid=z_valid, v_z_valid=vz_valid,
                        xy_valid=xy_valid, v_xy_valid=vxy_valid,
                        z_reset_counter=zrc, vz_reset_counter=vzrc,
                        delta_z=dz, sample_us=int(t * 1e6),
                        rx_mono=BASE + t - age)


class Replay:
    """Drive one tick sequence through the shared state-machine function."""
    def __init__(self, cfg=DEFAULT_CONFIG):
        self.cfg = cfg
        self.ref = None
        self.recaptures = 0
        self.steps = []

    def tick(self, t, s):
        st = advance_takeoff(s, self.ref, ALT, BASE + t, t,
                             self.cfg, self.recaptures)
        self.ref, self.recaptures = st.ref, st.recaptures
        self.steps.append((t, st))
        return st

    @property
    def states(self):
        return [s.state for _, s in self.steps]

    @property
    def throttles(self):
        return [s.throttle for _, s in self.steps]


class TakeoffReplayTest(unittest.TestCase):
    def test_normal_climb_sequence(self):
        # ULog 04_34_17: z 0.02 -> -1.09 over ~9 s, vertical valid,
        # horizontal INVALID throughout. This MUST still succeed.
        r = Replay()
        for i in range(0, 46):
            t = i * 0.25
            z = 0.02 - 1.10 * min(1.0, t / 9.0)
            st = r.tick(t, sample(z, t))
            if st.state != "TAKEOFF":
                break
        self.assertEqual(st.state, "TAKEOFF_SETTLE")
        self.assertEqual(r.states[0], "TAKEOFF")
        self.assertNotIn("FAULT", r.states)
        self.assertNotIn("EXCITE", r.states)   # removed transition
        self.assertEqual(st.throttle, 1500)
        self.assertEqual(r.steps[0][1].status, "capture")

    def test_startup_divergence_enters_explicit_fault(self):
        # ULog 04_09_40: z_valid=1 but z~+11 m, vz~15.5 m/s, z grows.
        # Old FIX6 would have called this a climb; now it must FAULT.
        r = Replay()
        st = None
        for i in range(0, 50):
            t = i * 0.25
            st = r.tick(t, sample(11.29 + 0.4 * t, t, vz=15.5))
            if st.state == "FAULT":
                break
        self.assertEqual(st.state, "FAULT")
        self.assertNotIn("TAKEOFF_SETTLE", r.states)
        self.assertNotIn("EXCITE", r.states)
        self.assertTrue(all(x == 1500 for x in r.throttles),
                        "must never keep full climb throttle on invalid feedback")

    def test_missing_dds_height_is_fail_closed(self):
        r = Replay()
        st = None
        for i in range(0, 50):
            t = i * 0.25
            st = r.tick(t, None)
            if st.state == "FAULT":
                break
        self.assertEqual(st.state, "FAULT")
        self.assertEqual(set(r.throttles), {1500})
        # Every tick before the timeout must be rejected for the missing sample,
        # and the timeout tick must still carry that cause.  (The timeout reason
        # is prefixed, so a plain equality check on the last tick is wrong.)
        for _, s in r.steps[:-1]:
            self.assertEqual(s.reason, "missing_sample")
        self.assertEqual(r.steps[-1][1].reason, "takeoff_timeout:missing_sample")
        self.assertNotIn("TAKEOFF_SETTLE", r.states)
        self.assertNotIn("EXCITE", r.states)

    def test_climb_throttle_is_held_while_climbing(self):
        # RUN np01_fix5b_func1m_20261115_142648: FIX6 pulsed 1700 on the capture
        # tick and then held 1500 (neutral in Altitude), so the vehicle climbed
        # 0.00 m in 12 s and timed out with reason=takeoff_timeout:climbing.
        r = Replay()
        r.tick(0.0, sample(0.02, 0.0))                 # capture
        for i in range(1, 12):                          # 2.75 s of real climb
            t = i * 0.25
            r.tick(t, sample(0.02 - 0.12 * t, t, vz=-0.12))
        self.assertNotIn("FAULT", r.states)
        self.assertEqual(r.throttles[0], 1700)
        self.assertTrue(all(x == 1700 for x in r.throttles),
                        "a confirmed climb must keep the climb throttle")

    def test_climb_throttle_drops_when_feedback_goes_bad(self):
        r = Replay()
        r.tick(0.0, sample(0.02, 0.0))
        r.tick(0.25, sample(-0.01, 0.25, vz=-0.12))
        self.assertEqual(r.throttles[-1], 1700)
        # one invalid sample must fall back to the fail-closed throttle
        r.tick(0.5, sample(-0.04, 0.5, vz=-0.12, z_valid=False))
        self.assertEqual(r.throttles[-1], 1500)
        # and recover to the climb throttle once the feedback is sane again
        r.tick(0.75, sample(-0.07, 0.75, vz=-0.12))
        self.assertEqual(r.throttles[-1], 1700)

    def test_small_downward_drift_still_gets_climb_throttle(self):
        # RUN np01_fix6b_func1m_20261115_142907: z drifted 5 mm positive (down)
        # right after arming, so reason became "descending" and the throttle
        # stayed at the neutral 1500 -> the vehicle could never start climbing.
        r = Replay()
        r.tick(0.0, sample(0.055, 0.0))                # capture at z=+0.055
        r.tick(0.25, sample(0.060, 0.25, vz=0.02))     # drift further down
        self.assertEqual(r.throttles[-1], 1700,
                         "a small downward drift must not withhold the climb throttle")
        self.assertNotIn("TAKEOFF_SETTLE", r.states)

    def test_descent_never_reaches(self):
        r = Replay()
        st = None
        for i in range(0, 50):
            t = i * 0.25
            st = r.tick(t, sample(0.02 + 0.1 * t, t))
            if st.state == "FAULT":
                break
        self.assertEqual(st.state, "FAULT")
        self.assertNotIn("TAKEOFF_SETTLE", r.states)

    def test_height_reset_recaptures_without_false_success(self):
        r = Replay()
        # capture at z=0.02
        r.tick(0.0, sample(0.02, 0.0))
        # climb half way, then EV height reset with delta_z=+0.463
        r.tick(2.0, sample(-0.55, 2.0))
        reset = r.tick(2.25, sample(-1.02, 2.25, zrc=2, dz=0.463))
        self.assertEqual(reset.status, "capture")
        self.assertEqual(reset.state, "TAKEOFF")
        self.assertNotEqual(reset.state, "TAKEOFF_SETTLE")
        self.assertEqual(r.recaptures, 1)
        self.assertEqual(r.ref.z_reset_counter, 2)
        # after recapture, need a full signed climb from the NEW reference
        st = r.tick(8.0, sample(-2.10, 8.0, zrc=2, dz=0.463))
        self.assertEqual(st.state, "TAKEOFF_SETTLE")

    def test_stale_gap_waits_then_recovers(self):
        r = Replay()
        r.tick(0.0, sample(0.02, 0.0))
        # freeze rx_mono while time advances -> stale
        stale = sample(-0.60, 0.25, age=3.0)
        st = r.tick(3.0, stale)
        self.assertEqual(st.status, "wait")
        self.assertEqual(st.reason, "stale_sample")
        self.assertEqual(st.throttle, 1500)
        # fresh sample resumes, reference preserved -> success
        st = r.tick(9.0, sample(-1.05, 9.0, age=0.05))
        self.assertEqual(st.state, "TAKEOFF_SETTLE")

    def test_reset_exhaustion_is_explicit_fault(self):
        r = Replay()
        r.tick(0.0, sample(0.02, 0.0))
        st = None
        for i in range(1, 8):
            t = i * 0.5
            st = r.tick(t, sample(-1.20, t, zrc=i + 1))
            if st.state == "FAULT":
                break
        self.assertEqual(st.state, "FAULT")
        self.assertEqual(st.reason, "reset_recapture_exhausted")
        self.assertNotIn("EXCITE", r.states)

    def test_vertical_valid_horizontal_invalid_full_sequence(self):
        r = Replay()
        st = None
        for i in range(0, 46):
            t = i * 0.25
            z = 0.02 - 1.10 * min(1.0, t / 9.0)
            st = r.tick(t, sample(z, t, xy_valid=False, vxy_valid=False))
            if st.state != "TAKEOFF":
                break
        self.assertEqual(st.state, "TAKEOFF_SETTLE")


if __name__ == "__main__":
    unittest.main(verbosity=2)
