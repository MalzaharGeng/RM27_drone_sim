#!/usr/bin/env python3
"""[NP-01 FIX9] Unlocked preflight health gate: contract tests.

The gate runs before any arming request and only reads state. These tests pin
the DECISION, not the ROS wiring: they build a Pilot instance without calling
Node.__init__ (no rclpy node, no MAVLink connection) and feed height samples.

Run with ROS sourced (rclpy is imported by sim_pilot):
    source /opt/ros/lyrical/setup.bash && python3 test_preflight_gate.py
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import sim_pilot
except Exception as exc:                      # pragma: no cover
    raise unittest.SkipTest("sim_pilot not importable: %s" % exc)

from takeoff_altitude_policy import HeightSample

Pilot = sim_pilot.Pilot


def make_pilot(**over):
    p = Pilot.__new__(Pilot)
    p.height = None
    p.armed = False
    p.vio_ready = False
    p.est = None
    p.est_seen = 0
    p.pf_z_valid_seen = 0
    p.pf_vz_valid_seen = 0
    p.pf_xy_valid_seen = 0
    p.pf_samples = 0
    p.pf_z_min = None
    p.pf_z_max = None
    p.pf_vz_absmax = 0.0
    p.pf_nonfinite = 0
    p.pf_zrc_first = None
    p.pf_zrc_last = None
    p.pf_zrc_changes = 0
    p.pf_cs = {}
    for k, v in over.items():
        setattr(p, k, v)
    return p


def sample(z=0.02, vz=0.0, z_valid=True, vz_valid=True, xy_valid=False,
           zrc=1, dz=0.0):
    return HeightSample(z=z, vz=vz, z_valid=z_valid, v_z_valid=vz_valid,
                        xy_valid=xy_valid, v_xy_valid=xy_valid,
                        z_reset_counter=zrc, vz_reset_counter=zrc,
                        delta_z=dz, sample_us=0, rx_mono=0.0)


def feed(p, n, **kw):
    for _ in range(n):
        p.height = sample(**kw)
        p.preflight_accumulate()


class PreflightGateTest(unittest.TestCase):

    def test_healthy_ground_vehicle_passes(self):
        p = make_pilot()
        feed(p, 40, z=0.02, vz=0.01)
        ok, reason, s = p.preflight_report()
        self.assertTrue(ok, reason)
        self.assertEqual(reason, "healthy")
        self.assertEqual(s["z_valid"], 40)
        self.assertEqual(s["xy_valid"], 0)      # recorded separately, not gated
        self.assertFalse(s["armed"])

    def test_no_feedback_fails_closed(self):
        p = make_pilot()
        ok, reason, _ = p.preflight_report()
        self.assertFalse(ok)
        self.assertIn("no_height_feedback", reason)

    def test_diverged_z_is_rejected(self):
        # The NP-01 startup anomaly: z ran to 19 m while disarmed.
        p = make_pilot()
        feed(p, 30, z=11.3, vz=15.5, z_valid=True, vz_valid=True)
        ok, reason, s = p.preflight_report()
        self.assertFalse(ok)
        self.assertTrue(reason.startswith("z_diverged"), reason)

    def test_diverged_vz_is_rejected_even_with_small_z(self):
        p = make_pilot()
        feed(p, 30, z=0.05, vz=6.9)
        ok, reason, _ = p.preflight_report()
        self.assertFalse(ok)
        self.assertIn("vz_diverged", reason)

    def test_non_finite_is_rejected(self):
        p = make_pilot()
        feed(p, 20, z=0.02)
        p.height = sample(z=float("nan"))
        p.preflight_accumulate()
        ok, reason, _ = p.preflight_report()
        self.assertFalse(ok)
        self.assertIn("non_finite", reason)

    def test_never_valid_z_is_rejected(self):
        p = make_pilot()
        feed(p, 20, z=0.02, z_valid=False)
        ok, reason, _ = p.preflight_report()
        self.assertFalse(ok)
        self.assertIn("z_never_valid", reason)

    def test_reset_churn_is_rejected(self):
        p = make_pilot()
        for i in range(30):
            p.height = sample(z=0.02, zrc=1 + i)
            p.preflight_accumulate()
        ok, reason, _ = p.preflight_report()
        self.assertFalse(ok)
        self.assertIn("z_reset_churn", reason)

    def test_reset_counter_never_started_is_rejected(self):
        p = make_pilot()
        feed(p, 20, z=0.02, zrc=0)
        ok, reason, _ = p.preflight_report()
        self.assertFalse(ok)
        self.assertIn("z_reset_counter_never_started", reason)

    def test_armed_during_preflight_is_rejected(self):
        p = make_pilot(armed=True)
        feed(p, 20, z=0.02)
        ok, reason, _ = p.preflight_report()
        self.assertFalse(ok)
        self.assertIn("armed_during_preflight", reason)

    def test_xy_invalid_is_not_treated_as_height_failure(self):
        # The real trace has xy_valid=0 during startup; height is independent.
        p = make_pilot()
        feed(p, 20, z=0.02, xy_valid=False)
        ok, reason, s = p.preflight_report()
        self.assertTrue(ok, reason)
        self.assertEqual(s["xy_valid"], 0)

    def test_summary_records_fusion_sources_separately(self):
        p = make_pilot()
        feed(p, 20, z=0.02)
        p.pf_cs = {"cs_ev_hgt": True, "cs_baro_hgt": True, "cs_ev_pos": False}
        ok, _, s = p.preflight_report()
        self.assertTrue(ok)
        self.assertTrue(s["fusion"]["cs_ev_hgt"])
        self.assertFalse(s["fusion"]["cs_ev_pos"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
