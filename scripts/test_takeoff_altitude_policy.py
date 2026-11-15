#!/usr/bin/env python3
"""Targeted regression for the REAL height-feedback contract.

Expectations follow PX4 message semantics and ULog measurements, NOT the old
abs(z-z0) shortcut. In particular an invalid / diverged z that grows by a
large amount is NEVER evidence of a successful climb.
"""
import math
import unittest

from takeoff_altitude_policy import (DEFAULT_CONFIG, HeightReference,
                                     HeightSample, TakeoffConfig,
                                     capture_reference, diagnostics,
                                     evaluate_takeoff, sample_rejection)

ALT = 1.0
NOW = 100.0


def S(z=0.0, vz=0.0, z_valid=True, vz_valid=True, xy_valid=False,
      vxy_valid=False, zrc=1, vzrc=1, dz=0.0, sample_us=1000, rx=None):
    return HeightSample(z=z, vz=vz, z_valid=z_valid, v_z_valid=vz_valid,
                        xy_valid=xy_valid, v_xy_valid=vxy_valid,
                        z_reset_counter=zrc, vz_reset_counter=vzrc,
                        delta_z=dz, sample_us=sample_us,
                        rx_mono=(NOW - 0.05) if rx is None else rx)


def REF(z=0.02, zrc=1, vzrc=1, sample_us=1000, rx=NOW - 0.5):
    return HeightReference(z, zrc, vzrc, sample_us, rx)


class HeightContractTest(unittest.TestCase):
    # ---------- normal inputs ----------
    def test_signed_climb_reaches(self):
        d = evaluate_takeoff(S(z=-1.05), REF(z=0.02), ALT, NOW)
        self.assertEqual(d.status, "reached", d.reason)

    def test_vertical_valid_horizontal_invalid_is_normal_takeoff(self):
        # ULog: takeoff has z_valid=v_z_valid=1 and xy_valid=v_xy_valid=0.
        d = evaluate_takeoff(S(z=-1.10, xy_valid=False, vxy_valid=False),
                             REF(), ALT, NOW)
        self.assertEqual(d.status, "reached")
        self.assertEqual(sample_rejection(S(xy_valid=True, vxy_valid=True), NOW), "")

    def test_small_fixed_origin_offset_is_absorbed(self):
        ref = REF(z=0.30)
        d = evaluate_takeoff(S(z=-0.80), ref, ALT, NOW)
        self.assertEqual(d.status, "reached")

    # ---------- the OLD abs() contract is explicitly revoked ----------
    def test_descent_is_never_takeoff(self):
        d = evaluate_takeoff(S(z=1.05), REF(z=0.02), ALT, NOW)
        self.assertEqual(d.status, "wait")
        self.assertEqual(d.reason, "descending")

    def test_diverged_invalid_z_growth_is_not_climb(self):
        # FIX6 fault input z0=+10.75 -> +18.48. z0 itself is outside the
        # ground bound, so the reference must be refused; and even with a
        # forced reference, a positive-going z is a DESCENT in NED.
        s = S(z=18.48, vz=1.0)
        ref, reason = capture_reference(s)
        self.assertIsNone(ref)
        self.assertEqual(reason, "reference_z_diverged")
        forced = REF(z=10.75)
        d = evaluate_takeoff(s, forced, ALT, NOW)
        self.assertNotEqual(d.status, "reached")
        self.assertEqual(d.reason, "descending")

    # ---------- reference capture, validity, freshness ----------
    def test_reference_capture_tick_is_not_success(self):
        d = evaluate_takeoff(S(z=0.03), None, ALT, NOW)
        self.assertEqual(d.status, "capture")
        self.assertIsNotNone(d.ref)

    def test_z_invalid_even_if_horizontal_valid(self):
        d = evaluate_takeoff(S(z=-1.2, z_valid=False, xy_valid=True,
                               vxy_valid=True), REF(), ALT, NOW)
        self.assertEqual(d.status, "wait")
        self.assertEqual(d.reason, "z_invalid")

    def test_vz_invalid(self):
        self.assertEqual(sample_rejection(S(vz_valid=False), NOW), "vz_invalid")

    def test_stale_and_future_samples(self):
        self.assertEqual(sample_rejection(S(rx=NOW - 0.6), NOW), "stale_sample")
        self.assertEqual(sample_rejection(S(rx=NOW + 1.0), NOW), "stale_sample")
        d = evaluate_takeoff(S(z=-1.2, rx=NOW - 5.0), REF(), ALT, NOW)
        self.assertEqual(d.status, "wait")

    def test_missing_sample(self):
        d = evaluate_takeoff(None, REF(), ALT, NOW)
        self.assertEqual(d.status, "wait")
        self.assertEqual(d.reason, "missing_sample")

    # ---------- non-finite values, all fields ----------
    def test_non_finite_every_field(self):
        inf = float("inf"); nan = float("nan")
        cases = [dict(z=nan), dict(z=inf), dict(z=-inf), dict(vz=nan),
                 dict(vz=inf), dict(dz=nan), dict(dz=inf),
                 dict(sample_us=nan), dict(rx=nan), dict(rx=inf)]
        for kw in cases:
            with self.subTest(kw=kw):
                self.assertEqual(sample_rejection(S(**kw), NOW), "non_finite")
                d = evaluate_takeoff(S(**kw), REF(), ALT, NOW)
                self.assertNotEqual(d.status, "reached")
        self.assertEqual(evaluate_takeoff(S(), REF(), nan, NOW).reason, "invalid_alt")
        self.assertEqual(evaluate_takeoff(S(), REF(), inf, NOW).reason, "invalid_alt")
        self.assertEqual(evaluate_takeoff(S(), REF(), 0.0, NOW).reason, "invalid_alt")
        self.assertEqual(evaluate_takeoff(S(), REF(), ALT, nan).reason, "invalid_alt")

    # ---------- reset handling ----------
    def test_height_reset_recaptures_and_does_not_succeed_that_tick(self):
        s = S(z=-1.20, zrc=2, dz=0.46)
        d = evaluate_takeoff(s, REF(zrc=1), ALT, NOW)
        self.assertEqual(d.status, "capture")
        self.assertTrue(d.recapture)
        self.assertEqual(d.ref.z_reset_counter, 2)
        # following tick with the NEW reference and insufficient climb -> wait
        d2 = evaluate_takeoff(S(z=-1.20, zrc=2, dz=0.46), d.ref, ALT, NOW)
        self.assertEqual(d2.status, "wait")

    def test_vz_reset_also_recaptures(self):
        d = evaluate_takeoff(S(z=-1.2, vzrc=2), REF(vzrc=1), ALT, NOW)
        self.assertEqual(d.status, "capture")

    def test_delta_z_too_large_is_rejected(self):
        self.assertEqual(sample_rejection(S(dz=2.5), NOW), "reset_delta_diverged")

    def test_recapture_exhaustion_is_explicit_failure(self):
        cfg = TakeoffConfig(max_recaptures=2)
        d = evaluate_takeoff(S(z=-1.2, zrc=9), REF(zrc=1), ALT, NOW,
                             cfg=cfg, recaptures=2)
        self.assertEqual(d.status, "wait")
        self.assertEqual(d.reason, "reset_recapture_exhausted")

    # ---------- divergence ----------
    def test_vz_diverged(self):
        self.assertEqual(sample_rejection(S(vz=15.5), NOW), "vz_diverged")
        d = evaluate_takeoff(S(z=-1.2, vz=15.5), REF(), ALT, NOW)
        self.assertEqual(d.status, "wait")

    # ---------- time-domain separation ----------
    def test_source_time_never_used_for_freshness(self):
        # A wildly different (or regressing) source sample time must not
        # change the freshness decision; only rx_mono does.
        a = S(z=-1.2, sample_us=1)
        b = S(z=-1.2, sample_us=10**15)
        self.assertEqual(sample_rejection(a, NOW), "")
        self.assertEqual(sample_rejection(b, NOW), "")
        self.assertEqual(evaluate_takeoff(a, REF(), ALT, NOW).status, "reached")

    def test_diagnostics_records_validity_separately(self):
        d = diagnostics(S(xy_valid=False, vxy_valid=False), NOW)
        self.assertEqual((d["z_valid"], d["v_z_valid"]), (1, 1))
        self.assertEqual((d["xy_valid"], d["v_xy_valid"]), (0, 0))
        self.assertEqual(d["h_reject"], "")
        self.assertLess(d["h_rx_age_ms"], 500)


if __name__ == "__main__":
    unittest.main(verbosity=2)
