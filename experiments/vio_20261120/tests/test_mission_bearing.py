#!/usr/bin/env python3
"""任务方位角回归：直接调用实际行为树与配置解析入口。
旧行为树夹具仅用于核对缺省方向，不作为运行入口。
"""
import importlib.util
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
CAND = os.path.abspath(os.path.join(HERE, "..", "scripts"))
PROD = os.path.join(HERE, "fixtures")


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


# pilot_navigation_bt imports `pilot_authority` by bare name.
sys.path.insert(0, CAND)
cand = load("bt_cand", os.path.join(CAND, "pilot_navigation_bt.py"))
prod = load("bt_prod", os.path.join(PROD, "pilot_navigation_bt_legacy.py"))
boot = load("boot_cand", os.path.join(CAND, "pilot_navigation_boot.py"))


class BB:
    """Minimal blackboard: only the fields GotoPoint reads."""

    def __init__(self, home, yaw, args, bearing=None, pos=None, vel=(0., 0., 0.)):
        self.home = home
        self.yaw = yaw
        self.args = args
        if bearing is not None:
            self.bearing = bearing
        self.sp = tuple(home)
        self.target = None
        self.pos = tuple(home if pos is None else pos)
        self.vel = tuple(vel)
        self.last_tick = 0.
        self.phase = "NAVIGATE"


class Args:
    speed = 0.35


class TestDefaultIsUnchanged(unittest.TestCase):
    def test_target_bit_identical_to_frozen_production(self):
        for yaw in (0.0, 1.62 * math.pi / 180, 0.7, -2.1, 3.0):
            for home in ((0., 0., -2.), (0.07, 4.76, -2.165), (-0.93, 3.596, -1.34)):
                a = BB(home, yaw, Args())
                b = BB(home, yaw, Args())
                prod.GotoPoint(5., 0.).tick(a, 0.0)
                cand.GotoPoint(5., 0.).tick(b, 0.0)
                self.assertEqual(a.target, b.target, (yaw, home))

    def test_default_bearing_does_not_rotate(self):
        bb = BB((0., 0., -1.), 0.4, Args())
        cand.GotoPoint(5., 0.).tick(bb, 0.0)
        azimuth = math.atan2(bb.target[1] - bb.home[1], bb.target[0] - bb.home[0])
        self.assertAlmostEqual(azimuth, 0.4, places=12)


class TestBearing(unittest.TestCase):
    def test_distance_is_preserved_for_every_bearing(self):
        for deg in (0., 45., 90., -90., 135., 180., -180.):
            bb = BB((1., 2., -1.5), 0.05, Args(), bearing=math.radians(deg))
            cand.GotoPoint(5., 0.).tick(bb, 0.0)
            self.assertAlmostEqual(math.dist(bb.home, bb.target), 5.0, places=12)
            self.assertAlmostEqual(bb.target[2], bb.home[2], places=12)

    def test_bearing_90_is_heading_plus_90(self):
        for yaw in (0.0, 0.3, -1.2):
            bb = BB((0., 0., -1.), yaw, Args(), bearing=math.pi / 2)
            cand.GotoPoint(5., 0.).tick(bb, 0.0)
            azimuth = math.atan2(bb.target[1] - bb.home[1], bb.target[0] - bb.home[0])
            expect = math.atan2(math.sin(yaw + math.pi / 2), math.cos(yaw + math.pi / 2))
            self.assertAlmostEqual(math.remainder(azimuth - expect, 2 * math.pi), 0.0, places=12)

    def test_lateral_leg_rotates_with_azimuth(self):
        # Only used by the square mission, but the lateral term must rotate with
        # the azimuth, otherwise a bearing offset would skew the square.
        bb = BB((0., 0., -1.), 0.0, Args(), bearing=math.pi / 2)
        cand.GotoPoint(0., 5.).tick(bb, 0.0)
        vec = (bb.target[0] - bb.home[0], bb.target[1] - bb.home[1])
        self.assertAlmostEqual(math.hypot(*vec), 5.0, places=9)
        # perpendicular to the mission azimuth (0, 1) after a +90 deg rotation
        self.assertAlmostEqual(vec[0] * 0.0 + vec[1] * 1.0, 0.0, places=9)


class TestArrivalUnchanged(unittest.TestCase):
    """Arrival tolerance 0.15 m, settle 0.2 m/s, dwell 2.0 s must be untouched."""

    def _run(self, offset, vel, seconds=12.0, dt=0.1):
        home = (0., 0., -2.)
        bb = BB(home, 0.0, Args(), bearing=0.0)
        g = cand.GotoPoint(5., 0.)
        target = (5., 0., -2.)
        t = 0.0
        result = cand.RUNNING
        while t <= seconds:
            bb.sp = target
            bb.pos = (target[0] - offset, target[1], target[2])
            bb.vel = (vel, 0., 0.)
            bb.last_tick = t - dt
            result = g.tick(bb, t)
            if result == cand.SUCCESS:
                return t
            t += dt
        return None

    def test_dwell_is_two_seconds(self):
        t = self._run(0.0, 0.0)
        self.assertIsNotNone(t)
        self.assertAlmostEqual(t, 2.0, delta=0.15)

    def test_0151_m_out_is_not_arrival(self):
        # EXP_CTRL_002 measured 0.151 m remaining at revoke: outside 0.15 m.
        self.assertIsNone(self._run(0.151, 0.0))

    def test_0195_m_out_is_not_arrival(self):
        # EXP_DIAG_003 measured 0.195 m remaining at revoke.
        self.assertIsNone(self._run(0.195, 0.0))

    def test_moving_faster_than_settle_is_not_arrival(self):
        # EXP_DIAG_003 measured 0.32 m/s while still 0.195 m out.
        self.assertIsNone(self._run(0.0, 0.32))


class TestBootConfig(unittest.TestCase):
    LEGACY = {"schema_version": 1, "rc_selector": "three_position", "mission": "line",
              "distance_m": 5.0, "side_m": 1.0, "speed_m_s": 0.35, "max_radius_m": 12.0,
              "max_alt_error_m": 1.5, "max_tracking_error_m": 1.8}

    def test_legacy_config_emits_no_bearing_option(self):
        self.assertNotIn("--bearing-deg", boot.arguments(dict(self.LEGACY)))

    def test_bearing_is_forwarded(self):
        cfg = dict(self.LEGACY, bearing_deg=90.0)
        args = boot.arguments(cfg)
        i = args.index("--bearing-deg")
        self.assertEqual(float(args[i + 1]), 90.0)

    def test_bad_bearing_is_rejected(self):
        for bad in (181.0, -180.5, float("nan"), float("inf"), "90", True):
            with self.assertRaises(ValueError):
                boot.arguments(dict(self.LEGACY, bearing_deg=bad))

    def test_parse_args_default_is_zero(self):
        a = cand.parse_args(["--mission", "line", "--distance", "5"])
        self.assertEqual(a.bearing_deg, 0.0)

    def test_parse_args_rejects_out_of_range(self):
        with self.assertRaises(SystemExit):
            cand.parse_args(["--bearing-deg", "200"])


class TestTreeShape(unittest.TestCase):
    def test_line_mission_is_one_goto_plus_hold(self):
        a = cand.parse_args(["--mission", "line", "--distance", "5", "--bearing-deg", "90"])
        tree = cand.build_tree(a)
        self.assertEqual(len(tree.children), 2)
        self.assertIsInstance(tree.children[0], cand.GotoPoint)
        self.assertIsInstance(tree.children[1], cand.Hold)
        self.assertEqual(tree.children[0].forward, 5.0)
        self.assertEqual(tree.children[0].right, 0.0)
        self.assertEqual(tree.children[0].tolerance, 0.15)
        self.assertEqual(tree.children[0].dwell, 2.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
