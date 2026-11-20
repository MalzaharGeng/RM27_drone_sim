#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[NP-01 FIX16 T2] Unit tests for the post-VIBA2 handoff contract.

These are pure-function tests: they do not start a simulator, a pilot process
or a node. They only pin the two decisions that changed:

  ready_is_fresh(...)  a latched `navigation_ready` that stopped updating is
                       NOT readiness;
  settle_exit(...)     INIT_SETTLE ends when fresh readiness AND PX4's own
                       position validity hold, and never later than its cap.

Run:  python3 -m unittest test_np01_handoff -v   (after sourcing ROS)
"""
import os
import re
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import sim_pilot as sp


class TestReadyFreshness(unittest.TestCase):
    def test_fresh_positive(self):
        self.assertTrue(sp.ready_is_fresh(True, 99.9, 100.0, 0.8))

    def test_stale_positive_is_rejected(self):
        self.assertFalse(sp.ready_is_fresh(True, 90.0, 100.0, 0.8))

    def test_never_received_is_rejected(self):
        self.assertFalse(sp.ready_is_fresh(True, None, 100.0, 0.8))

    def test_negative_is_rejected(self):
        self.assertFalse(sp.ready_is_fresh(False, 99.9, 100.0, 0.8))

    def test_boundary(self):
        self.assertTrue(sp.ready_is_fresh(True, 100.0 - 0.8, 100.0, 0.8))
        self.assertFalse(sp.ready_is_fresh(True, 100.0 - 0.801, 100.0, 0.8))


class TestSettleExit(unittest.TestCase):
    def test_waits_below_minimum(self):
        done, why = sp.settle_exit(0.2, True, True, True, 0.5, 3.0)
        self.assertFalse(done, why)

    def test_exits_when_everything_holds(self):
        done, why = sp.settle_exit(0.6, True, True, True, 0.5, 3.0)
        self.assertTrue(done)
        self.assertEqual(why, "px4_position_valid")

    def test_does_not_exit_without_px4_xy_validity(self):
        done, _ = sp.settle_exit(1.0, True, False, True, 0.5, 3.0)
        self.assertFalse(done)

    def test_does_not_exit_without_px4_vxy_validity(self):
        done, _ = sp.settle_exit(1.0, True, True, False, 0.5, 3.0)
        self.assertFalse(done)

    def test_does_not_exit_on_stale_readiness(self):
        done, _ = sp.settle_exit(1.0, False, True, True, 0.5, 3.0)
        self.assertFalse(done)

    def test_cap_is_honoured(self):
        done, why = sp.settle_exit(3.0, False, False, False, 0.5, 3.0)
        self.assertTrue(done)
        self.assertEqual(why, "max_wait")


class TestSerialWaitRemoved(unittest.TestCase):
    def test_translate_branch_has_no_extra_three_second_settle(self):
        src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "sim_pilot.py"), encoding="utf-8").read()
        # the translate branch must call select_position_when_ready directly,
        # with no `+ 3.0` serial settle left in front of it
        idx = src.index("self.select_position_when_ready(elapsed - self.excite_stop_since)")
        before = src[max(0, idx - 600):idx]
        self.assertNotIn("+ 3.0", before,
                         "serial 3 s settle must be removed from the translate branch")
        # the legacy attitude branch keeps its own documented 3 s wait
        legacy = src[src.index("# 正弦而非方波"):]
        self.assertIn("self.excite_stop_since + 3.0", legacy)


if __name__ == "__main__":
    unittest.main(verbosity=2)
