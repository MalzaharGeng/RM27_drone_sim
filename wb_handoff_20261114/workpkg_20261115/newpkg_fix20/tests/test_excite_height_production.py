#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[NP-01 FIX20] 生产路径测试：真实 Pilot.bounded_excite_decision + 历史回放。

不启动 PX4 / DDS / ROS 节点。用 object.__new__(Pilot) 构造真实生产对象，
把 flight1/flight2 的 pilot.csv 逐帧灌成 HeightSample，调用真正的
bounded_excite_decision()，观察返回值、excite_ref、excite_stop_reason、
excite_constraint 与 excite_reset_trace。

覆盖指令 §3 的 7 项 + 边界矩阵：
  1. flight2 旧实现 1.1929 m 可复现
  2. 正确平移后约 0.0373 m
  3. flight1 原正常流程不变（vio_ready_stable，非 fatal）
  4. sticky delta 只应用一次
  5. 真正超过 1 m 的高度变化仍触发
  6. reset 与真实运动同时发生时，真实运动不被抹掉
  7. 缺失/多次/非法 reset 不静默放行
  边界：无 reset 正常变化、正/负 reset、相同 counter 重复、uint8 回绕、
        跨多事件丢失中间 delta、delta 非有限、反馈陈旧、消息乱序、
        包络边界附近 reset、reset 当拍其它判定仍生效、下一拍恢复。
"""
import csv
import json
import math
import os
import sys
import unittest
from types import SimpleNamespace

BASE = "/home/gengyinghao/drone_sim/wb_handoff_20261114/workpkg_20261115/newpkg_fix20"
# 候选版放在 sys.path 首位：测试的是候选的 bounded_excite_decision，
# 但依赖模块（takeoff_altitude_policy / diag_exit_policy）来自生产 scripts 目录。
sys.path.insert(0, "/home/gengyinghao/drone_sim/scripts")
sys.path.insert(0, BASE + "/candidate")

import sim_pilot
print("TESTED sim_pilot from:", sim_pilot.__file__)
from takeoff_altitude_policy import HeightSample
from excite_height_reference import (capture_excite_height_ref,
                                     advance_excite_height_reference,
                                     ExciteHeightRef)

L = "/home/gengyinghao/drone_sim/logs"
R1 = L + "/vio_closed_loop_fix18_func5m_20261115_222616"
R2 = L + "/vio_closed_loop_intr_fix18_r1_20261115_222938"
EXCITE = {"flight1": 1794752844.191805316, "flight2": 1794753045.279537888}
STOP = {"flight1": 1794752856.755689837, "flight2": 1794753059.657865421}


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, m): self.lines.append(("info", m))
    def warning(self, m): self.lines.append(("warning", m))
    def error(self, m): self.lines.append(("error", m))


class _FakeLogger:
    def __init__(self, outer): self.outer = outer

    def get_logger(self): return self.outer


def _args():
    return SimpleNamespace(
        excite_max_rate_dps=300, excite_max_tilt_deg=25,
        excite_max_dz=1.00, excite_max_xy=25.0,
        excite_feedback_stale_s=0.6, excite_input_grace_s=2.0,
        excite_ready_hold=0.4, excite_min_seconds=2.0,
        excite_seconds=20.0, excite_wait_cap_s=25.0,
        excite_gate_progress_s=5.0, excite_slow_period=2.5,
        excite_fast_deg=8.0, excite_fast_deg_max=12.0,
        excite_target_davg=0.60, ready_stale_s=0.8,
    )


def make_pilot():
    p = object.__new__(sim_pilot.Pilot)
    p.args = _args()
    p.excite_ctl = True
    p.excite_ref = None
    p.excite_height_ref = None
    p.excite_last_sample_us = None
    p.excite_reset_trace = []
    p.excite_ready_since = None
    p.excite_ready_streak_s = 0.0
    p.excite_ever_ready = False
    p.excite_stop_reason = ""
    p.excite_constraint = ""
    p.excite_rc_prev = None
    p.excite_rc_rate = 0.0
    p.excite_body_rate_dps = 0.0
    p.excite_tilt_deg = 0.0
    p.excite_disp_xy_m = 0.0
    p.excite_feedback_age_ms = -1.0
    p.excite_stop_since = None
    p.excite_fast_deg = 8.0
    p.excite_gate_seen = True          # 跳过 no_davg_progress 分支
    p.est_davg = 0.61
    p.est_davg_best = 0.71
    p.att_rate_window = []
    p.att_rate_dps = 39.0
    p.body_rate = None
    p.body_tilt = None
    p.hx = 0.0
    p.hy = 0.0
    p.height = None
    p.vio_ready = False
    p.vio_rx_mono = 100.0
    p.body_att_rx_mono = 100.0
    p.ch1 = p.ch2 = p.ch3 = 1500
    p._logger = _Logger()
    p.get_logger = lambda: p._logger
    return p


def sample(z, zrc=1, vzrc=1, delta_z=0.0, sample_us=1000, rx_mono=100.0,
           z_valid=True, v_z_valid=True):
    return HeightSample(z=z, vz=0.02, z_valid=z_valid, v_z_valid=v_z_valid,
                        xy_valid=True, v_xy_valid=True,
                        z_reset_counter=zrc, vz_reset_counter=vzrc,
                        delta_z=delta_z, sample_us=sample_us, rx_mono=rx_mono)


def decide(p, s, now=100.05, elapsed=5.0):
    p.height = s
    return p.bounded_excite_decision(now, elapsed)


def csv_rows(rd, tag):
    """EXCITE 全部采样行 + 随后的**判定帧**（FAULT / INIT_SETTLE 首行）。

    pilot.csv 是定时采样；真实判定循环读的是 DDS 回调里的最新 height，
    那一帧落在 EXCITE 之后的第一行（FIX19 已确认）。漏掉它会把 1.1929
    误算成 0.1246。
    """
    allrows = list(csv.DictReader(open(rd + "/pilot.csv", newline="",
                                       errors="replace")))
    idx = max(i for i, x in enumerate(allrows) if x["state"] == "EXCITE")
    rows = [x for x in allrows if x["state"] == "EXCITE"]
    if idx + 1 < len(allrows):
        rows.append(allrows[idx + 1])      # 判定帧
    out = []
    for r in rows:
        try:
            out.append({
                "t": float(r["wall_us"]) / 1e6 - EXCITE[tag],
                "z": float(r["h_z"]),
                "zrc": int(r["h_zrc"]),
                "vzrc": int(r["h_vzrc"]),
                "dz": float(r["h_delta_z"]),
                "sample_us": int(r["h_sample_us"]),
                "z_valid": r["h_z_valid"] == "1",
                "vz_valid": r["h_vz_valid"] == "1",
                "rx_age_ms": float(r["h_rx_age_ms"] or 0),
                "tilt": float(r["ex_tilt_deg"] or 0),
                "body_rate": float(r["ex_body_rate_dps"] or 0),
                "disp_xy": float(r["ex_disp_xy_m"] or 0),
                "vio_ready": r["vio_ready"] == "1",
                "reason": r["ex_stop_reason"],
                "state": r["state"],
            })
        except (ValueError, TypeError, KeyError):
            continue
    return out


def replay_old(run_rows, ref_z):
    """旧实现：参考固定，逐帧 abs(z-ref)。"""
    mx, first = 0.0, None
    for r in run_rows:
        d = abs(r["z"] - ref_z)
        mx = max(mx, d)
        if d > 1.00 and first is None:
            first = {"t": round(r["t"], 3), "delta_z": round(d, 4), "zrc": r["zrc"]}
    return mx, first


def replay_new(run_rows):
    """ FIX20 生产路径：真实 Pilot.bounded_excite_decision 逐帧回放。"""
    p = make_pilot()
    max_delta = 0.0
    first_fatal = None
    trace = []
    for i, r in enumerate(run_rows):
        s = sample(r["z"], r["zrc"], r["vzrc"], r["dz"], r["sample_us"],
                   rx_mono=100.0 + i * 0.058, z_valid=r["z_valid"],
                   v_z_valid=r["vz_valid"])
        p.height = s
        p.excite_tilt_deg = 0.0     # 由下面的其它判据单独测试
        p.excite_body_rate_dps = 0.0
        p.excite_disp_xy_m = 0.0
        p.vio_ready = r["vio_ready"]     # 回放真实就绪序列，flight1 才能正常结束
        p.vio_rx_mono = 100.0 + i * 0.058
        p.body_att_rx_mono = 100.0 + i * 0.058
        now = 100.05 + i * 0.058
        done, fatal = p.bounded_excite_decision(now, 5.0 + i * 0.058)
        if p.excite_ref is not None:
            max_delta = max(max_delta, abs(r["z"] - p.excite_ref[2]))
        if fatal and first_fatal is None:
            first_fatal = {"t": round(r["t"], 3),
                           "reason": p.excite_stop_reason}
        trace.append({"t": round(r["t"], 3), "zrc": r["zrc"],
                      "dz": r["dz"],
                      "ref": round(p.excite_ref[2], 4) if p.excite_ref else None,
                      "delta": round(abs(r["z"] - p.excite_ref[2]), 4)
                      if p.excite_ref else None,
                      "fatal": fatal, "reason": p.excite_stop_reason})
    return p, max_delta, first_fatal, trace


class TestProductionPath(unittest.TestCase):

    def test_01_old_implementation_reproduces_1_1929(self):
        rows = csv_rows(R2, "flight2")
        ref_z = rows[0]["z"]
        mx, first = replay_old(rows, ref_z)
        self.assertAlmostEqual(ref_z, -1.0125, places=4)
        self.assertAlmostEqual(mx, 1.1929, places=4)
        self.assertIsNotNone(first)
        self.assertAlmostEqual(first["delta_z"], 1.1929, places=4)
        print("  T1 old max=%.4f first=%s" % (mx, first))

    def test_02_translation_yields_about_0_0373(self):
        rows = csv_rows(R2, "flight2")
        p, mx, fatal, trace = replay_new(rows)
        translations = [e for e in p.excite_reset_trace
                        if e["event"] == "reference_translated"]
        self.assertEqual(len(translations), 1, translations)
        tr = translations[0]
        self.assertAlmostEqual(tr["delta_z"], -1.1556, places=4)
        self.assertAlmostEqual(tr["ref_z_before"], -1.0125, places=4)
        self.assertAlmostEqual(tr["ref_z_after"], -2.1681, places=4)
        self.assertEqual((tr["zrc_before"], tr["zrc_after"]), (1, 2))
        self.assertIsNone(fatal, fatal)
        # 判定帧平移后的高度差 = 0.0373
        decision = [e for e in trace if e["t"] >= 14.40][0]
        self.assertAlmostEqual(decision["delta"], 0.0373, places=4)
        print("  T2 translate=%s max_delta=%.4f decision=%s" % (tr, mx, decision))

    def test_03_flight1_normal_flow_unchanged(self):
        rows = csv_rows(R1, "flight1")
        p, mx, fatal, trace = replay_new(rows)
        # flight1 也有一次 reset（+0.1808），但原本就因 vio_ready_stable 正常结束
        self.assertIsNone(fatal, fatal)
        stops = [e["reason"] for e in trace if e["reason"]]
        self.assertIn("vio_ready_stable", stops, stops[:5])
        self.assertNotIn(True, [e["fatal"] for e in trace])
        translations = [e for e in p.excite_reset_trace
                        if e["event"] == "reference_translated"]
        self.assertEqual(len(translations), 1)
        self.assertAlmostEqual(translations[0]["delta_z"], 0.1808, places=4)
        self.assertLess(mx, 1.00)
        print("  T3 flight1 stop=%s max_delta=%.4f translations=%d" %
              (stops[0] if stops else None, mx, len(translations)))

    def test_04_sticky_delta_applied_once(self):
        p = make_pilot()
        # 首帧：历史 sticky delta 已存在但 counter 相同 -> 不得误用
        decide(p, sample(-1.0125, 1, 1, -1.1556, 1000))
        self.assertAlmostEqual(p.excite_ref[2], -1.0125)
        self.assertAlmostEqual(p.excite_height_ref.z, -1.0125)
        d1 = advance_excite_height_reference(
            sample(-1.0125, 1, 1, -1.1556, 1040, 100.058),
            p.excite_height_ref, 100.108, 0.6, 2.0, 1000)
        self.assertEqual(d1.action, "none", d1.reason)
        # reset 新事件：平移一次
        decide(p, sample(-2.2054, 2, 2, -1.1556, 1080, 100.116), 100.166)
        self.assertAlmostEqual(p.excite_ref[2], -2.1681, places=4)
        translations = [e for e in p.excite_reset_trace
                        if e["event"] == "reference_translated"]
        self.assertEqual(len(translations), 1)
        # 后续 20 帧 sticky delta 保持，不得再平移
        for i in range(20):
            decide(p, sample(-2.2054, 2, 2, -1.1556, 1120 + 40 * i,
                             100.174 + 0.058 * i), 100.224 + 0.058 * i)
        self.assertAlmostEqual(p.excite_ref[2], -2.1681, places=4)
        self.assertEqual(len([e for e in p.excite_reset_trace
                              if e["event"] == "reference_translated"]), 1)
        print("  T4 sticky applied once, ref=%.4f" % p.excite_ref[2])

    def test_05_real_climb_over_1m_still_triggers(self):
        p = make_pilot()
        decide(p, sample(-1.0, 1, 1, 0.0, 1000))
        # 真实爬升 1.2 m（NED z 更负），无 reset
        done, fatal = decide(p, sample(-2.2, 1, 1, 0.0, 1040, 100.058), 100.108)
        self.assertTrue(fatal)
        self.assertIn("delta_z(1.20>1.00m)", p.excite_stop_reason)
        print("  T5 real climb ->", p.excite_stop_reason)

    def test_06_reset_plus_real_motion_not_erased(self):
        p = make_pilot()
        decide(p, sample(-1.0, 1, 1, 0.0, 1000))
        # 真实爬升 0.30 m + 坐标 reset -1.1556（同拍发生）
        # z_after = -1.0 + (-1.1556) + (-0.30) = -2.4556
        z_after = -1.0 - 1.1556 - 0.30
        decide(p, sample(z_after, 2, 2, -1.1556, 1040, 100.058), 100.108)
        self.assertAlmostEqual(p.excite_ref[2], -2.1556, places=4)
        # 下一有效样本恢复高度包络判定；真实 0.30 m 仍在
        done, fatal = decide(p, sample(z_after, 2, 2, -1.1556, 1080, 100.116),
                             100.166)
        self.assertFalse(fatal)
        self.assertAlmostEqual(abs(z_after - p.excite_ref[2]), 0.30, places=4)
        # 继续真实爬升到累计 > 1 m，必须仍触发（不是被抹掉）
        z2 = p.excite_ref[2] - 1.10
        done2, fatal2 = decide(p, sample(z2, 2, 2, -1.1556, 1120, 100.174),
                               100.224)
        self.assertTrue(fatal2, p.excite_stop_reason)
        self.assertIn("delta_z(", p.excite_stop_reason)
        print("  T6 real 0.30 m preserved, later 1.10 m ->", p.excite_stop_reason)


class TestBoundaryMatrix(unittest.TestCase):

    def _step(self, s, ref, last_us, now=100.05):
        return advance_excite_height_reference(s, ref, now, 0.6, 2.0, last_us)

    def test_07_boundary_matrix(self):
        r0 = capture_excite_height_ref(sample(-1.0, 1, 1, 0.0, 1000))[0]
        cases = []
        # 无 reset 正常变化
        cases.append(("no_reset_normal", sample(-1.3, 1, 1, 0.0, 1040, 100.058),
                      "none", r0))
        # 正 reset
        cases.append(("positive_reset", sample(-0.82, 2, 2, 0.1808, 1040, 100.058),
                      "translate", r0))
        # 负 reset
        cases.append(("negative_reset", sample(-2.16, 2, 2, -1.1556, 1040, 100.058),
                      "translate", r0))
        # 相同 counter 反复到达（sticky delta 不变）
        cases.append(("same_counter_repeated", sample(-2.16, 2, 2, -1.1556, 1040, 100.058),
                      "translate", r0))  # 单独在下面用 ref.zrc=2 验证
        # uint8 正常回绕 255 -> 0
        r255 = capture_excite_height_ref(sample(-1.0, 255, 255, 0.0, 1000))[0]
        cases.append(("uint8_wrap_255_to_0",
                      sample(-2.16, 0, 0, -1.1556, 1040, 100.058),
                      "translate", r255))
        # 跨多事件 / 丢失中间 delta
        cases.append(("multi_event_step2", sample(-2.16, 3, 3, -1.1556, 1040, 100.058),
                      "fail", r0))
        # 回退 / 乱序 counter
        cases.append(("counter_backwards", sample(-2.16, 0, 0, -1.1556, 1040, 100.058),
                      "fail", r0))
        # delta 非有限
        for bad in (float("nan"), float("inf")):
            cases.append(("non_finite_delta",
                          sample(-2.16, 2, 2, bad, 1040, 100.058), "fail", r0))
        # delta = 0 但 counter 变化（缺失 delta）
        cases.append(("missing_delta", sample(-2.16, 2, 2, 0.0, 1040, 100.058),
                      "fail", r0))
        # delta 超出健全性界
        cases.append(("delta_out_of_bounds", sample(-2.16, 2, 2, 3.5, 1040, 100.058),
                      "fail", r0))
        # 反馈陈旧
        cases.append(("stale_feedback",
                      sample(-2.16, 2, 2, -1.1556, 1040, 99.0), "fail", r0))
        # 消息乱序（sample_us 倒退）
        cases.append(("out_of_order",
                      sample(-2.16, 2, 2, -1.1556, 900, 100.058), "fail", r0))
        # z 无效
        cases.append(("z_invalid",
                      sample(-2.16, 2, 2, -1.1556, 1040, 100.058, z_valid=False),
                      "fail", r0))
        # 相同 counter + sticky delta：用 ref.zrc=2 验证不重复应用
        r2 = ExciteHeightRef(-2.1681, 2, 2, 1080)
        cases.append(("same_counter_sticky",
                      sample(-2.20, 2, 2, -1.1556, 1120, 100.116),
                      "none", r2))
        # 参考未捕获
        cases.append(("reference_not_captured",
                      sample(-2.16, 2, 2, -1.1556, 1040, 100.058), "fail", None))

        results = []
        for name, s, expected, ref in cases:
            last = 1000 if ref is not None else None
            d = self._step(s, ref, last)
            results.append({"case": name, "expected": expected,
                            "got": d.action, "reason": d.reason,
                            "pass": d.action == expected})
        print("  T7 boundary matrix:")
        for r in results:
            print("     %-26s exp=%-9s got=%-9s %s %s" %
                  (r["case"], r["expected"], r["got"], r["reason"],
                   "OK" if r["pass"] else "FAIL"))
        for r in results:
            self.assertEqual(r["got"], r["expected"], r)

    def test_08_reset_tick_other_checks_still_active(self):
        # reset 当拍：角速度/倾角/disp_xy 仍必须能触发
        p = make_pilot()
        decide(p, sample(-1.0, 1, 1, 0.0, 1000))
        s = sample(-2.16, 2, 2, -1.1556, 1040, 100.058)
        p.height = s
        p.excite_body_rate_dps = 400.0        # 超过 300
        p.excite_tilt_deg = 0.0
        p.excite_disp_xy_m = 0.0
        p.vio_ready = False
        p.vio_rx_mono = 100.058
        p.body_att_rx_mono = 100.058
        done, fatal = p.bounded_excite_decision(100.108, 5.0)
        self.assertTrue(fatal)
        self.assertIn("angular_rate", p.excite_stop_reason)
        print("  T8 reset tick angular_rate still fires:", p.excite_stop_reason)

        p2 = make_pilot()
        decide(p2, sample(-1.0, 1, 1, 0.0, 1000))
        p2.height = sample(-2.16, 2, 2, -1.1556, 1040, 100.058)
        p2.excite_body_rate_dps = 0.0
        p2.excite_tilt_deg = 30.0            # 超过 25
        p2.excite_disp_xy_m = 0.0
        p2.vio_ready = False
        p2.vio_rx_mono = 100.058
        p2.body_att_rx_mono = 100.058
        done2, fatal2 = p2.bounded_excite_decision(100.108, 5.0)
        self.assertTrue(fatal2)
        self.assertIn("tilt", p2.excite_stop_reason)
        print("  T8 reset tick tilt still fires:", p2.excite_stop_reason)

    def test_09_next_valid_sample_resumes_envelope(self):
        p = make_pilot()
        decide(p, sample(-1.0, 1, 1, 0.0, 1000))
        # 当前拍发生 reset（暂缓 delta_z）
        d1 = decide(p, sample(-2.16, 2, 2, -1.1556, 1040, 100.058), 100.108)
        self.assertFalse(d1[1])
        # 下一有效样本若真实高度差 > 1 m 必须触发
        d2 = decide(p, sample(-3.20, 2, 2, -1.1556, 1080, 100.116), 100.166)
        self.assertTrue(d2[1])
        self.assertIn("delta_z(", p.excite_stop_reason)
        print("  T9 next sample resumes ->", p.excite_stop_reason)

    def test_10_reset_near_envelope_boundary(self):
        p = make_pilot()
        decide(p, sample(-1.0, 1, 1, 0.0, 1000))
        # 真实高度已到 0.95 m（接近 1.00 边界），同时发生 -1.1556 reset
        z = -1.0 - 0.95 - 1.1556
        decide(p, sample(z, 2, 2, -1.1556, 1040, 100.058), 100.108)
        self.assertAlmostEqual(abs(z - p.excite_ref[2]), 0.95, places=4)
        # 下一拍仍在边界内 -> 不触发
        d = decide(p, sample(z, 2, 2, -1.1556, 1080, 100.116), 100.166)
        self.assertFalse(d[1], p.excite_stop_reason)
        # 再真实爬升 0.10 m 越过 1.00 -> 触发
        d2 = decide(p, sample(z - 0.10, 2, 2, -1.1556, 1120, 100.174), 100.224)
        self.assertTrue(d2[1])
        self.assertIn("delta_z(1.05", p.excite_stop_reason)
        print("  T10 boundary preserved ->", p.excite_stop_reason)

    def test_11_unrecoverable_reset_fails_closed_in_production(self):
        p = make_pilot()
        decide(p, sample(-1.0, 1, 1, 0.0, 1000))
        # counter 跨 2 步（丢失中间 delta）-> 生产路径必须 fatal 且明确报错
        done, fatal = decide(p, sample(-2.16, 3, 3, -1.1556, 1040, 100.058),
                             100.108)
        self.assertTrue(fatal)
        self.assertIn("height_reset_unrecoverable", p.excite_stop_reason)
        self.assertEqual(p.excite_constraint, p.excite_stop_reason)
        print("  T11 fail-closed ->", p.excite_stop_reason)


def main():
    suite = unittest.defaultTestLoader.loadTestsFromModule(
        sys.modules["__main__"])
    res = unittest.TextTestRunner(verbosity=2).run(suite)
    out = BASE + "/evidence/production_test_results.json"
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump({"tests_run": res.testsRun, "failures": len(res.failures),
               "errors": len(res.errors),
               "passed": res.wasSuccessful()},
              open(out, "w"), indent=2)
    return 0 if res.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main())
