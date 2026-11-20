#!/usr/bin/env python3
"""单轮实验的验收判定（PASS / FAIL / UNVERIFIED）。

判定只使用本轮目录里的证据，缺证据就是 UNVERIFIED，绝不用「RUN_EXIT=0」
或飞手打印 DONE 代替。每个条目都输出实测数值，便于在报告里复核。

用法：
  check_run.py --run <dir> --profile control_truth|vio_closed_loop --distance 5.0

退出码：0 = 全部 PASS；1 = 有 FAIL；2 = 无 FAIL 但有 UNVERIFIED。
"""
import argparse
import csv
import json
import math
import re
import sys
from collections import Counter
from pathlib import Path

import phase4_eval

PASS, FAIL, UNVERIFIED = "PASS", "FAIL", "UNVERIFIED"
EVALUATOR_VERSION = "phase4-v2"
SAMPLE_GAP = 0.5          # 遥测/真值采样允许的最大间隔（s）
READY_MIN_HOLD = 3.0      # navigation_ready 连续下限（s）
READY_MIN_COVER = 0.70    # 就绪窗口对任务窗口的覆盖率下限
POS_TOL = 0.60            # 真值水平位移与指令距离的允许偏差（m）
HOLD_TOL = 0.50           # 到点保持的允许偏差（m）
HOLD_MIN = 2.0            # 保持时长下限（s）
LAND_TOL = 0.35           # 落地后高度残差（m）
VIO_RMSE_MAX = 0.50       # VIO vs 真值水平 RMSE 上限（m）
VIO_MAX_ERR = 1.50        # VIO vs 真值水平最大误差上限（m）

ROS_LINE = re.compile(r"^\[[A-Z]+\] \[(\d+\.\d+)\]")


# ---------------------------------------------------------------- 读取工具

def read_csv(path):
    """读 CSV；列数与表头不一致的行（旧版错位行）直接跳过，不猜。"""
    p = Path(path)
    if not p.exists():
        return []
    with p.open(newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if not header:
            return []
        out = []
        for row in reader:
            if len(row) != len(header):
                continue
            out.append(dict(zip(header, row)))
        return out


def fnum(v, default=None):
    try:
        x = float(v)
        return x if math.isfinite(x) else default
    except (TypeError, ValueError):
        return default


def truthy(v):
    return str(v).strip().lower() in ("1", "true", "yes")


def load_slam_lines(path):
    p = Path(path)
    if not p.exists():
        return []
    out = []
    with p.open(errors="replace") as f:
        for line in f:
            m = ROS_LINE.match(line)
            out.append((float(m.group(1)) if m else None, line.rstrip("\n")))
    return out


# ---------------------------------------------------------------- 证据

def load_telemetry(path):
    rows = []
    for r in read_csv(path):
        wall = fnum(r.get("wall_us"))
        if wall is None:
            continue
        raw = r.get("vio_raw") or ""
        vio = {}
        if raw:
            try:
                parsed = json.loads(raw)
                vio = parsed if isinstance(parsed, dict) else {}
            except ValueError:
                vio = {}
        rows.append({
            "wall": wall / 1e6,
            "ready": truthy(r.get("vio_ready")) or vio.get("navigation_ready") is True,
            "fault": truthy(r.get("vio_fault")) or vio.get("airborne_fault") is True,
            "disarm": truthy(r.get("vio_disarm")) or vio.get("requires_disarm") is True,
            "reason": r.get("vio_reason") or vio.get("reason") or "",
            "gate": vio.get("gate_reason") or vio.get("fault_reason") or "",
            "lp_valid": truthy(r.get("lp_valid")),
            "lp_z": fnum(r.get("lp_z")),
            "lp_x": fnum(r.get("lp_x")),
            "lp_y": fnum(r.get("lp_y")),
            "ev_pos": fnum(r.get("es_ev_pos")),
            "ev_hgt": fnum(r.get("es_ev_hgt")),
            "ev_vel": fnum(r.get("es_ev_vel")),
            "nav": fnum(r.get("st_nav")),
            "arming": fnum(r.get("st_arming")),
            "landed": truthy(r.get("ld_landed")),
            "vio_raw": vio,
        })
    return rows


def load_truth(path):
    rows = []
    for r in read_csv(path):
        sim = fnum(r.get("sim_us"))
        x, y, z = fnum(r.get("x")), fnum(r.get("y")), fnum(r.get("z"))
        wall = fnum(r.get("recv_wall_us"))
        if None in (sim, x, y, z, wall):
            continue
        rows.append({"sim": sim / 1e6, "wall": wall / 1e6, "x": x, "y": y, "z": z})
    rows.sort(key=lambda r: r["sim"])
    return rows


def load_pilot(path):
    rows = []
    for r in read_csv(path):
        wall = fnum(r.get("wall_us"))
        if wall is None:
            continue
        rows.append({
            "wall": wall / 1e6,
            "mono": fnum(r.get("mono_s")),
            "state": (r.get("state") or "").strip(),
            "bt_phase": (r.get("bt_phase") or "").strip(),
            "bt_reason": (r.get("bt_reason") or "").strip(),
            "offboard_ready": truthy(r.get("offboard_ready")),
            "completed": truthy(r.get("task_completed")),
            "control": truthy(r.get("control_active")),
            "armed": truthy(r.get("armed")),
            "nav": fnum(r.get("nav")),
            "z": fnum(r.get("z")),
        })
    rows.sort(key=lambda r: r["wall"])
    return rows


def load_bt(path):
    rows = []
    for r in read_csv(path):
        wall = fnum(r.get("wall_us"))
        if wall is None:
            continue
        rows.append({
            "wall": wall / 1e6,
            "phase": (r.get("phase") or "").strip(),
            "n": fnum(r.get("north_m")),
            "e": fnum(r.get("east_m")),
            "d": fnum(r.get("down_m")),
            "tn": fnum(r.get("target_n")),
            "te": fnum(r.get("target_e")),
        })
    rows.sort(key=lambda r: r["wall"])
    return rows


# ---------------------------------------------------------------- 时间对齐

class TruthClock:
    """sim 时间 -> 主机 epoch 的窗口最小延迟映射（只做时钟域映射）。"""

    def __init__(self, rows):
        self.rows = rows
        self.offsets = [r["wall"] - r["sim"] for r in rows]  # 每帧单程延迟

    def epoch_for_sim(self, sim):
        # 取 sim 附近 ±2 s 内的最小延迟作为该窗口的映射（不修改样本时间戳）
        cand = [o for r, o in zip(self.rows, self.offsets) if abs(r["sim"] - sim) <= 2.0]
        if not cand:
            return None
        return sim + min(cand)

    def sim_for_epoch(self, epoch):
        if not self.rows:
            return None
        best = min(self.rows, key=lambda r: abs(self.epoch_for_sim(r["sim"]) - epoch))
        return best["sim"]

    def stats(self):
        if not self.offsets:
            return {}
        lat = sorted(self.offsets)
        n = len(lat)
        # 这是 clock 域映射的偏移（sim -> epoch），不是单跳传输延迟；
        # 窗口内 max-min 即 offset 抖动/漂移。
        return {"n": n,
                "offset_s": round(lat[0], 6),
                "offset_drift_ms": round((lat[-1] - lat[0]) * 1e3, 2)}


def mission_window(pilot):
    start = next((r["wall"] for r in pilot if r["state"] == "NAVIGATE"), None)
    if start is None:
        return None, None
    end = next((r["wall"] for r in pilot
                if r["wall"] > start and r["state"] in
                ("TAKEOVER", "FAULT", "LAND", "LANDED", "DONE")), None)
    return start, end


def ready_runs(rows):
    """返回 [(start, end, seconds)]，只保留样本间隔连续的 ready 段。"""
    runs, cur = [], None
    last = None
    for r in rows:
        if r["ready"]:
            if cur is None:
                cur = [r["wall"], r["wall"]]
            elif last is not None and r["wall"] - last <= SAMPLE_GAP:
                cur[1] = r["wall"]
            else:
                runs.append(cur)
                cur = [r["wall"], r["wall"]]
            last = r["wall"]
        else:
            if cur is not None:
                runs.append(cur)
                cur = None
            last = r["wall"]
    if cur is not None:
        runs.append(cur)
    out = []
    for a, b in runs:
        out.append((a, b, b - a + 0.1))
    return out


def overlap(a0, a1, b0, b1):
    return max(0.0, min(a1, b1) - max(a0, b0))


# ---------------------------------------------------------------- 判定

def check(checks, name, status, detail, required=True):
    checks.append({"name": name, "status": status, "required": required,
                   "detail": detail})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--profile", required=True)
    ap.add_argument("--distance", type=float, required=True)
    # Phase 4: offline re-evaluation must NOT overwrite the original
    # check.json / check.txt. Use --out-dir to write phase4-v2 results.
    ap.add_argument("--out-dir", default="")
    args = ap.parse_args()
    run = Path(args.run)
    checks = []
    diagnostics = {}

    manifest_p = run / "manifest.json"
    manifest = json.loads(manifest_p.read_text()) if manifest_p.exists() else {}
    tele = load_telemetry(run / "telemetry.csv")
    truth = load_truth(run / "truth.csv")
    pilot = load_pilot(run / "pilot.csv")
    # authority_diag.csv 是授权时效诊断，不是行为树相位记录。
    bt_files = sorted(f for f in (run / "bt").glob("*.csv")
                      if not f.name.startswith("authority_diag"))
    bt = load_bt(bt_files[-1]) if bt_files else []
    slam = load_slam_lines(run / "slam.log")
    slam_text = "\n".join(l for _, l in slam)
    clock = TruthClock(truth)
    m0, m1 = mission_window(pilot)

    # 1. 环境：禁止的绕过开关
    forbidden = manifest.get("forbidden_env_present") or []
    probe_in_bt = "PILOT_LINK_PROBE" in (run / "bt.log").read_text(errors="replace") \
        if (run / "bt.log").exists() else False
    if not manifest:
        check(checks, "no_bypass_env", UNVERIFIED, "manifest.json 缺失，无法确认环境开关")
    else:
        check(checks, "no_bypass_env", FAIL if (forbidden or probe_in_bt) else PASS,
              "forbidden_env=%s bt_probe_mentioned=%s" % (forbidden, probe_in_bt))
        cfg = manifest.get("vio_config")
        if cfg == "transport":
            check(checks, "time_sync_profile", PASS,
                  "vio_config=transport：仅 timesync/样本年龄为 [SIM-ONLY] 传输补偿，"
                  "该两项不得登记为实机同源验收", required=False)
        else:
            check(checks, "time_sync_profile", PASS,
                  "vio_config=%s（无传输层补偿）" % cfg, required=False)

    # 2. PX4 桥接启动（真实 Px4Bridge 在跑）
    if not slam:
        check(checks, "px4_bridge_start", UNVERIFIED, "slam.log 缺失或为空")
    else:
        line = next((l for _, l in slam if "[PX4] domain=" in l), "")
        check(checks, "px4_bridge_start", PASS if line else FAIL,
              line or "slam.log 中没有 [PX4] domain= 行")

    # 3. 行为树/飞手是否真的进入 Offboard 控制
    nav_rows = [r for r in pilot if r["state"] == "NAVIGATE"]
    # pilot.csv 的 nav 列是 MAVLink custom_mode（0x20000/0x30000 量级），
    # 不是 vehicle_status.nav_state；必须以 DDS 遥测的 st_nav 判定。
    nav14 = [r for r in tele if r["nav"] == 14 and
             (m0 is None or m0 - 3.0 <= r["wall"] <= (m1 or r["wall"]) + 3.0)]
    if not pilot or not tele:
        check(checks, "offboard_accepted", UNVERIFIED, "pilot.csv / telemetry.csv 缺失")
    elif nav_rows and nav14:
        check(checks, "offboard_accepted", PASS,
              "NAVIGATE 共 %d 行，DDS nav_state=14(Offboard) 共 %d 行，首末=%s..%s"
              % (len(nav_rows), len(nav14), nav14[0]["wall"], nav14[-1]["wall"]))
    elif nav_rows:
        check(checks, "offboard_accepted", FAIL,
              "飞手拨到 Offboard 档但 DDS 从未出现 nav_state=14"
              "（实测模式=%s）" % dict(Counter(str(r["nav"]) for r in tele).most_common(4)))
    else:
        check(checks, "offboard_accepted", FAIL, "飞手从未进入 NAVIGATE（见 pilot.log）")

    # 4. navigation_ready 连续 >=3 s 且覆盖任务窗口
    if not tele:
        check(checks, "nav_ready_continuous", UNVERIFIED, "telemetry.csv 缺失")
    elif m0 is None or m1 is None:
        check(checks, "nav_ready_continuous", UNVERIFIED, "未找到任务窗口（NAVIGATE 未发生）")
    else:
        runs = ready_runs(tele)
        best = max(runs, key=lambda r: r[2]) if runs else None
        cover = 0.0
        for a, b, _ in runs:
            cover += overlap(a, b, m0, m1)
        ratio = cover / max(1e-6, m1 - m0)
        ok = bool(best and best[2] >= READY_MIN_HOLD and ratio >= READY_MIN_COVER)
        check(checks, "nav_ready_continuous", PASS if ok else FAIL,
              "最长连续 ready=%.2fs（要求>=%.1fs），任务窗口 %.1fs，覆盖率=%.0f%%（要求>=%.0f%%）"
              % (best[2] if best else 0.0, READY_MIN_HOLD, m1 - m0, 100 * ratio,
                 100 * READY_MIN_COVER))

    # 5. 空中故障锁存 / 定位中断
    if not tele:
        check(checks, "no_airborne_fault", UNVERIFIED, "telemetry.csv 缺失")
    else:
        win = [r for r in tele
               if (m0 is None or r["wall"] >= m0 - 2.0) and (m1 is None or r["wall"] <= m1 + 2.0)]
        faults = [r for r in win if r["fault"]]
        gates = Counter(r["gate"] for r in faults if r["gate"])
        if not win:
            check(checks, "no_airborne_fault", UNVERIFIED, "任务窗口内无遥测样本")
        else:
            check(checks, "no_airborne_fault", FAIL if faults else PASS,
                  "窗口样本=%d，airborne_fault=%d，gate=%s"
                  % (len(win), len(faults), dict(gates.most_common(3))))

    # 6. EKF 是否真的把视觉观测用于位置/高度/速度
    if not tele or m0 is None:
        check(checks, "ekf_using_ev", UNVERIFIED, "缺少遥测或任务窗口")
    else:
        win = [r for r in tele if m0 <= r["wall"] <= (m1 or m0 + 1)]
        pos = sum(1 for r in win if r["ev_pos"] == 1) / max(1, len(win))
        hgt = sum(1 for r in win if r["ev_hgt"] == 1) / max(1, len(win))
        vel = sum(1 for r in win if r["ev_vel"] == 1) / max(1, len(win))
        ok = pos >= 0.9 and hgt >= 0.9 and vel >= 0.5
        check(checks, "ekf_using_ev", PASS if ok else FAIL,
              "任务窗口内 cs_ev_pos=%.0f%% hgt=%.0f%% vel=%.0f%%（pos/hgt 要求>=90%%，vel>=50%%）"
              % (100 * pos, 100 * hgt, 100 * vel))

    # 7. 行为树任务是否完成（Hold 阶段 + 目标距离）
    if not bt:
        check(checks, "bt_task_completed", UNVERIFIED, "bt/*.csv 缺失")
    else:
        phases = [r["phase"] for r in bt]
        held = "HOLD" in phases
        # BT 的 target_n/e 是绝对 NED 坐标；任务距离应相对接管锚点计算。
        anchor = next((r for r in bt if r["n"] is not None and r["e"] is not None), None)
        targets = ([math.hypot(r["tn"] - anchor["n"], r["te"] - anchor["e"])
                    for r in bt if r["tn"] is not None and r["te"] is not None]
                   if anchor else [])
        tgt = max(targets) if targets else None
        tgt_ok = tgt is not None and abs(tgt - args.distance) <= 0.15
        check(checks, "bt_task_completed",
              PASS if (held and tgt_ok) else FAIL,
              "phases=%s 到达过 HOLD=%s 目标距离=%s（指令 %.2f m）"
              % (dict(Counter(phases).most_common(6)), held,
                 "%.2f" % tgt if tgt is not None else "无", args.distance))

    # 8. 独立真值：是否真的飞出指令距离并稳定保持
    if not truth or not pilot:
        check(checks, "truth_distance_hold", UNVERIFIED, "truth.csv 或 pilot.csv 缺失")
    else:
        # 只看任务窗口（起飞点 -> 接管时刻），避免把故障后的失控轨迹算进任务距离。
        t_lo = (m0 - 5.0) if m0 is not None else truth[0]["wall"]
        t_hi = (m1 + 2.0) if m1 is not None else truth[-1]["wall"]
        seg = [r for r in truth if t_lo <= r["wall"] <= t_hi]
        if len(seg) < 10:
            seg = truth
        # 锚点取「任务开始时刻最近邻真值」，与行为树的接管锚点同义；
        # 旧写法取 t_lo-5 之后的第一行，实际是全轮最早一帧，口径不对。
        anchor = (min(truth, key=lambda r: abs(r["wall"] - m0))
                  if m0 is not None else seg[0])
        path = [math.hypot(r["x"] - anchor["x"], r["y"] - anchor["y"]) for r in seg]
        dmax = max(path)
        dmax_ok = abs(dmax - args.distance) <= POS_TOL
        # 旧写法 cur 初值为 0 且条件用 `if cur and ...`，导致累加器永远为 0。
        hold = 0.0
        cur = 0.0
        prev_wall = None
        for r, d in zip(seg, path):
            if abs(d - args.distance) <= HOLD_TOL:
                if prev_wall is not None:
                    cur += max(0.0, r["wall"] - prev_wall)
                hold = max(hold, cur)
            else:
                cur = 0.0
            prev_wall = r["wall"]
        check(checks, "truth_distance_hold",
              PASS if (dmax_ok and hold >= HOLD_MIN) else FAIL,
              "真值最大水平位移=%.2f m（指令 %.2f±%.2f），在 %.2f±%.2f m 内最长保持=%.1fs（要求>=%.1fs）"
              % (dmax, args.distance, POS_TOL, args.distance, HOLD_TOL, hold, HOLD_MIN))

    # 9. 接回 Position（人工接管，不是故障退出）
    if not pilot:
        check(checks, "takeover_position", UNVERIFIED, "pilot.csv 缺失")
    else:
        states = [r["state"] for r in pilot]
        takeover = "TAKEOVER" in states
        fault = "FAULT" in states
        # POSCTL 的 nav_state 是 2；22 是 AUTO_VTOL_TAKEOFF，旧判据写错了。
        tail = [r for r in tele if r["wall"] >= (m1 or 0)]
        back_pos = any(r["nav"] == 2 for r in tail)
        check(checks, "takeover_position",
              PASS if (takeover and not fault and back_pos) else FAIL,
              "TAKEOVER=%s FAULT=%s 接管后 DDS nav_state=2(Position)=%s，尾段模式=%s"
              % (takeover, fault, back_pos,
                 dict(Counter(str(r["nav"]) for r in tail).most_common(4))))

    # 10. 落地并解锁
    if not pilot or not truth:
        check(checks, "landed_disarmed", UNVERIFIED, "pilot.csv 或 truth.csv 缺失")
    else:
        last = pilot[-1]
        z0 = truth[0]["z"]
        zend = truth[-1]["z"]
        dz = abs(zend - z0)
        ok = (not last["armed"]) and last["state"] in ("LANDED", "DONE") and dz <= LAND_TOL
        check(checks, "landed_disarmed", PASS if ok else FAIL,
              "末态=%s armed=%s 真值高度残差=%.2f m（要求<=%.2f）"
              % (last["state"], last["armed"], dz, LAND_TOL))

    # ---------------- 以下仅 VIO 闭环需要 ----------------

    if args.profile == "vio_closed_loop":
        # Phase 4 evidence (read-only, kept inside the VIO branch so the
        # control_truth path is byte-for-byte unaffected).
        p4_ready0 = next((r["wall"] for r in tele if r["ready"]), None)
        p4_latency = phase4_eval.load_latency(run / "latency.csv")
        p4_header = phase4_eval.raw_latency_header(run / "latency.csv")
        p4_poses = phase4_eval.load_poses(run / "slam_pose.csv")
        p4_truth = phase4_eval.load_truth_full(run / "truth.csv")
        p4_records, p4_match = phase4_eval.match_pairs(p4_poses, p4_truth)

        # 11. 真实 IMU_STEREO 初始化：VIBA1 + VIBA2
        v1 = "start VIBA 1" in slam_text
        v2 = "start VIBA 2" in slam_text
        end2 = "end VIBA 2" in slam_text
        check(checks, "imu_stereo_viba", PASS if (v1 and v2) else FAIL,
              "VIBA1=%s VIBA2=%s VIBA2 完成=%s" % (v1, v2, end2))

        # 12. SLAM 确实发布过位姿
        pose_rows = read_csv(run / "slam_pose.csv")
        check(checks, "vio_published", PASS if pose_rows else FAIL,
              "/slam/pose 样本=%d" % len(pose_rows))

        # 13. 跟踪健康度（phase4-v2）：A 启动与初始化 / B 接管准备 /
        # C NAVIGATE+HOLD 三阶段分开报告，判定只用 C，且由飞手/BT 状态定义窗口。
        # 旧实现把每 3 s 的 [stat] 累计 ok/lost **求和**并统计整轮，把初始化前
        # NOT_INITIALIZED 冒充任务期丢跟踪；见 phase4/audit/criteria_audit.md。
        st, detail, diag = phase4_eval.evaluate_tracking_health(
            p4_latency, m0, m1, p4_ready0, slam_text)
        diagnostics["vio_tracking_health"] = diag
        check(checks, "vio_tracking_health", st, detail)

        # 14. 姿态门控是否触发（首次故障诊断）
        gate_faults = [l for _, l in slam if "[GATE_FAULT]" in l]
        discont = [l for _, l in slam if "attitude_discontinuity" in l]
        check(checks, "gate_attitude_continuity",
              FAIL if (gate_faults or discont) else PASS,
              "GATE_FAULT=%d 行，attitude_discontinuity=%d 行；首条=%s"
              % (len(gate_faults), len(discont),
                 (gate_faults or discont or ["无"])[0][:160]))

        # 15. VIO 对独立真值的误差（phase4-v2）。
        # 旧实现直接相减「SLAM 地图系中的相机光心」与「Gazebo ENU 中的
        # model 根节点」，把坐标系差异当成定位误差。修正：先按已知外参
        # IMU.T_b_c1 / SDF 杆臂统一参考点，再用任务前校准窗口冻结一次
        # yaw+平移变换，最后只在任务窗口评价；不在任务轨迹上拟合。
        st, detail, diag = phase4_eval.evaluate_vio_vs_truth(
            p4_records, m0, m1, p4_ready0, slam_text, p4_match)
        diagnostics["vio_vs_truth"] = diag
        check(checks, "vio_vs_truth", st, detail)

        # 16. 端到端延迟证据（只作为证据，不参与通过与否）。
        # phase4-v2：先确认时间语义，不用处理耗时代替样本年龄，
        # 不跨域相减，不把未知值填 0。
        st, detail, diag = phase4_eval.evaluate_latency_evidence(p4_latency, p4_header)
        diagnostics["latency_evidence"] = diag
        check(checks, "latency_evidence", st, detail, required=False)

    # ---------------- 汇总 ----------------

    result = {
        "run": str(run),
        "profile": args.profile,
        "distance_m": args.distance,
        "mission_window_epoch": [m0, m1],
        "truth_clock": clock.stats(),
        "evaluator_version": EVALUATOR_VERSION,
        "input_hashes": phase4_eval.input_hashes(run),
        "checks": checks,
        "counts": dict(Counter(c["status"] for c in checks)),
    }
    if args.profile == "vio_closed_loop":
        result["phase4_diagnostics"] = diagnostics
    failed = [c for c in checks if c["required"] and c["status"] == FAIL]
    unknown = [c for c in checks if c["required"] and c["status"] == UNVERIFIED]
    result["verdict"] = FAIL if failed else (UNVERIFIED if unknown else PASS)

    # Offline re-evaluation writes to --out-dir under a _v2 name so the
    # original check.json / check.txt stay byte-for-byte untouched.
    out_dir = Path(args.out_dir) if args.out_dir else run
    out_dir.mkdir(parents=True, exist_ok=True)
    suffix = "_v2" if args.out_dir else ""
    (out_dir / ("check%s.json" % suffix)).write_text(
        json.dumps(result, ensure_ascii=False, indent=2))

    lines = ["=" * 78,
             "run=%s profile=%s distance=%.2fm evaluator=%s"
             % (run, args.profile, args.distance, EVALUATOR_VERSION)]
    if m0 is not None and m1 is not None:
        win_txt = "%.3f..%.3f" % (m0, m1)
    elif m0 is not None:
        win_txt = "%.3f..(未结束)" % m0
    else:
        win_txt = "未找到"
    lines.append("任务窗口: %s  truth 延迟: %s" % (win_txt, result["truth_clock"]))
    lines.append("-" * 78)
    for c in checks:
        lines.append("[%-10s] %-24s %s" % (c["status"], c["name"], c["detail"]))
    lines.append("-" * 78)
    lines.append("VERDICT=%s  counts=%s" % (result["verdict"], result["counts"]))
    lines.append("=" * 78)
    text = "\n".join(lines)
    (out_dir / ("check%s.txt" % suffix)).write_text(text + "\n")
    print(text)
    return {PASS: 0, FAIL: 1, UNVERIFIED: 2}[result["verdict"]]


if __name__ == "__main__":
    sys.exit(main())
