#!/usr/bin/env python3
"""Phase 4 evaluator: corrected measurement semantics for two check_run.py checks.

Only the *evaluation* is changed. No control, gating, thresholds, flight or
simulation code is touched. All original numeric thresholds are kept verbatim
(VIO_RMSE_MAX / VIO_MAX_ERR / TRACK_LOST_MAX / MATCH_DT_MAX / MIN_SAMPLES).

Design constraints from the phase-4 task:
  * VIO is metric: never fit a scale.
  * One fixed transform, frozen BEFORE the first NAVIGATE.
  * Calibration window is defined by state + time, not chosen for lowest error.
  * Never fit on the NAVIGATE/HOLD trajectory that is being evaluated.
  * No per-segment / per-frame re-alignment, no time-offset search.
  * Reference points are converted with KNOWN extrinsics before any alignment.
  * Both inertial worlds are gravity-constrained: prefer yaw + translation;
    report the full SE(3) comparison and its roll/pitch corrections.
  * If the transform is not observable: UNVERIFIED, never force a fit.
  * Epoch / reference discontinuities are reported, not fitted away.

The functions here are pure enough to be unit-tested with synthetic records.
"""
import csv
import hashlib
import json
import math
import re
import statistics
from pathlib import Path

PASS, FAIL, UNVERIFIED = "PASS", "FAIL", "UNVERIFIED"

# ---- original numeric thresholds (UNCHANGED) -------------------------------
VIO_RMSE_MAX = 0.50        # horizontal RMSE (m)
VIO_MAX_ERR = 1.50         # horizontal max error (m)
TRACK_LOST_MAX = 0.20      # lost frame ratio (20%)
MATCH_DT_MAX = 0.020       # nearest-neighbour sim-time match (20 ms)
MIN_SAMPLES = 20           # minimum matched samples

# ---- known extrinsics (config / SDF, not estimated from data) --------------
# RM27_Sim_IMU_Stereo_baseline.yaml: IMU.T_b_c1
#   R_bc (camera optical -> body FLU), t_bc = camera origin in body FLU.
# x500_mono_cam/model.sdf: cam_left_link pose relative to base_link
#   = 0.12 0.025 0.242 (same numbers).
R_BC = ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0))
T_BC = (0.120, 0.025, 0.242)


def _transpose(M):
    return tuple(tuple(M[j][i] for j in range(3)) for i in range(3))


def _matvec(M, v):
    return tuple(sum(M[i][j] * v[j] for j in range(3)) for i in range(3))


def _neg(v):
    return tuple(-x for x in v)


# Body (IMU) origin expressed in the camera optical frame = -R_bc^T * t_bc.
D_CAM_TO_BODY = _neg(_matvec(_transpose(R_BC), T_BC))  # (0.025, 0.242, -0.12)

TRACK_OK = (2, 5)             # Tracking::OK, OK_KLT
TRACK_LOST = (3, 4)           # RECENTLY_LOST, LOST
TRACK_NOT_INIT = (1,)         # NOT_INITIALIZED
ROS_LINE = re.compile(r"^\[[A-Z]+\] \[(\d+\.\d+)\]")


# ---------------------------------------------------------------- geometry

def quat_to_R(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not n:
        return ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def euler_deg(R):
    """ZYX roll/pitch/yaw in degrees (diagnostic only)."""
    sy = min(1.0, max(-1.0, -R[2][0]))
    pitch = math.asin(sy)
    roll = math.atan2(R[2][1], R[2][2])
    yaw = math.atan2(R[1][0], R[0][0])
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def rigidyaw(A, B):
    """One yaw-only (about z) + translation transform, no scale, no roll/pitch.

    A/B are sequences of (x,y,z). Returns (c, s, tx, ty, tz, yaw_deg, residual).
    Physical contract: both worlds are inertial and gravity constrained, so
    their z axes are parallel; only a heading and an origin differ.
    """
    n = len(A)
    ax = sum(a[0] for a in A) / n
    ay = sum(a[1] for a in A) / n
    az = sum(a[2] for a in A) / n
    bx = sum(b[0] for b in B) / n
    by = sum(b[1] for b in B) / n
    bz = sum(b[2] for b in B) / n
    dot = cross = 0.0
    for a, b in zip(A, B):
        vx, vy = a[0] - ax, a[1] - ay
        tx, ty = b[0] - bx, b[1] - by
        dot += vx * tx + vy * ty
        cross += vx * ty - vy * tx
    theta = math.atan2(cross, dot)
    c, s = math.cos(theta), math.sin(theta)
    tvec = (bx - (c * ax - s * ay), by - (s * ax + c * ay), bz - az)
    residual = math.sqrt(sum(
        (c * a[0] - s * a[1] + tvec[0] - b[0]) ** 2 +
        (s * a[0] + c * a[1] + tvec[1] - b[1]) ** 2
        for a, b in zip(A, B)) / n)
    return (c, s, tvec[0], tvec[1], tvec[2], math.degrees(theta), residual)


def apply_rigidyaw(T, p):
    c, s, tx, ty, tz, _, _ = T
    return (c * p[0] - s * p[1] + tx,
            s * p[0] + c * p[1] + ty,
            p[2] + tz)


def procrustes_se3(A, B):
    """Full SE(3) (no scale) for DIAGNOSTIC comparison only."""
    try:
        import numpy as np
    except ImportError:
        return None
    a = np.asarray(A, float)
    b = np.asarray(B, float)
    ma, mb = a.mean(axis=0), b.mean(axis=0)
    H = (a - ma).T @ (b - mb)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = mb - R @ ma
    return tuple(tuple(float(x) for x in row) for row in R), tuple(float(x) for x in t)


# ---------------------------------------------------------------- loading

def _rows(path):
    p = Path(path)
    if not p.exists():
        return []
    with p.open(newline="") as f:
        return list(csv.DictReader(f))


def _fn(v, default=None):
    try:
        x = float(v)
    except (TypeError, ValueError):
        return default
    return x if math.isfinite(x) else default


def load_poses(path):
    """/slam/pose: camera optical centre in the SLAM map frame."""
    out = []
    for r in _rows(path):
        s = _fn(r.get("stamp_sim_us"))
        ep = _fn(r.get("recv_wall_us"))
        xyz = [_fn(r.get(k)) for k in ("x", "y", "z")]
        q = [_fn(r.get(k), 0.0) for k in ("qx", "qy", "qz", "qw")]
        if None in (s, ep, *xyz):
            continue
        R = quat_to_R(*q)                       # camera -> map
        lever = _matvec(R, D_CAM_TO_BODY)        # known extrinsics
        body = tuple(xyz[i] + lever[i] for i in range(3))
        out.append({"sim": s / 1e6, "epoch": ep / 1e6,
                    "cam": tuple(xyz), "body": body})
    return out


def load_truth_full(path):
    """truth.csv: model root == base_link == IMU origin, in Gazebo world ENU."""
    out = []
    for r in _rows(path):
        s = _fn(r.get("sim_us"))
        ep = _fn(r.get("recv_wall_us"))
        xyz = [_fn(r.get(k)) for k in ("x", "y", "z")]
        if None in (s, ep, *xyz):
            continue
        out.append({"sim": s / 1e6, "epoch": ep / 1e6, "xyz": tuple(xyz),
                    "R": quat_to_R(*[_fn(r.get(k), 0.0)
                                     for k in ("qx", "qy", "qz", "qw")])})
    out.sort(key=lambda r: r["sim"])
    return out


def load_latency(path):
    out = []
    for r in _rows(path):
        ep = _fn(r.get("rx_wall_us"))
        st = _fn(r.get("tracking_state"))
        pair = _fn(r.get("pair"))
        if None in (ep, st, pair):
            continue
        out.append({"epoch": ep / 1e6, "state": int(st), "pair": int(pair),
                    "sim_us": _fn(r.get("sim_us")),
                    "proc_ms": ((_fn(r.get("track_end_us"), 0) or 0) -
                                (_fn(r.get("track_start_us"), 0) or 0)) / 1000.0})
    out.sort(key=lambda r: r["epoch"])
    return out


def input_hashes(run):
    out = {}
    for name in ("slam_pose.csv", "truth.csv", "pilot.csv", "telemetry.csv",
                 "latency.csv", "slam.log", "check.json", "check.txt",
                 "manifest.json"):
        p = Path(run) / name
        if p.exists():
            out[name] = hashlib.md5(p.read_bytes()).hexdigest()
    return out


def match_pairs(poses, truth):
    """Nearest-neighbour in the SIM domain, max 20 ms. Returns records + stats."""
    tsim = [r["sim"] for r in truth]
    recs, dts = [], []
    for p in poses:
        lo, hi = 0, len(tsim) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if tsim[mid] < p["sim"]:
                lo = mid + 1
            else:
                hi = mid
        cand = [i for i in (lo - 1, lo, lo + 1) if 0 <= i < len(truth)]
        if not cand:
            continue
        i = min(cand, key=lambda k: abs(tsim[k] - p["sim"]))
        dt = abs(tsim[i] - p["sim"])
        if dt > MATCH_DT_MAX:
            continue
        recs.append({"sim": p["sim"], "epoch": p["epoch"],
                     "vio": p["body"], "truth": truth[i]["xyz"], "dt": dt})
        dts.append(dt)
    return recs, {"n": len(recs),
                  "dt_ms_p50": round(1000 * statistics.median(dts), 2) if dts else None,
                  "dt_ms_max": round(1000 * max(dts), 2) if dts else None}


def epoch_discontinuities(slam_text):
    """Report epoch / reference changes; never smooth them away by re-fitting."""
    changes, prev = [], None
    for line in slam_text.splitlines():
        m = ROS_LINE.match(line)
        em = re.search(r"\[stat\].*?epoch=(\d+)", line)
        if em:
            e = int(em.group(1))
            if prev is not None and e != prev:
                changes.append({"wall": float(m.group(1)) if m else None,
                                "from": prev, "to": e})
            prev = e
    return {"epoch_values_seen": sorted(set(
                int(x) for x in re.findall(r"\[stat\].*?epoch=(\d+)", slam_text))),
            "epoch_changes": changes,
            "time_base_reanchor": len(re.findall(r"time base re-anchored", slam_text)),
            "track_reset": len(re.findall(r"TRACK: Reset", slam_text))}


# ---------------------------------------------------------------- vio_vs_truth

def evaluate_vio_vs_truth(records, m0, m1, ready0, slam_text="", match_stats=None):
    """Return (status, detail, diagnostics). records: output of match_pairs().

    Calibration window: [first navigation_ready, first NAVIGATE), frozen before
    the mission. Mission window: [m0, m1]. Windows are in the epoch domain
    (each record carries recv_wall_us), time matching is in the sim domain.
    """
    diag = {"evaluator": "phase4-v2",
            "reference_point": "camera optical centre -> body/IMU via IMU.T_b_c1",
            "lever_arm_cam_to_body_m": list(D_CAM_TO_BODY),
            "scale_fitted": False,
            "transform_source": "calibration window only (pre-NAVIGATE)",
            "discontinuities": epoch_discontinuities(slam_text),
            "match_stats": match_stats or {}}
    if m0 is None or m1 is None or ready0 is None:
        return UNVERIFIED, ("无有效任务窗口或 navigation_ready 起点"
                            "（m0=%s m1=%s ready=%s）：没有冻结变换的依据" %
                            (m0, m1, ready0)), diag
    cal = [r for r in records if ready0 <= r["epoch"] < m0]
    mis = [r for r in records if m0 <= r["epoch"] <= m1]
    diag["calibration_window_epoch"] = [ready0, m0]
    diag["mission_window_epoch"] = [m0, m1]
    diag["calibration_pairs"] = len(cal)
    diag["mission_pairs"] = len(mis)
    if len(cal) < MIN_SAMPLES:
        return UNVERIFIED, ("校准窗口可对齐样本 %d < %d，变换不可观测，"
                            "不强行拟合（任务窗口样本 %d）" %
                            (len(cal), MIN_SAMPLES, len(mis))), diag
    if len(mis) < MIN_SAMPLES:
        return UNVERIFIED, ("任务窗口可对齐样本 %d < %d" %
                            (len(mis), MIN_SAMPLES)), diag

    A = [r["vio"] for r in cal]
    B = [r["truth"] for r in cal]
    # Observability: the calibration motion must actually constrain heading.
    span = math.hypot(B[-1][0] - B[0][0], B[-1][1] - B[0][1])
    dirs = []
    for a, b in zip(B, B[1:]):
        d = (b[0] - a[0], b[1] - a[1])
        if math.hypot(*d) > 1e-4:
            dirs.append(math.degrees(math.atan2(d[1], d[0])))
    spread = (max(dirs) - min(dirs)) if dirs else 0.0
    half = len(cal) // 2
    T_all = rigidyaw(A, B)
    T_a = rigidyaw(A[:half], B[:half]) if half >= MIN_SAMPLES else None
    T_b = rigidyaw(A[half:], B[half:]) if len(cal) - half >= MIN_SAMPLES else None
    yaw_split = (abs(((T_a[5] - T_b[5] + 180) % 360) - 180)
                 if T_a and T_b else None)
    diag["observability"] = {
        "calibration_truth_span_m": round(span, 4),
        "calibration_heading_spread_deg": round(spread, 1),
        "calibration_fit_residual_m": round(T_all[6], 4),
        "yaw_split_half_diff_deg": (round(yaw_split, 2) if yaw_split is not None
                                    else None)}
    if span < 0.05 or spread < 30.0 or (yaw_split is not None and yaw_split > 5.0):
        diag["observability"]["verdict"] = "INSUFFICIENT"
        return UNVERIFIED, ("任务前运动不足以确定 yaw：span=%.3f m，"
                            "方向散布=%.1f deg，分半 yaw 差=%s；不强行拟合" %
                            (span, spread,
                             "%.2f" % yaw_split if yaw_split is not None else "n/a")), diag
    diag["observability"]["verdict"] = "OBSERVABLE"

    fixed = {"type": "yaw+translation", "yaw_deg": round(T_all[5], 3),
             "tx": round(T_all[2], 4), "ty": round(T_all[3], 4),
             "tz": round(T_all[4], 4),
             "basis": "both inertial worlds gravity constrained (z parallel); "
                      "one rigid yaw + origin, no scale, frozen at first NAVIGATE"}
    se3 = procrustes_se3(A, B)
    if se3:
        fixed["se3_comparison_roll_pitch_yaw_deg"] = [
            round(v, 2) for v in euler_deg(se3[0])]
    diag["fixed_transform"] = fixed

    errs, curve, gaps = [], [], []
    for r in mis:
        p = apply_rigidyaw(T_all, r["vio"])
        e = math.hypot(p[0] - r["truth"][0], p[1] - r["truth"][1])
        errs.append(e)
        curve.append([round(r["sim"], 3), round(e, 4)])
    sims = sorted(r["sim"] for r in mis)
    nominal = (statistics.median([b - a for a, b in zip(sims, sims[1:])])
               if len(sims) > 1 else 1 / 30.0)
    for a, b in zip(sims, sims[1:]):
        if b - a > 3 * nominal:
            gaps.append([round(a, 3), round(b, 3), round(b - a, 3)])
    diag["mission_time_match"] = {
        "nominal_frame_interval_s": round(nominal, 4),
        "output_gaps": gaps, "gap_count": len(gaps),
        "gap_total_s": round(sum(g[2] for g in gaps), 3)}
    diag["error_curve"] = curve
    rmse = math.sqrt(sum(e * e for e in errs) / len(errs))
    mx = max(errs)
    diag["rmse_m"] = round(rmse, 4)
    diag["max_m"] = round(mx, 4)
    diag["thresholds"] = {"rmse_max_m": VIO_RMSE_MAX, "max_err_m": VIO_MAX_ERR,
                          "match_dt_s": MATCH_DT_MAX, "min_samples": MIN_SAMPLES}
    ok = rmse <= VIO_RMSE_MAX and mx <= VIO_MAX_ERR
    return (PASS if ok else FAIL,
            ("校准窗口冻结 yaw=%+.2f deg；任务窗口样本=%d，水平 RMSE=%.4f m"
             "（<=%.2f） 最大=%.4f m（<=%.2f）；时间匹配 p50=%s ms max=%s ms；"
             "输出空窗=%d 段/%.3f s" %
             (T_all[5], len(errs), rmse, VIO_RMSE_MAX, mx, VIO_MAX_ERR,
              diag.get("match_stats", {}).get("dt_ms_p50"),
              diag.get("match_stats", {}).get("dt_ms_max"),
              len(gaps), sum(g[2] for g in gaps))), diag)


# ---------------------------------------------------------------- tracking

def _stage_stats(frames):
    c = {"frames": len(frames), "ok": 0, "not_initialized": 0,
         "recently_lost": 0, "lost": 0, "other": 0}
    for f in frames:
        if f["state"] in TRACK_OK:
            c["ok"] += 1
        elif f["state"] in TRACK_NOT_INIT:
            c["not_initialized"] += 1
        elif f["state"] == 3:
            c["recently_lost"] += 1
        elif f["state"] == 4:
            c["lost"] += 1
        else:
            c["other"] += 1
    # The node counts "lost" as EVERY frame whose state is not OK/OK_KLT
    # (stereo_inertial_sim_node.cpp:575-578). Keep that semantic: a
    # NOT_INITIALIZED / NO_IMAGES_YET frame inside the mission is unhealthy.
    lost = c["frames"] - c["ok"]
    c["lost_frame_ratio"] = round(lost / c["frames"], 4) if c["frames"] else None
    return c


def evaluate_tracking_health(latency, m0, m1, ready0, slam_text=""):
    """Three stages A/B/C. The verdict uses stage C (NAVIGATE + HOLD) only.

    Frame-ratio threshold stays 20% (original). A separate time-coverage
    figure is reported so an output stall cannot fake a low lost ratio.
    """
    diag = {"evaluator": "phase4-v2",
            "source": "latency.csv per-frame tracking_state + rx_wall_us(epoch)",
            "windows_defined_by":
                "A=[first frame, first navigation_ready), "
                "B=[first navigation_ready, NAVIGATE), C=[NAVIGATE, TAKEOVER]"}
    if not latency:
        return UNVERIFIED, "无逐帧跟踪记录（latency.csv 缺失或为空），不能把没有记录到失败当作健康", diag
    first = latency[0]["epoch"]
    first_ok = next((f["epoch"] for f in latency if f["state"] in TRACK_OK), None)
    # A/B boundary is the Gate-accepted navigation_ready, NOT the first
    # isolated OK frame: the tracker briefly reports OK while repeatedly
    # re-initialising the map (which would put the whole 47 s init into B).
    init_boundary = ready0 if ready0 is not None else first_ok
    whole = _stage_stats(latency)
    A = [f for f in latency
         if f["epoch"] < (init_boundary if init_boundary is not None
                          else float("inf"))]
    B = ([f for f in latency
          if init_boundary is not None and init_boundary <= f["epoch"] < m0]
         if m0 is not None else [])
    C = ([f for f in latency if m0 <= f["epoch"] <= m1]
         if m0 is not None and m1 is not None else [])
    diag["whole_run_distribution"] = whole
    diag["stage_boundaries_epoch"] = {
        "A_start": first, "A_end_B_start": init_boundary,
        "B_end_C_start": m0, "C_end": m1}
    diag["stage_A_startup_init"] = _stage_stats(A)
    diag["stage_B_pre_takeover"] = _stage_stats(B)
    diag["stage_C_mission"] = _stage_stats(C)
    # "Initialisation completed" uses the Gate-accepted navigation_ready,
    # not the first isolated OK frame (the tracker briefly reports OK during
    # repeated map initialisation, which would give a bogus 0.04 s).
    diag["init_completed"] = ready0 is not None
    diag["init_seconds"] = (round(ready0 - first, 2)
                            if ready0 is not None else None)
    diag["init_basis"] = "first navigation_ready epoch minus first latency frame"
    diag["first_ok_frame_seconds"] = (round(first_ok - first, 2)
                                      if first_ok is not None else None)
    diag["never_initialized"] = first_ok is None
    diag["mission_window_epoch"] = [m0, m1]
    if m0 is None or m1 is None:
        return UNVERIFIED, (
            "未找到由飞手/BT 状态定义的任务窗口（NAVIGATE 未发生）；不能把整轮分布"
            "当任务段质量指标。整轮诊断：帧=%d 非OK=%d 丢失帧比例=%s，"
            "NOT_INITIALIZED=%d，首次OK=%s，navigation_ready=%s" %
            (whole["frames"], whole["frames"] - whole["ok"],
             whole["lost_frame_ratio"], whole["not_initialized"],
             first_ok is not None, ready0 is not None)), diag
    if len(C) < MIN_SAMPLES:
        return UNVERIFIED, ("任务窗口逐帧记录仅 %d 帧 < %d，记录不足，"
                            "不能判定任务期间健康" % (len(C), MIN_SAMPLES)), diag

    T = m1 - m0
    times = [f["epoch"] for f in C]
    gaps_epoch = [b - a for a, b in zip(times, times[1:])]
    nominal = statistics.median(gaps_epoch) if gaps_epoch else 1 / 30.0
    covered = 0.0
    lost_time = 0.0
    gap_time = 0.0
    for i, f in enumerate(C):
        nxt = C[i + 1]["epoch"] if i + 1 < len(C) else min(m1, f["epoch"] + nominal)
        dt = max(0.0, nxt - f["epoch"])
        if dt > 3 * nominal:            # output stall: do not let it vanish
            gap_time += dt - 3 * nominal
            dt = 3 * nominal
        covered += dt
        if f["state"] not in TRACK_OK:
            lost_time += dt
    uncovered = max(0.0, T - covered)
    time_uncovered_ratio = (uncovered + lost_time + gap_time) / max(1e-6, T)
    final_state = C[-1]["state"]
    frame_ratio = diag["stage_C_mission"]["lost_frame_ratio"]
    diag["time_coverage"] = {
        "window_s": round(T, 2), "nominal_frame_interval_s": round(nominal, 4),
        "covered_s": round(covered, 2), "uncovered_s": round(uncovered, 2),
        "lost_time_s": round(lost_time, 2), "gap_time_s": round(gap_time, 2),
        "time_uncovered_ratio": round(time_uncovered_ratio, 4)}
    diag["final_state_in_window"] = final_state
    diag["thresholds"] = {"lost_frame_ratio_max": TRACK_LOST_MAX,
                          "time_uncovered_ratio_max": TRACK_LOST_MAX,
                          "min_samples": MIN_SAMPLES}
    diag["legacy_bug"] = ("old check_run.py summed CUMULATIVE ok/lost from every "
                          "3-second [stat] line over the whole run")
    if uncovered > TRACK_LOST_MAX * T:
        return UNVERIFIED, ("任务窗口输出空窗 %.2f s（窗口 %.2f s，阈值 %.0f%%），"
                            "记录不足以判断任务期间健康，不把没记录到失败当作健康"
                            % (uncovered, T, 100 * TRACK_LOST_MAX)), diag
    ok = (final_state != 4 and frame_ratio <= TRACK_LOST_MAX
          and time_uncovered_ratio <= TRACK_LOST_MAX)
    return (PASS if ok else FAIL,
            ("C 任务段帧=%d 丢失帧比例=%.1f%%（<=%.0f%%）；时间未覆盖=%.1f%%"
             "（<=%.0f%%）；窗口末态=%d；A 初始化=%s 耗时=%s s；"
             "B 接管准备丢失帧比例=%s；整轮分布仅作诊断" %
             (len(C), 100 * frame_ratio, 100 * TRACK_LOST_MAX,
              100 * time_uncovered_ratio, 100 * TRACK_LOST_MAX, final_state,
              diag["init_completed"], diag["init_seconds"],
              diag["stage_B_pre_takeover"]["lost_frame_ratio"])), diag)


# ---------------------------------------------------------------- latency

def evaluate_latency_evidence(latency, raw_header):
    """Keep UNVERIFIED unless a real age_ms column exists (it does not)."""
    diag = {"evaluator": "phase4-v2", "header": raw_header,
            "clock_domains": {
                "sim_us": "Gazebo sim time (source sample)",
                "rx_wall_us": "host epoch (receive)",
                "rx_mono_us/track_start_us/track_end_us": "host steady/mono"},
            "cannot_use": [
                "rx_wall_us - sim_us: different clock domains",
                "track_end_us - track_start_us: processing duration, NOT sample age",
                "unknown values filled with 0",
                "replay wall clock as real-time age"]}
    procs = sorted(f["proc_ms"] for f in latency if f["proc_ms"] is not None
                   and f["proc_ms"] >= 0) if latency else []
    if procs:
        n = len(procs)
        diag["derived_processing_only_ms"] = {
            "n": n, "p50": round(procs[n // 2], 2),
            "p95": round(procs[min(n - 1, int(n * .95))], 2),
            "max": round(procs[-1], 2),
            "note": "TrackStereo duration only; explicitly NOT sample age"}
    if raw_header and "age_ms" in raw_header:
        return PASS, "latency.csv 含 age_ms 列（未在本阶段出现）", diag
    return UNVERIFIED, ("latency.csv 无 age_ms 列；现有字段分属 sim/epoch/mono 三个时钟域，"
                        "缺逐帧 sim->epoch 映射与发布时刻，不能可靠重算样本年龄；"
                        "仅派生处理耗时供诊断，不代替样本年龄"), diag


def raw_latency_header(path):
    p = Path(path)
    if not p.exists():
        return []
    with p.open(newline="") as f:
        return next(csv.reader(f), [])
