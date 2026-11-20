#!/usr/bin/env python3
"""[NP-01 FIX12] Independent TRUTH safety watchdog (SIM-ONLY, opt-in).

HARD CONTRACT
-------------
* This process READS truth.csv only. It never publishes to any ROS/MAVLink
  topic, never writes to pilot.csv, never passes a number to sim_pilot.py,
  and never touches Gate / LIMITS / age / fusion / acceptance thresholds.
* Its only effect on the run is OUT-OF-ENVELOPE TERMINATION: it writes
  TRUTH_BOUND_EXCEEDED into the run directory; run_profile.sh (opt-in) sees
  that marker, breaks its wait loop and runs the NORMAL cleanup + manifest +
  check_run.py. No error is hidden and no verdict is rewritten.
* Truth is therefore an INDEPENDENT MONITOR, never a control input. The
  excitation controller in sim_pilot.py uses only normal flight feedback.

Bounds here are SAFETY bounds for terminating a runaway simulation, not
acceptance thresholds and not control limits. They are deliberately far
outside the nominal 1 m / 5 m task envelope.
"""
import argparse
import csv
import json
import math
import os
import signal
import sys
import time


_watchdog_stop = False


def _on_term(signum, frame):
    """Runner kills background children during normal cleanup. Exit through
    the normal path so truth_watchdog.json is ALWAYS written, including the
    clean (not triggered) case."""
    global _watchdog_stop
    _watchdog_stop = True


def quat_angle_deg(q0, q1):
    d = abs(sum(q0[i] * q1[i] for i in range(4)))
    return 2.0 * math.degrees(math.acos(min(1.0, max(-1.0, d))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--truth", required=True)
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--max-xy", type=float, default=12.0,
                    help="task-phase horizontal displacement from the first truth row (m)")
    ap.add_argument("--max-z", type=float, default=4.0,
                    help="absolute z deviation from the first truth row (m)")
    ap.add_argument("--max-rate-dps", type=float, default=600.0,
                    help="sustained truth angular rate (deg/s)")
    ap.add_argument("--rate-samples", type=int, default=3,
                    help="consecutive samples above max-rate-dps to trigger")
    # [NP01-FIX15] 分阶段与净空判定。旧版只有「相对首个真值行的位移」一个门限，
    # 把初始化阶段的活动范围、任务航段、以及到墙的真实净空混成一条线：
    # 既可能在初始化时误报，也可能在真正接近墙面时来不及终止。
    # 这里分开：
    #   init  : 前 --init-seconds 秒，按初始化活动范围判（比任务航段更紧）
    #   task  : 之后按 --max-xy 判
    #   wall  : 任何阶段都按「到四面墙内表面的水平距离」判，用真实旋翼包络
    #           --envelope-m 做净空，warn 先预警、stop 直接终止。
    ap.add_argument("--max-xy-init", type=float, default=2.0,
                    help="init-phase horizontal displacement bound (m)")
    ap.add_argument("--init-seconds", type=float, default=90.0,
                    help="duration treated as the init phase for --max-xy-init (s)")
    ap.add_argument("--envelope-m", type=float, default=0.385971,
                    help="horizontal rotor envelope from base_link origin (m)")
    ap.add_argument("--walls", default="west=0.349,east=27.949,south=0.264,north=15.867",
                    help="arena wall inner surfaces: west=,east=,south=,north=")
    ap.add_argument("--warn-clearance", type=float, default=2.5,
                    help="warn when wall-to-origin distance falls below this (m)")
    ap.add_argument("--stop-clearance", type=float, default=1.0,
                    help="terminate when wall-to-origin distance falls below this (m)")
    ap.add_argument("--poll-s", type=float, default=0.5)
    ap.add_argument("--max-wait-s", type=float, default=600.0)
    a = ap.parse_args()
    walls = {}
    for item in a.walls.split(","):
        if "=" in item:
            k, v = item.split("=", 1)
            walls[k.strip()] = float(v)
    for need in ("west", "east", "south", "north"):
        if need not in walls:
            raise SystemExit("--walls missing %s" % need)
    signal.signal(signal.SIGTERM, _on_term)
    signal.signal(signal.SIGINT, _on_term)

    marker = os.path.join(a.run_dir, "TRUTH_BOUND_EXCEEDED")
    warn_marker = os.path.join(a.run_dir, "TRUTH_WARN")
    report = os.path.join(a.run_dir, "truth_watchdog.json")
    # Fail-closed: never leave a stale marker from a previous run.
    for stale in (marker, warn_marker):
        try:
            os.unlink(stale)
        except OSError:
            pass

    datum = None
    prev = None
    header = None
    rate_hits = 0
    rows_seen = 0
    triggered = None
    warn = None
    t_start = time.monotonic()
    pos = 0
    last_activity = time.monotonic()

    while (not _watchdog_stop
           and time.monotonic() - t_start < a.max_wait_s):
        # A clean run ends with pilot DONE; finalise the report immediately
        # instead of waiting for the runner's SIGTERM or a 30 s idle timeout.
        plog = os.path.join(a.run_dir, "pilot.log")
        if rows_seen > 0 and os.path.exists(plog):
            try:
                with open(plog, encoding="utf-8", errors="replace") as pf:
                    if "state -> DONE" in pf.read():
                        break
            except OSError:
                pass
        if not os.path.exists(a.truth):
            time.sleep(a.poll_s)
            continue
        # Binary mode: seek/tell are exact BYTE offsets, so a partially
        # written trailing CSV line can be safely carried to the next poll.
        with open(a.truth, "rb") as fh:
            fh.seek(pos)
            chunk = fh.read()
            pos = fh.tell()
        if not chunk:
            # No new data. If the run already ended (DONE) we can finalise.
            if os.path.exists(os.path.join(a.run_dir, "pilot.log")):
                with open(os.path.join(a.run_dir, "pilot.log"),
                          encoding="utf-8", errors="replace") as pf:
                    if "state -> DONE" in pf.read():
                        break
            if time.monotonic() - last_activity > 30.0:
                break
            time.sleep(a.poll_s)
            continue
        last_activity = time.monotonic()
        # A partially written trailing line is kept for the next poll.
        lines = chunk.splitlines(True)
        if lines and not lines[-1].endswith(b"\n"):
            pos -= len(lines[-1])
            lines = lines[:-1]
        text = b"".join(lines).decode("utf-8", errors="replace")
        if not text.strip():
            time.sleep(a.poll_s)
            continue
        # The header is present only in the FIRST chunk. Keep it explicitly;
        # a csv.DictReader on a later chunk would treat a data row as header.
        parsed = list(csv.reader(text.splitlines()))
        if header is None:
            if not parsed:
                time.sleep(a.poll_s)
                continue
            header = parsed[0]
            data = parsed[1:]
        else:
            data = parsed
        for row in data:
            if len(row) != len(header):
                continue
            r = dict(zip(header, row))
            try:
                w = float(r["recv_wall_us"])
                x, y, z = float(r["x"]), float(r["y"]), float(r["z"])
                q = [float(r[k]) for k in ("qx", "qy", "qz", "qw")]
            except (KeyError, TypeError, ValueError):
                continue
            rows_seen += 1
            if datum is None:
                datum = (w, x, y, z)
            disp = math.hypot(x - datum[1], y - datum[2])
            dz = abs(z - datum[3])
            rate = 0.0
            if prev is not None:
                dt = (w - prev[0]) / 1e6
                if 0.0 < dt < 1.0:
                    rate = quat_angle_deg(prev[1], q) / dt
            prev = (w, q)
            if rate > a.max_rate_dps:
                rate_hits += 1
            else:
                rate_hits = 0

            why = None
            tier = None
            elapsed_s = (w - datum[0]) / 1e6
            # 到四面墙内表面的最小水平距离（真值原点到墙），再减旋翼包络得到净空。
            clearance = min(
                x - walls["west"], walls["east"] - x,
                y - walls["south"], walls["north"] - y) - a.envelope_m
            xy_bound = a.max_xy_init if elapsed_s < a.init_seconds else a.max_xy
            # 预警独立评估，先于任何终止条件：净空一旦低于 warn 就立刻给出预警，
            # 不被其它终止条件抢先吞掉。预警只让飞手取消任务并请求 LAND。
            if clearance < a.warn_clearance and warn is None:
                warn = dict(
                    reason="truth_wall_clearance_warn", wall_us=w,
                    elapsed_from_first_truth_s=elapsed_s,
                    disp_xy_m=round(disp, 4), clearance_m=round(clearance, 4),
                    thresholds=dict(warn_clearance_m=a.warn_clearance,
                                    stop_clearance_m=a.stop_clearance,
                                    envelope_m=a.envelope_m),
                    note="independent monitor; no truth value reaches control")
                with open(warn_marker, "w", encoding="utf-8") as wf:
                    wf.write("truth_wall_clearance_warn\n")
            if clearance < a.stop_clearance:
                why, tier = "truth_wall_clearance_stop", "stop"
            elif dz > a.max_z:
                why, tier = "truth_z_exceeded", "stop"
            elif rate_hits >= a.rate_samples:
                why, tier = "truth_angular_rate_exceeded", "stop"
            elif disp > xy_bound:
                why, tier = "truth_xy_exceeded", "stop"
            if why and tier == "stop" and triggered is None:
                triggered = dict(
                    reason=why, wall_us=w, sim_us=r.get("sim_us"),
                    elapsed_from_first_truth_s=elapsed_s,
                    disp_xy_m=round(disp, 4), dz_m=round(dz, 4),
                    clearance_m=round(clearance, 4),
                    xy_bound_m=xy_bound,
                    angular_rate_deg_s=round(rate, 2),
                    consecutive_rate_hits=rate_hits,
                    thresholds=dict(max_xy_m=a.max_xy, max_xy_init_m=a.max_xy_init,
                                    init_seconds=a.init_seconds, max_z_m=a.max_z,
                                    max_rate_dps=a.max_rate_dps,
                                    rate_samples=a.rate_samples,
                                    warn_clearance_m=a.warn_clearance,
                                    stop_clearance_m=a.stop_clearance,
                                    envelope_m=a.envelope_m),
                    note=("independent monitor only; no truth value was given "
                          "to the pilot or the Gate"),
                )
                # Marker is the ONLY action taken on the run.
                with open(marker, "w", encoding="utf-8") as mf:
                    mf.write(why + "\n")
                break
        if triggered:
            break
        time.sleep(a.poll_s)

    result = dict(
        triggered=bool(triggered),
        warned=bool(warn),
        rows_seen=rows_seen,
        datum=dict(wall_us=datum[0], x=datum[1], y=datum[2], z=datum[3]) if datum else None,
        event=triggered,
        warn_event=warn,
        scope=("independent truth monitoring + two-tier warning/termination; "
               "init-phase and task-phase bounds are separate, wall clearance "
               "uses the real rotor envelope"),
        not_a_control_input=True,
        pilot_csv_modified=False,
    )
    with open(report, "w", encoding="utf-8") as fh:
        json.dump(result, fh, indent=2, ensure_ascii=False)
    print("TRUTH_WATCHDOG triggered=%s warned=%s rows=%d reason=%s" %
          (result["triggered"], result["warned"], rows_seen,
           (triggered or {}).get("reason", "none")), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
