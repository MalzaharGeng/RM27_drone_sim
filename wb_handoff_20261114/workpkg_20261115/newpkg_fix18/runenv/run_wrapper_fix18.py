#!/usr/bin/env python3
"""Outer wrapper: reliable long-run management, LIVE node evidence, two-phase manifest.

The tool waiting time is NOT a simulation termination condition: run_profile.sh
is started in its own session and the wrapper never kills it. Even if the
wrapper is interrupted, initial_manifest.json and runtime_status.json already
exist on disk.

Usage (frozen configuration is supplied through the environment):
  python3 run_wrapper.py --run-dir DIR --tag TAG --distance 1.0 \
      --duration 240 --node-bin BIN
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
# [FIX18] 本文件放在 newpkg_fix18/runenv/，而 runtime_tools.py 仍在共享的
# audit/ 目录里（未复制、未修改）。显式加入搜索路径，避免 import 失败。
_AUDIT = Path(__file__).resolve().parents[2] / "audit"
if _AUDIT.is_dir():
    sys.path.insert(0, str(_AUDIT))
import runtime_tools as rt

ROOT = "/home/gengyinghao/drone_sim"
ORB = "/home/gengyinghao/ORB_SLAM3"
WP = "/home/gengyinghao/drone_sim/wb_handoff_20261114/workpkg_20261115"
EVID = WP + "/evidence"


def jwrite(path, obj):
    with open(path, "w") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2, sort_keys=True)
        f.write("\n")


def now():
    return time.strftime("%Y-%m-%d %H:%M:%S")


def safe_md5(p):
    try:
        return rt.md5_file(p)
    except OSError:
        return "MISSING"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--tag", required=True)
    ap.add_argument("--distance", required=True)
    ap.add_argument("--duration", type=int, required=True)
    # The frozen run_profile.sh cleanup runs `pkill -f <node name>`.
    # The wrapper cmdline must NOT contain that literal: pass the binary
    # path via the environment (environ is not matched by pkill -f).
    ap.add_argument("--node-bin",
                    default=(os.environ.get("WORKPKG_NODE_BIN")
                             or os.environ.get("ORB_NODE_BIN") or ""))
    a = ap.parse_args()
    if not a.node_bin:
        ap.error("node bin missing: set WORKPKG_NODE_BIN / ORB_NODE_BIN "
                "or pass --node-bin")

    rd = Path(a.run_dir)
    (rd / "bt").mkdir(parents=True, exist_ok=True)
    E = Path(EVID)
    E.mkdir(parents=True, exist_ok=True)
    node_bin = os.path.realpath(a.node_bin)
    node_md5 = rt.md5_file(node_bin)
    # [NP-01 FIX18] the runner and the lifecycle module of THIS revision, so the
    # manifest says which process-ownership rules produced the run.
    runner = os.environ.get("NP01_RUNNER") or (ROOT + "/scripts/run_profile_fix18.sh")
    lifecycle = os.environ.get("NP01_LIFECYCLE") or ""

    cfg = {k: os.environ.get(k) for k in rt.ENV_WHITELIST if k in os.environ}
    t0 = time.time()
    initial = {
        "manifest_phase": "INITIAL",
        "written_when": "BEFORE_RUN_START",
        "written_at": now(),
        "note": "Written before any simulation process started; survives an interrupted run.",
        "run_dir": str(rd), "tag": a.tag, "distance": a.distance,
        "duration": a.duration,
        "config_whitelisted": cfg,
        "node_bin": node_bin, "node_bin_md5": node_md5,
        "fingerprints": {
            "core_a": safe_md5(ORB + "/build/liborbslam3_px4_core.a"),
            "libORB_SLAM3_so": safe_md5(ORB + "/build/lib/libORB_SLAM3.so"),
            "launcher": safe_md5(ROOT + "/scripts/run_stereo_inertial_sim.sh"),
            "run_profile": safe_md5(ROOT + "/scripts/run_profile.sh"),
            "check_run": safe_md5(ROOT + "/scripts/check_run.py"),
            "px4_vio_config": safe_md5(ORB + "/config/px4_vio_sim_transport.yaml"),
            "runner_used": runner,
            "runner_used_md5": safe_md5(runner),
            "lifecycle_module": lifecycle,
            "lifecycle_module_md5": safe_md5(lifecycle) if lifecycle else "UNSET",
        },
    }
    jwrite(rd / "initial_manifest.json", initial)
    status = {"phase": "STARTING", "updated_at": now(), "run_pid": None,
              "node_pid": None, "node_evidence": "PENDING", "exit": "PENDING"}
    jwrite(rd / "runtime_status.json", status)

    env = os.environ.copy()
    env["SIM_RUN"] = str(rd)
    # [NP-01 FIX18] 唯一改动：运行器脚本可配置，其余（会话隔离、node 存活证据、
    # 两段式 manifest、最终 manifest）与 fix17 逐字节相同。
    logf = open(rd / "wrapper_run_profile.log", "w")
    p = subprocess.Popen(["bash", runner],
                         cwd=ROOT, env=env, stdout=logf,
                         stderr=subprocess.STDOUT, start_new_session=True)
    status.update({"phase": "RUNNING", "updated_at": now(), "run_pid": p.pid,
                   "run_start_wall_us": int(t0 * 1e6)})
    jwrite(rd / "runtime_status.json", status)

    min_start = rt.boot_jiffies()
    node_pid, cap = None, None
    deadline = time.time() + 180
    while time.time() < deadline:
        hits = rt.find_node_pid(p.pid, node_bin, min_start)
        if hits:
            node_pid = hits[0]
            cap = rt.capture_live(node_pid, node_md5)
            jwrite(rd / "node_load_evidence.json", cap)
            with open(rd / "node_load_evidence.txt", "w") as f:
                f.write("WORKPKG RUNTIME LIVE NODE EVIDENCE / tag=%s\n" % a.tag)
                for k in ("capture_kind", "capture_wall_us", "pid",
                          "alive_at_capture", "exe", "exe_realpath", "exe_md5",
                          "exe_md5_expected", "exe_md5_match",
                          "starttime_s_after_boot", "cmdline",
                          "loaded_so_count", "loaded_so_key",
                          "loaded_orbslam3_so", "loaded_orbslam3_so_md5",
                          "env_whitelisted",
                          "env_other_keys_not_dumped"):
                    f.write("%s = %s\n" % (k, json.dumps(cap.get(k), ensure_ascii=False)))
            break
        if p.poll() is not None:
            break
        time.sleep(1)
    status.update({"node_pid": node_pid,
                   "node_evidence": "CAPTURED_LIVE" if cap else "NOT_FOUND",
                   "updated_at": now()})
    jwrite(rd / "runtime_status.json", status)
    jwrite(E / ("node_load_%s.json" % a.tag), cap or {"state": "NOT_FOUND"})

    overall_deadline = t0 + a.duration + 240
    monitor, last_sample = [], 0.0
    while p.poll() is None and time.time() < overall_deadline:
        try:
            plog = (rd / "pilot.log").read_text(errors="replace")
            states = [l.strip().split("state ->")[-1].strip()
                      for l in plog.splitlines() if "state ->" in l]
            cur = states[-1] if states else "-"
        except OSError:
            cur = "-"
        if time.time() - last_sample >= 10:
            last_sample = time.time()
            monitor.append("%s pilot=%s run_alive=True node_alive=%s" % (
                now(), cur,
                os.path.exists("/proc/%d" % node_pid) if node_pid else False))
            with open(E / ("runtime_monitor_%s.txt" % a.tag), "w") as f:
                f.write("WORKPKG RUNTIME MONITOR / tag=%s\n" % a.tag)
                f.write("\n".join(monitor) + "\n")
        status.update({"phase": "RUNNING", "updated_at": now(),
                       "pilot_state": cur, "elapsed_s": round(time.time() - t0, 1)})
        jwrite(rd / "runtime_status.json", status)
        time.sleep(5)

    if p.poll() is None:
        rc = None
        status["phase"] = "TIMEOUT"
        status["note"] = ("run_profile.sh still alive at the finite wrapper deadline; "
                          "the wrapper did NOT terminate the simulation")
        jwrite(rd / "runtime_status.json", status)
    else:
        rc = p.returncode
    logf.close()

    try:
        plog = (rd / "pilot.log").read_text(errors="replace")
    except OSError:
        plog = ""
    flags = [s for s in ("LAND", "LANDED", "DONE")
             if any("state -> " + s in l for l in plog.splitlines())]
    disarm_ok = "disarm not confirmed" not in plog
    exit_ok = all(s in flags for s in ("LAND", "LANDED", "DONE")) and disarm_ok

    bpath = rd / "bridge_telemetry.csv"
    ba = rt.bridge_audit(bpath)
    recovery = {"performed": False}
    if ba["state"] == "MISSING":
        src = Path(ROOT) / "px4_telemetry_sim_transport.csv"
        if src.exists() and src.stat().st_mtime >= t0 - 5:
            shutil.copy2(src, bpath)
            recovery = {
                "performed": True, "kind": "POST_RUN_RECOVERY",
                "reason": "run_profile.sh tail interrupted; bridge log not archived",
                "source": str(src), "source_mtime": src.stat().st_mtime,
                "source_md5": rt.md5_file(src), "target_md5": rt.md5_file(bpath),
                "note": ("Post-run copy of an EXISTING runtime file; "
                         "NOT a runtime manifest and NOT new data."),
            }
            ba = rt.bridge_audit(bpath)
    jwrite(E / ("bridge_audit_%s.json" % a.tag),
           {"bridge_audit": ba, "postrun_recovery": recovery})

    check_path = rd / "check.txt"
    evaluator = {"performed_by": "run_profile.sh",
                 "check_txt_present": check_path.exists()}
    if not check_path.exists():
        r = subprocess.run(["python3", ROOT + "/scripts/check_run.py",
                            "--run", str(rd), "--profile", "vio_closed_loop",
                            "--distance", a.distance],
                           capture_output=True, text=True, timeout=180)
        with open(check_path, "w") as f:
            f.write(r.stdout)
        evaluator = {"performed_by": "wrapper POST_RUN_RECOVERY",
                     "note": ("Offline re-run of the SAME frozen evaluator "
                              "after archive + integrity check"),
                     "returncode": r.returncode}

    segs = rt.rtf_segments(rd / "truth.csv", 10.0)
    jwrite(E / ("rtf_segments_%s.json" % a.tag),
           {"window_s": 10.0, "segments": segs})
    timing = rt.timing_evidence(rd / "latency.csv", rd / "slam.log")
    jwrite(E / ("timing_evidence_%s.json" % a.tag), timing)

    cleanup_summary = {"state": "MISSING"}
    cp = rd / "process_cleanup.json"
    if cp.exists():
        try:
            with open(cp, encoding="utf-8") as fh:
                cc = json.load(fh)
            cleanup_summary = {
                "state": "PRESENT", "reason": cc.get("reason"),
                "verified_empty": cc.get("verified_empty"),
                "targets": len(cc.get("targets") or []),
                "leftovers": len(cc.get("leftovers") or []),
                "source": str(cp),
            }
        except (OSError, ValueError) as exc:
            cleanup_summary = {"state": "UNREADABLE", "error": repr(exc)}

    final = {
        "manifest_phase": "FINAL", "written_at": now(),
        "initial_manifest_preserved": (rd / "initial_manifest.json").exists(),
        "run_profile_returncode": rc,
        # [FIX18] 进程归属的收尾证据（由 np01_lifecycle.py 写出，不是推断）
        "process_cleanup": cleanup_summary,
        "exit_sequence": flags, "disarm_confirmed": disarm_ok, "exit_ok": exit_ok,
        "node_live_evidence": bool(cap),
        "node_md5_match": (cap or {}).get("exe_md5_match"),
        "bridge_state": ba.get("state"),
        "bridge_publish_rows": ba.get("publish_rows"),
        "postrun_recovery": recovery, "evaluator": evaluator,
        "files": sorted(x.name for x in rd.iterdir() if x.is_file()),
        "note": ("Runtime fields come from node_load_evidence.json / "
                 "runtime_status.json; this file is the end-of-run supplement, "
                 "not runtime evidence."),
    }
    jwrite(rd / "final_manifest.json", final)
    status.update({"phase": "FINISHED", "updated_at": now(),
                   "run_profile_returncode": rc, "exit_ok": exit_ok,
                   "bridge_state": ba.get("state"),
                   "bridge_publish_rows": ba.get("publish_rows"),
                   "node_md5_match": (cap or {}).get("exe_md5_match"),
                   "evaluator": evaluator.get("performed_by")})
    jwrite(rd / "runtime_status.json", status)
    print("WRAPPER_DONE tag=%s rc=%s exit_ok=%s bridge=%s pub=%s" % (
        a.tag, rc, exit_ok, ba.get("state"), ba.get("publish_rows")))


if __name__ == "__main__":
    main()
