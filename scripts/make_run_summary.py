#!/usr/bin/env python3
"""汇总本机所有仿真轮次的小型证据：manifest / check / 关键日志窗口。

只读，不修改任何轮次文件。输出 runs_summary.json 与 04_轮次结果.md。
"""
import argparse
import csv
import json
import re
from pathlib import Path

OUT = Path("/mnt/c/Users/46292/Desktop/1111/Nano飞行仿真复现")


def load_json(p):
    try:
        return json.loads(Path(p).read_text())
    except (OSError, ValueError):
        return {}


def brief(path, pattern, limit=6):
    p = Path(path)
    if not p.exists():
        return []
    out = []
    with p.open(errors="replace") as f:
        for line in f:
            if re.search(pattern, line):
                out.append(line.rstrip()[:220])
                if len(out) >= limit:
                    break
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs", default="/home/gengyinghao/drone_sim/logs")
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args()
    root = Path(args.logs)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    runs = []
    for d in sorted(root.iterdir()):
        if not d.is_dir() or not (d.name.startswith("control_truth_")
                                  or d.name.startswith("vio_closed_loop_")
                                  or d.name.startswith("fault_injection_")):
            continue
        man = load_json(d / "manifest.json")
        chk = load_json(d / "check.json")
        env = {}
        ef = d / "env.txt"
        if ef.exists():
            for line in ef.read_text(errors="replace").splitlines():
                if "=" in line and not line.startswith("cmd="):
                    k, v = line.split("=", 1)
                    env[k] = v
        checks = {c["name"]: c for c in chk.get("checks", [])}
        slam = d / "slam.log"
        gate_faults = brief(slam, r"GATE_FAULT|airborne_fault_latched|attitude_discontinuity|clock reject|sensor clock epoch|RECENTLY_LOST|reference_or_clock_epoch_changed", 8)
        viba = brief(slam, r"start VIBA|end VIBA", 6)
        revokes = brief(d / "bt.log", r"Task revoked|OFFBOARD READY|TASK COMPLETED", 6)
        first_fault = {}
        tp = d / "telemetry.csv"
        if tp.exists():
            with tp.open() as f:
                for r in csv.DictReader(f):
                    if r.get("vio_fault") in ("True", "1", "true"):
                        try:
                            v = json.loads(r.get("vio_raw") or "{}")
                        except ValueError:
                            v = {}
                        first_fault = {"wall_us": r.get("wall_us"),
                                       "armed": r.get("st_arming"),
                                       "nav_state": r.get("st_nav"),
                                       "ev_pos": r.get("es_ev_pos"),
                                       "ev_hgt": r.get("es_ev_hgt"),
                                       "ev_vel": r.get("es_ev_vel"),
                                       "reason": r.get("vio_reason"),
                                       "health": v}
                        break
        lat = {}
        lp = d / "latency.csv"
        if lp.exists():
            with lp.open() as f:
                vals = []
                for r in csv.DictReader(f):
                    try:
                        vals.append(float(r.get("age_ms")))
                    except (TypeError, ValueError):
                        pass
            if vals:
                vals.sort()
                lat = {"n": len(vals), "p50": round(vals[len(vals) // 2], 1),
                       "p95": round(vals[min(len(vals) - 1, int(len(vals) * .95))], 1),
                       "max": round(vals[-1], 1)}
        runs.append({
            "run_id": d.name,
            "profile": man.get("profile") or env.get("PROFILE"),
            "distance_m": man.get("distance_m") or env.get("DISTANCE"),
            "vio_config": man.get("vio_config") or env.get("PX4_VIO_CONFIG"),
            "orb_profile": env.get("ORB_PROFILE"),
            "settings_md5": (man.get("files", {}).get("orb_settings_run") or {}).get("md5"),
            "verdict": chk.get("verdict", "NO_CHECK"),
            "counts": chk.get("counts", {}),
            "mission_window_epoch": chk.get("mission_window_epoch"),
            "px4_bin_md5": (man.get("files", {}).get("px4_bin") or {}).get("md5"),
            "orb_node_md5": (man.get("files", {}).get("orb_node") or {}).get("md5"),
            "wall_duration_s": man.get("wall_duration_s"),
            "checks": {k: {"status": v["status"], "detail": v["detail"]}
                       for k, v in checks.items()},
            "gate_fault_window": gate_faults,
            "first_health_fault": first_fault,
            "viba": viba,
            "bt_events": revokes,
            "latency": lat,
            "files_present": sorted(f.name for f in d.iterdir() if f.is_file()),
        })
    (out / "runs_summary.json").write_text(json.dumps(runs, ensure_ascii=False, indent=2))
    lines = ["# 每轮结果、统计与关键证据", "",
             "由 `make_run_summary.py` 从各轮目录只读生成；完整原始证据在 WSL "
             "`/home/gengyinghao/drone_sim/logs/<run_id>/`。", "",
             "| 轮次 | profile | 距离 | verdict | PASS/FAIL/UNVERIFIED | 关键变化 |",
             "|---|---|---:|---|---|---|"]
    for r in runs:
        c = r["counts"]
        if r["profile"] == "vio_closed_loop":
            key = "; ".join(x[:70] for x in r["viba"][:2]) or "无 VIBA 记录"
        else:
            oc = r["checks"].get("offboard_accepted", {}).get("status", "-")
            td = r["checks"].get("truth_distance_hold", {}).get("detail", "")[:60]
            key = "offboard=%s %s" % (oc, td)
        lines.append("| %s | %s | %s | %s | %s/%s/%s | %s |" % (
            r["run_id"], r["profile"], r["distance_m"], r["verdict"],
            c.get("PASS", 0), c.get("FAIL", 0), c.get("UNVERIFIED", 0), key))
    lines += ["", "## 各轮逐项判定", ""]
    for r in runs:
        lines += ["### %s（verdict=%s）" % (r["run_id"], r["verdict"]), "",
                  "- 配置：profile=%s distance=%s vio_config=%s orb_profile=%s settings_md5=%s" % (
                      r["profile"], r["distance_m"], r["vio_config"],
                      r["orb_profile"], r["settings_md5"]),
                  "- 版本：px4_bin=%s orb_node=%s" % (r["px4_bin_md5"], r["orb_node_md5"]),
                  "- 任务窗口：%s，主机耗时 %s s" % (
                      r["mission_window_epoch"], r["wall_duration_s"]), ""]
        for k, v in r["checks"].items():
            lines.append("- `[%s]` **%s**：%s" % (v["status"], k, v["detail"]))
        if r["viba"]:
            lines += ["", "VIBA 记录："] + ["  - " + x for x in r["viba"]]
        if r["gate_fault_window"]:
            lines += ["", "首次故障窗口："] + ["  - " + x for x in r["gate_fault_window"]]
        if r.get("first_health_fault"):
            lines += ["", "health 首次故障：```json", json.dumps(r["first_health_fault"], ensure_ascii=False, indent=1), "```"]
        if r["bt_events"]:
            lines += ["", "BT 事件："] + ["  - " + x for x in r["bt_events"]]
        if r["latency"]:
            lines += ["", "逐帧延迟 age_ms：%s" % r["latency"]]
        lines.append("")
    (out / "04_轮次结果.md").write_text("\n".join(lines))
    print("wrote runs_summary.json and 04_轮次结果.md; runs =", len(runs))


if __name__ == "__main__":
    main()
