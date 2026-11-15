#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""NP-01 FIX20 中断回归 离线审计（纯读取，不启动任何仿真/飞行，不发送任何信号）。
输出 analysis_raw.json，供 interruption_audit.json / 报告引用。"""
import csv, glob, json, os, hashlib, re, sys

R = "/home/gengyinghao/drone_sim/logs/vio_closed_loop_intr_fix20_resume_20261116_r1_20261116_000320"
D = "/mnt/c/Users/46292/Desktop/飞机/intr_resume_20261116"
W = "/home/gengyinghao/drone_sim/wb_handoff_20261114/workpkg_20261115"
OUT = os.path.join(D, "audit_offline_20261116", "analysis_raw.json")

def jread(p):
    with open(p, encoding="utf-8", errors="replace") as f:
        return json.load(f)

def rows(p):
    with open(p, newline="", encoding="utf-8", errors="replace") as f:
        return list(csv.DictReader(f))

def fnum(x, d=None):
    try:
        return float(x)
    except (TypeError, ValueError):
        return d

res = {"run_dir": R, "generated_by": "offline audit (read-only)", "notes": []}

# ---------- 0. 冻结判据与来源 ----------
crit = jread(os.path.join(W, "newpkg_fix19/FROZEN_INTERRUPTION_CRITERIA.json"))
res["criteria_md5"] = hashlib.md5(open(os.path.join(W, "newpkg_fix19/FROZEN_INTERRUPTION_CRITERIA.json"), "rb").read()).hexdigest()
res["criteria_ids"] = [c["id"] for c in crit["criteria"]]
res["verdict_rule"] = crit["verdict_rule"]

# ---------- 1. 时钟域映射 ----------
sig = jread(R + "/injection_signal.json")
rdy = jread(R + "/injection_readiness.json")
idt = jread(R + "/injection_target_identity.json")
irlog = {}
for line in open(R + "/interruption.log", encoding="utf-8", errors="replace"):
    if "=" in line:
        k, v = line.rstrip("\n").split("=", 1)
        irlog[k] = v

# monotonic(us) -> epoch(us) 偏移，从同一记录/同源 CSV 估计
tel = rows(R + "/telemetry.csv")
offsets = []
for r in tel[:200]:
    w, m = fnum(r.get("wall_us")), fnum(r.get("mono_us"))
    if w and m:
        offsets.append(w - m)
offset_med = sorted(offsets)[len(offsets)//2] if offsets else None
res["clock"] = {
    "injection_signal_requested_wall_us": sig["requested_wall_us"],
    "injection_signal_requested_mono_us": sig["requested_mono_us"],
    "injection_signal_sent_mono_us": sig["sent_mono_us"],
    "signal_delta_requested_to_sent_us": sig["sent_mono_us"] - sig["requested_mono_us"],
    "offset_epoch_minus_mono_us_from_signal_record": sig["requested_wall_us"] - sig["requested_mono_us"],
    "offset_epoch_minus_mono_us_median_telemetry": offset_med,
    "interruption_log_inject_mono": irlog.get("inject_mono"),
    "interruption_log_inject_epoch": irlog.get("inject_epoch"),
    "interruption_log_mono_minus_epoch_s": (fnum(irlog.get("inject_mono")) - fnum(irlog.get("inject_epoch"))),
    "supervisor_trace_t_wall_us_domain": "CLOCK_MONOTONIC us (time.monotonic_ns()//1000), NOT epoch; verified in np01_truth_supervisor.py:57,133,202",
    "proc_watch_t_wall_us_domain": "epoch us (from first row 1794758624056030)",
    "vio_obs_counter_t_wall_us_domain": "epoch us (time.time()*1e6); t_mono_s is relative monotonic",
}
SIG_WALL_US = sig["requested_wall_us"]
SIG_MONO_US = sig["sent_mono_us"]
# 用信号记录自身的偏移把 mono 换算到 epoch（同一进程同一时刻）
SIG_WALL_FROM_MONO = SIG_MONO_US + (sig["requested_wall_us"] - sig["requested_mono_us"])

# ---------- 2. 关键时间点（epoch us / s） ----------
pilot_log = open(R + "/pilot.log", encoding="utf-8", errors="replace").read().splitlines()
nav_lines = [l for l in pilot_log if "state -> NAVIGATE" in l]
def log_ts(l):
    m = re.search(r"\[([\d.]+)\]", l)
    return float(m.group(1)) if m else None
NAV_EPOCH_S = log_ts(nav_lines[0]) if nav_lines else None
state_trans = []
for l in pilot_log:
    m = re.search(r"\[([\d.]+)\].*state -> (\S+)", l)
    if m:
        state_trans.append((float(m.group(1)), m.group(2)))
bt_log = open(R + "/bt.log", encoding="utf-8", errors="replace").read().splitlines()
events = jread(R + "/pilot.csv.events.json")

res["timeline"] = {
    "navigate_epoch_s": NAV_EPOCH_S,
    "navigate_line": nav_lines[0] if nav_lines else None,
    "state_transitions": state_trans,
    "bt_events": [{"event": e["event"], "reason": e.get("reason"),
                   "wall_us": e["wall_us"], "wall_s": e["wall_us"]/1e6,
                   "rel_to_navigate_s": e["wall_us"]/1e6 - NAV_EPOCH_S} for e in events],
    "bt_log_revoked": [l for l in bt_log if "revoked" in l.lower()],
    "readiness_checked_wall_us": rdy["checked_wall_us"],
    "identity_recorded_wall_us": idt["recorded_wall_us"],
    "signal_requested_wall_us": sig["requested_wall_us"],
    "signal_sent_wall_us_derived": SIG_WALL_FROM_MONO,
    "signal_sent_mono_us": sig["sent_mono_us"],
    "cleanup_started_wall_us": jread(R + "/process_cleanup.json")["started_wall_us"],
    "cleanup_finished_wall_us": jread(R + "/process_cleanup.json")["finished_wall_us"],
}
t = res["timeline"]
t["biases_s"] = {
    "signal_requested_minus_NAVIGATE": sig["requested_wall_us"]/1e6 - NAV_EPOCH_S,
    "signal_requested_minus_DROP_AFTER_6s": sig["requested_wall_us"]/1e6 - NAV_EPOCH_S - 6.0,
    "readiness_minus_NAVIGATE": rdy["checked_wall_us"]/1e6 - NAV_EPOCH_S,
    "identity_minus_NAVIGATE": idt["recorded_wall_us"]/1e6 - NAV_EPOCH_S,
    "interruption_log_inject_epoch_minus_NAVIGATE": fnum(irlog.get("inject_epoch")) - NAV_EPOCH_S,
    "signal_minus_interruption_log_inject_epoch": sig["requested_wall_us"]/1e6 - fnum(irlog.get("inject_epoch")),
    "revoked_minus_signal_s": ((next((e["wall_us"] / 1e6 for e in events
        if e["event"] == "task_revoked"), None) or 0) - sig["requested_wall_us"] / 1e6),
}

# ---------- 3. C1：+6 s 时是否仍自主执行 ----------
pilot = rows(R + "/pilot.csv")
SIG_S = sig["requested_wall_us"]/1e6
def near(rows_, ts, key="wall_us", scale=1e6):
    best, bd = None, 1e18
    for r in rows_:
        v = fnum(r.get(key))
        if v is None: continue
        v = v/scale
        if abs(v-ts) < bd:
            bd, best = abs(v-ts), r
    return best, bd
p_at6, d6 = near(pilot, NAV_EPOCH_S + 6.0)
p_atsig, ds = near(pilot, SIG_S)
res["C1"] = {
    "navigate_epoch_s": NAV_EPOCH_S,
    "plus6_epoch_s": NAV_EPOCH_S + 6.0,
    "signal_epoch_s": SIG_S,
    "pilot_row_at_plus6": {k: p_at6.get(k) for k in ("wall_us","state","bt_phase","bt_reason",
        "task_completed","control_active","vio_ready","armed","ex_stop_reason")} if p_at6 else None,
    "pilot_row_at_plus6_delta_s": d6,
    "pilot_row_at_signal": {k: p_atsig.get(k) for k in ("wall_us","state","bt_phase","bt_reason",
        "task_completed","control_active","vio_ready","armed","ex_stop_reason")} if p_atsig else None,
    "pilot_row_at_signal_delta_s": ds,
    "first_task_completed_event": next((e for e in events if e["event"]=="task_completed"), None),
    "first_task_revoked_event": next((e for e in events if e["event"]=="task_revoked"), None),
    "task_revoked_reason": next((e.get("reason") for e in events if e["event"]=="task_revoked"), None),
    "bt_phase_counts": {},
    "FAULT_in_pilot_states": [s for _, s in state_trans if "FAULT" in s],
    "navigate_window_row_count": sum(1 for r in pilot if r.get("state")=="NAVIGATE"),
}
from collections import Counter
res["C1"]["bt_phase_counts"] = dict(Counter(r.get("bt_phase") for r in pilot))
res["C1"]["state_counts"] = dict(Counter(r.get("state") for r in pilot))

# ---------- 4. C2 / C3 / C4 ----------
reg = [json.loads(l) for l in open(R + "/proc_registry.jsonl", encoding="utf-8", errors="replace") if l.strip()]
npid = jread(R + "/node_pid.json")
res["C2"] = {
    "node_pid_json": npid,
    "injection_readiness_ready": rdy["ready"],
    "candidate_pids": rdy["candidate_pids"],
    "unique_vio_node": rdy["unique_vio_node"],
    "target_identity": rdy["target_identity"],
    "frozen_node_sha256": rdy["frozen_node_sha256"],
    "problems": rdy["problems"],
    "excluded_count": rdy["excluded_count"],
    "excluded_roles_launcher_runner_bridge": [x for x in rdy["excluded_non_node_processes"]
        if x.get("role") in ("launcher","runner","bridge")],
    "registry_node_entries": rdy["registry_node_entries"],
    "registry_bridge_entries": [{"pid": x["pid"], "starttime": x.get("starttime_s_boot"),
        "exe": x.get("exe_realpath")} for x in reg if x.get("role")=="bridge"],
    "identity_exe_hash": idt["exe_file_hash"],
    "identity_expect_hash": idt["expect_exe_hash"],
    "identity_hash_match": idt["hash_match"],
    "identity_is_only_vio_publisher": idt["is_only_vio_publisher"],
    "identity_exe_note": idt["exe"]["note"],
}
res["C3"] = {
    "injection_signal": sig,
    "interruption_log": irlog,
    "INJECTION_MARKER_exists": os.path.exists(R + "/INJECTION_MARKER"),
    "INJECTION_MARKER_size": os.path.getsize(R + "/INJECTION_MARKER"),
    "signal_stdout": open(R + "/injection_signal.stdout", encoding="utf-8", errors="replace").read(),
    "injector_console_launcher_rc": [l for l in open(D + "/injector_console.log", encoding="utf-8", errors="replace") if "LAUNCHER_RC" in l or "READINESS_RC" in l],
}
le = jread(R + "/launcher_exit.json")
pw = rows(R + "/proc_watch.csv")
node_rows = [r for r in pw if r.get("pid")=="337397"]
res["C4"] = {
    "launcher_exit": le,
    "proc_watch_node_rows_first": node_rows[:2],
    "proc_watch_node_rows_last": node_rows[-4:],
    "proc_watch_node_alive_last": node_rows[-1].get("alive") if node_rows else None,
    "proc_watch_node_last_wall_us": node_rows[-1].get("t_wall_us") if node_rows else None,
}

# ---------- 5. C5：消息通路停止 ----------
obs = rows(R + "/vio_obs_counter.csv")
bt_csv = rows(R + "/bridge_telemetry.csv")
pub = [r for r in bt_csv if r.get("event")=="publish"]
pub_sent = [(fnum(r.get("system_us")), fnum(r.get("sent")), r) for r in pub]
pub_sent = [(a,b,r) for a,b,r in pub_sent if a is not None and b is not None]
last_odo = max((fnum(r.get("odo_last_rx_wall_us"),0) or 0) for r in obs)
last_health = max((fnum(r.get("health_last_rx_wall_us"),0) or 0) for r in obs)
odo_final = fnum(obs[-1].get("odo_count")); health_final = fnum(obs[-1].get("health_count"))
# 注入时刻之后 obs 行中 odo/health 计数是否增长
obs_after = [r for r in obs if (fnum(r.get("t_wall_us"),0) or 0) > SIG_WALL_US]
odo_counts_after = [fnum(r.get("odo_count")) for r in obs_after]
health_counts_after = [fnum(r.get("health_count")) for r in obs_after]
pub_after_sig = [p for p in pub_sent if p[0] > SIG_WALL_US]
pub_before_sig = [p for p in pub_sent if p[0] <= SIG_WALL_US]
# 452 / 460 号 publish 的时间
pub452 = next((p for p in pub_sent if int(p[1])==452), None)
pub460 = next((p for p in pub_sent if int(p[1])==460), None)
res["C5"] = {
    "obs_rows_total": len(obs),
    "obs_last_row": {k: obs[-1].get(k) for k in ("t_mono_s","t_wall_us","health_count",
        "health_last_rx_wall_us","odo_count","odo_last_rx_wall_us","odo_last_stamp_us")},
    "odo_final_count": odo_final, "health_final_count": health_final,
    "odo_last_rx_wall_us_max": last_odo, "health_last_rx_wall_us_max": last_health,
    "odo_last_rx_relative_to_signal_s": (last_odo - SIG_WALL_US)/1e6 if last_odo else None,
    "health_last_rx_relative_to_signal_s": (last_health - SIG_WALL_US)/1e6 if last_health else None,
    "obs_rows_after_signal": len(obs_after),
    "odo_count_growth_after_signal": (max(odo_counts_after)-odo_final) if odo_counts_after else 0,
    "health_count_growth_after_signal": (max(health_counts_after)-health_final) if health_counts_after else 0,
    "publish_rows_total": len(pub_sent),
    "publish_rows_at_or_before_signal": len(pub_before_sig),
    "publish_rows_after_signal": len(pub_after_sig),
    "publish_last_system_us": pub_sent[-1][0] if pub_sent else None,
    "publish_last_relative_to_signal_s": (pub_sent[-1][0]-SIG_WALL_US)/1e6 if pub_sent else None,
    "publish_seq_452_system_us": pub452[0] if pub452 else None,
    "publish_seq_452_relative_to_signal_s": (pub452[0]-SIG_WALL_US)/1e6 if pub452 else None,
    "publish_seq_460_system_us": pub460[0] if pub460 else None,
    "publish_seq_460_relative_to_signal_s": (pub460[0]-SIG_WALL_US)/1e6 if pub460 else None,
    "interruption_log_publish_rows_before": irlog.get("publish_rows_before"),
    "interruption_log_publish_rows_after": irlog.get("publish_rows_after"),
    "runtime_status_bridge_publish_rows": jread(R + "/runtime_status.json")["bridge_publish_rows"],
    "after_snapshot_timing_note": "PUB_AFTER is sampled AFTER the up-to-5s node-exit wait loop, not at the instant of the signal; the 452->460 delta spans the pre-signal in-flight window plus the exit wait.",
    "publish_after_signal_rows": [{"system_us": p[0], "sent": p[1],
        "rel_s": round((p[0]-SIG_WALL_US)/1e6, 6)} for p in pub_after_sig[:20]],
    "publish_tail_12": [{"system_us": p[0], "sent": p[1],
        "rel_s": round((p[0]-SIG_WALL_US)/1e6, 6)} for p in pub_sent[-12:]],
    "obs_tail_6": [{k: r.get(k) for k in ("t_wall_us","health_count","health_last_rx_wall_us",
        "odo_count","odo_last_rx_wall_us")} for r in obs[-6:]],
}

# ---------- 6. C6 ----------
res["C6"] = {
    "task_revoked_event": next((e for e in events if e["event"]=="task_revoked"), None),
    "bt_log_revoked_lines": [l for l in bt_log if "revoked" in l.lower()],
    "pilot_log_revoked_lines": [l for l in pilot_log if "revoked" in l.lower() or "stale_health" in l.lower()],
    "state_transitions_after_signal": [x for x in state_trans if x[0] > SIG_S],
    "control_active_after_signal": dict(Counter(r.get("control_active") for r in pilot
        if (fnum(r.get("wall_us"),0) or 0) > SIG_WALL_US)),
    "navigate_rows_after_signal": sum(1 for r in pilot if r.get("state")=="NAVIGATE"
        and (fnum(r.get("wall_us"),0) or 0) > SIG_WALL_US),
}

# ---------- 7. C7：桥 / 监督器 / 真值 / 遥测 ----------
bo = jread(R + "/bridge_owner.fix19.json")
sp = jread(R + "/supervisor_pid.fix19.json")
sj = jread(R + "/supervisor.json")
st = rows(R + "/supervisor_trace.csv")
truth = rows(R + "/truth.csv")
cleanup = jread(R + "/process_cleanup.json")
BRIDGE_PIDS = [str(x["pid"]) for x in reg if x.get("role")=="bridge"]
SUP_PID = str(sp["pid"])
def alive_rows(pid):
    rr = [r for r in pw if r.get("pid")==pid]
    return {"count": len(rr), "first": rr[0] if rr else None, "last": rr[-1] if rr else None,
            "alive_after_signal": [r for r in rr if (fnum(r.get("t_wall_us"),0) or 0) > SIG_WALL_US and r.get("alive")=="1"][:3],
            "last_alive_wall_us": max((fnum(r.get("t_wall_us"),0) or 0) for r in rr if r.get("alive")=="1") or None}
res["C7"] = {
    "bridge_owner_record": {k: bo.get(k) for k in ("pid","starttime_s_boot","child_parameter_bridge_pids",
        "loaded_ros_gz_bridge_so","owner","state","recorded_wall_us")},
    "bridge_pid_from_registry": BRIDGE_PIDS,
    "bridge_proc_watch": {p: alive_rows(p) for p in BRIDGE_PIDS},
    "supervisor_pid": SUP_PID,
    "supervisor_proc_watch": alive_rows(SUP_PID),
    "supervisor_json": {k: sj.get(k) for k in ("rows_seen","lost_us_total","max_rx_gap_ms",
        "warned","warn_event","triggered","stop_event","stop_reason","hard_kill")},
    "supervisor_trace_rows": len(st),
    "supervisor_trace_first_t_wall_us": st[0].get("t_wall_us") if st else None,
    "supervisor_trace_last_t_wall_us": st[-1].get("t_wall_us") if st else None,
    "supervisor_trace_last_t_mono_s": st[-1].get("t_mono_s") if st else None,
    "supervisor_trace_actions": dict(Counter(r.get("action") for r in st)),
    "supervisor_trace_lost_us_max": max((fnum(r.get("lost_us_total"),0) or 0) for r in st),
    "truth_rows": len(truth),
    "truth_first_recv_wall_us": truth[0].get("recv_wall_us") if truth else None,
    "truth_last_recv_wall_us": truth[-1].get("recv_wall_us") if truth else None,
    "truth_last_relative_to_signal_s": (fnum(truth[-1].get("recv_wall_us"))-SIG_WALL_US)/1e6 if truth else None,
    "telemetry_rows": len(tel),
    "telemetry_first_wall_us": tel[0].get("wall_us") if tel else None,
    "telemetry_last_wall_us": tel[-1].get("wall_us") if tel else None,
    "telemetry_last_relative_to_signal_s": (fnum(tel[-1].get("wall_us"))-SIG_WALL_US)/1e6 if tel else None,
    "cleanup_finished_wall_us": cleanup["finished_wall_us"],
    "truth_input_lost_markers": [r.get("action") for r in st if r.get("action")],
}

# ---------- 8. C8 / C9 ----------
eg = jread(R + "/exit_grade.json")
res["C8"] = {
    "exit_grade": eg,
    "check_txt_landed_disarmed": next((l for l in open(R+"/check.txt", encoding="utf-8", errors="replace") if "landed_disarmed" in l), None),
    "state_transitions_tail": state_trans[-8:],
}
res["C9"] = {
    "process_cleanup": {k: cleanup.get(k) for k in ("reason","sid","started_wall_us","finished_wall_us",
        "leftovers","verified_empty","ownership_rule")},
    "cleanup_targets_count": len(cleanup.get("targets") or []),
    "cleanup_targets_all_exited": all(x.get("exited") for x in cleanup.get("targets") or []),
    "cleanup_verify": jread(R + "/cleanup_verify.json"),
    "shared_before": jread(D + "/shared_before.json"),
    "run_profile_returncode": jread(R + "/runtime_status.json")["run_profile_returncode"],
}

# ---------- 9. C10：两个普通 FAIL ----------
checks = jread(R + "/check.json")
res["C10"] = {
    "check_txt_verdict": next((l.strip() for l in open(R+"/check.txt", encoding="utf-8", errors="replace") if l.startswith("VERDICT")), None),
    "fail_checks": [c for c in checks.get("checks", []) if c.get("status")=="FAIL"],
    "unverified_checks": [c for c in checks.get("checks", []) if c.get("status")=="UNVERIFIED"],
    "pass_count": sum(1 for c in checks.get("checks", []) if c.get("status")=="PASS"),
}

# ---------- 10. 实际启动次数 / 来源 ----------
runs = sorted(glob.glob("/home/gengyinghao/drone_sim/logs/vio_closed_loop_intr_fix20_resume_20261116*"))
res["launch_accounting"] = {
    "matching_run_dirs": runs,
    "count": len(runs),
    "attempts_jsonl": [json.loads(l) for l in open(D+"/ATTEMPTS.jsonl", encoding="utf-8") if l.strip()],
    "run_json": jread(D+"/run.json"),
}
res["provenance"] = {
    "preflight_checks_mismatch": [c for c in jread(D+"/preflight.json")["checks"] if not c["match"]],
    "preflight_authorization_original": jread(D+"/preflight.json")["authorization"],
    "preflight_max_launches": jread(D+"/preflight.json")["max_simulation_launches"],
    "launcher_md5_history": {
        "FREEZE_FIX20_expected": jread(W+"/newpkg_fix20/FREEZE_FIX20.json")["files_md5"]["launch_run_wide_fix19.launcher.sh"],
        "before_scene_comma": hashlib.md5(open(D+"/launcher_before_scene_comma.sh","rb").read()).hexdigest(),
        "record_frozen": hashlib.md5(open(D+"/launcher_record_frozen.sh","rb").read()).hexdigest(),
        "live_production": hashlib.md5(open(W+"/newpkg_fix19/recordenv/launch_run_wide_fix19.sh","rb").read()).hexdigest(),
    },
}

os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w", encoding="utf-8") as f:
    json.dump(res, f, ensure_ascii=False, indent=2)
print("WROTE", OUT)
print("signal_epoch_s=", SIG_S, "NAV=", NAV_EPOCH_S,
      "bias=", round(SIG_S-NAV_EPOCH_S, 4))
print("C1 at+6:", res["C1"]["pilot_row_at_plus6"])
print("C5 publish total/after/before:", res["C5"]["publish_rows_total"],
      res["C5"]["publish_rows_after_signal"], res["C5"]["publish_rows_at_or_before_signal"])
print("C5 obs after:", res["C5"]["obs_rows_after_signal"],
      "odo growth", res["C5"]["odo_count_growth_after_signal"],
      "health growth", res["C5"]["health_count_growth_after_signal"])
print("C7 bridge pids:", BRIDGE_PIDS, "sup rows:", res["C7"]["supervisor_trace_rows"],
      "truth/tele last rel:", res["C7"]["truth_last_relative_to_signal_s"],
      res["C7"]["telemetry_last_relative_to_signal_s"])
print("C9 clean:", res["C9"]["cleanup_verify"], "targets:", res["C9"]["cleanup_targets_count"])
print("C10:", res["C10"]["check_txt_verdict"], [c["name"] for c in res["C10"]["fail_checks"]])
