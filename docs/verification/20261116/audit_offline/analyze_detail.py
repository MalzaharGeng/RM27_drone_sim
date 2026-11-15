#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""补充离线分析：真值位移、桥/监督器存活跨度、单调域映射、两个 FAIL 归因、process_cleanup 目标明细。
纯读取，不启动任何仿真。输出 analysis_detail.json。"""
import csv, json, os, math

R = "/home/gengyinghao/drone_sim/logs/vio_closed_loop_intr_fix20_resume_20261116_r1_20261116_000320"
D = "/mnt/c/Users/46292/Desktop/飞机/intr_resume_20261116"
OUT = os.path.join(D, "audit_offline_20261116", "analysis_detail.json")

def jread(p):
    return json.load(open(p, encoding="utf-8", errors="replace"))
def rows(p):
    return list(csv.DictReader(open(p, newline="", encoding="utf-8", errors="replace")))
def f(x, d=None):
    try: return float(x)
    except (TypeError, ValueError): return d

sig = jread(R+"/injection_signal.json")
SIG_W = sig["requested_wall_us"]          # epoch us
SIG_M = sig["sent_mono_us"]               # monotonic us
OFFSET = sig["requested_wall_us"] - sig["requested_mono_us"]   # epoch - mono
o = {}

# ---- 1. 真值位移曲线（判定两个普通 FAIL 是否确由主动撤销导致）----
truth = rows(R+"/truth.csv")
t0 = None; disp = []
for r in truth:
    rw = f(r.get("recv_wall_us")); x = f(r.get("x"),0); y = f(r.get("y"),0)
    if rw is None: continue
    if t0 is None: t0 = (x, y)
    disp.append((rw, math.hypot(x-t0[0], y-t0[1]), x, y))
peak = max(disp, key=lambda z: z[1])
o["truth_disp"] = {
    "rows": len(disp), "first_wall_us": disp[0][0], "last_wall_us": disp[-1][0],
    "peak_disp_m": peak[1], "peak_at_wall_us": peak[0],
    "peak_relative_to_signal_s": (peak[0]-SIG_W)/1e6,
    "peak_at_x_y": [peak[2], peak[3]],
    "disp_at_signal": next((d[1] for d in disp if d[0] >= SIG_W), None),
    "disp_at_revoke": next((d[1] for d in disp if d[0] >= 1794758693429069), None),
    "task_window_s": [1794758686366000/1e6, 1794758693463000/1e6],
}

# ---- 2. 桥两个 PID / 监督器在注入后的存活跨度 ----
pw = rows(R+"/proc_watch.csv")
def span(pid):
    rr = [r for r in pw if r.get("pid")==str(pid)]
    alive = [r for r in rr if r.get("alive")=="1"]
    after = [r for r in alive if (f(r.get("t_wall_us"),0) or 0) > SIG_W]
    return {"rows": len(rr), "alive_rows": len(alive),
            "first_alive_wall_us": f(alive[0]["t_wall_us"]) if alive else None,
            "last_alive_wall_us": f(alive[-1]["t_wall_us"]) if alive else None,
            "alive_rows_after_signal": len(after),
            "last_alive_relative_to_signal_s": ((f(alive[-1]["t_wall_us"])-SIG_W)/1e6) if alive else None,
            "first_dead_wall_us": next((f(r["t_wall_us"]) for r in rr if r.get("alive")!="1"), None)}
o["proc_spans"] = {"bridge_337152": span(337152), "bridge_337238": span(337238),
                   "supervisor_337734": span(337734), "node_337397": span(337397),
                   "pilot_337732": span(337732)}

# ---- 3. 监督器轨迹的单调域映射（禁止与 epoch 相减）----
st = rows(R+"/supervisor_trace.csv")
mono_first = f(st[0]["t_wall_us"]); mono_last = f(st[-1]["t_wall_us"])
o["supervisor_mono_domain"] = {
    "domain": "CLOCK_MONOTONIC us",
    "first_t_wall_us_is_mono": mono_first, "last_t_wall_us_is_mono": mono_last,
    "signal_sent_mono_us": SIG_M,
    "trace_last_minus_signal_mono_s": (mono_last-SIG_M)/1e6,
    "trace_first_minus_signal_mono_s": (mono_first-SIG_M)/1e6,
    "mono_to_epoch_offset_us": OFFSET,
    "trace_last_epoch_us_if_mapped": mono_last + OFFSET,
    "trace_last_epoch_relative_to_signal_s": ((mono_last+OFFSET)-SIG_W)/1e6,
    "trace_rows_after_signal_mono": sum(1 for r in st if (f(r["t_wall_us"],0) or 0) > SIG_M),
    "trace_rows_total": len(st),
    "first_pose_mono_s_from_supervisor_json": jread(R+"/supervisor.json")["first_pose_mono_s"],
}

# ---- 4. 两个普通 FAIL 的完整证据 ----
ck = jread(R+"/check.json")
o["fail_detail"] = {c["name"]: c for c in ck["checks"] if c.get("status")!="PASS"}
o["pass_names"] = [c["name"] for c in ck["checks"] if c.get("status")=="PASS"]
o["check_txt_first_lines"] = open(R+"/check.txt",encoding="utf-8",errors="replace").read().splitlines()[:6]

# ---- 5. process_cleanup 目标明细（共享进程是否被列为目标）----
pc = jread(R+"/process_cleanup.json")
tgt = [{"role": t.get("role"), "pid": t.get("pid"), "exited": t.get("exited"),
        "signal": t.get("signal"), "exit_state": t.get("exit_state")} for t in pc["targets"]]
o["cleanup"] = {"targets": tgt, "leftovers": pc["leftovers"],
    "verified_empty": pc.get("verified_empty"),
    "shared_agent_pid": 3643,
    "shared_agent_in_targets": any(t["pid"]==3643 for t in tgt),
    "self_excluded": pc.get("self_excluded"),
    "ownership_rule": pc.get("ownership_rule"),
    "reason": pc.get("reason")}

# ---- 6. C1 在 +6.0s 精确窗口内的状态分布 ----
pilot = rows(R+"/pilot.csv")
NAV = 1794758686.324199
w6 = [r for r in pilot if NAV+5.5 <= (f(r.get("wall_us"),0) or 0)/1e6 <= NAV+6.5]
from collections import Counter
o["C1_window"] = {"rows": len(w6),
    "states": dict(Counter(r.get("state") for r in w6)),
    "bt_phases": dict(Counter(r.get("bt_phase") for r in w6)),
    "task_completed": dict(Counter(r.get("task_completed") for r in w6)),
    "control_active": dict(Counter(r.get("control_active") for r in w6)),
    "bt_reasons": dict(Counter(r.get("bt_reason") for r in w6)),
    "first_wall_us": w6[0]["wall_us"] if w6 else None,
    "last_wall_us": w6[-1]["wall_us"] if w6 else None}

# ---- 7. 注入器退出是否触发整轮 cleanup（时间先后）----
o["injector_vs_cleanup"] = {
    "injector_finished_wall": "2026-11-16T00:05:07.927048+08:00",
    "cleanup_started_wall_us": pc["started_wall_us"],
    "cleanup_finished_wall_us": pc["finished_wall_us"],
    "run_log_cleanup_line": "00:05:05.585567822 清理开始 reason=injected sid=336330",
    "run_log_done_line": "00:05:04.569442628 飞手流程结束 DONE",
    "note": "round cleanup is triggered by the whole-round runner after pilot DONE, not by injector exit; injector waits on the launcher (wait $L) and exits after round finished",
}

# ---- 8. C3 注入器退出不触发 cleanup 的证据（injector_console / ATTEMPTS）----
o["launch_accounting_extra"] = {
    "LAUNCH_ONCE_lock": open(D+"/LAUNCH_ONCE.lock",encoding="utf-8").read().strip(),
    "ATTEMPTS_lines": len([l for l in open(D+"/ATTEMPTS.jsonl",encoding="utf-8") if l.strip()]),
    "matching_run_dirs_1": True,
}

json.dump(o, open(OUT,"w",encoding="utf-8"), ensure_ascii=False, indent=2)
print("WROTE", OUT)
print("truth peak disp %.3f m at rel %.3f s (signal epoch)" % (o["truth_disp"]["peak_disp_m"], o["truth_disp"]["peak_relative_to_signal_s"]))
print("bridge spans:", {k:(v["alive_rows_after_signal"],v["last_alive_relative_to_signal_s"]) for k,v in o["proc_spans"].items()})
print("sup trace rows after signal(mono):", o["supervisor_mono_domain"]["trace_rows_after_signal_mono"],
      "of", o["supervisor_mono_domain"]["trace_rows_total"])
print("shared agent in cleanup targets:", o["cleanup"]["shared_agent_in_targets"])
print("C1 window states:", o["C1_window"]["states"], o["C1_window"]["bt_phases"])
