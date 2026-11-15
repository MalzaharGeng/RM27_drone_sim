#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 interruption_audit.json。全部数值取自 analysis_raw.json / analysis_detail.json
及原始证据文件；纯读取，不启动任何仿真、不发送任何信号、不修改生产文件。"""
import json, os, datetime

D = "/mnt/c/Users/46292/Desktop/飞机/intr_resume_20261116"
SUB = os.path.join(D, "audit_offline_20261116")
A = json.load(open(os.path.join(SUB, "analysis_raw.json"), encoding="utf-8"))
B = json.load(open(os.path.join(SUB, "analysis_detail.json"), encoding="utf-8"))
OUT = os.path.join(D, "interruption_audit.json")

R = A["run_dir"]
sig = A["C3"]["injection_signal"]
S_W = sig["requested_wall_us"]          # epoch us，注入请求时刻（基准）
S_M = sig["sent_mono_us"]               # CLOCK_MONOTONIC us

def rel(epoch_us):
    return round((epoch_us - S_W) / 1e6, 6)

NAV = A["timeline"]["navigate_epoch_s"]
clock = A["clock"]

def ev(file, locator, values, note=None):
    d = {"file": file, "locator": locator, "key_values": values}
    if note:
        d["note"] = note
    return d

criteria = []

criteria.append({
    "id": "C1", "name": "注入前置条件", "verdict": "PASS",
    "evidence": [
        ev(R+"/pilot.log", "line 27", {"NAVIGATE_epoch_s": NAV,
            "text": A["timeline"]["navigate_line"]}),
        ev(R+"/pilot.csv", "NAV+5.5..NAV+6.5 s, 17 rows", B["C1_window"],
           "all 17 rows: state=NAVIGATE, bt_phase=NAVIGATE, task_completed=False, control_active=True, bt_reason=pilot_authorized"),
        ev(R+"/pilot.csv.events.json", "sequence 1..4", {"task_completed_event": None,
            "task_revoked_wall_us": 1794758693429069,
            "task_revoked_rel_to_NAVIGATE_s": round(1794758693429069/1e6 - NAV, 6)}),
        ev(R+"/check.txt", "line 6,9", {"no_airborne_fault": "PASS airborne_fault=0",
            "gate_attitude_continuity": "PASS GATE_FAULT=0"}),
        ev(R+"/gate_snapshot.csv", "837 rows", {"GATE_FAULT_rows": 0}),
    ],
    "key_numbers": {
        "NAVIGATE_epoch_s": NAV,
        "plus_6s_epoch_s": round(NAV+6.0, 6),
        "actual_signal_rel_to_NAVIGATE_s": A["timeline"]["biases_s"]["signal_requested_minus_NAVIGATE"],
        "bias_vs_DROP_AFTER_6.0_s": A["timeline"]["biases_s"]["signal_requested_minus_DROP_AFTER_6s"],
        "task_revoked_rel_to_signal_s": B["C1_window"] and round(1794758693429069/1e6 - S_W/1e6, 6),
        "FAULT_states": A["C1"]["FAULT_in_pilot_states"],
    },
    "reasoning": "在既定 +6.0 s 时刻及其后至实际发送时刻，pilot 始终处于 NAVIGATE、未 task_completed、未 REVOKED、无 FAULT/GATE_FAULT。DROP_AFTER=6.0 不等于精确 +6.000 s 发送：实际 +6.476194 s，偏差 +0.476194 s 来自条件检查、就绪复核、身份核验与 signal-pid 调用开销。按用户要求仅报告偏差，不调整既定判据。",
    "limitations": "条件检查（STATE_AT / count_matches）本身无独立单调时间戳，只能界定在 NAV+6.0 s 与就绪复核 NAV+6.140 s 之间。",
})

criteria.append({
    "id": "C2", "name": "唯一节点身份与就绪检查", "verdict": "PASS",
    "evidence": [
        ev(R+"/node_pid.json", "whole file", A["C2"]["node_pid_json"]),
        ev(R+"/injection_readiness.json", "candidate_pids / target_identity / problems / excluded_*",
           {"candidate_pids": A["C2"]["candidate_pids"], "unique_vio_node": A["C2"]["unique_vio_node"],
            "ready": A["C2"]["injection_readiness_ready"], "problems": A["C2"]["problems"],
            "excluded_count": A["C2"]["excluded_count"],
            "frozen_node_sha256": A["C2"]["frozen_node_sha256"],
            "target_sha256": A["C2"]["target_identity"]["sha256"],
            "target_starttime_s_boot": A["C2"]["target_identity"]["starttime_s_boot"]}),
        ev(R+"/injection_target_identity.json", "exe_file_hash / expect_exe_hash / exe.note",
           {"hash_match": A["C2"]["identity_hash_match"],
            "is_only_vio_publisher": A["C2"]["identity_is_only_vio_publisher"],
            "exe_note": A["C2"]["identity_exe_note"],
            "sha256": A["C2"]["identity_exe_hash"]["sha256"]}),
        ev(R+"/proc_registry.jsonl", "role=node (1 entry), role=bridge (2 entries), role=launcher (1 entry)",
           {"node_entries": A["C2"]["registry_node_entries"],
            "bridge_entries": A["C2"]["registry_bridge_entries"],
            "excluded_launcher_runner_bridge": A["C2"]["excluded_roles_launcher_runner_bridge"]}),
        ev(D+"/preflight_output.txt", "S2_non_nodes_excluded",
           {"pass": True, "excluded": 3, "note": "起飞前替身自检显式验证 launcher/runner/bridge 三者均被排除"}),
    ],
    "key_numbers": {"frozen_node_md5": "bfa646c84be62c15b1fcfaa5743dbd43",
                    "frozen_node_sha256": A["C2"]["frozen_node_sha256"],
                    "target_pid": 337397, "target_starttime_s_boot": 4990088},
    "reasoning": "恰好 1 个进程 exe 等于冻结节点二进制；PID/starttime/exe_realpath/完整 sha256 与 node_pid.json、登记表、冻结哈希三处一致；launcher(337312) 与桥(337152/337238)在运行期就绪清单中被显式排除；整轮运行器是会话首领(sid=336330)，由唯一 exe 匹配、起飞前 S2 替身自检和会话隔离共同保证不被当作节点。",
    "limitations": "运行期 injection_readiness.json 的 excluded_non_node_processes 里没有名为 role=runner 的独立条目（runner 是会话首领，未登记为 registry 角色）；其排除依据为唯一 exe 匹配 + 起飞前 S2 自检 + 注入器与整轮不同会话，非运行期显式清单项。",
})

criteria.append({
    "id": "C3", "name": "信号", "verdict": "PASS",
    "evidence": [
        ev(R+"/injection_signal.json", "whole file", sig),
        ev(R+"/interruption.log", "lines 5-12", {"signal": "SIGTERM", "signal_rc": 0,
            "target_pid": 337397, "inject_mono": A["C3"]["interruption_log"]["inject_mono"],
            "inject_epoch": A["C3"]["interruption_log"]["inject_epoch"],
            "publish_rows_before": 452, "publish_rows_after": 460}),
        ev(R+"/INJECTION_MARKER", "0-byte file", {"exists": True, "size_bytes": 0},
           "文件存在仅证明写入，不单独证明进程死亡或消息停止"),
        ev(D+"/injector_frozen.sh", "lines 58-59, 90-95, 106-133, 158",
           {"setsid": True, "wait_L": True, "signal": "TERM", "planned_target_only": True},
           "整轮由 setsid 独立会话启动；注入器 wait $L 后才退出，不在自身退出时调用整轮 cleanup"),
        ev(R+"/run.log", "lines 20-24", {"cleanup_reason": "injected", "cleanup_at": "00:05:05.585",
            "injector_finished_at": "2026-11-16T00:05:07.927048+08:00"}),
        ev(D+"/preflight_output.txt", "S6/S7", {"S6_only_node_signalled": True, "S7_injector_exit_no_round_cleanup": True}),
    ],
    "key_numbers": {"requested_mono_us": sig["requested_mono_us"],
                    "sent_mono_us": sig["sent_mono_us"],
                    "requested_to_sent_us": A["clock"]["signal_delta_requested_to_sent_us"],
                    "signal": "SIGTERM", "result": "sent", "signal_rc": 0},
    "reasoning": "SIGTERM 仅发给 C2 确认的 PID 337397（expect-exe + expect-starttime 双重校验，starttime_match=true、exe_match=true），result=sent。主机单调时间可靠：injection_signal.json 的 requested_mono_us/sent_mono_us 为 time.monotonic_ns()//1000（np01_lifecycle.py:57-58,434,453），相差 87 µs。注入器退出不触发整轮无条件 cleanup：整轮在 DONE 后由 run_profile 的 trap 触发 reason=injected 清理（00:05:05.585，早于注入器 00:05:07.927 退出）。",
    "limitations": "interruption.log 的 inject_mono 实为 date +%s.%N 的 epoch（与 inject_epoch 相差 0.006 s），不是单调时间，不能用于单调判据。按用户要求已改用 injection_signal.json 的 mono_us 字段。",
})

criteria.append({
    "id": "C4", "name": "进程实际退出", "verdict": "PASS",
    "evidence": [
        ev(R+"/launcher_exit.json", "whole file", A["C4"]["launcher_exit"]),
        ev(R+"/proc_watch.csv", "role=node pid=337397, alive 1->0",
           {"last_alive_wall_us": B["proc_spans"]["node_337397"]["last_alive_wall_us"],
            "first_dead_wall_us": B["proc_spans"]["node_337397"]["first_dead_wall_us"],
            "first_dead_rel_to_signal_s": rel(B["proc_spans"]["node_337397"]["first_dead_wall_us"]),
            "alive_rows_after_signal": 0}),
        ev(R+"/interruption.log", "line 10", {"node_exited_within_5s": 1}),
        ev("newpkg_fix18/runenv/run_stereo_inertial_sim_fix18.sh", "lines 170-186, 342-348",
           {"NODE_RC_initial": 0, "wait_loop": "while kill -0; do wait; NODE_RC=$?; [[ $NODE_RC -gt 128 ]] || break; done"}),
    ],
    "key_numbers": {"node_pid": 337397, "node_exit_code": 0, "launcher_rc": 0,
                    "node_starttime_s_boot": 4990088,
                    "proc_gone_first_sample_rel_to_signal_s": rel(B["proc_spans"]["node_337397"]["first_dead_wall_us"])},
    "reasoning": "proc_watch.csv 显示 337397 在信号后 +0.376290 s 的采样点已 alive=0（采样间隔 1 s，实际消失更早）；注入器以 0.1 s 轮询 /proc 得到 node_exited_within_5s=1；launcher_exit.json 记录同一 PID/starttime。进程证据与消息停更（C5）相互独立、互相印证。",
    "limitations": "launcher_exit.json 的 node_exit_code=0 与 SIGTERM 的常规退出码（143/-15）不一致：源码显示 NODE_RC 初值为 0 且 wait 循环在 rc<=128 时 break，节点自行处理 SIGTERM 后正常退出即记录 0。因此该字段只满足「记录了 node_exit_code」的判据字面，不能单独作为「信号导致退出」的证据；实际死亡由 proc_watch + /proc 轮询 + 消息停更共同证明。proc_watch 为 1 s 采样，无法给出毫秒级消失时刻。",
})

criteria.append({
    "id": "C5", "name": "消息通路停止", "verdict": "PASS",
    "evidence": [
        ev(R+"/vio_obs_counter.csv", "154 rows, tail 27 rows after signal",
           {"odo_final_count": 460, "health_final_count": 315,
            "odo_last_rx_wall_us": 1794758692818580, "health_last_rx_wall_us": 1794758692817653,
            "odo_last_rx_rel_to_signal_s": A["C5"]["odo_last_rx_relative_to_signal_s"],
            "health_last_rx_rel_to_signal_s": A["C5"]["health_last_rx_relative_to_signal_s"],
            "obs_rows_after_signal": A["C5"]["obs_rows_after_signal"],
            "odo_count_growth_after_signal": A["C5"]["odo_count_growth_after_signal"],
            "health_count_growth_after_signal": A["C5"]["health_count_growth_after_signal"],
            "first_frozen_sample_rel_to_signal_s": 0.1186}),
        ev(R+"/bridge_telemetry.csv", "460 publish rows; seq 452..460",
           {"publish_rows_total": 460,
            "seq452_system_us": A["C5"]["publish_seq_452_system_us"],
            "seq452_rel_to_signal_s": A["C5"]["publish_seq_452_relative_to_signal_s"],
            "seq460_system_us": A["C5"]["publish_seq_460_system_us"],
            "seq460_rel_to_signal_s": A["C5"]["publish_seq_460_relative_to_signal_s"],
            "publish_rows_at_or_before_signal": A["C5"]["publish_rows_at_or_before_signal"],
            "publish_rows_after_signal_by_system_us": A["C5"]["publish_rows_after_signal"],
            "publish_tail_12": A["C5"]["publish_tail_12"]}),
        ev(R+"/runtime_status.json", "bridge_publish_rows", {"bridge_publish_rows": 460},
           "收尾时仍为 460，无 461；证明后续计数冻结"),
        ev(D+"/injector_frozen.sh", "lines 126-127 vs 142",
           {"PUB_BEFORE_timing": "before marker & signal-pid (signal 前)",
            "PUB_AFTER_timing": "after node-exit wait loop (up to 5 s) and bridge check (signal 后)",
            "source_file": "$ROOT/px4_telemetry_sim_transport.csv (运行中的实时文件)"},
           "两个快照跨不同时刻，且实时文件存在 flush 滞后；不能当作同一时刻的发布计数"),
    ],
    "key_numbers": {"publish_rows_before": 452, "publish_rows_after": 460, "delta": 8,
                    "rows_of_delta_before_signal": 7, "rows_of_delta_after_signal": 1,
                    "last_inflight_row_sample_us_rel_to_signal_s": -0.071052,
                    "odo_health_frozen_for_s": 13.19},
    "reasoning": "对 452→460 的 8 行差值按要求做了三方核对：(1) 两个快照时刻不同——PUB_BEFORE 取自 signal 前、PUB_AFTER 取自节点退出等待之后，且读的是带 flush 滞后的实时文件；(2) 与信号时刻对齐——seq453..459 共 7 行的 system_us 全在信号前（-0.296986..-0.020367 s），唯一一行 seq460 的 system_us 在信号后 +0.017829 s，但其 sample_us=1794758692729341 在信号前 0.071052 s，属信号前采样、信号后落盘的在途消息；(3) 后续计数冻结——odo/health 在信号后 27 个采样（+0.1186 s 至 +13.19 s）零增长、最近接收时刻冻结在 +0.018 s，publish 最终停在 460 且收尾 runtime_status 仍为 460，无 461。因此不能据该差值判定「注入后仍持续发布」，也不能忽略这 1 条在途落盘行。判据主项（接收计数与最近接收时间冻结）成立，文件仅作辅助。",
    "limitations": "严格按 system_us 字面解释「bridge_telemetry.csv 无新 publish 行」，存在 1 行（seq460，+17.8 ms）；该行源采样在信号前且之后计数冻结，判据已注明「文件仅辅助」。无独立 ROS 层订阅计数快照，接收冻结证据来自 vio_obs_counter.csv 采样（0.5 s 间隔）。",
})

criteria.append({
    "id": "C6", "name": "授权撤销与任务停止", "verdict": "PASS",
    "evidence": [
        ev(R+"/bt.log", "line 14", {"text": A["C6"]["bt_log_revoked_lines"][0],
            "rel_to_signal_s": rel(1794758693418996)}),
        ev(R+"/pilot.log", "lines 30-32", A["C6"]["pilot_log_revoked_lines"]),
        ev(R+"/pilot.csv.events.json", "sequence 4", A["C6"]["task_revoked_event"]),
        ev(R+"/pilot.csv", "rows after signal", {"control_active_after_signal": A["C6"]["control_active_after_signal"],
            "navigate_rows_after_signal": A["C6"]["navigate_rows_after_signal"]}),
    ],
    "key_numbers": {"revoke_reason": "stale_health",
                    "bt_log_revoke_rel_to_signal_s": rel(1794758693418996),
                    "task_revoked_event_rel_to_signal_s": round(1794758693429069/1e6 - S_W/1e6, 6),
                    "TAKEOVER_rel_to_signal_s": rel(1794758693423524),
                    "state_transitions_after_signal": A["C6"]["state_transitions_after_signal"]},
    "reasoning": "BT 在 stale_health 下撤销授权（bt.log:14，信号后 +0.6186 s），pilot.log:30 记录 cycling RC selector back to Position bt_reason=stale_health，pilot.csv.events.json sequence=4 记录 task_revoked/reason=stale_health（+0.6287 s）；pilot 于 +0.6231 s 进入 TAKEOVER，此后 control_active=False 189 行，NAVIGATE 仅剩 11 行尾部采样，任务指令停止。撤销原因与 C5 的 health 停更一致。",
    "limitations": "无独立的「授权令牌」对象可核验，撤销以 BT/pilot 事件与状态迁移为证。",
})

criteria.append({
    "id": "C7", "name": "桥、监督器、真值与遥测持续", "verdict": "PASS",
    "evidence": [
        ev(R+"/bridge_owner.fix19.json", "child_parameter_bridge_pids / loaded_ros_gz_bridge_so",
           A["C7"]["bridge_owner_record"], "初始记录两个子进程列表均为空，属记录缺陷"),
        ev(R+"/proc_registry.jsonl", "role=bridge entries", {"bridge_pids": [337152, 337238],
            "entries": A["C2"]["registry_bridge_entries"]},
           "按用户要求不只看父进程：337152 是 ros2 run 包装进程，337238 是 parameter_bridge 本体"),
        ev(R+"/proc_watch.csv", "pids 337152/337238/337734 after signal",
           {"bridge_337152": B["proc_spans"]["bridge_337152"],
            "bridge_337238": B["proc_spans"]["bridge_337238"],
            "supervisor_337734": B["proc_spans"]["supervisor_337734"]}),
        ev(R+"/supervisor_trace.csv", "3242 rows (monotonic domain)",
           {"rows_after_signal_by_mono": B["supervisor_mono_domain"]["trace_rows_after_signal_mono"],
            "last_minus_signal_mono_s": B["supervisor_mono_domain"]["trace_last_minus_signal_mono_s"],
            "actions": A["C7"]["supervisor_trace_actions"],
            "lost_us_max": A["C7"]["supervisor_trace_lost_us_max"]}),
        ev(R+"/supervisor.json", "whole file", A["C7"]["supervisor_json"]),
        ev(R+"/truth.csv / telemetry.csv", "last rows",
           {"truth_rows": A["C7"]["truth_rows"], "truth_last_rel_to_signal_s": A["C7"]["truth_last_relative_to_signal_s"],
            "telemetry_rows": A["C7"]["telemetry_rows"],
            "telemetry_last_rel_to_signal_s": A["C7"]["telemetry_last_relative_to_signal_s"]}),
    ],
    "key_numbers": {"bridge_pids": [337152, 337238], "supervisor_pid": 337734,
                    "bridge_alive_rows_after_signal": 13,
                    "bridge_last_alive_rel_to_signal_s": B["proc_spans"]["bridge_337152"]["last_alive_relative_to_signal_s"],
                    "supervisor_trace_rows_after_signal": B["supervisor_mono_domain"]["trace_rows_after_signal_mono"],
                    "truth_last_rel_s": A["C7"]["truth_last_relative_to_signal_s"],
                    "telemetry_last_rel_s": A["C7"]["telemetry_last_relative_to_signal_s"],
                    "cleanup_started_rel_s": rel(A["C9"]["process_cleanup"]["started_wall_us"]),
                    "lost_us_total": 0, "stop_event": None, "warn_event": None},
    "reasoning": "桥两个真实 PID（337152/337238）与监督器 337734 在信号后各有 13 个 alive 采样，最后存活至 +12.39 s；supervisor_trace.csv 在信号后新增 643 行、最后一行 +12.818 s（按同域 mono 比较）；truth.csv 4128 行、telemetry.csv 763 行分别持续到 +13.194 s / +12.865 s，均覆盖到清理开始 +12.827 s。supervisor.json：lost_us_total=0、warned=false、triggered=false、stop_event=null、hard_kill=null；trace action 全为空，无 truth_input_lost、无 stop_event。",
    "limitations": "bridge_owner.fix19.json 的 child_parameter_bridge_pids 与 loaded_ros_gz_bridge_so 初始为空，须由 proc_registry.jsonl + proc_watch.csv 补齐两个 PID（已补齐）。supervisor_trace.csv 的列名 t_wall_us 实为 CLOCK_MONOTONIC µs（np01_truth_supervisor.py:57,133,202），已先核对源码并用同域 mono 比较/映射，未与 epoch 直接相减。",
})

criteria.append({
    "id": "C8", "name": "接回与落地", "verdict": "PASS",
    "evidence": [
        ev(R+"/pilot.log", "lines 31-35", {"TAKEOVER": 1794758693.423523699,
            "LAND": 1794758695.425228545, "LANDED": 1794758701.926318665, "DONE": 1794758703.927168394}),
        ev(R+"/exit_grade.json", "whole file", A["C8"]["exit_grade"]),
        ev(R+"/check.txt", "line 15", A["C8"]["check_txt_landed_disarmed"]),
    ],
    "key_numbers": {"TAKEOVER_rel_s": rel(1794758693423524),
                    "LAND_rel_s": rel(1794758695425229),
                    "LANDED_rel_s": rel(1794758701926319),
                    "DONE_rel_s": rel(1794758703927168),
                    "pilot_final_armed": False, "landing_confirmed": True,
                    "truth_altitude_residual_m": 0.00},
    "reasoning": "TAKEOVER→LAND→LANDED→DONE 完整状态链；pilot_final_armed=false，landing_confirmed=true，真值高度残差 0.00 m，landed_disarmed 为 PASS，disarm 确认。",
    "limitations": "exit_grade.json 的 grade=A / subtype=A_early 是退出分级（safe exit 语义），不是本次冻结判据的 A/B/C 专项分类，已按用户要求区分，不混用。",
})

criteria.append({
    "id": "C9", "name": "整轮清理", "verdict": "UNVERIFIED",
    "verdict_split": {
        "cleanup_of_owned_processes": "PASS",
        "shared_process_not_touched": "UNVERIFIED",
    },
    "evidence": [
        ev(R+"/process_cleanup.json", "whole file", {"reason": "injected", "sid": 336330,
            "targets": A["C9"]["cleanup_targets_count"],
            "all_exited": A["C9"]["cleanup_targets_all_exited"],
            "leftovers": A["C9"]["process_cleanup"]["leftovers"],
            "verified_empty": A["C9"]["process_cleanup"]["verified_empty"],
            "ownership_rule": A["C9"]["process_cleanup"]["ownership_rule"]}),
        ev(R+"/cleanup_verify.json", "whole file", A["C9"]["cleanup_verify"]),
        ev(D+"/shared_before.json", "whole file", {"pid": 3643,
            "exe": A["C9"]["shared_before"][0]["exe"],
            "stat": "sid=3643, pgrp=3643 (与整轮 sid=336330 不同)"}),
        ev(R+"/preflight_reclaim_node.json / preflight_reclaim_px4_gz.json", "hits", {"count": 0, "hits": []}),
        ev("(缺失)", "preflight_orphans.json / shared_after.json", {"exists": False},
           "冻结判据 C9 evidence 点名的 preflight_orphans.json 本轮未生成；亦无共享进程运行后存活快照"),
    ],
    "key_numbers": {"cleanup_targets": 16, "all_exited": True, "leftovers": 0,
                    "verified_empty": True, "cleanup_verify_clean": True,
                    "shared_agent_pid": 3643, "shared_agent_in_cleanup_targets": False,
                    "cleanup_started_rel_s": rel(A["C9"]["process_cleanup"]["started_wall_us"]),
                    "cleanup_finished_rel_s": rel(A["C9"]["process_cleanup"]["finished_wall_us"]),
                    "pid_3643_present_at_audit_time": False},
    "reasoning": "本轮拥有的 16 个进程全部 exited、leftovers=[]、verified_empty=true、cleanup_verify clean=true、alive_members=[]，且清理规则为「registry 身份(pid+starttime) 且 sid==本轮 sid，无 pkill」，清理目标清单不含共享 MicroXRCEAgent(3643)，预检 reclaim 命中数为 0 —— 这部分 PASS。但判据还要求「共享进程未被触碰」：本轮只有运行前 shared_before.json，没有运行后存活快照（shared_after.json 缺失），冻结判据点名的 preflight_orphans.json 也未生成；离线审计时 /proc/3643 已不存在，无法据此判定它是在本轮之后自然退出还是被别的任务清理。按「证据缺失即 UNVERIFIED，不强判 C」的要求，C9 整体判 UNVERIFIED。",
    "limitations": "当前 /proc/3643 不存在不能反向证明「被本轮清理」，也不能证明「未被触碰后仍存活」。两种可能（会话结束后自然退出 / 其他任务清理）均无证据可排除。",
})

criteria.append({
    "id": "C10", "name": "原始评估结果与专项结论分离", "verdict": "PASS",
    "evidence": [
        ev(R+"/check.txt", "line 23", {"VERDICT": A["C10"]["check_txt_verdict"]}),
        ev(R+"/check.json", "checks[]", {"fail_checks": A["C10"]["fail_checks"],
            "unverified_checks": A["C10"]["unverified_checks"],
            "pass_count": A["C10"]["pass_count"]}),
        ev("(本审计交付)", "interruption_audit.json / INTERRUPTION_AUDIT_REPORT.md",
           {"generated": True}, "专项结论单列，不把普通 FAIL 改成 PASS"),
        ev("(缺失)", "interruption_result.json / injection_verdict.txt", {"exists": False}),
    ],
    "key_numbers": {"PASS": 14, "FAIL": 2, "UNVERIFIED": 1,
                    "fail_names": ["bt_task_completed", "truth_distance_hold"],
                    "unverified_name": "latency_evidence"},
    "reasoning": "check.txt / check.json 的 VERDICT=FAIL 14/2/1 原样保留，未改成 PASS；本次离线审计另出专项结论单列，符合判据。C10 是交付约束，不参与 A/B/C 判定。",
    "limitations": "本轮收尾未落盘 interruption_result.json / injection_verdict.txt（冻结判据 C10 evidence 点名的文件之一）；专项结论由本次离线审计补出，属交付缺口而非结果篡改。",
})

# ---------- 关键响应时间线 ----------
def T(label, epoch_us, note=""):
    d = {"event": label, "epoch_us": epoch_us, "rel_to_signal_s": rel(epoch_us)}
    if note: d["note"] = note
    return d

timeline = [
    T("pilot state -> NAVIGATE", round(NAV*1e6), "pilot.log:27"),
    T("就绪复核 checked_wall_us", 1794758692464177, "injection_readiness.json；NAV+6.140 s"),
    T("interruption.log inject_epoch (实为 epoch)", 1794758692593313, "date +%s.%N，非单调时间"),
    T("身份核验 recorded_wall_us", 1794758692669756, "injection_target_identity.json；NAV+6.346 s"),
    T("publish seq452", 1794758692462264, "bridge_telemetry.csv；PUB_BEFORE 快照附近"),
    T("publish seq459", 1794758692780026, "最后一条 system_us 在信号前的 publish"),
    T("SIGTERM requested", sig["requested_wall_us"], "injection_signal.json；NAV+6.476194 s"),
    T("SIGTERM sent", S_W + 87, "sent_mono_us - requested_mono_us = 87 µs"),
    T("最后 odo 接收", 1794758692818580, "vio_obs_counter.csv"),
    T("最后 health 接收", 1794758692817653, "vio_obs_counter.csv"),
    T("publish seq460 落盘", 1794758692818222, "system_us 在信号后 17.8 ms，但 sample_us 在信号前 71 ms"),
    T("odo/health 计数首次冻结采样", 1794758692918991, "此后 27 个采样零增长"),
    T("节点 337397 首个 dead 采样", 1794758693176683, "proc_watch.csv，1 s 采样间隔"),
    T("BT Task revoked: stale_health", 1794758693418996, "bt.log:14"),
    T("pilot cycling RC back to Position", 1794758693423292, "pilot.log:30"),
    T("state -> TAKEOVER", 1794758693423524, "pilot.log:31"),
    T("BT EVENT task_revoked", 1794758693429069, "pilot.csv.events.json seq=4"),
    T("state -> LAND", 1794758695425229, "pilot.log:33"),
    T("state -> LANDED", 1794758701926319, "pilot.log:34"),
    T("state -> DONE / disarm", 1794758703927168, "pilot.log:35"),
    T("桥/监督器最后 alive 采样", 1794758705192060, "proc_watch.csv"),
    T("supervisor_trace 最后一行(mono 映射)", 1794758705618572, "supervisor_trace.csv；+12.818 s"),
    T("清理开始", 1794758705627475, "run.log:22 reason=injected"),
    T("telemetry.csv 最后一行", 1794758692800393 + 12864598, "telemetry.csv；+12.865 s"),
    T("truth.csv 最后一行", 1794758705994288, "truth.csv；+13.194 s"),
    T("清理完成", 1794758707078789, "process_cleanup.json"),
]
timeline = sorted(timeline, key=lambda x: x["epoch_us"])

audit = {
    "audit_id": "NP01_FIX20_INTR_OFFLINE_AUDIT_20261116",
    "audit_type": "offline read-only; NO new simulation, NO flight, NO signal, NO production file change",
    "generated_at": datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),
    "run_dir": R,
    "criteria_source": {
        "path": "/home/gengyinghao/drone_sim/wb_handoff_20261114/workpkg_20261115/newpkg_fix19/FROZEN_INTERRUPTION_CRITERIA.json",
        "md5": A["criteria_md5"], "md5_matches_user_declaration": A["criteria_md5"] == "6e5adef1bf208011f5979885b1b53a62",
        "criteria_ids": A["criteria_ids"], "verdict_rule": A["verdict_rule"],
        "unchanged": True,
    },
    "clock_domains": clock,
    "signal_reference": {"epoch_us_used_as_zero": S_W,
        "monotonic_us_sent": S_M,
        "note": "所有 rel_to_signal_s 以 injection_signal.json 的 requested_wall_us (epoch) 为基准；单调域比较在 C7 单独说明"},
    "injection_timeline": timeline,
    "criteria_results": criteria,
    "ordinary_evaluator": {
        "verdict": A["C10"]["check_txt_verdict"],
        "counts": {"PASS": 14, "FAIL": 2, "UNVERIFIED": 1},
        "fail_details": A["C10"]["fail_checks"],
        "unverified_details": A["C10"]["unverified_checks"],
        "preserved_unchanged": True,
        "two_FAIL_attribution": [
            {"name": "bt_task_completed",
             "detail": A["C10"]["fail_checks"][0]["detail"],
             "cause": "注入后 stale_health 撤销，BT phases 停在 NAVIGATE/REVOKED，从未进入 HOLD；目标距离 5.00 m 与指令一致，不是配置或导航错误。正常 FIX20 轮需 NAV+22.9 s 才 task_completed，本轮在 NAV+7.105 s 已撤销。",
             "cross_check": ["pilot.log:32 task_revoked stale_health", "bt.log:14", "check.txt:10 nav_ready_continuous PASS 覆盖率 100%", "check.txt:7 ekf_using_ev PASS 97%", "check.txt:17 vio_vs_truth PASS RMSE 0.0074 m"]},
            {"name": "truth_distance_hold",
             "detail": A["C10"]["fail_checks"][1]["detail"],
             "cause": "任务窗口被撤销截断为 7.1 s（正常轮 31.1 s）。按评估器口径重算：信号时位移 1.318 m、撤销时 1.476 m、窗口末 1.481 m、峰值 1.649 m，从未接近 5.00±0.60 m，保持 0.0 s。撤销后仍有约 0.17 m 惯性位移，属接回减速过程，非异常。",
             "cross_check": ["truth.csv 按 check_run.py:398-426 口径重算 dmax=1.649 m", "check.txt:6 offboard_accepted PASS 122 行 NAVIGATE", "check.txt:9 no_airborne_fault PASS", "check.txt:18 gate_attitude_continuity PASS GATE_FAULT=0"]},
        ],
        "unverified_note": "latency_evidence 是历史遗留：latency.csv 无 age_ms 列且字段分属 sim/epoch/mono 三时钟域，与本次主动中断无关，原样保留。",
        "attribution_caveat": "两个 FAIL 均按时间线归因于「任务在到达 5 m/HOLD 之前被撤销」，注入前全部健康门为 PASS；这是基于已有证据的归因，不排除未记录因素，但无证据显示存在独立的控制或定位失效。按用户要求原样保留 FAIL，不改成 PASS。",
    },
    "overall_verdict": {
        "strict_verdict": "UNVERIFIED（按用户要求「证据缺失就明确 UNVERIFIED，不强判 C」：C9 的共享进程子项缺运行后独立证据，因此不宣称响应全部通过）",
        "classification": "B",
        "classification_note": "冻结规则只有 A/B/C 三类；若必须三选一，因信号已发送（排除 A）且 C1–C9 未全部确认成立（排除 C），归入 B。B 在此处是「未能全部确认成立」，不是「已证实响应失败」。",
        "not_C": True,
        "not_A": True,
        "classification_basis": "按冻结 verdict_rule：C1/C2 成立且 SIGTERM 已实际发送 → 不属 A；C9 因共享进程未被触碰缺运行后独立证据、判据点名的 preflight_orphans.json 缺失而判 UNVERIFIED，C1–C9 未全部确认成立 → 不判 C，归入 B。",
        "B_qualification": "此处 B 的含义是「注入已执行但未能全部确认成立」，不是「已证实响应失败」：C1–C8、C10 判据均有直接证据 PASS，C9 的清理主体（16/16 全退、verified_empty、clean=true、leftovers=0）亦为 PASS；仅 C9 的共享进程子项为 UNVERIFIED。",
        "no_response_pass_claim": True,
        "scope_limit": "本轮为助手误执行的额外 1 次运行，不构成用户授权的中断回归验收；旧 FIX20 中断轮仍为 A，不追溯改写。",
    },
    "launch_accounting": {
        "actual_new_simulation_launches": 1,
        "retry": False,
        "matching_run_dirs": A["launch_accounting"]["matching_run_dirs"],
        "ATTEMPTS_jsonl": A["launch_accounting"]["attempts_jsonl"],
        "run_json": A["launch_accounting"]["run_json"],
        "LAUNCH_ONCE_lock": B["launch_accounting_extra"]["LAUNCH_ONCE_lock"],
        "old_FIX20_intr_round": {"run_dir": "vio_closed_loop_intr_fix20_r1_20261115_233636",
            "classification": "A", "signal_sent": "no", "unchanged": True},
    },
    "provenance": {
        "criteria_md5_all_copies": A["criteria_md5"],
        "preflight_mismatch_items": A["provenance"]["preflight_checks_mismatch"],
        "launcher_md5_chain": A["provenance"]["launcher_md5_history"],
        "launcher_diff_scope": "diff_scene_comma.patch 仅把 scene JSON 的 lifecycle 行尾补 1 个逗号；before→frozen 的实际 diff 与补丁逐字节一致；运行体（scene 生成块之外）逐字节相同；修复后生产 launcher 与 launcher_record_frozen.sh md5 均为 c5e6ca0a84077ba416e0223bb0b090bd，scene_wide_fix19.json 可被 json.load 解析。",
        "frozen_files_claim": "19 个实物与 FREEZE_FIX20.json 冻结值一致；launcher 1 项为证据输出段差异（c217e669… → c5e6ca0a…），不宣称「全部冻结一致」，不改写历史冻结清单。",
        "ATTEMPTS_preflight_authorization_original": A["provenance"]["preflight_authorization_original"],
        "authorization_correction": "用户澄清：上一个助手被要求只核验、规划并提供提示词，未授权启动仿真；该次 WSL 中断仿真为助手误执行的额外 1 次，不计为用户明确新增授权，不改写旧 5 次记录。原文保留，另见 EXECUTION_SCOPE_CORRECTION.md。",
    },
    "defects_found": [
        {"id": "D1", "severity": "记录层", "detail": "interruption.log 的 inject_mono 用 date +%s.%N，实为 epoch，命名误导；与 inject_epoch 相差 0.006 s。"},
        {"id": "D2", "severity": "记录层", "detail": "bridge_owner.fix19.json 初始 child_parameter_bridge_pids=[] 与 loaded_ros_gz_bridge_so=[]，未记录两个真实桥子进程；须由 proc_registry/proc_watch 补齐。"},
        {"id": "D3", "severity": "记录层", "detail": "launcher_exit.json 的 node_exit_code=0 与 SIGTERM 常规退出码不一致（NODE_RC 初值 0 + wait 循环 break 语义），不能单独证明信号导致退出。"},
        {"id": "D4", "severity": "交付缺口", "detail": "本轮未生成 interruption_result.json、injection_verdict.txt、preflight_orphans.json；收尾 verdict 停在 interruption.log 的 PENDING。"},
        {"id": "D5", "severity": "交付缺口", "detail": "共享 MicroXRCEAgent 只有运行前 shared_before.json，无运行后存活快照，导致 C9 该子项 UNVERIFIED。"},
        {"id": "D6", "severity": "测量方法", "detail": "PUB_BEFORE/PUB_AFTER 用 grep -c 对带 flush 滞后的实时文件在不同时刻取行数快照，容易被误读为「注入后持续发布」；应以 vio_obs_counter 接收计数与最近接收时刻为准。"},
        {"id": "D7", "severity": "测量方法", "detail": "sleep 6.0 之后到实际发信号有 +0.476 s 工具链开销，DROP_AFTER=6.0 不是精确 +6.000 s；阶段时间戳未全部落盘。"},
    ],
    "minimal_fixes_proposed_not_applied": [
        "D1：把 inject_mono 改名为 inject_epoch_wall，或直接引用 injection_signal.json 的 sent_mono_us；不改判据。",
        "D2：桥就绪后（子进程已派生）二次采样，或在记录时按 PPID/会话枚举子进程，写入 child_parameter_bridge_pids。",
        "D3：额外落盘 node_signal=TERM、node_exit_status_raw、node_exit_reason，并关联 signal_sent 与 proc_gone 证据。",
        "D4：收尾阶段按冻结 C1–C10 生成 interruption_result.json 与 injection_verdict.txt，把 verdict 从 PENDING 落定。",
        "D5：收尾时写 shared_after.json（含 3643 的存活与 /proc stat），闭合 C9。",
        "D6：改用 vio_obs_counter.csv 的 odo_count/health_count/last_rx 在同一单调时刻前后取样，publish 行数仅作辅助。",
        "D7：为条件检查、就绪复核、身份核验、实际发送各写一条 CLOCK_MONOTONIC 时间戳；不调整 DROP_AFTER=6.0 既定判据。",
    ],
    "follow_up_prompt": "见 INTERRUPTION_AUDIT_REPORT.md §7；任何重跑均需用户新的明确授权，且本次越权额外 1 次不计入额度。",
    "unchanged_boundaries": [
        "原场地(rm27_world 原出生点)可靠性：UNVERIFIED",
        "额外延迟性能达标：UNVERIFIED（latency_evidence 仍 UNVERIFIED）",
        "Nano 实机放行：UNVERIFIED",
        "全部故障类型：未验证",
        "旧 ACC03 3/3 属 FIX17/FIX20 原冻结运行环境，不改写为本次结果",
        "旧 FIX20 中断轮（intr_fix20_r1_20261115_233636）保持 A：注入器缺陷未发送信号",
    ],
    " files_in_audit_dir": ["analysis_raw.json", "analysis_detail.json", "analyze_offline.py", "analyze_detail.py"],
}
audit["analysis_scripts"] = audit.pop(" files_in_audit_dir")

with open(OUT, "w", encoding="utf-8") as f:
    json.dump(audit, f, ensure_ascii=False, indent=2)
print("WROTE", OUT)
print("classification =", audit["overall_verdict"]["classification"])
print("criteria verdicts =", [(c["id"], c["verdict"]) for c in criteria])
print("timeline events =", len(timeline))
