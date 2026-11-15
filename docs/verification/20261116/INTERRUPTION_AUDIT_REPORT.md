# NP-01 FIX20 中断回归 · 离线审计报告

日期：2026-11-16（GMT+8）
审计性质：**纯离线只读**。未启动任何新仿真或飞行、未发送任何信号、未修改任何生产文件（飞手/算法/配置/门限/场景/评估器/冻结判据）、未操作 Nano、未推送 GitHub。保留全部原始文件。
审计对象：`/home/gengyinghao/drone_sim/logs/vio_closed_loop_intr_fix20_resume_20261116_r1_20261116_000320`
准备记录：`C:/Users/46292/Desktop/飞机/intr_resume_20261116`（WSL：`/mnt/c/Users/46292/Desktop/飞机/intr_resume_20261116`）
冻结判据：`newpkg_fix19/FROZEN_INTERRUPTION_CRITERIA.json`，md5 `6e5adef1bf208011f5979885b1b53a62` —— 工作包原件、准备记录副本、本地 `newpkg_fix19` 副本三处一致，**原样沿用 C1–C10 与 A/B/C 规则，未改动**。

---

## 0. 专项结论（先给结论）

**按冻结规则分类：B（注入已执行，但响应未获完整通过）；严格口径为 UNVERIFIED，不判 C。**

- **不属 A**：C1/C2 成立，`injection_signal.json` 记录 `result=sent`、SIGTERM 已发给唯一节点 PID 337397，节点随后从 `/proc` 消失。
- **不属 C**：C9 的「共享 MicroXRCEAgent 未被触碰」子项缺运行后独立证据（无 `shared_after.json`，判据点名的 `preflight_orphans.json` 本轮未生成，离线审计时 `/proc/3643` 已不存在），判 UNVERIFIED。按用户「证据缺失就明确 UNVERIFIED，不强判 C」的要求，**不宣称响应全部通过**。
- **B 在此处的含义**是「C1–C9 未能全部确认成立」，**不是**「已证实响应失败」：C1–C8、C10 均有直接证据 PASS，C9 的清理主体（16/16 目标全退、`verified_empty=true`、`clean=true`、`leftovers=0`、清理目标不含 3643）亦为 PASS。
- **普通任务评估器**：`VERDICT=FAIL 14 PASS / 2 FAIL / 1 UNVERIFIED` **原样保留**，不改成 PASS；专项结论独立单列。
- **执行范围**：本次实际新增启动 **1 次，无重试**。该次运行是**助手误执行的额外 1 次**，用户当时的要求只是核验、规划并提供提示词，未授权启动仿真；**不计为用户明确新增授权**，不改写旧 5 次记录。旧 FIX20 中断轮 `intr_fix20_r1_20261115_233636` 保持 **A（注入器缺陷未发信号）**，不追溯改写。详见 `EXECUTION_SCOPE_CORRECTION.md`。

---

## 1. 来源与变更核实

### 1.1 冻结判据
三份副本 md5 均为 `6e5adef1bf208011f5979885b1b53a62`，与用户声明一致；C1–C10 及 `verdict_rule` 未改动。

### 1.2 19 个实物一致 + launcher 1 项差异（独立验证，不宣称全部一致）
`preflight.json` 的 20 项检查中，19 项 `expected == actual == frozen_actual`；唯一 `match=false` 的是 `launch_run_wide_fix19.launcher.sh`（`recording_only_exception=true`）。

独立复核 launcher 三方 md5：

| 版本 | md5 |
|---|---|
| `FREEZE_FIX20.json` 期望值 | `c217e669d448984030494a1ffac107f7` |
| `launcher_before_scene_comma.sh`（补逗号前） | `c4521ddc84650ab73ebe1339a666134e` |
| `launcher_record_frozen.sh`（补逗号后） | `c5e6ca0a84077ba416e0223bb0b090bd` |
| WSL 生产文件 `launch_run_wide_fix19.sh` | `c5e6ca0a84077ba416e0223bb0b090bd` |

`diff -u` 的实际差异与 `diff_scene_comma.patch` 逐字节一致：**只有一处**，即 scene JSON 生成块里 `lifecycle` 行尾补 1 个逗号（`false}` → `false},`），使原本非法的 JSON 可解析。`fix_scene_record.py` 的断言保证 scene 生成块之外的运行体与 FIX20 冻结版逐字节相同；修复后生产文件与 `launcher_record_frozen.sh` 完全一致，本轮 `scene_wide_fix19.json` 可被 `json.load` 正常解析。**结论：差异只在证据输出段，未触及运行逻辑、启动顺序、话题、QoS、控制或清理归属。**

按用户要求：不改写 `FREEZE_FIX20.json` 历史值，本次实际指纹单独记录于 `interruption_audit.json → provenance.launcher_md5_chain`。**不宣称「20 个实物全部与旧冻结值一致」。**

### 1.3 ATTEMPTS / preflight 中的授权解释（保留原文，另追加更正）
`preflight.json` 的 `authorization` 原文为 `User: 继续任务; one continuation interruption simulation, no retry`；`ATTEMPTS.jsonl` 使用批次 `NP01_FIX20_INTR_RESUME_20261116`。该解释**不符合用户澄清**：用户当时只要求核验、规划与提示词，未授权启动仿真。两处原文**均保留不改**，更正记录单列于 `EXECUTION_SCOPE_CORRECTION.md`。

---

## 2. C1–C10 逐项判定

| 项 | 判定 | 核心证据（文件 / 行号或时间范围） | 关键数值 |
|---|---|---|---|
| **C1** 注入前置条件 | **PASS** | `pilot.log:27`；`pilot.csv` NAV+5.5..6.5 s 的 17 行；`pilot.csv.events.json` seq1–4；`check.txt:6,9,18`；`gate_snapshot.csv` 837 行 | NAVIGATE `1794758686.324199`；+6 s 前后 17 行全为 `state=NAVIGATE, bt_phase=NAVIGATE, task_completed=False, control_active=True, bt_reason=pilot_authorized`；无 task_completed 事件、无 FAULT、GATE_FAULT=0；task_revoked 在 NAV+7.105 s |
| **C2** 唯一节点身份 | **PASS** | `node_pid.json`；`injection_readiness.json`（candidate_pids/problems/excluded）；`injection_target_identity.json`；`proc_registry.jsonl`；`preflight_output.txt` S2 自检 | 唯一候选 PID **337397**，`starttime_s_boot=4990088`，exe/sha256 与冻结值一致（md5 `bfa646c8…`、sha256 `0a1045f4…`）；`ready=true`、`problems=[]`、`excluded_count=15`；launcher 337312 与桥 337152/337238 显式排除；S2 替身自检验证 runner 排除。exe 字段注明「path only; binary contents NEVER read」 |
| **C3** 信号 | **PASS** | `injection_signal.json`；`interruption.log:5-12`；`INJECTION_MARKER`（0 字节）；`injector_frozen.sh:58-59,90-95,106-133,158`；`run.log:20-24`；`preflight_output.txt` S6/S7 | SIGTERM 只发给 337397（expect-exe + expect-starttime 双校验均 true），`result=sent`、`signal_rc=0`；`requested_mono_us=49966440720` → `sent_mono_us=49966440807`（`CLOCK_MONOTONIC`，差 **87 µs**）。注入器 `setsid` + `wait $L`，整轮清理由 run_profile 在 DONE 后触发（00:05:05.585），早于注入器 00:05:07.927 退出 |
| **C4** 进程实际退出 | **PASS** | `launcher_exit.json`；`proc_watch.csv` role=node；`interruption.log:10`；launcher 源码 170-186、342-348 | `node_pid=337397`、`node_exit_code=0`、`launcher_rc=0`；proc_watch 在信号后 **+0.376290 s** 采样已 `alive=0`，此后无 alive 行；`node_exited_within_5s=1` |
| **C5** 消息通路停止 | **PASS** | `vio_obs_counter.csv`（154 行，信号后 27 行）；`bridge_telemetry.csv`（460 行 publish，seq452–460）；`runtime_status.json`；`injector_frozen.sh:126-127 vs 142` | odo 冻结 **460**、health 冻结 **315**，最近接收冻结在 +0.018187 s / +0.017260 s，信号后 27 个采样（+0.1186 s 至 +13.19 s）**零增长**；publish 最终停在 460、收尾 `bridge_publish_rows=460`、无 461。452→460 的 8 行中 7 行 system_us 在信号前，仅 seq460 在 +0.017829 s 且其 `sample_us` 在信号前 0.071052 s |
| **C6** 授权撤销与任务停止 | **PASS** | `bt.log:14`；`pilot.log:30-32`；`pilot.csv.events.json` seq4；`pilot.csv` 信号后行 | 撤销原因 **stale_health**，bt.log 在 **+0.6186 s**、BT EVENT 在 **+0.6287 s**；pilot 于 +0.6231 s 进入 TAKEOVER；此后 `control_active=False` 189 行，NAVIGATE 仅剩 11 行尾部采样 |
| **C7** 桥/监督器/真值/遥测持续 | **PASS** | `bridge_owner.fix19.json`（初始 child pids 为空）；`proc_registry.jsonl` role=bridge；`proc_watch.csv`；`supervisor_trace.csv`（3242 行，单调域）；`supervisor.json`；`truth.csv`/`telemetry.csv` | 桥真实两 PID **337152**（ros2 run 包装）与 **337238**（parameter_bridge 本体）、监督器 **337734**，信号后各 13 个 alive 采样，存活至 **+12.39 s**；监督轨迹信号后新增 **643 行**、末行 +12.818 s；truth 4128 行至 +13.194 s、telemetry 763 行至 +12.865 s，覆盖清理开始 +12.827 s；`lost_us_total=0`、`warned/triggered=false`、`stop_event=null`、trace action 全空，无 truth_input_lost |
| **C8** 接回与落地 | **PASS** | `pilot.log:31-35`；`exit_grade.json`；`check.txt:15` | TAKEOVER(+0.623 s)→LAND(+2.625 s)→LANDED(+9.126 s)→DONE(+11.127 s)；`pilot_final_armed=false`、`landing_confirmed=true`、真值高度残差 0.00 m、`landed_disarmed=PASS` |
| **C9** 整轮清理 | **UNVERIFIED** | `process_cleanup.json`；`cleanup_verify.json`；`shared_before.json`；`preflight_reclaim_*.json`；**缺失** `preflight_orphans.json` / `shared_after.json` | 清理主体 PASS：16 目标全 `exited=true`、`leftovers=[]`、`verified_empty=true`、`cleanup_verify.clean=true`、`alive_members=[]`，规则为「registry 身份(pid+starttime) 且 sid==336330，无 pkill」，目标清单不含 3643，预检 reclaim 命中 0。共享进程子项 UNVERIFIED：只有运行前快照，无运行后存活证据；离线审计时 `/proc/3643` 已不存在 |
| **C10** 原始结果与专项结论分离 | **PASS** | `check.txt:23`；`check.json`；本审计交付；**缺失** `interruption_result.json` / `injection_verdict.txt` | `VERDICT=FAIL counts={'PASS':14,'FAIL':2,'UNVERIFIED':1}` 原样保留；两个 FAIL 及其 detail 未改；专项结论单列。C10 属交付约束，不参与 A/B/C 判定 |

**C1–C10 判定汇总：9 PASS（C1–C8、C10）/ 0 FAIL / 1 UNVERIFIED（C9）。**

---

## 3. 关键响应时间线

基准 `SIG_W = 1794758692800393`（`injection_signal.json.requested_wall_us`，epoch µs）；`NAVIGATE = 1794758686.324199`。

| 事件 | 相对信号 s | 证据 |
|---|---|---|
| state → NAVIGATE | −6.476194 | `pilot.log:27` |
| 就绪复核 `checked_wall_us` | −0.336216 | `injection_readiness.json`（NAV+6.140 s） |
| `interruption.log inject_epoch`（实为 epoch） | −0.207080 | 脚本 `date +%s.%N` |
| 身份核验 `recorded_wall_us` | −0.130637 | `injection_target_identity.json`（NAV+6.346 s） |
| **SIGTERM requested / sent** | **0 / +0.000087** | `injection_signal.json`，NAV+6.476194 s |
| 最后 odo / health 接收 | +0.018187 / +0.017260 | `vio_obs_counter.csv` |
| publish seq460 落盘 | +0.017829 | `bridge_telemetry.csv`（源采样在信号前 0.071052 s） |
| odo/health 计数首次冻结采样 | +0.118600 | `vio_obs_counter.csv` |
| 节点 337397 首个 dead 采样 | +0.376290 | `proc_watch.csv`（1 s 采样间隔） |
| BT Task revoked: stale_health | +0.618603 | `bt.log:14` |
| state → TAKEOVER | +0.623130 | `pilot.log:31` |
| BT EVENT task_revoked | +0.628676 | `pilot.csv.events.json` seq4 |
| state → LAND / LANDED / DONE | +2.624836 / +9.125926 / +11.126775 | `pilot.log:33-35` |
| 桥/监督器最后 alive 采样 | +12.391667 | `proc_watch.csv` |
| supervisor_trace 末行（mono 映射） | +12.818092 | `supervisor_trace.csv` |
| 清理开始 / 完成 | +12.827082 / +14.278396 | `run.log:22`、`process_cleanup.json` |
| truth.csv / telemetry.csv 末行 | +13.193895 / +12.864598 | 两个 CSV |

**注入时刻偏差（按用户要求报告，不调整判据）**：`DROP_AFTER=6.0` 不是精确 +6.000 s。实际发送在 **NAV+6.476194 s**，偏差 **+0.476194 s**，来自条件检查（`STATE_AT`/`count_matches`，无独立时间戳，介于 +6.0 与 +6.140 s）、就绪复核（+6.140 s）、身份核验（+6.346 s）、`PUB_BEFORE` grep 与 `signal-pid` 调用开销。**既定 C1 判据未作任何调整**；在 +6.0 s 与实际 +6.476 s 两个时点，pilot 均处于 NAVIGATE 自主执行段。

**时钟域说明（按要求先核对源码，禁止混算）**：
- `interruption.log` 的 `inject_mono=1794758692.587296955` 与 `inject_epoch=1794758692.593313497` **都是 `date +%s.%N` 的 epoch**，相差 0.006 s，不是单调时间。可靠的单调时间是 `injection_signal.json` 的 `requested_mono_us`/`sent_mono_us`（`np01_lifecycle.py:57-58,434,453` 的 `time.monotonic_ns()//1000`）。
- `supervisor_trace.csv` 的列名 `t_wall_us` **实为 `CLOCK_MONOTONIC` µs**（`np01_truth_supervisor.py:57,133,202`），值域 `49915167409..49979258899`。已用同一单调域与 `sent_mono_us=49966440807` 比较（末行差 +12.818 s，信号后 643 行），并用 `epoch−mono=1794708726359673 µs` 映射（`telemetry.csv` 中位数 `…359692`，差 19 µs）交叉验证，**未与 epoch 直接相减**。
- `proc_watch.csv` 的 `t_wall_us` 是 epoch 域；`vio_obs_counter.csv` 的 `t_wall_us` 是 `time.time()*1e6`（epoch），`t_mono_s` 是相对单调秒。三者分别处理。

---

## 4. 两个普通 FAIL 的具体解释（原样保留，不预设是中断的合理结果）

原始输出（`check.txt:12-13,23`）：

```
[FAIL] bt_task_completed   phases={'WAIT_PILOT': 1148, 'NAVIGATE': 141, 'REVOKED': 1} 到达过 HOLD=False 目标距离=5.00（指令 5.00 m）
[FAIL] truth_distance_hold 真值最大水平位移=1.65 m（指令 5.00±0.60），在 5.00±0.50 m 内最长保持=0.0s（要求>=2.0s）
VERDICT=FAIL  counts={'PASS': 14, 'FAIL': 2, 'UNVERIFIED': 1}
```

**`bt_task_completed`**：撤销事件使 BT phases 停在 `NAVIGATE`/`REVOKED`，从未进入 `HOLD`。目标距离字段为 **5.00 m，与指令一致**，说明不是配置错误或导航目标错误。对照 FIX20 正常轮：需要 NAV+22.9 s 才 `task_completed`（`pilot.log`），本轮在 NAV+7.105 s（信号后 0.629 s）已撤销，任务窗口只有 7.1 s（正常轮 31.1 s）。注入前的健康门全部 PASS：`offboard_accepted`（122 行 NAVIGATE、71 行 DDS nav_state=14）、`nav_ready_continuous`（覆盖率 100%）、`ekf_using_ev`（97%）、`vio_vs_truth`（RMSE 0.0074 m）、`vio_tracking_health`（丢失帧 0%）、`gate_attitude_continuity`（GATE_FAULT=0）、`no_airborne_fault`（0）。

**`truth_distance_hold`**：按 `check_run.py:398-426` 的口径独立重算（锚点取任务开始最近邻真值帧 `1794758686374468`，窗口 `m0−5s..m1+2s` 共 713 行）：`dmax=1.649 m`，与评估器 1.65 m 一致。分阶段位移：NAVIGATE 0.0 m → **信号时 1.318 m** → 撤销时 1.476 m → 窗口末 1.481 m，峰值 1.649 m（撤销后 102 行仍有约 0.17 m 惯性位移，属接回减速过程，不是失控）。飞机从未接近 5.00±0.60 m，保持 0.0 s。对照正常轮 `dmax=5.04 m / 保持 14.0 s`。

**归因边界**：两个 FAIL 均按时间线归因于「任务在到达 5 m / HOLD 之前被 `stale_health` 撤销」，且注入前全部健康门为 PASS、无独立控制或定位失效证据。这是基于已有证据的归因，**不是绝对排除未记录因素**；按用户要求原样保留 FAIL，不改成 PASS，专项结论另在 §0 单列。

**1 项 UNVERIFIED**：`latency_evidence` 是历史遗留（`latency.csv` 无 `age_ms` 列、字段分属 sim/epoch/mono 三时钟域），**与本次主动中断无关**，原样保留。

---

## 5. 证据局限与已发现缺陷（只给最小修复方案，未修改生产文件）

### 5.1 使 C9 无法判定的证据缺口
- 无 `shared_after.json`；冻结判据 C9 evidence 点名的 `preflight_orphans.json` 本轮未生成。
- 离线审计时 `/proc/3643` 已不存在 —— 既不能证明「被本轮清理」，也不能证明「未被触碰后仍存活」，两种可能（会话结束后自然退出 / 其他任务清理）均无证据排除。
- 本轮收尾未落盘 `interruption_result.json`、`injection_verdict.txt`，`interruption.log` 的 verdict 停在 `PENDING`；专项结论由本次审计补出。

### 5.2 记录/测量层缺陷清单（均不影响本次判据主证据，但需登记）

| # | 缺陷 | 最小修复方案（**未实施**） |
|---|---|---|
| D1 | `interruption.log` 的 `inject_mono` 用 `date +%s.%N`，实为 epoch，命名误导 | 改名为 `inject_epoch_wall`，或直接引用 `injection_signal.json` 的 `sent_mono_us`；不改判据 |
| D2 | `bridge_owner.fix19.json` 初始 `child_parameter_bridge_pids=[]`、`loaded_ros_gz_bridge_so=[]`，未记录两个真实桥子进程 | 桥就绪后二次采样，或按 PPID/会话枚举子进程写入；本次已用 `proc_registry.jsonl`+`proc_watch.csv` 补齐证据 |
| D3 | `launcher_exit.json` 的 `node_exit_code=0` 与 SIGTERM 常规退出码（143/-15）不一致（源码 `NODE_RC` 初值 0 且 `wait` 循环在 rc≤128 时 break） | 额外落盘 `node_signal=TERM`、`node_exit_status_raw`、`node_exit_reason`，关联 `signal_sent` 与 `proc_gone` |
| D4 | 缺 `interruption_result.json` / `injection_verdict.txt` / `preflight_orphans.json` | 收尾按冻结 C1–C10 生成专项结果文件与孤儿核验快照，把 verdict 从 PENDING 落定 |
| D5 | 共享进程只有运行前快照 | 收尾写 `shared_after.json`（含 3643 存活与 `/proc` stat），闭合 C9 |
| D6 | `PUB_BEFORE/PUB_AFTER` 用 `grep -c` 对带 flush 滞后的实时文件在不同时刻取行数快照，易被误读为「注入后持续发布」 | 改用 `vio_obs_counter.csv` 的接收计数与最近接收时刻，在同一单调时刻前后取样；publish 行数仅作辅助 |
| D7 | `sleep 6.0` 到实际发信号有 +0.476 s 工具链开销，阶段时间戳未全部落盘 | 为条件检查、就绪复核、身份核验、实际发送各写一条 `CLOCK_MONOTONIC` 时间戳；**不调整 `DROP_AFTER=6.0` 既定判据** |

### 5.3 其他须登记的边界
- `INJECTION_MARKER` 为 0 字节文件，其存在只证明写入，不单独证明进程死亡或消息停止；本次进程与消息流分别由 `proc_watch.csv` 和 `vio_obs_counter.csv` 取证。
- 文件 mtime 仅作辅助，未用于任何判定。
- `exit_grade.json` 的 `grade=A / subtype=A_early` 是**退出分级**（safe exit 语义），**不是**冻结判据的 A/B/C 专项分类，未混用。
- `proc_watch.csv` 为 1 s 采样，节点消失只能界定在 +0.376 s 采样点之前，无法给出毫秒级时刻。
- C2 的运行期排除清单没有名为 `role=runner` 的独立条目（runner 是会话首领，未登记为 registry 角色）；其排除依据为唯一 exe 匹配 + 起飞前 S2 替身自检 + 会话隔离，已如实标注。

---

## 6. 执行范围与未验证边界

- 本次实际新增启动 **1 次**（`run.json: actual_simulation_launches=1`；`ATTEMPTS.jsonl` 只有 `launch_started` 与 `injector_finished` 两条；匹配运行目录仅 1 个；`LAUNCH_ONCE.lock` 存在），**无重试**。
- 该次为**助手误执行的额外 1 次**，不计为用户明确新增授权；旧 5 次记录不改动；旧 FIX20 中断轮保持 A。详见 `EXECUTION_SCOPE_CORRECTION.md`。
- 本次审计未启动新仿真/飞行、未发信号、未操作 Nano、未推送 GitHub、未修改生产文件。
- **继续保持 UNVERIFIED**：原场地（`rm27_world` 原出生点）可靠性、额外延迟性能达标（`latency_evidence` 仍 UNVERIFIED）、Nano 实机放行、全部故障类型。旧 ACC03 3/3 属 FIX17/FIX20 原冻结运行环境，不改写为本次结果。

---

## 7. 后续操作提示词（供用户决定是否新授权，本次不执行）

> 明确授权在**记录层修订版**（不修改飞手/算法/配置/门限/场景/评估器/冻结判据）下，重跑 1 次真实 VIO 中断回归，用于闭合 FIX20 离线审计的 C9 证据缺口。
> 要求：起飞前冻结 C1–C10 与 A/B/C 规则（md5 沿用 `6e5adef1…`）；沿用 NAVIGATE 后 6.0 s 且仍处预定自主执行段的固定注入事件，不因观察结果提前/推迟；SIGTERM 只发给按 PID+starttime+exe+完整 sha256 确认的唯一 VIO 节点；注入器退出不触发整轮无条件 cleanup。
> 收尾必须落盘：`interruption_result.json`、`injection_verdict.txt`、`preflight_orphans.json`、`shared_after.json`（含共享 MicroXRCEAgent 的存活与 `/proc` stat）、`node_exit_status_raw`/`node_exit_reason`、各阶段 `CLOCK_MONOTONIC` 时间戳；`bridge_owner` 补齐两个真实桥子进程 PID；`inject_mono` 改名为 `inject_epoch_wall` 或直接引用 `sent_mono_us`；publish 计数改用 `vio_obs_counter` 接收计数与最近接收时刻。
> 保留普通评估器原始结果，专项结论单列；若未达注入点则不发信号、记为注入前失败、不自动重飞。本提示词本身不构成授权；需用户明确确认额度与执行范围后方可运行。

---

## 8. 交付文件（本次独立目录）

| 文件 | 路径 |
|---|---|
| 机器可读逐项结论 | `C:/Users/46292/Desktop/飞机/intr_resume_20261116/interruption_audit.json` |
| 本报告 | `C:/Users/46292/Desktop/飞机/intr_resume_20261116/INTERRUPTION_AUDIT_REPORT.md` |
| 执行范围更正记录 | `C:/Users/46292/Desktop/飞机/intr_resume_20261116/EXECUTION_SCOPE_CORRECTION.md` |
| 离线分析原始数值 | `…/intr_resume_20261116/audit_offline_20261116/analysis_raw.json`、`analysis_detail.json` |
| 可复现脚本（只读） | `…/audit_offline_20261116/analyze_offline.py`、`analyze_detail.py`、`build_audit_json.py` |

原始运行证据全部保留在 `/home/gengyinghao/drone_sim/logs/vio_closed_loop_intr_fix20_resume_20261116_r1_20261116_000320/`，未改动。
