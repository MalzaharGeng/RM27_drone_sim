# NP-01 FIX20 报告：EXCITE 高度参考 reset 修复、生产路径测试、正常回归、新版本 3 次冷启动与中断验证

日期：2026-11-15 23:06–23:50（WSL；未写 Nano、未推 GitHub）
本轮授权范围：修复及最多 **5 次实际飞行**（正常 1 + 冷启动 3 + 保留的中断 1）；
离线分析/测试/记录修复不占额度。

---

## 0. 验证结果

| 阶段 | 结果 |
|---|---|
| 最小修复 + 回滚入口 | 完成，diff **+79/−0**（纯增量） |
| 生产路径测试 | **11/11**；现有状态机/输入回归 **9/9** |
| 正常 5 m 回归 | **PASS 16/0/1**，`A_complete`，首次 EV 高度融合 reset 正确平移 |
| 新版本独立冷启动批次 `NP01_FIX20_COLDSTART_3X` | **3/3 PASS**（每轮 16/0/1、`A_complete`） |
| 中断回归 | **A：注入未执行** —— 注入器工具缺陷导致漏发信号，非飞行失败 |
| 实际飞行 | **5/5**（用满授权上限），不自动重飞、不追加中断额度 |
| 未改动 | `excite_max_dz`、Gate/LIMITS、初始化要求、场景、激励输入、评估器 |

中断响应仍为 UNVERIFIED。C1–C10 专项判据下本轮判为 **A（注入未执行）**。
根因是注入器条件判断的一个 shell 字符串比较缺陷（`grep -c` 无匹配时返回码 1，
`|| echo 0` 又追加一行 → 变量变成 `0\n0`，与 `0` 恒不等），离线证明
NAVIGATE+6 s 时注入条件**实际满足**。该缺陷已定位并给出修复（见 §6），
但按「任一阶段失败停止后续飞行、不自动修改后再补飞凑满成功次数」，
**本轮没有重飞**。

---

## 1. reset 符号、一次性应用与异常处理契约

PX4 `VehicleLocalPosition.msg`（`~/px4_ws_201/src/px4_msgs/msg/`）定义：
`z` = NED Down position；`delta_z` = "Amount of vertical shift of position estimate
in latest reset [m]"；`z_reset_counter` / `vz_reset_counter` = uint8。

**ULog 实测符号**（`offline/sign_contract.json`）：

| | `z` 阶跃 | `delta_z` | 残差（真实运动） |
|---|---|---|---|
| flight1 | +0.180943 | +0.180837 | 0.000106 m |
| flight2 | −1.157596 | −1.155577 | 0.002018 m |

即 `z_after = z_before + delta_z + 真实运动` → **参考必须 `ref_z += delta_z`（同号）**，
绝不能 `ref_z = 当前 z`（会抹掉真实累计高度偏移）。`delta_z` 是 **sticky**：
ULog 显示 reset 后连续 12 帧值完全相同，所以只能按 counter 新事件触发一次。

契约（`newpkg_fix20/excite_height_reference.py`，纯函数、不读真值/时钟）：

1. **首帧同时捕获**：进入 EXCITE 捕获 `ref_z` 时一并捕获 `zrc/vzrc/sample_us`，
   防止进入前已存在的历史 sticky delta 被误用。
2. **可确认的新事件**：`(zrc_new − zrc_old) % 256 == 1`。uint8 回绕 255→0 合法。
3. **一次性**：step==0（含相同 counter 反复到达）→ 不平移；step==1 → 平移一次。
4. **失败关闭**：step 为 2..255（跨多事件/丢失中间 delta/回退）、delta 非有限 /
   为零 / 超出健全性界（沿用 takeoff 契约 `max_abs_delta_z_m=2.0`，这是输入健全性界，
   不是控制门限）、样本缺失、`z_valid=False`、反馈陈旧/时钟异常、
   `sample_us` 倒退（乱序）→ 明确 `height_reset_unrecoverable:<reason>` 并 fatal 退出，
   **不猜测、不拿最后一个 delta 代表全部 reset**。
5. **per-tick 影响面**：reset 当拍只暂缓「高度位移」这一项判定；输入健康、角速度、
   倾角、水平位移、VIO 就绪、监督器、授权、硬上限等全部继续生效；
   下一有效样本恢复正常高度包络检查。
6. **不重置状态**：不延长初始化时间、不清除安全故障、不重新开始包络、
   不重置 `disp_xy` 参考、不重置 ready streak。

**保持未改**：`excite_max_dz=1.00`、Gate/LIMITS、初始化要求、`rm27_wide_world`、
激励波形/输入、`check_run.py` / `phase4_eval.py`、冻结算法节点与 `libORB_SLAM3.so`。
`sim_pilot.py` diff 为 **+79/−0**，只新增 import、3 个状态字段、事件落盘 helper、
捕获时的 counter 记录、(3b) reset 推进、以及 delta_z 判定上的一个 `not suspend` 条件。

---

## 2. 生产路径测试与历史回放

`tests/test_excite_height_production.py` 用 `object.__new__(sim_pilot.Pilot)`
构造**真实生产对象**，把 flight1/flight2 的 `pilot.csv` 逐帧灌成 `HeightSample`，
调用真正的 `bounded_excite_decision()`。测试导入的是候选版
（`candidate/sim_pilot.py`，md5 与部署一致），不是公式替身。

| # | 判据 | 结果 |
|---|---|---|
| 1 | 旧实现复现 flight2 的 1.1929 m | **1.1929**，首个越界 t=14.406 s |
| 2 | 正确平移后约 0.0373 m | **0.0373**，平移 `ref −1.0125 → −2.1681`（delta −1.1556），`zrc 1→2`，`vzrc 1→2`，无 fatal |
| 3 | flight1 原正常流程不变 | `vio_ready_stable`（fatal=False），平移 +0.1808 一次，max 0.1117 |
| 4 | sticky delta 只应用一次 | 后续 20 帧不再平移，`ref` 恒为 −2.1681 |
| 5 | 真实超过 1 m 的高度变化仍触发 | `delta_z(1.20>1.00m)` |
| 6 | reset 与真实运动同时发生，运动不被抹掉 | 真实 0.30 m 保留；再爬升 1.10 m 触发 |
| 7 | 缺失/多次/非法 reset 不静默放行 | 16 例边界矩阵全部符合预期 |

边界矩阵覆盖：无 reset 正常变化、正/负 reset、相同 counter 重复、uint8 回绕
255→0、跨 2 步/回退、delta NaN/Inf/零/超界、反馈陈旧、乱序、`z_valid=False`、
参考未捕获、包络边界附近 reset、reset 当拍角速度/倾角仍触发、下一拍恢复、
生产路径失败关闭。**现有回归 9/9 通过**（飞手回中、起飞高度策略、状态机回放、
预检门、导航入口、交接、权限、启动等）。

修改前保存：`prechange/` 5 个源码、`candidate/` 候选实物、
`evidence/diff_sim_pilot_fix20.patch` 独立 diff、`ROLLBACK.sh` 回滚入口。

---

## 3. 正常 5 m 回归（第 1 次飞行）

运行 `vio_closed_loop_fix20_func5m_20261115_232210`，
**PASS 16/0/1**（唯一 UNVERIFIED 是历史 `latency_evidence`），`A_complete`。

- **首次 EV 高度融合 reset**：事件文件 `excite_reset_events.jsonl` 落盘
  `reference_captured` `ref_z=−0.9663, zrc=1, vzrc=1` →
  `reference_translated` `delta_z=+0.235438, ref_z −0.9663 → −0.7309, zrc 1→2, vzrc 1→2`。
  差值 `−0.7308909297 − (−0.9663293958) = +0.235438` 与 delta 完全一致，**只应用一次**。
- **实际高度与包络连续性**：EXCITE 全程最大相对高度 **0.1168 m**（门限 1.00）；
  判定帧 `INIT_SETTLE / vio_ready_stable` 相对高度 **0.042 m**；
  真值高度范围 1.0515→1.1683 m（变化 0.117 m），与估计一致，无 reset 故障。
- **链路**：RMSE **0.0141 m**、最大 0.0249 m；跟踪丢失 0.0%、GATE_FAULT=0、
  `ekf_using_ev` 100/100/100%；`nav_ready` 61.90 s 连续；
  发布→EV→ready→POSCTL 实际确认；NAVIGATE / 5.04 m / HOLD 14.0 s /
  TAKEOVER / LAND / LANDED / DONE / `armed=False`，高度残差 0.00 m；无碰撞。
- **进程**：桥 pid 303053（runner 拥有）、监督器 pid 303511；
  `bridge_owner.fix19.json` / `supervisor_pid.fix19.json` 正确生成（新记录修订版）；
  `process_cleanup.json` 16 目标、全部退出、`verified_empty=true`、残留 0。

发现并修掉两个**证据层**瑕疵（不改运行参数）：FIX19 launcher 的 scene JSON
三处多余逗号导致无效、两个 md5 路径指向错误目录。备份与 diff 见
`prechange/launch_run_wide_fix19.before_scene_fix.sh`、
`evidence/diff_scene_evidence_fix.patch`。

---

## 4. 新版本独立冷启动批次 `NP01_FIX20_COLDSTART_3X`

`FREEZE_FIX20.json` 冻结 20 个实物（源码/配置/场景/运行环境/二进制），
**新批次编号**，不覆盖旧 ACC03。连续 3 次独立 PX4/gz/VIO 冷启动，
所有实际启动计入 `flight/COLDSTART_3X_ATTEMPTS.jsonl`，不筛失败。

| 轮次 | 运行目录 | VERDICT | exit | reset delta_z | 清理 |
|---|---|---|---|---|---|
| r1 | `..._232720` | PASS 16/0/1 | A_complete | **−0.369472** | verified_empty |
| r2 | `..._232943` | PASS 16/0/1 | A_complete | **−0.709580** | verified_empty |
| r3 | `..._233212` | PASS 16/0/1 | A_complete | **−0.475083** | verified_empty |

批次判定严格按 `check.json` 自己的 `required` 字段：3 轮必需项全部 PASS，
`time_sync_profile` / `latency_evidence` 为 `required=false`，不否决。
**3/3**，每轮都发生了一次真实的 EV 高度融合 z-reset，符号与幅度各不相同
（正常轮为 +0.2354），平移均只应用一次，无失败关闭事件。

---

## 5. 中断回归（保留的 1 次额度）—— **A：注入未执行**

运行 `vio_closed_loop_intr_fix20_r1_20261115_233636`，起飞前注入器
`--preflight` **7/7** 通过（唯一节点按 exe+完整 sha256 识别、
PID/starttime/exe/sha256 一致、launcher/运行器/桥显式排除、篡改拒绝、
注入器退出不触发整轮 cleanup），冻结判据 `FROZEN_INTERRUPTION_CRITERIA.json`
md5 `6e5adef1…` 未改。

**发生了什么**：注入器在 NAVIGATE 后 6.0 s 判定
`INJECTION_CONDITION_NOT_MET`，**没有发送任何信号**，按保护退出；
本轮飞行本身正常完成任务（PASS 16/0/1、`A_complete`、正常降落 disarm、
清理 16 目标 verified_empty）。`interruption.log`、`injection_signal.json`、
`INJECTION_MARKER`、`injection_readiness.json` **均不存在**。

**离线定位**（`evidence/intr_precise_diagnosis.json`）：

- 注入器第 87–88 行 `DONE_FLAG=$(grep -ac "task_completed" ... || echo 0)`：
  `grep -c` 无匹配时打印 `0` 但退出码为 1，`|| echo 0` **又打印一行**，
  变量变成 `'0\n0'`，与 `'0'` 比较恒不等 → 误判为「不在自主执行段」。
- **注入条件实际满足**：NAVIGATE+6 s 前后 `pilot.csv` 显示
  `state=NAVIGATE`、`bt_phase=NAVIGATE`、`control_active=True`、
  `task_completed=False`、`vio_ready=1`；注入时刻之前
  `task_completed` / `task_revoked` 事件数均为 **0**。
  这两个事件分别在 **+22.891 s** 和 **+31.079 s** 才出现（正常接管），
  用整轮结束后的文件回放会误判，按时间截断后结论明确。
- 因此这是**注入器工具缺陷导致的漏发信号**，不是注入前功能失败、
  不是控制/高度问题、不是「测试运气不好」。
- 修复：`count_matches()` 把命令替换的退出码与数值分离，结果恒为单个十进制数；
  备份 `prechange/run_vio_interruption_fix19.before_flag_fix.sh`、
  diff `evidence/diff_injector_flag_fix.patch`，离线复现验证通过。
  **本轮未重飞**。

**A/B/C 判定：A**。中断响应（信号发送、VIO 退出、发布/health 停更、授权撤销、
任务停止、接回、降落及 disarm、桥与监督器持续）**全部未验证**，
不得由本轮 PASS 推断。普通任务评估器本轮是 PASS（因为没有中断），
这与「专项中断响应通过」是两件事，报告分开记录。

---

## 6. 全部实际启动、失败与退出状态

| # | 类型 | 运行目录 | VERDICT | exit | 注入 | 清理 |
|---|---|---|---|---|---|---|
| 1 | 正常 5 m | `fix20_func5m_...232210` | PASS 16/0/1 | A_complete | — | verified_empty |
| 2 | 冷启动 r1 | `fix20_cold_r1_...232720` | PASS 16/0/1 | A_complete | — | verified_empty |
| 3 | 冷启动 r2 | `fix20_cold_r2_...232943` | PASS 16/0/1 | A_complete | — | verified_empty |
| 4 | 冷启动 r3 | `fix20_cold_r3_...233212` | PASS 16/0/1 | A_complete | — | verified_empty |
| 5 | 中断 | `intr_fix20_r1_...233636` | PASS 16/0/1 | A_complete | **未发送（A）** | verified_empty |

授权 5 次、实际 5 次；**无自动重飞、无筛选启动失败、无第二个新批次**。
收尾 `ps` 无残留；共享 `MicroXRCEAgent`(pid 3643) 保持运行、未被触碰。
历史 FIX17/18/19 运行目录、25.88 MB 旧证据文件、旧 ACC03 3/3 全部原样保留。

历史额度仍原样登记：FIX17 功能验证 3 授权/4 实耗（超用 1）；ACC03 3/3；
FIX17 中断 1/1（注入前失败）；FIX18 新增 2 次（#1 PASS、#2 注入前功能失败）；
FIX19 授权 1 次按前置分支未起飞。

---

## 7. 原 ACC03 与新版本证据的关系

- **旧 ACC03 3/3** 属于 FIX17 原冻结运行环境（旧飞手 `bc97c191…`），
  **不能**自动当作修复后飞手（`f9397861…`）的重复性证据，也不改写为 FIX18/FIX20 的 3/3。
- 本批次 `NP01_FIX20_COLDSTART_3X` 是**独立新编号**、独立冻结实物、
  独立 3 次冷启动，只对新飞手版本成立。
- 新飞手相对旧飞手的差异仅为 §1 的高度参考 reset 连续性契约；
  节点二进制、`libORB_SLAM3.so`、Gate 配置、场景、评估器 md5 全部未变。

---

## 8. 未验证边界

- **中断响应仍 UNVERIFIED**（A，注入未执行）。若要完成，需新授权：
  用已修的注入器（`count_matches`）重跑一次中断回归，沿用冻结的 C1–C10
  与 NAVIGATE+6 s 注入事件，不事后调整时刻。本轮不追加中断额度。
- 修复只覆盖「单个可确认的 z-reset 事件」；跨多事件/丢失中间 delta
  保持失败关闭，未在线验证（只在生产路径单测覆盖）。
- 原场地（`rm27_world` 原出生点）可靠性仍未通过；
  `latency_evidence` 仍 UNVERIFIED（延迟性能未达标/未定义，不判定）；
  `transport` 配置的 time_sync / 样本年龄仍是 `[SIM-ONLY]` 传输层补偿。
- 全部来自 WSL 仿真，**未实机飞行、未做 Nano 放行**。

---

## 9. 文件索引

| 内容 | 路径 |
|---|---|
| 契约模块（纯函数） | `newpkg_fix20/excite_height_reference.py` |
| 生产飞手 diff / 回滚 | `newpkg_fix20/evidence/diff_sim_pilot_fix20.patch`、`newpkg_fix20/ROLLBACK.sh` |
| 冻结源码/候选/实物 | `newpkg_fix20/prechange/`、`candidate/`、`frozen/`、`FREEZE_FIX20.json` |
| 生产路径测试 | `newpkg_fix20/tests/test_excite_height_production.py`、`evidence/production_test_output.txt` |
| 符号契约核实 | `newpkg_fix20/offline/sign_contract.json`、`verify_sign_contract.py` |
| 正常回归核查 | `newpkg_fix20/evidence/run1_verification.json` |
| 冷启动批次清单 | `newpkg_fix20/flight/COLDSTART_3X_ATTEMPTS.jsonl`、`COLDSTART_3X_RESULT.json` |
| 中断诊断 | `newpkg_fix20/evidence/intr_precise_diagnosis.json`、`offline/intr_precise.py` |
| 注入器修复 diff | `newpkg_fix20/evidence/diff_injector_flag_fix.patch` |
| 全轮汇总 | `newpkg_fix20/evidence/all_runs_summary.json` |
