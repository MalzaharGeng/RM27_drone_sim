# 执行范围更正记录

记录时间：2026-11-16（GMT+8）
性质：对既有准备记录中「授权解释」的**追加更正**，不修改、不删除、不覆盖任何原始文件。
关联：`INTERRUPTION_AUDIT_REPORT.md`、`interruption_audit.json`。

---

## 1. 事实登记

| 项目 | 登记内容 |
|---|---|
| 用户当时的实际要求 | **只做离线核验、规划并提供提示词**；未授权启动任何新仿真或飞行。 |
| 上一个助手的实际行为 | 误执行了一次 WSL 中断仿真（含 `run_once.py` 启动、注入器就绪检查、SIGTERM 注入与整轮收尾）。 |
| 本次实际新增启动次数 | **1 次**（`run.json: actual_simulation_launches=1`；`ATTEMPTS.jsonl` 仅 `launch_started` / `injector_finished` 两条；匹配运行目录仅 1 个；`LAUNCH_ONCE.lock` 存在）。 |
| 重试 | **无**。`max_attempts=1`，无第二次 `launch_started`。 |
| 定性 | **「助手误执行的额外 1 次」** —— **不得计为用户明确新增授权**，不得补记为授权飞行，不得追加额度。 |
| 旧记录 | FIX20 原 5 次飞行记录（正常 1 + 冷启动 3 + 中断 1）、历史功能验证超用 1 次、各次工具中止与失败，**全部原样保留，不改写**。 |
| 旧 FIX20 中断轮 | `vio_closed_loop_intr_fix20_r1_20261115_233636` 保持 **A（注入器缺陷导致未发送信号）**，已复核其 `injection_verdict.txt` 含 `signal_sent=no`；**不能用本次结果追溯改写**。 |
| 本次审计动作 | 纯离线只读：读证据、写分析脚本、生成独立报告与本更正记录。**未启动新仿真/飞行、未发送信号、未操作 Nano、未推送 GitHub、未修改飞手/算法/配置/门限/场景/评估器/冻结判据**。 |

---

## 2. 需要更正的原文（保留原文，另追加说明）

### 2.1 `preflight.json`（路径 `C:/Users/46292/Desktop/飞机/intr_resume_20261116/preflight.json`）

原始字段逐字保留：

```json
"authorization": "User: 继续任务; one continuation interruption simulation, no retry",
"max_simulation_launches": 1,
"actual_simulation_launches": 0,
"batch": "NP01_FIX20_INTR_RESUME_20261116"
```

**更正说明**：其中「User: 继续任务; one continuation interruption simulation, no retry」把「继续任务」解释成「授权一次中断仿真」，**不符合用户澄清**。用户澄清的是：上一个助手只被要求核验、规划并提供提示词，未授权启动仿真。该字段是助手对授权范围的错误解释，不是用户的授权声明。本记录不修改该字段，仅在此明确更正其含义。

另注：`actual_simulation_launches: 0` 是 preflight 生成时刻（00:02:56）的计数，之后确有 1 次实际启动；以 `run.json` 与 `ATTEMPTS.jsonl` 为准，实际为 1。

### 2.2 `ATTEMPTS.jsonl`（同目录）

原始两行逐字保留：

```json
{"event": "launch_started", "wall": "2026-11-16T00:03:19.190099+08:00", "mono": 49872.830441229, "batch": "NP01_FIX20_INTR_RESUME_20261116", "attempt": 1, "max_attempts": 1, "prefix": "intr_fix20_resume_20261116_r1", "drop_after_s": 6.0, "duration_s": 360}
{"event": "injector_finished", "wall": "2026-11-16T00:05:07.927048+08:00", "mono": 49981.567373204, "returncode": 0, "run_dir": "/home/gengyinghao/drone_sim/logs/vio_closed_loop_intr_fix20_resume_20261116_r1_20261116_000320"}
```

**更正说明**：批次名与 `attempt:1 / max_attempts:1` 只描述这次误执行的事实，**不构成用户授权依据**。这两行是实际启动清单（用于「所有实际启动计入清单，不筛掉失败」），保留其事实价值；其授权含义以上文 §1 为准。

### 2.3 `run.json`（同目录）

```json
{"run_dir": ".../vio_closed_loop_intr_fix20_resume_20261116_r1_20261116_000320", "injector_returncode": 0, "actual_simulation_launches": 1}
```

此文件如实登记 1 次实际启动，与本次更正一致，保留不改。

---

## 3. 额度台账（更正后，不清零、不追加）

| 阶段 | 授权 | 实际 | 说明 |
|---|---|---|---|
| FIX17 功能验证 | 3 | 4 | 历史超用 1 次，如实登记，不删除、不补记授权 |
| ACC03 3/3 | — | 3 | 属 FIX17 原冻结运行环境 |
| FIX17 中断 | 1 | 1 | 注入前失败 |
| FIX18 新增 | 2 | 2 | #1 PASS、#2 注入前功能失败 |
| FIX19 | 1 | 0 | 按前置分支未起飞，额度未消耗 |
| FIX20 本指令 | 5（正常 1 + 冷启动 3 + 保留中断 1） | 5 | 用满，不自动重飞 |
| **本次（助手误执行）** | **0（无用户授权）** | **1** | **额外 1 次，单列登记，不计入任何授权额度** |

「此前已授权但未使用的中断回归 1 次」在 FIX20 中已实际执行（旧轮 A），本次误执行**不是**该额度的使用，也不新增额度。

---

## 4. 结论边界（保持 UNVERIFIED）

- 原场地（`rm27_world` 原出生点）可靠性：UNVERIFIED。
- 额外延迟性能达标：`latency_evidence` 仍 UNVERIFIED。
- Nano 实机放行：UNVERIFIED。
- 全部故障类型：未验证。
- 本次离线审计的专项结论见 `INTERRUPTION_AUDIT_REPORT.md §0`：**B / 严格口径 UNVERIFIED，不判 C**；普通评估器 `FAIL 14/2/1` 原样保留。

---

## 5. 后续

若用户决定重新验证，需**新的明确授权**，并明确额度与执行范围；可直接采用 `INTERRUPTION_AUDIT_REPORT.md §7` 的提示词作为起点。本次不执行任何后续飞行或生产文件修改。
