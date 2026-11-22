# Harrier 地面侧翻及倒扣恢复验证（2026-11-22）

Harrier 采用独立 PX4 SITL 入口验证地面侧翻和倒扣恢复。平地试验中，实际约 90° 的 18 次全部通过；实际 180° 的 34 次通过 33 次。完整双目模型在平地和 RMUC 中间机库场地分别验证两个姿态，均通过。斜面和挡条工况单独统计。

## 历史试验复核

沿用 X500 试验评分程序复核历史记录。36 次均在约 90° 的稳定姿态触发，均达到真值正立并停桨；其中 35 次由 PX4 报告成功并自动退出，1 次为 `TRUTH_ONLY / NOT_SETTLED`。12 个配置改变了航向和倾倒方向，实际触发倾角仍约为 90°。历史构建含试验性姿态重置补丁，统计结果单独归档。

旧补丁以会话起点为姿态基准，角差同时包含正常翻转和估计器修正。Harrier SITL 改用 EKF 发布的 `delta_q_reset`，仅继续执行计数单步变化、四元数模长有效且旋转量不超过 1° 的修正；其余重置条件仍中止恢复。历史日志中的会话内重置修正最大约 0.43°。

## 变更与对照

- **仅参数对照：** 原 684 rad/s 输出标度下，Harrier 在实际 180° 的 14 次开启输出试验均未回正，2 次禁用对照保持倒扣。使用 1000 rad/s 标度和现有力偶组合曾从 180° 自主回正；跨航向复验 16 次仅 1 次完整通过，说明只增大推力不足以稳定收油。共面旋翼的位置差仍可形成横滚或俯仰力矩，精确倒扣不能据此判定为物理上必然无法翻正。
- **保留的 Harrier SITL 变更：** 根据 PX4 姿态和陀螺仪做转速反馈；接近倒扣先选定一个俯仰方向，之后沿重力方向纠正并在接近正立时制动。使用实际倾角减小量判断进度。侧躺保留原 684 rad/s 输出标度，倒扣起始会话使用 Harrier 模型已有的 1000 rad/s 标度。驱动上限仍为 0.60，稳定、出力时限及退出逻辑沿用原状态机。
- **范围：** 功能由 `Drone-Harrier` SITL 专用入口启用，冻结 PX4 候选、X500 配置、基线二进制和 Harrier 碰撞模型未改。大批量验证只在临时 SDF 中省去相机渲染，完整模型另做代表工况。无外力，无姿态瞬移，未接触真机。

## 可复查结果

| 批次 | 实际触发姿态 | 结果 |
| --- | --- | --- |
| 历史构建与侧躺参数，平地对照 | 约 90° / 约 180° | 90° 13/13；180° 0/18。 |
| 交付构建，平地矩阵与补充朝向 | 约 90° | 18/18 完整通过。 |
| 交付构建，平地矩阵与补充朝向 | 约 180° | 33/34 完整通过；其中一次最低约 5.61° 后回倒，判 `NOT_SETTLED`。 |
| 同一平地批次的无输出或未到达进入角 | 90°、180° 或 0° | 9 次禁用对照保持倾倒；8 次落稳正立，未进入恢复出力。 |
| 交付构建，双目相机保留的完整模型 | 约 90°、约 180° | 各 1 次，2/2。 |
| RMUC 中间机库场地，双目模型 | 92.792°、180° | 各 1 次，2/2。 |
| 固定斜面、斜面挡条 | 105°、120°、135°、150°、165°、175°、179° | 179° 通过。105°、120°、135° 进度不足；150°、165° 超时；175° 停油未稳定。斜面另有一次 135° 释放后自然正立，与有挡条的有效 135° 不合并。 |
| 出力中关闭 `FLIP_EN` | 约 90°、约 180° | 两次都停止恢复输出并退出，报告 `SAFETY_LOST`。 |
| 收油前停留时间复验 | 约 90°、约 180° | 原交付构建和另加 160 ms 收油等待的构建各 13/13，其中各含 4 次与原失败相同的 180° 姿态/航向，均通过。未证明增加等待有独立收益，保留改动较少的交付构建。 |

平地释放角覆盖 0°、30°、60°、75°、90°、105°、120°、135°、150°、165°、175°、179° 和 180°，另加航向和镜像复验。部分释放姿态在触发前归并到平地的稳定接触姿态，因此覆盖率按实际触发倾角统计。闭环通过须满足电机主动出力、独立真值倾角 ≤15° 且角速度 ≤10°/秒连续 1 秒、尾段停桨两秒、PX4 判成功及自动切回 Position。评分门限保持原值。

## 原始记录与复现

- [交接数据重算](handoff_rescored/results.csv)
- [交付版本的平地矩阵](results/closed_final_20261122/results.csv)
- [完整模型](results/closed_full_asset_20261122/results.csv)
- [中间机库场地](arena_checks/results.csv)
- [斜面](results/closed_slope_20261122/results.csv)及[有挡条的陡斜面](results/closed_steep_20261122/results.csv)
- [出力中关闭恢复](results/closed_cancel_20261122/results.csv)
- [交付版本重复试验](results/closed_recheck_20261122/results.csv)、[收油等待试验](results/dwell_20261122/results.csv)
- [X500 原参数护栏验证](x500_guard/results.csv)
- [抽样原始日志清单](evidence_manifest.json)

[证据目录](evidence/)收录 11 个代表性运行，包含独立姿态、四路电机命令、ULog 和运行元数据。原始测量数据保持不变；本地运行路径在元数据中改为模型或场景文件名。评分表保留各批次全部结果。

按 [Harrier SITL 构建与运行](../../../tools/flip_recovery/harrier/README.md)准备构建。以下命令从仓库根目录复跑平地矩阵；将 `flat` 换为 `slope` 或 `steep` 可复跑对应斜面批次。平地批量与斜面批量使用归档的无相机渲染模型。

```bash
px4="${HARRIER_FLIP_PX4:-$HOME/harrier_px4_sitl}"
work="$(mktemp -d)"
source /opt/ros/lyrical/setup.bash
source "$px4/build/px4_sitl_default/rootfs/gz_env.sh"
export GZ_SIM_RESOURCE_PATH="$PWD/rm27_sim/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export PX4_HARRIER_SERVO=1
python3 tools/flip_recovery/harrier/prepare_matrix.py flat "$work/matrix.json"
python3 tools/flip_recovery/sweep/run_batch.py "$work/matrix.json" "$px4" "$work/probe" --jobs 3 --first-instance 81
python3 tools/flip_recovery/sweep/score.py "$work/scored" "$work/probe"
```

两份补丁应用后的源码与参与编译的文件一致。构建哈希见[证据清单](evidence_manifest.json)；冻结源码的规划器与方向时序回归通过 64 项断言。
