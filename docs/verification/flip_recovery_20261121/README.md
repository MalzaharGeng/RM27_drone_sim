# PX4 地面侧翻恢复试验（2026-11-21）

Gazebo X500 侧躺配对试验中，默认参数通过 **0/6**，自动换向调参组通过 **6/6**。完全倒扣的 15 次出力试验均未通过。试验覆盖 PX4 地面翻正及 Gazebo 电机输出链，不包含 ORB-SLAM3、实机试验或原计划的 300 次验收。

## 试验工况与方法

| 组别 | 工况与记录量 |
| --- | --- |
| 默认与禁用对照 | X500 模型与初始落姿相同。默认组开启翻正，禁用组测量无恢复输出时的姿态变化。 |
| 参数筛选 | 调整翻转策略、两组油门、低位组转向、油门上升率、停油角、出力时限及进度判据，记录状态机、四路电机命令和姿态。 |
| 配对复验 | 两组筛选参数及低油门变体覆盖左右侧躺、不同航向和落高。六组有效工况均在约 79.924° 的稳定姿态触发。 |
| 倒扣 | 在接近 180° 的姿态下比较两种策略、不同油门和朝向，记录出力及机体角位移。 |

环境为 Ubuntu 26.04 WSL、ROS 2 Lyrical 和 Gazebo Sim 10.5.0。试验使用本仓库的 [PX4 源码快照](../../PX4_SOURCE.md)，在独立 SITL 树应用[试验输出桥](../../../tools/flip_recovery/sweep/sitl_bridge.diff)。输出桥将翻正请求转换为 Gazebo 带符号转速，未模拟真实 DShot 停机、换向保护及带桨负载。地面摩擦与接触几何沿用 X500 模型。

各轮使用独立的 Gazebo 分区、发现端口和 PX4 工作目录。模型以指定落姿生成，落稳后切入翻正模式。恢复期间未施加外力，也未直接修改姿态。

## 通过判据

通过必须同时满足四项条件：未解锁状态下记录到主动恢复输出；Gazebo 姿态真值 ≤15° 且角速度 ≤10°/秒连续至少 1 秒；观测尾段至少 2 秒电机命令为零；PX4 报告成功并自动切回 Position。机体回正但 PX4 未报告成功的样本单独统计。

## 试验结果

| 工况 | 禁用 | 默认 | 调参 B：强制低位反转 | 调参 A：自动换向 | 调参 A：高位降至 0.45 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 侧躺约 79.924°，六组配对 | 0/6 | 0/6 | 6/6 | **6/6** | 5/6 |
| 倒扣约 180°，一组配对 | 0/1 | 0/1 | 0/1 | 0/1 | 未执行 |

- 默认侧躺代表试验最低到 50.023°，最终停在 58.856°，报 `NOT_SETTLED`；同初态禁用组始终约 79.924°。调参 A 六次均回到约 0°，从开始出力到模块报成功耗时 2.04–2.30 秒，随后退出到 Position。
- 高位油门设为 0.45 时，六次均恢复正立；其中一次 PX4 因姿态估计重置（`ATTITUDE_RESET`）判失败，闭环通过 5/6。
- 俯仰落姿组最终倾角为 56.648°，低于允许的最小进入角 60°。对应的四条 `SETUP_ONLY` 记录不计入配对分母。
- 15 次倒扣出力试验中，14 次因进度不足（`NO_PROGRESS`）退出，1 次超时。电机命令峰值约 410 rad/s，机体最终姿态仍接近 180°。所测参数未实现完全倒扣恢复。
- 规划器和换向时序主机回归通过 64 项断言，实际混控功能测试通过 1 项。混控用例同步调用生产库，未覆盖飞控工作队列调度。

![侧躺姿态对比](side_recovery.png)

## 仿真配置与复现

[调参 A 完整配置](sitl_side_profile.json)在试验中启用 `FLIP_EN=1`，保留 `FLIP_MODE=0`、`FLIP_LOW_MODE=0`、进入角 75° 和最小进度 3°。变化量为高/低油门 0.60/0.35、停油角 38°、油门上升率 2.0、出力上限 4.0 秒、进度判定起点 1.5 秒。该配置仅用于本 X500 仿真。

已有实验构建可直接执行下方的试验命令。首次复现时，按 [PX4 构建说明](../../PX4_SOURCE.md)导出独立构建树并准备依赖，再执行下列代码中标注的三条构建命令。补丁仅用于 SITL。

```bash
SIM_REPO="$(pwd)"                 # 在 RM27_drone_sim 仓库根目录执行
SWEEP="$SIM_REPO/tools/flip_recovery/sweep"
PX4_SOURCE=/path/to/standalone/px4
RUN_BASE="$(mktemp -d)"
source /opt/ros/lyrical/setup.bash
# 若尚未构建带实验桥的 SITL，先执行以下三条命令：
# git -C "$PX4_SOURCE" apply --check "$SWEEP/sitl_bridge.diff"
# git -C "$PX4_SOURCE" apply "$SWEEP/sitl_bridge.diff"
# make -C "$PX4_SOURCE" px4_sitl_default
source "$PX4_SOURCE/build/px4_sitl_default/rootfs/gz_env.sh"
python3 "$SWEEP/run_batch.py" "$SWEEP/confirm.json" "$PX4_SOURCE" "$RUN_BASE/confirm1" --jobs 3
python3 "$SWEEP/run_batch.py" "$SWEEP/confirm_auto.json" "$PX4_SOURCE" "$RUN_BASE/confirm_auto1" --jobs 3
python3 "$SWEEP/run_batch.py" "$SWEEP/confirm_low.json" "$PX4_SOURCE" "$RUN_BASE/confirm_low1" --jobs 3
python3 "$SWEEP/score.py" "$RUN_BASE/scored" "$RUN_BASE"/confirm{1,_auto1,_low1}
python3 "$SWEEP/compare.py" "$RUN_BASE/scored"
```

脚本依赖 ROS Lyrical 的 Gazebo Python 绑定，以及 `numpy`、`pyulog`；确认实例编号和发现端口空闲，必要时通过 `--first-instance` 改起始编号。工具目录还包含初测和两轮筛选矩阵。

## 试验数据

[75 次运行结果](results.csv) · [配对统计](paired_confirmation.csv) · [JSON 数据](results.json) · [证据清单](evidence/manifest.json)

结果表包含禁用对照和准备场景，不含共享发现端口的早期诊断批次。归档的五个样例分别为侧躺禁用、侧躺默认、侧躺调参 A、完全倒扣和姿态估计重置。各样例包含独立姿态、电机命令及原始 ULog。软件回归见 [主机测试](host_regression.log) 和 [混控测试](mixer_regression.log)。
