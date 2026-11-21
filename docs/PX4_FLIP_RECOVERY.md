# PX4 地面侧翻恢复：实现与改动

Flip Recovery 是 PX4 多旋翼地面恢复模式。落地、未解锁、姿态与角速度有效、机型和输出映射满足要求时，模式指令可触发恢复。控制器按机体姿态和旋翼位置计算电机分组与作用方向；达到停油角后切断恢复输出，并在稳定判定结束后报告结果。发生安全条件丢失或中途退出时停止出力。该模式不参与飞行中的姿态保护，也不依赖 ORB-SLAM3。

## 源码来源

- **飞控基础：** [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)，版本 v1.17.0。
- **翻正功能直接来源：** [natgeng/PX4-MicoAir743v2-AIO 的 `feat/flip-recovery` 分支](https://github.com/natgeng/PX4-MicoAir743v2-AIO/tree/feat/flip-recovery)。原版提供模式入口、Quad-X 电机分组、恢复参数和状态消息。原仓库的拆桨台架试验确认四台电机可执行手动换向；原版恢复输出未将方向请求接入自动 DShot 换向。
- **本仓库收录：** [PX4-Autopilot 源码快照](../PX4-Autopilot/)。[源码清单](PX4_SOURCE_MANIFEST.json)固定上游基线、快照身份和子模块版本；[构建说明](PX4_SOURCE.md)记录独立构建方法。

## 改动对照

| 项目 | 原版行为或试验缺口 | 本仓库实现 | 代码与验证 |
| --- | --- | --- | --- |
| DShot 自动换向 | 恢复油门取绝对值，控制请求的方向未传到电调。 | 按输出功能匹配 Motor1–4。每次换向先保持停机 300 ms，再分组提交方向命令 20/21，每组 10 帧，等待 100 ms 后重新爬升油门。退出、请求超时或安全条件失效时停油；条件允许后恢复正常方向。 | [方向状态机](../PX4-Autopilot/src/lib/flip_recovery/DirectionSequencer.hpp)、[DShot 驱动](../PX4-Autopilot/src/drivers/dshot/DShot.cpp)。主机回归覆盖指令计数、中断和方向恢复。 |
| 恢复油门映射 | 将 0–1 的请求直接送进 −1 到 1 的映射，0.20 对应配置输出区间约 60%。 | 在输出限幅前转换量程；0.20 对应配置区间的 20%，零请求保持停机。 | [混控实现](../PX4-Autopilot/src/lib/mixer_module/mixer_module.cpp)。实际混控回归检查油门映射、零输出及安全条件。 |
| 电机选组与翻转进度 | 高低位排序未处理 NED 的 Z 向下；以倾角差表示进度，跨越 180° 或反向运动时可能误判。 | 以世界高度排序；在初始翻转平面累计有向角，方向由起始策略固定。 | [翻正规划器](../PX4-Autopilot/src/lib/flip_recovery/FlipPlanner.hpp)。主机回归覆盖高低位、两种策略、跨 180° 和无进度退出。 |
| 停油与结果 | 达到停止角或观察超时可被记为完成。 | 达到停油角仅停止出力。估计倾角 ≤15°、总角速度 ≤10°/秒连续 1 秒才判成功；停油后三秒未稳定则返回 `NOT_SETTLED`。 | [规划器](../PX4-Autopilot/src/lib/flip_recovery/FlipPlanner.hpp)、[状态消息](../PX4-Autopilot/msg/FlipRecoveryStatus.msg)。回归区分收油、回倒、中途退出和稳定完成。 |
| SITL 启动和记录 | 普通 Gazebo 电机后端不接收翻正模块的带符号命令。 | SITL 随启动加载翻正模块，`FLIP_EN` 默认关闭。控制与状态纳入 ULog、uXRCE-DDS；X500 试验另用补丁将请求转为 Gazebo 带符号转速并维持输出调度。 | [模块](../PX4-Autopilot/src/modules/flip_recovery/)、[Gazebo 试验补丁](../tools/flip_recovery/sweep/sitl_bridge.diff)。ULog 与 Gazebo 电机话题用于分别验证请求、执行、停油和退出。 |

## 验证结论

规划器和换向时序主机回归通过 64 项断言，实际混控功能测试通过 1 项。Gazebo X500 侧躺配对试验中，默认参数通过 0/6，自动换向调参组通过 6/6。15 次完全倒扣出力试验均未达到恢复判据。试验方法、参数和代表性 ULog 见 [Gazebo 验证报告](verification/flip_recovery_20261121/README.md)。

DShot 停机和换向等待为软件时序。Gazebo 试验桥直接设置带符号的转速命令，未复现真实电调换向保护、带桨负载及实机接触条件。侧躺通过结论适用于该 X500 仿真配置。
