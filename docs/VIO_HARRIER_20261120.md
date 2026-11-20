# Harrier 原场地 VIO 联调记录（2026-11-20）

本阶段在 WSL Ubuntu 26.04 / ROS 2 Lyrical / PX4 v1.17.0 / Gazebo Jetty 下验证原场地视觉惯性闭环。算法使用 ORB-SLAM3 IMU_STEREO 与 Px4Bridge；真值仅用于独立监督与离线评价。当前工程目标为同一冻结候选连续三轮完成 5 m、BT HOLD、飞手接回与落地上锁，尚未达到。

## 模型与输入

| 项 | 当前配置 |
|---|---|
| world | `rm27_midhangar_n_world` |
| 实体 / PX4 机型 | `Drone-Harrier` / `gz_x500_mono_cam` |
| 出生位姿 | `6.80,8.05,0.504900,0,0,1.5707963`，位置 m、姿态 rad |
| 机体资源 | Gabel 分色网格及简化碰撞；质量、惯量和旋翼参数见 `models/x500_mono_cam/model.sdf` |
| 相机 | 960 × 720、30 Hz、HFOV 1.590 rad、基线 0.050 m、图像噪声 0.003 |
| 左目安装 | base_link FLU 中 `(0.0485,0.025,-0.0057)` m，光轴向前 |
| IMU | 250 Hz，传感器及噪声定义在同一模型内 |
| ORB 配置 | `RM27_Harrier_IMU_Stereo_baseline.yaml`，保留 VIBA1/2 与 20 s 有界地图保留窗口 |
| 桥接配置 | `px4_vio_sim_transport.yaml`，时间传输补偿限仿真使用 |
| 任务 | 5.00 m、0.35 m/s、到达容差 0.15 m、驻留 2.0 s、跟踪误差上限 1.8 m |

场地采用官方 CAD 分色视觉和网格碰撞，停用人工 ORB 贴片。早期宽场地纹理 world 作为对照保留。机体显示名不改变 PX4 airframe/make target，传感器话题按 world、实体名与 link 解析。

## 代码变更及职责

| 文件 | 处理的问题 |
|---|---|
| `ros2/rm27_slam/src/stereo_inertial_sim_node.cpp` | 双目/IMU 输入、源时间映射、初始化入口及诊断钩子 |
| `ros2/rm27_slam/src/track_diag.*` | 有界队列后台批量写诊断，未设置 `RM27_TRACKDIAG_DIR` 时不启用 |
| `experiments/vio_20261120/scripts/sim_pilot.py` | FAULT 后按 PX4 水平位置/速度有效性决定 LAND 驻留；TAKEOVER 仍保留原 2 s |
| `experiments/vio_20261120/scripts/pilot_navigation_bt.py` | 相对初始航向的任务偏角与 ROS 上下文关闭保护 |
| `experiments/vio_20261120/scripts/pilot_navigation_boot.py` | 校验并转发 `bearing_deg`，缺省 0° 保留原任务方向 |
| `experiments/vio_20261120/runner/agent_registry.py` | 按 PID、starttime、命令和本轮归属回收自建会话 Agent，防止 PID 复用误杀 |
| `experiments/vio_20261120/runner/run_profile_isolated.sh` | 每轮独立 PX4 参数目录，保留进程归属、清理与逐项评价 |

候选与 `scripts/` 生产路径分别保留。本次整理调整了候选检出路径、测试导入和运行目录检查，并让外来实例观察进程由外层脚本等待及回收；这些仓库路径适配完成离线检查，未追加整组仿真。历史二进制与整理后构建产物不宣称逐字节相同。

## 六次运行与正式组结论

| 运行 | 用途 | 结论 | 主要证据 |
|---|---|---|---|
| D1 | 诊断 | 无效 | 外来 PX4/Gazebo 实例污染，共享 ROS 域 |
| D2 | 诊断 | FAIL | EXCITE 倾角 25.7° 超过 25.0° 包络；未覆盖目标失锁事件 |
| D3 | 对照，诊断关闭 | PASS | 最大位移 5.04 m，距离带保持 13.7 s |
| T1 | 正式，诊断关闭 | PASS | 最大位移 5.00 m，距离带保持 13.5 s，完成 HOLD 与接回落地 |
| T2 | 正式，诊断关闭 | PASS | 最大位移 5.07 m，距离带保持 13.7 s，完成 HOLD 与接回落地 |
| T3 | 正式，诊断关闭 | FAIL | 未进入 NAVIGATE；交接前 `at_home` 未满足，60 s 超时退出 |

正式组为 T1 → T2 → T3，同组不替换轮次，**整组未通过**。T1/T2 各为 16 PASS、0 FAIL、1 UNVERIFIED；T3 为 7 PASS、5 FAIL、5 UNVERIFIED。`latency_evidence` 属原非必选项，T1/T2 的该项未验证不否决单轮，也不表示性能验收通过。逐项记录见 [acceptance_results.csv](verification/20261120/acceptance_results.csv) 和 [原始评价输出](verification/20261120/runs/)。D3 及正式轮的 Agent 均完成身份登记和回收，清理扫描为零。

本组 `bearing_deg=90` 表示相对初始航向偏转 90°，在该出生和对齐条件下形成东向任务；代码未把世界正东方向作为通用硬编码。结果不能用于原北向任务。D3/T1/T2 任务段丢失帧比例为 0.0%，水平 RMSE 为 0.039–0.077 m，仅代表对应运行。

## 短时失锁与复位语义

PX4 v1.17.0 中，外部视觉路径的 `EV_MAX_INTERVAL` 为 200 ms，超过两倍间隔（400 ms）会停止 EV 融合；再次启动水平位置融合会复位位置参考。`no_aid_timeout_max=1 s` 另用于 fusion-failing 判定，不能把两者合并成一个可调的 `EKF2_EV_TIMEOUT` 参数。

Gate 拒绝后的 1 s 稳定等待可能跨过停融合窗口，但实际运行中的模式切换、fusion-failing 与重新启用存在时间重叠，尚不足以唯一归因全部复位。`stable_seconds=1.0`、`max_age=0.5`、`max_sample_gap=1.5`、本地系复位的 `requires_disarm` 锁存与监督 warn/STOP 判据均保留。

诊断轮观测到 3 帧 `tracking_state==OK` 而 `nav.valid==false`。跟踪枚举和导航快照有效性并非同一个判据；数量不足以解释历史 4 帧突发。`GetTrackedKeyPointsUn().size()` 取当前帧去畸变 ORB 关键点容器大小，不是匹配内点数。30/250 Hz 的平均输入到达率也不能排除逐帧同步差、IMU 覆盖不足或排队延迟。

逐帧诊断会增加关键点复制、队列互斥与后台 I/O；D2 未完成起飞段验证，不能把诊断称作零开销。正式入口默认关闭，仅在专用诊断轮使用。下一步需降低诊断采样代价，并分别处理交接前漂移、初始化姿态裕度和 NAVIGATE 内短时失锁。

## 验证边界

本轮没有 Nano 部署或实飞，没有碰撞回正与空中跨地图恢复验收。PX4 姿态控制、ORB 地图恢复和 Gate 重新接管仍需整机联调。11-16 的宽场地正常任务与中断审计保留在历史文档中，不并入本次原场地正式组分母。

本次同步的构建与回归结果见 [同步验证记录](verification/20261120/SYNC_VALIDATION.md)。
