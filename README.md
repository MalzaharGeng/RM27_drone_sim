# RM27 无人机导航仿真

本仓库维护 RMUC 2027 场地的 PX4 SITL 与 Gazebo 仿真环境。Harrier 机型用于双目惯性定位、PX4 外部视觉融合和航点跟踪；X500 用于独立验证 PX4 的地面侧翻恢复。已知地图 A* 演示保留为独立运行入口。

![RMUC 2027 场地的 Gazebo 渲染](docs/images/arena_gazebo_20261107.png)

## 仿真功能

| 功能 | 实现 | 文档 |
| --- | --- | --- |
| RMUC 2027 场地 | 官方 STP 经 OCCT 三角化；按 CAD 逐面颜色导出九组网格。Harrier 原场地使用分色外观和网格碰撞。 | [场地构建](tools/step_to_arena/) |
| 双目惯性导航 | Harrier 双目与 IMU 数据经 ROS 2 接入 ORB-SLAM3，定位结果送入 PX4；Gazebo 真值用于独立监督和误差评价。 | [运行配置](experiments/vio_20261120/README.md) · [联调记录](docs/VIO_HARRIER_20261120.md) |
| PX4 地面侧翻恢复 | 飞控在落地、未解锁且姿态满足进入条件时，按电机位置分组出力。达到停油角后等待姿态稳定，再判定结果并退出模式。 | [实现与改动](docs/PX4_FLIP_RECOVERY.md) · [试验报告](docs/verification/flip_recovery_20261121/README.md) |
| 已知地图规划 | A* 按场地障碍图生成航点，经 PX4 Offboard 接口执行。 | [操作说明](docs/TERMINAL_SLAM_GAZEBO.md) |

侧翻恢复由飞控执行，默认关闭（`FLIP_EN=0`）。启用后仍需切入 Flip Recovery 模式，功能只在落地且未解锁时出力，不参与飞行中的姿态保护。X500 试验通过独立 Gazebo 输出桥驱动电机，未运行 ORB-SLAM3。

## 上游项目与本仓库改动

| 来源 | 本仓库的使用方式 |
| --- | --- |
| [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)，v1.17.0 | 提供飞控、SITL 和 Gazebo 接入基础。 |
| [natgeng/PX4-MicoAir743v2-AIO · feat/flip-recovery](https://github.com/natgeng/PX4-MicoAir743v2-AIO/tree/feat/flip-recovery) | 提供 MicoAir 板级支持及原始翻正模式、参数和状态接口；本仓库收录接入后的 PX4 完整源码快照。 |
| [MalzaharGeng/ORB_SLAM3](https://github.com/MalzaharGeng/ORB_SLAM3) | 提供 Harrier 双目惯性定位和 PX4 外部视觉接口；算法在配套仓库维护。 |
| RMUC 2027 官方 CAD | 提供场地几何与颜色，转换脚本和 Gazebo 资源在本仓库维护。 |

本仓库的翻正改动分为两层：PX4 源码补齐 DShot 自动换向与方向恢复，修正电机高度排序、油门映射和有向进度，增加停油后的稳定判定；SITL 试验单独应用 Gazebo 可逆输出补丁，并记录禁用输出、默认参数和调参组的结果。每项改动的代码位置和验证依据见[翻正实现与改动说明](docs/PX4_FLIP_RECOVERY.md)，源码与固定依赖见 [PX4 构建说明](docs/PX4_SOURCE.md)。

## 构建与运行

验证环境为 Windows 11 / WSL2 Ubuntu 26.04、ROS 2 Lyrical、Gazebo Jetty（gz-sim 10.5.0）和 PX4 v1.17.0。场地渲染使用 WSLg；实验机的 GPU 为 NVIDIA RTX 5070 Ti Laptop。

从本仓库根目录导出独立 PX4 源码树。依赖和环境准备见 [PX4 构建说明](docs/PX4_SOURCE.md)。

```bash
python3 tools/px4/create_build_tree.py --output "$HOME/rm27_px4_source" --submodules
source /opt/ros/lyrical/setup.bash
make -C "$HOME/rm27_px4_source" px4_sitl_default
```

- **Harrier 双目惯性导航：** 按[实验运行说明](experiments/vio_20261120/README.md)配置模型、参数和运行目录。
- **X500 地面侧翻恢复：** 按[复现流程](docs/verification/flip_recovery_20261121/README.md#仿真配置与复现)在独立 PX4 树应用试验补丁，运行配对场景并生成评分。
- **已知地图 A*：** 在仓库根目录加载 `env.sh` 后，执行 `python3 scripts/rm27_nav.py --dry-run` 检查航线；仿真入口为 `./scripts/run_nav_demo.sh`。
- **仅加载场地：** 在 `rm27_sim/` 中运行 `./run_rm27.sh`。

## 目录

| 路径 | 内容 |
| --- | --- |
| [PX4-Autopilot/](PX4-Autopilot/) | PX4 源码快照、翻正模块、DShot 驱动和回归用例 |
| [rm27_sim/](rm27_sim/) | Gazebo 场地、机体、传感器和碰撞模型 |
| [ros2/rm27_slam/](ros2/rm27_slam/) | 双目惯性输入、时间映射和跟踪诊断 |
| [scripts/](scripts/) | 仿真控制、飞手接管、航点与独立真值评价 |
| [experiments/](experiments/) | 固定配置的实验运行环境 |
| [tools/flip_recovery/](tools/flip_recovery/) | X500 翻正试验脚本、评分程序和 Gazebo 输出桥补丁 |
| [docs/](docs/) | 工程文档与[验证记录](docs/verification/README.md) |
