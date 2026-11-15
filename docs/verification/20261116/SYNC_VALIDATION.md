# 构建与回归记录（2026-11-16）

本轮完成构建、离线测试和代码入库；未启动 PX4/Gazebo、飞行、故障注入或 Nano 操作。历史飞行结果与本次检查分开记录。

| 检查 | 结果与范围 |
|---|---|
| 算法构建 | 在原 WSL `ORB_SLAM3/build` 增量构建 ORB_SLAM3、orbslam3_px4_core 与 5 个新增回归目标完成；复用原依赖和构建目录，未做全新环境构建 |
| 算法 CTest | 原配置 20/20 全 PASS；随后增量构建重新链接的帧契约、Gate 快照两项再次 PASS |
| 源码对应 | 待提交的算法源码/CMake/测试与上述 WSL 构建输入逐字节核对一致；选择的文件 SHA256 另存来源清单 |
| ROS 包构建 | 在独立 build-check 目录配置并编译本仓库 ros2/rm27_slam：stereo_slam_node、stereo_inertial_sim_node、truth_odom_node 全完成，链接本机已构建且源码对应的 ORB 库；未运行节点 |
| FIX20 生产回放 | 仓库相对路径版 11/11，通过真实 Pilot.bounded_excite_decision，导入本次检出的 scripts/sim_pilot.py；原越界判定帧 1.1929→0.0373 m |
| 仿真离线回归 | 10 个 unittest 模块，共 117/117 PASS；规划、精度计算、飞手回中、起飞高度、状态回放、预检、入口授权、交接、权限、启动配置 |
| 静态检查 | 新增/修改的 Python 解析、Shell bash -n、JSON/XML/SDF 解析、资源引用与潜在凭据扫描通过 |
| 外部资源 | x500 基础模型由 PX4 提供，使用 GZ_SIM_RESOURCE_PATH；PX4、px4_msgs、ROS、Pangolin 不打包入库 |

编译输出保留了现有第三方 Eigen/Pangolin 等弃用/编译警告，无构建失败。未在 aarch64/Humble 编译或实机飞行。

最初一次构建命令误把 CTest 名 recovery_launcher_paths 当作 CMake target，前两个库完成后返回“无该 target”；已改为实际 target 列表并成功。一次两项 CTest 的命令经跨 shell 传参导致正则语法错误，未执行测试；随后分别执行两项并通过。两次命令均在更正后通过，未计入成功测试数量。

运行环境为 WSL Ubuntu 26.04 / ROS 2 Lyrical。完整 20 项 CTest 使用现有离线/合成回归环境，不代替飞行验收；测试仅导入 ROS/PX4 类型库，未创建节点。

历史中断轮仍 B / UNVERIFIED，不判 C；普通任务 FAIL 14/2/1、执行范围更正及 C9 证据缺口原样保留。

配套算法 master 提交：[572c2110ed51](https://github.com/MalzaharGeng/ORB_SLAM3/commit/572c2110ed510129413b6bf47d8fafd4443c9c36)。本次 ROS 编译输入与该提交对应源码核对一致。
