# 飞手授权与十米导航实施计划

飞行流程：飞手手动起飞，在 Position 模式确认 VIO 和 PX4 定位有效，再切入 Offboard 授权一次任务；退出 Offboard 即撤销任务。默认沿接管时机头方向前进十米并保持，A* 避障另行验证。

## 接口与方案

ORB_SLAM3 的 C++ 桥只负责定位输出。drone_sim 的 Python 方形任务取消自动解锁、切模式和降落，改为飞手授权执行。授权判定模块不依赖 ROS，并在任务更新和最终发布两个位置检查。

授权需同时满足：新鲜 RC 开关、RC 来源手动输入、实际与目标 Offboard 模式、新鲜 VIO health、完整参考标识。接管、遥测陈旧、故障、估计器 reset 或参考变化立即撤销任务。

仅检查实际 Offboard 模式无法排除地面站或其他伴随计算机切换；仅检查 nav_state_user_intention 也无法确认物理开关。因此采用 RC 来源输入，并确认物理 Offboard 开关或显式配置的模式槽位。将全部任务移入 PX4 需另行开发固件，本轮保留 PX4 failsafe，只调整伴随端任务。

环境：Python 3、ROS 2 Lyrical、px4_msgs（VehicleStatus v1）、Eigen/OpenCV C++17、CTest/unittest。

## 实施步骤

1. 在两个工程外保存现有未提交改动和基线，目录为 flight_control_review_20261107。
2. 新增 drone_sim/scripts/pilot_authority.py，处理新鲜度、物理开关、参考绑定、接管锁存及局部不连续，先做无 ROS 边界测试。
3. 将 drone_sim/scripts/px4_square_bt.py 改为受保护的 Sequence：捕获当前位置和航向，执行直线或方形任务，再保持。进入前只发送 OffboardControlMode 心跳，不发送轨迹、解锁或模式指令。首次授权前必须观察到非 Offboard/未选中状态；撤销后需重启任务并重新拨档。完成任务后保持至飞手接管。
4. 默认关闭 rm27_nav.py 和 rm27_waypoints.py 的 MAVSDK 自动飞行入口，仅保留显式 simulation-only 开关。前者会改 PX4 参数，两者均会发送自动飞行指令，不用于实机任务。
5. 在 Px4Vio.h/Px4Vio.cc 增加角度不连续门控及 YAML 默认值，保留位置/速度和 epoch 锁存。角度参考故障需 disarm，扩展 px4_vio_regression。
6. 精度报告使用独立时间戳的 VIO、真值和目标样本，记录漂移、终点误差、丢帧及跳变，不拟合消除偏差，不以控制器自身估计作为真值。
7. 记录物理开关 DDS 发布要求、RC 槽位映射、Offboard 进入语义、接管行为、重定位限制及分阶段十米测量方法。
8. 构建相关 C++ 目标，执行 VIO/recording/sortie 回归、隔离 ROS 集成及 authority/BT/accuracy/planner 测试。记录命令和限制；该计划不包含实机指令、刷固件或 GitHub 推送。

## 验收要求

- 实机任务 BT 中无解锁、disarm、模式、起飞和降落指令发布器。
- 物理授权 Offboard 确认前不发布轨迹；接管绕过 fallback 并清除目标。
- Position 模式下定位独立发布，定位有效性不授予控制权。
- 陈旧或重复消息不续期。参考或重定位变化后，即使坐标数值一致也撤销任务。
- 十米飞行绝对精度以独立测量为准；ROS 模拟输入测试仅验证软件行为，不代表实飞性能。
