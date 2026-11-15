# 遥控器优先、十米导航与重定位检查

## 当前实现和原来的区别

原 `scripts/px4_square_bt.py` 是内嵌 Python 行为树，无 XML：

```text
Fallback
  SafeGuard(Sequence(WaitLink, WaitPosition, WaitPreflight, WaitStill,
    StreamSetpoints, Arm, EnterOffboard, Takeoff, Square, Hold, LandAndWait))
  FailsafeLand
```

原来默认主动解锁/切 Offboard；`--pilot-offboard` 只改变进入阶段的等待，后续没有持续的遥控器权限门。定时器还会在安全检查前发送上一帧轨迹，Fallback 会在人工退出模式后继续发降落指令。LandAndWait 可主动上锁、切 Loiter。原来的 `rm27_nav.py` 是 MAVSDK+A* 的自动飞行流程，不是行为树；它会改 PX4 参数并自动起飞/降落。

现在 `px4_square_bt.py` 保留命令入口，实现在 `pilot_navigation_bt.py`；纯权限逻辑在 `pilot_authority.py`。控制流程：

```text
等待：飞手起飞，Position 验证，开关先处于非 Offboard
  → 只预发 OffboardControlMode 心跳
  → 飞手拨到 Offboard，PX4 实际模式及用户意图均为 Offboard
  → 记录接管点/航向/参考标识
  → Sequence(十米直线 或 正方形, 持续悬停)
任意权限失效 → 清空目标，停止定时器和控制流，结束本次任务
```

| 飞手/定位状态 | 行为树和控制输出 | 定位输出 |
|---|---|---|
| Altitude / Position，尚未接管 | 不执行任务；准备条件满足时仅发送 OffboardControlMode 心跳 | VIO 通过自身质量门控继续发给 PX4 |
| 物理开关选择 Offboard，实际/意图均 Offboard，其他条件满足 | 执行单次任务，20Hz 位置轨迹 | 同上 |
| 退出 Offboard、开关退出、锁机、RTL/Kill/Loiter/Termination、失控保护 | 立即撤销；不发切模/降落/上锁命令，不自动恢复 | 由独立 VIO 门控决定 |
| RC/状态/定位/健康消息过期，地图/时钟/估计器参考改变，位姿跳变 | 撤销任务并停止心跳/轨迹；PX4 执行自身 Offboard-loss 行为 | 不把异常定位伪造成平滑观测 |

“立即”指接收回调或下一个 50ms tick 撤销，没有硬实时保证；物理开关到 DDS 的延迟及飞控模式切换延迟需实测。已发送、尚在传输的包无法撤回，PX4 实际退出 Offboard 后应忽略其轨迹。接管/失效撤销后不再预发心跳；要执行新任务，重新启动脚本，并让程序看到一次非 Offboard 模式和未选中的开关。

## 为什么 Altitude/Position 下有心跳

PX4 要求进入 Offboard 前持续接收超过约 1 秒的控制存活信号。如果严格等实际 Offboard 才发送心跳，会永远进不去。这里仅预发 `OffboardControlMode(position=True)`；不发轨迹，不请求切模式，不解锁。该消息本身不会让 Altitude/Position 的飞机执行行为树。飞手退出后连这个心跳也停止。

2026-11-08 更新：条件持续满足、20Hz 心跳连续预流 1.2s 后，程序输出 `OFFBOARD READY`，状态话题置 `offboard_ready=true`。看到提示后再拨 Offboard。提前拨动不会延迟自动启动，需要切回非 Offboard，等提示再拨。正常选择后最多等待实际 Offboard 3s，期间维持心跳；超时后要求重新拨杆。循环间隔超过 0.2s 会重置预流准备状态。

## 飞控 DDS 的必要配置

当前 PX4 源码的默认 DDS 表没有导出 `manual_control_switches`。程序不会把 `nav_state == 14` 当作物理授权的替代条件。需要在**与实机固件对应**的 `src/modules/uxrce_dds_client/dds_topics.yaml` 的 `publications` 中补充：

```yaml
  - topic: /fmu/out/manual_control_switches
    type: px4_msgs::msg::ManualControlSwitches
```

准备好的差异文件是 `docs/rc_switch_dds.patch`。实机固件本次没有刷入。用户前一轮 SITL 构建产物已导出该消息，本次复测使用该 SITL 二进制，源配置文件未改写。开关变化时立即发布、未变化约 1Hz，不能要求必须 20Hz。

2026-11-08 根据真实发布率修正时限：

| 输入 | 发布/接收总时限 | 原始 sample 时限 | 依据 |
|---|---|---|---|
| VehicleStatus | 1.2s | 同发布 | 实测约2Hz，留单次漏帧余量 |
| ManualControlSetpoint | 0.7s | 0.8s | 实测约5Hz，未动杆时原始样本可重复，rc_update约300ms刷新 |
| ManualControlSwitches | 1.7s | 1.7s | 约1Hz，变化立即发布 |
| VehicleLocalPosition | 0.3s | 0.3s | 快速位置链路保持严格，未沿用诊断版0.75s |
| VIO navigation_health | 0.6s | 同发布 | 真实桥接约5Hz，未沿用诊断版1.5s |
| VehicleLandDetected | 1.6s | 同发布 | 实测约1Hz，原0.5s必然误判 |

消息 `timestamp` 判断发布新鲜度，`timestamp_sample` 独立检查原始观测。完全重复的发布包忽略、保留原数据、不续期；新发布可以带相同 sample，原始样本超期仍撤销。时间戳回退、同发布包内容冲突或来源过旧会失效。发布源时龄加上后续单调接收时龄一起计入时限，不能靠延迟接收获得额外续期。DDS Agent 与本机 UTC 必须同步。以上时限就是完整丢包预算，超期不再叠加宽限或自动续飞；明确的无效状态和飞手接管立即撤销。

默认检查专门的 `offboard_switch == SWITCH_POS_ON`；若使用六档模式开关，以**实机核对的槽位**传 `--rc-offboard-slot N`（1..6）。N 是模式槽编号，不是 PX4 nav_state 或 COM_FLTMODE 参数的枚举值。其他覆盖开关仍优先。摇杆源必须为 `SOURCE_RC`，GCS/MAVLink 手柄不能替代。

```bash
source ~/orbslam3_local/env.sh
ros2 topic echo --once /fmu/out/manual_control_switches
ros2 topic echo --once /fmu/out/manual_control_setpoint
ros2 topic echo --once /vio/navigation_health
```

若缺少开关话题，脚本会等待 `stale_switches`，不会发轨迹，也不会自动放宽条件。只启动 SLAM/VIO 不会启动行为树。

## 十米试验

用户确认的流程：手动起飞 → Position 确认 VIO+PX4 定位有效 → 遥控器切 Offboard。新任务以**实际接管点**为起点，沿接管时的机头方向走 10m，保持当时 NED 高度和航向，默认 0.25m/s。不是从地图原点起飞，也不是强制 z=-0.5。到位需要位置误差 ≤0.15m、速度 ≤0.2m/s 连续 2 秒，之后持续悬停，等待遥控器接回；这是估计位置判据，不能作为真实精度。

到位还要求设定点本身已推进到目标，避免切 HOLD 时突然跳完剩余距离。到位输出 `TASK COMPLETED` 日志及一次 `task_completed` 事件，随后保持位置控制，不会因“完成”静默退出 Offboard。

状态接口：`/navigation/task_status`，`std_msgs/String` JSON，5Hz，reliable volatile；话题前缀跟随 `--topic-prefix`。含 `phase`、`reason`、`offboard_ready`、`prestream_seconds`、`task_completed`、`control_active`、`dry_run`、各输入发布/采样/接收时龄、位置与目标。`prestream_seconds` 在接管后冻结，避免把整段飞行算成预流。一次性事件话题 `/navigation/task_event`，含实例、递增序号及 `offboard_ready/task_started/task_completed/task_revoked`。事件为 volatile，晚订阅用状态话题确认完成；使用方也需自行检查状态心跳是否新鲜。dry-run 仍输出诊断状态，但不发控制消息。

```bash
ros2 topic echo /navigation/task_status
ros2 topic echo /navigation/task_event
```

先进行拆桨台架切模式及实际 PX4 SITL/HIL 权限验收，再逐步进行短距离与十米试验。直线/正方形任务没有接入 A* 避障，测试路线需要足够净空。

仅观察（包括心跳在内均不发送）：

```bash
source ~/orbslam3_local/env.sh
python3 ~/drone_sim/scripts/px4_square_bt.py --dry-run --dry-run-secs 20
```

准备实测时的显式任务命令（本次未执行）：

```bash
python3 ~/drone_sim/scripts/px4_square_bt.py --mission line --distance 10 --speed 0.25 \
  --log ~/drone_sim/logs/ten_metre_control.csv
# 六档模式开关：在核实后增加 --rc-offboard-slot N
# 保留正方形任务：--mission square --side 1
```

`touch /tmp/px4_square_abort` 现在表示撤销控制，**不是紧急降落指令**；旧文件不会自动删除。飞手确认后自行删除文件。此操作会触发飞控自身 Offboard-loss 路径，要先验证其固件参数与遥控器退回 Altitude/Position 的实际行为。脚本不修改这些参数。旧命令 `--alt/--still-secs` 不再支持，避免误以为它仍会自动起飞。

`rm27_nav.py` 和 `rm27_waypoints.py` 的旧自动流程默认禁止；只规划仍可用 `rm27_nav.py --dry-run`。自动仿真及 `run_nav_demo.sh` 都必须显式传 `--simulation-auto`。这些兼容流程保留自动指令，导航演示还会写参数，不能用于实机权限验收；标记本身不是硬件隔离措施。`fmu_mavlink_bridge.py` 是模拟 DDS 的旧桥接，缺少本流程要求的物理 RC/健康证据，不能用它替代实机 DDS 来绕过门控。

## 重定位和跳变

两张地图的数值对齐不等于持续定位无跳变。检查到的 ORB_SLAM3 代码已经在重定位/地图切换/大更新/进入首地图参考时更新 `referenceEpoch`，VIO 桥接在解锁后遇到该变化会锁存故障，不在空中重建原点。已有位置-积分速度残差门继续保留。

本次补充 IMU 姿态单独跳变的门控：默认相邻有效样本的角差上限为 `10° + 180°/s × dt`。超过上限视为参考异常，停止 VIO，要求锁机后重新建立参考；不能靠落地稳定自动释放。参数是待实测调整的初值，不是精度指标。VIO 连续旋转和±π四元数等价关系由旋转角差处理。控制层还独立检查 PX4 local position/heading/reset counters。

不对重定位新旧位姿直接低通混合，也不把异常位姿限幅成合理观测。混合坐标系会制造虚假的速度/姿态。若后续要“空中重定位后无感续飞”，需要独立的连续 odom 控制系与 map→odom 修正，并变换所有任务目标；这次实现采用撤销任务、手动接管、锁机后重新验证参考的策略。

## 精度采集与评估

同步采集原始图像/IMU、SLAM 导航输出、`px4_telemetry.csv`、PX4 ULog、行为树 CSV 与独立真值（动捕/全站仪等）。卷尺与地面标记可以验收静止检查点，但不能证明动态全过程的精度。记录相机/IMU安装偏移与机体测量点转换，避免拿 IMU 中心和机体中心直接比较。

坐标系用独立标定预先对齐，不用整段待测轨迹拟合去掉偏差/比例误差。为每个有效检查点准备配对 CSV：

```csv
t_s,truth_t_s,vio_n,vio_e,vio_d,truth_n,truth_e,truth_d,target_n,target_e,target_d,valid,reference_epoch
0,0,0,0,-1,0,0,-1,10,0,-1,1,1
40,40,10,0,-1,9.7,0,-1,10,0,-1,1,1
```

示例是合成格式，不能当实测证据。VIO/真值是同一预标定 NED 系，单位米；两时间列使用同一秒时间轴，默认配对时差 ≤0.05s。控制 CSV 的 north/east/down 是**PX4 EKF 位置**，不能直接命名为独立 VIO 或真值。分别比较原始 VIO、EKF 与真值，才能区分定位误差和控制误差。

```bash
python3 ~/drone_sim/scripts/navigation_accuracy.py paired.csv --output accuracy.json
```

工具报告定位 RMSE/P95/最大误差、终点真实到靶误差、估计到靶误差、每个参考段起终点误差变化、无观测区间与跳变事件。缺少独立真值就拒绝评估。两行检查点只能说明两个点的误差，不能排除中途跳变。十米长度、重定位前后误差、接管时延及允许阈值要由实际数据验收；本次没有物理飞行数据。

## 软件验证

```bash
cd ~/drone_sim/scripts
python3 -m unittest test_pilot_authority test_navigation_accuracy test_nav_planner test_navigation_entrypoints -v
source ~/orbslam3_local/env.sh
python3 -m unittest test_pilot_navigation_ros -v
cd ~/ORB_SLAM3
cmake --build build_local --target px4_vio_regression px4_recording_gate_regression px4_sortie_regression px4_bridge_fixture -j2
ctest --test-dir build_local --output-on-failure -R 'px4_vio_regression|px4_recording_gate_regression|px4_sortie_regression|px4_bridge_ros_regression|px4_sortie_ros_regression'
```

ROS 控制层测试只用 domain44、`/pilot_test`；桥接测试 domain43、`/vio_test`。均为 mock DDS，既不连接实体飞控，也不验证真实 PX4 模式切换/失控保护或飞行精度。

2026-11-08 另有实际 PX4+Gazebo 复测，使用 DDS domain45、独立 rootfs 与 Agent UDP8889、GPS 定位＋模拟健康桩、恒定虚拟 RC（无抖动）。证据及修复报告在 `logs/bt_fix_20261108/`。它验证行为树闭环和接管，仍未验证真实 ORB-SLAM3 初始化/重定位、独立真值定位精度或实体飞机。
