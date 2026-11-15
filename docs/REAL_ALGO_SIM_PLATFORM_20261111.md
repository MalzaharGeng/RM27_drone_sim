# 实机同源算法仿真平台（IMU_STEREO + Px4Bridge）

记录日期：2026-11-11
适用范围：本机 WSL Ubuntu-26.04，Gazebo Jetty 10.5、PX4 SITL、ROS 2 Lyrical。
授权边界：只改本机；未写 Nano、未推送 GitHub、未改 ORB-SLAM3 算法源码、未改
`pilot_authority.py` 的 `LIMITS`/`SAMPLE_LIMITS` 数值。

## 1. 这次改了什么

旧链路是「STEREO（无 IMU）+ `/slam/pose` + MAVLink 注入 + `sim_vio_health.py` 仿真桩」。
它验证的是行为树流程，但 `/vio/navigation_health` 是仿真侧适配器伪造的，不能代表实机门控。

新链路把同一份 Gazebo 数据接到与 Nano 相同的算法路径：

```text
Gazebo 双目 960x720@30 ─┐
Gazebo IMU  250 Hz      ├─ ros_gz_bridge ─ ROS 2 ─ stereo_inertial_sim_node
Gazebo /clock           ┘                          │
                                                   ├─ ORB_SLAM3::System::IMU_STEREO
                                                   └─ ORB_SLAM3::px4::Bridge
                                                        ├─ /fmu/in/vehicle_visual_odometry (DDS)
                                                        ├─ /vio/navigation_health（真实门控输出）
                                                        └─ /slam/pose（仅监控/精度评估）
```

两条路径并存，旧脚本未删除：

| 项目 | 旧路径 | 新路径（本文件） |
|---|---|---|
| 启动脚本 | `scripts/run_stereo_slam.sh` | `scripts/run_stereo_inertial_sim.sh` |
| 节点 | `stereo_slam_node`（`rm27_slam`） | `stereo_inertial_sim_node`（`rm27_slam`） |
| ORB-SLAM3 模式 | `STEREO` | `IMU_STEREO` |
| 位姿下发 | `/slam/pose` → `slam_pose_to_px4.py` → MAVLink | `Px4Bridge` → DDS `vehicle_visual_odometry` |
| health | `sim_vio_health.py` 仿真桩 | `Px4Bridge` 真实 `Gate` |
| 配置 | `config/RM27_Sim_D435i_Stereo.yaml` | `config/RM27_Sim_IMU_Stereo.yaml` + `config/px4_vio_sim.yaml` |

## 2. 新增/修改文件

| 文件 | 状态 | 作用 |
|---|---|---|
| `~/ORB_SLAM3/config/RM27_Sim_IMU_Stereo.yaml` | 新增 | IMU_STEREO 相机/IMU 配置（仿真几何） |
| `~/ORB_SLAM3/config/px4_vio_sim.yaml` | 新增 | Px4Bridge 门控/发布配置（`R_BI=diag(1,-1,-1)`） |
| `~/px4_ws/src/rm27_slam/src/stereo_inertial_sim_node.cpp` | 新增 | ROS 2 节点：订阅双目/IMU/clock，运行 IMU_STEREO，提交 Px4Bridge |
| `~/px4_ws/src/rm27_slam/CMakeLists.txt` | 修改 | 新增 `stereo_inertial_sim_node` 目标，链接 `libORB_SLAM3.so` 与 `liborbslam3_px4_core.a` |
| `~/px4_ws/src/rm27_slam/package.xml` | 修改 | 增加 `rosgraph_msgs`、`px4_msgs` 依赖 |
| `~/drone_sim/scripts/run_stereo_inertial_sim.sh` | 新增 | 探测话题、起 `ros_gz_bridge`、启动节点 |
| `~/drone_sim/scripts/pilot_navigation_bt.py` | 修改 | 严格模式改用 `timestamp_sample`，放松模式保持 `timestamp` |
| `~/drone_sim/scripts/pilot_authority.py` | 修改 | `_reset_lease` 严格模式保留 `source/samples`，仅放松模式清空自愈 |

备份在 `~/drone_sim/logs/real_algo_platform_20261111/backup/`。

## 3. 内参/外参：按仿真几何取值

**不复制 Nano 标定。** 权威几何来自：

- 相机：`~/drone_sim/rm27_sim/models/x500_mono_cam/model.sdf`
- IMU：`PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf`

| 量 | 取值 | 依据 |
|---|---|---|
| 分辨率 | 960×720 @30 | SDF `<image>` |
| 水平 FOV | 1.590 rad (91.1°) | SDF `<horizontal_fov>` |
| fx = fy | 480 / tan(1.590/2) = **470.87** | 由 FOV 与像素网格反推 |
| cx, cy | 480, 360 | 图像中心 |
| 基线 | **0.050 m** | 左目 y=+0.025、右目 y=−0.025 |
| 左目安装 | (0.12, +0.025, 0.242) m，光轴 +x | SDF `<pose relative_to="base_link">` |
| IMU | base_link 原点，250 Hz，FLU | x500_base `<sensor name="imu_sensor">` |
| `IMU.T_b_c1` | `R_bc=[0 0 1; -1 0 0; 0 -1 0]`，`t=(0.12,0.025,0.242)` | 光学系（右/下/前）→ body FLU |
| Px4Bridge `R_BI` | `diag(1,-1,-1)` | IMU FLU → 飞控 body FRD |
| IMU 噪声 | gyro 5.5e-5, acc 4.0e-4 | 由 SDF stddev 与 250 Hz 折算 |

> 修改 SDF 的分辨率、FOV、安装位姿后，`RM27_Sim_IMU_Stereo.yaml` 必须同步重算。
> 实机必须用 `Examples/Stereo-Inertial/DECXIN_Camera.yaml` 与实测外参，两者不可互换。

## 4. 启动顺序

前置：`parameters.bson` 里的视觉融合参数保持第九节之前的设置
（`EKF2_EV_CTRL=3`、`EKF2_HGT_REF=3`、`EKF2_GPS_CTRL=0`、`EKF2_EV_DELAY=100` 等）。

```bash
# 0) 终端 A：DDS Agent（PX4 的 uXRCE 客户端默认连 127.0.0.1:8888，domain 0）
~/Micro-XRCE-DDS-Agent/build3/MicroXRCEAgent udp4 -p 8888

# 1) 终端 B：PX4 SITL + Gazebo（双目模型）
cd ~/drone_sim
PX4_GZ_MODEL=x500_mono_cam ./start_rm27_px4.sh

# 2) 终端 C：实机同源算法链路
cd ~/drone_sim/scripts
./run_stereo_inertial_sim.sh          # 自动探测 cam_left/cam_right/imu_sensor + /clock
# 可选：./run_stereo_inertial_sim.sh --viewer
# 桥已手动起好：./run_stereo_inertial_sim.sh --no-bridge

# 3) 终端 D：行为树（严格模式，勿设 WSL_RELAX_FRESHNESS）
cd ~/drone_sim/scripts
python3 pilot_navigation_bt.py --help   # 参数与旧路径相同
```

### 4.1 起飞必须有真实平移（IMU_STEREO 的硬要求）

`LocalMapping` 在惯性初始化阶段会检查关键帧之间的相机位移：累计运动时间 <10 s 且
位移 <0.02 m 时打印 `Not enough motion for initializing. Reseting...` 并重置当前地图
（`mbBadImu`）。因此**原地悬停永远无法完成 VIBA1/VIBA2**，这不是仿真缺陷，实机同样
需要起飞后先动起来再交给 VIO。

`sim_pilot.py` 为此新增了可选激励（默认关闭，旧 STEREO 路径行为不变）：

```bash
# 起飞后 20 s 俯仰/横滚方波激励，再切 Position 档
python3 sim_pilot.py --log /tmp/pilot.csv --excite-seconds 20 \
        --excite-pitch 180 --excite-roll 120 --excite-period 2.4
```

激励是**真实 RC 杆量**（`RC_CHANNELS_OVERRIDE`），不是伪造 IMU 数据，和实机飞手
起飞后的手动晃动等价。

严格模式的判定含义：

- `pilot_navigation_bt.py` 采样时戳取 PX4 的 `timestamp_sample`（缺失/为 0 才回退
  `timestamp`）。放松模式仍用发布时戳。
- `pilot_authority.py::_reset_lease` 在严格模式下只丢弃当前数据，保留
  `source/samples`：一次时戳倒退不会自动恢复，与 Nano 基线一致。放松模式才清空
  基准以自愈。
- `LIMITS` / `SAMPLE_LIMITS` 数值未改。

## 5. 时间语义（新链路最容易出错的地方）

- Gazebo 消息头是 **sim time**；`Px4Bridge` 的 `Gate` 用 `SystemMicros()`（epoch µs）
  判断样本年龄，并要求与 PX4 `vehicle_local_position` 的 epoch 时戳相差 <150 ms。
- 节点订阅 `/clock`，用 `(接收时刻 epoch − sim)` 的**短窗口中位数**建立映射，
  默认窗口 0.25 s。窗口不能太长：仿真 RTF<1 时 sim 会持续落后 wall，2 s 窗口在
  RTF 0.9 下就有约 200 ms 偏差，足以让 Gate 判失败；用中位数而不是最小值是为了
  避免把「窗口内最早一条」当成当前偏差。
- 图像/IMU 先过 `SensorClock`（4 路：左目、右目、gyro、accel）。任一路时间跳变都会
  递增 epoch，节点据此重置当前地图并丢弃旧 IMU，避免用旧观测续期。
- IMU 与 `/clock` 订阅使用 `MutuallyExclusive` 回调组：Reentrant 会让相邻两条消息
  并发处理，后一条先拿到锁时时间戳会倒退一个采样周期，`SensorClock` 会误判为时钟
  跳变并重置 epoch。节点另外对 `<50 ms` 的 IMU 时戳倒退做了乱序丢弃。
- 左右目按源时间排队配对，默认容差 5 ms（`SYNC_TOL` 可覆盖）。桥接后的到达顺序
  可能差一帧，直接「任一侧到达就消费最新一对」会把 t 与 t−33 ms 配对后丢弃。
- **epoch 变化在已解锁飞行中会被 Gate 锁存为 `reference_or_clock_epoch_changed`，
  要求上锁才能恢复**。因此节点只在真正的时钟域跳变时才 `ResetActiveMap()`；任何
  未解锁阶段的重置都只是重新建系。

## 6. 怎么判断它真的在工作

`stereo_inertial_sim_node` 每 3 秒打印 `[stat]`，每秒打印 `[VIO_PERF]`：

```text
[stat] img L=.. R=.. imu=.. | pubs 1/1/1 | pairs=.. ok=.. lost=.. skew=0
       no_imu=0 zero_stamp=0 time_regress=0 clock_reject=.. |
       clock ready=1 epoch=.. offset=+...s jumps=0 | state=OK
```

验收要点：

1. `pubs 1/1/1`：bridge 三路发布者都可见；`pairs` 持续增长。
2. `skew=0`、`no_imu=0`：左右目同步、图像前有 IMU。
3. `clock ready=1`、`jumps=0`：`/clock` 映射稳定。
4. `state=OK` 且 `nav(valid=1 init=1 ba2=1 vel=1)`：IMU 初始化与 BA2 完成。
   **静止悬停不会完成初始化**，需要按实机流程给平移/加减速激励。
5. `ros2 topic echo /vio/navigation_health`：`navigation_ready=true` 才允许行为树
   进入 Offboard；`airborne_fault`、`requires_disarm` 必须为 false。
6. `ros2 topic hz /fmu/in/vehicle_visual_odometry`：有稳定输出（30 Hz 量级）。
7. PX4 侧 `listener vehicle_visual_odometry` / QGC 应看到 EV 位置融合。

## 7. 已知限制

- IMU_STEREO 的初始化依赖运动激励；纯悬停或固定等待不能保证完成（与实机一致）。
- 仿真 IMU 不含 bias random walk，`IMU.GyroWalk/AccWalk` 取了很小的非零值。
- 相机是理想针孔、无畸变，左右目共面行对齐；真实模组需要 rectify 与实测外参。
- `Px4Bridge` 的 DDS domain 由 `px4_vio_sim.yaml` 的 `domain_id: 0` 决定，与
  `ROS_DOMAIN_ID` 无关；仿真栈默认 domain 0，保持一致。
- `px4_vio_sim.yaml` 把 `max_sync_residual_seconds` 放宽到加载器允许的上限 0.05 s：
  Gazebo 的 RTF 常小于 1，且本机 PX4 以 `ENABLE_LOCKSTEP_SCHEDULER` 构建，hrt 跟随
  sim time 而 DDS Agent 用主机时钟，timesync 残差会周期性超过实机的 10 ms 门限。
  这是仿真传输层伪影；实机 `px4_vio_local.yaml` 仍为 0.01。
- 静止悬停时若发生过一次地图重置，IMU_STEREO 不会重新初始化（需要约 0.5 m/s² 的
  加速度变化）；实机飞行中的运动通常满足，但地面调试时不要依赖它自愈。
- 旧路径（`run_stereo_slam.sh` + `sim_vio_health.py`）仍可用于对照，但它发布的
  health 不是真实门控，不能用于实机同源验收。
- 严格模式下一次时戳倒退不会自愈，这是 Nano 基线语义；仿真若因 RTF/暂停导致
  时戳跳变，应重启节点而不是依赖自愈。

## 8. 回退

- 停用新路径：不启动 `run_stereo_inertial_sim.sh`，改用 `run_stereo_slam.sh`。
- 回退脚本改动：
  `cp ~/drone_sim/logs/real_algo_platform_20261111/backup/pilot_navigation_bt.py ~/drone_sim/scripts/`
  `cp ~/drone_sim/logs/real_algo_platform_20261111/backup/pilot_authority.py ~/drone_sim/scripts/`
- 回退构建：删除 `px4_ws/src/rm27_slam/src/stereo_inertial_sim_node.cpp` 及
  CMake 中对应目标，重新 `colcon build --packages-select rm27_slam`。
