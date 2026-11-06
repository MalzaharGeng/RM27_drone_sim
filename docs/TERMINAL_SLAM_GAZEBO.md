# RM27 无人机仿真：终端调参、SLAM 接入与 Gazebo 使用手册

配套快速运行手册：[README.md](../README.md)。本文件面向“已经能起飞，接下来要调、要接 SLAM、要用 Gazebo 工具”的场景。原则是**先给可直接粘贴的命令**，项目自带的封装脚本统一放在最后。

环境基线：

- WSL2 Ubuntu 26.04，ROS 2 Lyrical（`/opt/ros/lyrical`）
- PX4 v1.17.0 SITL，Gazebo Jetty 10.5（`gz sim` 10.5.0）
- 渲染：WSLg → Mesa D3D12 → RTX 5070 Ti
- 飞控通信：MAVSDK gRPC 4.0.5，PX4 在 UDP `14540` 收 offboard
- 场地：`rm27_sim/worlds/rm27_world.sdf`（纯场地）和 `rm27_nav_world.sdf`（加障碍块）

---

## 0. 先把三种“定位”分清

| 路径 | 数据来源 | 用途 | 现状 |
|---|---|---|---|
| 本机导航脚本 | 已知地图 + PX4 本地 NED | 按 A* 航线飞，不依赖机载传感器 | 已实飞验证 |
| Gazebo 真值 | Gazebo 模型/传感器直接给出的位姿 | 做对照组、验证坐标和控制器 | 可用，但不是 SLAM |
| 外部 SLAM/VIO | 相机/IMU/雷达 → ORB-SLAM3 → PX4 EKF2 | 无 GPS 时的定位 | 实机链路已有代码，当前 26.04 环境未完整验证 |

`rm27_nav.py` 现在走的是第一条：它不读相机，也不跑 SLAM，只把已知障碍地图和 PX4 的 `position_velocity_ned` 对上。不要把它和 SLAM 混为一谈。

---

## 1. 之前改了什么，为什么

### 1.1 根因：Gazebo 是 ENU，PX4 是 NED

Gazebo 世界坐标是 ENU：

- `x` = 东
- `y` = 北
- `z` = 上

PX4 的 `position_ned` 是 NED：

- `N` = 北
- `E` = 东
- `D` = 下

PX4 的 gz 桥在 `PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.cpp` 里做的是：

```text
(north, east) = (gz.y, gz.x)
```

早期代码把世界的 `x` 当成北，飞远端基地（世界东侧）时实际朝北飞，撞上北侧围栏，看起来就是“空气墙”。日志里 `N≈9.5` 处高度骤降就是撞墙。

现在所有 ENU→NED 换算只走一个函数：

- `scripts/nav_planner.py:27` `world_to_ned()`
- `scripts/nav_planner.py:85` `load_world_map()`
- `scripts/rm27_nav.py` 里的 `--target` 也先换成 NED 再规划

**不要再在别的文件里手写 `x - spawn_x` / `y - spawn_y`**，否则还会把轴搞反。

### 1.2 速度：0.5 m 加密 + 限速

第一版把下一个航点直接给到 11 m 外，PX4 按默认 `MPC_XY_VEL_MAX=12 m/s` 全速冲，实测到 7.1 m/s，过冲后姿态发散，从 2 m 掉到地面。

现在两层限制：

- 规划出的折线按 `STEP_M = 0.5 m` 加密，每个 setpoint 只差半米（`scripts/rm27_nav.py:55`、`scripts/nav_planner.py:126`）
- 起飞后把 PX4 参数压到 `MPC_XY_VEL_MAX=2.0`、`MPC_Z_VEL_MAX_UP=1.0`、`MPC_Z_VEL_MAX_DN=0.7`（`scripts/rm27_nav.py:364` 附近）

### 1.3 机头朝向

早期 setpoint 的 yaw 固定写成 `0.0`，机头永远朝北，所以朝东飞基地时是侧着飞的。

现在按“当前航段起点 → 目标点”算 yaw：

- `scripts/rm27_nav.py:122` `_heading_deg()`
- 北 = 0°，东 = 90°，南 = 180°，西 = -90°
- 悬停和下降阶段沿用最后一段的 yaw

### 1.4 预检与解锁

仿真刚启动时 EKF、GCS 检查还没稳定，直接 `arm()` 会返回 `COMMAND_DENIED`。现在：

- 先等 `health.is_local_position_ok` 和 `health.is_global_position_ok`
- 再等 `health.is_armable`
- `arm()` 失败会重试 15 次，每次间隔 2 s（`scripts/rm27_nav.py` 的 `_arm_with_retry()`）

### 1.5 两段式降落和“陷进地面”

场地 STL `rmuc_2027.stl` 只有视觉、没有碰撞体。远端基地前抬升平台的可视顶面在 `z≈0.318 m`，而碰撞地面 `arena_ground` 在 `z=0.019 m`，直接降落会陷进网格约 0.3 m。

现在：

1. 飞到基地前落点
2. 先下降到 `--final-hover-altitude`（默认 0.8 m）
3. 悬停 `--hold` 秒
4. 再 `land()` 落地

并且两个世界文件里都加了静态碰撞盒 `base_landing_pad`，顶面 0.318 m，补上可视平台和物理地面之间的落差：

- `rm27_sim/worlds/rm27_world.sdf:81`
- `rm27_sim/worlds/rm27_nav_world.sdf:89`

### 1.6 避障和安全中断

避障不是靠机载传感器，而是靠已知地图：

- `scripts/obstacles_rm27.json`：导航世界用的障碍
- `scripts/obstacles_rm27_plain.json`：纯场地用的障碍（只有围栏和中央高地）

规划器：

- 0.2 m 网格 A*
- 障碍先按 `MARGIN_M = 1.2 m` 膨胀
- 八邻域、禁止切角
- 最后用视线法把网格拐点压成直线段

飞行时以 5 Hz 流式发 setpoint，同时实时算到最近障碍的距离。以下任一条件触发就中断、退出 offboard 并降落：

- 离障碍小于 `MIN_CLEARANCE_M = 0.6 m`
- 高度偏离目标超过 `ALTITUDE_GUARD_M = 0.8 m`
- 单段飞行超过 `LEG_TIMEOUT_S = 120 s`

### 1.7 日志

| 文件 | 内容 |
|---|---|
| `logs/nav_positions.csv` | 每个 setpoint 周期的位置、速度、离障碍距离、航段 |
| `logs/mission.log` | 规划结果和任务控制台输出 |
| `logs/sim_console.log` | PX4 + Gazebo 的 INFO/WARN/ERROR |
| `PX4-Autopilot/build/px4_sitl_default/rootfs/log/<日期>/*.ulg` | PX4 完整 ULog |

`nav_positions.csv` 的列：

```text
t,north_m,east_m,down_m,speed_m_s,vz_m_s,clearance_m,leg,setpoint
```

---

## 2. 终端操作与调参

### 2.1 每次先做这两件事

```bash
cd ~/drone_sim
source ./env.sh
```

`env.sh` 固定了渲染路径和 ROS 环境。漏掉它，Gazebo 可能掉到 llvmpipe 软件渲染。

自检：

```bash
glxinfo -B | grep -E "Device|Accelerated"
# 期望：D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU) / Accelerated: yes
```

### 2.2 两种运行方式

**一次跑完（起仿真 → 飞 → 关仿真）**

```bash
cd ~/drone_sim

# 带障碍块的世界（默认）
./scripts/run_nav_demo.sh

# 纯场地，直接飞到敌方基地，悬停 3 秒后降落
PX4_GZ_WORLD=rm27_world ./scripts/run_nav_demo.sh --enemy-base --hold 3
```

**两个终端分步运行**

终端 A：

```bash
cd ~/drone_sim
PX4_GZ_WORLD=rm27_world ./start_rm27_px4.sh
```

等 Gazebo 窗口出现、PX4 打印 `Ready for takeoff!` 后，终端 B：

```bash
cd ~/drone_sim
source ./env.sh
python3 scripts/rm27_nav.py \
  --map scripts/obstacles_rm27_plain.json \
  --enemy-base --hold 3
```

带障碍块的世界：

```bash
# 终端 A
cd ~/drone_sim
PX4_GZ_WORLD=rm27_nav_world ./start_rm27_px4.sh

# 终端 B
cd ~/drone_sim
source ./env.sh
python3 scripts/rm27_nav.py \
  --map scripts/obstacles_rm27.json \
  --enemy-base --hold 3
```

收尾：

```bash
cd ~/drone_sim
./stop_sim.sh
```

### 2.3 任务参数速查

| 参数 | 默认 | 说明 |
|---|---|---|
| `--map` | `scripts/obstacles_rm27.json` | 障碍地图；纯场地必须换 `_plain.json` |
| `--enemy-base` | 关 | 飞到远端基地装甲板前 `base_far_front` |
| `--own-base` | 关 | 飞到近端基地中心 `base_near` |
| `--target X,Y` | 无 | 飞到 Gazebo 世界坐标；可重复多次 |
| `--altitude` | 地图里的 `altitude_m`（2.0） | 巡航高度，米 |
| `--hold` | `3.0` | 最后落点低空悬停秒数；0 表示不悬停 |
| `--final-hover-altitude` | `0.8` | 降落前低空悬停高度；0 表示跳过下降 |
| `--log` | `logs/nav_positions.csv` | 飞行 CSV |
| `--dry-run` | 关 | 只规划、不连仿真 |

`--target` 给的是 **Gazebo ENU 世界坐标**，不是 PX4 NED：

```bash
# 飞到场中央偏东
python3 scripts/rm27_nav.py --target 18,8 --dry-run

# 连续飞两个点，最后悬停 5 秒
python3 scripts/rm27_nav.py --target 18,3 --target 14,13 --hold 5
```

### 2.4 改速度

优先级从高到低：

1. PX4 控制台实时改

   ```text
   param set MPC_XY_VEL_MAX 1.5
   param set MPC_Z_VEL_MAX_UP 0.8
   param set MPC_Z_VEL_MAX_DN 0.6
   ```

2. 改 `scripts/rm27_nav.py` 里起飞后设置的参数（`MPC_XY_VEL_MAX` 等）
3. 改 `STEP_M`（默认 0.5 m）。改小更慢更稳，改大更快但更容易过冲

`SPEED_HINT_MPS` 只是注释性常量，当前不参与控制；别以为改它就会变速。

### 2.5 改高度和悬停

```bash
# 巡航 3 m，落点前降到 0.5 m，悬停 5 s
python3 scripts/rm27_nav.py --enemy-base \
  --altitude 3.0 --final-hover-altitude 0.5 --hold 5

# 不要在落点前下降，直接在当前高度悬停后降落
python3 scripts/rm27_nav.py --enemy-base --final-hover-altitude 0 --hold 0
```

### 2.6 改地图和出生点

地图 JSON 是世界坐标（x=东、y=北），障碍是轴对齐矩形：

```json
{"name": "block_a", "x": [8.0, 10.0], "y": [4.0, 6.0]}
```

**改地图必须同步改两处**：

1. `scripts/obstacles_rm27.json` 或 `obstacles_rm27_plain.json`
2. `rm27_sim/models/nav_obstacles/model.sdf` 里对应的 collision/visual

否则规划器会绕一个不存在的障碍，或者撞上一个地图里没有的障碍。

改出生点也要同步两处：

```bash
export PX4_GZ_MODEL_POSE=5,6,0.5,0,0,0   # env.sh
```

```json
"spawn_world_xy": [5.0, 6.0]              // 地图 JSON
```

### 2.7 改安全阈值

都在 `scripts/rm27_nav.py` 顶部：

| 常量 | 默认 | 含义 |
|---|---|---|
| `MARGIN_M` | 1.2 | 规划时障碍膨胀半径 |
| `MIN_CLEARANCE_M` | 0.6 | 低于此距离立即中断 |
| `ALTITUDE_GUARD_M` | 0.8 | 高度偏离超过此值立即中断 |
| `ARRIVE_TOLERANCE_M` | 0.3 | 到点判定半径 |
| `LEG_TIMEOUT_S` | 120 | 单段超时 |
| `DESCENT_RATE_MPS` | 0.4 | 最后下降速度 |

### 2.8 PX4 控制台

终端 A 里 PX4 启动后会进入 `pxh>` 提示符，可直接输入：

```text
status
commander arm
commander takeoff
commander land
commander mode position
commander mode offboard

param show MPC_XY_VEL_MAX
param set MPC_XY_VEL_MAX 1.5
param show EKF2_EV_CTRL
param save

listener vehicle_local_position
listener estimator_status_flags
listener battery_status
listener sensor_combined
```

注意：

- `commander takeoff` 需要先解锁
- 任务脚本运行期间不要再手动切模式，否则 offboard 会被踢出
- `param save` 后需要重启才确保生效；调参阶段可以先不保存

### 2.9 看日志

```bash
cd ~/drone_sim

# 最近 40 行，按列对齐
column -s, -t < logs/nav_positions.csv | tail -40

# 最大水平速度
cut -d, -f5 logs/nav_positions.csv | sort -n | tail -1

# 最小离障距离（空行是悬停阶段）
cut -d, -f7 logs/nav_positions.csv | grep -v '^$' | sort -n | head -1

# 任务输出
tail -50 logs/mission.log

# PX4/Gazebo 报错
grep -E 'ERROR|WARN|Fatal' logs/sim_console.log | tail -40
```

### 2.10 常见异常

| 现象 | 先查 |
|---|---|
| 飞机朝错误方向飞/撞“空气墙” | 是不是又把世界 x 当北了；只走 `world_to_ned()` |
| 一起飞就冲出去、姿态发散 | `MPC_XY_VEL_MAX` 是否还是 12；`STEP_M` 是否被改大 |
| 降落后陷进地面 | 世界文件是否有 `base_landing_pad`；是否重启了仿真 |
| 到点后侧着飞 | yaw 是否按航段算，而不是固定 0 |
| `arm denied` | PX4 是否还没 `Ready for takeoff`；EKF/GCS 是否就绪 |
| 连不上 PX4 | `ss -lunp \| grep 14540`；仿真是否真的在跑 |
| 规划报 `goal is inside an inflated obstacle` | 目标点离障碍太近，或 `MARGIN_M` 太大 |
| Gazebo 只有几帧 | `source env.sh`；`glxinfo -B` 是否掉到 llvmpipe |
| 世界能开但场地是空的 | `model://rmuc_2027` 没同步到 PX4 models 目录 |

---

## 3. 接入 SLAM

### 3.1 目标链路

仿真里推荐先把链路拆成四段，逐段验收：

```text
Gazebo 相机/IMU
      │  gz topic
      ▼
ros_gz_bridge
      │  ROS 2 topic
      ▼
SLAM / VIO（ORB-SLAM3 或其它）
      │  位姿/速度
      ▼
PX4 EKF2（外部视觉）
```

任何一段没通，都不要直接解锁飞行。

### 3.2 让 Gazebo 带传感器

PX4 的 gz 机型在 `PX4-Autopilot/Tools/simulation/gz/models/`，常用的有：

| make 目标 | 传感器 |
|---|---|
| `gz_x500` | 基础 IMU/气压/磁罗盘/GPS |
| `gz_x500_mono_cam` | 单目相机（PX4 默认 1280×960 / HFOV 1.74 rad）；本项目已覆盖为双目 1360×960 / HFOV 1.590 / 50 mm 基线 |
| `gz_x500_depth` | RGB 1920×1080 + 深度 640×480（OakD-Lite） |
| `gz_x500_lidar_2d` | 2D 激光（1080 线，±135°） |
| `gz_x500_lidar_front` | 前向单点/短距激光 |
| `gz_x500_vision` | 只加 OdometryPublisher，输出真值里程计 |

手动启动相机机型（`start_rm27_px4.sh` 当前固定 `gz_x500`，所以这里直接调 make）：

```bash
cd ~/drone_sim
source ./env.sh

# 把世界和模型同步到 PX4 的搜索路径
mkdir -p PX4-Autopilot/Tools/simulation/gz/worlds PX4-Autopilot/Tools/simulation/gz/models
cp --remove-destination -f rm27_sim/worlds/*.sdf PX4-Autopilot/Tools/simulation/gz/worlds/
for d in rm27_sim/models/*/; do
  ln -sfn "$PWD/${d%/}" "PX4-Autopilot/Tools/simulation/gz/models/$(basename "$d")"
done

cd PX4-Autopilot
PX4_GZ_WORLD=rm27_world make px4_sitl gz_x500_mono_cam
```

深度相机把最后一行换成 `gz_x500_depth`，2D 雷达换成 `gz_x500_lidar_2d`。

启动后先确认 Gazebo 侧真的有传感器话题：

```bash
gz topic -l | grep -E 'camera|image|depth|imu|lidar|scan|odometry'
```

模型实例名通常带 `_0` 后缀，例如 `x500_mono_cam_0`。具体以 `gz topic -l` 输出为准，不要硬背。

### 3.3 把 Gazebo 话题桥到 ROS 2

先 source ROS：

```bash
source /opt/ros/lyrical/setup.bash
```

单目相机 + IMU + 时钟（把世界名和模型名换成实际值）：

```bash
ros2 run ros_gz_bridge parameter_bridge \
  "/world/rm27_world/model/x500_mono_cam_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image" \
  "/world/rm27_world/model/x500_mono_cam_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo" \
  "/world/rm27_world/model/x500_mono_cam_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU" \
  "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock"
```

2D 雷达：

```bash
ros2 run ros_gz_bridge parameter_bridge \
  "/world/rm27_world/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"
```

深度相机的话题名带 `<topic>depth_camera</topic>`，先用 `gz topic -l | grep depth` 确认，再按同样格式桥接。

检查：

```bash
ros2 topic list | grep -E 'image|imu|scan|clock|depth'
ros2 topic hz /world/rm27_world/model/x500_mono_cam_0/link/camera_link/sensor/imager/image
ros2 topic echo --once /world/rm27_world/model/x500_mono_cam_0/link/base_link/sensor/imu_sensor/imu
ros2 run rqt_image_view rqt_image_view
```

话题名太长时，用 `ros_gz_bridge` 的 YAML 配置做重命名，比在命令行里塞一堆 remap 清楚：

```yaml
# bridge.yaml
- ros_topic_name: "/camera/image"
  gz_topic_name: "/world/rm27_world/model/x500_mono_cam_0/link/camera_link/sensor/imager/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/camera_info"
  gz_topic_name: "/world/rm27_world/model/x500_mono_cam_0/link/camera_link/sensor/imager/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS
```

```bash
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:=$HOME/drone_sim/bridge.yaml
```

### 3.4 ORB-SLAM3 侧

本机已经有一份 ORB-SLAM3：

- 源码：`~/ORB_SLAM3`
- 实机 D435i 操作：`~/ORB_SLAM3/操作手册.md`
- PX4 VIO 桥：`~/ORB_SLAM3/tests/PX4_VIO.md`
- 摄像头直连示例：`~/slam_webcam_demo/webcam_orb_slam3.cpp`

要分清两条链路：

1. **实机 D435i + PX4**：直接用 ORB-SLAM3 现有的 RealSense 入口和 PX4 桥。按 `tests/PX4_VIO.md` 编译、填 `R_BI`、跑 Agent。该文档基线是 Ubuntu 22.04 / ROS 2 Humble；当前 26.04 / Lyrical 没有完整验证，`px4_msgs` 和 Agent 版本要对齐。
2. **Gazebo 相机 + PX4**：ORB-SLAM3 现有入口读的是 RealSense 或数据集，**不会自动订阅 ROS 2 话题**。需要写一个小的 C++ 节点，订阅桥接后的 `sensor_msgs/Image` + `sensor_msgs/Imu`，把图像指针交给 `ORB_SLAM3::System::TrackMonocular()` / `TrackRGBD()` / `TrackStereo()`。`slam_webcam_demo/webcam_orb_slam3.cpp` 就是现成的调用范例。

Gazebo 相机的内参可以直接从 SDF 算，不要照抄 TUM 参数：

```text
fx = fy = (width / 2) / tan(horizontal_fov / 2)
cx = width / 2
cy = height / 2
k1, k2, p1, p2, k3 = 0        # Gazebo 相机默认无镜头畸变
```

几个现成机型的数值：

| 机型 | 分辨率 | HFOV | fx≈ |
|---|---|---|---|
| `x500_mono_cam`（本项目覆盖后） | 1360×960 | 1.590 rad | 667.0 |
| `x500_mono_cam`（PX4 默认） | 1280×960 | 1.74 rad | 538.7 |
| `x500_depth` RGB | 1920×1080 | 1.204 rad | 1397.0 |
| `x500_depth` 深度 | 640×480 | 1.274 rad | 431.9 |

外参也要自己填。Gazebo 相机系是 `+X 前、+Y 左、+Z 上`，ORB-SLAM3 相机光学系是 `+Z 前、+X 右、+Y 下`。两者之间不是单位阵，常见的旋转是：

```text
R_optical_body = [ 0 -1  0
                   0  0 -1
                   1  0  0 ]
```

填到 ORB-SLAM3 的 `T_b_c` / `Tbc` 前，先用一帧已知方向的小样本验证符号，别直接抄。

### 3.5 把位姿喂给 PX4

#### 3.5.1 uXRCE-DDS（和 ORB-SLAM3 现有桥一致）

ORB-SLAM3 的 PX4 桥发布：

```text
/fmu/in/vehicle_visual_odometry
```

PX4 侧对应：

```text
EKF2_EV_CTRL
EKF2_HGT_REF
EKF2_EV_DELAY
EKF2_EV_POS_X/Y/Z
```

SITL 启动日志里已经能看到：

```text
INFO  [uxrce_dds_client] init UDP agent IP:127.0.0.1, port:8888
```

所以只要在另一个终端起 Agent，并让 ORB-SLAM3 桥连上即可：

```bash
MicroXRCEAgent udp4 -p 8888
```

具体编译和参数以 `~/ORB_SLAM3/tests/PX4_VIO.md` 为准。注意该文档的版本约束：

- `px4_msgs` 用 PX4 1.17 对应分支
- Agent 用文档固定版本
- 不要把 Agent 的 `lib` 加进 ORB-SLAM3 的 `LD_LIBRARY_PATH`
- `config/px4_vio.yaml` 里 `extrinsics_valid: 0` 时只回读状态，不发定位

#### 3.5.2 MAVSDK mocap（仿真里更省事）

当前仿真走的是 MAVSDK gRPC（`udpin://0.0.0.0:14540`），它自带 `mocap` 插件，可以直接发 `VISION_POSITION_ESTIMATE`，不需要额外装 ROS 2 的 PX4 消息包：

```python
import asyncio
from mavsdk_grpc import System
from mavsdk_grpc.mocap import (
    Mocap,
    VisionPositionEstimate,
    PositionBody,
    AngleBody,
    Covariance,
)

async def main():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    estimate = VisionPositionEstimate(
        time_usec=0,
        position_body=PositionBody(
            x_m=0.0,      # NED: 北
            y_m=0.0,      # NED: 东
            z_m=-1.0,     # NED: 下，-1 表示离地 1 m
        ),
        angle_body=AngleBody(
            roll_rad=0.0,
            pitch_rad=0.0,
            yaw_rad=0.0,
        ),
        pose_covariance=Covariance([0.0] * 36),
        reset_counter=0,
    )
    await drone.mocap.set_vision_position_estimate(estimate)

asyncio.run(main())
```

PX4 侧至少要配：

```text
param set EKF2_EV_CTRL 7
param set EKF2_HGT_REF 3
param set EKF2_EV_DELAY 0
param set EKF2_EV_POS_X 0
param set EKF2_EV_POS_Y 0
param set EKF2_EV_POS_Z 0
param set COM_ARM_WO_GPS 1
```

如果要做纯视觉、不用 GPS，再关 GPS：

```text
param set EKF2_GPS_CTRL 0
param save
```

`EKF2_EV_CTRL=7` 是“水平位置 + 垂直位置 + 3D 速度”；要融合视觉 yaw 时再加 8（即 15）。不同 PX4 版本的位定义可能变，改完用 `param show EKF2_EV_CTRL` 和 `listener estimator_status_flags` 确认。

### 3.6 坐标、时间、外参

这是最容易出错的三件事：

1. **坐标**：SLAM 的地图系通常是“首帧 IMU 系”，不是 NED。发送前要把它旋转到 PX4 能接受的本地 NED。别把 ORB-SLAM3 的相机位姿原样塞给 PX4。
2. **时间**：外部视觉要求时间戳单调、不能倒流。ROS 2 桥和 SLAM 节点用同一时钟；仿真里 `/clock` 要一起桥接。PX4 对超时很敏感，断流会停发或退出。
3. **外参**：三个东西不是一回事：
   - 相机 ↔ IMU：ORB-SLAM3 的 `IMU.T_b_c1`
   - IMU ↔ 机体 FRD：`config/px4_vio.yaml` 的 `R_BI`
   - IMU 相对重心的平移：PX4 的 `EKF2_EV_POS_X/Y/Z`

   不要在三处重复填同一个平移。

### 3.7 建议的验收顺序

1. Gazebo 能出图/出 IMU：`gz topic -l` 有话题
2. ROS 2 能收图：`ros2 topic hz` 稳定 20–30 Hz
3. ORB-SLAM3 能初始化：日志进入 `OK` / `Tracking`，不是一直 `NOT_INITIALIZED`
4. 只发速度或只发位置，确认 PX4 的 `estimator_status_flags` 里 EV 标志变化
5. 地面静置，检查位置不漂；手动平移 1 m，检查 NED 方向对
6. 解锁、低空悬停、再切 offboard

### 3.8 SLAM 常见坑

| 现象 | 可能原因 |
|---|---|
| ORB-SLAM3 一直不初始化 | 画面纹理不足；相机内参不对；没动起来（单目需要平移） |
| 位姿尺度不对 | 单目尺度不可观；改用双目/IMU/RGB-D |
| 飞控位置突然跳 | SLAM 重置/重定位；时间戳倒退；桥接丢帧 |
| 方向整体转 90° | 相机光学系 ↔ 机体系的旋转填错 |
| 高度方向反了 | NED 的 `z` 向下；SLAM 通常向上 |
| PX4 不采纳视觉 | `EKF2_EV_CTRL` 没配；GPS 还在抢；协方差/超时不合规 |
| uXRCE 连不上 | Agent 没起、端口不是 8888、版本不匹配 |

### 3.9 2D 雷达 SLAM 备选

如果只是想在仿真里跑一个 2D 栅格 SLAM，可以用 `gz_x500_lidar_2d`：

```bash
cd ~/drone_sim/PX4-Autopilot
PX4_GZ_WORLD=rm27_world make px4_sitl gz_x500_lidar_2d
```

桥出 `/scan` 后接 `slam_toolbox`（需额外安装）或 `cartographer`。这条链路需要有人发布 `map → odom → base_link` 的 TF，PX4 本身不提供，需要自己补 `robot_state_publisher` 和静态 TF。它适合做 2D 建图演示，不适合直接替代 PX4 的 EKF2 外部视觉。

---

## 4. Gazebo 各种功能

### 4.1 视角操作

在 3D 视图里：

- 左键拖动：平移
- 滚轮：缩放
- 中键拖动，或 `Shift` + 左键拖动：旋转
- 左键单击：选中模型
- 右键模型 → `Move to`：视角居中
- 右键模型 → `Follow` / `Track`：视角跟随无人机

右侧面板常用：

- `Entity Tree`：模型/链接树
- `Component Inspector`：选中实体的位姿、惯性等
- `Transform Control`：拖拽移动/旋转模型
- `World Control`：暂停、单步、重置
- `View Angle`：俯视、跟随等预设
- `Image Display`：显示相机话题
- `Topic Viewer` / `Plotting`：看话题和曲线

### 4.2 启动方式

```bash
cd ~/drone_sim
source ./env.sh

# 直接开世界（GUI + server）
gz sim -r rm27_sim/worlds/rm27_world.sdf

# 无界面 server
gz sim -s -r rm27_sim/worlds/rm27_world.sdf

# 先开 server，再单独开 GUI 客户端
gz sim -s -r rm27_sim/worlds/rm27_nav_world.sdf
gz sim -g

# 指定 GUI 配置
gz sim -r --gui-config rm27_sim/worlds/rm27_gui.config rm27_sim/worlds/rm27_world.sdf
```

注意：直接 `gz sim` 启动的是“没有 PX4 的世界”。要飞 PX4，仍然用 `start_rm27_px4.sh` 或 `make px4_sitl`。

### 4.3 模型和实体

```bash
# 列出模型
gz model --list

# 看某个模型的位姿
gz model -m x500_0 --pose

# 看所有链接
gz model -m x500_0 -l

# 看某个链接上的传感器
gz model -m x500_0 -l base_link -s
```

### 4.4 话题

```bash
# 列出话题
gz topic -l

# 看话题类型和发布者
gz topic -i -t /world/rm27_world/model/x500_0/link/base_link/sensor/imu_sensor/imu

# 回显一条
gz topic -e -t <topic> -n 1

# 看频率
gz topic -f -t <topic>

# 发布一条测试消息
gz topic -t /foo -m gz.msgs.StringMsg -p 'data: "hello"'
```

常用话题：

```text
/world/<world>/clock
/world/<world>/model/<model>/link/base_link/sensor/imu_sensor/imu
/world/<world>/model/<model>/link/base_link/sensor/magnetometer_sensor/magnetometer
/world/<world>/model/<model>/link/base_link/sensor/air_pressure_sensor/air_pressure
/world/<world>/model/<model>/link/base_link/sensor/navsat_sensor/navsat
```

### 4.5 服务：暂停、单步、重置、设位姿

```bash
# 列出服务
gz service -l

# 看一个服务的请求/响应类型
gz service -i -s /world/rm27_world/control

# 暂停
gz service -s /world/rm27_world/control \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 2000 --req 'pause: true'

# 继续
gz service -s /world/rm27_world/control \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 2000 --req 'pause: false'

# 单步
gz service -s /world/rm27_world/control \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 2000 --req 'step: true'

# 连续步进 10 步
gz service -s /world/rm27_world/control \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 2000 --req 'multi_step: 10'

# 重置整个世界
gz service -s /world/rm27_world/control \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 2000 --req 'reset: {all: true}'

# 设置模型位姿
gz service -s /world/rm27_world/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req 'name: "x500_0", position: {x: 5, y: 6, z: 2}, orientation: {w: 1}'
```

重置世界不会重启 PX4，飞控可能仍处于解锁/offboard 状态。要干净重来，先 `./stop_sim.sh` 再启动。

### 4.6 参数

```bash
# 列出参数注册表
gz param -l

# 读取
gz param -g -n <name>

# 设置
gz param -s -n <name> -t <type> -v <value>
```

具体类型取决于插件注册的参数；不确认时先 `-l` 看名字。

### 4.7 录制和回放

```bash
cd ~/drone_sim

# 录制 Gazebo Transport 话题
gz log record --file logs/flight.tlog

# 只录匹配的话题
gz log record --file logs/camera.tlog --pattern '.*image.*'

# 回放
gz log playback --file logs/flight.tlog

# 快放
gz log playback --file logs/flight.tlog -f
```

`gz sim` 也可以直接录状态和 console：

```bash
gz sim -r --record --record-path ~/drone_sim/logs/gz_record \
  rm27_sim/worlds/rm27_world.sdf
```

PX4 自己的 ULog 在：

```text
PX4-Autopilot/build/px4_sitl_default/rootfs/log/<日期>/*.ulg
```

### 4.8 传感器显示和 GUI 插件

```bash
# 列出所有 GUI 插件
gz gui -l

# 单独打开一个插件窗口
gz gui -s ImageDisplay
gz gui -s TopicViewer
gz gui -s Plotting
```

相机画面用 `Image Display`，在插件里填 Gazebo 图像话题。不要用 `gz topic -e` 直接回显图像，二进制会刷屏。

### 4.9 SDF、插件、Fuel

```bash
# 检查 SDF 语法
gz sdf --check rm27_sim/worlds/rm27_world.sdf

# 打印展开后的 SDF
gz sdf --print rm27_sim/worlds/rm27_world.sdf

# 看模型/链接关系
gz sdf --graph rm27_sim/worlds/rm27_world.sdf

# 查看插件信息
gz plugin -i /path/to/libplugin.so

# 消息定义
gz msg --list
gz msg --info=gz.msgs.WorldControl

# Fuel 资源
gz fuel list
```

### 4.10 渲染和物理

```bash
# 指定渲染引擎和 API
gz sim -r --render-engine ogre2 --render-engine-api-backend opengl \
  rm27_sim/worlds/rm27_world.sdf

# 指定物理引擎
gz sim -r --physics-engine gz-physics-dartsim-plugin \
  rm27_sim/worlds/rm27_world.sdf
```

世界文件里的关键项：

```xml
<physics name="default_physics" type="dart">
  <max_step_size>0.004</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

改 `max_step_size` 会影响稳定性和实时倍率；改 `real_time_factor` 会影响仿真快慢，但不会改变控制逻辑。

### 4.11 Gazebo 常见坑

- **世界必须同步到 PX4 的 worlds 目录**：PX4 只认 `PX4-Autopilot/Tools/simulation/gz/worlds/<world>.sdf`
- **模型必须能被 `GZ_SIM_RESOURCE_PATH` 找到**：PX4 会覆盖这个变量，所以要把 `rm27_sim/models/*` 软链到 PX4 的 models 目录
- **不要无脑 `cp` 硬链接文件**：`rm27_world.sdf` 可能是硬链接，`cp` 会报 `are the same file`
- **WSLg 的 OpenGL**：必须 `source env.sh`，否则可能选到 AMD 核显或 llvmpipe
- **GUI 卡顿**：先关 shadows、降分辨率，或改用 `-s` 无界面跑数据
- **重置后 PX4 状态不一致**：世界重置不等于飞控重置；干净重来请重启仿真

---

## 5. 脚本清单（最后）

| 脚本/文件 | 作用 | 常用命令 |
|---|---|---|
| `env.sh` | 渲染、ROS、世界、出生点环境变量 | `source ./env.sh` |
| `start_rm27_px4.sh` | 同步世界/模型，预检 GPU，启动 PX4 SITL + Gazebo | `PX4_GZ_WORLD=rm27_world ./start_rm27_px4.sh` |
| `stop_sim.sh` | 杀掉 PX4、gz-sim 残留进程 | `./stop_sim.sh` |
| `scripts/run_nav_demo.sh` | 一键起仿真、飞任务、关仿真 | `./scripts/run_nav_demo.sh --enemy-base --hold 3` |
| `scripts/rm27_nav.py` | 带避障的航点任务主程序 | `python3 scripts/rm27_nav.py --enemy-base --hold 3` |
| `scripts/nav_planner.py` | 纯标准库 A* 规划器、ENU→NED 换算 | 被 `rm27_nav.py` 调用 |
| `scripts/rm27_waypoints.py` | 最早的方形航点示例，无避障 | `python3 scripts/rm27_waypoints.py` |
| `scripts/test_nav_planner.py` | 规划器单元测试（13 项） | `python3 scripts/test_nav_planner.py` |
| `scripts/obstacles_rm27.json` | 导航世界障碍和目标点 | 改地图时同步 SDF |
| `scripts/obstacles_rm27_plain.json` | 纯场地障碍（围栏 + 中央高地） | `rm27_world` 用这份 |
| `rm27_sim/worlds/rm27_world.sdf` | 纯场地世界 | 可加 `base_landing_pad` |
| `rm27_sim/worlds/rm27_nav_world.sdf` | 场地 + 导航障碍块 | 默认导航世界 |
| `rm27_sim/worlds/rm27_gui.config` | Gazebo GUI 布局 | `--gui-config` 使用 |
| `rm27_sim/models/rmuc_2027/` | RM27 场地模型 | 只有视觉，无碰撞 |
| `rm27_sim/models/nav_obstacles/` | 3 m 高障碍块 | 与地图 JSON 对应 |

最常用的三条：

```bash
cd ~/drone_sim

# 1. 先规划，不碰仿真
python3 scripts/rm27_nav.py --enemy-base --dry-run

# 2. 一键飞敌方基地
PX4_GZ_WORLD=rm27_world ./scripts/run_nav_demo.sh --enemy-base --hold 3

# 3. 分步跑时，终端 B 的命令
source ./env.sh
python3 scripts/rm27_nav.py --map scripts/obstacles_rm27_plain.json --enemy-base --hold 3
```

---

## 6. 文件索引

```text
~/drone_sim
├── env.sh
├── start_rm27_px4.sh
├── stop_sim.sh
├── PX4-Autopilot/                      PX4 v1.17.0 源码与 build
├── rm27_sim/
│   ├── worlds/
│   │   ├── rm27_world.sdf              纯场地
│   │   ├── rm27_nav_world.sdf          场地 + 障碍块
│   │   └── rm27_gui.config             GUI 布局
│   └── models/
│       ├── rmuc_2027/                  RM27 场地（视觉）
│       └── nav_obstacles/              3 m 障碍块（带碰撞）
├── scripts/
│   ├── rm27_nav.py
│   ├── nav_planner.py
│   ├── rm27_waypoints.py
│   ├── test_nav_planner.py
│   ├── run_nav_demo.sh
│   └── obstacles_rm27*.json
├── logs/
│   ├── nav_positions.csv
│   ├── mission.log
│   └── sim_console.log
└── docs/
    ├── README.md
    └── TERMINAL_SLAM_GAZEBO.md        本文件
```

相关但不在本仓库：

- `~/ORB_SLAM3`：ORB-SLAM3 源码与 PX4 VIO 桥
- `~/ORB_SLAM3/tests/PX4_VIO.md`：PX4 外部视觉接入细节
- `~/slam_webcam_demo`：ORB-SLAM3 摄像头直连示例
