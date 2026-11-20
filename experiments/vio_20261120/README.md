# 原场地 VIO 候选运行环境

保存 2026-11-20 东向 5 m 候选的飞手、行为树、独立参数目录与进程回收代码。正式组 T1/T2 通过、T3 失败，整组未通过。结果与模型参数见 [联调记录](../../docs/VIO_HARRIER_20261120.md)。

## 目录

```text
vio_20261120/
├── run_round.sh            单轮入口，默认关闭逐帧诊断
├── scripts/                候选脚本；不覆盖仓库 scripts/ 生产路径
├── runner/                 隔离 PX4 启动器、运行器和 Agent 身份工具
├── reference/S_diag/       冻结 PX4 参数及 EEPROM 输入
├── tests/                  方位角与 Agent 归属回归，旧行为树仅作对照夹具
└── px4_sitl.patch          v1.17.0 的 RC 模式映射、EV 融合及 DDS 开关话题
```

## 构建前提

项目默认检出到 `~/drone_sim`，算法检出到 `~/ORB_SLAM3`。ROS 2 Lyrical 与 `~/px4_ws_201` 提供 px4_msgs 2.0.1，消息版本应满足 Odom=0、Status=1、Position=1。先按算法仓库说明构建核心、测试与 PX4 core；相机使用 `RM27_Harrier_IMU_Stereo_baseline.yaml`，不能复制实机标定或 aarch64 构建产物。

PX4 独立检出 v1.17.0。干净源码先检查再应用补丁；已经包含相同修改的工作区不要重复应用：

```bash
cd ~/drone_sim/PX4-Autopilot
git apply --check ../experiments/vio_20261120/px4_sitl.patch
git apply ../experiments/vio_20261120/px4_sitl.patch
make px4_sitl_default
```

运行器直接使用已构建的 PX4，**不负责同步 Gazebo world/model**。首次在独立 PX4 checkout 中准备资源；以下仅同步文件，不启动仿真：

```bash
cd ~/drone_sim
PX4_GZ_RES=PX4-Autopilot/Tools/simulation/gz
mkdir -p "$PX4_GZ_RES/worlds" "$PX4_GZ_RES/models/x500_mono_cam"
for f in rm27_sim/worlds/*.sdf; do
  install -m 644 "$f" "$PX4_GZ_RES/worlds/$(basename "$f").new"
  mv "$PX4_GZ_RES/worlds/$(basename "$f").new" "$PX4_GZ_RES/worlds/$(basename "$f")"
done
install -m 644 rm27_sim/models/x500_mono_cam/model.sdf "$PX4_GZ_RES/models/x500_mono_cam/model.sdf.new"
mv "$PX4_GZ_RES/models/x500_mono_cam/model.sdf.new" "$PX4_GZ_RES/models/x500_mono_cam/model.sdf"
for name in rmuc_2027 gabel_airframe; do
  test -e "$PX4_GZ_RES/models/$name" || ln -s "$PWD/rm27_sim/models/$name" "$PX4_GZ_RES/models/$name"
done
```

构建当前仓库的 ROS 节点，避免调用本机旧节点：

```bash
source /opt/ros/lyrical/setup.bash
source ~/px4_ws_201/install/setup.bash
cd ~/drone_sim
cmake -S ros2/rm27_slam -B build/ros2 -DCMAKE_BUILD_TYPE=Release   -DORB_ROOT="$HOME/ORB_SLAM3"
cmake --build build/ros2 --parallel 2
```

## 离线回归与单轮运行

```bash
cd ~/drone_sim
source /opt/ros/lyrical/setup.bash
source ~/px4_ws_201/install/setup.bash
python3 -m unittest discover -s experiments/vio_20261120/tests -p 'test_*.py' -v
# 24 项：方位角及默认行为 15 项，Agent 归属与回收 9 项

# 启动一轮东向 5 m 仿真，名称必须为新的目录名。
ORB_NODE_BIN="$PWD/build/ros2/stereo_inertial_sim_node"   bash experiments/vio_20261120/run_round.sh R1 off
```

入口固定原场地、出生位姿、5 m、0.35 m/s 与原评价门限；`off` 关闭逐帧诊断，`on` 用于专用诊断轮。启动前拒绝外来 PX4/Gazebo 实例；共享 ROS_DOMAIN_ID=0 仍需避免其他任务在运行中启动。Agent 路径可用 `XRCE_AGENT_BIN` 指定。运行输入的参数快照来自本机冻结候选，不能用于实机。

结果写入 `experiments/vio_20261120/runs/<轮名>/`。检查 `check.json`、`check.txt`、`cleanup_verify.json`、`cleanup_scan_summary.json` 与 `agent_owner.json`；程序正常退出、飞手 DONE 或落地均不单独代表任务通过。已有轮次不覆盖，正式组按预先确定的 T1→T2→T3 顺序运行，中间不修改候选。

本次仓库整理仅完成离线回归、编译及路径检查，未追加仿真飞行。恢复历史生产入口可直接使用仓库 `scripts/`，候选无须覆盖该目录。
