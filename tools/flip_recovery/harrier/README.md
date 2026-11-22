# Harrier 地面翻正仿真

该入口以独立 Gazebo 分区和 PX4 实例启动 `Drone-Harrier`，使用 `gz_x500_mono_cam` 机架配置。恢复模式仅处理落地后的侧翻和倒扣。X500 使用原启动入口和参数。

## 运行

在 WSL 中从本仓库根目录运行：

```bash
# 约 90° 落稳，打开 Gazebo 并等待手动触发
bash start_rm27_px4.sh --flip-recovery

# 精确 180° 自主翻正，打开 Gazebo 并等待手动触发
bash start_rm27_px4.sh --flip-recovery --tilt 180 --axis 90

# 在 RMUC 中间机库场地启动独立的 Harrier 实例
bash start_rm27_px4.sh --flip-recovery --arena --tilt 180
```

窗口启动后，终端输出 `READY`、实际倾角和触发文件路径。新建该文件触发一次恢复，例如 `touch /tmp/harrier-flip.XXXXXX/run/TRIGGER`；以当次输出的路径为准。关闭窗口或者在运行目录创建 `STOP` 结束展示。运行证据保存在相同目录，包括 PX4 ULog、姿态真值和四路 Gazebo 电机命令。

入口默认使用 `$HOME/harrier_px4_sitl`，可通过 `HARRIER_FLIP_PX4` 指定其他已构建的 Harrier SITL 树。通过 `HARRIER_SIM_REPO` 指定模型所在的工程目录；通过 `HARRIER_FLIP_INSTANCE` 和 `HARRIER_FLIP_OUT` 指定独立实例编号及新日志目录。直接执行 `bash tools/flip_recovery/harrier/launch.sh` 运行无图形界面的单轮验证。

## 构建与控制策略

依照 [PX4 源码与构建](../../../docs/PX4_SOURCE.md)准备依赖。在仓库根目录导出独立源码树，依次应用[可逆输出桥](../sweep/sitl_bridge.diff)与 [Harrier SITL 补丁](harrier_sitl.diff)，然后构建：

```bash
px4="$HOME/harrier_px4_sitl"
python3 tools/px4/create_build_tree.py --output "$px4" --submodules
git -C "$px4" apply "$PWD/tools/flip_recovery/sweep/sitl_bridge.diff"
git -C "$px4" apply "$PWD/tools/flip_recovery/harrier/harrier_sitl.diff"
source /opt/ros/lyrical/setup.bash
make -C "$px4" px4_sitl_default
```

补丁仅用于 Harrier SITL 构建。

[Harrier 参数](profile.json)提供进入角、油门上限、进度门限和出力时限。该实例根据 PX4 姿态与陀螺仪角速度控制翻转。接近 180° 时先给一个机体俯仰方向，随后根据机身竖直轴误差控制转速。用机体倾角减小量判定进展；仅在接近正立且横滚、俯仰角速度降低后停桨。触发时倾角达到 125° 的会话使用 Harrier SDF 的 1000 rad/s 转速标度；侧躺保留 684 rad/s 标度，0.60 的油门上限仍生效。

控制器取 PX4 估计值，不读取 Gazebo 真值，不施加外力，也不改变模型姿态。Gazebo 真值独立用于试验评分。补丁只在指定的 Harrier SITL 实例和专用启动环境中启用。

Harrier 仿真对单步、有效且不超过 1° 的估计器姿态修正采用 `delta_q_reset` 判断；较大的修正、无效四元数或跳过多个计数仍中止。进入模式、落地、未解锁、出力时限、稳定判据和自动退出沿用 PX4 翻正状态机。

验证工况、通过判据和失败样例见 [Harrier 地面恢复验证](../../../docs/verification/harrier_flip_20261122/README.md)。
