# Harrier 机库联调

默认场景 `rm27_world` 加载独立的 `drone_harrier` 模型，实体名为 `Drone-Harrier`。PX4 通过实体名接入；`gz_x500_mono_cam` 仍是 SITL 机型参数入口。其他历史场景保留原模型资源。

| 项目 | 世界坐标，米 | 备注 |
| --- | --- | --- |
| Harrier 出生点 | (13.116631, 7.032968, 1.9558063) | 默认机头朝 +X |
| 台面 Tag 0 | (13.142631, 7.032968, 1.9203) | 朝上，识别边长 136 mm |
| 立牌 Tag 1 | (12.826262, 7.032968, 2.035) | 朝 +X，识别边长 144 mm |

立牌绕平台中心从 +X 边移到 −X 边，牌面仍朝向平台。按默认航向停机时，两枚 Tag 都没有被机载相机识别。固定姿态复测中，前视双目转向立牌后读到 ID 1；抬高约 0.24 m 后，下视读到 ID 0。这次只验证图像和识别，没有执行自主起飞、定位交接或 5 米直线。

## 读下视画面

先启动场景。在同一 Gazebo 分区下启动桥：

```bash
source ./env.sh
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:="$PWD/config/harrier_ov9281_gz_bridge.yaml"
```

图像为 `/harrier/ov9281/image_rect`（`mono8`），内参为 `/harrier/ov9281/camera_info`。实机需重新标定内参和安装外参，并实测持续帧率与光通信误码率。

先用 [AprilTag 3](https://github.com/AprilRobotics/apriltag) 和 [apriltag_ros](https://github.com/christianrauch/apriltag_ros) 接识别，再用同一份原始数据比较 [NAPA](https://github.com/zwatx/NAPA) 的 CPU 负载；需要 GPU 分担时评估 [vkapriltag](https://github.com/yojobama/vkapriltag)。位姿解算可用 OpenCV [IPPE](https://github.com/tobycollins/IPPE) 检查单标双解。Tag 位姿还未接入 PX4。
