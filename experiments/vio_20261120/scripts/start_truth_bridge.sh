#!/usr/bin/env bash
# 只桥接 Gazebo 真值位姿 + /clock，供 link_probe.py 使用 [SIM-ONLY]。
# 刻意不启动图像/IMU 桥和 SLAM 节点：链路验证不需要实时初始化。
set -o pipefail
set +u
source /opt/ros/lyrical/setup.bash
source /home/gengyinghao/px4_ws_201/install/setup.bash
set -u

TOPIC_FILE=${TRUTH_TOPIC_FILE:?必须设置 TRUTH_TOPIC_FILE}
T=""
# 用 /world/<world>/dynamic_pose/info：它的 [0] 就是机体模型根的世界位姿，
# 且随飞行更新（实测 anchor 与位移都能对上）。注意不要换用 pose/info——
# pose/info 的 [0] 是某个静态实体 (0,0,0)，恒定不动，会让 odom 一直报
# (0,0,0)，EKF 把飞机钉在原点、机体被推着乱飞。
# 也不要试图按 child_frame_id 挑机体：ros_gz_bridge 转 TFMessage 后
# 所有 child_frame_id 都是空串。
for _ in $(seq 1 60); do
  T=$(gz topic -l 2>/dev/null | grep "dynamic_pose/info" | head -1)
  [[ -n "$T" ]] && break
  sleep 1
done
if [[ -z "$T" ]]; then
  echo "TRUTH_TOPIC not found" >&2
  exit 1
fi
printf '%s' "$T" > "$TOPIC_FILE"
echo "TRUTH: $T"
exec ros2 run ros_gz_bridge parameter_bridge \
  "$T@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V" \
  "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
