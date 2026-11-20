#!/usr/bin/env bash
# 启动无头仿真 + bridge，抓一帧左目图像，然后停掉。
set -uo pipefail
OUT=${OUT:-/home/gengyinghao/drone_sim/logs/grab}
mkdir -p "$OUT"
cd /home/gengyinghao/drone_sim || exit 1
source ./env.sh >/dev/null 2>&1

echo "1) 停止旧仿真"
bash ./stop_sim.sh >/dev/null 2>&1
sleep 2

echo "2) 启动 PX4 + Gazebo（无头）"
HEADLESS=1 bash ./start_px4_direct.sh >"$OUT/px4.log" 2>&1 &
PX4_PID=$!
sleep 25

echo "3) 启动 ros_gz_bridge（只桥相机）"
set +u
source /opt/ros/lyrical/setup.bash >/dev/null 2>&1
source /home/gengyinghao/px4_ws/install/setup.bash >/dev/null 2>&1
set -u

LEFT=/world/rm27_world/model/Drone-Harrier/link/cam_left_link/sensor/cam_left/image
ros2 run ros_gz_bridge parameter_bridge "$LEFT@sensor_msgs/msg/Image[gz.msgs.Image" \
  >"$OUT/bridge.log" 2>&1 &
BR_PID=$!
sleep 8

echo "4) 抓帧"
timeout 40 python3 /home/gengyinghao/drone_sim/scripts/grab_frame.py "$LEFT" "$OUT/frame_left.png" \
  >"$OUT/grab.log" 2>&1
echo "   grab rc=$?"
cat "$OUT/grab.log" | tail -3

echo "5) 收尾"
kill $BR_PID $PX4_PID 2>/dev/null
sleep 2
bash ./stop_sim.sh >/dev/null 2>&1
ls -la "$OUT"/frame_left.png* 2>/dev/null || echo "未生成图像"
