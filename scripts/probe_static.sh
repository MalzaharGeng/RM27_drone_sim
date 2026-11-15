#!/usr/bin/env bash
# 静态视觉探针：起飞前让飞机停在地面，只跑 SLAM 节点，看静止场景下的跟踪成功率。
# 目的：把「视觉/相机/配置问题」与「飞行段运动问题」一刀切开。
set -uo pipefail
OUT=${OUT:-/home/gengyinghao/drone_sim/logs/probe_static}
mkdir -p "$OUT"
cd /home/gengyinghao/drone_sim || exit 1

echo "1) 停旧仿真"
bash ./stop_sim.sh >/dev/null 2>&1
sleep 2

echo "2) 起 PX4（无头）"
HEADLESS=1 PX4_AGENT_LOG="$OUT/agent.log" bash ./start_px4_direct.sh >"$OUT/px4.log" 2>&1 &
PX4_PID=$!
for i in $(seq 1 120); do
  grep -qa "gz_bridge" "$OUT/px4.log" 2>/dev/null && break
  sleep 1
done
echo "   gz_bridge 就绪（${i}s）"

echo "3) 起 SLAM 节点（不启动飞手，飞机留在地面）"
cd "$OUT/../.." || exit 1
cd /home/gengyinghao/drone_sim/scripts || exit 1
ROS_SETUP=/opt/ros/lyrical/setup.bash WS=/home/gengyinghao/px4_ws \
ORB_ROOT=/home/gengyinghao/ORB_SLAM3 SYNC_TOL=0.005 \
PX4_VIO_CONFIG=transport \
  bash ./run_stereo_inertial_sim.sh --no-viewer >"$OUT/slam.log" 2>&1 &
SLAM_PID=$!

echo "4) 静止观察 50 s"
sleep 50
echo "5) 抓读数"
python3 - "$OUT/slam.log" <<'PY'
import re, sys
t = open(sys.argv[1], errors="replace").read()
st = [l for l in t.splitlines() if "[stat]" in l]
if st:
    print(st[-1][-200:])
else:
    print("no [stat] lines")
v = [l for l in t.splitlines() if "features=" in l]
if v:
    print("perf:", v[-1].split("]")[-1].strip()[:120])
PY

echo "6) 收尾"
kill $SLAM_PID 2>/dev/null
sleep 1
pkill -f stereo_inertial_sim_node 2>/dev/null
bash ./stop_sim.sh >/dev/null 2>&1
echo "done -> $OUT"
