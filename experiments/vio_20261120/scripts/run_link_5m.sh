#!/usr/bin/env bash
# 5 m 链路验证 [SIM-ONLY]：跳过 ORB-SLAM3 实时初始化，用 link_probe.py 提供
# 可控的视觉里程计 + health，专门验证
#   起飞 -> Position 定点 -> Offboard -> 5 m -> 悬停 -> 切回 Position -> 降落
# 不代表 VIO 精度，也不参与同源验收。
set -uo pipefail
SIM_RUN=${SIM_RUN:?必须设置 SIM_RUN}
BT_CFG=/home/gengyinghao/drone_sim/config/pilot_navigation_link_5m.json
mkdir -p "$SIM_RUN/bt"
PIDS=()
say() { echo "$(date +%T.%N) $*"; }
cleanup() {
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
  sleep 1
  pkill -f link_probe.py 2>/dev/null || true
  pkill -f truth_odom_node 2>/dev/null || true
  pkill -f pilot_navigation_bt.py 2>/dev/null || true
  pkill -f px4_square_bt.py 2>/dev/null || true
  pkill -f sim_pilot.py 2>/dev/null || true
  bash /home/gengyinghao/drone_sim/stop_sim.sh >/dev/null 2>&1
}
trap cleanup EXIT

say "停止旧仿真"
bash /home/gengyinghao/drone_sim/stop_sim.sh >/dev/null 2>&1
sleep 3
pkill -f "bin/px4" 2>/dev/null || true
sleep 1

say "PX4 SITL 冷启动（无头）"
cd /home/gengyinghao/drone_sim || exit 1
HEADLESS=1 PX4_AGENT_LOG="$SIM_RUN/agent.log" \
  bash /home/gengyinghao/drone_sim/start_px4_direct.sh >"$SIM_RUN/px4_gz.log" 2>&1 &
PIDS+=($!)
for i in $(seq 1 120); do
  grep -qa "gz_bridge" "$SIM_RUN/px4_gz.log" 2>/dev/null && break
  sleep 1
done
grep -qa "gz_bridge" "$SIM_RUN/px4_gz.log" 2>/dev/null \
  && say "gz_bridge 已就绪（等待 ${i}s）" || { say "PX4 未就绪，终止"; exit 1; }

set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source /home/gengyinghao/px4_ws_201/install/setup.bash
set -u

say "真值桥接启动（dynamic_pose/info + /clock）"
TRUTH_TOPIC_FILE="$SIM_RUN/truth_topic.txt" \
  bash /home/gengyinghao/drone_sim/scripts/start_truth_bridge.sh \
  >"$SIM_RUN/truth_bridge.log" 2>&1 &
PIDS+=($!)
for i in $(seq 1 40); do
  [[ -s "$SIM_RUN/truth_topic.txt" ]] && break
  sleep 1
done
[[ -s "$SIM_RUN/truth_topic.txt" ]] || { say "真值桥接失败，见 truth_bridge.log"; exit 1; }
TRUTH_ROS_TOPIC=$(cat "$SIM_RUN/truth_topic.txt")
say "真值话题 $TRUTH_ROS_TOPIC"
sleep 2

say "真值回灌启动（走 Px4Bridge，与 nano 同一坐标系/时间戳链）"
PX4_VIO_CFG=/home/gengyinghao/ORB_SLAM3/config/px4_vio_sim_transport.yaml
TRUTH_ROS_TOPIC="$TRUTH_ROS_TOPIC" \
  /home/gengyinghao/px4_ws_201/build/rm27_slam/truth_odom_node \
  --ros-args -p "px4_config:=$PX4_VIO_CFG" \
  -p "truth_topic:=$TRUTH_ROS_TOPIC" >"$SIM_RUN/slam.log" 2>&1 &
PIDS+=($!)
for i in $(seq 1 30); do
  grep -qa "\[PX4\] domain=" "$SIM_RUN/slam.log" 2>/dev/null && break
  sleep 1
done
grep -qa "\[PX4\] domain=" "$SIM_RUN/slam.log" 2>/dev/null \
  && say "真值回灌就绪（等待 ${i}s）" || { say "回灌节点启动失败，见 slam.log"; exit 1; }

say "遥测采样启动"
python3 "$SIM_RUN/telemetry.py" --out "$SIM_RUN/telemetry.csv" --seconds 300 \
  >"$SIM_RUN/telemetry.log" 2>&1 &
PIDS+=($!)

say "行为树启动 (mission=line distance=5.0 speed=0.25)"
env -u WSL_RELAX_FRESHNESS PILOT_LOG_ROOT="$SIM_RUN/bt" PILOT_LINK_PROBE=1 \
  python3 /home/gengyinghao/drone_sim/scripts/pilot_navigation_boot.py \
  --config "$BT_CFG" >"$SIM_RUN/bt.log" 2>&1 &
PIDS+=($!)
sleep 4

say "等待 EKF 收敛（解锁前留足估计时间）"
sleep 12

say "飞手启动（跳过初始化激励）"
python3 /home/gengyinghao/drone_sim/scripts/sim_pilot.py \
  --log "$SIM_RUN/pilot.csv" --gcs-port 14550 \
  --excite-seconds 0 --await-vio 30 --init-settle 2 --position-hold 8 \
  --takeoff-alt 0.8 --skip-ready-gate \
  >"$SIM_RUN/pilot.log" 2>&1 &
PIDS+=($!)

for i in $(seq 1 300); do
  grep -qa "state -> DONE" "$SIM_RUN/pilot.log" 2>/dev/null && break
  sleep 1
done
grep -qa "state -> DONE" "$SIM_RUN/pilot.log" 2>/dev/null \
  && say "飞手流程结束 DONE" || say "飞手未到 DONE（见 pilot.log）"
say "采集收尾"
