#!/usr/bin/env bash
# 5 m 链路验证：真实 ORB-SLAM3 IMU_STEREO（不是真值回灌）
#
# 链路：Gazebo 双目+IMU -> stereo_inertial_sim_node -> Px4Bridge -> EKF2
#       飞手正弦姿态激励 20 s 完成视觉惯性初始化，再切 Position 交 Offboard 走 5 m。
#
# 与 run_link_5m.sh 的区别：
#   run_link_5m.sh  用 truth_odom_node 把真值喂给同一个 Bridge，只验链路，不验 SLAM。
#   本脚本           跑真正的 stereo_inertial_sim_node，被验收的是 ORB-SLAM3 本身。
#
# 边界：相机内参/基线/外参按仿真几何取值，不适用于实机。
#       PX4_VIO_CONFIG 可由外部传入（strict|transport|绝对路径），默认 transport；
#       实际配置文件路径与 md5 记录在 run.log。transport 只补偿 timesync/样本年龄
#       两项 [SIM-ONLY]，不得登记为实机同源验收。
#       本入口只跑链路，不产出验收结论；验收用 run_profile.sh + check_run.py。
set -uo pipefail
SIM_RUN=${SIM_RUN:?必须设置 SIM_RUN}
BT_CFG=/home/gengyinghao/drone_sim/config/pilot_navigation_link_5m.json
SCRIPTS=/home/gengyinghao/drone_sim/scripts
ORB_ROOT=/home/gengyinghao/ORB_SLAM3
: "${PX4_VIO_CONFIG:=transport}"
export PX4_VIO_CONFIG
case "$PX4_VIO_CONFIG" in
  transport) VIO_CFG="$ORB_ROOT/config/px4_vio_sim_transport.yaml" ;;
  strict)    VIO_CFG="$ORB_ROOT/config/px4_vio_sim.yaml" ;;
  *)         VIO_CFG="$PX4_VIO_CONFIG" ;;
esac
if [[ ! -f "$VIO_CFG" ]]; then
  echo "配置文件不存在: $VIO_CFG" >&2; exit 2
fi
mkdir -p "$SIM_RUN/bt"
PIDS=()
say() { echo "$(date +%T.%N) $*"; }
cleanup() {
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
  sleep 1
  pkill -f stereo_inertial_sim_node 2>/dev/null || true
  pkill -f run_stereo_inertial_sim.sh 2>/dev/null || true
  pkill -f pilot_navigation_bt.py 2>/dev/null || true
  pkill -f read_ekf_flags.py 2>/dev/null || true
  pkill -f sim_pilot.py 2>/dev/null || true
  bash /home/gengyinghao/drone_sim/stop_sim.sh >/dev/null 2>&1

  # [SIM-ONLY] 收尾：只保留最近 N 轮仿真日志，防止每轮约 700 MB 的
  # px4_gz.log 累积吃满磁盘。只作用于本机 WSL ~/drone_sim/logs；
  # 不触碰 Nano、不进实机配置、不改任何 SLAM/门控口径。
  # KEEP_RUNS 默认 3，可用环境变量覆盖；SIM_RUN 保护当前轮不被删。
  KEEP_RUNS="${KEEP_RUNS:-3}" SIM_RUN="${SIM_RUN:-}" \
    bash /home/gengyinghao/drone_sim/scripts/prune_sim_runs.sh --apply \
    >> /home/gengyinghao/drone_sim/logs/_prune.log 2>&1 || true
}
trap cleanup EXIT

say "PX4_VIO_CONFIG=$PX4_VIO_CONFIG path=$VIO_CFG md5=$(md5sum "$VIO_CFG" | cut -d' ' -f1)"
say "停止旧仿真"
bash /home/gengyinghao/drone_sim/stop_sim.sh >/dev/null 2>&1
sleep 3
pkill -x px4 2>/dev/null || true
sleep 1

say "PX4 SITL 冷启动（无头）"
cd /home/gengyinghao/drone_sim || exit 1
HEADLESS=1 PX4_AGENT_LOG="$SIM_RUN/agent.log" \
  bash /home/gengyinghao/drone_sim/start_px4_direct.sh 2>&1 \
  | python3 -u "$SCRIPTS/log_cap.py" --out "$SIM_RUN/px4_gz.log" --max-mb 96 \
  >"$SIM_RUN/log_cap.log" 2>&1 &
PIDS+=($!)
for i in $(seq 1 120); do
  grep -qa "gz_bridge" "$SIM_RUN/px4_gz.log" 2>/dev/null && break
  sleep 1
done
grep -qa "gz_bridge" "$SIM_RUN/px4_gz.log" 2>/dev/null \
  && say "gz_bridge 已就绪（等待 ${i}s）" || { say "PX4 未就绪，终止"; exit 1; }

say "ORB-SLAM3 IMU_STEREO 启动（ros_gz_bridge + stereo_inertial_sim_node）"
WS=/home/gengyinghao/px4_ws_201 \
SYNC_TOL=0.005 \
  bash /home/gengyinghao/drone_sim/scripts/run_stereo_inertial_sim.sh \
  >"$SIM_RUN/slam.log" 2>&1 &
PIDS+=($!)
for i in $(seq 1 60); do
  grep -qa "stereo_inertial_sim_node" "$SIM_RUN/slam.log" 2>/dev/null && break
  sleep 1
done
grep -qa "stereo_inertial_sim_node" "$SIM_RUN/slam.log" 2>/dev/null \
  && say "SLAM 节点已拉起（等待 ${i}s）" || { say "SLAM 未启动，见 slam.log"; exit 1; }

set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source /home/gengyinghao/px4_ws_201/install/setup.bash
set -u

say "遥测采样启动"
python3 "$SCRIPTS/telemetry.py" --out "$SIM_RUN/telemetry.csv" --seconds 420 \
  >"$SIM_RUN/telemetry.log" 2>&1 &
PIDS+=($!)

say "行为树启动（严格 health，mission=line 5.0）"
env -u WSL_RELAX_FRESHNESS -u PILOT_LINK_PROBE \
  PILOT_LOG_ROOT="$SIM_RUN/bt" \
  python3 /home/gengyinghao/drone_sim/scripts/pilot_navigation_boot.py \
  --config "$BT_CFG" >"$SIM_RUN/bt.log" 2>&1 &
PIDS+=($!)
sleep 4

say "EKF 融合标志采样启动（判断 EV 到底有没有进滤波器）"
python3 /home/gengyinghao/drone_sim/scripts/read_ekf_flags.py 100 \
  >"$SIM_RUN/ekf_flags.log" 2>&1 &
PIDS+=($!)

say "等待 EKF 收敛"
sleep 8

say "飞手启动（正弦姿态激励 20 s 完成 VI 初始化）"
python3 /home/gengyinghao/drone_sim/scripts/sim_pilot.py \
  --log "$SIM_RUN/pilot.csv" --gcs-port 14550 \
  --excite-mode translate --excite-pitch 80 --excite-roll 80 --excite-vert 80 \
  --excite-period 0.6 --excite-seconds 45 \
  --await-vio 40 --init-settle 3 --position-hold 8 \
  --takeoff-alt 1.0 \
  >"$SIM_RUN/pilot.log" 2>&1 &
PIDS+=($!)

for i in $(seq 1 420); do
  grep -qa "state -> DONE" "$SIM_RUN/pilot.log" 2>/dev/null && break
  sleep 1
done
if grep -qa "state -> DONE" "$SIM_RUN/pilot.log" 2>/dev/null; then
  say "飞手流程结束 DONE（DONE 不等于任务通过）"
else
  say "FAIL: 飞手未到 DONE（见 pilot.log）"
  echo "FAIL pilot_not_done" > "$SIM_RUN/RESULT"
fi
say "采集收尾"
