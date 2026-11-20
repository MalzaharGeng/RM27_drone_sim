#!/usr/bin/env bash
# STAGE-20261120 单轮运行器：短时失锁诊断/验收通用。
# 任务和安全判据固定，环境变量仅指定依赖路径。
# bearing_deg=90 为相对初始航向偏角；当前出生条件下对应东向 5 m。
# 北向历史轮曾出现贴墙退化，本入口用于分离方向因素，不能据此断言
# 视觉失锁的唯一原因。正式组 T1/T2 通过、T3 失败。
#
# 诊断：
#   RM27_DIAG_DIR        —— 旧电机/真值诊断（默认关，no-op）
#   RM27_TRACKDIAG_DIR   —— 本轮新增的逐帧跟踪诊断（默认关；本脚本默认关闭）
set -uo pipefail
ROUND=${1:?usage: run_stage_round.sh <ROUND_NAME> [on|off]}
DIAG_MODE=${2:-off}
BASE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT=${DRONE_SIM_ROOT:-$(cd "$BASE/../.." && pwd)}
WP=$ROOT/wb_handoff_20261114/workpkg_20261115
source "$ROOT/env.sh"
export NP01_RUNNER=$BASE/runner/run_profile_isolated.sh
export NP01_LIFECYCLE=$WP/newpkg_fix18/runenv/np01_lifecycle.py
export NP01_IDENTITY_RECORD=$WP/newpkg_fix19/recordenv/np01_identity_record.py
export NP01_LAUNCHER=$WP/newpkg_fix18/runenv/run_stereo_inertial_sim_fix18.sh
export NP01_OBS_COUNTER=$WP/newpkg_fix18/runenv/vio_obs_counter.py
export BT_CFG=$BASE/runner/pilot_navigation_link_5m_east.json
export PROFILE=vio_closed_loop DISTANCE=5.0 DURATION=240 RECORD=0 KEEP_RUNS=100
export PX4_VIO_CONFIG=transport SIM_DDS_DEDUP=1 SIM_LATENCY_OBSERVATION=on
export WS=${WS:-$HOME/px4_ws_201} ORB_ROOT=${ORB_ROOT:-$HOME/ORB_SLAM3}
export ORB_PROFILE=$ORB_ROOT/config/RM27_Harrier_IMU_Stereo_baseline.yaml
export PX4_GZ_WORLD=rm27_midhangar_n_world NP01_WORLD=rm27_midhangar_n_world
export PX4_GZ_MODEL_NAME=Drone-Harrier TRUTH_ENTITY=Drone-Harrier
export PX4_GZ_MODEL_POSE=6.80,8.05,0.504900,0,0,1.5707963
export SUP_GEOMETRY_JSON=$ROOT/scripts/harrier_20261117/geometry_midhangar_n.json
export NP01_SUPERVISOR=1 NP01_TRUTH_WATCHDOG=0
export SUP_WARN_CLEARANCE=1.5 SUP_STOP_CLEARANCE=1.2 SUP_ENVELOPE=0.385971
export SUP_MAX_XY=20.0 SUP_MAX_XY_INIT=20.0 SUP_INIT_SECONDS=55 SUP_ACK_TIMEOUT=2.0
export EXCITE_CONTROLLER=bounded EXCITE_WAIT_CAP_S=25.0 EXCITE_MAX_XY=9.5
export NP01_VIO_START=immediate INIT_GATE=airborne INIT_GATE_AIRBORNE_HOLD=0.50
export INIT_GATE_HEALTH_WINDOW=1.00 INIT_GATE_MIN_IMG_HZ=15.0
export EXCITE_SECONDS=20 EXCITE_WAVEFORM=square EXCITE_SLOW_PERIOD=2.5 EXCITE_SLOW_DEG=4.0
export EXCITE_FAST_PERIOD=0.5 EXCITE_FAST_DEG=8.0 INIT_SETTLE=3.0
unset SIM_DIAG_EXIT_AFTER PILOT_LINK_PROBE WSL_RELAX_FRESHNESS RM27_DIAG_DIR
export GZ_PARTITION=s2_$(date +%H%M%S)_$$
export ROS_DOMAIN_ID=0
export NP01_VIEWER=0
export PX4_STATE_SNAPSHOT=$BASE/reference/S_diag
export ORB_NODE_BIN=${ORB_NODE_BIN:-$WS/install/rm27_slam/lib/rm27_slam/stereo_inertial_sim_node}
export SIM_RUN=$BASE/runs/$ROUND
[[ "$ROUND" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "invalid round name" >&2; exit 2; }
[[ "$DIAG_MODE" == on || "$DIAG_MODE" == off ]] || { echo "diagnostic mode: on|off" >&2; exit 2; }
[[ ! -e "$SIM_RUN" ]] || { echo "round already exists: $SIM_RUN" >&2; exit 2; }
[[ -x "$ORB_NODE_BIN" ]] || { echo "build rm27_slam and set ORB_NODE_BIN first" >&2; exit 2; }
mkdir -p "$SIM_RUN"
export WORKPKG_NODE_BIN=$ORB_NODE_BIN

if [[ "$DIAG_MODE" == "on" ]]; then
  export RM27_TRACKDIAG_DIR=$SIM_RUN/trackdiag
  mkdir -p "$RM27_TRACKDIAG_DIR"
else
  unset RM27_TRACKDIAG_DIR
fi

# 外来实例预检：共享 ROS_DOMAIN_ID=0，其他任务的 PX4/Gazebo 会造成 /fmu/* 串流。
FOREIGN=$(ps -eo pid,cmd | grep -E "bin/px4|gz-sim-main" | grep -v grep \
          | grep -v "$SIM_RUN" | grep -v "run_profile_isolated" || true)
if [[ -n "$FOREIGN" ]]; then
  echo "ABORT_PREFLIGHT: another PX4/Gazebo instance is running on this host:"
  echo "$FOREIGN"
  exit 3
fi

# [STAGE-20261120] 运行期外来实例监视：共享 ROS_DOMAIN_ID=0，别的任务中途启动
# 的 PX4/Gazebo 会和本轮 /fmu/* 串流。这里只观察并留证，不干预、不杀。
FOREIGN_WATCH=$SIM_RUN/foreign_watch.csv
(
  echo "t_epoch_s,pid,cmd" > "$FOREIGN_WATCH"
  while true; do
    sleep 10
    ts=$(date +%s.%N)
    ps -eo pid,cmd | grep -E "bin/px4|gz-sim-main" | grep -v grep \
      | grep -v "$SIM_RUN" | grep -v "run_profile_isolated" \
      | awk -v t="$ts" '{cmd=""; for(i=2;i<=NF;i++) cmd=cmd" "$i; print t","$1","cmd}' \
      >> "$FOREIGN_WATCH"
  done
) &
WATCH_PID=$!
trap 'kill $WATCH_PID 2>/dev/null' EXIT

echo "===$ROUND START $(date +%T)==="
echo "SIM_RUN=$SIM_RUN BEARING=90 DIAG=$DIAG_MODE TRACKDIAG=${RM27_TRACKDIAG_DIR:-off} PARTITION=$GZ_PARTITION"
echo "NODE_BIN=$ORB_NODE_BIN"
bash "$NP01_RUNNER"
rc=$?
kill "$WATCH_PID" 2>/dev/null || true
wait "$WATCH_PID" 2>/dev/null || true
exit "$rc"
