#!/usr/bin/env bash
# 统一入口：按 profile 跑一轮可复现实验，并写出 manifest.json。
#
#   PROFILE=control_truth   DISTANCE=5.0 ./run_profile.sh
#   PROFILE=vio_closed_loop DISTANCE=5.0 ./run_profile.sh
#
# 显式约定（不再依赖旧脚本的隐式行为）：
#   * PX4_VIO_CONFIG 原样传给 run_stereo_inertial_sim.sh，默认 transport。
#     旧 run_stereo_5m.sh 在内部写死 transport，外部设 strict 也无效；本脚本不覆盖，
#     并把所用配置写进 manifest。transport 只补偿 timesync/样本年龄（[SIM-ONLY]），
#     该两项不得登记为实机同源验收；需要严格口径时显式 PX4_VIO_CONFIG=strict。
#   * 绝不设置 PILOT_LINK_PROBE / WSL_RELAX_FRESHNESS。
#   * 退出码由 check_run.py 判定，RUN_EXIT=0 / 飞手 DONE 都不等于任务通过。
#   * 每轮独立目录 + manifest.json + 关键证据文件。
set -uo pipefail

PROFILE=${PROFILE:?必须设置 PROFILE: control_truth | vio_closed_loop}
DISTANCE=${DISTANCE:-5.0}
DURATION=${DURATION:-180}
TAKEOFF_ALT=${TAKEOFF_ALT:-1.0}
PX4_VIO_CONFIG=${PX4_VIO_CONFIG:-transport}
EXCITE_SECONDS=${EXCITE_SECONDS:-20}
EXCITE_PITCH=${EXCITE_PITCH:-60}
EXCITE_ROLL=${EXCITE_ROLL:-60}
EXCITE_VERT=${EXCITE_VERT:-40}
EXCITE_PERIOD=${EXCITE_PERIOD:-0.6}
# [NP01-FIX15] 双分量激励：slow 给 mTinit 提供关键帧位移（VIBA2 需要累计 10 s），
# fast 给 StereoInitialization 提供 >=0.5 m/s² 的相邻帧比力阶跃。
EXCITE_WAVEFORM=${EXCITE_WAVEFORM:-square}
EXCITE_SLOW_PERIOD=${EXCITE_SLOW_PERIOD:-3.0}
EXCITE_SLOW_DEG=${EXCITE_SLOW_DEG:-3.0}
EXCITE_FAST_PERIOD=${EXCITE_FAST_PERIOD:-0.5}
EXCITE_FAST_DEG=${EXCITE_FAST_DEG:-8.0}
INIT_SETTLE=${INIT_SETTLE:-4}
POSITION_HOLD=${POSITION_HOLD:-8}
# [NP-01 FIX12] Opt-in bounded excitation controller + independent truth
# watchdog. Default "legacy" reproduces every historical run bit-for-bit.
EXCITE_CONTROLLER=${EXCITE_CONTROLLER:-legacy}
EXCITE_READY_HOLD=${EXCITE_READY_HOLD:-0.4}
EXCITE_MIN_SECONDS=${EXCITE_MIN_SECONDS:-2.0}
EXCITE_INPUT_GRACE_S=${EXCITE_INPUT_GRACE_S:-2.0}
EXCITE_WAIT_CAP_S=${EXCITE_WAIT_CAP_S:-25.0}
EXCITE_FEEDBACK_STALE_S=${EXCITE_FEEDBACK_STALE_S:-0.6}
EXCITE_MAX_RATE_DPS=${EXCITE_MAX_RATE_DPS:-300}
EXCITE_MAX_TILT_DEG=${EXCITE_MAX_TILT_DEG:-25}
EXCITE_MAX_DZ=${EXCITE_MAX_DZ:-1.0}
EXCITE_MAX_XY=${EXCITE_MAX_XY:-8.0}
NP01_TRUTH_WATCHDOG=${NP01_TRUTH_WATCHDOG:-0}
TRUTH_WATCHDOG_MAX_XY=${TRUTH_WATCHDOG_MAX_XY:-12.0}
TRUTH_WATCHDOG_MAX_Z=${TRUTH_WATCHDOG_MAX_Z:-4.0}
TRUTH_WATCHDOG_MAX_RATE_DPS=${TRUTH_WATCHDOG_MAX_RATE_DPS:-600.0}
# [NP-01 FIX16] Low-latency truth supervisor: direct pose subscription, no CSV
# in the safety path, warning carries an ACK contract, stop pauses the Gazebo
# physics FIRST. Opt-in; the old file-polling watchdog stays available and is
# never run at the same time (both write the same markers).
NP01_SUPERVISOR=${NP01_SUPERVISOR:-0}
NP01_WORLD=${PX4_GZ_WORLD:-rm27_world}
# [NP-01 FIX17] the truth recorder must listen to THIS world's pose topic. Its
# default is the original world name; without this the recorder writes an empty
# truth.csv and every truth-based acceptance check becomes UNVERIFIED (round 16).
TRUTH_GZ_TOPIC=${TRUTH_GZ_TOPIC:-/world/${NP01_WORLD}/dynamic_pose/info}
export TRUTH_GZ_TOPIC
SUP_WARN_CLEARANCE=${SUP_WARN_CLEARANCE:-${TRUTH_WATCHDOG_WARN_CLEARANCE:-2.0}}
SUP_STOP_CLEARANCE=${SUP_STOP_CLEARANCE:-${TRUTH_WATCHDOG_STOP_CLEARANCE:-1.2}}
SUP_ENVELOPE=${SUP_ENVELOPE:-${TRUTH_WATCHDOG_ENVELOPE:-0.385971}}
SUP_MAX_XY=${SUP_MAX_XY:-${TRUTH_WATCHDOG_MAX_XY:-6.5}}
SUP_MAX_XY_INIT=${SUP_MAX_XY_INIT:-${TRUTH_WATCHDOG_MAX_XY_INIT:-6.5}}
SUP_INIT_SECONDS=${SUP_INIT_SECONDS:-${TRUTH_WATCHDOG_INIT_SECONDS:-90}}
SUP_ACK_TIMEOUT=${SUP_ACK_TIMEOUT:-2.0}
# [NP-01 FIX17] geometry generated together with the world; when set the
# supervisor takes walls/spawn/envelope from it and refuses a world mismatch,
# so an old wall coordinate cannot be reused by accident.
SUP_GEOMETRY_JSON=${SUP_GEOMETRY_JSON:-}

ROOT=/home/gengyinghao/drone_sim
SCRIPTS=$ROOT/scripts
BT_CFG=$ROOT/config/pilot_navigation_link_5m.json
# 关键：实际安装空间是 px4_ws_201；run_stereo_inertial_sim.sh 默认 $HOME/px4_ws
# 里是 11-12 的旧二进制。这里显式指定，保证跑到的是本轮源码构建产物。
WS=${WS:-/home/gengyinghao/px4_ws_201}
ORB_ROOT=${ORB_ROOT:-/home/gengyinghao/ORB_SLAM3}
export WS ORB_ROOT
# [SIM-ONLY] 输入适配开关：DDS 重复时戳去重（见 pilot_navigation_bt.py 注释）。
# 实机不设置此变量；这里显式 export，便于 manifest/env.txt 如实记录。
export SIM_DDS_DEDUP="${SIM_DDS_DEDUP:-1}"
# ORB 地图策略 profile：baseline = 与 Nano 同源（关闭两个 [SIM-ONLY] 实验开关）；
# experiment = 保留历史补丁（never_reset_map / retain_without_ba2 = 1）做 A/B。
ORB_PROFILE=${ORB_PROFILE:-baseline}
case "$ORB_PROFILE" in
  baseline)   ORB_SETTINGS_FILE="$ORB_ROOT/config/RM27_Sim_IMU_Stereo_baseline.yaml" ;;
  experiment) ORB_SETTINGS_FILE="$ORB_ROOT/config/RM27_Sim_IMU_Stereo.yaml" ;;
  *)          ORB_SETTINGS_FILE="$ORB_PROFILE" ;;
esac
export ORB_PROFILE ORB_SETTINGS_FILE
case "$PROFILE" in
  control_truth|vio_closed_loop) ;;
  *) echo "未知 PROFILE=$PROFILE" >&2; exit 2 ;;
esac

if [[ -n "${SIM_RUN:-}" ]]; then
  D=$SIM_RUN
else
  D=$ROOT/logs/${PROFILE}_$(date +%Y%m%d_%H%M%S)
fi
mkdir -p "$D/bt"
echo "$D" > /tmp/last_simrun
PIDS=()
T0_WALL=$(date +%s.%N)
say() { echo "$(date +%T.%N) $*" | tee -a "$D/run.log"; }

# 每轮独立行为树配置：距离/限幅写进本轮目录，manifest 会记录其 md5。
python3 - "$BT_CFG" "$D/bt_config.json" "$DISTANCE" <<'PY'
import json, sys
base, out, dist = sys.argv[1], sys.argv[2], float(sys.argv[3])
cfg = json.load(open(base))
cfg["distance_m"] = dist
json.dump(cfg, open(out, "w"), ensure_ascii=False, indent=2)
PY
say "bt_config: $(md5sum "$D/bt_config.json" | cut -d' ' -f1)"

cleanup() {
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null || true; done
  sleep 1
  pkill -f stereo_inertial_sim_node 2>/dev/null || true
  pkill -f truth_odom_node 2>/dev/null || true
  pkill -f truth_gz_node.py 2>/dev/null || true
  pkill -f truth_recorder.py 2>/dev/null || true
  pkill -f telemetry.py 2>/dev/null || true
  pkill -f "parameter_bridge.*rosgraph_msgs" 2>/dev/null || true
  pkill -f pilot_navigation_bt.py 2>/dev/null || true
  pkill -f sim_pilot.py 2>/dev/null || true
  bash "$ROOT/stop_sim.sh" >/dev/null 2>&1
  # 只保留最近 3 轮，保护本轮
  KEEP_RUNS="${KEEP_RUNS:-3}" SIM_RUN="$D" \
    bash "$SCRIPTS/prune_sim_runs.sh" --apply >> "$ROOT/logs/_prune.log" 2>&1 || true
}
trap cleanup EXIT

say "profile=$PROFILE distance=$DISTANCE vio_config=$PX4_VIO_CONFIG run=$D"
# 重要标注：transport 只补偿 timesync/样本年龄两项传输层门限（[SIM-ONLY]），
# 其余门控与实机同值；用它跑出的 time_sync 结论不得登记为实机同源验收。
if [[ "$PX4_VIO_CONFIG" == "transport" ]]; then
  say "注意：time_sync/sample_age 使用 transport 补偿（[SIM-ONLY]，不可登记为实机同源）"
fi
{
  echo "profile=$PROFILE distance=$DISTANCE vio_config=$PX4_VIO_CONFIG"
  echo "WS=$WS ORB_ROOT=$ORB_ROOT"
  echo "cmd=$0"
  env | grep -E '^(PROFILE|DISTANCE|PX4_VIO_CONFIG|SYNC_TOL|RECORD|SIM_NAV_GUARD|WS|ORB_ROOT|PILOT_LINK_PROBE|WSL_RELAX_FRESHNESS|SIM_DDS_DEDUP|ORB_PROFILE|ORB_SETTINGS_FILE|RMW_IMPLEMENTATION|CYCLONEDDS_URI)=' || true
} > "$D/env.txt"
say "停止旧仿真"
bash "$ROOT/stop_sim.sh" >/dev/null 2>&1
sleep 3
pkill -x px4 2>/dev/null || true
sleep 1

say "PX4 SITL 冷启动（无头）"
cd "$ROOT" || exit 1
HEADLESS=1 PX4_AGENT_LOG="$D/agent.log" \
  bash "$ROOT/start_px4_direct.sh" 2>&1 \
  | python3 -u "$SCRIPTS/log_cap.py" --out "$D/px4_gz.log" --max-mb 96 \
  >"$D/log_cap.log" 2>&1 &
PIDS+=($!)
for i in $(seq 1 120); do
  grep -qa "gz_bridge" "$D/px4_gz.log" 2>/dev/null && break
  sleep 1
done
grep -qa "gz_bridge" "$D/px4_gz.log" 2>/dev/null \
  && say "gz_bridge 就绪（${i}s）" || { say "PX4 未就绪"; echo "FAIL px4_start" > "$D/RESULT"; exit 1; }

set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source /home/gengyinghao/px4_ws_201/install/setup.bash
set -u

# 独立真值记录（不参与控制，实体名/sim 时间）
say "真值记录启动"
TRUTH_ENTITY="${TRUTH_ENTITY:-x500_mono_cam_0}" \
  python3 "$SCRIPTS/truth_recorder.py" --out "$D/truth.csv" \
  >"$D/truth.log" 2>&1 &
PIDS+=($!)

if [[ "$PROFILE" == "control_truth" ]]; then
  say "真值桥接启动（gz Pose_V -> /truth/pose -> 真实 Px4Bridge）"
  # /clock 必须桥过来：truth_odom_node 用它把 gz sim 时间映射到 epoch µs。
  # 没有 /clock 时节点不会提交任何样本（此前一轮即因此全程 sample_clock_invalid）。
  ros2 run ros_gz_bridge parameter_bridge \
    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
    >"$D/clock_bridge.log" 2>&1 &
  PIDS+=($!)
  python3 "$SCRIPTS/truth_gz_node.py" >"$D/truth_gz.log" 2>&1 &
  PIDS+=($!)
  for i in $(seq 1 30); do
    grep -qa "truth_gz published=[1-9]" "$D/truth_gz.log" 2>/dev/null && break
    sleep 1
  done
  case "$PX4_VIO_CONFIG" in
    transport) PX4_VIO_CFG="$ORB_ROOT/config/px4_vio_sim_transport.yaml" ;;
    strict|"") PX4_VIO_CFG="$ORB_ROOT/config/px4_vio_sim.yaml" ;;
    *)         PX4_VIO_CFG="$PX4_VIO_CONFIG" ;;
  esac
  say "真值->Px4Bridge（同一套门控，config=$PX4_VIO_CFG）"
  /home/gengyinghao/px4_ws_201/build/rm27_slam/truth_odom_node \
    --ros-args -p "px4_config:=$PX4_VIO_CFG" -p "pose_topic:=/truth/pose" \
    >"$D/slam.log" 2>&1 &
  PIDS+=($!)
else
  say "ORB-SLAM3 IMU_STEREO 启动（bridge + stereo_inertial_sim_node）"
  RECORD_DIR=""
  if [[ "${RECORD:-0}" == "1" ]]; then
    RECORD_DIR="$D/recording"
    rm -rf "$RECORD_DIR"
  fi
  PX4_VIO_CONFIG="$PX4_VIO_CONFIG" SYNC_TOL="${SYNC_TOL:-0.005}" \
  ORB_SETTINGS_FILE="$ORB_SETTINGS_FILE" \
  ORB_RECORD_DIR="$RECORD_DIR" LATENCY_CSV="$D/latency.csv" \
  SIM_NAV_GUARD="${SIM_NAV_GUARD:-false}" \
  INIT_GATE="${INIT_GATE:-airborne}" \
    bash "$SCRIPTS/run_stereo_inertial_sim.sh" >"$D/slam.log" 2>&1 &
  PIDS+=($!)
fi

for i in $(seq 1 90); do
  grep -qa "\[PX4\] domain=" "$D/slam.log" 2>/dev/null && break
  sleep 1
done
grep -qa "\[PX4\] domain=" "$D/slam.log" 2>/dev/null \
  && say "桥接节点就绪（${i}s）" || say "警告：桥接节点未打印 domain，继续观察"

say "遥测采样启动"
python3 "$SCRIPTS/telemetry.py" --out "$D/telemetry.csv" \
  --slam-csv "$D/slam_pose.csv" --seconds "$((DURATION + 60))" \
  >"$D/telemetry.log" 2>&1 &
PIDS+=($!)

say "行为树启动（严格 health，mission=line distance=$DISTANCE）"
env -u WSL_RELAX_FRESHNESS -u PILOT_LINK_PROBE SIM_DDS_DEDUP=1 \
  PILOT_LOG_ROOT="$D/bt" \
  python3 "$SCRIPTS/pilot_navigation_boot.py" --config "$D/bt_config.json" \
  >"$D/bt.log" 2>&1 &
PIDS+=($!)
sleep 4

say "等待 EKF 收敛"
sleep 8

# [NP-01 FIX12] Build the pilot argument tail. Legacy path is unchanged.
if [[ "$EXCITE_CONTROLLER" == "bounded" ]]; then
  EXCITE_EXTRA=(--excite-controller bounded
    --excite-ready-hold "$EXCITE_READY_HOLD"
    --excite-min-seconds "$EXCITE_MIN_SECONDS"
    --excite-wait-cap-s "$EXCITE_WAIT_CAP_S"
    --excite-input-grace-s "$EXCITE_INPUT_GRACE_S"
    --excite-feedback-stale-s "$EXCITE_FEEDBACK_STALE_S"
    --excite-max-rate-dps "$EXCITE_MAX_RATE_DPS"
    --excite-max-tilt-deg "$EXCITE_MAX_TILT_DEG"
    --excite-max-dz "$EXCITE_MAX_DZ"
    --excite-max-xy "$EXCITE_MAX_XY")
  say "FIX12 bounded excitation controller: ON (no --excite-no-early-stop)"
else
  EXCITE_EXTRA=(--excite-no-early-stop)
  say "FIX12 bounded excitation controller: OFF (legacy, reproducible)"
fi

say "飞手启动"
if [[ "$PROFILE" == "control_truth" ]]; then
  python3 "$SCRIPTS/sim_pilot.py" \
    --log "$D/pilot.csv" --gcs-port 14550 \
    --excite-seconds 0 --await-vio 40 --init-settle 0.5 --position-hold "$POSITION_HOLD" \
    --takeoff-alt "$TAKEOFF_ALT" --no-home-gate \
    >"$D/pilot.log" 2>&1 &
else
  python3 "$SCRIPTS/sim_pilot.py" \
    --log "$D/pilot.csv" --gcs-port 14550 \
    --excite-mode translate --excite-pitch "$EXCITE_PITCH" --excite-roll "$EXCITE_ROLL" \
    --excite-vert "$EXCITE_VERT" --excite-period "$EXCITE_PERIOD" \
    --excite-waveform "$EXCITE_WAVEFORM" \
    --excite-slow-period "$EXCITE_SLOW_PERIOD" --excite-slow-deg "$EXCITE_SLOW_DEG" \
    --excite-fast-period "$EXCITE_FAST_PERIOD" --excite-fast-deg "$EXCITE_FAST_DEG" \
    --excite-seconds "$EXCITE_SECONDS" "${EXCITE_EXTRA[@]}" \
    --warn-marker "$D/TRUTH_WARN" \
    --ack-marker "$D/TRUTH_WARN_ACK" \
    --await-vio 45 --init-settle "$INIT_SETTLE" --position-hold "$POSITION_HOLD" \
    --takeoff-alt "$TAKEOFF_ALT" \
    >"$D/pilot.log" 2>&1 &
fi
PIDS+=($!)

# [NP-01 FIX12] Independent, READ-ONLY truth monitor. It never publishes to
# any topic and never writes pilot.csv. Out-of-envelope it only writes a
# marker which the wait loop below uses to terminate this run.
if [[ "$NP01_TRUTH_WATCHDOG" == "1" ]]; then
  python3 "$SCRIPTS/truth_safety_watchdog.py" \
    --truth "$D/truth.csv" --run-dir "$D" \
    --max-xy "$TRUTH_WATCHDOG_MAX_XY" \
    --max-xy-init "${TRUTH_WATCHDOG_MAX_XY_INIT:-2.0}" \
    --init-seconds "${TRUTH_WATCHDOG_INIT_SECONDS:-90}" \
    --max-z "$TRUTH_WATCHDOG_MAX_Z" \
    --max-rate-dps "$TRUTH_WATCHDOG_MAX_RATE_DPS" \
    --envelope-m "${TRUTH_WATCHDOG_ENVELOPE:-0.385971}" \
    --warn-clearance "${TRUTH_WATCHDOG_WARN_CLEARANCE:-2.5}" \
    --stop-clearance "${TRUTH_WATCHDOG_STOP_CLEARANCE:-1.0}" \
    >"$D/truth_watchdog.log" 2>&1 &
  PIDS+=($!)
  say "独立真值监测启动（只读；预警→飞手取消任务并 LAND；净空不足→终止）"
fi

# [NP-01 FIX16] Low-latency independent TRUTH supervisor. Subscribes to the
# bridged Gazebo pose directly (no CSV, no polling of files for the decision),
# writes the same TRUTH_WARN / TRUTH_BOUND_EXCEEDED markers, requires an ACK
# from the pilot for a warning, and on STOP pauses the Gazebo physics before
# anything else. Truth never becomes a control input.
if [[ "$NP01_SUPERVISOR" == "1" ]]; then
  python3 "$SCRIPTS/np01_truth_supervisor.py" \
    --run-dir "$D" --world "$NP01_WORLD" \
    --geometry-json "$SUP_GEOMETRY_JSON" \
    --warn-clearance "$SUP_WARN_CLEARANCE" \
    --stop-clearance "$SUP_STOP_CLEARANCE" \
    --envelope-m "$SUP_ENVELOPE" \
    --max-xy "$SUP_MAX_XY" --max-xy-init "$SUP_MAX_XY_INIT" \
    --init-seconds "$SUP_INIT_SECONDS" \
    --max-z "$TRUTH_WATCHDOG_MAX_Z" \
    --max-rate-dps "$TRUTH_WATCHDOG_MAX_RATE_DPS" \
    --ack-timeout-s "$SUP_ACK_TIMEOUT" \
    >"$D/supervisor.log" 2>&1 &
  PIDS+=($!)
  say "低延迟真值监督启动（直订阅位姿；clearance_net warn=$SUP_WARN_CLEARANCE stop=$SUP_STOP_CLEARANCE；stop 先暂停物理）"
  # Fail closed: no pose on the supervision topic -> no independent monitor ->
  # the run must not proceed.
  for _ in $(seq 1 80); do
    [[ -f "$D/SUPERVISOR_NO_POSE" ]] && break
    sleep 0.1
  done
  if [[ -f "$D/SUPERVISOR_NO_POSE" ]]; then
    say "监督器没有收到任何真值位姿：终止本轮（不做无监督飞行）"
    echo "FAIL supervisor_no_pose" > "$D/RESULT"
    exit 4
  fi
fi

if [[ "$NP01_TRUTH_WATCHDOG" == "1" && "$NP01_SUPERVISOR" == "1" ]]; then
  say "拒绝同时运行两套监督器（会互相覆盖标记）"; echo "FAIL double_watchdog" > "$D/RESULT"; exit 3
fi

for i in $(seq 1 "$DURATION"); do
  grep -qa "state -> DONE" "$D/pilot.log" 2>/dev/null && break
  if [[ -f "$D/TRUTH_BOUND_EXCEEDED" ]]; then
    say "真值监督终止：物理已请求暂停（证据 supervisor.json；不隐藏错误）"
    # give the supervisor a bounded moment to finish writing its own report,
    # the pause is already in effect so no motion continues meanwhile.
    for _ in $(seq 1 30); do
      [[ -f "$D/supervisor.json" ]] && break
      sleep 0.1
    done
    break
  fi
  sleep 1
done
if grep -qa "state -> DONE" "$D/pilot.log" 2>/dev/null; then
  say "飞手流程结束 DONE（注意：DONE 不等于任务通过）"
else
  say "飞手未到 DONE（见 pilot.log）"
fi

say "采集收尾"
sleep 1
# 停后台，保留 manifest 生成时间
for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null || true; done
sleep 2
# 冻结证据文件：节点/桥/记录器全部停掉后再判定，避免边写边读。
pkill -f stereo_inertial_sim_node 2>/dev/null || true
pkill -f truth_odom_node 2>/dev/null || true
pkill -f truth_gz_node.py 2>/dev/null || true
pkill -f truth_recorder.py 2>/dev/null || true
pkill -f telemetry.py 2>/dev/null || true
sleep 2

# [PHASE5-OBS] Evidence archiving only (no control/parameter change).
# Px4Bridge writes its publish-boundary telemetry to a fixed relative path
# (px4_vio_sim_transport.yaml log_path); the next cold start truncates it.
# Copy it into THIS run directory after all nodes stopped, so the publish
# boundary can be joined to latency.csv per frame via sample_us.
if [[ -f "$ROOT/px4_telemetry_sim_transport.csv" ]]; then
  cp "$ROOT/px4_telemetry_sim_transport.csv" "$D/bridge_telemetry.csv"
  say "Bridge 发布边界日志已归档 bridge_telemetry.csv"
fi

T1_WALL=$(date +%s.%N)
python3 "$SCRIPTS/write_manifest.py" --run "$D" --profile "$PROFILE" \
  --distance "$DISTANCE" --t0 "$T0_WALL" --t1 "$T1_WALL" \
  --vio-config "$PX4_VIO_CONFIG" >>"$D/run.log" 2>&1 || true

say "结果判定"
python3 "$SCRIPTS/check_run.py" --run "$D" --profile "$PROFILE" \
  --distance "$DISTANCE" | tee "$D/check.txt"
RC=${PIPESTATUS[0]}
echo "CHECK_EXIT=$RC run=$D"
exit "$RC"
