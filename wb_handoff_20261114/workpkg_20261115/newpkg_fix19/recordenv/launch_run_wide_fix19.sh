#!/usr/bin/env bash
# [NP-01 FIX19] Launcher: WIDE-CLEARANCE baseline, lifecycle-fixed + record revision.
#
# Single environmental change under test: the world is rm27_wide_world
# (62 m inner half extent, centre spawn at (0,0,0.019), yaw 0, floor texture at
# the original physical density). Vehicle, sensors, algorithm requirements,
# Gate/LIMITS, warn/stop thresholds and the evaluator are unchanged.
#
# Code under test is the FIX16 supervision/handoff version plus the exit-path
# fix; the supervisor reads its geometry from the generated geometry file and
# refuses a world mismatch.
#
# [FIX18] Compared with launch_run_wide_fix17.sh the ONLY differences are:
#   * NP01_RUNNER / NP01_RUN_WRAPPER point at the FIX18 run environment
#     (run_profile_fix18.sh / run_wrapper_fix18.py).
#   * the scene record additionally lists the run-environment md5s and the
#     process-ownership contract.  No topic, QoS, parameter, threshold,
#     scenario, algorithm or evaluator value is touched.
#
# Registered BEFORE the run:
#   W1 the run is world rm27_wide_world with walls at +/-62 and the supervisor
#      reports wall_clearance_net_m at spawn = 61.614 m (+-0.05).
#   W2 no warn/stop during the init phase; the first supervision event, if any,
#      must not be a clearance event (there is 61 m of room).
#   W3 VIBA2 -> first publish -> PX4 EV adopted -> xy/vxy valid -> ready is
#      recorded in that order from bridge_telemetry.csv, not from gate_snapshot.
#   W4 excitation stops on fresh-ready + the pilot selects Position; PX4
#      nav_state must actually be POSCTL (2) AND the aircraft must stop
#      drifting (truth speed < 0.15 m/s for 3 s) before Offboard is requested.
#   W5 the task is attempted; all required checks of check_run.py decide.
#   W6 [FIX18] the ros_gz_bridge is owned by the whole-round runner: killing the
#      VIO node must not stop it, and every process of this round is reclaimed
#      at the end (process_cleanup.json verified_empty=true).
set -o pipefail
source /opt/ros/lyrical/setup.bash

WP=/home/gengyinghao/drone_sim/wb_handoff_20261114/workpkg_20261115
FIX16=$WP/newpkg_fix16
RUNENV=$WP/newpkg_fix19/recordenv
# 清理归属与观测计数仍引用 FIX18 冻结模块（md5 不变），只替换证据记录层
FIX18_RUNENV=$WP/newpkg_fix18/runenv
ROOT=/home/gengyinghao/drone_sim
GEO=$ROOT/rm27_sim/worlds/rm27_wide_geometry.json
DIST="$1"
PREFIX="$2"
DUR="${3:-300}"

# [FIX18] 运行环境修订版：整轮运行器 + 进程归属模块 + 包装器
export NP01_RUNENV="$RUNENV"
export NP01_RUNNER="$RUNENV/run_profile_fix19.sh"
export NP01_LIFECYCLE="$FIX18_RUNENV/np01_lifecycle.py"
export NP01_IDENTITY_RECORD="$RUNENV/np01_identity_record.py"
export NP01_OBS_COUNTER="$FIX18_RUNENV/vio_obs_counter.py"
export NP01_LAUNCHER="$FIX18_RUNENV/run_stereo_inertial_sim_fix18.sh"
export NP01_RUN_WRAPPER="$FIX18_RUNENV/run_wrapper_fix18.py"

export PROFILE=vio_closed_loop
export DISTANCE="$DIST"
export RECORD=0
export PX4_VIO_CONFIG=transport
export ORB_PROFILE=baseline
export SIM_DDS_DEDUP=1
export KEEP_RUNS=100
export WS=/home/gengyinghao/px4_ws_201
export ORB_ROOT=/home/gengyinghao/ORB_SLAM3
export SIM_LATENCY_OBSERVATION=on
export ORB_NODE_BIN="$WP/bin/stereo_inertial_sim_node.fix15"
export WORKPKG_NODE_BIN="$ORB_NODE_BIN"
export DURATION="$DUR"
unset SIM_DIAG_EXIT_AFTER

# --- WIDE baseline: independent world, centre spawn, yaw unchanged ---------
export PX4_GZ_WORLD=rm27_wide_world
export PX4_GZ_MODEL_POSE="0,0,0.019,0,0,0"
export NP01_WORLD=rm27_wide_world
export SUP_GEOMETRY_JSON="$GEO"

# --- supervision: same thresholds, applied to the NEW geometry ------------
export NP01_SUPERVISOR=1
export NP01_TRUTH_WATCHDOG=0
export SUP_WARN_CLEARANCE=2.0
export SUP_STOP_CLEARANCE=1.2
export SUP_ENVELOPE=0.385971
# Displacement guards are geometry-derived (NOT the clearance thresholds):
# init 30 m << 61.6 m net; task 12 m covers the 5 m mission with tolerance.
export SUP_MAX_XY=12.0
export SUP_MAX_XY_INIT=30.0
export SUP_INIT_SECONDS=120
export SUP_ACK_TIMEOUT=2.0

# --- startup lifecycle / excitation (unchanged) ---------------------------
export EXCITE_CONTROLLER=bounded
export EXCITE_WAIT_CAP_S=25.0
export NP01_VIO_START=immediate
export INIT_GATE=airborne
export INIT_GATE_AIRBORNE_HOLD=0.50
export INIT_GATE_HEALTH_WINDOW=1.00
export INIT_GATE_MIN_IMG_HZ=15.0
# [FIX17] The pilot's excitation envelope is DERIVED FROM THE ARENA, not a
# requirement: 8.0 m came from the old 7.8 m half arena. In the wide baseline
# an 8 m bound aborted the 5 m attempt during excitation (disp 8.00 > 8.00)
# while the supervisor still had 53 m of clearance. Re-derived here:
#   25 m at the worst measured 0.093 m/s^2 = 23.2 s of excitation, which covers
#   the worst observed init window (20.5 s), and leaves 36 m of clearance.
# This is NOT a Gate/LIMIT/warn/stop threshold and NOT an init requirement.
export EXCITE_MAX_XY=25.0
export EXCITE_SECONDS=20
export EXCITE_WAVEFORM=square
export EXCITE_SLOW_PERIOD=2.5
export EXCITE_SLOW_DEG=4.0
export EXCITE_FAST_PERIOD=0.5
export EXCITE_FAST_DEG=8.0
export INIT_SETTLE=3.0

TAG="${PREFIX}_$(date +%Y%m%d_%H%M%S)"
RD="$ROOT/logs/vio_closed_loop_${TAG}"
mkdir -p "$RD"
export NP01_GATE_SNAPSHOT="$RD/gate_snapshot.csv"
export ORB_INIT_METRICS="$RD/orb_init_metrics.csv"

echo "RUN_DIR=$RD"
echo "TAG=$TAG"
echo "WORLD=$PX4_GZ_WORLD spawn=$PX4_GZ_MODEL_POSE"
echo "DISTANCE=$DISTANCE DURATION=$DUR NODE_MD5=$(md5sum "$ORB_NODE_BIN" | cut -d' ' -f1)"
echo "SUPERVISOR md5=$(md5sum "$ROOT/scripts/np01_truth_supervisor.py" | cut -d' ' -f1)"
echo "PILOT md5=$(md5sum "$ROOT/scripts/sim_pilot.py" | cut -d' ' -f1)"
echo "RUNNER md5=$(md5sum "$ROOT/scripts/run_profile.sh" | cut -d' ' -f1) (frozen fix17, reference only)"
echo "RUNNER_FIX18 md5=$(md5sum "$NP01_RUNNER" | cut -d' ' -f1)"
echo "LIFECYCLE md5=$(md5sum "$NP01_LIFECYCLE" | cut -d' ' -f1)"
echo "IDENTITY_RECORD md5=$(md5sum "$NP01_IDENTITY_RECORD" | cut -d' ' -f1)"
echo "WORLD md5=$(md5sum "$ROOT/rm27_sim/worlds/$PX4_GZ_WORLD.sdf" | cut -d' ' -f1)"

python3 - "$GEO" "$RD/geometry_at_run.json" <<'PY'
import json, sys
geo = json.load(open(sys.argv[1]))
json.dump(geo, open(sys.argv[2], "w"), indent=2, ensure_ascii=False)
PY

{
  echo "{"
  echo "  \"condition\": \"NP01_WIDE_BASELINE_FIX19_RECORD_REV\","
  echo "  \"recorded_before_run\": true,"
  echo "  \"single_environmental_change\": \"world rm27_wide_world (62 m half extent, centre spawn, tiled arena floor texture); original world untouched\","
  echo "  \"run_environment_revision\": \"fix19 record revision of process ownership only (ros_gz_bridge owned by the whole-round runner); no topic/QoS/parameter/threshold/scenario/algorithm/evaluator change\","
  echo "  \"predictions\": ["
  echo "    \"W1 supervisor reads geometry json: walls +/-62, clearance_net at spawn 61.614 m\","
  echo "    \"W2 no clearance event during init\","
  echo "    \"W3 ba2 -> publish -> ev -> xy_valid -> ready recorded in that order (bridge_telemetry)\","
  echo "    \"W4 Position actually accepted (PX4 nav_state=2 POSCTL) and the drift stops before Offboard\","
  echo "    \"W5 5 m task attempted; check_run decides\","
  echo "    \"W6 killing the only VIO node leaves the runner-owned bridge alive; all round-owned processes reclaimed at exit\""
  echo "  ],"
  echo "  \"spawn_pose_gz_enu\": [0.0, 0.0, 0.019, 0, 0, 0],"
  echo "  \"arena_inner_faces\": {\"west_x\": -62.0, \"east_x\": 62.0, \"south_y\": -62.0, \"north_y\": 62.0},"
  echo "  \"rotor_envelope_m\": 0.385971,"
  echo "  \"geometry_file\": \"$GEO\","
  echo "  \"geometry_md5\": \"$(md5sum "$GEO" | cut -d' ' -f1)\","
  echo "  \"world_md5\": \"$(md5sum "$ROOT/rm27_sim/worlds/$PX4_GZ_WORLD.sdf" | cut -d' ' -f1)\","
  echo "  \"world_copy_px4_md5\": \"$(md5sum "$ROOT/PX4-Autopilot/Tools/simulation/gz/worlds/$PX4_GZ_WORLD.sdf" | cut -d' ' -f1)\","
  echo "  \"floor_obj_md5\": \"$(md5sum "$ROOT/rm27_sim/models/rm27_wide_floor/meshes/wide_floor.obj" | cut -d' ' -f1)\","
  echo "  \"floor_texture_md5\": \"$(md5sum "$ROOT/rm27_sim/models/rm27_wide_floor/meshes/floor_detail.png" | cut -d' ' -f1)\","
  echo "  \"supervision\": {\"warn_clearance_net_m\": 2.0, \"stop_clearance_net_m\": 1.2, \"max_xy_init_m\": 30.0, \"max_xy_task_m\": 12.0, \"ack_timeout_s\": 2.0},"
  echo "  \"excite\": {\"waveform\": \"square\", \"slow_deg\": 4.0, \"slow_period_s\": 2.5, \"fast_deg\": 8.0, \"fast_period_s\": 0.5, \"seconds\": 20},"
  echo "  \"distance_m\": $DIST,"
  echo "  \"duration_s\": $DUR,"
  echo "  \"node_bin\": \"$ORB_NODE_BIN\","
  echo "  \"node_md5\": \"$(md5sum "$ORB_NODE_BIN" | cut -d' ' -f1)\","
  echo "  \"libORB_SLAM3_md5\": \"$(md5sum "$ORB_ROOT/build/lib/libORB_SLAM3.so" | cut -d' ' -f1)\","
  echo "  \"run_profile_md5\": \"$(md5sum "$ROOT/scripts/run_profile.sh" | cut -d' ' -f1)\","
  echo "  \"sim_pilot_md5\": \"$(md5sum "$ROOT/scripts/sim_pilot.py" | cut -d' ' -f1)\","
  echo "  \"supervisor_md5\": \"$(md5sum "$ROOT/scripts/np01_truth_supervisor.py" | cut -d' ' -f1)\","
  echo "  \"gate_cfg_md5\": \"$(md5sum "$ORB_ROOT/config/px4_vio_sim_transport.yaml" | cut -d' ' -f1)\","
  echo "  \"check_run_md5\": \"$(md5sum "$ROOT/scripts/check_run.py" | cut -d' ' -f1)\","
  echo "  \"lifecycle\": {\"owner\": \"run_profile_fix18.sh (whole-round runner)\", \"bridge_owned_by\": \"runner\", \"launcher_mode\": \"--no-bridge --topics-file\", \"cleanup_rule\": \"registry identity (pid+starttime) AND sid == round sid\", \"uses_pkill\": false},"
  echo "  \"runenv\": {\"run_profile_fix18_md5\": \"$(md5sum "$NP01_RUNNER" | cut -d' ' -f1)\", \"launcher_fix18_md5\": \"$(md5sum "$FIX18_RUNENV/run_stereo_inertial_sim_fix18.sh" | cut -d' ' -f1)\", \"np01_lifecycle_md5\": \"$(md5sum "$NP01_LIFECYCLE" | cut -d' ' -f1)\", \"run_wrapper_fix18_md5\": \"$(md5sum "$NP01_RUN_WRAPPER" | cut -d' ' -f1)\", \"vio_obs_counter_md5\": \"$(md5sum "$FIX18_RUNENV/vio_obs_counter.py" | cut -d' ' -f1)\"}",
  echo "  \"record_revision\": {\"name\": \"fix19\", \"scope\": \"evidence layer only\", \"algorithm_reference\": \"FIX18 frozen node bfa646c84be62c15b1fcfaa5743dbd43\", \"normal_5m_reference_run\": \"vio_closed_loop_fix18_func5m_20261115_222616 (PASS 16/0/1)\", \"does_not_repeat_normal_5m\": true, \"does_not_overwrite_history\": true},"
  echo "  \"frozen_fix17_reference\": {\"run_profile_md5\": \"b1a2c121518eb46ace35dd908de34fc2\", \"launcher_md5\": \"15bd61820b8dfa37cebf64aa31f4accf\", \"note\": \"FIX17 artifacts untouched; the ACC03 3/3 result was produced by that frozen set, NOT by this run environment\"}",
  echo "  \"written_at\": \"$(date -Is)\""
  echo "}"
} > "$RD/scene_wide_fix19.json"
echo "SCENE=$RD/scene_wide_fix19.json"

python3 "$NP01_RUN_WRAPPER" \
    --run-dir "$RD" --tag "$TAG" --distance "$DIST" --duration "$DUR"
RC=$?
echo "WRAPPER_RC=$RC"

ULOG_DIR="$ROOT/PX4-Autopilot/build/px4_sitl_default/rootfs/log"
ULOG=$(ls -1t "$ULOG_DIR"/*/*.ulg 2>/dev/null | head -1)
if [ -n "${ULOG:-}" ]; then
  cp -p "$ULOG" "$RD/$(basename "$ULOG")"
  echo "ULOG_ARCHIVED=$RD/$(basename "$ULOG")"
fi

python3 "$WP/newpkg_20261115/tools/np01_exit_grade.py" "$RD" --write > "$RD/exit_grade.log" 2>&1
echo "EXIT_GRADE_RC=$?"

exit "$RC"
