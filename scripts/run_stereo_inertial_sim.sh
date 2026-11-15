#!/usr/bin/env bash
# 启动「实机同源算法」仿真链路（Gazebo 双目 + IMU → ORB_SLAM3 IMU_STEREO → Px4Bridge）
#
#   ./run_stereo_inertial_sim.sh              # 自动探测话题 + 自动起 ros_gz_bridge
#   ./run_stereo_inertial_sim.sh --no-bridge  # 桥已在别处起好，只跑节点
#   ./run_stereo_inertial_sim.sh --viewer     # 打开 ORB-SLAM3 Viewer
#
# 与 run_stereo_slam.sh 的区别（两条路径并存，互不替代）：
#   run_stereo_slam.sh          旧路径：STEREO + /slam/pose + MAVLink 注入 + sim_vio_health 桩
#   run_stereo_inertial_sim.sh  本脚本：IMU_STEREO + Px4Bridge（DDS vehicle_visual_odometry
#                               + 真实 /vio/navigation_health），与 Nano 同一套算法/门控
#
# 依赖：仿真已用 PX4_GZ_MODEL=x500_mono_cam 启动，且 IMU 传感器存在（x500_base）。
#
# 注意：
#   * 本路径发布的 /vio/navigation_health 是 Px4Bridge 的真实门控输出，不是仿真桩。
#     行为树应走严格模式（不要设 WSL_RELAX_FRESHNESS=1），否则验收结论不成立。
#   * 相机内参/基线/相机-IMU 外参全部按仿真几何取值，见
#     ORB_SLAM3/config/RM27_Sim_IMU_Stereo.yaml，不可复制到实机。
set -uo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/lyrical/setup.bash}"
WS="${WS:-$HOME/px4_ws}"
ORB_ROOT="${ORB_ROOT:-$HOME/ORB_SLAM3}"

# libORB_SLAM3.so 运行时需要 libDBoW2/libg2o
export LD_LIBRARY_PATH="$ORB_ROOT/build/lib:$ORB_ROOT/build/Thirdparty/g2o/lib:${LD_LIBRARY_PATH:-}"

# ROS 的 setup.bash 会引用未定义变量，与本脚本的 set -u 冲突，source 期间临时关掉 nounset
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
# WSL 多网卡下的历史兼容层；现在是空实现，负责清掉残留的自定义 DDS 配置
# shellcheck disable=SC1091
[[ -f "$HOME/drone_sim/scripts/ros_dds_local.sh" ]] && \
  source "$HOME/drone_sim/scripts/ros_dds_local.sh"
set -u

WITH_BRIDGE=1
VIEWER=0
for _a in "$@"; do
  case "$_a" in
    --no-bridge) WITH_BRIDGE=0 ;;
    --viewer)    VIEWER=1 ;;
    --no-viewer) VIEWER=0 ;;
  esac
done
if [[ "${VIEWER}" == "1" ]]; then VIEWER_ARG=true; else VIEWER_ARG=false; fi

# ---- 探测 gz 话题（PX4 生成的模型名常带 _0 后缀）----
find_topic() {  # $1 = 正则片段
  gz topic -l 2>/dev/null | grep -E "$1" | head -1
}

LEFT=""
RIGHT=""
IMU=""
for _ in $(seq 1 12); do
  LEFT=$(find_topic "/sensor/cam_left/image$")
  RIGHT=$(find_topic "/sensor/cam_right/image$")
  IMU=$(find_topic "/sensor/imu_sensor/imu$")
  [[ -n "$LEFT" && -n "$RIGHT" && -n "$IMU" ]] && break
  sleep 0.5
done

# 只探测到一侧时按命名规则推出另一侧（左右目总是成对创建）。
if [[ -z "$LEFT" && -n "$RIGHT" ]]; then
  LEFT="${RIGHT//cam_right_link/cam_left_link}"
  LEFT="${LEFT//cam_right/cam_left}"
fi
if [[ -z "$RIGHT" && -n "$LEFT" ]]; then
  RIGHT="${LEFT//cam_left_link/cam_right_link}"
  RIGHT="${RIGHT//cam_left/cam_right}"
fi

if [[ -z "$LEFT" || -z "$RIGHT" ]]; then
  echo "!! 没找到双目相机话题——仿真大概率没在跑，或用的是没接相机的机型" >&2
  echo "   当前匹配到：LEFT='$LEFT'  RIGHT='$RIGHT'" >&2
  echo "   现存话题里与 cam 相关的有：" >&2
  gz topic -l 2>/dev/null | grep -iE 'cam' | sed 's/^/     /' >&2
  echo "   确认仿真用 PX4_GZ_MODEL=x500_mono_cam 启动" >&2
  exit 1
fi

if [[ -z "$IMU" ]]; then
  echo "!! 没找到 IMU 话题（/sensor/imu_sensor/imu）" >&2
  echo "   IMU_STEREO 必须有 IMU；确认模型 include 了 x500_base 且传感器未被禁用" >&2
  echo "   现存话题里与 imu 相关的有：" >&2
  gz topic -l 2>/dev/null | grep -iE 'imu' | sed 's/^/     /' >&2
  exit 1
fi

echo "LEFT : $LEFT"
echo "RIGHT: $RIGHT"
echo "IMU  : $IMU"

BRIDGE_PID=""
cleanup() {
  [[ -n "$BRIDGE_PID" ]] && kill "$BRIDGE_PID" 2>/dev/null
}
trap cleanup EXIT INT TERM

# /clock 必须桥接：节点用它把 gz sim time 映射到 epoch µs，再交给 Px4Bridge 的
# SensorClock。没有 /clock 时节点会退化为「首帧时刻」对齐，跨时钟域验收不成立。
BRIDGE_ARGS=(
  "$LEFT@sensor_msgs/msg/Image[gz.msgs.Image"
  "$RIGHT@sensor_msgs/msg/Image[gz.msgs.Image"
  "$IMU@sensor_msgs/msg/Imu[gz.msgs.IMU"
  "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
)

# 真值位姿（供 accuracy 评估/对照使用）
WORLD_PREFIX="${LEFT%%/model/*}"
TRUTH_TOPIC="$WORLD_PREFIX/dynamic_pose/info"
if gz topic -l 2>/dev/null | grep -qx "$TRUTH_TOPIC"; then
  BRIDGE_ARGS+=("$TRUTH_TOPIC@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V")
  echo "TRUTH: $TRUTH_TOPIC"
else
  echo "TRUTH: (未找到 $TRUTH_TOPIC)"
fi

if [[ "$WITH_BRIDGE" == "1" ]]; then
  echo "--- 启动 ros_gz_bridge ---"
  BRIDGE_LOG=/tmp/rm27_bridge_inertial.log
  : > "$BRIDGE_LOG"
  ros2 run ros_gz_bridge parameter_bridge "${BRIDGE_ARGS[@]}" >"$BRIDGE_LOG" 2>&1 &
  BRIDGE_PID=$!

  bridge_ok=0
  for _ in $(seq 1 20); do
    if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
      echo "!! ros_gz_bridge 已退出，日志如下：" >&2
      sed 's/^/   /' "$BRIDGE_LOG" >&2
      exit 1
    fi
    if timeout 6 ros2 topic list --no-daemon 2>/dev/null | grep -qxF "$LEFT"; then
      bridge_ok=1
      break
    fi
    sleep 0.5
  done

  if [[ "$bridge_ok" == "1" ]]; then
    pubs=$(timeout 6 ros2 topic info --no-daemon "$LEFT" 2>/dev/null | awk -F': ' '/Publisher count/{print $2}')
    echo "bridge 就绪 | ROS 话题可见 | publisher=${pubs:-?}"
  else
    echo "提示：ROS 侧暂时看不到相机话题（可能是 ros2 daemon 或 DDS 发现问题）。"
    echo "      仍继续启动节点；若 [stat] 一直是 L=0，运行 scripts/diag_slam_link.sh 定位。"
    echo "      bridge 日志：$BRIDGE_LOG"
  fi
fi

# PX4 门控配置：默认走严格版（与实机同值）。
# PX4_VIO_CONFIG=transport 时改用传输层补偿版，跑出来的结果不能登记 time_sync 同源验收。
case "${PX4_VIO_CONFIG:-strict}" in
  strict|"")  VIO_CFG="$ORB_ROOT/config/px4_vio_sim.yaml" ;;
  transport)  VIO_CFG="$ORB_ROOT/config/px4_vio_sim_transport.yaml" ;;
  *)          VIO_CFG="$PX4_VIO_CONFIG" ;;
esac
if [[ "$VIO_CFG" == *transport* ]]; then
  echo "!!! 使用传输层补偿配置（[SIM-ONLY]）：time_sync 门限 0.25 s，非实机等价"
fi

echo "--- 启动 stereo_inertial_sim_node (viewer=$VIEWER_ARG, sync_tol=${SYNC_TOL:-0.005}s) ---"
echo "    settings: ${ORB_SETTINGS_FILE:-$ORB_ROOT/config/RM27_Sim_IMU_Stereo.yaml}"
echo "    px4 cfg : $VIO_CFG"
echo "    record  : ${ORB_RECORD_DIR:-<off>}  latency_csv: ${LATENCY_CSV:-<off>}"
echo "    sim_nav_guard: ${SIM_NAV_GUARD:-false}"
echo "    提醒：这是真实门控路径，行为树请勿设 WSL_RELAX_FRESHNESS=1"

# 空字符串不能作为 -p 参数值传给 rcl（'-p record_dir:=' 会让节点直接
# RCLInvalidROSArgsError 退出）。只在真正需要时追加这两个参数。
EXTRA_ARGS=()
if [[ -n "${ORB_RECORD_DIR:-}" ]]; then
  EXTRA_ARGS+=(-p "record_dir:=$ORB_RECORD_DIR")
fi
if [[ -n "${LATENCY_CSV:-}" ]]; then
  EXTRA_ARGS+=(-p "latency_csv:=$LATENCY_CSV")
fi
# [WORKPKG-TRACE] same run directory as latency.csv; TRACE_CSV overrides.
TRACE_PATH=""
if [[ -n "${LATENCY_CSV:-}" ]]; then
  TRACE_PATH="${LATENCY_CSV%.csv}_time_trace.csv"
fi
if [[ -n "${TRACE_CSV:-}" ]]; then TRACE_PATH="$TRACE_CSV"; fi
if [[ -n "$TRACE_PATH" ]]; then
  EXTRA_ARGS+=(-p "trace_csv:=$TRACE_PATH")
fi

# 不用 exec：保留 shell 才能让 EXIT trap 收掉上面的 bridge 进程
# [PHASE7-DIAG] Independent binary path for the phase7 build; install/ is
# left untouched. When ORB_NODE_BIN is unset this is byte-identical to before.
if [[ -n "${ORB_NODE_BIN:-}" ]]; then
  NODE_CMD=("$ORB_NODE_BIN")
else
  NODE_CMD=(ros2 run rm27_slam stereo_inertial_sim_node)
fi
"${NODE_CMD[@]}" --ros-args \
  -p left_topic:="$LEFT" \
  -p right_topic:="$RIGHT" \
  -p imu_topic:="$IMU" \
  -p clock_topic:="/clock" \
  -p vocab_file:="$ORB_ROOT/Vocabulary/ORBvoc.txt" \
  -p settings_file:="${ORB_SETTINGS_FILE:-$ORB_ROOT/config/RM27_Sim_IMU_Stereo.yaml}" \
  -p px4_config:="$VIO_CFG" \
  -p use_viewer:="$VIEWER_ARG" \
  -p sync_tolerance:="${SYNC_TOL:-0.005}" \
  "${EXTRA_ARGS[@]}" \
  -p init_gate:="${INIT_GATE:-off}" \
  -p init_gate_airborne_hold:="${INIT_GATE_AIRBORNE_HOLD:-0.50}" \
  -p init_gate_health_window:="${INIT_GATE_HEALTH_WINDOW:-1.00}" \
  -p init_gate_min_img_hz:="${INIT_GATE_MIN_IMG_HZ:-15.0}" \
  -p sim_nav_guard:="${SIM_NAV_GUARD:-false}"
