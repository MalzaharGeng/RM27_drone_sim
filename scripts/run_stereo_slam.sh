#!/usr/bin/env bash
# 启动双目 ORB_SLAM3 节点（仿真）
#
#   ./run_stereo_slam.sh            # 自动探测左右目话题 + 自动起 ros_gz_bridge
#   ./run_stereo_slam.sh --no-bridge  # 桥已在别处起好，只跑节点
#
# 依赖：仿真已用 PX4_GZ_MODEL=x500_mono_cam 启动
set -uo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/lyrical/setup.bash}"
WS="${WS:-$HOME/px4_ws}"
ORB_ROOT="${ORB_ROOT:-$HOME/ORB_SLAM3}"

# libORB_SLAM3.so 运行时需要 libDBoW2/libg2o
export LD_LIBRARY_PATH="$ORB_ROOT/build/lib:$ORB_ROOT/build/Thirdparty/g2o/lib:${LD_LIBRARY_PATH:-}"

# ROS 的 setup.bash 会引用未定义变量，与本脚本的 set -u 冲突，
# 所以在 source 期间临时关掉 nounset。
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
# WSL 多网卡下 Fast DDS 组播发现可能整体失效，限制到 loopback
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

# ---- 探测 gz 相机话题（PX4 生成的模型名常带 _0 后缀）----
find_topic() {  # $1 = cam_left|cam_right
  gz topic -l 2>/dev/null | grep -E "/sensor/$1/image$" | head -1
}

# 相机是仿真启动后逐步 advertise 的，单次查询容易只抓到一半，所以重试几轮
LEFT=""
RIGHT=""
for _ in $(seq 1 12); do
  LEFT=$(find_topic cam_left)
  RIGHT=$(find_topic cam_right)
  [[ -n "$LEFT" && -n "$RIGHT" ]] && break
  sleep 0.5
done

# 只探测到一侧时按命名规则推出另一侧（左右目总是成对创建）。
# 先替换 *_link，再替换裸的 cam_left/cam_right，避免误改。
if [[ -z "$LEFT" && -n "$RIGHT" ]]; then
  LEFT="${RIGHT//cam_right_link/cam_left_link}"
  LEFT="${LEFT//cam_right/cam_left}"
fi
if [[ -z "$RIGHT" && -n "$LEFT" ]]; then
  RIGHT="${LEFT//cam_left_link/cam_right_link}"
  RIGHT="${RIGHT//cam_left/cam_right}"
fi

if [[ -z "$LEFT" || -z "$RIGHT" ]]; then
  echo "!! 没找到相机话题——仿真大概率没在跑（或用的是没接相机的机型）" >&2
  echo "   当前匹配到：LEFT='$LEFT'  RIGHT='$RIGHT'" >&2
  echo "   现存话题里与 cam 相关的有：" >&2
  gz topic -l 2>/dev/null | grep -iE 'cam' | sed 's/^/     /' >&2
  echo "   确认仿真用 PX4_GZ_MODEL=x500_mono_cam 启动，且日志里有 Loaded system [gz::sim::systems::Sensors]" >&2
  exit 1
fi

echo "LEFT : $LEFT"
echo "RIGHT: $RIGHT"

BRIDGE_PID=""
cleanup() {
  [[ -n "$BRIDGE_PID" ]] && kill "$BRIDGE_PID" 2>/dev/null
}
trap cleanup EXIT INT TERM

BRIDGE_ARGS=(
  "$LEFT@sensor_msgs/msg/Image[gz.msgs.Image"
  "$RIGHT@sensor_msgs/msg/Image[gz.msgs.Image"
)

# 真值位姿（供 slam_pose_to_px4.py --compare 使用）。话题名前缀取自相机话题，
# 形如 /world/rm27_world/model/xxx/link/... → /world/rm27_world
WORLD_PREFIX="${LEFT%%/model/*}"
TRUTH_TOPIC="$WORLD_PREFIX/dynamic_pose/info"
if gz topic -l 2>/dev/null | grep -qx "$TRUTH_TOPIC"; then
  BRIDGE_ARGS+=("$TRUTH_TOPIC@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V")
  echo "TRUTH: $TRUTH_TOPIC"
  TRUTH_ENABLED=1
else
  echo "TRUTH: (未找到 $TRUTH_TOPIC，--compare 将拿不到真值)"
  TRUTH_ENABLED=0
fi

if [[ "$WITH_BRIDGE" == "1" ]]; then
  echo "--- 启动 ros_gz_bridge ---"
  BRIDGE_LOG=/tmp/rm27_bridge.log
  : > "$BRIDGE_LOG"
  ros2 run ros_gz_bridge parameter_bridge "${BRIDGE_ARGS[@]}" >"$BRIDGE_LOG" 2>&1 &
  BRIDGE_PID=$!

  # 桥进程崩了要立刻报错；但 ROS 话题列表不可靠（daemon 可能卡住），
  # 所以只用它做提示，不阻塞启动——节点自己的 [stat] 才是权威。
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

echo "--- 启动 stereo_slam_node (viewer=$VIEWER_ARG, sync_tol=${SYNC_TOL:-0.15}s) ---"
# 存前两对图像，便于确认相机画面内容（全黑/纯色时 ORB 永远初始化不了）
DUMP_DIR="${DUMP_DIR:-/tmp/slam_frames}"
mkdir -p "$DUMP_DIR" 2>/dev/null || DUMP_DIR=""

# 不用 exec：保留 shell 才能让 EXIT trap 收掉上面的 bridge 进程
ros2 run rm27_slam stereo_slam_node --ros-args \
  -p left_topic:="$LEFT" \
  -p right_topic:="$RIGHT" \
  -p vocab_file:="$ORB_ROOT/Vocabulary/ORBvoc.txt" \
  -p settings_file:="$ORB_ROOT/config/RM27_Sim_D435i_Stereo.yaml" \
  -p use_viewer:="$VIEWER_ARG" \
  -p sync_tolerance:="${SYNC_TOL:-0.15}" \
  -p dump_dir:="${DUMP_DIR}"
