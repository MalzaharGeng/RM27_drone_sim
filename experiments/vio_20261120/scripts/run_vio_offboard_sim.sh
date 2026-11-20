#!/usr/bin/env bash
# 仿真里跑「VIO(真值占位) + Offboard」的一键启动器。
#
#   终端 A:  ./run_vio_offboard_sim.sh          # 起 ros_gz_bridge + VIO 注入
#   终端 B:  ~/drone_sim/start_rm27_px4.sh      # 起 PX4 SITL + Gazebo（先起也行）
#   终端 C:  python3 ~/drone_sim/scripts/rm27_nav.py --help   # 跑 offboard
#
# 说明：本脚本只负责把 gz 真值当 VIO 注入 PX4，让 EKF2 具备视觉位置；
#       offboard 航线仍用现有的 rm27_nav.py（MAVSDK）。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="${PX4_GZ_WORLD:-rm27_world}"
ORIGIN="${VIO_ORIGIN:-5 6 0.5}"     # 与 env.sh 的 PX4_GZ_MODEL_POSE 一致
MAV="${VIO_MAV:-udpout:127.0.0.1:14540}"

set +u
source /opt/ros/lyrical/setup.bash
set -u

python3 - <<'PY' || { echo '缺依赖: pip install pymavlink numpy' >&2; exit 1; }
import pymavlink, numpy, rclpy, tf2_msgs  # noqa: F401
print('[check] pymavlink / numpy / rclpy / tf2_msgs 就绪')
PY

PIDS=()
cleanup() {
  echo
  echo '[stop] 收尾 ...'
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[1/2] ros_gz_bridge: /world/$WORLD/dynamic_pose/info -> /tf"
ros2 run ros_gz_bridge parameter_bridge \
  "/world/$WORLD/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V" \
  >/tmp/vio_bridge_gz.log 2>&1 &
PIDS+=($!)

sleep 2
echo "[2/2] VIO 注入（真值 → VISION_POSITION_ESTIMATE）"
python3 "$ROOT/scripts/vio_truth_bridge.py" --world "$WORLD" --mav "$MAV" --origin $ORIGIN &
PIDS+=($!)

cat <<EOF

────────────────────────────────────────────
两个进程已在后台：
  ros_gz_bridge → /tmp/vio_bridge_gz.log
  vio_truth_bridge → 本终端输出

验证 EKF 是否吃到视觉（在 PX4 的 pxh> 终端里）：
  listener vehicle_local_position        # xy/z 应稳定、无漂移
  listener estimator_status_flags        # ev_pos / ev_hgt / ev_yaw 应为 true
  listener vehicle_attitude              # 姿态正常

确认视觉生效后再跑 offboard：
  python3 ~/drone_sim/scripts/rm27_nav.py --help
  # 或只做起飞悬停：
  python3 ~/drone_sim/scripts/rm27_waypoints.py --help

注意：
  * 本脚本会把 EKF2 切成纯视觉（EKF2_GPS_CTRL=0, HGT_REF=3）。
    想恢复 GPS 主用：pxh> param set EKF2_GPS_CTRL 3 && param set EKF2_HGT_REF 1
  * Ctrl-C 停注入；offboard 断流 PX4 会 failsafe 降落，属预期。
────────────────────────────────────────────
EOF

wait
