#!/usr/bin/env bash
# 诊断 SLAM 图像链路：gz 相机 → ros_gz_bridge → ROS 话题
#
#   ./diag_slam_link.sh
#
# 在「仿真 + run_stereo_slam.sh 都开着」的情况下运行，一次性输出各段状态。
set -uo pipefail

set +u
source "${ROS_SETUP:-/opt/ros/lyrical/setup.bash}"
[[ -f "$HOME/drone_sim/scripts/ros_dds_local.sh" ]] && \
  source "$HOME/drone_sim/scripts/ros_dds_local.sh"
set -u

hr() { printf '\n======== %s ========\n' "$1"; }

hr "0. 环境与进程"
echo "ROS_DISTRO=${ROS_DISTRO:-<unset>}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset，默认 rmw_fastrtps_cpp>}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset=0>}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset>}"
echo "ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-<unset>}"
echo "相关进程："
pgrep -af 'parameter_bridge|stereo_slam_node' | sed 's/^/  /' | head -5

hr "1. gz 侧话题"
LEFT=$(gz topic -l 2>/dev/null | grep -E '/sensor/cam_left/image$' | head -1)
RIGHT=$(gz topic -l 2>/dev/null | grep -E '/sensor/cam_right/image$' | head -1)
echo "LEFT : ${LEFT:-<未找到>}"
echo "RIGHT: ${RIGHT:-<未找到>}"

if [[ -n "${LEFT:-}" ]]; then
  hr "2. gz 侧该话题的连接方（完整输出）"
  gz topic -i -t "$LEFT" 2>&1 | sed 's/^/  /' | head -20
  echo "  ↑ Subscribers 段里应出现 ros_gz_bridge 的地址；为空=桥没连上 gz"
fi

hr "3. ROS 侧话题（--no-daemon）"
timeout 10 ros2 topic list --no-daemon 2>&1 | sed 's/^/  /' | head -20

hr "4. ROS 节点（--no-daemon）"
timeout 10 ros2 node list --no-daemon 2>&1 | sed 's/^/  /' | head -10

hr "5. publisher/subscriber 数量"
if [[ -n "${LEFT:-}" ]]; then
  timeout 10 ros2 topic info --no-daemon "$LEFT" 2>&1 | sed 's/^/  /' | head -6
fi

hr "6. 数据流采样（8 秒）"
if [[ -n "${LEFT:-}" ]]; then
  # 注意：ros2 topic hz 不支持 --no-daemon
  timeout 8 ros2 topic hz "$LEFT" 2>&1 | tail -4 | sed 's/^/  /'
  echo "  ↑ 出现 average rate 即链路通；空=没有消息"
fi

hr "判读"
cat <<'EOF'
  第2段 Subscribers 为空             -> 桥没订阅上 gz（gz-transport 版本/分区不匹配）
  第4段节点为空                      -> DDS 发现层失效（组播问题 / domain / RMW）
  第3段无话题但第4段有节点           -> 桥没建 ROS 发布者
  第5段 publisher=1、第6段无 hz      -> 桥收到 gz 但没转成 ROS
  第6段有 rate                       -> 链路通，问题在节点侧

  第4段为空（连自己的节点都看不到）时，先在本终端确认：
      source ~/drone_sim/scripts/ros_dds_local.sh
  然后重跑本脚本；仍为空则试 ROS_DOMAIN_ID=42 换域。
EOF
