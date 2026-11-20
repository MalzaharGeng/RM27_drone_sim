#!/usr/bin/env bash
# DDS 自检：两个独立 ROS 进程互发一条消息，不涉及 gz / ORB_SLAM3。
#
#   ./dds_selftest.sh
set -uo pipefail

set +u
source "${ROS_SETUP:-/opt/ros/lyrical/setup.bash}"
[[ -f "$HOME/drone_sim/scripts/ros_dds_local.sh" ]] && \
  source "$HOME/drone_sim/scripts/ros_dds_local.sh"
set -u

hr() { printf '\n---- %s ----\n' "$1"; }

hr "环境"
echo "ROS_DOMAIN_ID = ${ROS_DOMAIN_ID:-<unset=0>}"
echo "RMW           = ${RMW_IMPLEMENTATION:-<默认 rmw_fastrtps_cpp>}"
echo "PROFILES      = ${FASTDDS_DEFAULT_PROFILES_FILE:-<unset>}"
echo "LOCALHOST_ONLY= ${ROS_LOCALHOST_ONLY:-<unset>}"
echo "CYCLONEDDS_URI= ${CYCLONEDDS_URI:-<unset>}" | cut -c1-160

hr "网络接口"
ip -o addr show 2>/dev/null | awk '{printf "  %-8s %-10s %s\n", $2, $3, $4}' || echo "  (ip 不可用)"
hr "路由"
ip route 2>/dev/null | sed 's/^/  /' || true

hr "0. UDP 回环自测（不依赖任何 DDS）"
python3 - <<'PY'
import socket, threading
srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
srv.bind(('127.0.0.1', 0))
port = srv.getsockname()[1]
got = []

def recv():
    srv.settimeout(3.0)
    try:
        got.append(srv.recvfrom(64)[0])
    except Exception:
        pass

t = threading.Thread(target=recv)
t.start()
c = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
c.sendto(b'ping', ('127.0.0.1', port))
t.join()
if got and got[0] == b'ping':
    print('  UDP 127.0.0.1 -> OK')
else:
    print('  UDP 127.0.0.1 -> 失败  ← 本机 UDP 回环不通，任何 DDS 都不可能发现彼此')
PY

hr "DDS 共享内存段"
ls -1 /dev/shm 2>/dev/null | grep -c fastdds | sed 's/^/  数量: /'
ls -1 /dev/shm 2>/dev/null | grep fastdds | head -5 | sed 's/^/  /'

hr "清理 ros2 CLI 守护进程（可能持有旧图状态）"
ros2 daemon stop 2>&1 | sed 's/^/  /' || true

TOPIC=/__dds_selftest
RXLOG=/tmp/dds_selftest_rx.log
TXLOG=/tmp/dds_selftest_tx.log
: > "$RXLOG"; : > "$TXLOG"

hr "1/2 后台监听 $TOPIC"
timeout 25 ros2 topic echo "$TOPIC" std_msgs/msg/String >"$RXLOG" 2>&1 &
RX=$!
sleep 4

hr "2/2 发布 6 条消息"
timeout 15 ros2 topic pub -r 5 -t 6 "$TOPIC" std_msgs/msg/String "{data: hello_dds}" \
  >"$TXLOG" 2>&1
sleep 2
kill "$RX" 2>/dev/null
wait "$RX" 2>/dev/null

hr "接收端输出"
head -14 "$RXLOG" | sed 's/^/  /'

echo
if grep -q hello_dds "$RXLOG"; then
  echo "结果：DDS 正常 —— 收发链路可用"
  exit 0
fi

echo "结果：DDS 不通"
echo "  发布端输出（前 6 行）："
head -6 "$TXLOG" | sed 's/^/    /'
cat <<'EOF'

  逐条排查：
    a) 接收端是否提前退出（出现 !rclpy.ok()）：换一个更长的等待时间再试
    b) 接口列里 lo 是否有 127.0.0.1/8；没有则 loopback 异常
    c) /dev/shm 里 fastdds_* 数量很多（几十个）说明有僵死 participant，
       可清理旧段：rm -f /dev/shm/fastdds_*（进程全停后再删）
    d) 仍不通时可临时换域：ROS_DOMAIN_ID=77 ./dds_selftest.sh
EOF
exit 1
