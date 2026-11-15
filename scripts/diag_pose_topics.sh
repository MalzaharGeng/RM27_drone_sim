#!/usr/bin/env bash
# 诊断 Gazebo 位姿话题：找出哪个是飞机本体的绝对世界位姿 [SIM-ONLY]
set -uo pipefail
cd /home/gengyinghao/drone_sim || exit 1
bash ./stop_sim.sh >/dev/null 2>&1
sleep 2
HEADLESS=1 bash ./start_px4_direct.sh >/tmp/diag_gz.log 2>&1 &
P=$!
for _ in $(seq 1 60); do
  grep -qa "gz_bridge" /tmp/diag_gz.log 2>/dev/null && break
  sleep 1
done
sleep 2
echo "=== pose topics ==="
gz topic -l 2>/dev/null | grep -i pose | head -20
echo "=== model topics ==="
gz topic -l 2>/dev/null | grep -iE "x500|model" | head -14
echo "=== dynamic_pose raw sample ==="
T=$(gz topic -l 2>/dev/null | grep "dynamic_pose/info" | head -1)
if [[ -n "$T" ]]; then
  timeout 4 gz topic -e -t "$T" -n 1 2>/dev/null | head -60
fi
bash ./stop_sim.sh >/dev/null 2>&1
kill "$P" 2>/dev/null || true
