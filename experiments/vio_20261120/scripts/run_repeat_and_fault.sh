#!/usr/bin/env bash
# 成功轮之后的 3 次冷启动重复 + 1 次定位中断测试。
# 每轮都是独立冷启动（run_profile.sh 内部 stop_sim + PX4 SITL 重启）。
set -uo pipefail
cd /home/gengyinghao/drone_sim/scripts || exit 1
export KEEP_RUNS=18
for i in 1 2 3; do
  echo "=== COLD_REPEAT $i  $(date +%T) ==="
  PROFILE=control_truth DISTANCE=5.0 DURATION=180 \
    PX4_VIO_CONFIG=transport bash ./run_profile.sh > /tmp/wb_repeat_$i.log 2>&1
  echo "repeat_$i exit=$? $(grep -a 'VERDICT=' /tmp/wb_repeat_$i.log | tail -1)"
done
echo "=== FAULT_INJECTION  $(date +%T) ==="
DROP_AFTER=6 DURATION=180 bash ./run_fault_injection.sh > /tmp/wb_fault.log 2>&1
echo "fault exit=$? log=/tmp/wb_fault.log"
tail -35 /tmp/wb_fault.log
