#!/usr/bin/env bash
# 定位中断测试：control_truth 1 m 正常起飞并进入 NAVIGATE 后，
# 停掉唯一的外部定位源（truth_odom_node），观察真实 health/授权撤销与飞手退出。
# 不改变任何门限；只做进程级故障注入，并留下时间线证据。
set -uo pipefail
ROOT=/home/gengyinghao/drone_sim
SCRIPTS=$ROOT/scripts
DROP_AFTER=${DROP_AFTER:-6}          # 进入 NAVIGATE 后等待秒数
DURATION=${DURATION:-180}
D=$ROOT/logs/fault_injection_$(date +%Y%m%d_%H%M%S)
mkdir -p "$D"
echo "$D" > /tmp/last_simrun
PROFILE=control_truth DISTANCE=1.0 DURATION=$DURATION \
  PX4_VIO_CONFIG=transport SIM_RUN="$D" KEEP_RUNS=12 \
  bash "$SCRIPTS/run_profile.sh" >"$D/outer.log" 2>&1 &
RUNNER=$!
echo "runner_pid=$RUNNER run=$D"
INJECTED=0
for i in $(seq 1 $((DURATION + 60))); do
  if grep -qa "state -> NAVIGATE" "$D/pilot.log" 2>/dev/null; then
    T0=$(date +%s.%N)
    NAV=$(grep -a "state -> NAVIGATE" "$D/pilot.log" | head -1)
    echo "NAVIGATE: $NAV" >> "$D/fault_inject.log"
    sleep "$DROP_AFTER"
    T1=$(date +%s.%N)
    if pkill -x truth_odom_node; then
      echo "DROP_OK epoch=$T1 target=truth_odom_node after=${DROP_AFTER}s" >> "$D/fault_inject.log"
      INJECTED=1
    else
      echo "DROP_FAILED epoch=$T1" >> "$D/fault_inject.log"
    fi
    echo "{\"injected\":$INJECTED,\"nav_line\":\"$NAV\",\"t_nav_epoch\":$T0,\"t_drop_epoch\":$T1,\"drop_after_s\":$DROP_AFTER,\"target\":\"truth_odom_node\"}" > "$D/fault_inject.json"
    break
  fi
  kill -0 "$RUNNER" 2>/dev/null || break
  sleep 1
done
wait "$RUNNER"; RC=$?
echo "runner_exit=$RC injected=$INJECTED run=$D" | tee -a "$D/fault_inject.log"
echo "=== 撤销/退出证据 ==="
grep -a -E "Task revoked|task_revoked|state -> (TAKEOVER|LAND|LANDED|DONE|FAULT)|navigation_health" "$D/bt.log" "$D/pilot.log" 2>/dev/null | tail -25
echo "=== check ==="; tail -20 "$D/check.txt" 2>/dev/null
