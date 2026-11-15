#!/usr/bin/env bash
# [NP-01 FIX19] 真实 VIO 中断回归 + 注入器就绪检查（本指令授权 1 次飞行）
#
# 固定注入计划（起飞前冻结，不因观察结果调整）：
#   * 实际进入 NAVIGATE 后 6.0 s，且届时仍处于预定自主任务执行状态
#   * 只向唯一 VIO 发布节点发送 SIGTERM
#   * 未达注入点：不发送任何信号，按现有保护退出，记为新的注入前失败，不自动重飞
#
# 就绪检查在本轮启动前完成（--preflight 模式），运行期再复核一次：
#   唯一节点识别、PID/starttime/exe/完整 sha256 一致、
#   launcher/整轮运行器/桥显式排除、注入只发预定信号。
# 注入器与整轮运行器在不同会话：注入器退出不触发整轮无条件 cleanup。
set -uo pipefail
ROOT=/home/gengyinghao/drone_sim
WP=$ROOT/wb_handoff_20261114/workpkg_20261115
RECENV=$WP/newpkg_fix19/recordenv
FIX18=$WP/newpkg_fix18/runenv
LC=$FIX18/np01_lifecycle.py
READY=$RECENV/injection_readiness.py
IDREC=$RECENV/np01_identity_record.py
LAUNCH=$RECENV/launch_run_wide_fix19.sh
NODE_BIN=$WP/bin/stereo_inertial_sim_node.fix15
DROP_AFTER=${DROP_AFTER:-6.0}
PREFIX=${PREFIX:-intr_fix19_r1}
DUR=${DUR:-360}
EV=$WP/newpkg_fix19/evidence
mkdir -p "$EV"

# ---------- 起飞前就绪检查（不启动任何仿真）----------
preflight() {
  echo "===== 注入器起飞前就绪检查 ====="
  [[ -x "$NODE_BIN" ]] || { echo "FAIL frozen node binary missing"; return 1; }
  [[ -f "$READY" ]] || { echo "FAIL readiness module missing"; return 1; }
  [[ -f "$LAUNCH" ]] || { echo "FAIL launcher missing"; return 1; }
  [[ -f "$WP/newpkg_fix19/FROZEN_INTERRUPTION_CRITERIA.json" ]] || {
    echo "FAIL frozen criteria missing"; return 1; }
  python3 "$READY" self-check --work-dir "$EV/injection_preflight" \
    > "$EV/injection_preflight_output.txt" 2>&1
  local rc=$?
  echo "  readiness self-check rc=$rc"
  cat "$EV/injection_preflight_output.txt"
  echo "  frozen node: md5=$(md5sum "$NODE_BIN" | cut -d' ' -f1) sha256=$(sha256sum "$NODE_BIN" | cut -d' ' -f1)"
  echo "  DROP_AFTER=$DROP_AFTER DUR=$DUR PREFIX=$PREFIX"
  return $rc
}

if [[ "${1:-}" == "--preflight" ]]; then
  preflight
  exit $?
fi

# ---------- 离线核查：若发现明确实现错误，本脚本不应被启动 ----------
if [[ -f "$WP/newpkg_fix19/NO_LAUNCH_IMPLEMENTATION_DEFECT" ]]; then
  echo "STOP: 离线核查发现明确实现错误，按指令不原样起飞。见 FIX19_REPORT.md"
  exit 2
fi

# ---------- 启动整轮（独立会话；注入器退出不触发整轮 cleanup）----------
setsid bash "$LAUNCH" 5.0 "$PREFIX" "$DUR" > /tmp/intr_fix19_launcher.log 2>&1 &
L=$!
RD=""
for i in $(seq 1 120); do
  RD=$(grep -m1 '^RUN_DIR=' /tmp/intr_fix19_launcher.log 2>/dev/null | cut -d= -f2)
  [[ -n "${RD:-}" ]] && break
  sleep 0.5
done
echo "RUN_DIR=$RD"
[[ -n "${RD:-}" ]] || { echo "no run dir"; exit 1; }

# 等 NAVIGATE
for i in $(seq 1 1200); do
  grep -qa "state -> NAVIGATE" "$RD/pilot.log" 2>/dev/null && break
  sleep 0.5
done
if ! grep -qa "state -> NAVIGATE" "$RD/pilot.log" 2>/dev/null; then
  echo "NEVER_NAVIGATE（新的注入前失败，不发送信号）"
  { echo "verdict=A"; echo "reason=NEVER_NAVIGATE"; echo "signal_sent=no"; } \
    > "$RD/injection_verdict.txt"
  wait $L; exit 1
fi
NAV_LINE=$(grep -a "state -> NAVIGATE" "$RD/pilot.log" | head -1)
echo "NAVIGATE: $NAV_LINE"
sleep "$DROP_AFTER"

# 6 s 后必须仍处于预定自主任务执行状态
STATE_AT=$(grep -a "state -> " "$RD/pilot.log" | tail -1 | sed 's/.*state -> //')
# [FIX20-TOOL] grep -c 无匹配时输出 "0" 但 rc=1；旧写法 `|| echo 0` 会
# 追加第二行，使变量变成 "0\n0" 并与 "0" 恒不等，导致注入器误判条件不满足。
# 用函数把命令替换的退出码与数值分离，结果永远是单个十进制数。
count_matches() {
  local n
  n=$(grep -ac "$1" "$2" 2>/dev/null | tr -d '\n') || n=0
  [[ -n "$n" ]] || n=0
  printf '%s' "$n"
}
DONE_FLAG=$(count_matches "task_completed" "$RD/bt.log")
REVOKED_FLAG=$(count_matches "task_revoked" "$RD/pilot.log")
if [[ "$STATE_AT" != "NAVIGATE" || "$DONE_FLAG" != "0" || "$REVOKED_FLAG" != "0" ]]; then
  echo "INJECTION_CONDITION_NOT_MET state=$STATE_AT done=$DONE_FLAG revoked=$REVOKED_FLAG"
  { echo "verdict=A"; echo "reason=not_in_autonomous_execution_at_6s";
    echo "state=$STATE_AT"; echo "signal_sent=no"; } > "$RD/injection_verdict.txt"
  wait $L; exit 1
fi

# 运行期就绪复核：唯一节点 + 完整身份
python3 "$READY" check --run-dir "$RD" --node-bin "$NODE_BIN" \
  --registry "$RD/proc_registry.jsonl" --node-pid "$RD/node_pid.json" \
  --out "$RD/injection_readiness.json" > "$RD/injection_readiness.stdout" 2>&1
READY_RC=$?
echo "READINESS_RC=$READY_RC"
if [[ "$READY_RC" != "0" ]]; then
  echo "READINESS_FAILED：不发送任何信号"
  { echo "verdict=A"; echo "reason=readiness_failed"; echo "signal_sent=no"; } \
    > "$RD/injection_verdict.txt"
  wait $L; exit 1
fi
NODE_PID=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["unique_vio_node"])' "$RD/injection_readiness.json")
NODE_ST=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["target_identity"]["starttime_s_boot"])' "$RD/injection_readiness.json")

T_MONO=$(date +%s.%N)
T_EPOCH=$(date +%s.%N)
python3 "$IDREC" node --pid "$NODE_PID" --run-dir "$RD" \
  --expect-exe "$NODE_BIN" --out "$RD/injection_target_identity.json" \
  > "$RD/injection_target_identity.stdout" 2>&1

BT=$ROOT/px4_telemetry_sim_transport.csv
PUB_BEFORE=$(grep -c ',publish,' "$BT" 2>/dev/null || echo 0)

: > "$RD/INJECTION_MARKER"
python3 "$LC" signal-pid --pid "$NODE_PID" --expect-exe "$NODE_BIN" \
  --expect-starttime "$NODE_ST" --signal TERM \
  --out "$RD/injection_signal.json" > "$RD/injection_signal.stdout" 2>&1
SIG_RC=$?

EXITED=0
for i in $(seq 1 50); do
  if [[ ! -e "/proc/$NODE_PID" ]]; then EXITED=1; break; fi
  sleep 0.1
done
BRIDGE_PID=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["pid"])' "$RD/bridge_owner.fix19.json" 2>/dev/null || echo "")
BRIDGE_AFTER=$([[ -n "$BRIDGE_PID" && -e "/proc/$BRIDGE_PID" ]] && echo alive || echo gone)
PUB_AFTER=$(grep -c ',publish,' "$BT" 2>/dev/null || echo 0)

{
  echo "verdict=PENDING（收尾后由专项判据判定）"
  echo "scenario=NP01_FIX19_REAL_VIO_INTERRUPTION"
  echo "drop_after_s=$DROP_AFTER"
  echo "pilot_state_at_inject=$STATE_AT"
  echo "inject_mono=$T_MONO"; echo "inject_epoch=$T_EPOCH"
  echo "target_pid=$NODE_PID"; echo "target_starttime_s_boot=$NODE_ST"
  echo "target_exe=$NODE_BIN"; echo "signal=SIGTERM"
  echo "signal_rc=$SIG_RC"; echo "node_exited_within_5s=$EXITED"
  echo "bridge_pid=$BRIDGE_PID"; echo "bridge_after=$BRIDGE_AFTER"
  echo "publish_rows_before=$PUB_BEFORE"; echo "publish_rows_after=$PUB_AFTER"
} > "$RD/interruption.log"
cat "$RD/interruption.log"

wait $L
echo "LAUNCHER_RC=$?"
tail -5 "$RD/check.txt" 2>/dev/null
