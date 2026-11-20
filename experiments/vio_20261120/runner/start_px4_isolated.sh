#!/usr/bin/env bash
# [STAGE-20261120] 隔离 PX4 SITL 启动器（本阶段副本）
# 相对 autoinit_ekf_semantics_20261119_2336/start_px4_isolated.sh 的唯一差异：
#   MicroXRCEAgent 启动后立刻按「pid + starttime + cmdline」登记，
#   供清理阶段按身份回收。agent 由 setsid 自成会话，sid 不等于运行器 sid，
#   旧回收条件永远匹配不上，是每轮残留 8888 占用的根因。
# 其余：二进制/world/model/参数目录完全不变。
set -uo pipefail
ROOT=${DRONE_SIM_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}
source "$ROOT/env.sh"
ISO_DIR=${ISO_DIR:?ISO_DIR required}
GZ_PARTITION=${GZ_PARTITION:-iso_diag}
MODEL=x500_mono_cam
SIM_MODEL=gz_$MODEL
BUILD=$ROOT/PX4-Autopilot/build/px4_sitl_default
[[ -d "$ISO_DIR" ]] || { echo "no ISO_DIR $ISO_DIR" >&2; exit 1; }
[[ -L "$ISO_DIR/etc" ]] || ln -sfn "$BUILD/etc" "$ISO_DIR/etc"
[[ -L "$ISO_DIR/test_data" ]] || ln -sfn "$ROOT/PX4-Autopilot/test_data" "$ISO_DIR/test_data"

AGENT_BIN=${XRCE_AGENT_BIN:-$HOME/Micro-XRCE-DDS-Agent/build3/MicroXRCEAgent}
AGENT_LOG=${PX4_AGENT_LOG:-/tmp/microxrce_agent.log}
AGENT_OWNER_JSON=${PX4_AGENT_OWNER_JSON:-}
REG=${AGENT_REGISTRY_TOOL:-$(dirname "$0")/agent_registry.py}

AGENT_PID=0
AGENT_STARTED=0
if [[ -x "$AGENT_BIN" ]] && ! pgrep -f "MicroXRCEAgent.*udp4" >/dev/null 2>&1; then
  setsid "$AGENT_BIN" udp4 -p 8888 >"$AGENT_LOG" 2>&1 </dev/null &
  AGENT_PID=$!
  AGENT_STARTED=1
  sleep 2
fi

# 无论本轮是否启动，都落一份登记：started_by_us=0 时清理阶段一定不杀。
if [[ -n "$AGENT_OWNER_JSON" ]] && [[ -f "$REG" ]]; then
  python3 "$REG" record --out "$AGENT_OWNER_JSON" --binary "$AGENT_BIN" \
    --pid "$AGENT_PID" --started-by-us "$AGENT_STARTED" \
    --round-sid "${ROUND_SID:-0}" >/dev/null 2>&1 \
    || echo "[AGENT] 登记失败（不影响启动）" >&2
fi

cd "$ROOT" || exit 1
exec 3< <(sleep 86400)
exec /usr/bin/cmake -E env \
  PX4_SIM_MODEL="$SIM_MODEL" PX4_GZ_WORLD="$PX4_GZ_WORLD" GZ_IP=127.0.0.1 \
  GZ_PARTITION="$GZ_PARTITION" \
  "$BUILD/bin/px4" -w "$ISO_DIR" "$ISO_DIR" <&3
