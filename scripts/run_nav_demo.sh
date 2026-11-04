#!/usr/bin/env bash
# One-shot demo: start the sim, fly the mission, stop the sim.
#
#   ./scripts/run_nav_demo.sh                                   # 四段避障演示
#   ./scripts/run_nav_demo.sh --enemy-base --hold 5             # 直接飞到敌方基地
#   ./scripts/run_nav_demo.sh --target 25.5,8 --hold 5          # 飞到指定 ENU 世界坐标
#   PX4_GZ_WORLD=rm27_world ./scripts/run_nav_demo.sh           # 纯场地对照
#
# 额外参数会原样传给 rm27_nav.py（--target / --hold / --altitude / --dry-run）。
# PX4 与 Gazebo 的输出写进 logs/sim_console.log。
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-rm27_nav_world}"
# shellcheck disable=SC1091
source "$ROOT/env.sh"

mkdir -p "$ROOT/logs"
SIM_LOG="$ROOT/logs/sim_console.log"
: > "$SIM_LOG"

echo "== 清理残留进程 =="
./stop_sim.sh >/dev/null 2>&1 || true

echo "== 启动仿真（世界 $PX4_GZ_WORLD）=="
# PX4 的 pxh 提示符在非交互 stdin 下会死循环刷屏（一分钟能灌几百 MB），
# 只保留 INFO/WARN/ERROR 三类有用日志，顺便把 \r 换成 \n。
( setsid nohup "$ROOT/start_rm27_px4.sh" </dev/null 2>&1 \
    | sed -u 's/\r/\n/g' \
    | grep --line-buffered -a -E "INFO |WARN |ERROR |Fatal|FATAL" \
    > "$SIM_LOG" ) &

echo "== 等 PX4 拉起 gz_bridge =="
ready=0
for i in $(seq 1 150); do
  if grep -q "gz_bridge" "$SIM_LOG" 2>/dev/null; then
    ready=1
    break
  fi
  if [[ "$i" -gt 15 ]] && ! pgrep -f "bin/px4" >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

if [[ "$ready" -ne 1 ]]; then
  echo "PX4 没起来，$SIM_LOG 最后 30 行：" >&2
  tail -30 "$SIM_LOG" >&2
  ./stop_sim.sh >/dev/null 2>&1 || true
  exit 1
fi
echo "   gz_bridge 已启动"

# 纯场地没有 nav_obstacles 方块，地图也得跟着换，否则会绕不存在的障碍。
if [[ "$PX4_GZ_WORLD" == "rm27_world" ]]; then
  MAP="$ROOT/scripts/obstacles_rm27_plain.json"
else
  MAP="$ROOT/scripts/obstacles_rm27.json"
fi
echo "   地图: $(basename "$MAP")"

echo "== 执行任务 =="
MISSION_LOG="$ROOT/logs/mission.log"
python3 "$ROOT/scripts/rm27_nav.py" --map "$MAP" "$@" 2>&1 | tee "$MISSION_LOG"
status=${PIPESTATUS[0]}

echo "== 关闭仿真 =="
./stop_sim.sh || true
exit "$status"
