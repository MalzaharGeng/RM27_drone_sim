#!/usr/bin/env bash
# start_sim.sh —— 一键起 rm27 仿真（自动找 PX4 目录 + world 文件）
#
#   ./start_sim.sh --dry      # 只打印将要执行的命令，不启动
#   ./start_sim.sh --list     # 只列出找到的 world 候选
#   ./start_sim.sh            # 正常启动
#
# 可覆盖：PX4_DIR=...  AIRFRAME=gz_x500_mono_cam  WORLD_FILE=/abs/path.sdf
#
set -u

DRY=0
MODE="run"
case "${1:-}" in
  --dry)  DRY=1 ;;
  --list) MODE="list" ;;
esac

SIM_DIR="$HOME/drone_sim"
AIRFRAME="${AIRFRAME:-gz_x500_mono_cam}"

# ---------- 找 PX4 源码 ----------
PX4="${PX4_DIR:-}"
if [ -z "$PX4" ]; then
  for d in "$HOME/PX4-Autopilot" "$HOME/px4" "$HOME/PX4_Firmware" \
           "$HOME/drone_sim/PX4-Autopilot" "$HOME/src/PX4-Autopilot" \
           "$HOME/PX4-Autopilot-main" "$HOME/PX4"; do
    if [ -f "$d/Makefile" ] && [ -d "$d/Tools/simulation/gz" ]; then
      PX4="$d"; break
    fi
  done
fi
if [ -z "$PX4" ]; then
  # 兜底：在 home 下浅层搜
  PX4=$(find "$HOME" -maxdepth 3 -type d -name "Tools" -path "*/simulation" 2>/dev/null \
        | head -1 | sed 's|/Tools/simulation$||')
fi
if [ -z "$PX4" ] || [ ! -d "$PX4/Tools/simulation" ]; then
  echo "!! 没找到 PX4 源码目录，请手动指定：PX4_DIR=/你的路径 $0" >&2
  exit 1
fi

# ---------- 找 world ----------
WORLD_FILE="${WORLD_FILE:-}"
WORLD=""

if [ -n "$WORLD_FILE" ]; then
  [ -f "$WORLD_FILE" ] || { echo "!! WORLD_FILE 不存在：$WORLD_FILE" >&2; exit 1; }
  WORLD=$(basename "$WORLD_FILE" .sdf)
else
  # 按优先级找：rm27_nav_world > rm27_world > 名字带 world 的任意 sdf
  for name in rm27_nav_world rm27_world; do
    hit=$(find "$SIM_DIR" "$HOME" -maxdepth 5 -type f -name "$name.sdf" 2>/dev/null | head -1)
    if [ -n "$hit" ]; then WORLD_FILE="$hit"; WORLD="$name"; break; fi
  done
  if [ -z "$WORLD_FILE" ]; then
    echo "-- 没找到 rm27_*_world.sdf，退回搜索任意 *world*.sdf" >&2
    WORLD_FILE=$(find "$SIM_DIR" "$HOME" -maxdepth 5 -type f -name "*world*.sdf" 2>/dev/null | head -1)
    [ -n "$WORLD_FILE" ] && WORLD=$(basename "$WORLD_FILE" .sdf)
  fi
fi

if [ -z "$WORLD_FILE" ]; then
  echo "!! 一个 world 文件都没找到。先看看有什么：" >&2
  find "$HOME" -maxdepth 4 -type f -name "*.sdf" 2>/dev/null | head -20 >&2
  exit 1
fi

WORLDS_DIR=$(dirname "$WORLD_FILE")
# 若路径形如 .../rm27_sim/worlds/xx.sdf，则 assets 根是 .../rm27_sim
ASSET_ROOT=$(dirname "$WORLDS_DIR")

if true; then
  echo "=================================================="
  echo " PX4      : $PX4"
  echo " AIRFRAME : $AIRFRAME"
  echo " WORLD    : $WORLD"
  echo " world 文件: $WORLD_FILE"
  echo " 资源根    : $ASSET_ROOT"
  echo "=================================================="
  if [ "$MODE" = "list" ]; then
    echo "全部 world 候选："
    find "$SIM_DIR" -maxdepth 5 -type f -name "*.sdf" 2>/dev/null | sed 's/^/  /'
    exit 0
  fi
fi

# ---------- 环境 ----------
# gz 要能按名字找到 world，并能解析 model:// 与贴图
export GZ_SIM_RESOURCE_PATH="$ASSET_ROOT:$WORLDS_DIR:$ASSET_ROOT/models:$SIM_DIR:$SIM_DIR/models:$PX4/Tools/simulation/gz/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export PX4_GZ_WORLD="$WORLD"

if [ -z "${DISPLAY:-}" ] || [ -n "${HEADLESS:-}" ]; then
  export HEADLESS=1
  echo " 显示     : 无界面（HEADLESS=1）"
else
  echo " 显示     : 有界面 DISPLAY=$DISPLAY"
fi
echo " GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
echo "=================================================="

if [ $DRY -eq 1 ]; then
  echo "将执行： cd $PX4 && make px4_sitl $AIRFRAME"
  exit 0
fi

cd "$PX4" || exit 1
exec make px4_sitl "$AIRFRAME"
