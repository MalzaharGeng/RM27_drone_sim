#!/usr/bin/env bash
# 运行现成的 PX4 SITL 二进制，环境与 start_rm27_px4.sh 完全一致。
#
# 为什么不经 make：本机 build 缓存的 CMakeCache 里记录了 Windows MSYS 的
# protobuf/curl/absl 路径，任何重新 configure 都会把 gz_bridge 的 include
# 指向 /mnt/c/msys64，编译必失败；WSL 侧缺 libprotobuf-dev 且无 sudo 无法补装。
# 因此直接复用 11-11 02:24 已链接好的二进制（lockstep 版本，行为与原来一致），
# 不改动任何飞行或门控参数。
set -uo pipefail

ROOT=/home/gengyinghao/drone_sim
# shellcheck disable=SC1091
source "$ROOT/env.sh"

MODEL="${PX4_GZ_MODEL:-x500_mono_cam}"
# rcS 按文件名 [0-9]+_${PX4_SIM_MODEL} 找机型，实际文件名带 gz_ 前缀，
# 因此这里必须传 gz_x500_mono_cam（与 make 目标同名），而不是裸模型名。
SIM_MODEL="gz_${MODEL#gz_}"
BUILD=$ROOT/PX4-Autopilot/build/px4_sitl_default

[[ -x "$BUILD/bin/px4" ]] || { echo "缺少 $BUILD/bin/px4" >&2; exit 1; }
[[ -f "$ROOT/PX4-Autopilot/Tools/simulation/gz/worlds/$PX4_GZ_WORLD.sdf" ]] || {
  echo "缺少 world $PX4_GZ_WORLD.sdf" >&2; exit 1; }
[[ -d "$ROOT/PX4-Autopilot/Tools/simulation/gz/models/$MODEL" ]] || {
  echo "缺少 model $MODEL" >&2; exit 1; }

echo "Model: $MODEL  World: $PX4_GZ_WORLD  二进制: $BUILD/bin/px4"

# 确保 micro-XRCE-DDS Agent 在跑：PX4 的 uxrce_dds_client 走 udp4:8888 连它，
# Agent 再转给 ROS 2。它原先依赖手工启动后残留的进程，stop_sim.sh 不管理它，
# 一旦被清理，后续所有轮次都收不到 /fmu/* 话题——表现为遥测全空、
# Px4Bridge 报 px4_local_frame_unready，但 PX4 本身照常起降，很容易误判。
# 这里做成幂等启动：已在跑就不重复起。
AGENT_BIN=${XRCE_AGENT_BIN:-/home/gengyinghao/Micro-XRCE-DDS-Agent/build3/MicroXRCEAgent}
AGENT_LOG=${PX4_AGENT_LOG:-/tmp/microxrce_agent.log}
if [[ -x "$AGENT_BIN" ]] && ! pgrep -f "MicroXRCEAgent.*udp4" >/dev/null 2>&1; then
  echo "启动 MicroXRCEAgent (udp4:8888) -> $AGENT_LOG"
  setsid "$AGENT_BIN" udp4 -p 8888 >"$AGENT_LOG" 2>&1 </dev/null &
  sleep 2
fi

cd "$BUILD/src/modules/simulation/gz_bridge" || exit 1
exec /usr/bin/cmake -E env \
  PX4_SIM_MODEL="$SIM_MODEL" \
  PX4_GZ_WORLD="$PX4_GZ_WORLD" \
  GZ_IP=127.0.0.1 \
  "$BUILD/bin/px4"
