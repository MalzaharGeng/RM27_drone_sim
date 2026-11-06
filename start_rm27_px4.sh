#!/usr/bin/env bash
# Launch PX4 SITL (x500) inside the RM27 arena with Gazebo Jetty + WSLg GUI.
#
#   ./start_rm27_px4.sh                               # rm27_world (plain arena)
#   PX4_GZ_WORLD=rm27_nav_world ./start_rm27_px4.sh   # arena + nav blocks
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$ROOT/env.sh"

# PX4 launches gz with Tools/simulation/gz/worlds/<PX4_GZ_WORLD>.sdf, so the
# project copies have to be present there before every launch. rm27_world.sdf
# is hardlinked into that directory, and cp refuses to copy a file onto
# itself, so skip anything that already is the same inode.
PX4_WORLDS="$ROOT/PX4-Autopilot/Tools/simulation/gz/worlds"
mkdir -p "$PX4_WORLDS"
for src in "$ROOT"/rm27_sim/worlds/*.sdf; do
  dst="$PX4_WORLDS/$(basename "$src")"
  if [[ -e "$dst" && "$src" -ef "$dst" ]]; then
    continue
  fi
  # 不能直接 cp -f：若 dst 是指向 src 的硬链接，cp 会先截断目标（也就是 src
  # 本身），把两边一起毁掉。先写临时文件，再用 mv 原子替换目录项。
  tmp="$dst.new.$$"
  cp -f -- "$src" "$tmp" && mv -f -- "$tmp" "$dst" || {
    echo "同步 world 失败: $src -> $dst" >&2
    rm -f -- "$tmp" 2>/dev/null
    exit 1
  }
done

if [[ ! -f "$PX4_WORLDS/$PX4_GZ_WORLD.sdf" ]]; then
  echo "world '$PX4_GZ_WORLD' not found in $PX4_WORLDS" >&2
  exit 1
fi

# PX4's launcher points GZ_SIM_RESOURCE_PATH at its own models directory, so
# model://rmuc_2027 and model://nav_obstacles are only found if they are
# reachable from there. Symlink the project models in instead of copying.
PX4_MODELS="$ROOT/PX4-Autopilot/Tools/simulation/gz/models"
mkdir -p "$PX4_MODELS"
for src in "$ROOT"/rm27_sim/models/*/; do
  src="${src%/}"
  name="$(basename "$src")"
  dst="$PX4_MODELS/$name"
  # x500_mono_cam 覆盖 PX4 自带模型：目标是真实目录，不能整目录 symlink，
  # 只替换 model.sdf。首次部署把目标里的现有文件备份为 model.sdf.orig_px4；
  # 若该目录此前已被项目覆盖，备份内容是上一次部署版，不是 PX4 原版。
  if [[ "$name" == "x500_mono_cam" ]]; then
    mkdir -p "$dst"
    if [[ -f "$dst/model.sdf" && ! -e "$dst/model.sdf.orig_px4" ]]; then
      cp -f -- "$dst/model.sdf" "$dst/model.sdf.orig_px4" || true
    fi
    if ! cmp -s -- "$src/model.sdf" "$dst/model.sdf"; then
      cp -f -- "$src/model.sdf" "$dst/model.sdf"
    fi
    continue
  fi
  if [[ ! -e "$dst" ]]; then
    ln -s "$src" "$dst"
  fi
done

# Rendering must not silently fall back to the software rasteriser.
if command -v glxinfo >/dev/null 2>&1; then
  gpu="$(glxinfo -B 2>/dev/null || true)"
  if ! grep -q "RTX 5070 Ti" <<<"$gpu" || ! grep -q "Accelerated: yes" <<<"$gpu"; then
    echo "GPU preflight failed, Gazebo would render on llvmpipe." >&2
    grep -E "Device|Accelerated" <<<"$gpu" >&2 || true
    echo "Expected: D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU) / Accelerated: yes" >&2
    exit 1
  fi
else
  echo "glxinfo not installed (mesa-utils), skipping the GPU preflight check" >&2
fi

cd "$ROOT/PX4-Autopilot"
echo "World: $PX4_GZ_WORLD | Pose: $PX4_GZ_MODEL_POSE"
MODEL="${PX4_GZ_MODEL:-x500}"    # x500 | x500_mono_cam | x500_mono_cam_down | x500_depth
# rm27_d435i 只是一个参考模型目录：PX4 的 gz_<name> make 目标是在 configure
# 阶段按已知机型列表生成的，新建目录不会被识别。它的双目参数已经合并进
# x500_mono_cam（50 mm 基线、1360x960@30、91.1° 视场），这里自动纠正。
if [[ "$MODEL" == "rm27_d435i" ]]; then
  echo "提示：rm27_d435i 没有对应的 make 目标，自动改用 x500_mono_cam（同一套双目参数）"
  MODEL=x500_mono_cam
fi
if [[ ! -d "$ROOT/PX4-Autopilot/Tools/simulation/gz/models/$MODEL" ]]; then
  echo "gz model '$MODEL' not found under Tools/simulation/gz/models" >&2
  exit 1
fi

# 提前确认 make 目标存在，否则 ninja 只会丢一句 unknown target
NINJA="$ROOT/PX4-Autopilot/build/px4_sitl_default/build.ninja"
if [[ -f "$NINJA" ]] && ! grep -q "gz_$MODEL\b" "$NINJA"; then
  echo "make 目标 gz_$MODEL 不存在（PX4 的机型列表在 configure 阶段固定）" >&2
  echo "可用机型：$(grep -ohE 'gz_[a-z0-9_]+' "$NINJA" | sort -u | tr '\n' ' ')" >&2
  exit 1
fi
echo "Model: $MODEL"
case "$MODEL" in
  x500_mono_cam)      W="/world/${PX4_GZ_WORLD:-rm27_world}/model/$MODEL/link"
                      echo "Stereo LEFT : $W/cam_left_link/sensor/cam_left/image   (1360x960@30, 平视)"
                      echo "Stereo RIGHT: $W/cam_right_link/sensor/cam_right/image" ;;
  x500_mono_cam_down) echo "Camera topic: /world/${PX4_GZ_WORLD:-rm27_world}/model/$MODEL/link/camera_link/sensor/imager/image  (1270x960@30, 朝下)" ;;
  x500_depth)         W="/world/${PX4_GZ_WORLD:-rm27_world}/model/$MODEL/link/camera_link/sensor"
                      echo "RGB   topic: $W/IMX214/image          (1920x1080@30, 平视)"
                      echo "Depth topic: $W/depth_camera/image    + /depth_image  (640x480@30)" ;;
esac
echo "提示：PX4 生成的实例名常带 _0 后缀，实际话题以 gz topic -l | grep cam_left 为准"
exec make px4_sitl "gz_$MODEL"
