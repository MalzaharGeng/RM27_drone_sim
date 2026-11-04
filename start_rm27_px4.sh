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
  cp -f "$src" "$dst"
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
  dst="$PX4_MODELS/$(basename "$src")"
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
exec make px4_sitl gz_x500
