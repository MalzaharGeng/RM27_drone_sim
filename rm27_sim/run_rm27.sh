#!/usr/bin/env bash
# Linux/WSL port of start_rm27.ps1 — launches the RMUC 2027 arena in Gazebo Jetty.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_ROOT="$HERE/models"
MODEL_SDF="$MODEL_ROOT/rmuc_2027/model.sdf"
MODEL_CONFIG="$MODEL_ROOT/rmuc_2027/model.config"
MESH="$MODEL_ROOT/rmuc_2027/meshes/arena_textured.obj"
MAP="$MODEL_ROOT/rmuc_2027/meshes/rmuc2027_floor.png"
WORLD="$HERE/worlds/rm27_world.sdf"
GUI_CONFIG="$HERE/worlds/rm27_gui.config"

for f in "$MODEL_SDF" "$MODEL_CONFIG" "$MESH" "$MAP" "$WORLD" "$GUI_CONFIG"; do
  [[ -f "$f" ]] || { echo "Required file not found: $f" >&2; exit 1; }
done

# ROS 2 Lyrical vendors Gazebo Jetty (gz-sim 10) and the `gz` CLI.
if [[ -f /opt/ros/lyrical/setup.bash ]]; then
  # ROS setup scripts reference unset variables; relax nounset while sourcing.
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/lyrical/setup.bash
  set -u
fi
command -v gz >/dev/null 2>&1 || {
  echo "gz CLI not found. Install Gazebo: sudo apt install ros-lyrical-ros-gz" >&2
  exit 1
}

# Render through WSLg -> Mesa D3D12 -> the NVIDIA adapter (never llvmpipe).
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export QT_XCB_GL_INTEGRATION="${QT_XCB_GL_INTEGRATION:-xcb_glx}"
export QT_OPENGL="${QT_OPENGL:-desktop}"

export GZ_SIM_RESOURCE_PATH="$MODEL_ROOT:$MODEL_ROOT/rmuc_2027/meshes${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

if command -v glxinfo >/dev/null 2>&1; then
  renderer="$(glxinfo -B 2>/dev/null | grep -m1 'OpenGL renderer' || true)"
  echo "Renderer: ${renderer#*: }"
  if ! glxinfo -B 2>/dev/null | grep -q 'Accelerated: yes'; then
    echo "WARNING: GPU acceleration not detected; Gazebo may use software rendering." >&2
  fi
fi

echo "World: $WORLD"
echo "Resource path: $GZ_SIM_RESOURCE_PATH"

exec gz sim -v 4 --gui-config "$GUI_CONFIG" "$WORLD" "$@"
