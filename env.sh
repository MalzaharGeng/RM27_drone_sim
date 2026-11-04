#!/usr/bin/env bash
# Source this before launching PX4/Gazebo:  source ~/drone_sim/env.sh

# Keep Windows/MSYS paths out of the toolchain search path.
export PATH="/opt/ros/lyrical/bin:$HOME/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/lib/wsl/lib"

set +u
if [[ -f /opt/ros/lyrical/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/lyrical/setup.bash
fi
set -u

# Render through WSLg -> Mesa D3D12 -> NVIDIA adapter (never llvmpipe).
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export QT_XCB_GL_INTEGRATION="${QT_XCB_GL_INTEGRATION:-xcb_glx}"
export QT_OPENGL="${QT_OPENGL:-desktop}"

# RM27 arena resources (model://rmuc_2027 lives under models/).
export GZ_SIM_RESOURCE_PATH="$HOME/drone_sim/rm27_sim/models:$HOME/drone_sim/rm27_sim/models/rmuc_2027/meshes${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

# PX4 SITL selects the RM27 world and a start pose inside the arena.
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-rm27_world}"
export PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:-5,6,0.5,0,0,0}"
