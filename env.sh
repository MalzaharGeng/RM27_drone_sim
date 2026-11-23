#!/usr/bin/env bash
# Source this before launching PX4/Gazebo:  source ~/drone_sim/env.sh

# Keep Windows/MSYS paths out of the toolchain search path.
export PATH="/opt/ros/lyrical/bin:/home/gengyinghao/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/lib/wsl/lib"

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
_rm27_env_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GZ_SIM_RESOURCE_PATH="$_rm27_env_root/rm27_sim/models:$_rm27_env_root/rm27_sim/models/rmuc_2027/meshes${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
unset _rm27_env_root

# PX4 SITL selects the RM27 world and a start pose inside the arena.
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-rm27_world}"
# Default pose for PX4's self-spawn path. When attaching to Drone-Harrier, the
# world SDF already places the model; keep the rm27_world fallback in sync.
case "$PX4_GZ_WORLD" in
  rm27_world) _rm27_spawn_pose=13.116631,7.032968,1.9558063,0,0,0 ;;
  *)          _rm27_spawn_pose=14.149,8.065,0.504806301,0,0,0 ;;
esac
export PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:-$_rm27_spawn_pose}"
unset _rm27_spawn_pose

# [Drone-Harrier] Attach PX4 to the pre-spawned model named Drone-Harrier in the world.
# Airframe/make target stays x500_mono_cam (no rebuild); only the Gazebo entity is renamed.
export PX4_GZ_MODEL_NAME="${PX4_GZ_MODEL_NAME:-Drone-Harrier}"

# ROS 2 setup hooks can restore Windows PATH entries inherited from the Windows
# caller.  PX4/CMake must only see the Linux toolchain, otherwise it discovers
# MSYS headers/libs (/mnt/c/msys64/ucrt64) and the build fails with
# "Only Win32 target is supported!".  Filter those entries after setup.bash.
IFS=: read -ra _drone_sim_path_parts <<< "$PATH"
_drone_sim_clean_path=""
for _p in "${_drone_sim_path_parts[@]}"; do
  case "$_p" in
    /mnt/c/*|/mnt/d/*) continue ;;
  esac
  [[ -n "$_p" ]] || continue
  if [[ -z "$_drone_sim_clean_path" ]]; then
    _drone_sim_clean_path="$_p"
  else
    _drone_sim_clean_path="$_drone_sim_clean_path:$_p"
  fi
done
export PATH="$_drone_sim_clean_path"
unset _drone_sim_path_parts _p _drone_sim_clean_path
