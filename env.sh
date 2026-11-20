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
export GZ_SIM_RESOURCE_PATH="$HOME/drone_sim/rm27_sim/models:$HOME/drone_sim/rm27_sim/models/rmuc_2027/meshes${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

# PX4 SITL selects the RM27 world and a start pose inside the arena.
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-rm27_world}"
# [NP01_CENTER_START] 场馆中心 (14.149, 8.065)：CAD 支撑面 z=0.4686563；出生高度包含机身下半部与 2 mm 初始间隙。
export PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:-14.149,8.065,0.504806301,0,0,0}"

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
