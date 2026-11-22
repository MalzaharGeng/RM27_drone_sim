#!/usr/bin/env bash
set -eo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="${HARRIER_SIM_REPO:-$(realpath "$here/../../..")}"
if [[ -z "${HARRIER_SIM_REPO:-}" && ! -r "$root/rm27_sim/models/x500_mono_cam/model.sdf" ]]; then
    root="$HOME/drone_sim"
fi
px4="${HARRIER_FLIP_PX4:-$HOME/harrier_px4_sitl}"
profile="${HARRIER_FLIP_PROFILE:-$here/profile.json}"
out="${HARRIER_FLIP_OUT:-$(mktemp -d -t harrier-flip.XXXXXX)/run}"
instance="${HARRIER_FLIP_INSTANCE:-81}"
models="$root/rm27_sim/models"
build="$px4/build/px4_sitl_default"
[[ -x "$build/bin/px4" && -f "$build/rootfs/gz_env.sh" ]] || {
    echo "指定 HARRIER_FLIP_PX4 为已构建、带 Harrier SITL 补丁的 PX4 源码树。" >&2
    exit 1
}
[[ -r "$models/x500_mono_cam/model.sdf" && -r "$models/gabel_airframe/meshes/gabel_collision.stl" ]] || {
    echo "不存在 Harrier 模型资源：$models" >&2
    exit 1
}
source /opt/ros/lyrical/setup.bash
source "$build/rootfs/gz_env.sh"
export GZ_SIM_RESOURCE_PATH="$models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
unset GZ_IP
export PX4_HARRIER_SERVO=1
scene=()
if [[ "${1:-}" == "--arena" ]]; then
    shift
    world_dir=$(mktemp -d -t harrier-arena.XXXXXX)
    python3 "$here/arena_world.py" "$root/rm27_sim/worlds/rm27_midhangar_n_world.sdf" "$world_dir/arena.sdf"
    scene=(--world "$world_dir/arena.sdf" --xy 6.80 8.05 --yaw 90 --height 0.504900 --startup-timeout 240)
fi
printf 'Harrier 仿真日志：%s\n' "$out"
exec python3 -u "$here/../sweep/run_case.py" --repo "$px4" --out "$out" --instance "$instance" \
    --airframe gz_x500_mono_cam --entity Drone-Harrier --model "$models/x500_mono_cam/model.sdf" \
    --tilt 105 --axis 90 --height 0.38 --startup-timeout 120 --params "$(<"$profile")" "${scene[@]}" "$@"
