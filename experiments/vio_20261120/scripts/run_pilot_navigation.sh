#!/usr/bin/env bash
# Device startup: no environment from an interactive SSH session is required.
set -eo pipefail
pilot_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
pilot_config=${PILOT_CONFIG:-$HOME/.config/pilot-navigation.json}
pilot_ros_setup=${PILOT_ROS_SETUP:-/opt/ros/humble/setup.bash}
pilot_messages_setup=${PILOT_MSGS_SETUP:-$HOME/.cache/orbslam3-px4/ros2_ws/install/local_setup.bash}
[[ -r "$pilot_config" && -r "$pilot_ros_setup" && -r "$pilot_messages_setup" ]] || {
    echo 'Missing pilot config, ROS setup or px4_msgs setup.' >&2
    exit 2
}
source "$pilot_ros_setup"
source "$pilot_messages_setup"
set -u
pilot_state=${PILOT_LOG_ROOT:-$HOME/.local/state/pilot-navigation}
mkdir -p "$pilot_state"
export PILOT_LOG_ROOT="$pilot_state"
exec flock --nonblock "$pilot_state/instance.lock" /usr/bin/python3 "$pilot_dir/pilot_navigation_boot.py" --config "$pilot_config" "$@"
