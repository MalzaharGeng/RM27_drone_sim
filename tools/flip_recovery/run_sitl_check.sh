#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/lyrical/setup.bash
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
P=${PX4_SOURCE:?set PX4_SOURCE to an independently built PX4 source tree}
source "$P/build/px4_sitl_default/rootfs/gz_env.sh"
python3 "$HERE/sitl_check.py" --repo "$P" --out "${1:?new output directory}" \
  --instance "${PX4_INSTANCE:-7}" --torque 6.0 --pulse 0.45
