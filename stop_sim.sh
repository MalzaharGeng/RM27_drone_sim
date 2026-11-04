#!/usr/bin/env bash
# Stop PX4 SITL and Gazebo processes started by start_rm27_px4.sh.
set +e

pkill -9 -f "gz/sim10/gz-sim-gui-client"
pkill -9 -f "gz/sim10/gz-sim-main"
pkill -9 -x px4
pkill -9 -f "make px4_sitl"

sleep 1
leftover="$(ps -eo pid,comm | awk '$2 == "px4" || $2 ~ /^gz-sim/')"
if [[ -n "$leftover" ]]; then
  echo "Still running:" >&2
  echo "$leftover" >&2
  exit 1
fi
echo "Simulation stopped."
