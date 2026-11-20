#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
build_dir=${1:-"$root/build/flip_recovery_host"}
mkdir -p "$build_dir"
printf '#define CONFIG_PLATFORM_POSIX 1\n' > "$build_dir/px4_boardconfig.h"
g++ -std=c++14 -D__PX4_POSIX -D__PX4_LINUX -Wall -Wextra -Werror -O1 -g \
  -I"$build_dir" \
  -I"$root/src/lib" -I"$root/src/lib/matrix" \
  -I"$root/platforms/common/include" -I"$root/platforms/posix/include" \
  "$root/test/flip_recovery/regression.cpp" -o "$build_dir/regression"
"$build_dir/regression"
