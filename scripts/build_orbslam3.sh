#!/usr/bin/env bash
# 编译 ORB_SLAM3（WSL x86_64）。只编库 + RGB-D/单目示例，跳过 ROS 示例（没有 ROS 1）。
#
#   ./build_orbslam3.sh              # 增量编译
#   ./build_orbslam3.sh clean        # 先清空 build 再编
set -euo pipefail

SRC="${ORB_SLAM3_SRC:-$HOME/ORB_SLAM3}"
LOG="${HOME}/orbslam3_build_$(date +%Y%m%d).log"

# 并发数：ORB_SLAM3 每个 cc1plus 峰值 1.5-2 GB（Eigen/g2o 模板展开），
# 直接 -j$(nproc) 会在这台机器（32 线程 / 16 GB）上被 OOM 杀掉。
if [[ -z "${JOBS:-}" ]]; then
  cores=$(nproc)
  mem_gb=$(awk '/MemTotal/{printf "%d", $2/1024/1024}' /proc/meminfo)
  by_mem=$(( mem_gb / 2 ))
  JOBS=$(( by_mem < cores ? by_mem : cores ))
  if (( JOBS < 2 )); then JOBS=2; fi
  if (( JOBS > 8 )); then JOBS=8; fi
fi

cd "$SRC"

if [[ "${1:-}" == "clean" ]]; then
  echo "[clean] 删除 build/ 与 Examples 二进制"
  rm -rf build
fi

mkdir -p build && cd build
echo "[cmake] $SRC/build   (jobs=$JOBS, log=$LOG)"

{
  # Pangolin 0.9.x 的 config 不导出 Pangolin_INCLUDE_DIRS，
  # 而 ORB_SLAM3 的 CMakeLists 用它取头文件路径，这里显式补上。
  PANGOLIN_PREFIX="${PANGOLIN_PREFIX:-/usr/local}"
  cmake .. -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_C_FLAGS="-std=c11" -DCMAKE_CXX_FLAGS="-std=c++17" \
      -DPangolin_DIR="$PANGOLIN_PREFIX/lib/cmake/Pangolin" \
      -DPangolin_INCLUDE_DIRS="$PANGOLIN_PREFIX/include"
  # 只编库 + 其依赖（Thirdparty 会随 ORB_SLAM3 自动构建）。
  # 不编全部目标：本 fork 里还有个 orbslam3_px4（src/Px4Bridge.cc），
  # 它要求特定版本的 px4_msgs 定义（static_assert），版本不匹配就会让
  # 整个 make 返回 Error 2——但我们的 PX4 注入走 slam_pose_to_px4.py
  # （MAVLink），完全用不到那个目标。
  # 需要连示例一起编时：BUILD_ALL=1 JOBS=6 ./build_orbslam3.sh
  if [[ "${BUILD_ALL:-0}" == "1" ]]; then
    make -j"$JOBS"
  else
    make -j"$JOBS" ORB_SLAM3
  fi
} 2>&1 | tee "$LOG"

echo
echo "────────────────────────────────"
echo "产物检查（ROS 节点链接的是 build/lib 下的这几个库）："
for lib in \
  "$SRC/build/lib/libORB_SLAM3.so" \
  "$SRC/build/lib/libDBoW2.so" \
  "$SRC/build/Thirdparty/g2o/lib/libg2o.so"
do
  if [[ -f "$lib" ]]; then
    printf "  OK  %-46s %s\n" "${lib#$SRC/}" "$(du -h "$lib" | cut -f1)"
  else
    echo "  !! 缺失 ${lib#$SRC/}" >&2
  fi
done
for ex in build/Examples/RGB-D/rgbd_tum build/Examples/Monocular/mono_euroc build/Examples/Stereo/stereo_euroc; do
  [[ -x "$SRC/$ex" ]] && echo "  示例 OK  $ex" || true
done
echo "日志：$LOG"
echo "────────────────────────────────"
echo "下一步："
echo "  1) 相机桥与节点：~/drone_sim/scripts/run_stereo_slam.sh"
echo "  2) 链路排障：~/drone_sim/scripts/diag_slam_link.sh"
