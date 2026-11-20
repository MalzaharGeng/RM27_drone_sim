#!/usr/bin/env bash
# 安装 Pangolin 0.9.6（与 nano 上 /home/nano/orb_dependencies/pangolin-0.9.6 同版本）
# 目的：让 ORB_SLAM3 的 find_package(Pangolin REQUIRED) 能找到 PangolinConfig.cmake
#
#   ./install_pangolin.sh            # 源码编译安装到 /usr/local
#   ./install_pangolin.sh /opt/pg    # 自定义前缀（之后需设 Pangolin_DIR）
set -euo pipefail

VER="${PANGOLIN_VER:-v0.9.6}"
PREFIX="${1:-/usr/local}"
WORK="${HOME}/orb_dependencies"
SRC="${WORK}/Pangolin-${VER}"

mkdir -p "$WORK"
if [[ ! -d "$SRC" ]]; then
  echo "[clone] Pangolin $VER -> $SRC"
  git clone --depth 1 --branch "$VER" https://github.com/stevenlovegrove/Pangolin.git "$SRC"
fi

echo "[deps] 安装编译依赖（需要 sudo）"
sudo apt-get update -qq
sudo apt-get install -y -qq \
  build-essential cmake libgl1-mesa-dev libglew-dev pkg-config \
  libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols \
  libjpeg-dev libpng-dev libavcodec-dev libavutil-dev libswscale-dev \
  libavformat-dev libeigen3-dev

cd "$SRC"
echo "[build] Pangolin $VER -> $PREFIX"
cmake -B build -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="$PREFIX" \
      -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_PANGOLIN_PYTHON=OFF
cmake --build build -j"$(nproc)"
sudo cmake --install build

echo
echo "────────────────────────────────"
if [[ -f "$PREFIX/lib/cmake/Pangolin/PangolinConfig.cmake" ]]; then
  echo "OK  $PREFIX/lib/cmake/Pangolin/PangolinConfig.cmake"
elif [[ -f "$PREFIX/lib64/cmake/Pangolin/PangolinConfig.cmake" ]]; then
  echo "OK  $PREFIX/lib64/cmake/Pangolin/PangolinConfig.cmake"
else
  echo "! 未找到 PangolinConfig.cmake，请手动检查安装前缀" >&2
  exit 1
fi
echo
echo "若安装在非默认前缀，重编 ORB_SLAM3 时加："
echo "  cmake .. -DPangolin_DIR=$PREFIX/lib/cmake/Pangolin"
echo "────────────────────────────────"
