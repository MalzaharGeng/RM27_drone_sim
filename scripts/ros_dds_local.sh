#!/usr/bin/env bash
# 已废弃的 DDS 兼容层。
#
# 历史问题：WSL 的 networkingMode=mirrored 会把 Windows 网卡（含 Tailscale
# 100.x）映射进 WSL，并在 lo 上添加 10.255.255.254，导致 ROS 2 DDS 发现
# 完全失效（两进程互不可见，Fast DDS / CycloneDDS 都一样）。
#
# 根治方法：~/.wslconfig 改为 networkingMode=NAT 后重启 WSL，默认配置即通。
# 因此这里不再设置任何自定义 DDS 参数——仿真与实机保持一致的默认行为。
#
# 本文件保留为占位：各脚本仍会 source 它，这里负责把历史遗留的
# 自定义 DDS 变量清掉，避免某个终端里残留旧配置造成行为不一致。

unset FASTDDS_DEFAULT_PROFILES_FILE  2>/dev/null || true
unset FASTRTPS_DEFAULT_PROFILES_FILE 2>/dev/null || true
unset CYCLONEDDS_URI                 2>/dev/null || true
unset ROS_LOCALHOST_ONLY             2>/dev/null || true
unset ROS_AUTOMATIC_DISCOVERY_RANGE  2>/dev/null || true
