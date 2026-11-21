# PX4 源码与构建

`PX4-Autopilot/` 收录基于 [PX4-Autopilot v1.17.0](https://github.com/PX4/PX4-Autopilot) 的完整源码快照。地面翻正代码源于 [natgeng/PX4-MicoAir743v2-AIO 的 `feat/flip-recovery` 分支](https://github.com/natgeng/PX4-MicoAir743v2-AIO/tree/feat/flip-recovery)，本仓库补入的功能列于 [翻正实现与改动](PX4_FLIP_RECOVERY.md)。

快照含 13,210 个普通文件和 29 个按版本固定的子模块。源码身份、子模块来源和版本见 [源码清单](PX4_SOURCE_MANIFEST.json)。快照保留上游文档、许可证、示例及测试；本机构建产物和设备参数不属于快照。

## 翻正相关目录

| 目录 / 文件 | 职责 |
|---|---|
| `src/modules/flip_recovery/` | 模式判断、安全条件、参数与状态发布 |
| `src/lib/flip_recovery/FlipPlanner.hpp` | 电机高低分组、翻转策略、有向进度与停油后稳定判定 |
| `src/lib/flip_recovery/DirectionSequencer.hpp` | 停机、方向命令重复、等待、实际油门爬升与方向恢复 |
| `src/drivers/dshot/` | 真实电调输出接入及 Motor1..4 映射 |
| `src/lib/mixer_module/` | 恢复输出归一化与安全仲裁 |
| `src/modules/commander/` | Flip Recovery 原生模式、切入切出与禁止解锁 |
| `msg/FlipRecovery*.msg` | 控制请求和执行结果 |
| `test/flip_recovery/` | 真实 C++ 规划器、时序和混控回归 |

表中路径均相对于 `PX4-Autopilot/`。DShot 实机路径与 Gazebo 试验桥分开维护；SITL 参数、试验方法和结论见 [地面翻正验证](verification/flip_recovery_20261121/README.md)。

## 独立构建树

PX4 原生 Makefile 要求源码根具有独立 Git 元数据。导出工具从本仓库的快照生成独立源码树并重建子模块索引，目标目录必须是新目录。在本仓库根目录执行：

```bash
python3 tools/px4/create_build_tree.py \
  --output "$HOME/rm27_px4_source" --submodules
cd ~/rm27_px4_source
bash Tools/setup/ubuntu.sh
source /opt/ros/lyrical/setup.bash
make px4_sitl_default

# 规划器与换向回归，不启动仿真。
bash test/flip_recovery/run_host_tests.sh
```

导出树的源码身份以 `SOURCE_REVISION.json` 为准。子模块按清单固定的版本初始化，首次操作需要下载。板级构建另需 ARM 工具链。

翻正动力学验证复用既有 WSL 构建，在独立 SITL 树应用 Gazebo 试验补丁。Harrier 原场地 VIO 使用独立的 PX4 参数配置。
