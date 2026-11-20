# PX4 源码与构建

`PX4-Autopilot/` 收录地面翻正分支的完整已跟踪源码快照，共 13210 个普通文件；源码身份为 `8313a4a342374d867518e8b4de97cfdabecfe7c0`，翻正基底为 `60889745af001515a62872420de2d32fb602803b`，上游版本为 v1.17.0。29 个外部依赖按原提交固定在仓库根 `.gitmodules`；不是仅保存补丁或一个仓库链接。

原始子模块 SHA 与来源见 [清单](PX4_SOURCE_MANIFEST.json)。上游自带的文档、许可证、示例数据和测试源码一并保留；本机构建、参数缓存和设备产物不进入源码快照。

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

这些路径均相对于 `PX4-Autopilot/`。完整接入行为见 [翻正接入说明](../PX4-Autopilot/docs/flip-recovery/INTEGRATION-20261120.md)，按日期变化的结果放在 [回正验证记录](verification/flip_recovery_20261120/README.md)。实机 DShot 路径与 Gazebo 输出路径分别维护，仿真不代表电调实测。

## 独立构建树

PX4 原生 Makefile 要求其源码根存在独立 `.git`，不能直接在嵌入 drone-sim 的快照目录运行 `make`。导出工具从本仓库的已跟踪快照生成独立构建源码，并重建原子模块索引；不会覆盖现有目录。

```bash
cd ~/drone_sim
python3 tools/px4/create_build_tree.py \
  --output "$HOME/rm27_px4_source" --submodules
cd ~/rm27_px4_source
bash Tools/setup/ubuntu.sh
source /opt/ros/lyrical/setup.bash
make px4_sitl_default

# 规划器与换向回归，不启动仿真。
bash test/flip_recovery/run_host_tests.sh
```

导出树的 Git commit 是本机生成的快照版本；原始源码 SHA 记录在 `SOURCE_REVISION.json`，不得将二者混为同一历史提交。依赖按清单中的 SHA 初始化，首次操作需要下载。板级构建另需相应 ARM 工具链。

本次测试复用了具有相同核心源码的既有 WSL 构建；新导出树只完成源码和索引核验，未另做一次全量板级编译。原场地 VIO 使用的 PX4 参数与本翻正模式试验不同，不能直接复用参数快照。
