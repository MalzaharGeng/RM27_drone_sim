> **2026-11-20 本地接入更新**：自动换向、油门换算、NED 选边、进度与停油后稳定判定已经更新。以下旧章节中“换向待接入”等描述属于历史状态；当前行为、已知限制和验证边界以 [接入说明](docs/flip-recovery/INTEGRATION-20261120.md) 为准。本地更新尚未推送 GitHub。

# `feat/flip-recovery` — 反乌龟 / 侧翻自动纠正

> 落地侧翻或倒扣后，用一个**原生飞行模式**接管电机，按当前姿态自动选出高位/低位电机，
> 把机身翻回正立，然后自动切回常规模式。

> **状态**：模式接入、姿态算法、参数体系、状态观测、未解锁出力全部完成。
> **电调换向能力已在实机确认** —— 4/4 电机可反向并立即恢复，见 [测试报告](docs/flip-recovery/TEST-REPORT.md)。
> **驱动层的换向时序尚未接入**，现在刷进去桨能转，但方向只有出厂方向（见第 6 节）。

分支关系：`v1.17.0` → `fix/bdshot-micoair743v2-aio`（BDShot 修复）→ **本分支**
（已包含 bdshot 修复，可独立编译刷机）

| 项目 | 值 |
|---|---|
| 基线 | PX4 v1.17.0 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| 目标硬件 | MicoAir743v2-AIO-35A（target `micoair_h743-v2`，board_id 1179） |
| 新增飞行模式 | `NAVIGATION_STATE_FLIP_RECOVERY = 16`（复用 FREE1 槽位） |
| 新增参数 | 14 个 `FLIP_*` |
| 详细文档 | [`docs/flip-recovery/README.md`](docs/flip-recovery/README.md) |

---

# 使用说明（必读）

## 1. 编译与刷写

```bash
make micoair_h743-v2_default
```

只刷 app，**不要动 bootloader**。QGC 里选 `.px4` 文件即可。

## 2. 参数配置

### 必须先设的两项

```
param set FLIP_EN 1            # 总开关，0 = 拨进模式也不转桨
param set COM_FLTMODE5 17      # 把第 5 档模式开关映射成 Flip Recovery
param save
```

> `COM_FLTMODE1..6` 是模式槽位，值 17 = Flip Recovery。
> 不用拨杆的话这一项可以跳过，直接发指令触发。

### 常用可调参数

| 参数 | 默认 | 范围 | 作用 |
|---|---|---|---|
| **`FLIP_EN`** | 0 | bool | **总开关**。0 时模式可进、状态可见，但不转桨 |
| **`FLIP_THR_HIGH`** | 0.20 | 0.02–0.6 | **高位组油门**。唯一真正决定翻正快慢的参数，从 0.15 起步 |
| **`FLIP_THR_LOW`** | 0.10 | 0–0.6 | **低位组油门**。只负责卸压/减摩擦，正常应明显低于高位 |
| **`FLIP_LOW_MODE`** | 0 | 0–3 | 低位转向调试：`0`=Auto（永远抬）、`1`=强制正转、`2`=强制反转、`3`=关闭 |
| **`FLIP_MODE`** | 0 | 0–1 | 策略：`0`=翻过去（90–180° 抬高位翻过 180°）、`1`=压回去（全程压高位，路径最短） |
| `FLIP_ENTRY_ANG` | 75 | 60–180 | 超过这个倾角才出力 |
| `FLIP_STOP_ANG` | 60 | 0–120 | 低于这个倾角收油，交给重力 |
| `FLIP_EXIT_MODE` | 2 | 0–8 | 翻完自动切回的模式（2 = Position） |
| `FLIP_RC_CONFIRM` | 0 | 0–6 | 0 = 免二次确认；1–6 = 还要对应 aux 通道在高位 |
| `FLIP_SLEW` | 0.50 | 0.05–2.0 | 油门上升率（1/s） |
| `FLIP_TIME` | 3.0 | 0.2–5.0 | 总出力时长上限（s） |
| `FLIP_MIN_PROG` | 3.0 | 0–30 | 进度检查阈值（度） |
| `FLIP_PROG_T` | 0.80 | 0.2–2.0 | 多久后开始查进度（s） |
| `FLIP_OUT_DIRECT` | 0 | bool | 1 = 台架直发 `actuator_motors` |

## 3. 触发方式

四种方式，最终都落到 `vehicle_status.nav_state = 16`。按场景选一种即可。

### A. QGC 参数页（不用写代码）

1. QGC 左侧 **齿轮 → Parameters**，搜索 `COM_FLTMODE`
2. 把 `COM_FLTMODE5` 设成 **Flip Recovery**，点 Save
3. 有遥控就拨到第 5 档；没遥控就走 **MAVLink Console** 输入 `commander mode flip`

> 下拉里能出现 "Flip Recovery"，是因为固件在 `commander/module.yaml` 里把这个枚举描述成
> `17: Flip Recovery`。
>
> **QGC 的 "Flight Modes" 页面里没有这个模式** —— 那个页面的列表是 QGC 硬编码的，
> 必须走 **Parameters** 页面。

### B. 遥控器拨杆

先在 QGC → **Vehicle Setup → Radio** 完成遥控器和开关校准，然后：

| 开关位置 | 对应参数 |
|---|---|
| 第 1 档 | `COM_FLTMODE1` |
| 第 2 档 | `COM_FLTMODE2` |
| … | … |
| 第 6 档 | `COM_FLTMODE6` |

把想用的那一档设成 **Flip Recovery**，拨过去即可；拨回任意其它档立即收油。

可选二次确认（防误碰）：`FLIP_RC_CONFIRM = 6` 要求 aux6 通道同时在高位。

### C. ROS2 话题（和发降落一样简单）

```python
from px4_msgs.msg import VehicleCommand

cmd = VehicleCommand()
cmd.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
cmd.command          = 10071     # VEHICLE_CMD_SET_NAV_STATE
cmd.param1           = 16.0      # NAVIGATION_STATE_FLIP_RECOVERY
cmd.target_system    = 1
cmd.target_component = 1
cmd.from_external    = True
self.cmd_pub.publish(cmd)
```

对比发降落只差一行：那边是 `cmd.command = VehicleCommand.VEHICLE_CMD_NAV_LAND`。

> `10071` 直接给 `nav_state` 数值，不用换算 `custom_mode` 位域。

### D. 板载命令行

```
commander mode flip
```

## 4. 执行结果回传

翻正结束（成功或失败）后模块持续发布 `flip_recovery_status`，上位机订阅它就能发下一步，
不用轮询 `nav_state`、不用读日志：

```python
from px4_msgs.msg import FlipRecoveryStatus

def status_cb(self, msg):
    if msg.result == FlipRecoveryStatus.RESULT_SUCCESS:
        # 用时 msg.duration_s，结束倾角 msg.tilt_deg
        # 下一步：解锁 / 起飞 / 切回 Position
        pass
    elif msg.result == FlipRecoveryStatus.RESULT_FAILED:
        # msg.reason: 1=TIMEOUT 2=NO_PROGRESS 3=SAFETY_LOST 4=MODE_EXITED 5=ATTITUDE_RESET
        pass
```

`result` 会一直保持到最后一次结果，直到下次开始出力才回到 `RESULT_NONE`，
所以上位机要按**跳变**去重（记住上一次的值），或者用 `session_id` 区分不同的尝试。

## 5. 观测与退出

```
listener flip_recovery_control      # 倾角、zone、掩码、逐路油门
listener flip_recovery_status       # 执行结果、时长、失败原因
flip_recovery status                # 全部内部状态
```

退出：拨杆拨回 / 在 QGC 切到其它模式 / 翻完自动切回 `FLIP_EXIT_MODE` / `FLIP_EN=0`。

## 6. DShot 出力层

拆成两层：**能不能输出**（软件门控，已完成）、**输出什么方向**（电调能力已实测，换向时序待做）。

### 6.1 已完成：未解锁也允许出力

`MixingOutput` 加了一条与 `actuator_test` 平行的通路，完全绕开 `_functions[]` 的 armed 门控：

```cpp
void MixingOutput::setRecoveryOutputs(bool active, const float outputs[MAX_ACTUATORS]);
```

三个安全条件缺一不可：调用方要求接管 + 确实未解锁 + 没有 kill/lockdown/termination。
默认关闭，对其它机型零影响。

`DShot::handle_flip_recovery()` 订阅 `flip_recovery_control`，把 `control[]` 的**绝对值**喂进去，
并带 200 ms 话题超时兜底（模块挂了就自动放手）。

### 6.2 已确认：电调支持反向（2026-11-19 实机测试）

拆桨、上电、30% 油门短脉冲，读无桨机身的反作用角速度 `zgyro` 判定转向：

| 电机 | 正常方向 | `dshot reverse` 后 | 结论 | 正常转向（俯视） |
|---|---|---|---|---|
| 1 | **+233** | −267 | 可反向 | 逆时针 |
| 2 | **+271** | −328 | 可反向 | 逆时针 |
| 3 | **−349** | +275 | 可反向 | 顺时针 |
| 4 | **−290** | +202 | 可反向 | 顺时针 |

单位 mrad/s，符号即转向。**4/4 通过**，且发命令 20 后立刻恢复。

两个附带结论：

- **电机编号与 PX4 标准 Quad-X 一致**（1/2 同向、3/4 同向且相反），`CA_ROTORn_PX/PY` 的几何映射可信
- **未解锁出力通道在官方固件里本来就存在**：`actuator_test` 给 10% 油门时 `actuator_outputs` 读到 298，
  而 `armed=False`，说明 `setRecoveryOutputs()` 照这条路实现是对的

完整的测量方法、原始波形、两种失败的方法见 [测试报告](docs/flip-recovery/TEST-REPORT.md)。

### 6.3 仍待做：驱动层换向时序

现在 `control[i]` 的**符号被丢弃**了，所有电机都按出厂方向转。要完整翻正还需要：

1. **换向时序** —— DShot 方向命令必须在电机**停转**时发，所以要「停机 → 发命令 → 等电调 → 再给油门」
2. **命令发送** —— 复用 `DShot_cmd_spin_direction_normal(20)` / `reversed(21)`，
   但 `_current_command` 一次只能一个 motor_mask，高/低位两组要分两轮发

详见 [`docs/flip-recovery/README.md` 第 8 节](docs/flip-recovery/README.md)。

### 6.4 你自己怎么复测

**测电调支不支持方向命令**（不需要任何新代码，PX4 原生就有）：

```
dshot reverse -m 1      # 对 Motor1 发命令 21，重复 10 帧
dshot normal  -m 1      # 恢复
dshot normal            # 不带 -m 则四路一起恢复
```

拆桨，执行后对比电机转向。本板 v2-AIO-35A 是 **Bluejay 0.19.2**，已确认支持；
45A 版是 AM32，另外还支持 3D 油门（负油门反转），本板不支持。

**测未解锁出力**（需要本分支固件）：

```
param set FLIP_EN 1
commander mode flip
```

看电机是否在未解锁状态下转起来。这一步**只验证能不能转**，方向还不对。

---

# 这个分支改了什么

## 新增

| 文件 | 内容 |
|---|---|
| `msg/FlipRecoveryControl.msg` | 控制请求：`state` / `action` / `zone` / 掩码 / `float32[4] control` / `tilt_deg` |
| `msg/FlipRecoveryStatus.msg` | 执行结果回传：`result` / `reason` / `duration_s` / `mode_active`，给上位机判断何时发下一步 |
| `src/lib/flip_recovery/FlipPlanner.hpp` | 纯逻辑：倾角、选边、方向映射、状态机、逐路油门 |
| `src/modules/flip_recovery/` | 模块骨架 + 14 个参数 + 构建配置 |
| `docs/flip-recovery/` | 完整文档 |

## 修改

| 文件 | 改了什么 | 为什么 |
|---|---|---|
| `msg/versioned/VehicleStatus.msg` | `FREE1=16` → `FLIP_RECOVERY=16` | 复用空闲槽位，`MAX` 保持 31 |
| `commander/px4_custom_mode.h` | 加 `MAIN_MODE_FLIP_RECOVERY`（=12） | MAVLink `custom_mode` 字节 |
| `lib/modes/ui.hpp` | 有效模式位 + 显示名 | 不加位切换被判非法 |
| `ModeUtil/control_mode.cpp` | 该模式所有 flag 保持 false | 让 `control_allocator` 停止发布 `actuator_motors` |
| `ModeUtil/mode_requirements.cpp` | 加 `prevent_arming` 等 | 禁止带着翻正模式解锁 |
| `Commander.cpp` | `DO_SET_MODE` 分支 + CLI | 所有触发路径的汇聚点 |
| `commander/module.yaml` | `17: Flip Recovery` | QGC 下拉选项 |
| `manual_control/ManualControl.cpp` | `navStateFromParam` case 17 | 拨杆链路 |
| `boards/.../default.px4board` | `CONFIG_MODULES_FLIP_RECOVERY=y` | 编译进固件 |
| `boards/.../rc.board_extras` | 无条件启动模块 | 模式化后必须常驻 |

## 核心原理（三段话）

**选边**：倾斜时贴地的那两路电机，其连线就是翻正转轴。它们力臂≈0，只负责"抬一点"卸掉地面压力；
真正干活的是高位两路，力臂最大。

**方向**：`Normal`（出厂转向）产生的机身受力方向由 `R(2,2)=cos(tilt)` 决定——
正立时朝上，倒扣后朝下。所以只在 90° 处换算一次，其余全部用 `Push::Up/Down` 表达。

**策略**：`FLIP_MODE=0`（默认）在 60–90° 压高位回正、90–180° 抬高位翻过 180°，高位全程 Reversed；
`FLIP_MODE=1` 全程压高位，路径最短但 90° 要换向。

---

# 文档

| 文件 | 内容 |
|---|---|
| [`docs/flip-recovery/README.md`](docs/flip-recovery/README.md) | 完整实现说明：架构、数据流、算法推导、逐文件改动、调试流程 |
| [`docs/flip-recovery/TEST-REPORT.md`](docs/flip-recovery/TEST-REPORT.md) | 电调换向能力实测报告：测量方法、四路数据、未验证项 |
| [`docs/flip-recovery/IMPLEMENTATION.md`](docs/flip-recovery/IMPLEMENTATION.md) | 演进记录（每次设计变更与纠错） |

---

# 刷写注意

两个分支都可以只刷 app，bootloader 不动。QGC 刷 `.px4` 文件即可，
MAVLink 刷写不会覆盖 bootloader，也不会丢电调校准。
