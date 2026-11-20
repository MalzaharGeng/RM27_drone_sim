> **2026-11-20 本地接入更新**：自动换向、油门换算、NED 选边、进度与停油后稳定判定已经更新。以下旧章节中“换向待接入”等描述属于历史状态；当前行为、已知限制和验证边界以 [接入说明](INTEGRATION-20261120.md) 为准。本地更新尚未推送 GitHub。

# 倾角翻正 flip_recovery —— 实现说明 v2（模式化版本）

> 基线：PX4 v1.17.0（`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`）
> 目标板：MicoAir743v2-AIO-35A（`micoair_h743-v2`）
> 本版本相对 v1 的唯一结构性变化：**触发方式从「AUX 开关电平」改为「飞行模式」**，
> 拨杆、上位机话题、MAVLink、命令行四条路径统一落到同一个 `nav_state`。

---

## 0. 一句话说清楚它现在怎么工作

```
遥控拨杆 ──┐
MAVLink ───┤
vehicle_cmd┼──► Commander ──► vehicle_status.nav_state = FLIP_RECOVERY(16)
CLI ───────┘                          │
                                      ▼
                          flip_recovery 模块被唤醒
                                      │
              IMU → EKF2 → 四元数 → 倾角 → 选低位桨 → 正/反转掩码
                                      │
                                      ▼
                        flip_recovery_control（+ 可选 actuator_motors）
```

模式本身不产生推力；它只是「允许翻正」的许可。是否出力还叠加 `FLIP_EN`、未解锁、落地、
姿态有效、可选遥控二次确认这几道闸门。

---

## 1. 改动清单（每一处 + 为什么）

### 1.1 模式槽位：`msg/versioned/VehicleStatus.msg`

```diff
-uint8 NAVIGATION_STATE_FREE1 = 16
+uint8 NAVIGATION_STATE_FLIP_RECOVERY = 16      # Tilt flip recovery (reuses the former FREE1 slot)
```

**为什么用 16 而不是新增 31：**

| 方案 | 影响 |
|---|---|
| 新增 `= 31` 并把 `MAX` 改成 32 | 需要动 `static_assert(NAVIGATION_STATE_MAX == 31)`，至少 4 个文件；`nav_state_names[]` 数组长度变化；ROS2 `px4_msgs` 需要同步；`uint32` 的 `valid_nav_states_mask` 刚好能放 32 位，再多就溢出 |
| **复用 FREE1 = 16**（本方案） | 数值不变、`MAX` 不变、位掩码不变、外部工具零改动；`FREE1` 全仓库只在消息定义里出现，没有任何代码引用 |

PX4 官方把 7/9/11/16 留成 `FREE*`，本来就是给这类内部扩展用的。

### 1.2 自定义模式号：`src/modules/commander/px4_custom_mode.h`

```diff
 	PX4_CUSTOM_MAIN_MODE_TERMINATION,
 	PX4_CUSTOM_MAIN_MODE_ALTITUDE_CRUISE,
+	PX4_CUSTOM_MAIN_MODE_FLIP_RECOVERY /* 12: tilt flip recovery */
 };
```

```diff
 	case vehicle_status_s::NAVIGATION_STATE_STAB:
 		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
 		break;
 
+	case vehicle_status_s::NAVIGATION_STATE_FLIP_RECOVERY:
+		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_FLIP_RECOVERY;
+		break;
```

**为什么：** `PX4_CUSTOM_MAIN_MODE_*` 是 MAVLink 的 `custom_mode.main_mode` 字节，
上位机 QGC / MAVSDK / pymavlink 发 `SET_MODE` 时填的就是它。
现有枚举到 `ALTITUDE_CRUISE = 11`，所以新值自然是 **12**。
第二个改动是反向映射（`nav_state → custom_mode`），MAVLink 上报心跳时用它，
不做的话地面站只会看到 `UNKNOWN`。

### 1.3 模式合法性：`src/lib/modes/ui.hpp`

```diff
 	       (1u << vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF) |
+	       (1u << vehicle_status_s::NAVIGATION_STATE_FLIP_RECOVERY);
```

```diff
 	"Stabilized",
-	"16: UNUSED2",
+	"Flip Recovery",
 	"Takeoff",
```

**为什么：** `getValidNavStates()` 生成的位掩码会进 `vehicle_status.valid_nav_states_mask`，
`ModeManagement::getModeStatus()` 用它决定「这个模式能不能被选中」。
不加这一位，拨杆切过去会被直接判为非法模式并拒绝。
`nav_state_names[]` 是日志 / OSD / MAVLink 的显示名，不改就是 `16: UNUSED2`。

### 1.4 控制模式：`src/modules/commander/ModeUtil/control_mode.cpp`

```diff
 	case vehicle_status_s::NAVIGATION_STATE_STAB:
 		vehicle_control_mode.flag_control_manual_enabled = true;
 		...
 		vehicle_control_mode.flag_control_allocation_enabled = true;
 		break;
 
+	case vehicle_status_s::NAVIGATION_STATE_FLIP_RECOVERY:
+		// 翻正模式不使用任何控制器：所有 flag 保持 false。
+		// 电机输出由 flip_recovery 模块独占（control_allocator 在该模式下让位），
+		// 因此这里既不置 attitude/rate，也不置 allocation。
+		break;
```

**这是整个模式化里最关键的一处。** 原因链：

1. `Commander::updateControlMode()` 每周期先 `_vehicle_control_mode = {}` 再调这个函数；
2. `ControlAllocator.cpp:372` 有 `_publish_controls = vehicle_control_mode.flag_control_allocation_enabled;`
3. `ControlAllocator::publish_actuator_controls()` 开头 `if (!_publish_controls) return;`

所以把 `flag_control_allocation_enabled` 留成 `false`，就等于让 control_allocator
**主动闭嘴**，不再发布 `actuator_motors`。
这样 flip_recovery 发布 `actuator_motors` 时才不会和分配器抢同一个 topic
（uORB 是「后写覆盖」，两个模块同时发会打架）。

同时不置 `attitude/rates/position` 等 flag，保证 `flight_mode_manager`、
`mc_att_control`、`mc_pos_control` 全部不启动，不会有控制器在地面上试图「稳住姿态」。

### 1.5 模式依赖：`src/modules/commander/ModeUtil/mode_requirements.cpp`

```diff
+	// NAVIGATION_STATE_FLIP_RECOVERY
+	// 需要姿态 + 角速度才能算倾角和电机相对地面高度；需要遥控以便随时拨杆退出。
+	// mode_req_prevent_arming：该模式只允许在地面未解锁时使用，绝不允许带着翻正模式起飞。
+	setRequirement(vehicle_status_s::NAVIGATION_STATE_FLIP_RECOVERY, flags.mode_req_angular_velocity);
+	setRequirement(vehicle_status_s::NAVIGATION_STATE_FLIP_RECOVERY, flags.mode_req_attitude);
+	setRequirement(vehicle_status_s::NAVIGATION_STATE_FLIP_RECOVERY, flags.mode_req_manual_control);
+	setRequirement(vehicle_status_s::NAVIGATION_STATE_FLIP_RECOVERY, flags.mode_req_prevent_arming);
```

**为什么逐条：**

| flag | 作用 | 不加会怎样 |
|---|---|---|
| `mode_req_attitude` | 没姿态估计就不允许运行 | 估计器还没收敛时可能用垃圾四元数算倾角 |
| `mode_req_angular_velocity` | 同上，陀螺必须在线 | 无法判断是不是在剧烈翻滚 |
| `mode_req_manual_control` | 遥控链路必须存在 | 拨进去后如果 RC 掉了，没有任何手段退出（只能断电） |
| `mode_req_prevent_arming` | **本模式下禁止解锁** | 最危险：用户可能带着翻正模式解锁起飞，四个桨按反乌龟逻辑乱转 |

`mode_req_prevent_arming` 的效果由 `HealthAndArmingChecks` 读取
（`modePreventsArming(nav_state)` 查的就是这个位），解锁时会直接报
"mode not suitable for arming"。

> 注意：`mode_req_*` 只在 **armed 之后** 才参与 failsafe 判定，
> disarmed 时 `UserModeIntention::change()` 是无条件放行的，
> 所以「没接遥控、纯上位机」也能切进这个模式。

### 1.6 命令解析：`src/modules/commander/Commander.cpp`

```diff
 				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_STABILIZED) {
 					desired_nav_state = vehicle_status_s::NAVIGATION_STATE_STAB;
 
+				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_FLIP_RECOVERY) {
+					desired_nav_state = vehicle_status_s::NAVIGATION_STATE_FLIP_RECOVERY;
+
 				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD) {
```

**为什么：** 这是 `VEHICLE_CMD_DO_SET_MODE` 的唯一解析点。
MAVLink `SET_MODE`、`MAV_CMD_DO_SET_MODE`、ROS2 `px4_ros2` 的 `setMode`、
以及本模块翻正结束后自动切回模式，全部走这里。不加就一律 `TRANSITION_DENIED`。

```diff
 			} else if (!strcmp(argv[1], "stabilized")) {
 				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_STABILIZED);
 
+			} else if (!strcmp(argv[1], "flip")) {
+				// 倾角翻正模式：与遥控拨杆、MAVLink SET_MODE 走同一条路径
+				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_FLIP_RECOVERY);
```

**为什么：** 台架调试最方便的一条路 —— `commander mode flip` 不用开地面站。
它和拨杆走的是同一条命令链路，不会出现「CLI 能用但拨杆不行」的分裂。

```diff
-	PRINT_MODULE_USAGE_ARG("manual|acro|offboard|stabilized|altctl|...",
+	PRINT_MODULE_USAGE_ARG("manual|acro|offboard|stabilized|flip|altctl|...",
```

只是 usage 文本，方便 `commander` 回车时能看到。

### 1.7 拨杆映射：`src/modules/manual_control/ManualControl.cpp`

```diff
 		case 16: return vehicle_status_s::NAVIGATION_STATE_ALTITUDE_CRUISE;
+		case 17: return vehicle_status_s::NAVIGATION_STATE_FLIP_RECOVERY;
```

**为什么：** PX4 1.16 起把遥控模式切换从 commander 拆到了独立的 `manual_control` 模块。
完整链路是：

```
RC 模式通道 → manual_control_switches.mode_slot (1..6)
            → ManualControl::evaluateModeSlot()
            → navStateFromParam(COM_FLTMODE<n>)
            → action_request{ACTION_SWITCH_MODE, mode=nav_state}
            → Commander::handle_action_request()
            → _user_mode_intention.change()
```

`navStateFromParam()` 的 case 值 = `COM_FLTMODE` 的 enum 值。
所以拨杆要能切进翻正，**必须同时改这里和 `module.yaml`**，漏一个就失效。

### 1.8 参数枚举：`src/modules/commander/module.yaml`

```diff
                 16: Altitude Cruise
+                17: Flip Recovery
```

**为什么：** 这是 QGC 参数页里 `COM_FLTMODE1..6` 的下拉选项。
没有它，用户只能手填数字 17，且 QGC 会把它显示成非法值。
`module.yaml` 是 PX4 1.16+ 的新参数描述系统，编译时生成参数元数据。

### 1.9 板级配置：`boards/micoair/h743-v2/default.px4board`

```diff
 CONFIG_MODULES_FLIGHT_MODE_MANAGER=y
+CONFIG_MODULES_FLIP_RECOVERY=y
```

编译进固件。不加这个 Kconfig 开关，源码在树里但根本不会被编译。

### 1.10 启动脚本：`boards/micoair/h743-v2/init/rc.board_extras`

```diff
-# 倾角翻正：只有显式打开 FLIP_EN 才启动，避免默认占用 CPU
-if param compare FLIP_EN 1
-then
-	flip_recovery start
-fi
+# 倾角翻正：模块常驻（20ms 周期，开销可忽略）。
+# 是否真出力由 FLIP_EN 和 FLIP_RECOVERY 模式共同决定
+flip_recovery start
```

**为什么改：** v1 是「`FLIP_EN=1` 才启动」。模式化之后如果还这样，
用户把参数改成 1 必须**重启飞控**模块才会起来，拨杆切进去毫无反应，体验极差。
现在模块常驻，`FLIP_EN` 退化成「出不出力」的软开关：
- `FLIP_EN=0`：可以拨进模式，能看倾角/掩码（调试友好），但桨不转；
- `FLIP_EN=1`：正常翻正。

20 ms 周期 + 6 个订阅，实测 CPU 占用在 0.1% 量级。

### 1.11 消息定义：`msg/FlipRecoveryControl.msg`（重写）

```diff
 uint8 ACTION_STOP = 0
 uint8 ACTION_DRIVE = 1
 ...
+uint8 STATE_IDLE = 0
+uint8 STATE_RAMP = 1
+uint8 STATE_HOLD = 2
+uint8 STATE_COAST = 3
+uint8 STATE_DONE = 4
+uint8 STATE_FAILED = 5
+
+uint8 state
 uint8 action
 uint8 zone
 uint8 fwd_mask
 uint8 rev_mask
 float32 thr_fwd
 float32 thr_rev
+float32 tilt_deg
```

**为什么加 `state` 和 `tilt_deg`：** 模式化之后，地面站/上位机需要在**不接串口**
的情况下判断「现在翻到哪一步了」。加这两个字段后，`listener flip_recovery_control`
或者 ROS2 订阅一条话题就能画出倾角曲线和状态迁移，不用 ssh 进飞控。
`state` 的数值与 `FlipPlanner::State` 枚举一一对应，零转换。

### 1.12 新增：`src/lib/flip_recovery/FlipPlanner.hpp`

纯逻辑，不依赖 uORB，可以拿 gtest 直接跑。详见第 3 节。

**相对 v1 的三处改动：**

| v1 | v2 | 原因 |
|---|---|---|
| `Input.switch_high` | `Input.mode_active` | 触发源从 AUX 电平换成模式 |
| `_off_seen` 回差（必须先 OFF 再 ON） | 删除，改用 `_rearm_needed` | 模式切换天然是边沿，不需要人为回差；改为「Done/Failed 后必须退出模式再进」 |
| 无 | `Plan.state` | 把状态机状态透出到 uORB，供观测 |
| 两组同油门 | **高位 `FLIP_THR_HIGH` + 低位 `FLIP_THR_LOW`** | 转轴是贴地棱，低位在轴上力臂≈0，必须靠高位出力 |
| 无 | `Settings.thr_high` / `thr_low` / `low_mode` | 高位、低位油门独立可调；低位还能强制正/反转或直接关掉 |

### 1.13 新增：`src/modules/flip_recovery/FlipRecovery.{hpp,cpp}`

**相对 v1 的改动：**

| v1 | v2 | 原因 |
|---|---|---|
| `FLIP_AUX` 电平触发 | 订阅 `vehicle_status.nav_state` | 模式化 |
| `_switch_high` | `_mode_active` | 同上 |
| 无 | `_param_rc_confirm`（0=不需要） | 拨杆容易误碰，保留一路 AUX 做二次确认的选项 |
| 无 | `_param_exit_mode` + `requestExitMode()` | 翻完自动切回 Position/Acro 等，不用用户再拨一次 |
| 无条件要求 RC 有效 | 有 RC 才检查油门最低 | 纯上位机场景（无遥控）也要能用 |
| 要求 `!failsafe` | 去掉 | 无遥控时 failsafe 常亮，但未解锁本来就飞不起来，安全性由 armed 保证 |

### 1.14 新增：`src/modules/flip_recovery/flip_recovery_params.c`

参数从 12 个变成 14 个：删 `FLIP_AUX`、`FLIP_THR_FWD/REV`、`FLIP_AUX_SCALE`，加 `FLIP_RC_CONFIRM`、`FLIP_EXIT_MODE`、`FLIP_THR_HIGH`、`FLIP_THR_LOW`、`FLIP_LOW_MODE`。

---

## 2. 模式触发的三条路径（都能用）

### 2.1 遥控器拨杆（推荐）

```
QGC → 参数 → COM_FLTMODE5 = 17 (Flip Recovery)
```

假设你的模式通道是 6 档，第 5 档就变成翻正开关。
拨进去 → `nav_state = 16` → 模块开始工作；拨出来 → 立即收油。

### 2.2 上位机话题（uORB / ROS2）

**方式 A：标准 MAVLink SET_MODE**

```python
# pymavlink
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    (12 << 16)   # custom_mode = main_mode(12) << 16 | sub_mode(0)
)
```

**方式 B：MAV_CMD_DO_SET_MODE**

```python
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    1,    # param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    12,   # param2 = PX4_CUSTOM_MAIN_MODE_FLIP_RECOVERY
    0, 0, 0, 0, 0)
```

**方式 C：直接发 uORB 话题（ROS2 / px4_ros2）**

```python
# 方式 C-1：VEHICLE_CMD_SET_NAV_STATE，直接给 nav_state 数值（ROS 专用后门）
cmd.command = 10071          # VEHICLE_CMD_SET_NAV_STATE
cmd.param1  = 16.0           # NAVIGATION_STATE_FLIP_RECOVERY

# 方式 C-2：VEHICLE_CMD_DO_SET_MODE
cmd.command = 176            # VEHICLE_CMD_DO_SET_MODE
cmd.param1  = 1.0
cmd.param2  = 12.0
cmd.param3  = 0.0
```

三条方式最终都进 `Commander::handle_command()`，结果完全一致。

### 2.3 命令行（台架）

```
commander mode flip
```

---

## 3. 从 IMU 到翻正完成的完整算法

### 3.1 数据链路（谁产生四元数）

```
BMI088 / BMI270 (SPI, 1 kHz)
        │  sensor_accel / sensor_gyro / sensor_temp
        ▼
sensors 模块 ──── 校准（bias / scale）、温度补偿、低通
        │  sensor_combined
        ▼
vehicle_imu（多 IMU 聚合、时间对齐）
        │
        ├─► EKF2：状态 [p, v, q, bg, ba, ...]，15 维
        │      - 陀螺积分得姿态预测
        │      - 加速度计重力方向做观测更新（静止时权重最高）
        │      - 磁力计（若有）修正航向
        │      - 输出：vehicle_attitude.q（四元数，NED 机体系→世界系）
        │
        └─► vehicle_angular_velocity（原始角速率，用来判断是否在翻滚）

        ▼
flip_recovery 每 20 ms 读一次 vehicle_attitude.q
```

**为什么用 EKF2 的四元数而不是直接读加速度计？**
加速度计测的是「比力」，静止时正好等于 −重力，可以算倾角；但翻正过程中机身
在转动，存在线加速度和振动，直接 atan2 出来会抖十几度。EKF2 已经把陀螺积分
和加速度观测融合过，输出的 `q` 平滑且延迟约 5~10 ms，足够 20 ms 的控制周期。

### 3.2 第一步：四元数 → 倾角（不用欧拉角）

```cpp
// FlipPlanner::tiltDegrees()
matrix::Quatf attitude{q};
if (!std::isfinite(norm) || fabsf(norm - 1.f) > 0.02f) return false;  // 模长异常直接丢弃
attitude.normalize();
alignment = matrix::Dcmf{attitude}(2, 2);   // R(2,2)
```

`R(2,2)` 的含义：机体系 Z 轴（也就是机体「向上」方向）在世界系 Z 轴（天）上的投影。

- 正立：机体 Z 朝上 → `R(2,2) = +1` → `tilt = 0°`
- 侧躺：机体 Z 水平 → `R(2,2) = 0` → `tilt = 90°`
- 完全倒扣：机体 Z 朝下 → `R(2,2) = −1` → `tilt = 180°`

`tilt = acos(clamp(R(2,2), -1, 1))`。

**为什么不用 `roll/pitch` 欧拉角：**
1. 万向锁：pitch = ±90° 时 roll 无定义，而 90° 恰好是我们最关心的分界点；
2. 欧拉角需要两次三角函数 + 一次 atan2，且 roll/pitch 的组合不能直接给出"总倾角"
   （pitch 90° + roll 90° 的总倾角不是 127°）；
3. `R(2,2)` 一步到位，且对 0°/180° 附近数值稳定。

### 3.3 第二步：算四路电机的「世界高度」

```cpp
// FlipPlanner::lowestPair()
h[i] = R(2,0) * layout.px[i] + R(2,1) * layout.py[i];
```

`layout.px[i] / py[i]` 来自 PX4 自己的几何参数：
`CA_ROTOR_COUNT`、`CA_ROTOR0..3_PX/PY`（机体坐标，前 +X，右 +Y，单位 m）。

`R(2,0)` 是机体系 X 轴在世界 Z 上的投影，`R(2,1)` 是 Y 轴的投影。
所以 `h[i]` 就是「电机 i 相对重心的竖直高度」，负数 = 贴地。

把 4 个 h 排序，最小的两个 = **低位组（low）**，剩下两个 = **高位组（high）**。

**为什么要选最低两路？** 因为「最低两路的连线」就是翻正的**瞬时转轴**。
倾斜的机身落地时，是两个低位电机 + 机架接触地面，机身绕这条贴地的棱转动：

```
        高位组（悬空，力臂 = 轴距，最大）
       ●───────────●
       │           │
       │     CG    │
       ●───────────●
     ═════════════════════   ← 地面
        低位组（在转轴上，力臂 ≈ 0）
```

由此得到两条结论，直接决定了出力策略：

1. **主攻必须是高位桨**。低位电机在转轴上，力臂约等于 0，给它加油门几乎不产生力矩，
   只会白耗电、让低位桨贴地打桨；高位电机离转轴最远，同样的推力能产生最大扭矩。
2. **低位桨只能当辅助**。给它一点反向推力可以减轻地面摩擦、防止低位桨叶直接顶在地上，
   但绝不能让它当主力。

> 早期版本这里写成了「抬最低两路力臂最短、最省力」，是错的 —— 说反了。
> 现在已经改为「高位主攻 + 低位辅助」，两个油门 `FLIP_THR_HIGH` / `FLIP_THR_LOW` 完全独立。

这套判定和 LLJ5 的固定「右翻」不同：LLJ5 只往右翻，我们是「哪边高压哪边」，
任何侧翻方向都能处理。

### 3.4 第三步：两个方向 + 两种策略

**先分清两个 180° 相反的力**：

| 概念 | 含义 | 关系 |
|---|---|---|
| **桨对空气** | 螺旋桨把空气往哪甩 | — |
| **机身受力 `Push`** | 空气对机身的反作用力 | **永远与桨对空气相反** |
| **电机转向** | `Normal` 出厂 / `Reversed` 反着转 | 决定桨对空气沿机体 ±Z |

正常飞行（tilt=0）：桨**朝下**甩空气 → 机身受力**朝上**（升力）。这是 `Normal` 的定义基准。

| 倾角 | `R(2,2)=cos θ` | 机体 −Z 指向 | `Normal` 桨对空气 | `Normal` 机身受力 |
|---|---|---|---|---|
| 60–90° | `> 0` | 朝下偏侧 | 朝下 | **朝上** |
| 90–180° | `< 0` | 朝上偏侧 | **朝上** | **朝下** |

#### 两组的分工不一样，不能用同一套逻辑套

| 组 | 位置 | 力臂 | 职责 | 油门 |
|---|---|---|---|---|
| **高位** | 离转轴最远 | 最大 | **唯一产生翻正力矩的组** | 全额，永远是主攻 |
| **低位** | 就在转轴上 | ≈ 0 | **卸压**：把机身抬一点，减小地面正压力与绕轴摩擦 | `FLIP_THR_LOW` 独立设定 |

低位为什么不参与产生力矩：它在转轴上，力臂≈0，推力再大也几乎不产生力矩。
它真正的价值在**摩擦**：

| 低位电机 | 机身受力 | 地面正压力 | 后果 |
|---|---|---|---|
| 正转（90–180°） | 朝下 | 增大 | 支点被压死 → 摩擦锁住转轴 → **拖累高位** |
| 反转（90–180°） | 朝上 | 减小 | 支点"浮"起来 → 摩擦小 → 高位转得动 |

所以低位组的目标恒定：**永远 `Push::Up`（抬）**，只是 90° 前后"抬"对应不同电机转向。

#### 策略 A「翻过去」（`FLIP_MODE=0`，默认）：高位全程反转

| 倾角 | 高位：机身受力 | 高位电机 | 低位：机身受力 | 低位电机 | 净效果 |
|---|---|---|---|---|---|
| **60–90°** | 朝下（压回正） | **Reversed** | 朝上（卸压） | **Normal** | tilt 减小，回正 |
| **90–180°** | 朝上（**抬，翻过去**） | **Reversed** | 朝上（卸压） | **Reversed** | tilt 增大，翻过 180° |

高位电机在 90° 处**不换向**；低位从 Normal 换成 Reversed（因为它要一直"抬"）。

#### 策略 B「压回去」（`FLIP_MODE=1`）：高位全程 Push::Down

| 倾角 | 高位 | 低位 | 净效果 |
|---|---|---|---|
| **60–90°** | Reversed（压） | Normal（抬） | tilt 减小 |
| **90–180°** | **Normal**（压） | **Reversed**（抬） | tilt 减小 |

始终朝 0° 拉，路径最短，但高位在 90° 处要换向。

#### 换算代码

```cpp
const bool normal_is_up = (R(2, 2) > 0.f);          // Normal 的机身受力是否朝上

Push high_push = Push::Down;                         // 默认压回去
if (zone == Zone::Inverted && strategy == FlipOver) {
    high_push = Push::Up;                            // 90~180 翻过去：抬高位
}
const Push low_push = Push::Up;                      // 低位恒为"抬"（卸压/减摩擦）

const bool high_normal = ((high_push == Push::Up) == normal_is_up);
const bool low_normal  = ((low_push  == Push::Up) == normal_is_up);
```

#### 120° 算例（策略 A）

```
R(2,0) = -0.866    R(2,2) = cos120° = -0.5  ->  normal_is_up = false
```

| 组 | 目标受力 | 换算转向 | 桨对空气 | 作用 |
|---|---|---|---|---|
| 高位（后两路） | `Push::Up` | **Reversed** | 朝下 → 机身被抬 | 产生翻正力矩（+X），tilt 增大 |
| 低位（前两路） | `Push::Up` | **Reversed** | 朝下 → 机身被抬 | 力臂≈0，只负责卸压减摩擦 |
| ❌ 低位改 Normal | `Push::Down` | — | 机身被压 | 支点压死，摩擦锁轴，拖累高位 |

#### 代价对比

| 策略 | 90–120° 段要转过的角度 | 优点 |
|---|---|---|
| **翻过去** | 240–270° | 高位转向固定，驱动最简单；过 180° 后重力帮忙 |
| **压回去** | 90–120° | 90° 附近路径最短 |

### 3.5 第四步：油门斜坡（防止电流冲击）

```cpp
target = min(max(fwd_mask ? thr_fwd : 0, rev_mask ? thr_rev : 0), 0.60);
_thr = min(target, _thr + slew * dt);   // slew 默认 0.5/s，dt = 20ms
```

从 0 线性升到目标值。默认主攻 0.20、辅助 0.20×0.5 = 0.10，上升率 0.5/s → 0.4 s 到顶。
**为什么要斜坡：** 电调从 0 直接跳到 20% 会产生很大电流尖峰，容易触发低压保护；
且突然的扭矩会让机身「弹」一下，反而打乱姿态估计。

硬上限 `HARD_MAX_THROTTLE = 0.60` 写死在代码里，参数怎么填都超不过去。

### 3.6 第五步：状态机

```
        ┌──────────────────────────────────────────────┐
        │                                              │
   Idle ─┤ mode_active && safe && tilt ≥ entry_angle   │
        ▼                                              │
      Ramp ──(油门到目标)──► Hold ──(tilt ≤ stop)──► Coast ──(稳定/1.5s)──► Done
        │                     │                                            │
        │                     └─(超时 / 无进度)──► Failed                  │
        └─(安全丢失 / 模式退出)──────────────────► Failed                  │
                                                                           │
   Done / Failed 后置 _rearm_needed，必须退出模式再进才能重试 ◄────────────┘
```

**三个退出条件：**

| 条件 | 判据 | 含义 |
|---|---|---|
| 成功收油 | `tilt ≤ stop_angle`（默认 60°） | 已经足够正，交给重力 |
| 超时 | 出力时间 > `FLIP_TIME`（3 s） | 电机/电调过热保护 |
| 无进度 | 0.8 s 内倾角减少 < 3° | 选错边、推力不够、或者被卡住了 |

「无进度」这一条很重要：如果电机方向配反了，倾角会**增大**而不是减小，
0.8 s 内就能发现并停机，不会一直顶着把机身推飞。

### 3.7 第六步：发布与执行

```cpp
flip_recovery_control_s msg;
msg.state    = 状态机状态;
msg.action   = driving ? ACTION_DRIVE : ACTION_STOP;
msg.zone     = ZONE_UPRIGHT / ZONE_INVERTED / ZONE_OFF;
msg.fwd_mask = control[i] > 0 的电机位掩码（bit0 = Motor1）;
msg.rev_mask = control[i] < 0 的电机位掩码;
msg.control[i] = 逐路带符号油门：+ = Normal，- = Reversed，0 = 停;
msg.tilt_deg = 当前倾角;
```

**为什么必须逐路给而不是「正转组/反转组两个油门」：**

策略 A 的 90~180° 段，高位和低位**同为 Reversed**，但油门不同（高位 0.20 全额、低位 0.10 打折）。
单一 `thr_rev` 无法同时表达这两个值，必须退回逐电机粒度。

典型输出（`FLIP_THR_HIGH = 0.20`、`FLIP_THR_LOW = 0.10`、`FLIP_LOW_MODE = 0`）：

| 倾角 | 策略 | 高位 | 低位 |
|---|---|---|---|
| 70° | 0 | Reversed 0.20 | Normal 0.10 |
| 85° | 0 | Reversed 0.20 | Normal 0.10 |
| 95° | 0 | Reversed 0.20 | **Reversed 0.10** |
| 120° | 0 | Reversed 0.20 | **Reversed 0.10** |
| 160° | 0 | Reversed 0.20 | **Reversed 0.10** |
| 120° | 1 | Normal 0.20 | Reversed 0.10 |

策略 0 下高位**全程 Reversed**，只有低位在 90° 处从 Normal 换成 Reversed。

`mask` 的位序与 `CA_ROTORn` 序号一致，也就是 PX4 的 Motor1..Motor4。
不需要另外维护「电机号→方位」的映射表 —— 因为方位本来就是从
`CA_ROTORn_PX/PY` 读出来的，两者天然对齐。

### 3.8 第七步：结束后自动切回

`Done` 或 `Failed` 且仍停留在翻正模式时，发一条
`VEHICLE_CMD_DO_SET_MODE(param1=1, param2=main_mode(FLIP_EXIT_MODE), param3=sub_mode)`。

默认切回 **Position**（`FLIP_EXIT_MODE=2`）。用户不用再拨一次杆。
`_exit_requested` 保证一条命令只发一次，避免刷屏。

---

## 4. 参数表

| 参数 | 默认 | 单位 | 说明 |
|---|---|---|---|
| `FLIP_EN` | 0 | — | 总开关。0 = 拨进模式也不转桨，只报状态 |
| `FLIP_RC_CONFIRM` | 0 | — | 0 = 不需二次确认；1..6 = 还要对应 aux 通道在高位 |
| `FLIP_EXIT_MODE` | 2 | — | 翻完切回的模式（0 Manual / 1 Alt / 2 Position / 3 Mission / 4 Hold / 5 Return / 6 Acro / 7 Offboard / 8 Stabilized） |
| `FLIP_MODE` | 0 | — | 0 = 翻过去（60–90 压高位回正，90–180 抬高位翻过 180°，两组转向固定）；1 = 压回去（全程压高位，路径最短但 90° 要换向） |
| `FLIP_ENTRY_ANG` | 75 | deg | 超过这个倾角才开始出力 |
| `FLIP_STOP_ANG` | 60 | deg | 低于这个倾角收油，交给重力 |
| `FLIP_THR_HIGH` | 0.20 | — | **高位组油门**（离地较高那两路）。产生翻正力矩的主攻组，全额 |
| `FLIP_THR_LOW` | 0.10 | — | **低位组油门**（贴地那两路）。卸压/减摩擦的辅助组，可独立调，甚至设得比高位大 |
| `FLIP_LOW_MODE` | 0 | — | 低位转向：0=Auto（永远抬，60–90 正转 / 90–180 反转）、1=强制正转、2=强制反转、3=关闭 |

> **调参建议**：`FLIP_THR_HIGH` 是唯一真正决定翻正快慢的参数，先从 0.15 起步；
> `FLIP_THR_LOW` 用来治"低位贴地磨得厉害/翻不动"，调大到 0.15~0.20 试试；
> 若怀疑低位在帮倒忙，直接 `FLIP_LOW_MODE=3` 关掉做对照。
| `FLIP_SLEW` | 0.50 | 1/s | 油门上升率 |
| `FLIP_TIME` | 3.0 | s | 总出力上限 |
| `FLIP_MIN_PROG` | 3.0 | deg | 进度检查阈值 |
| `FLIP_PROG_T` | 0.8 | s | 多久后开始查进度 |
| `FLIP_OUT_DIRECT` | 0 | — | 1 = 台架直发 `actuator_motors`（只驱动正转组） |

---

## 5. 调试顺序（拆桨！）

1. **只观测不出力**：`FLIP_EN=0`，`commander mode flip`，用
   `listener flip_recovery_control` 看 `tilt_deg / zone / fwd_mask / rev_mask`。
   把机身摆到 80°、120°、180°，确认低位掩码选对了。
2. **验证模式链路**：拨杆切进切出，`listener vehicle_status` 看
   `nav_state` 是否变成 16、拨出是否变回。同时试
   `commander mode flip` 和上位机 `SET_MODE`，确认三条路径都通。
3. **验证禁止解锁**：停留在翻正模式，试 `commander arm`，应当被拒绝并提示
   "mode not suitable for arming"。
4. **验证让位**：`listener actuator_motors`，在翻正模式下应该**没有**新数据
   （control_allocator 已闭嘴）；切回 Position 后恢复。
5. **台架转桨**：`FLIP_EN=1`、`FLIP_OUT_DIRECT=1`、`FLIP_THR_HIGH=0.1`、`FLIP_LOW_MODE=3`（先只让高位转），
   垫软物摆 80°，确认最低两路转、方向朝上。
6. **带桨低推力**：`FLIP_THR_HIGH=0.15`、`FLIP_THR_LOW=0.08`，连续 10 次，检查电机温度、
   每次结束是否归零。

---

## 6. 还没做完的一步（必须说清楚）

**反转的物理执行仍然缺驱动层。**

现在的链路是：

```
flip_recovery → flip_recovery_control.rev_mask / thr_rev
                        │
                        └─✗ 没有消费方
```

两个原因叠加：

1. **未解锁不输出**：`MixingOutput::output_limit_calc(armed=false, ...)` 会走
   `OutputLimitState::OFF`，把所有通道强制写成 `_disarmed_value`（DShot 是 0）。
   `FLIP_OUT_DIRECT` 直发 `actuator_motors` 也绕不过这一层。
2. **没有反向油门/方向命令**：DShot 常规模式只有 0..2047 单向油门，
   要让电机反着转，只有两条路：
   - **3D 油门**（`DSHOT_3D_ENABLE=1`，48..1047 反转 / 1047 停 / 1047..2047 正转）
     —— 需要电调固件支持 3D。**Bluejay 不支持 3D 模式**，BLHeli_S 支持。
   - **DShot 方向命令 20/21**（`spin_direction_normal` / `reversed`）
     —— Bluejay 0.19 支持，但需要给 `src/drivers/dshot/DShot.cpp` 加一个
     从 `flip_recovery_control` 到 `up_dshot_motor_command()` 的通道，
     并在 `MixingOutput` 里开一个「未解锁允许出力」的 override。

建议的下一步（照 LLJ5 的做法）：

| 文件 | 要加的东西 |
|---|---|
| `src/drivers/dshot/DShot.{h,cpp}` | 订阅 `flip_recovery_control`，`ACTION_DRIVE` 时调 `up_dshot_motor_command(i, DShot_cmd_spin_direction_reversed/normal)` |
| `src/lib/mixer_module/mixer_module.{hpp,cpp}` | `setFlipRecoveryOverride(bool, const float[])`：`_armed.armed==false` 时也走 `OutputLimitState::ON` |
| `src/modules/flip_recovery/FlipRecovery.cpp` | `Done/Failed/退出模式` 时发 `ACTION_STOP`，驱动侧把方向复位成 normal |

**这一步没有做，是因为它必须上真机验证**（不同 Bluejay 版本、不同电调对命令 20/21
的响应不一样，写错了会让电机一直反着转）。代码可以先写，但没有实测的驱动层改动
我不建议直接进固件。

---

## 7. Flash 占用

v1.17.0 基线在 `micoair_h743-v2` 上剩余约 **61 KB**。
本次模式化改动估计增加 **1.5–2.5 KB**（commander 分支 + 模块重写 + 新消息）：

| 项 | 估算 |
|---|---|
| `FlipRecovery.cpp` 重写 + `requestExitMode` | +0.6 KB |
| `FlipPlanner.hpp`（模板/内联，变化不大） | ±0 |
| 新消息字段（`state` + `tilt_deg`） | +0.1 KB |
| Commander / ui.hpp / mode_requirements 分支 | +0.5 KB |
| 参数（新增 2 个，删除 1 个） | +0.3 KB |

编译后请用 `arm-none-eabi-size` 确认，如果超了优先砍 `print_usage` 的长文本。
