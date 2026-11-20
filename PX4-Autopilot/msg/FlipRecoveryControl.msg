# 翻正控制请求：由 flip_recovery 模块产生。
# 两个消费方：
#   1) 电机输出后端（DShot / PWM 驱动）—— DShot 已接入停机/换向/恢复，仿真后端需独立适配
#   2) 地面站 / 上位机 —— 用于观测状态机与实时倾角
#
# 该话题只在 NAVIGATION_STATE_FLIP_RECOVERY 模式下才有 DRIVE 动作。

uint64 timestamp
uint64 session_id              # 本次翻正会话的唯一标识（进入模式的时刻，us）

uint8 ACTION_STOP = 0          # 停桨，后端应恢复电机默认转向
uint8 ACTION_DRIVE = 1         # 按 control[] 出力

uint8 ZONE_OFF = 0             # 倾角 < 停止角，不出力，靠重力
uint8 ZONE_UPRIGHT = 1         # 60~90 度：cos(tilt)>0，电机 Normal 的机身受力朝上
uint8 ZONE_INVERTED = 2        # 90~180 度：cos(tilt)<0，电机 Normal 的机身受力朝下

uint8 STATE_IDLE = 0
uint8 STATE_RAMP = 1           # 油门斜坡上升中
uint8 STATE_HOLD = 2           # 达到目标油门，持续出力
uint8 STATE_COAST = 3          # 已收油，等重力把机身放平
uint8 STATE_DONE = 4
uint8 STATE_FAILED = 5

uint8 state                    # 状态机当前状态（STATE_*）
uint8 action                   # ACTION_*
uint8 zone                     # ZONE_*
uint8 fwd_mask                 # control[i] > 0 的电机（Normal，bit0=Motor1）
uint8 rev_mask                 # control[i] < 0 的电机（Reversed）

# 逐路电机的带符号油门，与 CA_ROTORn 序号一致（bit/下标 0 = Motor1）：
#   > 0 : 该路按 Normal 方向转，油门取绝对值
#   < 0 : 该路按 Reversed 方向转（DShot 方向命令 20/21 或 3D 油门）
#   = 0 : 该路停转
# 必须逐路给：策略 A 的 90~180° 段高/低位同为 Reversed，但油门不同（高位全额、低位打折），
# 用单一"反转组油门"无法表达。
float32[4] control            # [@range -1, 1] 归一化 DShot 油门，不是推力百分比
float32 tilt_deg               # 当前倾角：0 正立，90 侧躺，180 倒扣
float32 slew_rate              # 归一化油门/s；后端在换向完成后重新限制实际上升率
