# 翻正状态回传：给上位机判断"这一步做完了没有"。
#
# 与控制请求 flip_recovery_control 的区别：
#   flip_recovery_control 是模块 -> 电机后端的控制指令
#   flip_recovery_status  是模块 -> 上位机的执行结果
#
# 上位机的典型用法：
#   发完切模式命令后开始订阅本话题，等 result != RESULT_NONE 即可发下一步
#   （解锁、起飞、切回其它模式…），不需要轮询 nav_state 或读日志。

uint64 timestamp
uint64 session_id              # 本次翻正会话标识，与 flip_recovery_control 一致

uint8 STATE_IDLE = 0
uint8 STATE_RAMP = 1
uint8 STATE_HOLD = 2
uint8 STATE_COAST = 3
uint8 STATE_DONE = 4
uint8 STATE_FAILED = 5

uint8 RESULT_NONE = 0          # 还没结束（进行中或未开始）
uint8 RESULT_SUCCESS = 1       # 停油后倾角<=15度、角速度<=10度/秒保持一秒
uint8 RESULT_FAILED = 2        # 失败，看 reason

uint8 REASON_NONE = 0
uint8 REASON_TIMEOUT = 1       # 出力超过 FLIP_TIME
uint8 REASON_NO_PROGRESS = 2   # 进度检查未通过（选错边 / 方向反 / 推力不足）
uint8 REASON_SAFETY_LOST = 3   # 安全条件丢失（解锁、离地、遥控丢失、FLIP_EN=0）
uint8 REASON_MODE_EXITED = 4   # 翻正中途被切出模式
uint8 REASON_ATTITUDE_RESET = 5 # 姿态估计被重置

uint8 REASON_NOT_SETTLED = 6  # 停油后三秒内未保持正立/低角速度一秒

uint8 state                    # STATE_*
uint8 result                   # RESULT_*
uint8 reason                   # REASON_*，仅 result == RESULT_FAILED 时有意义
float32 tilt_deg               # 结束时（或当前）倾角
float32 duration_s             # 从开始出力到结束的时长
bool mode_active               # 当前是否仍停留在翻正模式（用于判断是否已自动切回）
