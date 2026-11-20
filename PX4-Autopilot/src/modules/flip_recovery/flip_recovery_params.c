/****************************************************************************
 * flip_recovery 参数
 ****************************************************************************/

/**
 * 使能倾角翻正
 *
 * 关闭时模块仍可运行（模式可以进，但不出力、只报状态）。
 * 这是一个总保险：即使拨错开关，也不会转桨。
 *
 * @boolean
 * @group Flip Recovery
 */
PARAM_DEFINE_INT32(FLIP_EN, 0);

/**
 * 遥控二次确认通道
 *
 * 0 = 不需要二次确认，进入模式即开始翻正。
 * 1..6 = 除模式开关外，还必须把对应 aux1..aux6 通道打到高位才开始出力。
 * 作用：模式开关在拨杆上容易被误碰，加一路开关做双保险。
 *
 * @min 0
 * @max 6
 * @group Flip Recovery
 */
PARAM_DEFINE_INT32(FLIP_RC_CONFIRM, 0);

/**
 * 翻正结束后切回的模式
 *
 * 与 COM_FLTMODE 的取值含义一致：
 * 0=Manual, 1=Altitude, 2=Position, 3=Mission, 4=Hold, 5=Return,
 * 6=Acro, 7=Offboard, 8=Stabilized。
 *
 * @min 0
 * @max 8
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 3 Mission
 * @value 4 Hold
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @group Flip Recovery
 */
PARAM_DEFINE_INT32(FLIP_EXIT_MODE, 2);

/**
 * 翻正策略
 *
 * 0 = 翻过去（默认）：60~90° 把高位压下去回正；90~180° 把高位抬起来，
 *     让机身绕低位那条棱继续翻过 180°（180°→270°→360° 回到正立）。
 *     高位全程反转；低位在 90° 处切换，所有换向由后端停机后执行。
 * 1 = 压回去：60~180° 全程把高位压下去，始终朝 0° 方向拉。路径最短，
 *     但 90° 处要切换一次电机转向。
 *
 * 两者在 60~90° 完全一致，只在 90~180° 不同。
 *
 * @min 0
 * @max 1
 * @value 0 Flip over
 * @value 1 Press back
 * @group Flip Recovery
 */
PARAM_DEFINE_INT32(FLIP_MODE, 0);

/**
 * 进入门限（倾角）
 *
 * 倾角超过该值才开始出力。倾角定义：0° 正立，90° 侧躺，180° 完全倒扣。
 *
 * @unit deg
 * @min 60
 * @max 180
 * @group Flip Recovery
 */
PARAM_DEFINE_FLOAT(FLIP_ENTRY_ANG, 75.0f);

/**
 * 停止门限（倾角）
 *
 * 倾角小于该值即收油，剩余部分交给重力完成。必须小于进入门限（形成回差）。
 *
 * @unit deg
 * @min 0
 * @max 120
 * @group Flip Recovery
 */
PARAM_DEFINE_FLOAT(FLIP_STOP_ANG, 60.0f);

/**
 * 高位组油门
 *
 * 高位 = 在真实世界里离地面较高的那两路电机。它们离翻正转轴最远、力臂最大，
 * 是唯一真正产生翻正力矩的组，所以这里给的是全额油门（不打折）。
 *
 * DShot 归一化油门（0..1），不是推力或功率百分比。起步建议 0.15~0.25。
 *
 * @min 0.02
 * @max 0.6
 * @increment 0.01
 * @group Flip Recovery
 */
PARAM_DEFINE_FLOAT(FLIP_THR_HIGH, 0.20f);

/**
 * 低位组油门
 *
 * 低位 = 贴地的那两路电机。它们就在翻正转轴上，力臂约等于 0，
 * 推力几乎不产生力矩，只负责"抬一点、卸掉地面正压力、减小绕轴摩擦"。
 *
 * 正常应明显低于高位（默认 0.10）。测试时可以随便调，甚至设得比高位还大。
 * 配合 FLIP_LOW_MODE 可以进一步强制低位的转向或直接关掉。
 *
 * @min 0
 * @max 0.6
 * @increment 0.01
 * @group Flip Recovery
 */
PARAM_DEFINE_FLOAT(FLIP_THR_LOW, 0.10f);

/**
 * 低位组转向模式
 *
 * 调试接口。正常用 0（自动），低位会一直朝"抬"的方向转，只是 90° 前后
 * 对应的电机转向不同（60~90° 正转，90~180° 反转）。
 *
 * 1/2 用来做对照实验：强制低位正转或反转，观察是否更顺。
 * 3 用来验证"低位完全不转、纯靠高位"能不能翻得动。
 *
 * @min 0
 * @max 3
 * @value 0 Auto (always lift)
 * @value 1 Force normal
 * @value 2 Force reversed
 * @value 3 Off
 * @group Flip Recovery
 */
PARAM_DEFINE_INT32(FLIP_LOW_MODE, 0);

/**
 * 油门上升率
 *
 * 单位：每秒增加的归一化油门。越小越温和。
 *
 * @unit 1/s
 * @min 0.05
 * @max 2.0
 * @increment 0.05
 * @group Flip Recovery
 */
PARAM_DEFINE_FLOAT(FLIP_SLEW, 0.50f);

/**
 * 总出力时长上限
 *
 * 超过即判失败并停止（防止电机/电调过热）。
 *
 * @unit s
 * @min 0.2
 * @max 5.0
 * @increment 0.1
 * @group Flip Recovery
 */
PARAM_DEFINE_FLOAT(FLIP_TIME, 3.0f);

/**
 * 最短进度要求
 *
 * 在 FLIP_PROG_T 后，按初始翻转平面累计的有向进度不足该值，判定"选错边或推力不够"并退出。
 *
 * @unit deg
 * @min 0
 * @max 30
 * @group Flip Recovery
 */
PARAM_DEFINE_FLOAT(FLIP_MIN_PROG, 3.0f);

/**
 * 进度判定起始时间
 *
 * 出力多久之后开始检查 FLIP_MIN_PROG。
 *
 * @unit s
 * @min 0.2
 * @max 2.0
 * @increment 0.1
 * @group Flip Recovery
 */
PARAM_DEFINE_FLOAT(FLIP_PROG_T, 0.8f);

/**
 * 台架直出模式
 *
 * 1 = 直接发布 actuator_motors 驱动"正转组"（仅台架测试）。
 * 进入翻正模式后 control_allocator 会让位，不会与本模块抢 actuator_motors。
 * 反转组仍然只能靠驱动层实现。
 *
 * @boolean
 * @group Flip Recovery
 */
PARAM_DEFINE_INT32(FLIP_OUT_DIRECT, 0);
