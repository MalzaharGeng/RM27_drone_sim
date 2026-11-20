/****************************************************************************
 * 倾角翻正规划器（纯逻辑，无 uORB 依赖，方便单测）
 *
 * 输入：姿态四元数、电机几何、参数、安全条件、模式是否处于激活状态
 * 输出：需要正转/反转的电机掩码 + 各自油门 + 当前工作区（zone）+ 状态机状态
 *
 * ============================ 术语约定（唯一权威） ============================
 *
 * 【桨对空气】螺旋桨把空气往哪甩。正常飞行（tilt=0）时朝下甩，所以机身获得升力。
 *
 * 【机身受力 Push】空气对机身的反作用力方向。永远与"桨对空气"相反。
 *      Push::Up   = 空气把机身往天上推
 *      Push::Down = 空气把机身往地面压
 *   本文件全程只用它表达"想让机身怎么动"，因为真正让机身穿转起来的是这个力。
 *
 * 【电机转向】描述电机自己怎么转：
 *      Normal   = 出厂方向 -> 桨对空气沿机体 -Z
 *      Reversed = 反着转   -> 桨对空气沿机体 +Z
 *
 *   换算（全部歧义都在这里）：
 *      R(2,2) = cos(tilt) > 0（tilt < 90°，机体 -Z 朝下）：
 *          Normal -> 桨对空气朝下 -> Push::Up
 *      R(2,2) < 0（tilt > 90°，机体 -Z 朝上）：
 *          Normal -> 桨对空气朝上 -> Push::Down
 *
 *   代码里统一用 normal_is_up = (R(2,2) > 0) 做一次换算，绝不手写区间 if/else。
 *
 * ================================ 物理约定 ================================
 *
 *   R = Dcmf(q) 机体->世界，alignment = R(2,2) = cos(倾角)
 *   tilt = acos(alignment)：0° 正立，90° 侧躺，180° 完全倒扣
 *   电机高度 h_i = -(R(2,0)*PX_i + R(2,1)*PY_i)（NED Z 向下），越小越贴地
 *
 * ================================ 力臂与策略 ================================
 *
 *   翻正的瞬时转轴 = 两个低位电机的连线（贴地那条棱）。
 *
 *   高位组：离转轴最远，力臂最大 -> 唯一产生翻正力矩的组，永远全额油门。
 *   低位组：就在转轴上，力臂≈0    -> 力矩贡献可忽略，唯一作用是
 *           「把机身往上抬一点，减轻地面正压力、减小绕轴摩擦」。
 *           若反过来往下压，支点会被压死、摩擦锁住转轴，反而拖累高位。
 *           所以低位**永远 Push::Up（抬）**，只是 90° 前后"抬"对应的电机转向不同。
 *
 *   两种策略（60~90° 完全一致，只在 90~180° 分道扬镳）：
 *     FlipOver（默认）: 高位全程 Reversed
 *        60~90  : 高位 Push::Down（压回正） + 低位 Push::Up（抬）
 *        90~180 : 高位 Push::Up  （抬，翻过去）+ 低位 Push::Up（抬）
 *        低位在 90° 处 Normal -> Reversed，高位不换向
 *     PressBack: 高位全程 Push::Down（压回去）
 *        60~180 : 高位 Push::Down + 低位 Push::Up
 *        路径最短，但高位在 90° 处要换向
 *
 * ================================ 触发模型 ================================
 *
 *   不再用 AUX 开关电平触发，而是由飞行模式驱动。
 *   上层把 nav_state == FLIP_RECOVERY 传成 in.mode_active，
 *   本文件只负责"模式激活期间"的状态机；模式的进出由 Commander 管。
 ****************************************************************************/
#pragma once

#include <cmath>
#include <cstdint>
#include <matrix/matrix/math.hpp>

namespace flip_recovery
{
// 油门硬上限：DShot 归一化值，不是推力/功率百分比
static constexpr float HARD_MAX_THROTTLE = 0.60f;

// 机身受力方向（= 空气对飞机的反作用力方向）。
// 注意：螺旋桨"对空气"的推力方向永远与它相反 —— 本文件统一只描述机身受力，
// 因为真正让机身转起来的是这个力。
enum class Push : uint8_t {
	Up = 0,    // 把机身往天上推（对应桨把空气朝下甩）
	Down = 1,  // 把机身往地面压（对应桨把空气朝上甩）
};

// 翻正策略。两种在 60~90° 完全一致，只在 90~180° 分道扬镳。
// 两种策略下"低位永远 Push::Up（抬，减摩擦）"都不变，区别只在高位。
enum class Strategy : uint8_t {
	// 翻过去（默认）：60~90 压高位回正；90~180 抬高位，让机身绕低位棱继续翻过 180°。
	// 等效于「高位全程反转」，高位在 90° 处不换向。
	FlipOver = 0,

	// 压回去：60~180 全程压高位，始终朝 0° 方向拉。路径最短，但高位在 90° 处要换向。
	PressBack = 1,
};

// 工作区：只用来决定"电机转向怎么换算到机身受力方向"，不承载出力策略。
enum class Zone : uint8_t {
	Off = 0,        // < 停止角（默认 60°），完全不出力，靠重力
	Upright = 1,    // 60°~90°：cos>0，机体 +Z 朝地 -> Normal 受力向上, Reversed 受力向下
	Inverted = 2,   // 90°~180°：cos<0，机体 +Z 朝天 -> Normal 受力向下, Reversed 受力向上
};

// 电机几何，来自 PX4 的 CA_ROTOR_COUNT / CA_ROTORn_PX / CA_ROTORn_PY
struct Layout {
	bool valid{false};
	float px[4] {};   // 前为正
	float py[4] {};   // 右为正
};

// 低位组的转向模式（调试用，正常飞行用 Auto 即可）
enum class LowMode : uint8_t {
	Auto = 0,       // 按倾角自动决定 —— 目标永远是"抬"（卸压/减摩擦）
	ForceNormal,    // 强制电机正转（Normal），不管倾角
	ForceReversed,  // 强制电机反转（Reversed），不管倾角
	Off,            // 低位完全不转
};

struct Settings {
	float entry_angle{75.f};     // 进入门限（度）
	float stop_angle{60.f};      // 停止门限（度），低于它交给重力
	float thr_high{0.20f};       // 高位组油门（产生力矩的主攻组，全额）
	float thr_low{0.10f};        // 低位组油门（卸压/减摩擦的辅助组）
	int32_t low_mode{0};         // 低位转向模式，见 enum LowMode
	float slew{0.50f};           // 油门上升率 (1/s)
	float max_time{3.0f};        // 总出力时长上限 (s)
	float min_progress{3.0f};    // 最短进度要求 (度)
	float progress_time{0.80f};  // 进度判定起始时间 (s)
	int32_t strategy{0};         // 0=翻过去（默认）1=压回去，见 enum Strategy
};

struct Input {
	uint64_t now{0};
	bool enabled{false};      // FLIP_EN
	bool safe{false};         // 综合安全：未解锁、落地、姿态/陀螺有效、（需要时）遥控就绪
	bool mode_active{false};  // nav_state == NAVIGATION_STATE_FLIP_RECOVERY
	float q[4] {1.f, 0.f, 0.f, 0.f};
	float gyro[3] {};         // rad/s，仅用于日志
};

// 失败原因码，数值与 msg/FlipRecoveryStatus.msg 的 REASON_* 一一对应
enum ReasonCode : uint8_t {
	ReasonNone = 0,
	ReasonTimeout = 1,
	ReasonNoProgress = 2,
	ReasonSafetyLost = 3,
	ReasonModeExited = 4,
	ReasonAttitudeReset = 5,
	ReasonNotSettled = 6,
};

struct Plan {
	bool active{false};
	uint8_t state{0};         // 与 Plan::State 数值一致，直接写进 uORB 消息
	Zone zone{Zone::Off};
	uint8_t fwd_mask{0};      // control[i] > 0 的电机（Normal）
	uint8_t rev_mask{0};      // control[i] < 0 的电机（Reversed）
	uint8_t high_mask{0};     // 高位组（主攻），供日志/调试用
	uint8_t low_mask{0};      // 低位组（辅助/卸压），供日志/调试用
	// 逐路电机的带符号油门：+ = Normal，- = Reversed，0 = 停。
	// 必须逐路给：策略 A 的 90~180° 段高/低位同为 Reversed，但油门不同（全额 vs 打折）。
	float control[4] {};
	float tilt_deg{0.f};
	uint8_t reason_code{ReasonNone};   // 失败原因，见 ReasonCode
	const char *reason{"idle"};
};

class FlipPlanner
{
public:
	enum class State : uint8_t { Idle = 0, Ramp = 1, Hold = 2, Coast = 3, Done = 4, Failed = 5 };

	Plan update(const Input &in, const Settings &s, const Layout &layout)
	{
		Plan out{};
		out.tilt_deg = _tilt;
		out.state = static_cast<uint8_t>(_state);
		out.reason_code = _fail_code;
		out.reason = _reason;
		const bool running = _state == State::Ramp || _state == State::Hold || _state == State::Coast;

		// Mode exit must work even if attitude/layout is no longer available.
		if (!in.mode_active) {
			if (running) { fail("mode exited", ReasonModeExited); }
			_rearm_needed = false;
			_thr = 0.f;
			out.state = static_cast<uint8_t>(_state);
			out.reason_code = _fail_code;
			out.reason = _reason;
			return out;
		}

		float alignment = NAN;
		bool valid = tiltDegrees(in.q, alignment) && layout.valid;
		for (unsigned i = 0; i < 3; ++i) { valid = valid && std::isfinite(in.gyro[i]); }
		if (!valid) {
			if (running) { fail("invalid attitude, gyro or layout", ReasonSafetyLost); }
			out.state = static_cast<uint8_t>(_state);
			out.reason_code = _fail_code;
			out.reason = _reason;
			return out;
		}

		const float tilt = acosf(fminf(1.f, fmaxf(-1.f, alignment))) * 57.29577951308232f;
		out.tilt_deg = tilt;
		_tilt = tilt;
		matrix::Quatf attitude{in.q};
		attitude.normalize();
		const matrix::Dcmf R{attitude};
		uint8_t low = 0, high = 0;
		if (!lowestPair(layout, R, low, high)) {
			if (running) { fail("invalid motor geometry", ReasonSafetyLost); }
			out.state = static_cast<uint8_t>(_state);
			out.reason_code = _fail_code;
			out.reason = _reason;
			return out;
		}

		// ---- 4. 启动判定 ----
		if (_state == State::Idle || _state == State::Done || _state == State::Failed) {
			// Done / Failed 之后必须把模式切出去再切回来，才能开始下一次
			if (_rearm_needed) {
				out.reason = "waiting for mode exit before re-arm";
				out.state = static_cast<uint8_t>(_state);
				return out;
			}

			if (!in.enabled) {
				out.reason = "disabled";
				out.state = static_cast<uint8_t>(_state);
				return out;
			}

			if (!in.safe) {
				out.reason = "safety blocked";
				out.state = static_cast<uint8_t>(_state);
				return out;
			}

			if (tilt < s.entry_angle) {
				out.reason = "tilt below entry angle";
				out.state = static_cast<uint8_t>(_state);
				return out;
			}

			_state = State::Ramp;
			_started = in.now;
			_fail_code = ReasonNone;
			_last_now = in.now;
			_settled_since = 0;
			// Track a signed angle in the initial recovery plane. Unlike acos,
			// atan2 can be unwrapped across 180 degrees without reversing progress.
			_plane_x = R(0, 2);
			_plane_y = R(1, 2);
			float plane_norm = hypotf(_plane_x, _plane_y);
			if (plane_norm < 1e-4f) {
				float vx = 0.f, vy = 0.f;
				for (unsigned i = 0; i < 4; ++i) {
					const float sign = (high & (1u << i)) ? 1.f : -1.f;
					vx += sign * layout.px[i]; vy += sign * layout.py[i];
				}
				_plane_x = R(0, 0) * vx + R(0, 1) * vy;
				_plane_y = R(1, 0) * vx + R(1, 1) * vy;
				plane_norm = hypotf(_plane_x, _plane_y);
			}
			if (plane_norm < 1e-6f) {
				fail("degenerate recovery plane", ReasonSafetyLost);
			} else {
				_plane_x /= plane_norm; _plane_y /= plane_norm;
			}
			_previous_angle = atan2f(R(0, 2) * _plane_x + R(1, 2) * _plane_y, R(2, 2));
			_path_progress = 0.f;
			_flip_over = s.strategy == 0 && tilt >= 90.f;
			_session = in.now;
			_thr = 0.f;
			_reason = "ramping up";
		}

		// ---- 5. 运行中掉安全条件 -> 直接停 ----
		if (running && (!in.safe || !in.enabled)) {
			fail("safety lost or disabled", ReasonSafetyLost);
		}

		if (_state == State::Ramp || _state == State::Hold) {
			const float angle = atan2f(R(0, 2) * _plane_x + R(1, 2) * _plane_y, R(2, 2));
			const float delta = atan2f(sinf(angle - _previous_angle), cosf(angle - _previous_angle));
			_path_progress += (_flip_over ? delta : -delta) * 57.29577951308232f;
			_previous_angle = angle;
		}

		// ---- 6. 选择工作区 ----
		Zone zone = Zone::Off;

		if (tilt >= 90.f) {
			zone = Zone::Inverted;

		} else if (tilt >= s.stop_angle) {
			zone = Zone::Upright;
		}

		// 到达停止角：收油，交给重力
		if (_state == State::Ramp || _state == State::Hold) {
			if (tilt <= s.stop_angle) {
				_state = State::Coast;
				_coast_started = in.now;
				_thr = 0.f;
				_reason = "tilt below stop angle, gravity takes over";

			} else if (in.now - _started >= static_cast<uint64_t>(s.max_time * 1e6f)) {
				fail("timeout", ReasonTimeout);

			} else if (in.now - _started >= static_cast<uint64_t>(s.progress_time * 1e6f)
				   && _path_progress < s.min_progress) {
				fail("no directed recovery progress", ReasonNoProgress);
			}
		}

		// ---- 7. 目标受力方向 -> 每组电机转向 ----
		//
		// 分工原则（两组职责完全不同，不要用同一套逻辑套）：
		//   高位组 —— 唯一产生翻正力矩的组，力臂最大，所以永远全额油门。
		//   低位组 —— 就在转轴上，力臂≈0，力矩贡献可忽略；它唯一的作用是
		//             把机身往上抬一点，减轻对地面的正压力、减小绕轴摩擦。
		//             若反过来往下压，支点会被压死、摩擦锁住转轴，反而拖累高位。
		//             所以低位**永远 Push::Up**（抬），只是 90° 前后"抬"对应的电机转向不同。
		//
		// 策略 A「翻过去」(默认, strategy=0)：高位全程反转
		//   60~90  : 高位 Push::Down（压回正） + 低位 Push::Up（抬） -> tilt 减小
		//   90~180 : 高位 Push::Up  （抬，翻过去）+ 低位 Push::Up（抬） -> tilt 增大
		// 策略 B「压回去」(strategy=1)：高位全程 Push::Down
		//   60~180 : 高位 Push::Down（压）+ 低位 Push::Up（抬） -> 始终朝 0° 拉
		//
		// 换算：Normal 产生的机身受力方向 = 机体 -Z 的世界指向
		//   normal_is_up = R(2,2) > 0
		//   true (tilt<90) : Normal -> Push::Up
		//   false(tilt>90) : Normal -> Push::Down
		const bool normal_is_up = (R(2, 2) > 0.f);

		// 高位：默认压回去；选"翻过去"且在 90~180 段时改为抬
		Push high_push = Push::Down;

		if (zone == Zone::Inverted && static_cast<Strategy>(s.strategy) == Strategy::FlipOver) {
			high_push = Push::Up;
		}

		// 低位：Auto 时永远抬（卸压/减摩擦）；调试可强制转向或直接关掉
		const Push low_push = Push::Up;

		const bool high_normal = ((high_push == Push::Up) == normal_is_up);

		bool low_normal = ((low_push == Push::Up) == normal_is_up);
		bool low_enabled = true;

		switch (static_cast<LowMode>(s.low_mode)) {
		case LowMode::ForceNormal:
			low_normal = true;
			break;

		case LowMode::ForceReversed:
			low_normal = false;
			break;

		case LowMode::Off:
			low_enabled = false;
			break;

		case LowMode::Auto:
		default:
			break;
		}

		// ---- 8. 逐路油门 ----
		// 必须逐电机算：策略 A 的 90~180° 段高/低位同为 Reversed，但油门不同
		// （高位全额、低位打折），用单一"反转组油门"无法表达。
		const float dt = (_last_now != 0 && in.now >= _last_now) ? fminf(0.05f, (in.now - _last_now) * 1e-6f) : 0.f;
		_last_now = in.now;

		const float thr_high = fminf(s.thr_high, HARD_MAX_THROTTLE);
		const float thr_low  = low_enabled ? fminf(s.thr_low, HARD_MAX_THROTTLE) : 0.f;

		const bool driving_now = (zone != Zone::Off);
		const float target = driving_now ? fmaxf(thr_high, thr_low) : 0.f;

		if (_state == State::Ramp) {
			_thr = fminf(target, _thr + s.slew * dt);

			if (_thr >= target) {
				_state = State::Hold;
				_reason = "holding";
			}
		}

		if (_state == State::Coast) {
			const float rate = sqrtf(in.gyro[0] * in.gyro[0] + in.gyro[1] * in.gyro[1] + in.gyro[2] * in.gyro[2]);
			if (tilt <= 15.f && rate <= 0.1745329252f) {
				if (_settled_since == 0) { _settled_since = in.now; }
				if (in.now - _settled_since >= 1000000) {
					_state = State::Done;
					_rearm_needed = true;
					_fail_code = ReasonNone;
					_reason = "upright and settled for one second";
				}
			} else {
				_settled_since = 0;
			}
			if (_state == State::Coast && in.now - _coast_started >= 3000000) {
				fail("did not settle after throttle cut", ReasonNotSettled);
			}
		} else if (_state == State::Done || _state == State::Failed) {
			_thr = 0.f;
		}

		// ---- 9. 组装输出 ----
		const bool driving = (_state == State::Ramp || _state == State::Hold) && driving_now;

		out.active = _state != State::Idle && _state != State::Done && _state != State::Failed;
		out.state = static_cast<uint8_t>(_state);
		out.zone = driving ? zone : Zone::Off;
		out.high_mask = driving ? high : 0;
		out.low_mask = (driving && low_enabled) ? low : 0;

		for (unsigned i = 0; i < 4; ++i) {
			out.control[i] = 0.f;

			if (!driving) {
				continue;
			}

			float thr = 0.f;
			bool normal = false;

			if (high & (1u << i)) {
				thr = fminf(_thr, thr_high);
				normal = high_normal;

			} else if (low & (1u << i)) {
				thr = fminf(_thr, thr_low);
				normal = low_normal;
			}

			if (thr > 0.f) {
				out.control[i] = normal ? thr : -thr;

				if (normal) {
					out.fwd_mask |= static_cast<uint8_t>(1u << i);

				} else {
					out.rev_mask |= static_cast<uint8_t>(1u << i);
				}
			}
		}

		out.tilt_deg = tilt;
		out.reason_code = _fail_code;
		out.reason = _reason;
		return out;
	}

	State state() const { return _state; }
	const char *reason() const { return _reason; }
	uint64_t session() const { return _session; }
	float tilt() const { return _tilt; }
	float throttle() const { return _thr; }

	void fail(const char *why, uint8_t code = ReasonNone)
	{
		_state = State::Failed;
		_thr = 0.f;
		_rearm_needed = true;
		_reason = why;
		_fail_code = code;
	}

	// 四元数 -> alignment（R(2,2)）；模长异常返回 false
	static bool tiltDegrees(const float q[4], float &alignment)
	{
		alignment = NAN;
		matrix::Quatf attitude{q};
		const float norm = attitude.norm();

		if (!std::isfinite(norm) || fabsf(norm - 1.f) > 0.02f) {
			return false;
		}

		attitude.normalize();
		alignment = matrix::Dcmf{attitude}(2, 2);
		return true;
	}

	// 按世界系高度排序，取最低的两路 = 贴地那两路；它们的连线就是翻正转轴，
	// 剩下两路是高位组（力臂最大，负责产生力矩）
	static bool lowestPair(const Layout &layout, const matrix::Dcmf &R, uint8_t &low, uint8_t &high)
	{
		if (!layout.valid) {
			return false;
		}

		float h[4];

		for (unsigned i = 0; i < 4; ++i) {
			if (!std::isfinite(layout.px[i]) || !std::isfinite(layout.py[i])) { return false; }
			// NED Z points down: larger down coordinate means LOWER height.
			h[i] = -(R(2, 0) * layout.px[i] + R(2, 1) * layout.py[i]);
		}

		// 选最小的两个（简单插入排序，n=4）
		unsigned idx[4] {0, 1, 2, 3};

		for (unsigned a = 0; a < 3; ++a) {
			for (unsigned b = a + 1; b < 4; ++b) {
				if (h[idx[b]] < h[idx[a]]) {
					const unsigned t = idx[a];
					idx[a] = idx[b];
					idx[b] = t;
				}
			}
		}

		low = static_cast<uint8_t>((1u << idx[0]) | (1u << idx[1]));
		high = static_cast<uint8_t>(0x0Fu & ~low);
		return true;
	}

private:
	State _state{State::Idle};
	bool _rearm_needed{false};   // Done/Failed 后置位，模式退出才清零
	uint8_t _fail_code{ReasonNone};
	uint64_t _session{0}, _started{0}, _last_now{0}, _coast_started{0}, _settled_since{0};
	float _thr{0.f}, _tilt{NAN};
	float _plane_x{0.f}, _plane_y{0.f}, _previous_angle{0.f}, _path_progress{0.f};
	bool _flip_over{false};
	const char *_reason{"idle"};
};

} // namespace flip_recovery
