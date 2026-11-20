/****************************************************************************
 * flip_recovery 主模块
 *
 * 这个文件只做四件事：
 *   1. 判断当前是否处于 NAVIGATION_STATE_FLIP_RECOVERY 模式
 *   2. 汇总安全条件（未解锁 / 落地 / 姿态有效 / 可选遥控二次确认）
 *   3. 调 FlipPlanner 算出「哪几路正转、哪几路反转、油门多少」
 *   4. 把结果发布成 flip_recovery_control（以及可选的 actuator_motors 直出）
 ****************************************************************************/
#include "FlipRecovery.hpp"

#include <commander/px4_custom_mode.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <parameters/param.h>

using namespace time_literals;

FlipRecovery::FlipRecovery() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	updateLayout();
}

// 从 PX4 自己的电机配置里读几何：CA_ROTOR_COUNT / CA_ROTORn_PX / CA_ROTORn_PY
// 这样"电机号"始终和 PX4 里的定义一致，不需要另外维护映射表
void FlipRecovery::updateLayout()
{
	_layout.valid = false;

	int32_t rotor_count = 0;

	if (param_get(param_find("CA_ROTOR_COUNT"), &rotor_count) != PX4_OK || rotor_count != 4) {
		return;
	}

	for (unsigned i = 0; i < 4; ++i) {
		char name[17];
		snprintf(name, sizeof(name), "CA_ROTOR%u_PX", i);

		if (param_get(param_find(name), &_layout.px[i]) != PX4_OK) {
			return;
		}

		snprintf(name, sizeof(name), "CA_ROTOR%u_PY", i);

		if (param_get(param_find(name), &_layout.py[i]) != PX4_OK) {
			return;
		}
	}

	_layout.valid = true;
}

// FLIP_RC_CONFIRM = 1..6 时返回对应 aux 通道值，0 返回 NAN
float FlipRecovery::auxInput() const
{
	switch (_param_rc_confirm.get()) {
	case 1: return _manual.aux1;
	case 2: return _manual.aux2;
	case 3: return _manual.aux3;
	case 4: return _manual.aux4;
	case 5: return _manual.aux5;
	case 6: return _manual.aux6;
	default: return NAN;
	}
}

// COM_FLTMODE 的枚举编号 -> PX4 custom mode。只覆盖地面翻正可能用到的几个模式
static bool modeFromParam(int32_t value, uint8_t &main_mode, uint8_t &sub_mode)
{
	sub_mode = 0;

	switch (value) {
	case 0: main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL; return true;
	case 1: main_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL; return true;
	case 2: main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL; return true;
	case 3: main_mode = PX4_CUSTOM_MAIN_MODE_AUTO; sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION; return true;
	case 4: main_mode = PX4_CUSTOM_MAIN_MODE_AUTO; sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LOITER; return true;
	case 5: main_mode = PX4_CUSTOM_MAIN_MODE_AUTO; sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL; return true;
	case 6: main_mode = PX4_CUSTOM_MAIN_MODE_ACRO; return true;
	case 7: main_mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD; return true;
	case 8: main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED; return true;
	default: return false;
	}
}

void FlipRecovery::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s update;
		_parameter_update_sub.copy(&update);
		updateParams();
		updateLayout();
	}

	_manual_sub.update(&_manual);
	_attitude_sub.update(&_attitude);
	_angular_velocity_sub.update(&_angular_velocity);
	_status_sub.update(&_status);
	_land_sub.update(&_land);
	_armed_sub.update(&_armed);

	const hrt_abstime now = hrt_absolute_time();

	// ---------- 模式判定：只看 nav_state，不关心是谁切的 ----------
	_mode_active = (_status.nav_state == vehicle_status_s::NAVIGATION_STATE_FLIP_RECOVERY);

	if (!_mode_active) {
		_exit_requested = false;   // 退出模式后允许下一次进入时重新触发
	}

	// ---------- 数据有效性 ----------
	_attitude_valid = fresh(now, _attitude.timestamp_sample, 100_ms);
	_gyro_valid = fresh(now, _angular_velocity.timestamp_sample, 100_ms);
	for (unsigned i = 0; i < 3; ++i) { _gyro_valid = _gyro_valid && PX4_ISFINITE(_angular_velocity.xyz[i]); }

	float alignment = NAN;

	if (_attitude_valid) {
		_attitude_valid = flip_recovery::FlipPlanner::tiltDegrees(_attitude.q, alignment);
		_tilt = _attitude_valid ? math::degrees(acosf(math::constrain(alignment, -1.f, 1.f))) : NAN;

	} else {
		_tilt = NAN;
	}

	_rc_ready = _manual.valid && fresh(now, _manual.timestamp_sample, 1_s);

	// 可选二次确认：FLIP_RC_CONFIRM = 1..6 时必须把对应 aux 通道打到高位
	bool confirm_ok = true;

	if (_param_rc_confirm.get() > 0) {
		const float aux = auxInput();
		confirm_ok = _rc_ready && PX4_ISFINITE(aux) && aux > 0.5f;
	}

	// 有遥控在线时要求油门摇杆放到最低，避免用户一边推油门一边翻正
	bool throttle_ok = true;

	if (_rc_ready && PX4_ISFINITE(_manual.throttle)) {
		throttle_ok = _manual.throttle < -0.8f;
	}

	// ---------- 安全门限（任一不满足立即停） ----------
	// 注意：这里不看 failsafe —— 纯上位机触发时可能没有遥控，failsafe 会一直亮，
	// 但未解锁就飞不起来，安全性由 armed 状态保证。
	_safe = _attitude_valid
		&& _gyro_valid
		&& _layout.valid
		&& fresh(now, _land.timestamp, 2_s) && _land.landed
		&& fresh(now, _armed.timestamp, 2_s)
		&& !_armed.armed && !_armed.kill && !_armed.lockdown && !_armed.termination
		&& !_armed.in_esc_calibration_mode
		&& fresh(now, _status.timestamp, 2_s)
		&& _status.arming_state != vehicle_status_s::ARMING_STATE_ARMED
		&& confirm_ok
		&& throttle_ok;

	const auto previous = _planner.state();

	// 姿态被外部重置（重定位/切换估计器）时，放弃本次会话
	if ((_planner.state() == flip_recovery::FlipPlanner::State::Ramp
	     || _planner.state() == flip_recovery::FlipPlanner::State::Hold
	     || _planner.state() == flip_recovery::FlipPlanner::State::Coast)
	    && _quat_reset != _attitude.quat_reset_counter) {
		_planner.fail("attitude reset during recovery", flip_recovery::ReasonAttitudeReset);
	}

	if (_planner.state() == flip_recovery::FlipPlanner::State::Idle
	    || _planner.state() == flip_recovery::FlipPlanner::State::Done
	    || _planner.state() == flip_recovery::FlipPlanner::State::Failed) {
		_quat_reset = _attitude.quat_reset_counter;
	}

	// ---------- 参数 -> 规划器设置 ----------
	flip_recovery::Settings settings{};
	settings.entry_angle = _param_entry_ang.get();
	settings.stop_angle = _param_stop_ang.get();
	settings.thr_high = _param_thr_high.get();
	settings.thr_low = _param_thr_low.get();
	settings.low_mode = _param_low_mode.get();
	settings.slew = _param_slew.get();
	settings.max_time = _param_time.get();
	settings.min_progress = _param_min_prog.get();
	settings.progress_time = _param_prog_t.get();
	settings.strategy = _param_mode.get();

	// 参数合法性兜底
	const bool settings_valid = PX4_ISFINITE(settings.entry_angle) && settings.entry_angle > settings.stop_angle
				    && settings.entry_angle >= 60.f && settings.entry_angle <= 180.f
				    && PX4_ISFINITE(settings.stop_angle) && settings.stop_angle >= 0.f && settings.stop_angle <= 120.f
				    && PX4_ISFINITE(settings.thr_high) && settings.thr_high >= 0.02f
				    && settings.thr_high <= flip_recovery::HARD_MAX_THROTTLE
				    && PX4_ISFINITE(settings.thr_low) && settings.thr_low >= 0.f
				    && settings.thr_low <= flip_recovery::HARD_MAX_THROTTLE
				    && settings.low_mode >= 0 && settings.low_mode <= 3
				    && PX4_ISFINITE(settings.slew) && settings.slew >= 0.05f && settings.slew <= 2.f
				    && PX4_ISFINITE(settings.max_time) && settings.max_time >= 0.2f && settings.max_time <= 5.f
				    && settings.strategy >= 0 && settings.strategy <= 1
				    && PX4_ISFINITE(settings.min_progress) && settings.min_progress >= 0.f && settings.min_progress <= 30.f
				    && PX4_ISFINITE(settings.progress_time) && settings.progress_time >= 0.2f && settings.progress_time <= 2.f
				    && _param_rc_confirm.get() >= 0 && _param_rc_confirm.get() <= 6
				    && _param_exit_mode.get() >= 0 && _param_exit_mode.get() <= 8;

	flip_recovery::Input in{};
	in.now = now;
	in.enabled = _param_en.get() && settings_valid;
	in.safe = _safe;
	in.mode_active = _mode_active;
	in.gyro[0] = _angular_velocity.xyz[0];
	in.gyro[1] = _angular_velocity.xyz[1];
	in.gyro[2] = _angular_velocity.xyz[2];

	for (unsigned i = 0; i < 4; ++i) {
		in.q[i] = _attitude.q[i];
	}

	_plan = _planner.update(in, settings, _layout);

	if (previous != _planner.state()) {
		PX4_INFO("flip state %u: %s (tilt %.1f)", static_cast<unsigned>(_planner.state()), _planner.reason(),
			 (double)_tilt);

		// 给上位机的结果：只有 Done 才算成功，Failed 带原因码
		if (_planner.state() == flip_recovery::FlipPlanner::State::Ramp) {
			_session_start = now;
			_result = flip_recovery_status_s::RESULT_NONE;
			_duration = 0.f;

		} else if (_planner.state() == flip_recovery::FlipPlanner::State::Done) {
			_result = flip_recovery_status_s::RESULT_SUCCESS;
			_duration = (_session_start != 0 && now >= _session_start) ? (now - _session_start) * 1e-6f : 0.f;

		} else if (_planner.state() == flip_recovery::FlipPlanner::State::Failed) {
			_result = flip_recovery_status_s::RESULT_FAILED;
			_duration = (_session_start != 0 && now >= _session_start) ? (now - _session_start) * 1e-6f : 0.f;
		}
	}

	// 工作区切换时打印一次"谁在往哪推"，方便台架上直接核对
	if (_plan.zone != _last_zone) {
		_last_zone = _plan.zone;

		if (_plan.zone != flip_recovery::Zone::Off) {
			// 逐路 control 才是权威值；掩码只是它的符号摘要
			double thr_high = 0.0;

			for (unsigned i = 0; i < 4; ++i) {
				if (_plan.high_mask & (1u << i)) {
					thr_high = fabs((double)_plan.control[i]);
				}
			}

			double thr_low = 0.0;

			for (unsigned i = 0; i < 4; ++i) {
				if (_plan.low_mask & (1u << i)) {
					thr_low = fabs((double)_plan.control[i]);
				}
			}

			const bool high_normal = (_plan.fwd_mask & _plan.high_mask) != 0;
			const bool low_normal  = (_plan.fwd_mask & _plan.low_mask) != 0;

			// 高位在抬还是压：策略 A 的 90~180° 段高位是"抬"（Push::Up）
			const bool high_lifts = (_plan.zone == flip_recovery::Zone::Inverted) && (_param_mode.get() == 0);

			PX4_INFO("flip zone %u (tilt %.0f): HIGH(m%02x) %s @ %.2f %s | LOW(m%02x) %s @ %.2f",
				 static_cast<unsigned>(_plan.zone), (double)_tilt,
				 _plan.high_mask, high_normal ? "normal" : "reversed", thr_high,
				 high_lifts ? "[lifts]" : "[presses]",
				 _plan.low_mask, low_normal ? "normal" : "reversed", thr_low);

		} else {
			PX4_INFO("flip zone off (tilt %.0f)", (double)_tilt);
		}
	}

	// 翻正结束（成功或失败）且仍停留在该模式 -> 自动切回 FLIP_EXIT_MODE
	if (_mode_active && (!_exit_requested || now - _last_exit_request >= 1_s)
	    && (_planner.state() == flip_recovery::FlipPlanner::State::Done
		|| _planner.state() == flip_recovery::FlipPlanner::State::Failed)) {
		requestExitMode();
	}

	publishControl(_plan, _param_out_direct.get());
	publishStatus(_plan);
}

void FlipRecovery::requestExitMode()
{
	_exit_requested = true;
	_last_exit_request = hrt_absolute_time();

	uint8_t main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
	uint8_t sub_mode = 0;

	if (!modeFromParam(_param_exit_mode.get(), main_mode, sub_mode)) {
		main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
		sub_mode = 0;
	}

	vehicle_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	cmd.param1 = 1.f;   // VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED
	cmd.param2 = static_cast<float>(main_mode);
	cmd.param3 = static_cast<float>(sub_mode);
	cmd.target_system = _status.system_id;
	cmd.target_component = _status.component_id;
	cmd.source_system = _status.system_id;
	cmd.source_component = _status.component_id;
	_command_pub.publish(cmd);

	PX4_INFO("flip done, switching back to mode %" PRId32, _param_exit_mode.get());
}

void FlipRecovery::publishControl(const flip_recovery::Plan &plan, bool direct)
{
	// 1) 对外的控制请求（给 DShot 后端：逐路带符号油门，负值 = 需要反向）
	flip_recovery_control_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.session_id = _planner.session();
	// 只有在真有掩码时才请求 DRIVE；Coast/Done 阶段一律发 STOP，让后端恢复方向
	const bool driving = plan.active && (plan.fwd_mask || plan.rev_mask) && plan.zone != flip_recovery::Zone::Off;
	msg.state = plan.state;
	msg.action = driving ? flip_recovery_control_s::ACTION_DRIVE : flip_recovery_control_s::ACTION_STOP;
	msg.zone = static_cast<uint8_t>(plan.zone);
	msg.fwd_mask = plan.fwd_mask;
	msg.rev_mask = plan.rev_mask;
	msg.tilt_deg = plan.tilt_deg;
	msg.slew_rate = _param_slew.get();

	for (unsigned i = 0; i < 4; ++i) {
		msg.control[i] = plan.control[i];
	}

	_control_pub.publish(msg);

	// 2) 台架直出（FLIP_OUT_DIRECT=1）
	//    该模式下 control_allocator 已让位（allocation flag = false），不会抢这个话题
	//    负值表示反向推力，需要后端支持可逆输出；DShot 驱动在未解锁时仍会输出
	//    disarmed 值，所以真实桨转还需要驱动层改造（见 IMPLEMENTATION.md 第 6 节）
	if (direct) {
		actuator_motors_s m{};
		m.timestamp = msg.timestamp;
		m.timestamp_sample = msg.timestamp;

		for (unsigned i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
			m.control[i] = NAN;
		}

		for (unsigned i = 0; i < 4; ++i) {
			m.control[i] = plan.control[i];
		}

		_motors_pub.publish(m);
	}
}

// 给上位机的执行结果。上位机订阅本话题，等 result 从 NONE 变成 SUCCESS/FAILED
// 就可以发下一步（解锁、起飞、切模式…），不需要轮询 nav_state 或翻日志。
void FlipRecovery::publishStatus(const flip_recovery::Plan &plan)
{
	flip_recovery_status_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.session_id = _planner.session();
	msg.state = plan.state;
	msg.result = _result;
	msg.reason = plan.reason_code;
	msg.tilt_deg = plan.tilt_deg;
	msg.duration_s = _duration;
	msg.mode_active = _mode_active;
	_flip_status_pub.publish(msg);
}

int FlipRecovery::print_status()
{
	PX4_INFO("enabled: %s, mode active: %s, state %u: %s", _param_en.get() ? "yes" : "no",
		 _mode_active ? "yes" : "no",
		 static_cast<unsigned>(_planner.state()), _planner.reason());
	PX4_INFO("tilt: %.1f deg, entry %.1f, stop %.1f, strategy %" PRId32 " (%s)",
		 (double)_tilt, (double)_param_entry_ang.get(), (double)_param_stop_ang.get(), _param_mode.get(),
		 _param_mode.get() == 0 ? "flip over" : "press back");
	PX4_INFO("throttle: high %.2f, low %.2f (mode %" PRId32 "), slew %.2f/s, drive time %.1f s",
		 (double)_param_thr_high.get(), (double)_param_thr_low.get(), _param_low_mode.get(),
		 (double)_param_slew.get(), (double)_param_time.get());
	PX4_INFO("masks: fwd 0x%02x, rev 0x%02x, zone %u", _plan.fwd_mask, _plan.rev_mask,
		 static_cast<unsigned>(_plan.zone));
	PX4_INFO("safety: %s, RC confirm: ch %" PRId32 " %s, layout: %s",
		 _safe ? "OK" : "BLOCKED", _param_rc_confirm.get(), _rc_ready ? "ready" : "no RC",
		 _layout.valid ? "4 rotors" : "unsupported");
	PX4_INFO("exit mode param: %" PRId32, _param_exit_mode.get());

	if (_layout.valid) {
		for (unsigned i = 0; i < 4; ++i) {
			PX4_INFO("Motor %u: PX=%.3f PY=%.3f", i + 1, (double)_layout.px[i], (double)_layout.py[i]);
		}
	}

	return PX4_OK;
}

int FlipRecovery::task_spawn(int argc, char *argv[])
{
	FlipRecovery *instance = new FlipRecovery();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		instance->ScheduleOnInterval(20_ms);
		return PX4_OK;
	}

	return PX4_ERROR;
}

int FlipRecovery::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
Tilt flip recovery for quad-X (60-180 deg), driven by a dedicated flight mode.

This module does nothing on its own: it only becomes active while
vehicle_status.nav_state == NAVIGATION_STATE_FLIP_RECOVERY. That mode can be
entered from three places, all of which end up in the same nav_state:

  - RC switch: set a mode slot to "Flip Recovery" (COM_FLTMODE<n> = 17)
  - MAVLink:   SET_MODE / MAV_CMD_DO_SET_MODE with custom main mode 12
  - Console:   commander mode flip

Once active it uses the attitude quaternion to find the two lowest motors,
which define the roll axis (the edge resting on the ground). The high pair has
the largest lever arm and does all the work; the low pair sits on the axis, so
it only lifts slightly to unload the contact and reduce friction.

  - tilt  60.. 90 deg: high pair presses down (mode "press back" would end here)
  - tilt  90..180 deg: high pair lifts, rolling the frame over 180 deg
                       (FLIP_MODE=0, default) or keeps pressing down (FLIP_MODE=1)
  - tilt below the stop angle: cut throttle and let gravity settle it

FLIP_THR_HIGH / FLIP_THR_LOW set the two groups independently; FLIP_LOW_MODE
can force the low pair normal/reversed or turn it off for bench comparison.

The mode can never be armed (mode_req_prevent_arming), so this always runs on
the ground with the vehicle disarmed. When the recovery is done or has failed
the module sends VEHICLE_CMD_DO_SET_MODE to return to FLIP_EXIT_MODE.

Requires an output backend that can reverse motor direction (DShot command
20/21 or 3D throttle). FLIP_OUT_DIRECT=1 publishes actuator_motors directly for
bench testing.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("flip_recovery", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return PX4_OK;
}

extern "C" __EXPORT int flip_recovery_main(int argc, char *argv[])
{
	return FlipRecovery::main(argc, argv);
}
