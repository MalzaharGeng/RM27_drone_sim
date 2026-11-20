/****************************************************************************
 * flip_recovery —— 倾角翻正（75°~180°）
 *
 * 触发方式（v2，模式驱动）：
 *   1) 遥控器拨杆：把某个模式槽位 COM_FLTMODE<n> 设为 "Flip Recovery"（值 17）
 *   2) 上位机话题：vehicle_command 发 VEHICLE_CMD_DO_SET_MODE
 *      (param1=1, param2=PX4_CUSTOM_MAIN_MODE_FLIP_RECOVERY=12)
 *   3) 上位机 MAVLink：SET_MODE / MAV_CMD_DO_SET_MODE（与 2 同一条链路）
 *   4) 命令行：commander mode flip
 *   四条路径最终都落到 vehicle_status.nav_state == NAVIGATION_STATE_FLIP_RECOVERY，
 *   本模块只订阅 vehicle_status，不关心是谁切的。
 *
 * 安全模型：
 *   - 只允许未解锁（disarmed）时出力；一旦 armed 立即停
 *   - 进入该模式本身禁止解锁（mode_req_prevent_arming）
 *   - 任何安全条件丢失立即停止，且必须退出再进入模式才能重试
 *   - 完成后自动切回 FLIP_EXIT_MODE
 ****************************************************************************/
#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/flip_recovery_control.h>
#include <uORB/topics/flip_recovery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

#include <flip_recovery/FlipPlanner.hpp>

class FlipRecovery : public ModuleBase<FlipRecovery>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	FlipRecovery();
	~FlipRecovery() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

private:
	void Run() override;
	void updateLayout();
	void publishControl(const flip_recovery::Plan &plan, bool dry_run);
	void publishStatus(const flip_recovery::Plan &plan);
	void requestExitMode();
	float auxInput() const;

	static bool fresh(hrt_abstime now, hrt_abstime timestamp, hrt_abstime timeout)
	{
		return timestamp != 0 && now >= timestamp && now - timestamp <= timeout;
	}

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _manual_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _land_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};

	uORB::Publication<flip_recovery_control_s> _control_pub{ORB_ID(flip_recovery_control)};
	uORB::Publication<flip_recovery_status_s> _flip_status_pub{ORB_ID(flip_recovery_status)};
	uORB::Publication<actuator_motors_s> _motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<vehicle_command_s> _command_pub{ORB_ID(vehicle_command)};

	manual_control_setpoint_s _manual{};
	vehicle_attitude_s _attitude{};
	vehicle_angular_velocity_s _angular_velocity{};
	vehicle_status_s _status{};
	vehicle_land_detected_s _land{};
	actuator_armed_s _armed{};

	flip_recovery::FlipPlanner _planner;
	flip_recovery::Layout _layout{};
	flip_recovery::Plan _plan{};

	bool _mode_active{false};      // nav_state == FLIP_RECOVERY
	bool _attitude_valid{false};
	bool _gyro_valid{false};
	bool _rc_ready{false};
	bool _safe{false};
	hrt_abstime _last_exit_request{0};
	bool _exit_requested{false};   // Done/Failed 后已发过切回命令
	float _tilt{NAN};
	uint8_t _quat_reset{0};
	flip_recovery::Zone _last_zone{flip_recovery::Zone::Off};

	// 给上位机的执行结果（见 flip_recovery_status.msg）
	uint64_t _session_start{0};
	uint8_t _result{flip_recovery_status_s::RESULT_NONE};
	float _duration{0.f};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::FLIP_EN>)         _param_en,
		(ParamInt<px4::params::FLIP_RC_CONFIRM>)  _param_rc_confirm,
		(ParamInt<px4::params::FLIP_EXIT_MODE>)   _param_exit_mode,
		(ParamInt<px4::params::FLIP_MODE>)        _param_mode,
		(ParamFloat<px4::params::FLIP_ENTRY_ANG>) _param_entry_ang,
		(ParamFloat<px4::params::FLIP_STOP_ANG>)  _param_stop_ang,
		(ParamFloat<px4::params::FLIP_THR_HIGH>)  _param_thr_high,
		(ParamFloat<px4::params::FLIP_THR_LOW>)   _param_thr_low,
		(ParamInt<px4::params::FLIP_LOW_MODE>)    _param_low_mode,
		(ParamFloat<px4::params::FLIP_SLEW>)      _param_slew,
		(ParamFloat<px4::params::FLIP_TIME>)      _param_time,
		(ParamFloat<px4::params::FLIP_MIN_PROG>)  _param_min_prog,
		(ParamFloat<px4::params::FLIP_PROG_T>)    _param_prog_t,
		(ParamBool<px4::params::FLIP_OUT_DIRECT>) _param_out_direct
	)
};
