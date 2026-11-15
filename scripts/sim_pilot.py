#!/usr/bin/env python3
"""SITL virtual pilot + GCS heartbeat.

[SIM-ONLY] 本文件属于仿真专属：它替代的是「坐在遥控器前的人」，不是实机上
运行的代码，不应迁移。可迁移的只有它体现的操作顺序约定——先等位置估计可用
再拨 Position 档、被飞控拒绝时重新拨档——实机由飞行员照同样顺序操作。

There is no physical RC in Gazebo. This process injects RC_CHANNELS_OVERRIDE
into PX4's normal MAVLink (GCS) instance on udp 14550. PX4's rc_update then
produces manual_control_setpoint with data_source=SOURCE_RC and the real
manual_control_switches/mode_slot, so the flight controller state follows the
switch positions; nothing here forges those uORB/DDS topics.

It sends only HEARTBEAT and RC_CHANNELS_OVERRIDE. It never sends COMMAND_LONG
(no arm, mode, takeoff or land command). Arming and mode changes are performed
with the simulated RC arm switch (ch5) and mode switch (ch6).
"""
import argparse
import csv
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pymavlink import mavutil
from diag_exit_policy import diagnostic_exit_due
# [NP-01 FIX6-corrected] Height feedback comes from the DDS
# VehicleLocalPosition message (validity + reset counters), NOT from MAVLink
# LOCAL_POSITION_NED which drops all validity fields. See
# newpkg_20261115/evidence/np01_height_feedback_contract.md
from takeoff_altitude_policy import (DEFAULT_CONFIG as TAKEOFF_CFG,
                                     HeightSample, advance_takeoff,
                                     diagnostics as height_diagnostics)
# [NP-01 FIX20] EXCITE 高度参考在 PX4 局部 NED z-reset 后的连续性契约。
# 纯函数、不读真值、不改任何门限；见 newpkg_fix20/excite_height_reference.py
from excite_height_reference import (
    capture_excite_height_ref, advance_excite_height_reference)
try:
    from px4_msgs.msg import VehicleLocalPosition
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    _HAS_DDS_LP = True
except ImportError:            # fail-closed: no DDS height -> no takeoff success
    VehicleLocalPosition = None
    _HAS_DDS_LP = False

# [NP-01 FIX9] Unlocked preflight health gate. EstimatorStatusFlags is the only
# message that reports WHICH fusion source the EKF actually uses (cs_ev_hgt,
# cs_baro_hgt, cs_gps_hgt, cs_ev_pos ...), which is stronger evidence than the
# parameter file. Missing message type degrades to "unverifiable", never to
# "assume healthy".
try:
    from px4_msgs.msg import EstimatorStatusFlags
    _HAS_EST = True
except ImportError:
    EstimatorStatusFlags = None
    _HAS_EST = False

if _HAS_DDS_LP:
    _LP_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

    def _lp_topic():
        v = getattr(VehicleLocalPosition, "MESSAGE_VERSION", 0)
        return "/fmu/out/vehicle_local_position" + ("_v%d" % v if v else "")

NONE16 = 65535

# POSITION 档回中的坐标契约（纯函数，便于单测）：
#   输入 home/pos 都是 PX4 LOCAL_POSITION_NED（NED 世界系，米），
#   yaw 是同一时刻的机头偏航（弧度，NED 中从北顺时针）。
#   输出 ch1/ch2 是 RC 覆盖值：POSCTL 下 roll/pitch 作用于机体系，
#   所以必须先把世界系误差转到机体系，再线性映射成摇杆量。
# 约束：
#   * 目标固定，不能随 pos 变化；
#   * 输入无效（None）时回中，不沿用上一拍指令；
#   * 死区内回中；
#   * 增益 80 us/m、限幅 ±80 us（≈0.8 m/s）与旧代码一致。
HOME_DEADZONE_M = 0.35
HOME_STICK_GAIN = 80.0
HOME_STICK_LIMIT = 80

# [NP-01 FIX16] Readiness freshness contract.
# `navigation_ready` is delivered as a latched boolean on a best-effort topic.
# A latched True that stops updating is NOT evidence that the estimate is
# still valid (round 14 kept True after the stream would have died). Every
# consumer of readiness in this file therefore goes through this helper, which
# requires a message received within READY_STALE_S on the HOST MONOTONIC clock.
READY_STALE_S = 0.8


def ready_is_fresh(vio_ready, rx_mono, now, stale_s=READY_STALE_S):
    """True only for a FRESH, positive readiness message."""
    return bool(vio_ready) and rx_mono is not None and (now - rx_mono) <= stale_s


def settle_exit(elapsed, fresh_ready, xy_valid, vxy_valid, min_s, max_s):
    """[NP-01 FIX16] When may INIT_SETTLE hand over to the Position detent?

    The window exists to (a) leave the RC step behind, (b) require fresh
    navigation readiness, (c) require PX4's OWN position validity. It is NOT a
    fixed sleep: it ends as soon as all three hold, and never later than max_s.
    Truth is not an input. No threshold is relaxed here.
    """
    if elapsed >= max_s:
        return True, "max_wait"
    if elapsed >= min_s and fresh_ready and xy_valid and vxy_valid:
        return True, "px4_position_valid"
    return False, ""



def sticks_for_home(home, pos, yaw,
                    deadzone=HOME_DEADZONE_M,
                    gain=HOME_STICK_GAIN,
                    limit=HOME_STICK_LIMIT):
    """世界系回中误差 -> 机体系 RC 摇杆。任何输入缺失都回中。"""
    if home is None or pos is None or yaw is None:
        return 1500, 1500
    ex = home[0] - pos[0]
    ey = home[1] - pos[1]
    if math.hypot(ex, ey) <= deadzone:
        return 1500, 1500
    c, s = math.cos(yaw), math.sin(yaw)
    # body +x = (c, s)；body +y = (-s, c)，即机头右侧。
    fwd = ex * c + ey * s
    right = -ex * s + ey * c
    ch1 = int(max(1500 - limit, min(1500 + limit, 1500 + right * gain)))
    ch2 = int(max(1500 - limit, min(1500 + limit, 1500 + fwd * gain)))
    return ch1, ch2


class Pilot(Node):
    def __init__(self, args):
        super().__init__("sim_pilot")
        self.args = args
        self.bt = {}
        self.events = []
        self.create_subscription(String, args.topic_prefix + "/navigation/task_status",
                                 self.on_status, 1)
        self.create_subscription(String, args.topic_prefix + "/navigation/task_event",
                                 self.on_event, 10)
        # VIO 导航是否真的就绪。飞行员在实机上也是先看到位置估计可用，才会从
        # Altitude 拨到 Position；PX4 在没有位置时拒绝进入 POSCTL，档位停在原处
        # 也不会自己重试，这一点仿真与实机一致。
        self.vio_ready = False
        self.vio_reason = ""
        self.await_logged = False
        self.create_subscription(String, args.topic_prefix + "/vio/navigation_health",
                                 self.on_vio_health, 1)
        self.mav = mavutil.mavlink_connection("udpin:0.0.0.0:" + str(args.gcs_port),
                                              source_system=255, source_component=190)
        self.get_logger().info("waiting for PX4 heartbeat on udp " + str(args.gcs_port))
        self.mav.wait_heartbeat(timeout=60)
        self.mav.target_system = 1
        self.mav.target_component = 1
        self.get_logger().info("PX4 heartbeat received")
        self.reset = time.monotonic()
        self.state = "INIT"
        self.state_since = self.reset
        self.armed = False
        self.nav = None
        self.intended = None
        self.z = None
        # [NP-01 FIX6-corrected] DDS height feedback and takeoff state.
        self.height = None            # latest HeightSample (or None)
        self.takeoff_ref = None       # HeightReference, captured on a sane sample
        self.takeoff_recaptures = 0
        self.height_status = ""
        self.height_reason = ""
        self.height_seen = 0
        if _HAS_DDS_LP:
            self.create_subscription(VehicleLocalPosition, _lp_topic(),
                                     self.on_local_position, _LP_QOS)
        # [NP-01 FIX9] Preflight health accumulators. Read-only.
        self.est = None
        self.est_seen = 0
        self.pf_z_valid_seen = 0
        self.pf_vz_valid_seen = 0
        self.pf_xy_valid_seen = 0
        self.pf_samples = 0
        self.pf_z_min = None
        self.pf_z_max = None
        self.pf_vz_absmax = 0.0
        self.pf_nonfinite = 0
        self.pf_zrc_first = None
        self.pf_zrc_last = None
        self.pf_zrc_changes = 0
        self.pf_cs = {}
        if _HAS_EST:
            _v = getattr(EstimatorStatusFlags, "MESSAGE_VERSION", 0)
            _t = "/fmu/out/estimator_status_flags" + ("_v%d" % _v if _v else "")
            self.create_subscription(EstimatorStatusFlags, _t,
                                     self.on_estimator_flags, _LP_QOS)
        # 反馈位置：PX4 LOCAL_POSITION_NED（与行为树 vehicle_local_position 同域）。
        self.hx = None
        self.hy = None
        # 回中目标：必须在同一反馈坐标系里「捕获一次并固定」，不能每帧等于当前位置。
        # 目标不是 PX4 本地原点 (0,0)：外视觉初始化在飞行中完成，原点是 VIO 被接受
        # 时的位置，不是起飞点（103614 实测两者相差约 12 m）。
        self.home = None
        self.home_pending = True          # 下一个有效本地样本到来时捕获
        self.home_reset = None            # 捕获时的 px4_xy_reset_counter
        self.home_logged = False
        # 机体系偏航：POSITION 档摇杆是机体系，位置误差是 NED 世界系，必须旋转。
        # PX4 在 GCS (Normal) 链路以 15 Hz 发 ATTITUDE；超过 2 s 未更新即视为
        # 航向无效并回中（fail-closed），避免用陈旧偏航把误差打错轴。
        self.yaw = None
        self.yaw_time = None
        self.slot = args.slot_altitude
        # Position 档被 PX4 拒绝时重新拨档的状态（见 POSITION 分支）。
        self.detent_retries = 0
        self.detent_retry_pending = False
        self.detent_retry_since = 0.0
        self.ch1 = self.ch2 = 1500
        self.ch3 = 1500
        self.ch4 = 1500
        self.ch5 = 1000
        self.control_since = None
        self.completed_since = None
        self.takeover_sent = False
        # 授权链要求「OFFBOARD READY 之后重新拨档」，且预热窗口只有 0.2 s。
        # 第一次没抢到窗口时真人飞手会拨回 Position 再拨一次；仿真飞手照做。
        self.offboard_cycles = 0
        self.offboard_cycle_low_since = None
        self.offboard_cycle_wait_since = 0.0
        self.excite_logged = False
        self.excite_stop_since = None
        # [NP-01 FIX12] Bounded, state-driven excitation (opt-in). Every input
        # below is NORMAL FLIGHT FEEDBACK (VIO health, PX4 local position,
        # MAVLink ATTITUDE). No truth value is ever read by the pilot.
        self.excite_ctl = (args.excite_controller == "bounded")
        self.excite_ref = None
        # [NP-01 FIX20] 与参考同时捕获的坐标系身份 + 逐帧源时间（乱序检测）
        self.excite_height_ref = None        # ExciteHeightRef
        self.excite_last_sample_us = None   # 上一拍 source sample_us
        self.excite_reset_trace = []        # 仅证据，不参与判定
        self.excite_ready_since = None
        self.excite_ready_streak_s = 0.0
        self.excite_ever_ready = False
        self.excite_stop_reason = ""
        self.excite_constraint = ""
        self.excite_rc_prev = None
        self.excite_rc_rate = 0.0
        self.excite_body_rate_dps = 0.0
        self.excite_tilt_deg = 0.0
        self.excite_disp_xy_m = 0.0
        self.excite_feedback_age_ms = -1.0
        self.vio_rx_mono = None
        self.vio_seq = 0
        self.body_rate = None
        self.body_tilt = None
        self.body_att_rx_mono = None
        # [NP01-FIX15] 姿态变化率估计：MAVLink ATTITUDE 只有 15 Hz，
        # 无法直接分辨 33 ms 帧间隔；用相邻两条 ATTITUDE 的角度差除以时间差
        # 得到角速率，再乘 0.033 s 折算成"帧间倾角变化"，最后换算成
        # ORB 门限使用的 |ΔavgA| ≈ g·sin(Δθ)。全程只用正常飞控反馈，不读真值。
        self.att_prev = None          # (mono, tilt_deg_signed, axis)
        self.att_rate_dps = 0.0
        self.att_rate_window = []     # [(mono, rate_dps)] 最近 0.4 s
        self.est_davg = 0.0
        self.est_davg_best = 0.0
        self.excite_fast_deg = None   # 自适应幅值（deg），首次使用时初始化
        self.excite_gate_seen = False
        self.warn_reacted = False
        self.last_hb = 0.0
        self.last_send = 0.0
        self.counter = 0
        self.csv = open(args.log, "w", newline="")
        self.writer = csv.writer(self.csv)
        self.writer.writerow(["wall_us", "mono_s", "state", "bt_phase", "bt_reason",
                              "offboard_ready", "task_completed", "control_active",
                              "armed", "nav", "intended", "z", "slot", "ch1", "ch2",
                              "ch3", "ch5",
                              # 有界诊断：回中目标/反馈/误差/偏航/重置计数
                              "home_x", "home_y", "err_x", "err_y", "err_norm",
                              "yaw", "vio_ready", "xy_reset",
                              # [NP-01 FIX6-corrected] height feedback contract
                              "h_z", "h_vz", "h_z_valid", "h_vz_valid",
                              "h_xy_valid", "h_vxy_valid", "h_zrc", "h_vzrc",
                              "h_delta_z", "h_sample_us", "h_rx_age_ms",
                              "h_reject", "h_status", "h_recaptures",
                              # [NP-01 FIX12] excitation observations.
                              # ex_rc_rate_us_s  = RC COMMAND derivative, not motion
                              # ex_body_rate_dps = MAVLink ATTITUDE rate, real motion
                              # ex_disp_xy_m     = PX4 LOCAL_POSITION_NED displacement
                              # truth is NEVER a column here; see truth_safety_watchdog.py
                              "ex_controller", "ex_bounded", "ex_stop_reason",
                              "ex_rc_rate_us_s", "ex_body_rate_dps", "ex_tilt_deg",
                              "ex_disp_xy_m", "ex_ready_streak_s",
                              "ex_feedback_age_ms", "ex_constraint"])
        self.get_logger().info("state INIT")

    def on_local_position(self, msg):
        # SOURCE time (msg.timestamp_sample) is recorded only. Freshness is
        # judged on the host monotonic RECEIVE time; they are never mixed.
        self.height = HeightSample(
            z=float(msg.z), vz=float(msg.vz),
            z_valid=bool(msg.z_valid), v_z_valid=bool(msg.v_z_valid),
            xy_valid=bool(msg.xy_valid), v_xy_valid=bool(msg.v_xy_valid),
            z_reset_counter=int(msg.z_reset_counter),
            vz_reset_counter=int(getattr(msg, "vz_reset_counter", 0)),
            delta_z=float(getattr(msg, "delta_z", 0.0)),
            sample_us=int(msg.timestamp_sample),
            rx_mono=time.monotonic())
        self.height_seen += 1

    def on_estimator_flags(self, msg):
        # [NP-01 FIX9] Which fusion source is ACTUALLY active. Recorded, not
        # gated: baro-backed height is the current reality at takeoff.
        self.est = msg
        self.est_seen += 1
        for name in ("cs_tilt_align", "cs_yaw_align", "cs_ev_pos", "cs_ev_vel",
                     "cs_ev_hgt", "cs_ev_yaw", "cs_baro_hgt", "cs_gps_hgt",
                     "cs_rng_hgt", "cs_mag_hdg", "cs_in_air"):
            self.pf_cs[name] = bool(getattr(msg, name, False))

    def preflight_accumulate(self):
        """Fold the latest height sample into the preflight health counters."""
        h = self.height
        if h is None:
            return
        self.pf_samples += 1
        vals = (h.z, h.vz, h.delta_z)
        if not all(math.isfinite(v) for v in vals):
            self.pf_nonfinite += 1
            return
        if h.z_valid:
            self.pf_z_valid_seen += 1
        if h.v_z_valid:
            self.pf_vz_valid_seen += 1
        if h.xy_valid:
            self.pf_xy_valid_seen += 1
        self.pf_z_min = h.z if self.pf_z_min is None else min(self.pf_z_min, h.z)
        self.pf_z_max = h.z if self.pf_z_max is None else max(self.pf_z_max, h.z)
        self.pf_vz_absmax = max(self.pf_vz_absmax, abs(h.vz))
        if self.pf_zrc_first is None:
            self.pf_zrc_first = h.z_reset_counter
        elif h.z_reset_counter != self.pf_zrc_last:
            self.pf_zrc_changes += 1
        self.pf_zrc_last = h.z_reset_counter

    def preflight_report(self):
        """Return (ok, reason, summary). Bounded, read-only, no unlock.

        Thresholds here are HEALTH bounds for a disarmed vehicle on the ground,
        NOT control limits and NOT acceptance criteria.
        """
        summary = dict(
            samples=self.pf_samples, est_seen=self.est_seen,
            z_valid=self.pf_z_valid_seen, vz_valid=self.pf_vz_valid_seen,
            xy_valid=self.pf_xy_valid_seen, nonfinite=self.pf_nonfinite,
            z_min=self.pf_z_min, z_max=self.pf_z_max,
            vz_absmax=self.pf_vz_absmax,
            z_reset_counter=(self.pf_zrc_first, self.pf_zrc_last),
            z_reset_changes=self.pf_zrc_changes,
            fusion=dict(self.pf_cs), armed=self.armed, vio_ready=self.vio_ready)
        if self.armed:
            return False, "armed_during_preflight", summary
        if self.pf_samples < 10:
            return False, "no_height_feedback(%d)" % self.pf_samples, summary
        if self.pf_nonfinite:
            return False, "non_finite_height(%d)" % self.pf_nonfinite, summary
        if self.pf_z_valid_seen == 0:
            return False, "z_never_valid", summary
        if self.pf_vz_valid_seen == 0:
            return False, "vz_never_valid", summary
        if self.pf_z_min is None or self.pf_z_max is None:
            return False, "no_finite_z", summary
        # A disarmed vehicle on the ground: |z| must stay small and vz must not
        # run away. 3 m / 2 m/s are sanity bounds, far outside any real motion.
        if max(abs(self.pf_z_min), abs(self.pf_z_max)) > 3.0:
            return False, "z_diverged(%.2f..%.2f)" % (self.pf_z_min, self.pf_z_max), summary
        if self.pf_vz_absmax > 2.0:
            return False, "vz_diverged(%.2f)" % self.pf_vz_absmax, summary
        if self.pf_zrc_changes > 3:
            return False, "z_reset_churn(%d)" % self.pf_zrc_changes, summary
        if self.pf_zrc_first is not None and self.pf_zrc_first == 0:
            return False, "z_reset_counter_never_started", summary
        return True, "healthy", summary

    def on_status(self, m):
        try:
            self.bt = json.loads(m.data)
        except (ValueError, TypeError):
            pass

    def on_vio_health(self, m):
        try:
            p = json.loads(m.data)
        except (ValueError, TypeError):
            return
        self.vio_ready = p.get("navigation_ready") is True
        self.vio_reason = str(p.get("reason", ""))
        # [NP-01 FIX12] Receive time on HOST MONOTONIC. A sticky boolean is
        # never accepted as "fresh feedback" without this.
        self.vio_rx_mono = time.monotonic()
        self.vio_seq += 1
        # 本地坐标系被重置后，旧锚点在新坐标里没有意义：丢弃并在下一个有效
        # 本地样本重新捕获。这里只处理 xy 重置；heading 重置不改变水平回中。
        xy = p.get("px4_xy_reset_counter")
        if xy is not None and xy != self.home_reset:
            if self.home is not None:
                self.get_logger().warning(
                    "PX4 local frame reset xy_reset_counter %s -> %s; dropping home anchor"
                    % (self.home_reset, xy))
                self.home = None
            self.home_reset = xy
            self.home_pending = True

    def on_event(self, m):
        try:
            p = json.loads(m.data)
        except (ValueError, TypeError):
            return
        self.events.append(dict(wall_us=time.time_ns() // 1000, **p))
        self.get_logger().info("BT EVENT " + json.dumps(p, ensure_ascii=False))

    def set_state(self, state):
        # [NP-01 FIX6-corrected] Do NOT presume a height reference: it is
        # captured later from the first SANE DDS sample. A stale/zero/diverged
        # value at state entry must never become the takeoff datum.
        if state == "TAKEOFF":
            self.takeoff_ref = None
            self.takeoff_recaptures = 0
            self.height_status = ""
            self.height_reason = ""
        if state == "EXCITE" and self.excite_ctl:
            # [NP-01 FIX12] Bounded-excite state is strictly per-entry.
            self.excite_ref = None
            # [NP-01 FIX20] 参考与坐标系身份同时重置（不重置计时/故障/streak）
            self.excite_height_ref = None
            self.excite_last_sample_us = None
            self.excite_ready_since = None
            self.excite_ready_streak_s = 0.0
            self.excite_ever_ready = False
            self.excite_stop_reason = ""
            self.excite_constraint = ""
            self.excite_rc_prev = None
            self.excite_rc_rate = 0.0
            self.excite_body_rate_dps = 0.0
            self.excite_tilt_deg = 0.0
            self.excite_disp_xy_m = 0.0
            self.excite_feedback_age_ms = -1.0
            self.excite_stop_since = None
            self.get_logger().info(
                "[NP01_FIX12] bounded excitation active: open-loop RC only, "
                "effective fresh VIO feedback drives the stop, truth NOT used; "
                "legacy --excite-no-early-stop is ignored by design")
        self.state = state
        self.state_since = time.monotonic()
        self.get_logger().info("state -> " + state)

    def send_rc(self, now):
        if now - self.last_send < 0.02:
            return
        self.last_send = now
        vals = [self.ch1, self.ch2, self.ch3, self.ch4, self.ch5, self.slot]
        # 当前 pymavlink 的 MAVLink 1 RC_CHANNELS_OVERRIDE 只有 8 路，
        # PX4 也只配置了 RC_CHAN_CNT=8；多发的通道会被拒绝。
        vals += [NONE16] * (8 - len(vals))
        self.mav.mav.rc_channels_override_send(1, 1, *vals)
        if now - self.last_hb >= 1.0:
            self.last_hb = now
            self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def drain(self):
        while True:
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                break
            t = msg.get_type()
            if t == "HEARTBEAT" and msg.get_srcSystem() == 1:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.nav = msg.custom_mode
            elif t == "LOCAL_POSITION_NED":
                self.z = float(msg.z)
                self.hx = float(msg.x)
                self.hy = float(msg.y)
                # 只有导航就绪后的第一个有效样本才能作为回中锚点：此前
                # EKF 可能还没有外部位置，输出的是未初始化值。
                # [NP-01 FIX16] 就绪必须是新鲜消息（不是粘住的 True）。
                if (self.home_pending
                        and ready_is_fresh(self.vio_ready, self.vio_rx_mono,
                                           time.monotonic(), self.args.ready_stale_s)):
                    self.home = (self.hx, self.hy)
                    self.home_pending = False
                    self.get_logger().info(
                        "home anchor captured in PX4 local frame: (%.3f, %.3f) xy_reset=%s"
                        % (self.hx, self.hy, self.home_reset))
            elif t == "ATTITUDE":
                # POSITION 档的 roll/pitch 是机体系指令，回中误差在 NED 世界系，
                # 必须用当前机头偏航把误差旋到机体系。
                self.yaw = float(msg.yaw)
                self.yaw_time = time.monotonic()
                # [NP-01 FIX12] Real body rates / tilt from the SAME normal
                # MAVLink stream. These are NOT truth and NOT RC commands.
                try:
                    rates = (float(msg.rollspeed), float(msg.pitchspeed),
                             float(msg.yawspeed))
                    tilt = (float(msg.roll), float(msg.pitch))
                    if all(math.isfinite(v) for v in rates + tilt):
                        self.body_rate = rates
                        self.body_tilt = tilt
                        now_att = time.monotonic()
                        # [NP01-FIX15] 姿态变化率 → 帧间倾角变化 → |ΔavgA|。
                        # ATTITUDE 只有 15 Hz，直接看 33 ms 帧间隔不可能；
                        # 用 (Δθ/Δt)·0.033 s 折算，平方波换向段角速率最大，
                        # 正好是 ORB 判据最关心的时刻。
                        if self.att_prev is not None:
                            p_mono, p_roll, p_pitch = self.att_prev
                            dt_att = now_att - p_mono
                            if 1e-3 < dt_att < 0.5:
                                r_roll = (tilt[0] - p_roll) / dt_att
                                r_pitch = (tilt[1] - p_pitch) / dt_att
                                rate_dps = max(abs(r_roll), abs(r_pitch)) * 180.0 / math.pi
                                self.att_rate_dps = rate_dps
                                self.att_rate_window.append((now_att, rate_dps))
                        self.att_prev = (now_att, tilt[0], tilt[1])
                        self.body_att_rx_mono = now_att
                except (AttributeError, TypeError, ValueError):
                    pass

    def log(self, now):
        def nz(v):
            return "" if v is None else round(v, 4)
        # 目标、反馈、误差和摇杆放在同一行，便于逐拍核对坐标契约。
        if self.home is not None and self.hx is not None and self.hy is not None:
            err_x, err_y = self.home[0] - self.hx, self.home[1] - self.hy
            err_n = math.hypot(err_x, err_y)
        else:
            err_x = err_y = err_n = None
        hd = height_diagnostics(self.height, now, TAKEOFF_CFG)
        hz = "" if self.height is None else round(self.height.z, 4)
        hvz = "" if self.height is None else round(self.height.vz, 4)
        hs = "" if self.height is None else self.height.sample_us
        self.writer.writerow([time.time_ns() // 1000, now, self.state,
                              self.bt.get("phase", ""), self.bt.get("reason", ""),
                              self.bt.get("offboard_ready", ""),
                              self.bt.get("task_completed", ""),
                              self.bt.get("control_active", ""),
                              self.armed, self.nav, self.intended, self.z, self.slot,
                              self.ch1, self.ch2, self.ch3, self.ch5,
                              nz(self.home[0] if self.home else None),
                              nz(self.home[1] if self.home else None),
                              nz(err_x), nz(err_y), nz(err_n),
                              nz(self.yaw), int(self.vio_ready), self.home_reset,
                              hz, hvz, hd["z_valid"], hd["v_z_valid"],
                              hd["xy_valid"], hd["v_xy_valid"],
                              hd["z_reset_counter"], hd["vz_reset_counter"],
                              hd["delta_z"], hs, hd["h_rx_age_ms"],
                              hd["h_reject"], self.height_status,
                              self.takeoff_recaptures,
                              self.args.excite_controller, int(self.excite_ctl),
                              self.excite_stop_reason,
                              round(self.excite_rc_rate, 1),
                              round(self.excite_body_rate_dps, 1),
                              round(self.excite_tilt_deg, 1),
                              round(self.excite_disp_xy_m, 4),
                              round(self.excite_ready_streak_s, 3),
                              round(self.excite_feedback_age_ms, 1),
                              self.excite_constraint])
        self.csv.flush()

    def _write_ack(self, action):
        """[NP-01 FIX16] Explicit receipt/processing confirmation for a warning.

        The supervisor writes a WARN marker; this file is the pilot's ACK. It
        carries the HOST MONOTONIC time of the moment the warning was handled,
        so the supervisor can report a real end-to-end latency instead of
        assuming the file was noticed. No truth VALUE is read or written here.
        """
        path = self.args.ack_marker
        if not path:
            return
        payload = dict(
            ack_mono_us=time.monotonic_ns() // 1000,
            state_before=self.state, action=action,
            vio_ready=bool(self.vio_ready),
            vio_age_ms=((time.monotonic() - self.vio_rx_mono) * 1000.0
                        if self.vio_rx_mono else -1.0),
            armed=bool(self.armed), slot=self.slot)
        try:
            tmp = path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump(payload, fh, ensure_ascii=False)
            os.replace(tmp, path)
        except OSError as exc:
            self.get_logger().warning("warning ACK write failed: %r" % (exc,))

    def _emit_reset_event(self, ev):
        """[NP-01 FIX20] 把 reset 事件原子追加到本轮目录，供离线核对。

        纯证据：任何写盘失败都不影响飞行判定，也不改变任何门限。
        """
        self.excite_reset_trace.append(ev)
        try:
            path = os.path.join(os.path.dirname(os.path.abspath(self.args.log)),
                                "excite_reset_events.jsonl")
            with open(path, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(ev, ensure_ascii=False, sort_keys=True) + "\n")
                fh.flush()
        except (OSError, AttributeError, TypeError, ValueError):
            pass

    def _excite_stop(self, reason, now, elapsed, fatal):
        """Record a bounded-excitation stop. Observation + decision only."""
        self.excite_stop_reason = reason
        if fatal:
            self.excite_constraint = reason
        # [NP-01 FIX16] 这条日志原来每 5 ms 打一次（第 14 轮 0.27 s 内打了 47 行），
        # 既污染证据又增加负载。只在原因变化或每 0.5 s 打一次。
        _now = time.monotonic()
        _key = (reason, bool(fatal))
        if (_key != getattr(self, "_stop_log_key", None)
                or _now - getattr(self, "_stop_log_t", 0.0) >= 0.5):
            self._stop_log_key = _key
            self._stop_log_t = _now
            self.get_logger().info(
                "[NP01_FIX12] stop t=%.2fs reason=%s fatal=%s "
            "rc_cmd_rate=%.1fus/s body_rate=%.1fdeg/s tilt=%.1fdeg "
            "disp_xy=%.3fm vio_ready=%s streak=%.2fs feedback_age=%.1fms "
            "| est_davg=%.2f(best %.2f, target %.2f) fast_deg=%.1f att_rate=%.0fdps"
                % (elapsed, reason, fatal, self.excite_rc_rate,
                   self.excite_body_rate_dps, self.excite_tilt_deg,
                   self.excite_disp_xy_m, self.vio_ready,
                   self.excite_ready_streak_s, self.excite_feedback_age_ms,
                   self.est_davg, self.est_davg_best, self.args.excite_target_davg,
                   self.excite_fast_deg if self.excite_fast_deg is not None else -1.0,
                   self.att_rate_dps))
        return True, fatal

    def bounded_excite_decision(self, now, elapsed):
        """[NP-01 FIX12] Bounded, state-driven excitation stop.

        Contract:
          * Inputs are ONLY normal flight feedback: /vio/navigation_health
            (navigation_ready + host-monotonic receive time), PX4
            LOCAL_POSITION_NED x/y, VehicleLocalPosition height + validity,
            and MAVLink ATTITUDE roll/pitch/rates.
          * NO truth value is read here. Truth is used only by the independent
            run-level watchdog (truth_safety_watchdog.py) for monitoring and
            out-of-envelope termination of the whole run.
          * The excitation waveform is UNCHANGED (open-loop RC sines). Only
            the STOP decision and the safety envelope are new. Therefore NO
            claim of a "0.3 m position round trip" is made: while VIO is not
            ready there is no position feedback at all, only open-loop sticks.
          * Nothing here touches Gate / LIMITS / age / fusion tolerance /
            acceptance thresholds / fault latching / sample timestamps.

        Returns (done, fatal). fatal=True -> centre sticks and enter the
        EXISTING FAULT state machine (no hidden retry, no smoothing).
        """
        a = self.args
        # (1) Separate RC COMMAND derivative from real motion.
        rc = (self.ch1, self.ch2, self.ch3)
        if self.excite_rc_prev is not None:
            dt = now - self.excite_rc_prev[0]
            if 0.0 < dt < 0.5:
                self.excite_rc_rate = max(
                    abs(rc[i] - self.excite_rc_prev[1][i]) / dt
                    for i in range(3))
        self.excite_rc_prev = (now, rc)
        if self.body_rate is not None:
            self.excite_body_rate_dps = (max(abs(v) for v in self.body_rate)
                                         * 180.0 / math.pi)
        if self.body_tilt is not None:
            self.excite_tilt_deg = (max(abs(v) for v in self.body_tilt)
                                    * 180.0 / math.pi)
        # [NP01-FIX15] 用正常飞控姿态反馈估计 ORB 门限关心的量：
        #   frame_dtheta = max_att_rate_dps * (1/30)      （30 Hz 双目帧间隔）
        #   est_davg     = 9.81 * sin(radians(frame_dtheta))
        # 只用于"激励够不够"的判断与日志，不参与门限本身，也不读真值。
        if self.att_rate_window:
            cutoff = now - 0.4
            self.att_rate_window = [w for w in self.att_rate_window if w[0] >= cutoff]
            if self.att_rate_window:
                peak = max(w[1] for w in self.att_rate_window)
                frame_dtheta = peak / 30.0
                self.est_davg = 9.81 * math.sin(math.radians(frame_dtheta))
                self.est_davg_best = max(self.est_davg_best, self.est_davg)
        # [NP01-FIX15] 自适应幅值：目标 |ΔavgA| 未达到就加大 fast 幅值（有上限），
        # 明显超出就收小，避免无谓地把机体甩得更远。只用姿态反馈，不读真值。
        if self.excite_fast_deg is None:
            self.excite_fast_deg = a.excite_fast_deg
        if self.est_davg >= a.excite_target_davg:
            self.excite_gate_seen = True
        if elapsed > 1.0 and self.body_tilt is not None:
            if self.est_davg < a.excite_target_davg * 0.9:
                self.excite_fast_deg = min(a.excite_fast_deg_max,
                                           self.excite_fast_deg * 1.08)
            elif self.est_davg > a.excite_target_davg * 3.0:
                self.excite_fast_deg = max(a.excite_fast_deg * 0.5,
                                           self.excite_fast_deg * 0.94)
        # 无进展及时退出：给足一个完整 slow 周期仍然一次都没见过目标加速度，
        # 说明这套激励在该 plant 上产生不了 ORB 要的比力阶跃，不再硬撑。
        # 走既有的 FAULT -> LAND -> LANDED -> DONE 安全通道，算失败，不冒充降落。
        if (not self.excite_gate_seen and not self.vio_ready
                and elapsed >= max(a.excite_gate_progress_s, a.excite_slow_period)):
            return self._excite_stop("no_davg_progress", now, elapsed, True)
        # (2) Real displacement in the PX4 local frame (NOT truth).
        if self.hx is not None and self.hy is not None and self.height is not None:
            if self.excite_ref is None:
                self.excite_ref = (self.hx, self.hy, self.height.z)
                # [NP-01 FIX20] 同时捕获 counter/vzrc/sample_us：
                # 进入 EXCITE 前已存在的历史 sticky delta 绝不能被误用。
                self.excite_height_ref, _cap_reason = \
                    capture_excite_height_ref(self.height)
                self.excite_last_sample_us = self.height.sample_us
                self._emit_reset_event({
                    "event": "reference_captured",
                    "reason": _cap_reason,
                    "ref_z": self.height.z,
                    "zrc": self.height.z_reset_counter,
                    "vzrc": self.height.vz_reset_counter,
                    "sample_us": self.height.sample_us,
                    "wall_us": time.time_ns() // 1000,
                })
            self.excite_disp_xy_m = math.hypot(
                self.hx - self.excite_ref[0], self.hy - self.excite_ref[1])
        # (3) Effective FRESH feedback: all three streams must be alive.
        ages = []
        if self.vio_rx_mono is not None:
            ages.append(now - self.vio_rx_mono)
        if self.body_att_rx_mono is not None:
            ages.append(now - self.body_att_rx_mono)
        if self.height is not None:
            ages.append(now - self.height.rx_mono)
        self.excite_feedback_age_ms = (max(ages) * 1000.0) if ages else -1.0
        inputs_ok = (len(ages) == 3
                     and max(ages) <= a.excite_feedback_stale_s
                     and self.height is not None and self.height.z_valid)
        if not inputs_ok and elapsed >= a.excite_input_grace_s:
            return self._excite_stop("input_stale_or_invalid", now, elapsed, True)
        # (3b) [NP-01 FIX20] PX4 局部 NED z 坐标系 reset 的参考连续性。
        # 严格契约（见 excite_height_reference.py）：
        #   * 只在 (zrc_new - zrc_old) % 256 == 1 的可确认新事件上平移一次；
        #   * ref += delta_z（同号），绝不把参考重设为当前 z（会抹掉真实累计偏移）；
        #   * sticky delta / 相同 counter 重复帧不重复平移；
        #   * 跨多事件、回退、乱序、delta 非有限/为零/超界 -> 失败关闭并明确报错；
        #   * reset 当拍只暂缓「高度位移」这一项判定，输入健康/角速度/倾角/
        #     水平位移/VIO 就绪/监督器/授权/硬上限等其它条件全部继续生效；
        #   * 下一有效样本恢复正常高度包络检查；不重置计时、故障或 streak。
        suspend_dz_this_tick = False
        if self.excite_ctl and self.excite_ref is not None:
            d = advance_excite_height_reference(
                self.height, self.excite_height_ref, now,
                a.excite_feedback_stale_s, TAKEOFF_CFG.max_abs_delta_z_m,
                self.excite_last_sample_us)
            if d.action == "fail":
                self._emit_reset_event({
                    "event": "reset_unrecoverable", "reason": d.reason,
                    "zrc_before": d.zrc_before, "zrc_after": d.zrc_after,
                    "wall_us": time.time_ns() // 1000,
                })
                return self._excite_stop(
                    "height_reset_unrecoverable:" + d.reason, now, elapsed, True)
            if d.action == "translate":
                self.excite_height_ref = d.ref
                self.excite_ref = (self.excite_ref[0], self.excite_ref[1], d.ref.z)
                suspend_dz_this_tick = d.suspend_dz_this_tick
                self._emit_reset_event({
                    "event": "reference_translated",
                    "reason": d.reason,
                    "delta_z": d.applied_delta_z,
                    "ref_z_before": d.old_ref_z, "ref_z_after": d.ref.z,
                    "zrc_before": d.zrc_before, "zrc_after": d.zrc_after,
                    "vzrc_before": d.vzrc_before, "vzrc_after": d.vzrc_after,
                    "sample_us": d.ref.sample_us,
                    "z_at_event": self.height.z,
                    "wall_us": time.time_ns() // 1000,
                })
            if self.height is not None:
                self.excite_last_sample_us = self.height.sample_us
        # (4) Envelope from NORMAL feedback, never from truth.
        violation = None
        if self.excite_body_rate_dps > a.excite_max_rate_dps:
            violation = "angular_rate(%.1f>%.1fdeg/s)" % (
                self.excite_body_rate_dps, a.excite_max_rate_dps)
        elif self.excite_tilt_deg > a.excite_max_tilt_deg:
            violation = "tilt(%.1f>%.1fdeg)" % (
                self.excite_tilt_deg, a.excite_max_tilt_deg)
        elif (self.excite_ref is not None and self.height is not None
              and not suspend_dz_this_tick
              and abs(self.height.z - self.excite_ref[2]) > a.excite_max_dz):
            violation = "delta_z(%.2f>%.2fm)" % (
                abs(self.height.z - self.excite_ref[2]), a.excite_max_dz)
        elif self.excite_disp_xy_m > a.excite_max_xy:
            violation = "disp_xy(%.2f>%.2fm)" % (
                self.excite_disp_xy_m, a.excite_max_xy)
        if violation:
            return self._excite_stop("envelope:" + violation, now, elapsed, True)
        # (5) Effective fresh VIO readiness, stable for a bounded hold.
        fresh_ready = (self.vio_ready and self.vio_rx_mono is not None
                       and now - self.vio_rx_mono <= a.excite_feedback_stale_s)
        if fresh_ready:
            self.excite_ever_ready = True
            if self.excite_ready_since is None:
                self.excite_ready_since = now
        else:
            self.excite_ready_since = None
        self.excite_ready_streak_s = (
            (now - self.excite_ready_since) if self.excite_ready_since else 0.0)
        if (fresh_ready and self.excite_ready_streak_s >= a.excite_ready_hold
                and elapsed >= a.excite_min_seconds):
            return self._excite_stop("vio_ready_stable", now, elapsed, False)
        # (6) Hard maximum duration / no-init-progress. If VIO never became
        # ready inside the bounded window there is NO useful fresh feedback:
        # stop the open-loop excitation and take the safe exit instead of
        # drifting in Altitude for --await-vio (45 s in the failing run).
        if elapsed >= a.excite_seconds:
            # [NP-01 FIX13b] At the excitation cap, STOP the open-loop RC but
            # do NOT abort the run immediately.
            #
            # Evidence from np01_fix12_func1m_20261115_165203: ORB-SLAM3 needs
            # a cumulative motion-gated "VIBA2Time: 10 s" of mTinit. That run
            # printed "not enough acceleration" 1608 times and VIBA2 never
            # started; aborting at exactly 12 s removed any chance for VIBA2
            # (total sim span 62 s vs 174 s in the run where VIBA2 completed).
            #
            # The envelope checks above (rate/tilt/dz/disp_xy/input freshness)
            # keep running EVERY tick during this wait, and the independent
            # truth watchdog can still terminate the whole run. So this is a
            # BOUNDED wait, not the unbounded 45 s Altitude drift of run
            # 143101 that produced 19.84 m.
            if self.excite_stop_since is None:
                if not self.excite_stop_reason:
                    self._excite_stop("max_duration_pause", now, elapsed, False)
                return True, False
            if fresh_ready:
                # Readiness just appeared; let step (5) confirm the hold.
                return True, False
            if elapsed - self.excite_stop_since >= a.excite_wait_cap_s:
                return self._excite_stop("no_init_progress", now, elapsed, True)
            return True, False
        return False, False

    def select_position_when_ready(self, waited):
        """等到 VIO 导航就绪后再拨 Position 档。

        PX4 在位置估计无效时拒绝进入 POSCTL，而档位停在原处不会自己重试；
        实机上飞行员也是先确认位置可用再拨档。超过 --await-vio 仍未见就绪时
        照常拨档，交给 POSITION 状态的重新拨档逻辑兜底，不在这里无限等待。

        [NP-01 FIX16] "就绪"必须是**新鲜**的就绪消息，不是粘住的布尔值；
        超时兜底只改变档位选择动作，不构成控制输入，PX4 仍会自行拒绝无效位置。
        """
        now = time.monotonic()
        fresh = ready_is_fresh(self.vio_ready, self.vio_rx_mono, now,
                               self.args.ready_stale_s)
        if fresh or waited >= self.args.await_vio:
            if not fresh:
                self.get_logger().warning(
                    "no FRESH VIO readiness after %.0fs (sticky_ready=%s reason=%s); "
                    "selecting Position anyway, PX4 still gates on its own validity"
                    % (waited, self.vio_ready, self.vio_reason))
            # 不立刻切挡：先回中悬停，让残余角速率先衰减；进入 INIT_SETTLE 后
            # 由"新鲜就绪 + PX4 位置有效"同时满足时立即转 POSITION（见该分支）。
            self.ch1 = self.ch2 = self.ch3 = 1500
            self.set_state("INIT_SETTLE")
        elif not self.await_logged:
            self.await_logged = True
            self.get_logger().info("holding in Altitude until VIO navigation is ready")

    def step(self, now):
        elapsed = now - self.state_since
        # [NP01-FIX16] 独立真值监督器预警 → 取消任务并立即走既有接回/降落流程。
        # 与 FIX15 的区别：不再在 FAULT 里驻留 2 s 才开始降落。2 s 驻留没有
        # 任何独立契约（它只是旧版"先让授权门控看到非 Offboard 档位"的副作用），
        # 而它恰好是漂移的主要消耗。这里先回中、把档位拨回 Position（接回），
        # 直接进入 LAND；同时写 ACK 文件让监督器有"已接收/已处理"的证据。
        if (self.args.warn_marker and not self.warn_reacted
                and os.path.exists(self.args.warn_marker)):
            self.warn_reacted = True
            self.ch1 = self.ch2 = self.ch3 = 1500
            self.slot = self.args.slot_position
            self._write_ack("warn_cancel_task_land")
            self.get_logger().error(
                "[NP01_FIX16] independent truth monitor raised a WARNING; "
                "cancelling the task, selecting the Position detent and entering "
                "LAND immediately (no FAULT dwell; only the marker's existence is "
                "observed, never its value)")
            self.set_state("LAND")
            return
        ready = bool(self.bt.get("offboard_ready"))
        completed = bool(self.bt.get("task_completed"))
        control = bool(self.bt.get("control_active"))
        if control and self.control_since is None:
            self.control_since = now
            self.get_logger().info("BT control active")
        if completed and self.completed_since is None:
            self.completed_since = now
            self.get_logger().info("BT task completed")

        if self.state == "INIT":
            if elapsed >= 3.0:
                # [NP-01 FIX9] Health gate BEFORE any arming request.
                self.set_state("PREFLIGHT")
        elif self.state == "PREFLIGHT":
            self.preflight_accumulate()
            if elapsed >= self.args.preflight_seconds:
                ok, reason, summary = self.preflight_report()
                self.get_logger().info(
                    "PREFLIGHT_SUMMARY " + json.dumps(
                        dict(ok=ok, reason=reason, window_s=elapsed, **summary),
                        ensure_ascii=False, default=str))
                if ok:
                    self.get_logger().info(
                        "preflight healthy: z=%s..%s vz_absmax=%.2f "
                        "z_valid=%d vz_valid=%d xy_valid=%d fusion=%s"
                        % (summary["z_min"], summary["z_max"],
                           summary["vz_absmax"], summary["z_valid"],
                           summary["vz_valid"], summary["xy_valid"],
                           json.dumps(summary["fusion"])))
                    self.ch5 = 2000
                    self.set_state("ARM")
                else:
                    # Explicit failure. No silent re-filter, no retry loop: the
                    # attempt is recorded as not-ready and stays there.
                    self.get_logger().error(
                        "PREFLIGHT UNHEALTHY (%s); refusing to arm this attempt"
                        % reason)
                    self.set_state("FAULT")
        elif self.state == "ARM":
            if self.armed:
                self.set_state("TAKEOFF")
            elif elapsed > 12.0:
                self.get_logger().warning("arm not confirmed; holding for inspection")
                self.set_state("FAULT")
        elif self.state == "TAKEOFF":
            # [NP-01 FIX6-corrected] Signed NED climb with validity, freshness,
            # reset and sanity contract. Never enter EXCITE on an unconfirmed
            # takeoff: that path let a diverged EKF keep full throttle and then
            # start horizontal excitation (RUN-NP01-06, truth z 5.56 m).
            st = advance_takeoff(self.height, self.takeoff_ref,
                                 self.args.takeoff_alt, now, elapsed,
                                 TAKEOFF_CFG, self.takeoff_recaptures)
            self.takeoff_ref = st.ref
            self.takeoff_recaptures = st.recaptures
            self.height_status = st.status
            self.height_reason = st.reason
            self.ch3 = st.throttle
            if st.state == "TAKEOFF_SETTLE":
                self.get_logger().info("takeoff confirmed: %s (z=%.3f)"
                                       % (st.reason, self.height.z))
                self.set_state("TAKEOFF_SETTLE")
            elif st.state == "FAULT":
                # Invalid / stale / diverged / reset-exhausted / timeout:
                # explicit failure, NEVER fall through to EXCITE.
                self.get_logger().error(
                    "takeoff NOT confirmed after %.1fs (reason=%s, seen=%d, "
                    "recaptures=%d); entering FAULT, no excitation"
                    % (elapsed, st.reason, self.height_seen,
                       self.takeoff_recaptures))
                self.set_state("FAULT")
            elif st.status == "capture":
                self.get_logger().info("height reference captured: z=%.3f reason=%s"
                                       % (st.ref.z, st.reason))
        elif self.state == "TAKEOFF_SETTLE":
            if elapsed >= 1.5:
                self.set_state("EXCITE")
        elif self.state == "EXCITE":
            # 默认（--excite-seconds 0）：原地悬停，位置锁定交给随后切入的
            # Position 档，行为树锚点≈起飞点。旧 STEREO 路径依赖这个行为。
            #
            # IMU_STEREO 路径必须显式打开激励：ORB-SLAM3 的 LocalMapping 在
            # 关键帧之间位移 <2cm 时会判定 "Not enough motion for initializing"
            # 并重置当前地图，纯悬停永远无法完成 VIBA2。这里用俯仰/横滚方波
            # 产生真实平移（不是 IMU 假数据），和实机起飞后的手动激励一致。
            self.ch3 = 1500
            if self.args.excite_seconds > 0.0:
                if not self.excite_logged:
                    self.excite_logged = True
                    if self.args.excite_mode == "translate":
                        self.get_logger().info(
                            "init excitation: dual-component, slow(pitch) %.1fdeg@%.1fs "
                            "+ fast(roll, %s) %.1fdeg@%.1fs, %.1fs"
                            % (self.args.excite_slow_deg, self.args.excite_slow_period,
                               self.args.excite_waveform, self.args.excite_fast_deg,
                               self.args.excite_fast_period, self.args.excite_seconds))
                    else:
                        self.get_logger().info(
                            "excitation: +/-%d pitch, +/-%d roll, %.1fs"
                            % (self.args.excite_pitch, self.args.excite_roll,
                               self.args.excite_seconds))
                if self.args.excite_mode == "translate":
                    # [NP01-FIX15] 两个叠加分量，各自对应 ORB-SLAM3 的一条独立要求：
                    #   slow（pitch）: 低频平移，关键帧间位移 > 5 cm，LocalMapping
                    #                  的 mTinit 才会累加，才能走到 VIBA1/VIBA2。
                    #   fast（roll） : 高频姿态抖动，相邻帧 |ΔavgA| >= 0.5 m/s²，
                    #                  即帧间倾角变化 >= 2.9°（g·Δθ = 0.5）。
                    # 第 8 轮实测：旧 ±60 us 慢正弦只产生约 ±2° 且帧间变化约 0.4°，
                    # 对应 |ΔavgA| ≈ 0.07 m/s²，比门限低一个量级，所以整轮都在
                    # "not enough acceleration"。两轴分开，互不抵消。
                    deg_per_us = max(1e-3, self.args.excite_deg_per_us)
                    if self.excite_fast_deg is None:
                        self.excite_fast_deg = self.args.excite_fast_deg
                    slow_period = max(0.5, self.args.excite_slow_period)
                    fast_period = max(0.1, self.args.excite_fast_period)
                    slow_deg = self.args.excite_slow_deg * math.sin(
                        2.0 * math.pi * elapsed / slow_period)
                    phase_fast = math.sin(2.0 * math.pi * elapsed / fast_period)
                    if self.args.excite_waveform == "square":
                        fast_deg = self.excite_fast_deg * (1.0 if phase_fast >= 0.0 else -1.0)
                    else:
                        fast_deg = self.excite_fast_deg * phase_fast
                    ch_roll = int(round(1500 + fast_deg / deg_per_us))
                    ch_pitch = int(round(1500 + slow_deg / deg_per_us))
                    ch_vert = 1500
                    if self.args.excite_vert > 0:
                        ch_vert = int(round(
                            1500 + self.args.excite_vert
                            * math.sin(2.0 * math.pi * elapsed / slow_period)))
                    self.ch1 = max(1000, min(2000, ch_roll))
                    self.ch2 = max(1000, min(2000, ch_pitch))
                    self.ch3 = max(1000, min(2000, ch_vert))
                    # [SIM-ONLY] 第 10 轮单变量：translate 分支同步「VIO 就绪即停」。
                    # 依据（第 9 轮 2049 实测）：translate 分支原本没有提前停止，
                    # 激励跑满 45s；故障前飞机速度 <0.5m/s、EV=1/1/1、
                    # vio_ready=True 已持续 2.8s，但 SLAM 位置从 (3.03,-1.69)
                    # 一路漂到 (5.18,-3.14)，features 1001->12、tracking=LOST，
                    # 同一刻 age 109->203.6ms 并命中 attitude_discontinuity 锁存。
                    # attitude 分支的 12s 下限晚于本轮故障(EXCITE+10.9s)，
                    # 故按 vio_ready 直接停，再回中 3s 让残余速度衰减。
                    # 门控/桥接/SLAM/任一 yaml 口径一字未动。
                    if self.excite_ctl:
                        excite_done, excite_fatal = self.bounded_excite_decision(
                            now, elapsed)
                        if excite_fatal:
                            # [NP-01 FIX12] Safe exit through the EXISTING
                            # FAULT -> LAND -> LANDED -> DONE state machine.
                            # Sticks centred; no truth input; no hidden retry.
                            self.ch1 = self.ch2 = self.ch3 = 1500
                            self.get_logger().error(
                                "[NP01_FIX12] bounded excitation safety exit: %s; "
                                "open-loop RC stopped, entering FAULT"
                                % (self.excite_stop_reason or "unknown"))
                            self.set_state("FAULT")
                            return
                    else:
                        excite_done = (elapsed >= self.args.excite_seconds
                                       or (self.vio_ready
                                           and not self.args.excite_no_early_stop))
                    if excite_done:
                        self.ch1 = self.ch2 = 1500
                        self.ch3 = 1500
                        if self.excite_stop_since is None:
                            self.excite_stop_since = elapsed
                            self.get_logger().info(
                                "translate excitation stop at %.1fs (vio_ready=%s, cap=%.0fs)"
                                % (elapsed, self.vio_ready, self.args.excite_seconds))
                        # [NP-01 FIX16] 删除这里原本固定 3.0 s 的"激励后沉降"。
                        # 它与 INIT_SETTLE 是同一个契约：摇杆回中、让残余角速率
                        # 衰减、等就绪稳定。旧版把两段**串行**执行，第 14 轮实测
                        # 因此白等 3.004 s（≈1.4 m 漂移）。现在只保留一个窗口，
                        # 且该窗口在"新鲜就绪 + PX4 位置有效"同时成立时提前结束。
                        # 不降低连续 ready 要求，也不绕过 PX4 有效性。
                        self.select_position_when_ready(elapsed - self.excite_stop_since)
                else:
                    # 正弦而非方波：方波在翻转瞬间角速度无穷大，画面整幅跳变，
                    # 视觉先丢跟踪（实测 ±60/±40 方波 -> tracking_invalid 550 次），
                    # IMU 再够也没用。正弦平滑，同样摆幅下相邻帧只转几度。
                    # roll 用半频：verify_sine 那轮（ev=1 312 / sent 7346 / maps 13）
                    # 用的就是这个频谱；改成同频 90° 相位（圆轨迹）后再未复现过。
                    w = 2.0 * math.pi * elapsed / max(0.2, self.args.excite_period)
                    self.ch2 = int(round(1500 + math.sin(w) * self.args.excite_pitch))
                    self.ch1 = int(round(1500 + math.sin(w * 0.5) * self.args.excite_roll))
                    # [SIM-ONLY] 提前停止条件：VIO 导航一旦就绪就停止激励。
                    # 第 4 轮实测：VIBA2 在激励中途完成，但继续摆到 45 s 使飞机
                    # 过度机动，ORB 特征降到 0、tracking=LOST 且不可恢复。
                    # 实机飞手也是看到 READY 就停手。12 s 下限保证 VIBA1(3s)+
                    # VIBA2(10s) 有足够运动激励；excite_seconds 保留为兜底上限。
                    excite_done = (elapsed >= self.args.excite_seconds
                                   or (self.vio_ready and elapsed >= 12.0
                                       and not self.args.excite_no_early_stop))
                    if excite_done:
                        # 激励结束后先回中悬停 3 s，让残余速度衰减，
                        # 再切 Position，保证切档瞬间飞机留在原点上空。
                        self.ch1 = self.ch2 = 1500
                        if self.excite_stop_since is None:
                            self.excite_stop_since = elapsed
                            self.get_logger().info(
                                "excitation stop at %.1fs (vio_ready=%s, cap=%.0fs)"
                                % (elapsed, self.vio_ready, self.args.excite_seconds))
                        if elapsed >= self.excite_stop_since + 3.0:
                            self.select_position_when_ready(0.0)
            else:
                # 真值链路（无 IMU_STEREO 初始化）不需要悬停等 VIBA：Altitude
                # 模式没有水平位置保持，悬停越久漂得越远（实测 6.5 s 漂 3 m）。
                # 只留 0.5 s 回中，随即切 Position 交给位置环。
                self.ch1 = self.ch2 = 1500
                if elapsed >= 0.5:
                    self.select_position_when_ready(elapsed - 0.5)
        elif self.state == "INIT_SETTLE":
            # 初始化完成后的控制悬停：所有摇杆回中，Altitude 模式的速度环会阻尼掉
            # 激励留下的水平速度；同时给 VIO 一段稳定窗口。
            #
            # [NP-01 FIX16] 不再是"固定睡 --init-settle 秒"：
            #   * 必须同时满足：新鲜的就绪消息 + PX4 自己的 xy/vxy 有效；
            #   * 满足后立即转 POSITION（下限 --init-settle-min 只用来避开
            #     摇杆阶跃瞬间，不是稳定性门限）；
            #   * 上限仍是 --init-settle（与旧版同值），到点照常转档，
            #     由 POSITION 的重新拨档逻辑兜底，PX4 仍会拒绝无效位置。
            self.ch1 = self.ch2 = self.ch3 = 1500
            now_mono = time.monotonic()
            fresh = ready_is_fresh(self.vio_ready, self.vio_rx_mono, now_mono,
                                   self.args.ready_stale_s)
            xy_ok = bool(self.height is not None and self.height.xy_valid)
            vxy_ok = bool(self.height is not None and self.height.v_xy_valid)
            done, why = settle_exit(elapsed, fresh, xy_ok, vxy_ok,
                                    self.args.init_settle_min,
                                    self.args.init_settle_max or self.args.init_settle)
            if done:
                self.slot = self.args.slot_position
                self.get_logger().info(
                    "[NP01_FIX16] handoff settle done after %.2fs (%s): "
                    "fresh_ready=%s vio_age=%.0fms xy_valid=%s vxy_valid=%s "
                    "-> Position detent, then --position-hold %.1fs before Offboard"
                    % (elapsed, why, fresh,
                       (now_mono - self.vio_rx_mono) * 1000.0 if self.vio_rx_mono else -1.0,
                       xy_ok, vxy_ok, self.args.position_hold))
                self.set_state("POSITION")
        elif self.state == "POSITION":
            # 回中目标 = 导航就绪后捕获一次的本地坐标锚点（见 drain/on_vio_health）。
            # 不是本地原点 (0,0)：外视觉的本地原点在飞行中建立，与起飞点相差约 12 m。
            # 误差在 NED 世界系，POSITION 档摇杆在机体系，必须按当前偏航旋转；
            # 否则机头偏航约 96° 时 x/y 分量会被打到错误的轴上。
            # 死区 0.35 m，增益按位置误差线性给俯仰/横滚，接近后回中。
            yaw = self.yaw
            if yaw is not None and (self.yaw_time is None or now - self.yaw_time > 2.0):
                yaw = None
            have_home = (not self.args.no_home_gate
                         and ready_is_fresh(self.vio_ready, self.vio_rx_mono, now,
                                            self.args.ready_stale_s)
                         and self.home is not None and self.hx is not None
                         and self.hy is not None and yaw is not None)
            if have_home:
                ex, ey = self.home[0] - self.hx, self.home[1] - self.hy
                if math.hypot(ex, ey) > HOME_DEADZONE_M:
                    self.ch1, self.ch2 = sticks_for_home(
                        self.home, (self.hx, self.hy), yaw)
                    self.home_logged = True
                else:
                    self.ch1 = self.ch2 = 1500
            else:
                # 位置/航向无效时不得沿用上一拍的饱和摇杆（103614 实测：VIO 在
                # 96.7 s 失跟踪后 1420 一直保留到 128 s）。失效即回中。
                self.ch1 = self.ch2 = 1500
            # 切到 Position 后先定点悬停一段，确认位置估计收敛、飞机真的停在锚点，
            # 才把控制权交给行为树去 Offboard。
            if self.args.no_home_gate:
                at_home = True
            else:
                at_home = (self.home is not None and self.hx is not None
                           and self.hy is not None
                           and math.hypot(self.home[0] - self.hx,
                                          self.home[1] - self.hy) <= 0.35)
            gate_ok = ready or self.args.skip_ready_gate
            if gate_ok and at_home and elapsed >= self.args.position_hold:
                self.set_state("OFFBOARD_ARMED")
            elif self.detent_retry_pending:
                # 回到 Altitude 档 0.8 s，让 mode_slot 真的回退一次。这一步同时
                # 重置授权门控里「档位与模式不一致」的观察计时，避免它在等待
                # PX4 追上模式的过程中累积成故障。
                if now - self.detent_retry_since >= 0.8:
                    self.slot = self.args.slot_position
                    self.detent_retry_pending = False
                    self.get_logger().info(
                        "re-selecting Position detent (%d/3)" % self.detent_retries)
            elif self.detent_retries < 3 and elapsed >= 8.0 * (self.detent_retries + 1):
                # PX4 对「档位变化」只处理一次：若当时位置估计尚不可用，它拒绝
                # 进入 POSCTL 且不会自己重试。实机上飞行员也是把开关拨回来再拨
                # 一次，这里照做，间隔 8 s 与门控的观察窗口对齐。
                self.detent_retries += 1
                self.slot = self.args.slot_altitude
                self.detent_retry_pending = True
                self.detent_retry_since = now
                self.get_logger().info(
                    "Position mode not accepted yet; cycling detent %d/3"
                    % self.detent_retries)
            elif elapsed > 60.0:
                self.get_logger().warning("OFFBOARD READY not seen; bt reason=" +
                                          str(self.bt.get("reason")))
                self.set_state("FAULT")
        elif self.state == "OFFBOARD_ARMED":
            if elapsed >= 0.8:
                self.slot = self.args.slot_offboard
                self.offboard_cycle_wait_since = now
                self.get_logger().info("switch to Offboard slot")
                self.set_state("NAVIGATE")
        elif self.state == "NAVIGATE":
            # 只改飞手的物理档位动作，不改行为树/授权门控的任何判据。
            # BT 未 started 时循环拨档，让它有机会重新完成 1.2 s 预热。
            if self.offboard_cycle_low_since is not None:
                if now - self.offboard_cycle_low_since >= self.args.offboard_cycle_low:
                    self.slot = self.args.slot_offboard
                    self.offboard_cycle_low_since = None
                    self.offboard_cycle_wait_since = now
                    self.get_logger().info("re-selecting Offboard detent (cycle %d/%d)"
                                           % (self.offboard_cycles, self.args.offboard_cycle_max))
            elif (not self.bt.get("control_active")
                  and now - self.offboard_cycle_wait_since >= self.args.offboard_cycle_after):
                if (self.offboard_cycles >= self.args.offboard_cycle_max
                        or elapsed > self.args.navigate_timeout):
                    self.get_logger().warning(
                        "Offboard authorization not acquired; cycles=%d elapsed=%.1fs bt_reason=%s"
                        % (self.offboard_cycles, elapsed, self.bt.get("reason")))
                    self.set_state("FAULT")
                else:
                    self.offboard_cycles += 1
                    self.slot = self.args.slot_position
                    self.offboard_cycle_low_since = now
                    self.get_logger().info(
                        "cycling RC selector back to Position (cycle %d/%d) bt_reason=%s"
                        % (self.offboard_cycles, self.args.offboard_cycle_max,
                           self.bt.get("reason")))
            if self.args.takeover_after > 0 and self.control_since is not None:
                if time.monotonic() - self.control_since >= self.args.takeover_after \
                        and not self.takeover_sent:
                    self.takeover_sent = True
                    self.slot = self.args.slot_position
                    self.get_logger().info("injected takeover: switch back to Position slot")
                    self.set_state("TAKEOVER")
            elif completed and self.completed_since is not None:
                if time.monotonic() - self.completed_since >= self.args.completion_hold:
                    self.slot = self.args.slot_position
                    self.get_logger().info("completion hold elapsed: pilot takeover")
                    self.set_state("TAKEOVER")
            if not control and self.bt.get("phase") == "REVOKED":
                self.set_state("TAKEOVER")
        elif self.state in ("TAKEOVER", "FAULT"):
            # Let the authority gate observe a non-Offboard selector, then land
            # under pilot control in Position mode.
            # 摇杆回中（与实机飞手一致）：故障后若保留最后的位置修正量，
            # 飞机会在 Altitude 模式下持续带坡度飞出视距。
            self.ch1 = self.ch2 = 1500
            if elapsed >= 2.0:
                self.slot = self.args.slot_position
                self.set_state("LAND")
        elif self.state == "LAND":
            self.ch1 = self.ch2 = 1500
            # 不靠 EKF z 提前宣布落地：z 只是相对起飞点的 NED 高度，
            # 实测提前回中会让 PX4 认为仍有推力需求，落地检测与 disarm 都不触发。
            self.ch3 = 1150
            self.ch5 = 1000
            if self.armed is False:
                self.set_state("LANDED")
            elif elapsed > 35.0:
                self.get_logger().warning("land timeout; armed=%s z=%s" % (self.armed, self.z))
                self.set_state("LANDED")
        elif self.state == "LANDED":
            self.ch3 = 1000
            self.ch5 = 1000
            # 验收要求「降落并解锁」：等到 PX4 真正报 disarm 再结束，
            # 超时则如实记录（检查项会判 FAIL），不用超时冒充成功。
            if not self.armed and elapsed >= 2.0:
                self.set_state("DONE")
            elif elapsed >= 20.0:
                self.get_logger().warning("disarm not confirmed after landing")
                self.set_state("DONE")

    def run(self):
        end = None
        diag_t0 = time.monotonic()
        while rclpy.ok():
            now = time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.0)
            self.drain()
            self.send_rc(now)
            # [PHASE7-DIAG] Bounded collection cutoff: stop advancing the task
            # and ENTER the existing LAND state machine. Independent of VIO
            # readiness / OFFBOARD; runner keeps a separate exit time budget.
            if diagnostic_exit_due(self.state, now - diag_t0,
                                   self.args.diagnostic_exit_after):
                self.get_logger().warning(
                    "diagnostic cutoff reached (%.1fs); entering LAND "
                    "(vio_ready=%s armed=%s); no further task progression"
                    % (now - diag_t0, self.vio_ready, self.armed))
                self.set_state("LAND")
            self.step(now)
            self.counter += 1
            if self.counter % 10 == 0:
                self.log(now)
            if self.state == "DONE":
                self.log(now)
                break
            time.sleep(0.005)
        self.csv.close()
        with open(self.args.log + ".events.json", "w") as f:
            json.dump(self.events, f, indent=2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", required=True)
    ap.add_argument("--gcs-port", type=int, default=14550)
    # [NP-01 FIX9] Bounded unlocked preflight window. 0 disables the gate.
    ap.add_argument("--preflight-seconds", type=float, default=12.0,
                    help="unlocked preflight health window (s); 0 = disabled")
    ap.add_argument("--slot-altitude", type=int, default=1000)
    ap.add_argument("--slot-position", type=int, default=1234)
    ap.add_argument("--slot-offboard", type=int, default=1400)
    ap.add_argument("--takeover-after", type=float, default=0.0,
                    help="seconds after control_active to force RC back to Position; 0=success run")
    ap.add_argument("--completion-hold", type=float, default=8.0)
    ap.add_argument("--topic-prefix", default="")
    # IMU_STEREO 验证用：起飞后施加水平激励，让 ORB-SLAM3 完成惯性初始化。
    # 默认 0 = 原地悬停，保持旧 STEREO 路径的行为不变。
    ap.add_argument("--excite-seconds", type=float, default=0.0,
                    help="length of the post-takeoff horizontal excitation; 0=hover")
    ap.add_argument("--excite-pitch", type=int, default=180,
                    help="pitch stick amplitude (us around 1500)")
    ap.add_argument("--excite-roll", type=int, default=120,
                    help="roll stick amplitude (us around 1500)")
    ap.add_argument("--excite-period", type=float, default=2.4,
                    help="pitch square-wave period in seconds")
    ap.add_argument("--excite-mode", choices=("attitude", "translate"),
                    default="attitude",
                    help="attitude: original pitch/roll square wave; "
                         "translate: gentle fore/aft, left/right, up/down sines "
                         "(for IMU init without losing visual tracking)")
    ap.add_argument("--excite-no-early-stop", action="store_true",
                    help="[SIM-ONLY] keep exciting for the full --excite-seconds even "
                         "after VIO reports ready; used when the excitation window must "
                         "be fixed in advance (reproducible runs)")
    # [NP-01 FIX12] Opt-in bounded state-driven excitation. Default "legacy"
    # keeps every historical invocation unchanged, including the
    # --excite-no-early-stop switch. Truth is never an input to these bounds.
    ap.add_argument("--excite-controller", choices=("legacy", "bounded"),
                    default="legacy",
                    help="[SIM-ONLY] legacy=unchanged; bounded=state-driven stop "
                         "from effective fresh VIO feedback with a safety envelope")
    ap.add_argument("--excite-ready-hold", type=float, default=0.4,
                    help="[FIX12] continuous FRESH navigation_ready required (s)")
    ap.add_argument("--excite-wait-cap-s", type=float, default=25.0,
                    help="[FIX13b] bounded wait after the excitation cap for "
                         "effective fresh VIO readiness; then safe exit (s)")
    ap.add_argument("--excite-min-seconds", type=float, default=2.0,
                    help="[FIX12] minimum excitation before a ready stop (s)")
    ap.add_argument("--excite-feedback-stale-s", type=float, default=0.6,
                    help="[FIX12] max host-monotonic receive age of each of the "
                         "three normal feedback streams (s)")
    ap.add_argument("--excite-input-grace-s", type=float, default=2.0,
                    help="[FIX12] startup grace before a missing stream is fatal (s)")
    ap.add_argument("--excite-max-rate-dps", type=float, default=300.0,
                    help="[FIX12] MAVLink ATTITUDE body-rate envelope (deg/s)")
    ap.add_argument("--excite-max-tilt-deg", type=float, default=25.0,
                    help="[FIX12] MAVLink ATTITUDE roll/pitch envelope (deg)")
    ap.add_argument("--excite-max-dz", type=float, default=1.0,
                    help="[FIX12] VehicleLocalPosition z deviation (m)")
    ap.add_argument("--excite-max-xy", type=float, default=6.0,
                    help="[FIX12] PX4 LOCAL_POSITION_NED horizontal displacement "
                         "from the first in-EXCITE sample (m). [NP01-FIX15] 3.0 -> 6.0: "
                         "第 12/13 轮真值显示无位置保持漂移约 0.5 m/s，方向朝南；"
                         "中心起点离南墙 7.80 m（旋翼接触 7.41 m）。3.0/3.5 m 会误杀"
                         "已完成惯性初始化（nav init=1 ba2=1）的健康轮次。"
                         "6.0 m 仍比旋翼接触少 1.4 m，且独立真值监督器还有按净空的"
                         "两档（warn 2.0 m / stop 1.2 m）兜底")
    ap.add_argument("--excite-vert", type=int, default=40,
                    help="vertical stick amplitude for translate mode (us around 1500)")
    # [NP01-FIX15] 激励由两个叠加分量组成，分别对应 ORB-SLAM3 的两条独立要求：
    #   slow : 低频平移（关键帧间位移 > 5 cm）→ LocalMapping 的 mTinit 才累加，
    #          才能走到 VIBA1(3 s) / VIBA2(10 s)。
    #   fast : 高频小幅姿态抖动 → Tracking::StereoInitialization 的
    #          norm(avgA_cur - avgA_prev) >= 0.5 m/s² 才能通过。
    # 只有 slow 过不了加速度门限（第 8 轮实测帧间倾角变化仅约 0.4°）；
    # 只有 fast 过不了 mTinit 的位移门限。
    ap.add_argument("--excite-waveform", choices=["sine", "square"], default="square",
                    help="[NP01-FIX15] fast-component waveform; square maximises d(theta)/dt")
    ap.add_argument("--excite-slow-period", type=float, default=3.0,
                    help="[NP01-FIX15] slow translation sway period (s)")
    ap.add_argument("--excite-slow-deg", type=float, default=3.0,
                    help="[NP01-FIX15] slow sway tilt amplitude (deg); 3 deg @3 s ~= +-13 cm")
    ap.add_argument("--excite-fast-period", type=float, default=0.5,
                    help="[NP01-FIX15] fast dither period (s)")
    ap.add_argument("--excite-fast-deg", type=float, default=8.0,
                    help="[NP01-FIX15] fast dither tilt amplitude (deg)")
    ap.add_argument("--excite-fast-deg-max", type=float, default=12.0,
                    help="[NP01-FIX15] adaptation cap for the fast dither (deg)")
    ap.add_argument("--excite-deg-per-us", type=float, default=0.033,
                    help="[NP01-FIX15] measured stick->tilt gain (deg per us); "
                         "derived from the centre-start run, verified in the log")
    ap.add_argument("--excite-target-davg", type=float, default=0.6,
                    help="[NP01-FIX15] target |d avgA| between frames (m/s^2); "
                         "ORB requires 0.5, this keeps margin")
    ap.add_argument("--excite-gate-progress-s", type=float, default=8.0,
                    help="[NP01-FIX15] if the target d avgA is never reached within "
                         "this window, stop with an explicit no-progress exit (s)")
    ap.add_argument("--warn-marker", default="",
                    help="[NP01-FIX15/16] path written by the independent truth monitor on "
                         "a WARNING. Its presence makes the pilot cancel the task and "
                         "request LAND. No truth VALUE is ever read, only the file's "
                         "existence, so truth still never enters normal control.")
    ap.add_argument("--ack-marker", default="",
                    help="[NP01-FIX16] path written by the pilot when it has PROCESSED "
                         "a warning (receipt confirmation for the supervisor).")
    ap.add_argument("--ready-stale-s", type=float, default=READY_STALE_S,
                    help="[NP01-FIX16] a navigation_ready message older than this on the "
                         "host monotonic clock is NOT treated as ready.")
    ap.add_argument("--init-settle-min", type=float, default=0.5,
                    help="[NP01-FIX16] minimum INIT_SETTLE before the Position detent "
                         "(only to leave the RC step behind; not a stability gate)")
    ap.add_argument("--init-settle-max", type=float, default=0.0,
                    help="[NP01-FIX16] optional hard cap; 0 = use --init-settle")
    ap.add_argument("--takeoff-alt", type=float, default=1.7,
                    help="takeoff target altitude in metres (positive)")
    ap.add_argument("--init-settle", type=float, default=6.0,
                    help="seconds to hold in Altitude with sticks centred after IMU init")
    ap.add_argument("--position-hold", type=float, default=10.0,
                    help="seconds to hold in Position at the origin before Offboard")
    ap.add_argument("--skip-ready-gate", action="store_true",
                    help="[SIM-ONLY] link probe: switch to Offboard detent without waiting "
                         "for BT offboard_ready status")
    ap.add_argument("--no-home-gate", action="store_true",
                    help="[SIM-ONLY] link probe: skip the return-to-origin RC correction "
                         "and its 0.35 m gate (real-flight link keeps it)")
    ap.add_argument("--offboard-cycle-after", type=float, default=5.0,
                    help="BT 未授权时，保持 Offboard 档多久后拨回 Position 重试（s）")
    ap.add_argument("--offboard-cycle-low", type=float, default=1.8,
                    help="每次循环在 Position 档停留时长（s），须覆盖 BT 1.2 s 预热")
    ap.add_argument("--offboard-cycle-max", type=int, default=6,
                    help="最多循环拨档次数，超出转 FAULT")
    ap.add_argument("--navigate-timeout", type=float, default=120.0,
                    help="NAVIGATE 状态最长等待授权时间（s），超出转 FAULT")
    ap.add_argument("--await-vio", type=float, default=30.0,
                    help="seconds to hold in Altitude waiting for VIO readiness")
    ap.add_argument("--diagnostic-exit-after", type=float,
                    default=float(os.environ.get("SIM_DIAG_EXIT_AFTER", "0") or 0.0),
                    help="[DIAG-ONLY] bounded collection cutoff (s); on expiry"
                         " enter the existing LAND state machine from any state."
                         " 0 disables. Default from SIM_DIAG_EXIT_AFTER.")
    args = ap.parse_args()
    rclpy.init()
    n = Pilot(args)
    try:
        n.run()
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
