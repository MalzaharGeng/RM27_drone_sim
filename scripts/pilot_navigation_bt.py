#!/usr/bin/env python3
"""Manual takeoff -> Position verification -> physical RC Offboard -> task.

Reactive authority guard around Sequence(Line OR Square, HoldUntilTakeover).
Failure revokes the task and stops publication. PX4/pilot own recovery.
No arm/disarm, mode, takeoff or land command publisher exists here.
Pre-entry sends OffboardControlMode proof-of-life ONLY, never a trajectory.
"""
import argparse
import csv
import json
import math
import os
import pathlib
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleStatus,
                          VehicleLocalPosition, VehicleLandDetected,
                          ManualControlSetpoint, ManualControlSwitches)
from std_msgs.msg import String
from pilot_authority import PilotAuthority

SUCCESS, RUNNING = "SUCCESS", "RUNNING"
NAN = float("nan")
ABORT_FILE = "/tmp/px4_square_abort"


class Sequence:
    def __init__(self, children):
        self.children, self.index = children, 0

    def tick(self, bb, now):
        while self.index < len(self.children):
            result = self.children[self.index].tick(bb, now)
            if result != SUCCESS:
                return result
            self.index += 1
        return SUCCESS


class GotoPoint:
    def __init__(self, forward, right, tolerance=.15, dwell=2.):
        self.forward, self.right = forward, right
        self.tolerance, self.dwell = tolerance, dwell
        self.arrived = self.started = None

    def tick(self, bb, now):
        if self.started is None:
            self.started = now
        c, s = math.cos(bb.yaw), math.sin(bb.yaw)
        target = (bb.home[0] + c*self.forward - s*self.right,
                  bb.home[1] + s*self.forward + c*self.right, bb.home[2])
        bb.target = target
        elapsed = min(.1, max(0., now - bb.last_tick))
        distance = math.dist(bb.sp, target)
        fraction = min(1., bb.args.speed * elapsed / distance) if distance > 0 else 1.
        bb.sp = tuple(a + fraction*(b-a) for a, b in zip(bb.sp, target))
        if math.dist(bb.pos, target) <= self.tolerance and math.dist(bb.vel, (0., 0., 0.)) <= .2:
            if self.arrived is None:
                self.arrived = now
            if now - self.arrived >= self.dwell and math.dist(bb.sp, target) <= 1e-6:
                return SUCCESS
        else:
            self.arrived = None
        if now - self.started > math.hypot(self.forward, self.right) / bb.args.speed * 4 + 20:
            raise RuntimeError("waypoint_timeout")
        return RUNNING


class Hold:
    def tick(self, bb, now):
        bb.sp = bb.target
        if bb.phase != "HOLD" and hasattr(bb, "complete"):
            bb.complete()
        bb.phase = "HOLD"
        return RUNNING


def build_tree(args):
    points = ([(args.distance, 0.)] if args.mission == "line" else
              [(args.side, 0.), (args.side, args.side), (0., args.side), (0., 0.)])
    return Sequence([GotoPoint(*p) for p in points] + [Hold()])


class SquareBT(Node):
    def __init__(self, args):
        super().__init__("px4_pilot_navigation")
        # 只改本机：WSL_RELAX_FRESHNESS=1 时跳过时效判定，其余门控不变；
        # 不设该变量时（实机/回归测试）走 nano 同源的原始门控逻辑。
        wsl_relax = os.environ.get("WSL_RELAX_FRESHNESS") == "1"
        self.args, self.authority = args, PilotAuthority(
            args.rc_offboard_slot, args.rc_three_position, relax_freshness=wsl_relax)
        self.done = False
        self.sp = self.target = self.home = self.pos = self.vel = None
        self.yaw = 0.
        self.last_tick = self.started_at = time.monotonic()
        self.phase, self.last_note = "WAIT_PILOT", ""
        self.completed = False
        self.event_sequence = 0
        self.prestream_since = self.last_prestream = self.selector_since = None
        self.offboard_ready = False
        self.prestream_duration = 0.
        self.tree = build_tree(args)
        self.log_file = self.writer = None
        if args.log:
            path = pathlib.Path(args.log).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.log_file = path.open("w", newline="")
            self.writer = csv.writer(self.log_file)
            self.writer.writerow(["wall_us", "steady_s", "phase", "north_m", "east_m", "down_m",
                                  "sp_n", "sp_e", "sp_d", "target_n", "target_e", "target_d", "reason"])
            self.log_file.flush()
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        def subscribe(cls, name, fn):
            version = getattr(cls, "MESSAGE_VERSION", 0)
            topic = args.topic_prefix + "/fmu/out/" + name + (f"_v{version}" if version else "")
            self.create_subscription(cls, topic, fn, qos)
        subscribe(VehicleStatus, "vehicle_status", self.on_status)
        subscribe(VehicleLocalPosition, "vehicle_local_position", self.on_local)
        subscribe(ManualControlSetpoint, "manual_control_setpoint", self.on_rc)
        subscribe(ManualControlSwitches, "manual_control_switches", self.on_switches)
        subscribe(VehicleLandDetected, "vehicle_land_detected", self.on_land)
        self.create_subscription(String, args.topic_prefix + "/vio/navigation_health", self.on_health, 1)
        self.pub_ocm = self.create_publisher(OffboardControlMode, args.topic_prefix + "/fmu/in/offboard_control_mode", qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, args.topic_prefix + "/fmu/in/trajectory_setpoint", qos)
        self.pub_status = self.create_publisher(String, args.topic_prefix + "/navigation/task_status", 1)
        self.pub_event = self.create_publisher(String, args.topic_prefix + "/navigation/task_event", 10)
        self.instance_id = str(time.time_ns())
        # 授权时效诊断（只读）：PILOT_LOG_ROOT 由 run_profile.sh 指向本轮 bt/。
        # [SIM-ONLY] 输入适配：uXRCE-DDS SYNCT 在 SITL 下会给同一条 uORB 消息
        # 产生「相同 timestamp、不同 timestamp_sample」的重复发布。严格门控把
        # 它判为 conflicting same-publication payload 并丢弃该流租约（stale_rc）。
        # 这里只过滤 stamp 不递增的重复帧，不修改任何时戳、门限或授权判据；
        # 实机不设置该环境变量，路径与 nano 完全一致。事件写入 authority_diag.csv。
        self.dds_dedup = os.environ.get("SIM_DDS_DEDUP") == "1"
        self.last_stamp = {}
        self.last_sample = {}
        self.diag = self.diag_file = None
        log_root = os.environ.get("PILOT_LOG_ROOT")
        if log_root:
            try:
                pathlib.Path(log_root).mkdir(parents=True, exist_ok=True)
                self.diag_file = open(str(pathlib.Path(log_root) / "authority_diag.csv"),
                                      "w", newline="")
                self.diag = csv.writer(self.diag_file)
                self.diag.writerow(["wall_us", "stream", "stamp_us", "sample_us",
                                    "wall_minus_stamp_ms", "wall_minus_sample_ms",
                                    "source_us", "samples_us", "reason"])
                self.diag_file.flush()
                # 诊断 IO 改为有界批量写：每个 clamp/dedup 事件不再同步
                # flush（原实现每次事件都落盘，位于订阅回调里）。行数与时间
                # 双上限；撤销类关键行仍立即落盘，内容与列完全不变。
                self.diag_pending = 0
                self.diag_flushed_at = time.monotonic()
            except OSError:
                self.diag = self.diag_file = None
                self.diag_pending = 0
                self.diag_flushed_at = 0.
        self.status_timer = self.create_timer(.2, self.publish_status)
        self.timer = self.create_timer(.05, self.on_tick)
        self.get_logger().info("Pilot priority: manually take off, verify Position, then select RC Offboard. "
                               "Missing physical switch DDS/health blocks control. dry_run=" + str(args.dry_run))

    def diag_event(self, row, important=False):
        """诊断行有界批量落盘：行数/时间双上限，撤销类关键行立即 flush。"""
        if not self.diag:
            return
        self.diag.writerow(row)
        self.diag_pending += 1
        now = time.monotonic()
        if important or self.diag_pending >= 64 or now - self.diag_flushed_at >= .3:
            self.diag_file.flush()
            self.diag_pending = 0
            self.diag_flushed_at = now

    def ingest(self, stream, m, payload):
        stamp = int(m.timestamp)
        # 只过滤「连续相同 stamp」的传输层重传。不能用 <= 的最大值语义：
        # 一条被 timesync 推到未来的时戳会让后续数秒内所有合法消息被误丢
        # （实测 dedup:local 被吞 101 条）。倒退/超前的时戳仍交给门控按原语义判定。
        if self.dds_dedup and stamp == self.last_stamp.get(stream):
            if self.diag:
                self.diag_event([time.time_ns() // 1000, "dedup:" + stream,
                                 stamp, int(getattr(m, "timestamp_sample", 0) or 0),
                                 "", "", self.last_stamp.get(stream), "",
                                 "dds_duplicate_stamp_dropped"])
            return
        if self.dds_dedup:
            self.last_stamp[stream] = stamp
        if self.authority.relax_freshness:
            # WSL 本地仿真（流程演示）：PX4 在 RC 通道值/档位恒定时不刷新
            # timestamp_sample，冻结的采样时戳会让 sample_age 无限增长。
            # 放松模式直接用发布时戳，配合 pilot_authority 的 relax 分支跳过时效。
            sample = stamp
        else:
            # 严格模式与 nano 实机同源：采样时戳取 timestamp_sample，
            # 仅当字段缺失或为 0 时才回退到 timestamp。
            sample = int(getattr(m, "timestamp_sample", 0) or 0) or stamp
        if self.dds_dedup:
            # uXRCE-DDS 的时间转换会让相邻两帧的 timestamp_sample 出现
            # 1~3 us 级倒退（三次 5 m 冷启动重复实测 diff 恒为 3 us）。
            # PilotAuthority 的严格单调判据会把整条 RC 租约 pop 掉
            # （stale_rc -> task revoked）。这里只把 <=10us 的抖动钳到同一
            # 采样时刻；10 us 仍比最小采样间隔 4 ms 小两个数量级，
            # 不会掩盖真实倒退、不推进样本时间、
            # 不改成接收时刻、不动任何门限；事件写入诊断 CSV 可审计。
            prev_s = self.last_sample.get(stream)
            if prev_s is not None and 0 <= prev_s - sample <= 10:
                if self.diag:
                    self.diag_event([time.time_ns() // 1000, "clamp:" + stream,
                                     stamp, sample, "", "", prev_s, "",
                                     "sample_us_jitter_clamped"])
                sample = prev_s
            self.last_sample[stream] = sample
        had = stream in self.authority.data
        a = self.authority
        wall = time.time_ns() // 1000
        repeats = a.sample_repeat_ignored.get(stream, 0)
        self.authority.ingest(stream, stamp, time.monotonic(), wall,
                              payload, sample_us=sample)
        # 取证：幂等忽略的重复样本单独记一行（不续租、不撤销），便于审计
        # 区分「重复投递被忽略」与「真实倒退触发撤销」。
        if self.diag and a.sample_repeat_ignored.get(stream, 0) > repeats:
            self.diag_event([wall, stream, stamp, sample,
                             round((wall - stamp) / 1e6, 3),
                             round((wall - sample) / 1e6, 3),
                             a.source.get(stream), a.samples.get(stream),
                             "sample_repeat_ignored"])
        # 取证：严格模式下一次时戳倒退/超龄会 pop 该流的 data（_reset_lease），
        # 并清掉 BT 的预热状态。这里只记录事件，不改任何判据或门限。
        if had and stream not in a.data and getattr(self, "diag", None):
            self.diag_event([wall, stream, stamp, sample,
                             round((wall - stamp) / 1e6, 3),
                             round((wall - sample) / 1e6, 3),
                             a.source.get(stream), a.samples.get(stream),
                             a.reason], important=True)
        self.check_takeover()

    def check_takeover(self):
        if self.authority.started and not self.authority.authorized(time.monotonic()):
            self.stop(self.authority.reason)

    def on_status(self, m):
        self.ingest("status", m, dict(armed=m.arming_state == 2, nav_state=m.nav_state,
                                     intended_mode=getattr(m, "nav_state_user_intention", -1), failsafe=m.failsafe))

    def on_rc(self, m):
        self.ingest("rc", m, dict(valid=m.valid, data_source=m.data_source))

    def on_switches(self, m):
        self.ingest("switches", m, {k: getattr(m, k) for k in
                    ("mode_slot", "offboard_switch", "return_switch", "loiter_switch", "kill_switch", "termination_switch")})

    def on_land(self, m):
        self.ingest("land", m, dict(landed=m.landed, freefall=m.freefall))

    def on_local(self, m):
        self.ingest("local", m, dict(position=(float(m.x), float(m.y), float(m.z)),
                    velocity=(float(m.vx), float(m.vy), float(m.vz)), heading=float(m.heading),
                    valid=bool(m.xy_valid and m.z_valid and m.v_xy_valid and m.v_z_valid
                               and getattr(m, "heading_good_for_control", False)),
                    resets=(m.xy_reset_counter, m.z_reset_counter, m.heading_reset_counter)))

    def on_health(self, m):
        try:
            p = json.loads(m.data)
            if not isinstance(p, dict):
                raise ValueError("health must be an object")
            self.authority.ingest("health", p.get("timestamp_us"), time.monotonic(), time.time_ns()//1000, p)
        except (ValueError, TypeError):
            self.authority.data.pop("health", None)
        self.check_takeover()

    def stop(self, reason):
        if self.done:
            return
        self.authority.revoke(reason)
        self.phase = "REVOKED"
        self.record(reason)
        self.sp = self.target = None
        self.done = True
        self.timer.cancel()
        self.offboard_ready = False
        self.event("task_revoked", reason)
        self.publish_status()
        self.get_logger().warning("Task revoked: " + reason + "; PX4/pilot own recovery; restart and cycle selector for a new task")

    def heartbeat(self):
        if self.args.dry_run or self.done:
            return False
        now = time.monotonic()
        if not (self.authority.authorized(now) if self.authority.started else self.authority.prestream(now)):
            return False
        msg = OffboardControlMode(timestamp=time.time_ns()//1000, position=True)
        self.pub_ocm.publish(msg)
        return True

    def reset_prestream(self):
        self.prestream_since = self.last_prestream = self.selector_since = None
        self.offboard_ready = False
        self.prestream_duration = 0.

    def prepare_acquisition(self, now):
        self.authority.learn_manual_selector(now)
        # [SIM-ONLY] 链路探针：跳过真人拨档时序相关的 ready_reason 检查。
        # 只作用于准备阶段；started 之后 authorized() 的持续检查仍走原逻辑。
        reason = "" if os.environ.get("PILOT_LINK_PROBE") == "1" \
            else self.authority.ready_reason(now)
        if reason:
            self.reset_prestream()
            self.authority.reason = reason
            return False
        # 数据未齐时直接等待，不能让下面的 selector() 抛 KeyError。
        switches = self.authority.data.get("switches")
        status = self.authority.data.get("status")
        if switches is None or status is None:
            return False
        selected = self.authority.selector(switches)
        mode = status["nav_state"]
        if selected:
            # [SIM-ONLY] 链路探针模式：飞手按脚本切档，不存在真实飞行员的
            # "先看到 READY 再拨档" 时序，原生 cycle 保护会永久要求重新循环
            # （实测 cycle_rc_selector_after_prestream 5549 次、行为树从未 started）。
            # 只在 PILOT_LINK_PROBE=1 时直接尝试 acquire；真实链路逻辑完全不变。
            if os.environ.get("PILOT_LINK_PROBE") == "1":
                if self.selector_since is None:
                    self.selector_since = now
                if self.authority.acquire(now):
                    self.offboard_ready = False
                    return True
                if self.heartbeat():
                    self.last_prestream = now
                return False
            # Selecting early must never latch a delayed auto-start. Cycle the
            # physical switch after the READY indication instead.
            if not self.offboard_ready or self.last_prestream is None or now-self.last_prestream > .2:
                self.reset_prestream()
                self.authority.reason = "cycle_rc_selector_after_prestream"
                return False
            if self.selector_since is None:
                self.selector_since = now
            if now-self.selector_since > 3.:
                self.reset_prestream()
                self.authority.reason = "offboard_transition_timeout_cycle_selector"
                return False
            if self.authority.acquire(now):
                self.offboard_ready = False
                return True
            # Continue proof-of-life during switch -> commander mode latency.
            if self.heartbeat():
                self.last_prestream = now
            return False
        if mode == 14:
            self.reset_prestream()
            self.authority.reason = "waiting_for_manual_mode"
            return False
        if self.heartbeat():
            if self.last_prestream is None or now-self.last_prestream > .2:
                self.prestream_since = now
                self.offboard_ready = False
            self.last_prestream = now
            self.prestream_duration = now-self.prestream_since
            self.selector_since = None
            if now-self.prestream_since >= 1.2:
                if not self.offboard_ready:
                    self.get_logger().info("OFFBOARD READY: pilot may now select Offboard")
                    self.event("offboard_ready")
                self.offboard_ready = True
            self.authority.reason = "offboard_ready" if self.offboard_ready else "prestream_warming"
        else:
            self.reset_prestream()
            self.authority.reason = "cycle_rc_selector_before_start"
        return False

    def event(self, name, reason=""):
        self.event_sequence += 1
        self.pub_event.publish(String(data=json.dumps(dict(schema_version=1,
            instance_id=self.instance_id, timestamp_us=time.time_ns()//1000,
            sequence=self.event_sequence, event=name, reason=reason))))

    def complete(self):
        self.completed = True
        self.get_logger().info("TASK COMPLETED: arrival dwell passed; holding until pilot takeover")
        self.event("task_completed")

    def publish_status(self):
        now = time.monotonic()
        reason = self.authority.ready_reason(now)
        p = dict(schema_version=1, instance_id=self.instance_id, timestamp_us=time.time_ns()//1000,
                 phase=self.phase, reason=reason or self.authority.reason,
                 offboard_ready=self.offboard_ready and not reason and not self.done,
                 prestream_seconds=self.prestream_duration,
                 task_completed=self.completed, control_active=self.authority.started and not self.done and not reason,
                 dry_run=self.args.dry_run, ages=self.authority.ages(now),
                 rc_three_position=self.authority.three_position,
                 rc_manual_slots=self.authority.manual_slots,
                 position=list(self.pos) if self.pos else None,
                 target=list(self.target) if self.target else None)
        self.pub_status.publish(String(data=json.dumps(p, allow_nan=False)))

    def publish_stream(self):
        # Final publication boundary; callers cannot bypass authority/dry-run.
        # 注意 prestream 也要放行：PX4 进入 Offboard 模式前必须先看到 trajectory_setpoint
        # 流，而交接判据 ready_reason() 又要求 nav_state 已经是 OFFBOARD ——
        # 两者互相等待会死锁，实测卡在 offboard_transition_timeout_cycle_selector。
        # prestream() 本身已限定「未 started + 已见过手动档 + nav_state 仍非 OFFBOARD」，
        # 所以这里放行不会让行为树在飞行员切档前就控制飞机。
        now = time.monotonic()
        if self.args.dry_run or self.done:
            return
        if not (self.authority.authorized(now) or self.authority.prestream(now)):
            return
        if self.sp is None or not all(math.isfinite(v) for v in self.sp):
            if self.authority.prestream(now):
                return   # 预热阶段尚未建立锚点，本轮不发，也不判故障
            self.stop("invalid_setpoint")
            return
        self.heartbeat()
        msg = TrajectorySetpoint()
        msg.timestamp = time.time_ns()//1000
        msg.position = list(self.sp)
        msg.velocity = msg.acceleration = msg.jerk = [NAN, NAN, NAN]
        msg.yaw, msg.yawspeed = self.yaw, NAN
        self.pub_sp.publish(msg)

    def record(self, reason=""):
        if self.writer:
            empty = (NAN, NAN, NAN)
            self.writer.writerow([time.time_ns()//1000, time.monotonic(), self.phase,
                                  *(self.pos or empty), *(self.sp or empty), *(self.target or empty), reason])
            self.log_file.flush()

    def on_tick(self):
        if self.done:
            return
        now = time.monotonic()
        if self.args.dry_run:
            if now - self.started_at >= self.args.dry_run_secs:
                self.stop("dry_run_complete")
            return
        if os.path.exists(ABORT_FILE):
            self.stop("operator_abort_file")
            return
        if not self.authority.started:
            if self.prepare_acquisition(now):
                local = self.authority.data["local"]
                self.home = self.sp = self.target = tuple(local["position"])
                self.pos, self.vel = self.home, tuple(local["velocity"])
                self.yaw = local["heading"]
                self.phase, self.last_tick = "NAVIGATE", now
                self.get_logger().info("RC Offboard acquired; anchor=" + str(self.home))
                self.event("task_started")
            else:
                if self.args.pilot_timeout and now - self.started_at > self.args.pilot_timeout:
                    self.stop("pilot_wait_timeout")
                    return
                if self.authority.reason != self.last_note:
                    self.last_note = self.authority.reason
                    self.get_logger().info("Waiting: " + self.last_note)
                # 等待接管阶段也要落盘：否则「行为树从未 started」这种情况
                # 在采样里完全看不出来（csv 只有表头），排查时无法区分
                # 「没接管」与「接管后没动」。
                self.record("wait:" + str(self.authority.reason))
                return
        if not self.authority.authorized(now):
            self.record("stop:" + self.authority.reason)
            self.stop(self.authority.reason)
            return
        local = self.authority.data["local"]
        self.pos, self.vel = tuple(local["position"]), tuple(local["velocity"])
        if math.dist(self.pos, self.home) > self.args.max_radius:
            self.record("stop:navigation_radius_exceeded")
            self.stop("navigation_radius_exceeded")
            return
        if abs(self.pos[2] - self.home[2]) > self.args.max_alt_error:
            self.record("stop:altitude_error")
            self.stop("altitude_error")
            return
        if math.dist(self.pos, self.sp) > self.args.max_err:
            self.record("stop:tracking_error")
            self.stop("tracking_error")
            return
        try:
            self.tree.tick(self, now)
        except RuntimeError as exc:
            self.stop(str(exc))
            return
        self.last_tick = now
        self.publish_stream()
        self.record()

    def destroy_node(self):
        if self.diag_file:
            try:
                if self.diag_pending:
                    self.diag_file.flush()
            finally:
                self.diag_file.close()
            self.diag = self.diag_file = None
        if self.log_file:
            self.log_file.close()
        return super().destroy_node()


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mission", choices=("line", "square"), default="line")
    ap.add_argument("--distance", type=float, default=10.)
    ap.add_argument("--side", type=float, default=1.)
    ap.add_argument("--speed", type=float, default=.25)
    ap.add_argument("--max-radius", type=float, default=0.)
    ap.add_argument("--max-alt-error", type=float, default=.5)
    ap.add_argument("--max-err", type=float, default=.8)
    ap.add_argument("--rc-offboard-slot", type=int, choices=range(7), default=0,
                    help="0: dedicated switch; 1..6: verified COM_FLTMODE slot")
    ap.add_argument("--rc-three-position", action="store_true",
                    help="learn Altitude/Position detents, require the third detent plus actual Offboard")
    ap.add_argument("--pilot-offboard", action="store_true", help="compatibility alias; pilot control is mandatory")
    ap.add_argument("--pilot-timeout", type=float, default=600., help="seconds to wait; 0 waits indefinitely at boot")
    ap.add_argument("--topic-prefix", default="")
    ap.add_argument("--dry-run", action="store_true", help="publishes no control messages")
    ap.add_argument("--dry-run-secs", type=float, default=10.)
    ap.add_argument("--log", default="")
    args = ap.parse_args(argv)
    for name in ("distance", "side", "speed", "max_alt_error", "max_err", "dry_run_secs"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            ap.error(name + " must be finite and positive")
    if not math.isfinite(args.pilot_timeout) or args.pilot_timeout < 0:
        ap.error("pilot-timeout must be finite and nonnegative")
    if args.rc_three_position and args.rc_offboard_slot:
        ap.error("rc-three-position cannot be combined with rc-offboard-slot")
    if args.speed > 1.:
        ap.error("speed must be <= 1 m/s for these trials")
    if not math.isfinite(args.max_radius) or args.max_radius < 0:
        ap.error("max-radius must be finite and nonnegative")
    if not args.max_radius:
        args.max_radius = (args.distance if args.mission == "line" else args.side*math.sqrt(2)) + 1.
    p = args.topic_prefix
    if p and (not p.startswith("/") or p.endswith("/") or "//" in p
              or any(not (c.isalnum() or c in "/_") for c in p)):
        ap.error("topic-prefix must be empty or /namespace without trailing slash")
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = SquareBT(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        node.stop("operator_interrupt")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.authority.reason in ("pilot_takeover", "dry_run_complete") else 1


if __name__ == "__main__":
    sys.exit(main())
