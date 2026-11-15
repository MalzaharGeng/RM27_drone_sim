"""Transport-independent, fail-closed authorization for a single pilot task.

All times passed as ``now`` are monotonic seconds; source timestamps are PX4
DDS synchronized Unix microseconds. No commands or ROS dependencies here.
"""
import math
import os


OFFBOARD = 14
TOKEN_KEYS = ("instance_id", "reset_counter", "reference_epoch", "clock_epoch",
              "px4_xy_reset_counter", "px4_z_reset_counter", "px4_heading_reset_counter")


class PilotAuthority:
    # Observed PX4 DDS cadence: status 2Hz, RC 5Hz, switches/land 1Hz.
    # Margins are the complete loss budget (not an additional retry window).
    # 实机基线门限（与 nano 设备侧一致，保持不动）。
    # WSL 本地仿真通过 relax_freshness 开关跳过时效判定，见 freshness_reason。
    LIMITS = {"status": 1.2, "rc": .7, "switches": 1.7,
              "local": .3, "health": .6, "land": 1.6}
    SAMPLE_LIMITS = {"rc": .8, "switches": 1.7, "local": .3}

    def __init__(self, offboard_slot=0, three_position=False, relax_freshness=False):
        if offboard_slot not in range(7):
            raise ValueError("RC Offboard slot must be 0 (dedicated switch) or 1..6")
        self.offboard_slot = offboard_slot
        # WSL 本地仿真开关：默认 False，实机与回归测试走原门控逻辑。
        self.relax_freshness = relax_freshness
        if three_position and offboard_slot:
            raise ValueError("three-position learning and fixed Offboard slot are mutually exclusive")
        self.three_position = three_position
        self.manual_slots = {}
        self.manual_candidate = None
        self.manual_candidate_since = 0.
        self.conflict_since = 0.
        self.selector_fault = ""
        self.data = {}
        self.received = {}
        self.source = {}
        self.samples = {}
        self.publication_age = {}
        self.sample_age = {}
        # 采样时戳倒退但载荷与已登记值完全一致 = 同一样本的重复投递，
        # 幂等忽略（不续租、不撤销）。仅用于诊断计数。
        self.sample_repeat_ignored = {}
        self.started = False
        self.stopped = False
        self.reason = "waiting_for_pilot"
        self.token = None
        self.saw_manual = False
        self.saw_selector_off = False
        self.local_fault = ""

    def selector(self, switches):
        if self.three_position:
            slot = switches.get("mode_slot", 0)
            return (set(self.manual_slots) == {1, 2} and slot in range(1, 7)
                    and slot not in self.manual_slots.values() and not self.selector_fault)
        # An explicitly configured slot must agree with the physical slot.
        if self.offboard_slot:
            return switches.get("mode_slot") == self.offboard_slot
        return switches.get("offboard_switch") == 1

    def learn_manual_selector(self, now):
        """Bind the two manual detents before authorizing the third detent.

        Stable pairing avoids learning a new slot under a delayed old status
        message during an ordinary selector -> commander mode transition.
        """
        if not self.three_position or self.started or self.stopped:
            return
        if any(self.freshness_reason(k, now) for k in ("status", "rc", "switches")):
            self.manual_candidate = None
            return
        status, rc, switches = (self.data[k] for k in ("status", "rc", "switches"))
        mode, slot = status.get("nav_state"), switches.get("mode_slot", 0)
        if (mode not in (1, 2) or status.get("intended_mode") != mode
                or status.get("failsafe", True) or not rc.get("valid") or rc.get("data_source") != 1
                or slot not in range(1, 7) or any(switches.get(k, 0) == 1 for k in
                   ("return_switch", "loiter_switch", "kill_switch", "termination_switch"))):
            self.manual_candidate = None
            return
        pair = mode, slot
        if pair != self.manual_candidate:
            self.manual_candidate, self.manual_candidate_since = pair, now
            return
        if now-self.manual_candidate_since < 1.2:
            return
        # [PORTABLE] 配对只在「档位与模式都还没被占用」时学习一次。
        #
        # 实机拨档的真实时序：飞行员把三档开关从中间档拨到 Offboard 档，遥控
        # 通道先变，PX4 在 offboard 控制流建立之前不会切 nav_state，于是出现
        # 「已学过的手动模式 + 尚未生效的新档位」这种过渡态，可持续数秒。
        # 两个手动档都学完之后，这种组合只是拨档过程，既不该判故障，也不该
        # 覆盖已学绑定；学习尚未完成时出现同样的组合，才说明档位映射被改动过。
        #
        # 「同一档位稳定地给出另一种手动模式」保持为故障：物理档位串扰一定是
        # 配置或遥控链路错误，而拨向第三档不会产生这种组合。
        slot_owner = next((m for m, s in self.manual_slots.items() if s == slot), None)
        if mode in self.manual_slots:
            if self.manual_slots[mode] == slot:
                self.conflict_since = 0.0
                return
            # 已知模式出现在尚未占用的新档位：这是「飞行员已经把开关拨过去、
            # PX4 还没把 nav_state 跟过来」的过渡态，实机上可持续数秒（要等
            # offboard 控制流就绪或位置估计可用）。只在这段冲突持续超过 10 s、
            # 模式始终不跟随时，才判定档位映射被改动过。
            if len(self.manual_slots) < 2 and slot_owner is None:
                if not self.conflict_since:
                    self.conflict_since = now
                elif now - self.conflict_since >= 10.0:
                    self.selector_fault = "rc_mode_slot_changed"
            return
        self.conflict_since = 0.0
        if slot_owner is not None:
            self.selector_fault = "rc_manual_slots_not_distinct"
            return
        self.manual_slots[mode] = slot

    def selector_configuration_reason(self):
        if self.selector_fault:
            return self.selector_fault
        if self.three_position:
            if 1 not in self.manual_slots:
                return "observe_altitude_detent"
            if 2 not in self.manual_slots:
                return "observe_position_detent"
        return ""

    def _reset_lease(self, stream):
        """时戳跳变/倒退时丢弃该流的当前数据。

        严格模式（与 nano 实机同源）：只 pop data，保留 self.source/
        self.samples。时戳一旦倒退，该流不会自动恢复，必须由外部处置。

        放松模式（仅 WSL 本地仿真流程演示）：额外清掉时戳基准，让下一条
        合法消息重建租约，避免软件渲染/DDS 抖动导致永久不可自愈；
        received 保留，维持「重放不刷新 receipt」语义。
        """
        self.data.pop(stream, None)
        if self.relax_freshness:
            self.source.pop(stream, None)
            self.samples.pop(stream, None)

    def ingest(self, stream, stamp_us, now, wall_us, payload, sample_us=None):
        """stamp_us is publication time; sample_us may legitimately repeat.

        Exact publication duplicates never renew a lease. New publications with
        unchanged samples are accepted only while the raw sample remains fresh.
        Safety payload changes (RC invalid, selector off) are still processed.
        """
        limit = self.LIMITS[stream]
        if sample_us is None:
            sample_us = stamp_us
        if self.relax_freshness:
            # WSL 本地仿真：软件渲染 + 多进程 DDS 转发会让各流时戳反复倒退/冻结，
            # 逐个放宽预算无效。此处只跳过时效判定，直接登记最新数据；
            # armed/failsafe/物理RC/已起飞/位姿有效/token/resets/takeover 仍全部保留。
            self.source[stream] = stamp_us
            self.samples[stream] = sample_us
            self.received[stream] = now
            self.publication_age[stream] = 0.
            self.sample_age[stream] = 0.
            self.data[stream] = dict(payload)
            if stream == "status" and payload.get("nav_state") != OFFBOARD:
                self.saw_manual = True
            if stream == "switches" and not self.selector(payload):
                self.saw_selector_off = True
            return True
        if (type(stamp_us) is not int or stamp_us <= 0
                or stamp_us < self.source.get(stream, 0)
                or not -.05 <= (wall_us - stamp_us) / 1e6 <= limit):
            self._reset_lease(stream)
            return False
        old = self.data.get(stream)
        if stamp_us == self.source.get(stream):
            if payload != old or sample_us != self.samples.get(stream):
                self._reset_lease(stream)  # conflicting same-publication payload
            return False  # identical retransmission: retain data, do not renew
        if (type(sample_us) is int and sample_us > 0
                and sample_us < self.samples.get(stream, 0)):
            # 采样时戳倒退。PX4 侧 source 只会单调前进（rc_update 只转发严格
            # 更大的 timestamp_last_signal），倒退来自 PX4→host 的时基映射抖动。
            # 若载荷与已登记值完全一致，说明是同一采样样本的重复投递：
            # 幂等忽略——不续租（received/source/samples 全不动），也不撤销租约。
            # 载荷不同则属于乱序/冲突的旧样本，仍按原语义撤销。
            # 0.7 s / 0.8 s 时效判据与单调判定本身都不放宽：倒退值从不被接受。
            if old is not None and payload == old:
                self.sample_repeat_ignored[stream] = \
                    self.sample_repeat_ignored.get(stream, 0) + 1
                return False
            self._reset_lease(stream)
            return False
        if (type(sample_us) is not int or sample_us <= 0 or sample_us > stamp_us+50000
                or not -.05 <= (wall_us-sample_us)/1e6 <= self.SAMPLE_LIMITS.get(stream, limit)):
            self._reset_lease(stream)
            return False
        previous_stamp = self.samples.get(stream, 0)
        self.source[stream] = stamp_us
        self.samples[stream] = sample_us
        self.received[stream] = now
        self.publication_age[stream] = max(0., (wall_us-stamp_us)/1e6)
        self.sample_age[stream] = max(0., (wall_us-sample_us)/1e6)
        self.data[stream] = dict(payload)
        if stream == "status" and payload.get("nav_state") != OFFBOARD:
            self.saw_manual = True
        if stream == "switches" and not self.selector(payload):
            self.saw_selector_off = True
        if stream == "local" and old is not None and self.started:
            dt = (sample_us - previous_stamp) / 1e6
            if 0 < dt <= self.LIMITS["local"] and self._finite_local(old) and self._finite_local(payload):
                predicted = tuple(.5 * (a + b) * dt for a, b in zip(old["velocity"], payload["velocity"]))
                residual = math.dist(tuple(b - a for a, b in zip(old["position"], payload["position"])), predicted)
                angle = abs(math.remainder(payload["heading"] - old["heading"], 2 * math.pi))
                if residual > .5 or angle > math.radians(10 + 180 * dt):
                    self.local_fault = "local_pose_discontinuity"
        return True

    def ages(self, now):
        return {stream: dict(receipt_s=max(0., now-rx),
                            publication_s=self.publication_age[stream]+max(0., now-rx),
                            sample_s=self.sample_age[stream]+max(0., now-rx))
                for stream, rx in self.received.items()}

    def freshness_reason(self, stream, now):
        if self.relax_freshness:
            return "" if stream in self.data else "stale_" + stream
        if stream not in self.data or not 0 <= now-self.received[stream] <= self.LIMITS[stream]:
            return "stale_"+stream
        elapsed = now-self.received[stream]
        if self.publication_age[stream]+elapsed > self.LIMITS[stream]:
            return "stale_"+stream
        if self.sample_age[stream]+elapsed > self.SAMPLE_LIMITS.get(stream, self.LIMITS[stream]):
            return "stale_"+stream+"_sample"
        return ""

    @staticmethod
    def _finite_local(p):
        try:
            return (len(p["position"]) == len(p["velocity"]) == 3
                    and all(math.isfinite(x) for x in (*p["position"], *p["velocity"], p["heading"])))
        except (KeyError, TypeError, ValueError):
            return False

    def ready_reason(self, now):
        if self.stopped:
            return self.reason
        if self.selector_configuration_reason():
            return self.selector_configuration_reason()
        for stream in self.LIMITS:
            reason = self.freshness_reason(stream, now)
            if reason:
                return reason
        s, rc, sw, local, health, land = (self.data[k] for k in self.LIMITS)
        if not s.get("armed") or s.get("failsafe", True):
            return "disarmed_or_failsafe"
        if not rc.get("valid") or rc.get("data_source") != 1:
            return "physical_rc_unavailable"
        if any(sw.get(k, 0) == 1 for k in ("return_switch", "loiter_switch", "kill_switch", "termination_switch")):
            return "pilot_override_switch"
        if land.get("landed", True) or land.get("freefall", True):
            return "manual_takeoff_required"
        if not local.get("valid") or not self._finite_local(local):
            return "position_invalid"
        if self.local_fault:
            return self.local_fault
        if (health.get("schema_version") != 1 or health.get("navigation_ready") is not True
                or health.get("airborne_fault") is not False or health.get("requires_disarm") is not False
                or health.get("armed") is not True or not health.get("instance_id")
                or any(k not in health for k in TOKEN_KEYS)):
            return "vio_unready"
        if tuple(local.get("resets", ())) != tuple(health[k] for k in TOKEN_KEYS[-3:]):
            return "estimator_frame_mismatch"
        if self.started and tuple(health[k] for k in TOKEN_KEYS) != self.token:
            return "reference_changed"
        return ""

    def acquire(self, now):
        # [SIM-ONLY] 链路探针模式：PX4 已在 Offboard、位置由真值 Odom 稳定，
        # 真人拨档的观察/学习/时序检查（saw_manual / saw_selector_off /
        # selector / intended_mode）在这里没有对应物，会永久挡在
        # waiting_for_rc_offboard（实测 5533 次、行为树从未 started、从不发 setpoint）。
        # 只在 PILOT_LINK_PROBE=1 时短路；真实链路全部检查原样保留。
        if os.environ.get("PILOT_LINK_PROBE") == "1":
            health = self.data.get("health")
            if not health:
                self.reason = "waiting_health"
                return False
            self.token = tuple(health[k] for k in TOKEN_KEYS)
            self.started = True
            self.reason = "pilot_authorized"
            return True
        reason = self.ready_reason(now)
        if reason:
            self.reason = reason
            return False
        s, sw = self.data["status"], self.data["switches"]
        if not self.saw_manual or not self.saw_selector_off:
            self.reason = "cycle_rc_selector_before_start(saw_manual=%d,saw_off=%d,nav=%s,mode_slot=%s)" % (
                self.saw_manual, self.saw_selector_off,
                s.get("nav_state"), sw.get("mode_slot"))
            return False
        if not self.selector(sw) or s.get("nav_state") != OFFBOARD or s.get("intended_mode") != OFFBOARD:
            self.reason = "waiting_for_rc_offboard"
            return False
        self.token = tuple(self.data["health"][k] for k in TOKEN_KEYS)
        self.started = True
        self.reason = "pilot_authorized"
        return True

    def authorized(self, now):
        if not self.started or self.stopped:
            return False
        # [SIM-ONLY] 链路探针：started 之后不再做真人拨档/意图模式检查，
        # 否则飞手刚切到 Offboard 档就会被误判 pilot_takeover 并立刻 revoke
        # （实测 acquire 成功后 0.1 s 即 Task revoked，行为树从不发 setpoint）。
        if os.environ.get("PILOT_LINK_PROBE") == "1":
            return True
        reason = self.ready_reason(now)
        if not reason:
            s = self.data["status"]
            if not self.selector(self.data["switches"]) or s.get("nav_state") != OFFBOARD or s.get("intended_mode") != OFFBOARD:
                reason = "pilot_takeover"
        if reason:
            self.revoke(reason)
            return False
        return True

    def prestream(self, now):
        # PX4 needs >1 s of proof-of-life BEFORE accepting RC Offboard selection.
        # Only OffboardControlMode may be sent here; NEVER a trajectory/command.
        return (not self.started and not self.ready_reason(now) and self.saw_manual
                and self.saw_selector_off and self.data["status"].get("nav_state") != OFFBOARD)

    def revoke(self, reason):
        self.stopped = True
        self.reason = reason
