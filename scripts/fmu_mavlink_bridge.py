#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fmu_mavlink_bridge —— 用 MAVLink 假装成一架"带 uXRCE-DDS 的 PX4"

为什么需要它
  行为树脚本 px4_square_bt.py 发的是 ROS 2 uORB 消息（/fmu/in/*），
  在实机上靠机载 uXRCE-DDS Agent + PX4 里的 uxrce_dds_client 打通。
  仿真机（WSL）里没装 Agent 时，这些消息就发到了空气里。

做法
  行为树一行不改。本节点：
    /fmu/in/offboard_control_mode   → 只当"流在跑"的心跳，忽略内容
    /fmu/in/trajectory_setpoint     → SET_POSITION_TARGET_LOCAL_NED
    /fmu/in/vehicle_command         → SET_MODE / ARM_DISARM / NAV_LAND
    PX4 的 HEARTBEAT/LOCAL_POSITION_NED/ATTITUDE
                                    → /fmu/out/vehicle_status_v1 等消息
  于是行为树以为自己在跟飞控说话。

用法
  python3 fmu_mavlink_bridge.py                 # 自动扫 14540 / 14550
  python3 fmu_mavlink_bridge.py --url udpin:0.0.0.0:14540
  python3 fmu_mavlink_bridge.py --probe         # 只打印心跳，不建 ROS 话题
  python3 fmu_mavlink_bridge.py --preflight-timeout   # 演示"前检不通过"

依赖
  pip install pymavlink --break-system-packages
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
    VehicleAttitude,
    EstimatorStatusFlags,
)

try:
    from pymavlink import mavutil
except ImportError:
    print("缺少 pymavlink：pip install pymavlink --break-system-packages", file=sys.stderr)
    raise

NAN = float("nan")

ARMING_DISARMED = 1
ARMING_ARMED = 2
NAV_STATE_OFFBOARD = 14
NAV_STATE_AUTO_TAKEOFF = 17
NAV_STATE_AUTO_LAND = 18

CMD_DO_SET_MODE = 176
CMD_ARM_DISARM = 400
CMD_NAV_LAND = 21

MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_FRAME_LOCAL_NED = 1
# 只用位置：忽略 vx,vy,vz / ax,ay,az / force / yaw / yaw_rate
TYPE_MASK_POSITION_ONLY = 0b0000111111111000

# PX4 主模式编码：custom_mode = main_mode << 16 | sub_mode
PX4_MAIN_MODE_OFFBOARD = 6
PX4_MAIN_MODE_AUTO = 4

DEFAULT_URLS = [
    "udpin:0.0.0.0:14540",     # 常规：PX4 主动往 14540 打（MAVSDK 同款）
    "udpout:127.0.0.1:14540",  # 反向：我们先发，PX4 认源地址
    "udpin:0.0.0.0:14550",     # QGC 口，作最后兜底
]


def fill(msg, **kw):
    """px4_msgs 各版本字段名略有差异，存在才写，避免 AttributeError。"""
    for k, v in kw.items():
        if hasattr(msg, k):
            try:
                setattr(msg, k, v)
            except Exception:
                pass
    return msg


def stamp_us():
    return int(time.monotonic() * 1e6)


def euler_to_quat(roll, pitch, yaw):
    """ZYX 欧拉角 → 四元数 [w,x,y,z]（NED/FRD，与 px4_msgs 同义）"""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


class FmuBridge(Node):
    TICK_HZ = 50.0
    PUB_HZ = 20.0
    SP_MAX_AGE = 0.5     # 超过这么久没新 setpoint 就不再代发（让 PX4 触发失控保护）

    def __init__(self, args):
        super().__init__("fmu_mavlink_bridge")
        self.args = args
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        # ---- 假装出来的飞控话题（行为树订的就是这两个名字）----
        self.pub_status = self.create_publisher(VehicleStatus, "/fmu/out/vehicle_status_v1", qos)
        self.pub_local = self.create_publisher(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", qos)
        self.pub_att = self.create_publisher(VehicleAttitude, "/fmu/out/vehicle_attitude", qos)
        self.pub_est = self.create_publisher(EstimatorStatusFlags, "/fmu/out/estimator_status_flags", qos)

        # ---- 行为树的指令 ----
        self.create_subscription(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", self.on_sp, qos)
        self.create_subscription(OffboardControlMode, "/fmu/in/offboard_control_mode", self.on_ocm, qos)
        self.create_subscription(VehicleCommand, "/fmu/in/vehicle_command", self.on_cmd, qos)

        # ---- MAVLink 侧状态 ----
        self.mav = None
        self.target_sys = 1
        self.target_comp = 1
        self.hb_t = 0.0
        self.hb_count = 0
        self.armed = False
        self.custom_mode = 0
        self.lpos = None          # (x, y, z, vx, vy, vz)
        self.lpos_t = 0.0
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.att_t = 0.0
        self.sp = None            # (x, y, z, yaw)
        self.sp_t = 0.0
        self.ocm_t = 0.0
        self.ocm_seen = False
        self.stat_t = 0.0
        self.n_sp_fwd = 0
        self.n_cmd = 0
        self._warned = {}
        self._pos_hint = False

        self.timer = self.create_timer(1.0 / self.TICK_HZ, self.on_tick)

    # ================= MAVLink 连接 =================
    def connect(self):
        urls = [self.args.url] if self.args.url else list(DEFAULT_URLS)
        for url in urls:
            self.log("尝试连接 %s ..." % url)
            try:
                mav = mavutil.mavlink_connection(url, source_system=250, source_component=191)
            except Exception as e:
                self.log("  %s 打不开：%s" % (url, e), "WARN")
                continue
            hb = None
            t0 = time.time()
            while time.time() - t0 < 3.0:
                hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
                if hb is not None and hb.get_srcSystem() != 250:
                    break
                hb = None
            if hb is None:
                self.log("  %s 3s 内无心跳" % url, "WARN")
                try:
                    mav.close()
                except Exception:
                    pass
                continue
            self.mav = mav
            self.target_sys = hb.get_srcSystem()
            self.target_comp = hb.get_srcComponent()
            self.note_heartbeat(hb)
            self.log("链路通了：%s | sys=%d comp=%d type=%d autopilot=%d"
                     % (url, self.target_sys, self.target_comp,
                        hb.type, hb.autopilot))
            self.request_streams()
            return True
        return False

    def request_streams(self):
        """PX4 的 onboard 链路默认不一定推 LOCAL_POSITION_NED，主动要一次。"""
        # 首选：逐条消息设间隔（PX4 一定认）
        try:
            for msgid, hz in ((32, self.args.rate),   # LOCAL_POSITION_NED
                              (30, 20),               # ATTITUDE
                              (31, 10)):              # ATTITUDE_QUATERNION
                self.mav.mav.command_long_send(
                    self.target_sys, self.target_comp, 511, 0,   # SET_MESSAGE_INTERVAL
                    float(msgid), float(int(1e6 / hz)), 0, 0, 0, 0, 0)
        except Exception as e:
            self.log("SET_MESSAGE_INTERVAL 失败：%s" % e, "WARN")
        # 兜底：老式按组请求（部分固件只认这个）
        reqs = [
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION, self.args.rate),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 20),   # ATTITUDE
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10),   # VFR_HUD
        ]
        for sid, rate in reqs:
            try:
                self.mav.mav.request_data_stream_send(
                    self.target_sys, self.target_comp, sid, rate, 1)
            except Exception as e:
                self.log("请求数据流 %s 失败：%s" % (sid, e), "WARN")
        self.log("已请求数据流：LOCAL_POSITION_NED@%dHz ATTITUDE@20Hz" % self.args.rate)

    def log(self, msg, level="INFO"):
        print("[%s][%s] %s" % (time.strftime("%H:%M:%S"), level, msg), flush=True)

    # ================= 收：PX4 -> ROS =================
    def note_heartbeat(self, hb):
        self.hb_t = time.time()
        self.hb_count += 1
        self.armed = bool(hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
        self.custom_mode = hb.custom_mode

    def pump(self):
        for _ in range(200):          # 单次最多处理 200 帧，避免饿死定时器
            m = self.mav.recv_match(blocking=False)
            if m is None:
                break
            try:
                self.on_mavlink_msg(m)
            except Exception as e:
                # 单条消息出错不影响后续收包；同类错误限速打印
                key = (m.get_type(), str(e))
                if self._warned.get(key, 0) < 2:
                    self._warned[key] = self._warned.get(key, 0) + 1
                    self.log("跳过 %s：%s" % (m.get_type(), e), "WARN")

    def on_mavlink_msg(self, m):
        t = m.get_type()
        if t == "HEARTBEAT":
            if m.get_srcSystem() != 250:
                self.note_heartbeat(m)
        elif t == "LOCAL_POSITION_NED":
            self.lpos = (m.x, m.y, m.z, m.vx, m.vy, m.vz)
            self.lpos_t = time.time()
        elif t == "ATTITUDE":
            # 老式 ATTITUDE 只有欧拉角（rad）
            self.q = euler_to_quat(m.roll, m.pitch, m.yaw)
            self.att_t = time.time()
        elif t == "ATTITUDE_QUATERNION":
            # 新版：q 是长度 4 的数组；极老的方言里是 q1..q4
            if hasattr(m, "q"):
                self.q = (m.q[0], m.q[1], m.q[2], m.q[3])
            elif hasattr(m, "q1"):
                self.q = (m.q1, m.q2, m.q3, m.q4)
            self.att_t = time.time()

    def nav_state_from_mode(self):
        main = (self.custom_mode >> 16) & 0xFF
        sub = self.custom_mode & 0xFFFF
        if main == PX4_MAIN_MODE_OFFBOARD:
            return NAV_STATE_OFFBOARD
        if main == PX4_MAIN_MODE_AUTO:
            if sub == 6:      # AUTO_LAND
                return NAV_STATE_AUTO_LAND
            if sub == 5:      # AUTO_TAKEOFF
                return NAV_STATE_AUTO_TAKEOFF
        return 0

    def publish_out(self):
        now = time.time()
        ts = stamp_us()
        hb_fresh = (now - self.hb_t) < 2.0

        vst = fill(VehicleStatus(),
                   timestamp=ts,
                   arming_state=ARMING_ARMED if self.armed else ARMING_DISARMED,
                   nav_state=self.nav_state_from_mode(),
                   pre_flight_checks_pass=(hb_fresh and not self.args.preflight_timeout),
                   failsafe=False,
                   rc_signal_lost=False,
                   data_link_lost=False)
        self.pub_status.publish(vst)

        if self.lpos is not None:
            x, y, z, vx, vy, vz = self.lpos
            fresh = (now - self.lpos_t) < 1.0
            yaw = self.yaw()
            loc = fill(VehicleLocalPosition(),
                       timestamp=ts,
                       x=x, y=y, z=z,
                       vx=vx, vy=vy, vz=vz,
                       xy_valid=fresh, z_valid=fresh,
                       v_xy_valid=fresh, v_z_valid=fresh,
                       heading=yaw,
                       heading_good_for_control=fresh,
                       delta_xy=[NAN, NAN],
                       delta_z=NAN,
                       delta_heading=NAN)
            self.pub_local.publish(loc)

        if (now - self.att_t) < 1.0:
            att = fill(VehicleAttitude(), timestamp=ts,
                       timestamp_sample=ts, q=[float(v) for v in self.q])
            self.pub_att.publish(att)

        est = fill(EstimatorStatusFlags(), timestamp=ts,
                   cs_ev_pos=True, cs_ev_hgt=True,
                   ev_pos=True, ev_hgt=True, ev_yaw=True,
                   filter_fault=False, timeout=False, dead_reckoning=False)
        self.pub_est.publish(est)

    def yaw(self):
        w, x, y, z = self.q
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    # ================= 发：ROS -> PX4 =================
    def on_sp(self, m):
        yaw = float(m.yaw)
        if not math.isfinite(yaw):
            yaw = 0.0
        self.sp = (float(m.position[0]), float(m.position[1]), float(m.position[2]), yaw)
        self.sp_t = time.time()

    def on_ocm(self, m):
        self.ocm_t = time.time()
        self.ocm_seen = True

    def on_cmd(self, m):
        self.n_cmd += 1
        cmd = int(m.command)
        if self.mav is None:
            return
        if cmd == CMD_DO_SET_MODE:
            self.send_mode(int(m.param2))
        elif cmd == CMD_ARM_DISARM:
            self.send_arm(bool(m.param1 > 0.5))
        elif cmd == CMD_NAV_LAND:
            self.send_land()

    def send_sp(self):
        if self.sp is None:
            return
        if time.time() - self.sp_t > self.SP_MAX_AGE:
            return
        x, y, z, yaw = self.sp
        if not all(math.isfinite(v) for v in (x, y, z)):
            return
        self.mav.mav.set_position_target_local_ned_send(
            int((time.monotonic() * 1e3) % 2 ** 32),   # time_boot_ms
            self.target_sys, self.target_comp,
            MAV_FRAME_LOCAL_NED,
            TYPE_MASK_POSITION_ONLY,
            x, y, z,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0)
        self.n_sp_fwd += 1

    def send_mode(self, main_mode):
        custom = (int(main_mode) & 0xFF) << 16
        self.mav.mav.set_mode_send(self.target_sys,
                                   MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom)
        self.log("→ SET_MODE main=%d (custom=0x%X)" % (main_mode, custom))

    def send_arm(self, arm):
        self.mav.mav.command_long_send(
            self.target_sys, self.target_comp,
            CMD_ARM_DISARM, 0,
            1.0 if arm else 0.0, 0, 0, 0, 0, 0, 0)
        self.log("→ %s" % ("ARM" if arm else "DISARM"))

    def send_land(self):
        self.mav.mav.command_long_send(
            self.target_sys, self.target_comp,
            CMD_NAV_LAND, 0,
            0, 0, 0, NAN, 0, 0, 0)
        self.log("→ LAND")

    # ================= 主循环 =================
    def on_tick(self):
        try:
            self.pump()
        except Exception as e:
            self.log("收包异常：%s" % e, "WARN")
        self.publish_out()
        if self.mav is not None:
            try:
                self.send_sp()
            except Exception as e:
                self.log("发 setpoint 异常：%s" % e, "WARN")

        now = time.time()
        if now - self.stat_t >= 3.0:
            self.stat_t = now
            link = "OK" if (now - self.hb_t) < 2.0 else "LOST"
            self.log("[stat] link=%s hb=%d armed=%d mode=0x%X nav=%d | sp_fwd=%d cmd=%d | pos=%s att=%s ocm=%s"
                     % (link, self.hb_count, int(self.armed), self.custom_mode,
                        self.nav_state_from_mode(), self.n_sp_fwd, self.n_cmd,
                        "无" if (now - self.lpos_t) > 1.0 else "(%.2f,%.2f,%.2f)" % self.lpos[:3],
                        "有" if (now - self.att_t) < 1.0 else "无",
                        "有" if (now - self.ocm_t) < 2.0 else "无"))
            if (now - self.lpos_t) > 10.0 and not self._pos_hint:
                self._pos_hint = True
                self.log("10s 没收到 LOCAL_POSITION_NED：行为树会卡在“等视觉位置有效”。"
                         "手动补一条：mavlink 里发 SET_MESSAGE_INTERVAL(32)，或改 --rate 再试", "WARN")


def probe(urls):
    print("== MAVLink 探测（不建 ROS 话题）==", flush=True)
    for url in urls:
        print("-- %s" % url, flush=True)
        try:
            mav = mavutil.mavlink_connection(url, source_system=250, source_component=191)
        except Exception as e:
            print("   打不开：%s" % e, flush=True)
            continue
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=3.0)
        if hb is None:
            print("   3s 无心跳", flush=True)
            mav.close()
            continue
        print("   ✓ 心跳 sys=%d comp=%d type=%d autopilot=%d base_mode=0x%X custom=0x%X"
              % (hb.get_srcSystem(), hb.get_srcComponent(), hb.type, hb.autopilot,
                 hb.base_mode, hb.custom_mode), flush=True)
        mav.close()
        return True
    print("全部失败：确认 PX4 SITL 已启动，且 mavlink 实例在跑", flush=True)
    return False


def main():
    ap = argparse.ArgumentParser(description="MAVLink ↔ /fmu/* 桥（无 uXRCE-DDS 代理时的替代方案）")
    ap.add_argument("--url", default="", help="MAVLink 地址，留空自动扫")
    ap.add_argument("--probe", action="store_true", help="只探测链路")
    ap.add_argument("--preflight-timeout", action="store_true",
                    help="故意让 pre_flight_checks_pass=False，用来演示前检拦截")
    ap.add_argument("--rate", type=int, default=30, help="向 PX4 请求的位置流频率 Hz（默认 30）")
    args = ap.parse_args()

    if args.probe:
        sys.exit(0 if probe([args.url] if args.url else DEFAULT_URLS) else 1)

    rclpy.init()
    node = FmuBridge(args)
    ok = node.connect()
    if not ok and not args.url:
        node.log("自动扫描全失败。手动指定：--url udpin:0.0.0.0:14540", "ERR")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
