#!/usr/bin/env python3
"""5 m 链路探针 [SIM-ONLY]。

跳过 ORB-SLAM3 的实时 IMU 初始化，直接提供一个可控的「视觉里程计 + health」：
  * 从 PX4 的 vehicle_local_position 取位置/速度，按 vehicle_visual_odometry 格式回灌；
  * 按 Px4Bridge 的 health schema 发布 /vio/navigation_health，恒定 navigation_ready。

用途：单独验证 flyer / 行为树 / PX4 的
    起飞 -> Position 定点 -> Offboard -> 5 m -> 悬停 -> 切回 Position -> 降落
这条链路，不参与任何同源验收，也不代表 VIO 精度。
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from px4_msgs.msg import (
    VehicleLocalPosition,
    VehicleStatus,
    VehicleLandDetected,
    VehicleOdometry,
)

def qmul(a, b):
    """四元数乘法，入参/出参均为 (w, x, y, z)。"""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


# body(FLU) 相对世界(ENU) 的 Gazebo 四元数 → PX4 需要的 body(FRD) 相对 NED：
#   C_ned_enu = Rz(+90°)·Rx(180°)   （把 ENU 向量换到 NED 的纯旋转）
#   C_flu_frd = Rx(180°)            （左前上 → 前右下）
#   R_ned_frd = C_ned_enu · R_enu_flu · C_flu_frd
# 姿态必须与位置同源。只喂 yaw、roll/pitch 恒 0 的话，EKF 会认为机体永远水平，
# 而真实机体已倾斜，控制器就会继续朝同一方向加倾角，实测机头每秒上百度的自旋、
# 高度冲到 2 m 以上。
_QZ_NED = (math.cos(math.pi / 4.0), 0.0, 0.0, math.sin(math.pi / 4.0))
_QX_FRD = (0.0, 1.0, 0.0, 0.0)


def enu_flu_to_ned_frd(q):
    return qmul(qmul(qmul(_QZ_NED, _QX_FRD), q), _QX_FRD)


QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=5)


def topic(name, cls):
    v = getattr(cls, "MESSAGE_VERSION", 0)
    return "/fmu/out/" + name + ("_v%d" % v if v else "")


class LinkProbe(Node):
    def __init__(self):
        super().__init__("link_probe")
        self.local = None
        self.local_mono = None
        self.sim_us = None
        self.armed = False
        self.landed = True
        self.instance_id = "link_probe_%d" % (time.time_ns() // 1000)
        # Gazebo 真值（独立于 PX4 估计），避免用自身估计回灌造成正反馈自举。
        self.truth_ned = None
        self.truth_q = None
        self.truth_vel = (0.0, 0.0, 0.0)
        self.last_truth = None
        self.last_truth_t = None
        self.last_stamp_us = None
        self.truth_origin = None
        self.truth_logged = False
        self.last_probe_log = None

        self.create_subscription(
            VehicleLocalPosition, topic("vehicle_local_position", VehicleLocalPosition),
            self.on_local, QOS)
        self.create_subscription(
            VehicleStatus, topic("vehicle_status", VehicleStatus), self.on_status, QOS)
        self.create_subscription(
            VehicleLandDetected, topic("vehicle_land_detected", VehicleLandDetected),
            self.on_land, QOS)
        import os
        truth_topic = os.environ.get(
            "TRUTH_ROS_TOPIC", "/world/rm27/dynamic_pose/info")
        self.create_subscription(TFMessage, truth_topic, self.on_truth, QOS)
        # /clock 是 Gazebo 仿真时间，PX4 SITL（lockstep）的 uORB 时间戳与它同源。
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.get_logger().info("truth topic=" + truth_topic)

        self.health_pub = self.create_publisher(String, "/vio/navigation_health", 1)
        odom_topic = "/fmu/in/vehicle_visual_odometry"
        v = getattr(VehicleOdometry, "MESSAGE_VERSION", 0)
        if v:
            odom_topic += "_v%d" % v
        self.odom_pub = self.create_publisher(VehicleOdometry, odom_topic, QOS)
        self.create_timer(0.05, self.tick)
        self.get_logger().info("link probe ready; odom topic=" + odom_topic)

    def on_local(self, m):
        self.local = m
        self.local_mono = time.monotonic()

    def on_status(self, m):
        self.armed = (m.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def on_land(self, m):
        self.landed = bool(m.landed)

    def on_clock(self, m):
        self.sim_us = int(m.clock.sec) * 1000000 + int(m.clock.nanosec) // 1000

    def on_truth(self, m):
        """Gazebo 真值位姿 -> PX4 NED。这是独立观测，不与 PX4 估计形成自举。

        dynamic_pose/info 的 [0] 是机体模型根的世界绝对位姿，随飞行更新；
        [1] 之后是各 link 的局部坐标（恒定不随飞行变化），取到它们会得到
        "永远没起飞"的假位姿。桥接成 TFMessage 后 child_frame_id 全是空串，
        无法按名字挑机体，只能取 [0]。
        """
        if not m.transforms:
            return
        tf = m.transforms[0]
        p = tf.transform.translation
        q = tf.transform.rotation
        raw = (float(p.x), float(p.y), float(p.z))
        if self.truth_origin is None:
            self.truth_origin = raw
        d = tuple(raw[i] - self.truth_origin[i] for i in range(3))
        # gz 真值 ENU → PX4 局部 NED，含 spawn yaw 带来的 90° 旋转。
        # 与旧链路 slam_pose_to_px4.py:86-89 的 enu_to_ned() 一致：
        #     NED = (d[1], d[0], -d[2])
        pos = (d[1], d[0], -d[2])
        now = time.monotonic()
        # 用消息自带的仿真时间戳做差分：用主机单调钟会把调度抖动算进速度，
        # 实测噪声足以让 EKF 的速度环把飞机带着匀速漂 0.5 m/s。
        stamp_us = (int(tf.header.stamp.sec) * 1000000
                    + int(tf.header.stamp.nanosec) // 1000)
        dt = 0.0
        if self.last_stamp_us is not None and stamp_us > self.last_stamp_us:
            dt = (stamp_us - self.last_stamp_us) / 1e6
        elif self.last_truth_t is not None:
            # 桥接没带 header 时间戳时退回单调钟差分（此时精度差一些，但不会是 0）。
            dt = now - self.last_truth_t
        if 1e-4 < dt < 0.5:
            raw_v = tuple((pos[i] - self.last_truth[i]) / dt for i in range(3))
            alpha = 0.3
            self.truth_vel = tuple(
                (1.0 - alpha) * self.truth_vel[i] + alpha * raw_v[i]
                for i in range(3))
        self.last_truth = pos
        self.last_truth_t = now
        self.last_stamp_us = stamp_us
        self.truth_ned = pos
        qn = enu_flu_to_ned_frd((q.w, q.x, q.y, q.z))
        self.truth_q = [qn[1], qn[2], qn[3], qn[0]]   # PX4 顺序 x,y,z,w
        # 诊断：每 2 s 打一行真值 vs PX4 估计，用来判定坐标旋转/航向是否自洽。
        if self.last_probe_log is None or now - self.last_probe_log > 2.0:
            self.last_probe_log = now
            gz_yaw = math.degrees(math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            loc = ((float(self.local.x), float(self.local.y),
                    float(self.local.z))
                   if self.local is not None else (float("nan"),) * 3)
            hdg = (math.degrees(float(self.local.heading))
                   if self.local is not None else float("nan"))
            vxy = ((float(self.local.vx), float(self.local.vy))
                   if self.local is not None else (float("nan"),) * 2)
            self.get_logger().info(
                "[TRUTH] dworld=(%.2f,%.2f,%.2f) odom=(%.2f,%.2f,%.2f) "
                "gz_yaw=%.1f | px4=(%.2f,%.2f,%.2f) hdg=%.1f vxy=(%.2f,%.2f) "
                "tvxy=(%.2f,%.2f)"
                % (*d, *pos, gz_yaw, *loc, hdg, *vxy,
                   self.truth_vel[0], self.truth_vel[1]))

    def publish_odom(self):
        if self.truth_ned is None or self.local is None:
            return
        # 时间戳必须落在 PX4 自己的时基上（SITL 是 lockstep，uORB 时间戳等于
        # 仿真时间）。用 time.time_ns() 会差出上亿微秒；用 vehicle_local_position
        # 的 timestamp 也不行——EKF2 未就绪时它是 0，观测时间戳跟着变 0，EKF2
        # 判定数据无效而拒绝融合，于是永远起不来（实测 Preflight Fail:
        # ekf2 missing data）。这里统一取 /clock 的仿真时间。
        now_us = (self.sim_us if self.sim_us is not None
                  else time.time_ns() // 1000)
        o = VehicleOdometry()
        o.timestamp = now_us
        o.timestamp_sample = now_us
        o.pose_frame = VehicleOdometry.POSE_FRAME_NED
        o.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        o.position = list(self.truth_ned)
        o.velocity = list(self.truth_vel)
        # 姿态与位置同源：用真值四元数换算到 NED/FRD。否则 EKF 认为机体恒水平，
        # 与真实姿态矛盾，控制器会持续加倾角导致自旋。
        o.q = list(self.truth_q) if self.truth_q else [1.0, 0.0, 0.0, 0.0]
        o.angular_velocity = [float("nan")] * 3
        o.position_variance = [0.05, 0.05, 0.05]
        # 速度来自真值位置差分，帧间隔抖动会带来明显噪声；给它高方差让 EKF2
        # 以位置为主、速度只作参考。给 0.05 时飞机会被速度噪声带着匀速漂走
        # （实测 Altitude 起飞阶段 0.5 m/s 横漂 3 m）。
        o.velocity_variance = [1.0, 1.0, 1.0]
        o.orientation_variance = [0.05, 0.05, 0.05]
        o.reset_counter = 0
        o.quality = 50
        self.odom_pub.publish(o)

    def publish_health(self, now_us, steady_us):
        p = self.local
        h = dict(
            schema_version=1,
            instance_id=self.instance_id,
            timestamp_us=now_us,
            steady_us=steady_us,
            navigation_ready=True,
            ground_ready=True,
            ground_confirmed=True,
            armed=self.armed,
            # 起飞后按 armed 推断离地，避免 land_detected 抖动把行为树挡在
            # manual_takeoff_required（实测该 reason 出现过）。
            landed=not self.armed,
            airborne_fault=False,
            requires_disarm=False,
            ground_recoveries=0,
            reset_counter=0,
            reference_epoch=1,
            clock_epoch=0,
            # 与行为树订阅同一路 vehicle_local_position，直接用最新一帧的 reset counters。
            # 之前硬编码 0 会在回灌观测触发 EKF reset 后报 estimator_frame_mismatch。
            px4_xy_reset_counter=int(p.xy_reset_counter) if p else 0,
            px4_z_reset_counter=int(p.z_reset_counter) if p else 0,
            px4_heading_reset_counter=int(p.heading_reset_counter) if p else 0,
            reason="navigation_ready",
            gate_reason="publishing",
            fault_reason="",
        )
        m = String()
        m.data = json.dumps(h, allow_nan=False)
        self.health_pub.publish(m)

    def tick(self):
        if self.local is None:
            return
        now_us = time.time_ns() // 1000
        steady_us = time.monotonic_ns() // 1000
        self.publish_odom()
        self.publish_health(now_us, steady_us)


def main():
    rclpy.init()
    rclpy.spin(LinkProbe())


if __name__ == "__main__":
    main()
