#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真真值 → PX4 VIO 注入桥（SLAM 的占位替身）。

数据链:
    gz /world/<world>/dynamic_pose/info  (Pose_V, ENU)
        └─ ros_gz_bridge → tf2_msgs/TFMessage
            └─ 本节点: ENU/FLU → NED/FRD + 相对起飞原点
                └─ MAVLink VISION_POSITION_ESTIMATE (#102) → PX4 udp 14540
                    └─ EKF2 视觉位置融合 → offboard 位置控制可用

等 ORB_SLAM3 接通后，只需把本节点的输入从 TF 换成 SLAM 位姿，
后面的 NED 转换 / MAVLink 注入 / 参数设置都可以原样复用。

用法:
    # 终端1: 起仿真
    ~/drone_sim/start_rm27_px4.sh
    # 终端2: 桥出位姿
    source /opt/ros/lyrical/setup.bash
    ros2 run ros_gz_bridge parameter_bridge \
        '/world/rm27_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
    # 终端3: 本脚本
    python3 vio_truth_bridge.py --origin 5 6 0.5
"""
import argparse
import math
import sys
import time

import numpy as np

# ---------- 坐标系转换 ----------
# gz: 世界 ENU，机体系 FLU（x 前, y 左, z 上）
# PX4/MAVLink: 世界 NED，机体系 FRD（x 前, y 右, z 下）
C_ENU2NED = np.array([[0.0, 1.0, 0.0],
                      [1.0, 0.0, 0.0],
                      [0.0, 0.0, -1.0]])
D_FLU2FRD = np.diag([1.0, -1.0, -1.0])


def quat_to_R(q):
    """(w,x,y,z) → 3x3 旋转矩阵（body→world）。"""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def euler_from_R(R):
    """NED/FRD 旋转矩阵 → (roll, pitch, yaw)，rad。"""
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def enu_pose_to_ned(pos_enu, quat_enu, origin_enu):
    """gz 位姿 → NED 位置（相对原点）+ NED/FRD 欧拉角。"""
    p_rel = np.asarray(pos_enu, float) - np.asarray(origin_enu, float)
    p_ned = C_ENU2NED @ p_rel
    R_enu = quat_to_R(quat_enu)               # FLU body → ENU world
    R_ned = C_ENU2NED @ R_enu @ D_FLU2FRD     # FRD body → NED world
    return p_ned, euler_from_R(R_ned)


# ---------- MAVLink ----------
def connect_px4(addr, sysid=250, compid=191):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(addr, source_system=sysid, source_component=compid)
    print(f'[mav] {addr}  (sysid {sysid})')
    return m


def set_param(m, name, value, retries=8, timeout=1.5):
    """PARAM_SET + 等 ACK；int 参数用 MAV_PARAM_TYPE_INT32。"""
    is_int = float(value).is_integer()
    ptype = (mavutil.mavlink.MAV_PARAM_TYPE_INT32 if is_int
             else mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    for _ in range(retries):
        m.mav.param_set_send(m.target_system or 1, m.target_component or 1,
                             name.encode(), float(value), ptype)
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
            if msg and msg.param_id.rstrip('\x00') == name:
                ok = abs(msg.param_value - float(value)) < 1e-6
                print(f'[param] {name} = {msg.param_value:g} {"OK" if ok else "MISMATCH"}')
                return ok
    print(f'[param] {name} 设置超时（继续）', file=sys.stderr)
    return False


VISION_PARAMS = [
    ('EKF2_EV_CTRL', 3),    # 1=水平位置 2=垂直位置 → 用视觉位置与高度
    ('EKF2_HGT_REF', 3),    # 3 = Vision
    ('EKF2_EV_DELAY', 0),   # 仿真无延迟
    ('EKF2_EV_POS_X', 0.0),
    ('EKF2_EV_POS_Y', 0.0),
    ('EKF2_EV_POS_Z', 0.0),
    ('EKF2_EV_QMIN', 0),    # 只接受位置、不要求质量指标（真值注入）
    ('EKF2_GPS_CTRL', 0),   # 关掉 GPS 辅助，强制走视觉
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='rm27_world')
    ap.add_argument('--model', default='x500', help='TF child_frame_id 里匹配的模型名')
    ap.add_argument('--origin', nargs=3, type=float, default=[5.0, 6.0, 0.5],
                    metavar=('E', 'N', 'U'), help='仿真起飞点（ENU，米）')
    ap.add_argument('--mav', default='udpout:127.0.0.1:14540')
    ap.add_argument('--rate', type=float, default=30.0, help='注入频率 Hz')
    ap.add_argument('--no-attitude', action='store_true', help='只发位置，姿态置零')
    ap.add_argument('--no-params', action='store_true', help='不改 EKF 参数')
    args = ap.parse_args()

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from tf2_msgs.msg import TFMessage
    except ImportError as e:
        print('需要 ROS 2 环境: source /opt/ros/lyrical/setup.bash', file=sys.stderr)
        raise SystemExit(2) from e

    m = connect_px4(args.mav)
    if not args.no_params:
        for name, val in VISION_PARAMS:
            set_param(m, name, val)

    latest = {'stamp': 0.0, 'pos': None, 'quat': None}

    class PoseSink(Node):
        def __init__(self):
            super().__init__('vio_truth_bridge')
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(TFMessage, '/tf', self.on_tf, qos)
            self.get_logger().info(f'等 /tf 里的 {args.model} 位姿 ...')

        def on_tf(self, msg):
            for tr in msg.transforms:
                cid = tr.child_frame_id.lstrip('/')
                if args.model not in cid:
                    continue
                t, q = tr.transform.translation, tr.transform.rotation
                latest['pos'] = (t.x, t.y, t.z)
                latest['quat'] = (q.w, q.x, q.y, q.z)
                latest['stamp'] = time.time()
                return

    rclpy.init()
    node = PoseSink()
    period = 1.0 / max(args.rate, 1.0)
    sent = 0
    t_start = time.time()
    last_report = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=period)
            if latest['pos'] is None:
                continue
            if time.time() - latest['stamp'] > 1.0:
                continue   # 位姿过期，不注入陈旧数据
            p_ned, (roll, pitch, yaw) = enu_pose_to_ned(
                latest['pos'], latest['quat'], args.origin)
            if args.no_attitude:
                roll = pitch = yaw = 0.0
            usec = int((time.time() - t_start) * 1e6)
            m.mav.vision_position_estimate(
                usec, float(p_ned[0]), float(p_ned[1]), float(p_ned[2]),
                float(roll), float(pitch), float(yaw))
            sent += 1
            now = time.time()
            if now - last_report > 2.0:
                hz = sent / max(now - t_start, 1e-6)
                print(f'[vio] {hz:5.1f} Hz  NED=({p_ned[0]:+.2f},{p_ned[1]:+.2f},'
                      f'{p_ned[2]:+.2f})  yaw={math.degrees(yaw):+6.1f}°')
                last_report = now
    except KeyboardInterrupt:
        print('\n[vio] 停止注入')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
