#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ORB_SLAM3 位姿 → PX4 注入（含真值对比模式）。

坐标链（三段，全部是常值旋转，与飞行姿态无关）：

    SLAM 世界系 W  = 第一帧相机系（x 右 / y 下 / z 前）
        │  R_BODY_CAM    相机光学系 → 机体系 FLU
        ▼
    初始机体系（固定）
        │  C(yaw0)       FLU → NED，yaw0 = 起飞时机头相对正北的偏航
        ▼
    NED

为什么需要 --yaw0：gz 里模型用 ENU、spawn 时 yaw=0 表示机头朝 +x（东），
换成 NED 就是 yaw=+90°。若沿用"朝北"的假设，位置会整体差一个 90° 旋转
（表现为 --compare 的 err 一直很大、且随距离线性增大）。

用法：
    # 1) 先校验（不注入，只看数字）
    python3 slam_pose_to_px4.py --compare
    # 2) 数值对得上再注入
    python3 slam_pose_to_px4.py --inject --origin 5 6 0.5
"""
import argparse
import math
import time

import numpy as np

# PX4 EKF2 视觉参数（与 vio_truth_bridge.py 保持一致）
VISION_PARAMS = [
    ('EKF2_EV_CTRL', 3), ('EKF2_HGT_REF', 3), ('EKF2_EV_DELAY', 0),
    ('EKF2_EV_POS_X', 0.0), ('EKF2_EV_POS_Y', 0.0), ('EKF2_EV_POS_Z', 0.0),
    ('EKF2_EV_QMIN', 0), ('EKF2_GPS_CTRL', 0),
]

# 相机光学系(右/下/前) → 机体系 FLU(前/左/上)
R_BODY_CAM = np.array([[0.0, 0.0, 1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0]])


def quat_to_R(q):
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


def flu_to_ned_matrix(yaw0_rad):
    """FLU → NED 的常值旋转（yaw0 = 起飞时机头相对正北，顺时针为正）。

    yaw0 = 0   机头朝北：N=f, E=-l, D=-u
    yaw0 = 90° 机头朝东：N=l, E=f, D=-u
    """
    c, s = math.cos(yaw0_rad), math.sin(yaw0_rad)
    return np.array([[c, s, 0.0],
                     [s, -c, 0.0],
                     [0.0, 0.0, -1.0]])


def euler_from_R(R):
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def slam_to_ned(pos_slam, quat_slam, yaw0_rad):
    """SLAM 位姿（世界=首帧相机系）→ 局部 NED（以起飞点为原点）。"""
    C = flu_to_ned_matrix(yaw0_rad)
    p_flu = R_BODY_CAM @ np.asarray(pos_slam, float)
    p_ned = C @ p_flu
    R_body_flu = R_BODY_CAM @ quat_to_R(quat_slam) @ R_BODY_CAM.T
    R_ned = C @ R_body_flu @ C.T
    return p_ned, euler_from_R(R_ned)


def enu_to_ned(pos_enu, origin_enu):
    """gz 真值（ENU，米）→ 相对起飞点的 NED。"""
    d = np.asarray(pos_enu, float) - np.asarray(origin_enu, float)
    return np.array([d[1], d[0], -d[2]])


def px4_clock_offset(mav, timeout=3.0):
    """用 MAVLink TIMESYNC 估算 (本地时钟 - 飞控时钟)，单位秒。

    vision_position_estimate 的 time_usec 必须是飞控自己的时间基准；
    直接发 0 或本地 epoch 会让 EKF2 判定数据过期而静默丢弃。

    部分 SITL 构建的 hrt_absolute_time 就是 CLOCK_REALTIME（即 epoch µs），
    这时正确 offset 为 0；若 TIMESYNC 给出明显不合理的值（如 >1 小时），
    说明对端的时间基准不可信，返回 None 让调用方回退。
    """
    from pymavlink import mavutil  # noqa: F401
    tc1 = int(time.time() * 1e6)
    t0 = time.time()
    while time.time() - t0 < timeout:
        mav.mav.timesync_send(tc1, 0)
        msg = mav.recv_match(type='TIMESYNC', blocking=True, timeout=0.5)
        if msg is not None and getattr(msg, 'ts1', 0) not in (0, None):
            off = (msg.ts1 - msg.tc1) / 1e6
            if abs(off) > 3600.0:
                print(f'[warn] TIMESYNC 偏差不合理 {off:+.0f}s，忽略')
                return None
            return off
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--compare', action='store_true', help='只对比真值，不注入')
    ap.add_argument('--inject', action='store_true', help='向 PX4 注入 VIO')
    ap.add_argument('--slam-topic', default='/slam/pose')
    # gz Pose_V 桥成 TFMessage 后，ROS 话题名沿用 gz 话题名
    ap.add_argument('--tf-topic', default='/world/rm27_world/dynamic_pose/info')
    ap.add_argument('--model', default='x500_mono_cam', help='真值 TF 里的机型名')
    ap.add_argument('--origin', nargs=3, type=float, default=[5.0, 6.0, 0.5],
                    metavar=('E', 'N', 'U'), help='起飞点 ENU（PX4_GZ_MODEL_POSE）')
    ap.add_argument('--yaw0', type=float, default=90.0,
                    help='起飞时机头相对正北的偏航（度）。gz 默认 ENU yaw=0 → 朝东 → 90')
    ap.add_argument('--mav', default='udpout:127.0.0.1:14540')
    ap.add_argument('--no-params', dest='set_params', action='store_false',
                    help='跳过 EKF2 参数重设（启动时已由 PX4_PARAM_* 设好）')
    ap.add_argument('--no-align', dest='align', action='store_false',
                    help='不按首帧对齐，直接比较绝对 NED（默认按首帧对齐，消掉原点差）')
    args = ap.parse_args()
    if not args.inject:
        args.compare = True
    yaw0 = math.radians(args.yaw0)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from tf2_msgs.msg import TFMessage

    state = {'slam': None, 'truth': None, 't_slam': 0.0,
             'n_slam': 0, 'n_tf': 0, 'n_tf_match': 0,
             'slam0': None, 'truth0': None}

    class Bridge(Node):
        def __init__(self):
            super().__init__('slam_pose_to_px4')
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(PoseStamped, args.slam_topic, self.on_slam, 10)
            self.create_subscription(TFMessage, args.tf_topic, self.on_tf, qos)
            # PX4 uORB timestamp 才是 vision_position_estimate 的正确时基。
            # 不同 SITL/实机构建的 hrt_absolute_time 可能是启动时间也可能是
            # epoch，所以直接从 /fmu/out/vehicle_status 的 timestamp 取，
            # 再用本地单调钟外推，避免 TIMESYNC 的歧义。
            self.px4_last = None       # (px4_us, local_mono)
            try:
                from px4_msgs.msg import VehicleStatus
                ver = getattr(VehicleStatus, 'MESSAGE_VERSION', 0)
                topic = '/fmu/out/vehicle_status' + (f'_v{ver}' if ver else '')
                tx_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                    durability=DurabilityPolicy.VOLATILE)
                self.create_subscription(VehicleStatus, topic, self.on_px4_status, tx_qos)
                print(f'[time] PX4 时基取 {topic}.timestamp')
            except Exception as e:  # px4_msgs 不可用时退化为 epoch
                print(f'[warn] 无法订阅 PX4 status（{e}），使用本机 epoch 时间戳')

        def px4_now_us(self):
            if self.px4_last is None:
                return None
            px4_us, mono = self.px4_last
            return int(px4_us + (time.monotonic() - mono) * 1e6)

        def on_px4_status(self, m):
            self.px4_last = (int(m.timestamp), time.monotonic())

        def on_slam(self, m):
            p = m.pose.position
            q = m.pose.orientation
            state['slam'] = ((p.x, p.y, p.z), (q.w, q.x, q.y, q.z))
            state['t_slam'] = time.time()
            state['n_slam'] += 1

        def on_tf(self, msg):
            state['n_tf'] += 1
            if state['n_tf'] == 1:
                ids = [tr.child_frame_id for tr in msg.transforms][:6]
                print(f'[tf] 首帧共 {len(msg.transforms)} 个 transform，前几个 child_frame_id: {ids}')
            # dynamic_pose/info 会给出模型自身以及各 link 的位姿，
            # 优先取 base_link，保证与飞控的机体原点一致。
            cand = [tr for tr in msg.transforms
                    if args.model in tr.child_frame_id.lstrip('/')]
            if not cand:
                return
            pick = next((tr for tr in cand
                         if tr.child_frame_id.rstrip('/').endswith('base_link')), cand[0])
            t, q = pick.transform.translation, pick.transform.rotation
            state['truth'] = ((t.x, t.y, t.z), (q.w, q.x, q.y, q.z))
            state['n_tf_match'] += 1

    mav = None
    clock_off = 0.0
    if args.inject:
        from pymavlink import mavutil
        mav = mavutil.mavlink_connection(args.mav, source_system=250, source_component=191)
        print(f'[mav] {args.mav}')
        for name, val in (VISION_PARAMS if args.set_params else []):
            ptype = (mavutil.mavlink.MAV_PARAM_TYPE_INT32 if float(val).is_integer()
                     else mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            for _ in range(5):
                mav.mav.param_set_send(1, 1, name.encode(), float(val), ptype)
                ack = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
                if ack and ack.param_id.rstrip('\x00') == name:
                    print(f'[param] {name} = {ack.param_value:g}')
                    break
        # 不要发 TIMESYNC 去“估算”PX4 时钟：PX4 的 tc1>0 分支会把它当成
        # 自己发起的同步包并更新滤波器，反而把 sync_stamp 的 offset 弄坏。
        # 时基统一走上面的 /fmu/out/vehicle_status.timestamp。

    rclpy.init()
    node = Bridge()
    print('等待 /slam/pose 与真值 TF ...')
    t0 = time.time()
    last = 0.0
    last_beat = 0.0
    n_sent = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            # 每 3 秒报一次收包情况：哪一路没到一目了然
            if now - last_beat > 3.0:
                age = (now - state['t_slam']) if state['t_slam'] else -1.0
                print(f"[wait] slam={state['n_slam']} (age {age:.1f}s) | "
                      f"tf={state['n_tf']} match={state['n_tf_match']} | "
                      f"tf_topic={args.tf_topic}")
                last_beat = now
            if state['slam'] is None:
                continue
            if now - state['t_slam'] > 1.0:
                continue   # SLAM 掉线，停止注入
            pos, quat = state['slam']
            p_ned, rpy = slam_to_ned(pos, quat, yaw0)

            if args.compare and state['truth'] is not None:
                truth_ned = enu_to_ned(state['truth'][0], args.origin)
                # SLAM 的世界原点 = 它初始化时相机所在位置，与真值的起飞点
                # 一般不同。默认比较「相对首帧的位移」，消掉这个常值偏移。
                if args.align:
                    if state['slam0'] is None:
                        state['slam0'] = p_ned.copy()
                    if state['truth0'] is None:
                        state['truth0'] = truth_ned.copy()
                    p_show = p_ned - state['slam0']
                    t_show = truth_ned - state['truth0']
                else:
                    p_show, t_show = p_ned, truth_ned
                err = p_show - t_show
                if now - last > 1.0:
                    print(f'ΔSLAM =({p_show[0]:+6.2f},{p_show[1]:+6.2f},{p_show[2]:+6.2f})  '
                          f'ΔTRUTH=({t_show[0]:+6.2f},{t_show[1]:+6.2f},{t_show[2]:+6.2f})'
                          f'  err=({err[0]:+5.2f},{err[1]:+5.2f},{err[2]:+5.2f})'
                          f'  yaw={math.degrees(rpy[2]):+6.1f}°')
                    last = now

            if mav is not None:
                usec = node.px4_now_us() or int((time.time() - clock_off) * 1e6)
                mav.mav.vision_position_estimate_send(
                    usec, float(p_ned[0]), float(p_ned[1]), float(p_ned[2]),
                    float(rpy[0]), float(rpy[1]), float(rpy[2]))
                n_sent += 1
                if n_sent % 90 == 0:
                    print(f'[inject] {n_sent / max(now - t0, 1e-6):.1f} Hz  '
                          f'NED=({p_ned[0]:+.2f},{p_ned[1]:+.2f},{p_ned[2]:+.2f})')
            time.sleep(0.01)
    except KeyboardInterrupt:
        print('\n停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
