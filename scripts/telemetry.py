#!/usr/bin/env python3
"""只读遥测采样器（正式版，替代此前从旧日志里复制来的 telemetry.py）。

记录内容：
  * PX4 各话题的「源时间戳」与「主机接收时刻」，用于分层统计延迟，
    不用接收时刻冒充采样时刻。
  * Px4Bridge 的 /vio/navigation_health 原始 JSON（真实门控输出）。
  * 可选：/slam/pose（SLAM 世界系原始位姿，源时间 = 图像 sim 时间戳）。

只订阅，不发布任何控制量。

用法：
  telemetry.py --out telemetry.csv [--slam-csv slam_pose.csv]
               [--period 0.1] [--seconds 0]
"""
import argparse
import csv
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from px4_msgs.msg import (EstimatorStatusFlags, TimesyncStatus,
                          VehicleLandDetected, VehicleLocalPosition,
                          VehicleStatus)
from std_msgs.msg import String

try:
    from geometry_msgs.msg import PoseStamped
except ImportError:  # pragma: no cover
    PoseStamped = None

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=1)

COLS = ["wall_us", "mono_us", "elapsed_s",
        "lp_x", "lp_y", "lp_z", "lp_vx", "lp_vy", "lp_vz",
        "lp_heading", "lp_valid", "lp_ts_us", "lp_rx_us",
        "st_arming", "st_nav", "st_intent", "st_failsafe",
        "es_ev_pos", "es_ev_hgt", "es_ev_yaw", "es_ev_vel",
        "es_in_air", "es_rej_hp", "es_rej_vp", "es_ts_us",
        "ld_landed", "ld_freefall", "ld_moving",
        "ts_rtt", "ts_residual",
        "vio_ready", "vio_fault", "vio_disarm", "vio_reason",
        "vio_rx_us", "vio_raw"]

SLAM_COLS = ["recv_wall_us", "recv_mono_us", "stamp_sim_us", "x", "y", "z",
             "qx", "qy", "qz", "qw"]


def topic_of(cls, name):
    version = getattr(cls, "MESSAGE_VERSION", 0)
    return "/fmu/out/" + name + (f"_v{version}" if version else "")


def num(v, default=""):
    try:
        return round(float(v), 6)
    except (TypeError, ValueError):
        return default


def stamp_us(msg):
    s = msg.header.stamp
    return int(s.sec) * 1000000 + int(s.nanosec) // 1000


class Sampler(Node):
    def __init__(self, path, slam_path, period):
        super().__init__("vio_telemetry_sampler")
        self.lp = self.st = self.es = self.ts = self.vio = self.ld = None
        self.lp_rx = self.es_rx = self.vio_rx = 0
        self.t0_mono = time.monotonic()
        self.create_subscription(VehicleLocalPosition,
                                 topic_of(VehicleLocalPosition, "vehicle_local_position"),
                                 self.on_lp, QOS)
        self.create_subscription(VehicleStatus,
                                 topic_of(VehicleStatus, "vehicle_status"),
                                 self.on_st, QOS)
        self.create_subscription(EstimatorStatusFlags,
                                 topic_of(EstimatorStatusFlags, "estimator_status_flags"),
                                 self.on_es, QOS)
        self.create_subscription(TimesyncStatus,
                                 topic_of(TimesyncStatus, "timesync_status"),
                                 self.on_ts, QOS)
        self.create_subscription(VehicleLandDetected,
                                 topic_of(VehicleLandDetected, "vehicle_land_detected"),
                                 self.on_ld, QOS)
        self.create_subscription(String, "/vio/navigation_health", self.on_vio, 1)
        self.csv = open(path, "w", newline="")
        self.writer = csv.writer(self.csv)
        self.writer.writerow(COLS)
        self.slam = None
        if slam_path and PoseStamped is not None:
            self.slam = open(slam_path, "w", newline="")
            self.slam_writer = csv.writer(self.slam)
            self.slam_writer.writerow(SLAM_COLS)
            self.create_subscription(PoseStamped, "/slam/pose", self.on_pose, 10)
        self.create_timer(period, self.sample)
        self.rows = 0

    def on_lp(self, m):
        self.lp = m
        self.lp_rx = time.time_ns() // 1000

    def on_st(self, m):
        self.st = m

    def on_es(self, m):
        self.es = m
        self.es_rx = time.time_ns() // 1000

    def on_ts(self, m):
        self.ts = m

    def on_ld(self, m):
        self.ld = m

    def on_vio(self, m):
        self.vio = m.data
        self.vio_rx = time.time_ns() // 1000

    def on_pose(self, m):
        p = m.pose
        self.slam_writer.writerow([
            time.time_ns() // 1000, time.monotonic_ns() // 1000, stamp_us(m),
            "%.9f" % p.position.x, "%.9f" % p.position.y, "%.9f" % p.position.z,
            "%.9f" % p.orientation.x, "%.9f" % p.orientation.y,
            "%.9f" % p.orientation.z, "%.9f" % p.orientation.w])
        self.slam.flush()

    def sample(self):
        lp, st, es, ts, ld = self.lp, self.st, self.es, self.ts, self.ld
        vio = {}
        raw = ""
        if self.vio:
            raw = self.vio
            try:
                vio = json.loads(raw)
            except ValueError:
                vio = {}
        row = [time.time_ns() // 1000, time.monotonic_ns() // 1000,
               round(time.monotonic() - self.t0_mono, 3)]
        if lp:
            row += [num(lp.x), num(lp.y), num(lp.z),
                    num(lp.vx), num(lp.vy), num(lp.vz),
                    num(lp.heading),
                    int(bool(lp.xy_valid and lp.z_valid and lp.v_xy_valid
                            and lp.v_z_valid
                            and getattr(lp, "heading_good_for_control", False))),
                    int(lp.timestamp), self.lp_rx]
        else:
            row += [""] * 10
        if st:
            row += [int(st.arming_state), int(st.nav_state),
                    int(getattr(st, "nav_state_user_intention", -1)),
                    int(st.failsafe)]
        else:
            row += [""] * 4
        if es:
            row += [int(es.cs_ev_pos), int(es.cs_ev_hgt), int(es.cs_ev_yaw),
                    int(getattr(es, "cs_ev_vel", 0)), int(es.cs_in_air),
                    int(getattr(es, "reject_hor_pos", 0)),
                    int(getattr(es, "reject_ver_pos", 0)), self.es_rx]
        else:
            row += [""] * 8
        if ld:
            row += [int(ld.landed), int(ld.freefall),
                    int(ld.vertical_movement or ld.horizontal_movement
                        or ld.rotational_movement)]
        else:
            row += [""] * 3
        if ts:
            row += [int(ts.round_trip_time),
                    int(ts.observed_offset) - int(ts.estimated_offset)]
        else:
            row += ["", ""]
        row += [vio.get("navigation_ready", ""), vio.get("airborne_fault", ""),
                vio.get("requires_disarm", ""), vio.get("reason", ""),
                self.vio_rx, raw]
        self.writer.writerow(row)
        self.csv.flush()
        self.rows += 1

    def close(self):
        self.csv.close()
        if self.slam:
            self.slam.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--slam-csv", default="")
    ap.add_argument("--period", type=float, default=0.1)
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = 不限时")
    args = ap.parse_args()
    rclpy.init()
    node = Sampler(args.out, args.slam_csv, args.period)
    start = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if args.seconds and time.monotonic() - start > args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
