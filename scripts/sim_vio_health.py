#!/usr/bin/env python3
"""SITL evaluation bridge for /vio/navigation_health.

This is NOT the flight ORB_SLAM3 Px4Bridge health publisher. It is a
simulation-side adapter that publishes the same schema only while all of the
following are actually true:

  * /slam/pose is fresh and finite (the stereo ORB_SLAM3 node is tracking)
  * PX4 VehicleLocalPosition is fresh and valid
  * PX4 EstimatorStatusFlags reports EV position + EV height fusion active
    (EKF2_EV_CTRL position/height bits) and no rejected EV innovations

If the SLAM/injection side disappears, this node either reports
navigation_ready=false (if it is still alive) or stops publishing entirely
(if the whole VIO stack is stopped), which the authority gate treats as a
loss of VIO health.
"""
import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (VehicleStatus, VehicleLocalPosition,
                          EstimatorStatusFlags)
from std_msgs.msg import String

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)


def fresh(now, stamp, limit):
    return stamp is not None and now - stamp <= limit


class Health(Node):
    def __init__(self, prefix, instance_id):
        super().__init__("sim_vio_health")
        self.prefix = prefix
        self.instance_id = instance_id
        self.slam = None
        self.slam_rx = None
        self.local = None
        self.local_rx = None
        self.status = None
        self.status_rx = None
        self.flags = None
        self.flags_rx = None
        self.count = 0
        self.last_reason = None
        self.create_subscription(PoseStamped, "/slam/pose", self.on_slam, 10)
        for cls, name, fn in (
                (VehicleLocalPosition, "vehicle_local_position", self.on_local),
                (VehicleStatus, "vehicle_status", self.on_status),
                (EstimatorStatusFlags, "estimator_status_flags", self.on_flags)):
            version = getattr(cls, "MESSAGE_VERSION", 0)
            topic = prefix + "/fmu/out/" + name + (f"_v{version}" if version else "")
            self.create_subscription(cls, topic, fn, PX4_QOS)
        self.pub = self.create_publisher(String, prefix + "/vio/navigation_health", 1)
        self.create_timer(0.05, self.tick)
        self.get_logger().info("sim health: gated on /slam/pose + PX4 EV fusion flags")

    def on_slam(self, m):
        self.slam = m
        self.slam_rx = time.monotonic()

    def on_local(self, m):
        self.local = m
        self.local_rx = time.monotonic()

    def on_status(self, m):
        self.status = m
        self.status_rx = time.monotonic()

    def on_flags(self, m):
        self.flags = m
        self.flags_rx = time.monotonic()

    def tick(self):
        now = time.monotonic()
        reason = ""
        if not fresh(now, self.slam_rx, 0.6):
            reason = "slam_pose_stale"
        elif self.slam is None or not all(math.isfinite(x) for x in
                                         (self.slam.pose.position.x, self.slam.pose.position.y,
                                          self.slam.pose.position.z)):
            reason = "slam_pose_invalid"
        elif not fresh(now, self.local_rx, 0.5):
            reason = "px4_local_stale"
        elif not fresh(now, self.flags_rx, 2.0):
            reason = "px4_estimator_flags_stale"
        elif not fresh(now, self.status_rx, 2.0):
            reason = "px4_status_stale"
        elif self.local is None or not (self.local.xy_valid and self.local.z_valid
                                        and self.local.v_xy_valid and self.local.v_z_valid):
            reason = "px4_local_invalid"
        elif self.flags is None:
            reason = "px4_estimator_flags_missing"
        else:
            f = self.flags
            if not f.cs_ev_pos:
                reason = "ev_pos_not_fused"
            elif not f.cs_ev_hgt:
                reason = "ev_hgt_not_fused"
            elif not f.cs_tilt_align:
                reason = "tilt_not_aligned"
            elif not f.cs_yaw_align:
                reason = "yaw_not_aligned"
            elif f.reject_hor_pos:
                reason = "ev_hor_pos_rejected"
            elif f.reject_ver_pos:
                reason = "ev_ver_pos_rejected"

        local = self.local
        status = self.status
        ready = not reason
        if self.count % 20 == 0 or reason != self.last_reason:
            self.get_logger().info(
                "health ready=%s reason=%s ev_pos=%s ev_hgt=%s tilt=%s yaw=%s"
                % (ready, reason or "ok",
                   getattr(self.flags, "cs_ev_pos", None) if self.flags else None,
                   getattr(self.flags, "cs_ev_hgt", None) if self.flags else None,
                   getattr(self.flags, "cs_tilt_align", None) if self.flags else None,
                   getattr(self.flags, "cs_yaw_align", None) if self.flags else None))
        self.last_reason = reason
        self.count += 1

        payload = dict(
            schema_version=1,
            timestamp_us=time.time_ns() // 1000,
            instance_id=self.instance_id,
            reset_counter=1,
            reference_epoch=1,
            clock_epoch=1,
            px4_xy_reset_counter=int(local.xy_reset_counter) if local else 0,
            px4_z_reset_counter=int(local.z_reset_counter) if local else 0,
            px4_heading_reset_counter=int(local.heading_reset_counter) if local else 0,
            navigation_ready=ready,
            airborne_fault=False,
            requires_disarm=False,
            armed=bool(status and status.arming_state == 2),
            reason=reason or "ok",
        )
        self.pub.publish(String(data=json.dumps(payload)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic-prefix", default="")
    ap.add_argument("--instance-id", default="sitl-eval-20261111")
    args = ap.parse_args()
    rclpy.init()
    n = Health(args.topic_prefix, args.instance_id)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
