#!/usr/bin/env python3
"""打印 EKF2 融合/拒绝标志（仿真诊断用）。

用来回答「为什么 cs_ev_pos 一直是 0」：是 EV 融合没使能、还是被 innovation
门限拒掉、还是高度参考源没切到 EV。
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import EstimatorStatusFlags

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=5)

BITS = [
    "cs_tilt_align", "cs_yaw_align", "cs_gnss_pos", "cs_opt_flow",
    "cs_baro_hgt", "cs_gps_hgt", "cs_ev_pos", "cs_ev_yaw", "cs_ev_hgt",
    "cs_ev_vel", "cs_in_air", "cs_inertial_dead_reckoning",
    "cs_fake_pos", "cs_fake_hgt", "cs_constant_pos", "cs_valid_fake_pos",
    "cs_gnss_vel", "cs_mag_3d", "cs_mag_hdg",
    "reject_hor_vel", "reject_ver_vel", "reject_hor_pos", "reject_ver_pos",
    "reject_yaw",
]


class Flags(Node):
    def __init__(self, every):
        super().__init__("ekf_flags_probe")
        self.every = every
        self.n = 0
        topic = "/fmu/out/estimator_status_flags"
        v = getattr(EstimatorStatusFlags, "MESSAGE_VERSION", 0)
        if v:
            topic += "_v%d" % v
        self.create_subscription(EstimatorStatusFlags, topic, self.cb, QOS)
        self.get_logger().info("listening " + topic)

    def cb(self, m):
        self.n += 1
        if self.n % self.every:
            return
        on = [b for b in BITS if getattr(m, b, False)]
        self.get_logger().info(
            "#%d on=[%s]" % (self.n, ",".join(on) if on else "-"))


def main():
    every = int(sys.argv[1]) if len(sys.argv) > 1 else 50
    rclpy.init()
    node = Flags(every)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
