#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[NP-01 FIX18] 消息通路计数（只作证据，不参与任何控制）。

为什么需要这个进程
------------------
FIX17 的中断回归只能用「日志文件最后写入时间」说明桥停了。日志 mtime 只能
证明文件被写过或没被写过，不能证明进程死了，也不能证明消息停了。本轮指令
要求把三类证据分开：

  * 进程存在/退出：PID + 身份 + 退出状态（由 np01_lifecycle.py 记录）
  * 消息通路：接收计数 + 最近接收时间（本进程记录）
  * 文件记录：写入与 flush（各节点自己的 CSV）

本进程只订阅，不发布、不写 pilot.csv、不碰 Gate/LIMITS/评估。订阅的是两个
真实通路：
  /vio/navigation_health              std_msgs/String，5 Hz，Gate::Health() 输出
  /fmu/in/vehicle_visual_odometry     px4_msgs/VehicleOdometry，每次发布决策一条

注入后若本表的 health/odometry 计数停止增长、最近接收时间不再刷新，才说明
「没有新观测发布」；与节点 PID 消失、退出状态一起构成完整证据链。
"""
import argparse
import csv
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from std_msgs.msg import String

try:
    from px4_msgs.msg import VehicleOdometry
except Exception as exc:  # pragma: no cover - 环境缺 px4_msgs 时降级
    VehicleOdometry = None
    _ODO_IMPORT_ERROR = repr(exc)
else:
    _ODO_IMPORT_ERROR = None

HEALTH_TOPIC = "/vio/navigation_health"


def odo_topic():
    v = getattr(VehicleOdometry, "MESSAGE_VERSION", 0) if VehicleOdometry else 0
    return "/fmu/in/vehicle_visual_odometry" + (("_v%d" % v) if v else "")


class Counter(Node):
    def __init__(self, out_path, period):
        super().__init__("np01_vio_obs_counter")
        self.t0 = time.monotonic()
        self.wall0 = time.time()
        self.h_count = 0
        self.h_last_mono = ""
        self.h_last_wall_us = ""
        self.h_text = ""
        self.o_count = 0
        self.o_last_mono = ""
        self.o_last_wall_us = ""
        self.o_last_stamp_us = ""
        self.odo_enabled = False

        # 发布端 /vio/navigation_health 是 reliable、depth 1；reliable 订阅兼容。
        qos_h = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE,
                           history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(String, HEALTH_TOPIC, self.on_health, qos_h)

        # PX4 发布端是 best_effort（SensorDataQoS）。订阅端必须同为 best_effort。
        if VehicleOdometry is not None:
            qos_o = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE,
                               history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(VehicleOdometry, odo_topic(), self.on_odo, qos_o)
            self.odo_enabled = True

        self._csv = open(out_path, "w", newline="", encoding="utf-8")
        self._wr = csv.writer(self._csv)
        self._wr.writerow([
            "t_mono_s", "t_wall_us", "health_count", "health_last_rx_mono_s",
            "health_last_rx_wall_us", "health_text",
            "odo_count", "odo_last_rx_mono_s", "odo_last_rx_wall_us",
            "odo_last_stamp_us", "odo_topic", "odo_enabled"])
        self._csv.flush()
        self.create_timer(period, self.emit)

        self.get_logger().info(
            "[NP01_FIX18] counter up: health=%s odo=%s (enabled=%s%s)"
            % (HEALTH_TOPIC, odo_topic(), self.odo_enabled,
               "" if self.odo_enabled else " import_error=%s" % _ODO_IMPORT_ERROR))

    def _mono(self):
        return time.monotonic() - self.t0

    def on_health(self, msg):
        self.h_count += 1
        self.h_last_mono = "%.6f" % self._mono()
        self.h_last_wall_us = str(int(time.time() * 1e6))
        self.h_text = (msg.data or "")[:120].replace("\n", " ")

    def on_odo(self, msg):
        self.o_count += 1
        self.o_last_mono = "%.6f" % self._mono()
        self.o_last_wall_us = str(int(time.time() * 1e6))
        try:
            self.o_last_stamp_us = str(int(msg.timestamp))
        except Exception:
            self.o_last_stamp_us = ""

    def emit(self):
        self._wr.writerow([
            "%.3f" % self._mono(), int(time.time() * 1e6),
            self.h_count, self.h_last_mono, self.h_last_wall_us, self.h_text,
            self.o_count, self.o_last_mono, self.o_last_wall_us,
            self.o_last_stamp_us, odo_topic(), int(self.odo_enabled)])
        self._csv.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--period", type=float, default=0.5)
    a = ap.parse_args()

    rclpy.init()
    node = Counter(a.out, a.period)

    def _stop(_sig, _frm):
        raise KeyboardInterrupt

    for s in (signal.SIGTERM, signal.SIGINT):
        try:
            signal.signal(s, _stop)
        except ValueError:
            pass

    rc = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rc = 0
    finally:
        try:
            node.emit()
        except Exception:
            pass
        try:
            node._csv.close()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
