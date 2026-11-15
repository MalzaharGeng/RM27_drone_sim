#!/usr/bin/env python3
"""Gazebo Pose_V -> ROS PoseStamped 适配器（只读）。

为什么要单独一层：
  ros_gz_bridge 把 gz.msgs.Pose_V 转成 tf2_msgs/TFMessage 时丢掉了实体名和
  源时间戳（实测 header.stamp=0、child_frame_id=''），所以旧 truth_odom_node
  只能取 transforms[0]，既无法按名选机，也无法用源时间求速度。

本节点直接读 gz transport 文本流，按实体名筛选，把「源 sim 时间戳」原样放进
PoseStamped.header.stamp，位置是世界系 ENU。速度不在这一层算，交给下游按源
时间差分，避免用接收时刻冒充采样时刻。

输出：/truth/pose (geometry_msgs/PoseStamped)，frame_id=truth_world_enu
"""
import argparse
import math
import os
import re
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

NUM = re.compile(r"^\s*([A-Za-z_]+)\s*:\s*(.*?)\s*$")


def _f(v, default=0.0):
    try:
        x = float(v)
        return x if math.isfinite(x) else default
    except (TypeError, ValueError):
        return default


def parse_pose_block(lines):
    name = ""
    p = {"x": 0.0, "y": 0.0, "z": 0.0}
    q = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    cur = None
    for line in lines:
        s = line.strip()
        if s.startswith("name:"):
            name = s.split(":", 1)[1].strip().strip('"')
        elif s == "position {":
            cur = p
        elif s == "orientation {":
            cur = q
        elif s == "}":
            cur = None
        elif cur is not None:
            m = NUM.match(s)
            if m and m.group(1) in cur:
                cur[m.group(1)] = _f(m.group(2))
    return name, p, q


class TruthGzNode(Node):
    def __init__(self, entity, topic):
        super().__init__("truth_gz_node")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub = self.create_publisher(PoseStamped, "/truth/pose", qos)
        self.entity = entity
        self.topic = topic
        self.published = 0
        self.stop = False
        self.create_timer(1.0, self.report)

    def report(self):
        self.get_logger().info("truth_gz published=%d entity=%s" %
                               (self.published, self.entity))

    def pump(self):
        proc = subprocess.Popen(["gz", "topic", "-e", "-t", self.topic],
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                text=True, bufsize=1)
        header_sec = None
        header_nsec = None
        in_header = False
        block = None
        try:
            for line in proc.stdout:
                if self.stop:
                    break
                s = line.rstrip("\n")
                st = s.strip()
                if s == "header {":
                    in_header = True
                    header_sec = header_nsec = None
                    continue
                if in_header:
                    if st == "}":
                        in_header = False
                        continue
                    m = NUM.match(st)
                    if m:
                        if m.group(1) == "sec":
                            header_sec = _f(m.group(2))
                        elif m.group(1) == "nsec":
                            header_nsec = _f(m.group(2))
                    continue
                if s == "pose {":
                    block = []
                    continue
                if block is not None:
                    if s == "}":
                        name, p, q = parse_pose_block(block)
                        block = None
                        if name != self.entity or header_sec is None:
                            continue
                        msg = PoseStamped()
                        msg.header.stamp.sec = int(header_sec)
                        msg.header.stamp.nanosec = int(header_nsec or 0.0)
                        msg.header.frame_id = "truth_world_enu"
                        msg.pose.position.x = p["x"]
                        msg.pose.position.y = p["y"]
                        msg.pose.position.z = p["z"]
                        msg.pose.orientation.x = q["x"]
                        msg.pose.orientation.y = q["y"]
                        msg.pose.orientation.z = q["z"]
                        msg.pose.orientation.w = q["w"]
                        self.pub.publish(msg)
                        self.published += 1
                    else:
                        block.append(s)
                rclpy.spin_once(self, timeout_sec=0.0)
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except Exception:
                proc.kill()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--entity", default=os.environ.get("TRUTH_ENTITY", "x500_mono_cam_0"))
    ap.add_argument("--topic", default=os.environ.get(
        "TRUTH_GZ_TOPIC", "/world/rm27_world/dynamic_pose/info"))
    args = ap.parse_args()
    rclpy.init()
    node = TruthGzNode(args.entity, args.topic)

    def on_term(_s, _f):
        node.stop = True
    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGINT, on_term)
    try:
        node.pump()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
