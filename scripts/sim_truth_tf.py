#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真真值适配器：把 ros_gz_bridge 的 dynamic_pose/info 转成可用的 TF。

ros_gz_bridge 的 gz.msgs.Pose_V -> tf2_msgs/TFMessage 映射会丢掉 name 字段，
导致 child_frame_id 全为空，slam_pose_to_px4.py --compare 匹配不到模型。
这个节点只保留 Pose_V 的第一项（gz 里就是模型自身的世界位姿），补上
child_frame_id 后发到 /truth/model_pose。

只用于仿真对比，不参与实机链路。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class TruthAdapter(Node):
    def __init__(self, model):
        super().__init__('sim_truth_tf')
        self.model = model
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(TFMessage, '/truth/model_pose', 10)
        # 仿真侧的桥发布端是 reliable，订阅端也用 reliable，避免大消息被丢
        self.sub = self.create_subscription(
            TFMessage, '/world/rm27_world/dynamic_pose/info', self.cb, qos)
        self.get_logger().info('truth adapter: transforms[0] -> /truth/model_pose')

    def cb(self, msg):
        if not msg.transforms:
            return
        src = msg.transforms[0]
        tr = TransformStamped()
        tr.header.stamp = self.get_clock().now().to_msg()
        tr.header.frame_id = 'world'
        tr.child_frame_id = f'{self.model}/base_link'
        tr.transform = src.transform
        out = TFMessage()
        out.transforms = [tr]
        self.pub.publish(out)


def main():
    rclpy.init()
    n = TruthAdapter('x500_mono_cam_0')
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
