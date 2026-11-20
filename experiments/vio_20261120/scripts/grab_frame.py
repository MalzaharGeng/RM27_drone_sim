#!/usr/bin/env python3
"""抓一帧仿真左目图像存盘，用于判断特征纹理是否足够。"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Grab(Node):
    def __init__(self, topic, out):
        super().__init__("grab_frame")
        self.out = out
        self.done = False
        self.sub = self.create_subscription(Image, topic, self.cb, 10)
        self.get_logger().info("waiting on " + topic)

    def cb(self, m):
        if self.done:
            return
        self.done = True
        ch = 3 if m.encoding in ("rgb8", "bgr8") else 1
        arr = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width, ch)
        if m.encoding == "bgr8":
            arr = arr[:, :, ::-1]
        try:
            from PIL import Image as PImage
            PImage.fromarray(arr[:, :, :3] if ch == 3 else arr[:, :, 0]).save(self.out)
            self.get_logger().info("saved %dx%d %s -> %s" % (m.width, m.height, m.encoding, self.out))
        except Exception as exc:  # noqa: BLE001
            np.save(self.out + ".npy", arr)
            self.get_logger().error("PIL failed (%s); saved npy instead" % exc)
        rclpy.shutdown()


def main():
    topic = sys.argv[1]
    out = sys.argv[2]
    rclpy.init()
    node = Grab(topic, out)
    try:
        rclpy.spin(node)
    except Exception:  # noqa: BLE001
        pass


if __name__ == "__main__":
    main()
