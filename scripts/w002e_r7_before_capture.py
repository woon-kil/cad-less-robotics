#!/usr/bin/env python3
"""W-002E-R7 이동 전 color 한 장. solvePnP와 마커 검출은 하지 않는다."""

import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

OUT = Path("/ws/cad-less-robotics/evidence/W-002E-R7/id2_before_move.jpg")


class OneColor(Node):
    def __init__(self):
        super().__init__("w002e_r7_before")
        self.done = False
        self.create_subscription(Image, "/camera/color/image_raw", self.on_image, 10)

    def on_image(self, message):
        if self.done:
            return
        if message.encoding != "rgb8":
            raise RuntimeError(f"unexpected encoding {message.encoding}")
        import cv2
        import numpy as np

        frame = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.width, 3
        )
        OUT.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(OUT), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)):
            raise RuntimeError("imwrite failed")
        self.done = True
        print(f"SAVED {OUT} {message.width}x{message.height} frame={message.header.frame_id}")


def main():
    rclpy.init()
    node = OneColor()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if OUT.is_file() else 1


if __name__ == "__main__":
    sys.exit(main())
