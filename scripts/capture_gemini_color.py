#!/usr/bin/env python3
"""Gemini 335L 컬러 프레임 한 장을 PNG로 저장한다. 보정은 하지 않는다."""

import argparse
import struct
import time
import zlib

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def qos(reliable):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class Grab(Node):
    def __init__(self, topic):
        super().__init__("gemini_color_grab")
        self.message = None
        self.create_subscription(Image, topic, self.on_image, qos(True))
        self.create_subscription(Image, topic, self.on_image, qos(False))

    def on_image(self, message):
        if self.message is None and message.width > 0 and message.height > 0:
            self.message = message


def write_png(path, image):
    height, width = image.shape[:2]
    if image.ndim == 2:
        color_type = 0
        rows = image
    else:
        color_type = 2
        rows = image
    raw = b"".join(b"\x00" + np.ascontiguousarray(rows[row]).tobytes() for row in range(height))

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    header = struct.pack(">IIBBBBB", width, height, 8, color_type, 0, 0, 0)
    payload = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b"")
    with open(path, "wb") as handle:
        handle.write(payload)


def save_png(message, path):
    array = np.frombuffer(message.data, dtype=np.uint8)
    height = int(message.height)
    width = int(message.width)
    encoding = message.encoding
    if encoding == "rgb8":
        image = array.reshape(height, width, 3)
    elif encoding == "bgr8":
        image = array.reshape(height, width, 3)[:, :, ::-1]
    elif encoding == "rgba8":
        image = array.reshape(height, width, 4)[:, :, :3]
    elif encoding == "bgra8":
        image = array.reshape(height, width, 4)[:, :, 2::-1]
    elif encoding == "mono8":
        image = array.reshape(height, width)
    else:
        raise SystemExit(f"지원하지 않는 encoding: {encoding}")
    write_png(path, np.ascontiguousarray(image))
    return encoding


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/color/image_raw")
    parser.add_argument("--png", required=True)
    parser.add_argument("--meta", required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    rclpy.init()
    node = Grab(args.topic)
    started = time.monotonic()
    while rclpy.ok() and node.message is None and time.monotonic() - started < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.message is None:
        raise SystemExit("컬러 프레임을 받지 못했습니다.")
    message = node.message
    encoding = save_png(message, args.png)
    with open(args.meta, "w", encoding="utf-8") as handle:
        handle.write(f"topic {args.topic}\n")
        handle.write(f"frame_id {message.header.frame_id}\n")
        handle.write(f"stamp_sec {message.header.stamp.sec}\n")
        handle.write(f"stamp_nanosec {message.header.stamp.nanosec}\n")
        handle.write(f"width {message.width}\n")
        handle.write(f"height {message.height}\n")
        handle.write(f"encoding {encoding}\n")
        handle.write("calibration_solve no\n")
    print(f"saved {args.png} {message.width}x{message.height} {encoding} frame {message.header.frame_id}")


if __name__ == "__main__":
    main()
