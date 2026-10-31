#!/usr/bin/env python3
"""W1 witness용 정지 촬영. 로봇 명령은 없다. Dataset v2와 hand-eye 샘플이 아니다."""

import json
import struct
import time
import zlib
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def qos(reliable):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def write_png(path, image):
    height, width = image.shape[:2]
    if image.ndim == 2 and image.dtype == np.uint16:
        color_type = 0
        bit_depth = 16
        rows = image.astype(">u2")
    elif image.ndim == 2:
        color_type = 0
        bit_depth = 8
        rows = image
    else:
        color_type = 2
        bit_depth = 8
        rows = image
    raw = b"".join(b"\x00" + np.ascontiguousarray(rows[row]).tobytes() for row in range(height))

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    header = struct.pack(">IIBBBBB", width, height, bit_depth, color_type, 0, 0, 0)
    payload = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b"")
    path.write_bytes(payload)


def image_array(message):
    if message.encoding == "rgb8":
        return np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width, 3).copy()
    if message.encoding == "bgr8":
        array = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width, 3)
        return np.ascontiguousarray(array[:, :, ::-1])
    if message.encoding == "16UC1":
        return np.frombuffer(message.data, dtype=np.uint16).reshape(message.height, message.width).copy()
    raise RuntimeError(f"지원하지 않는 encoding: {message.encoding}")


def info_dict(message):
    return {
        "frame_id": message.header.frame_id,
        "width": int(message.width),
        "height": int(message.height),
        "distortion_model": message.distortion_model,
        "d": [float(value) for value in message.d],
        "k": [float(value) for value in message.k],
        "r": [float(value) for value in message.r],
        "p": [float(value) for value in message.p],
    }


class Grab(Node):
    def __init__(self):
        super().__init__("p008r5_w1_static_grab")
        self.color = None
        self.depth = None
        self.color_info = None
        self.depth_info = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        for reliable in (True, False):
            self.create_subscription(Image, "/camera/color/image_raw", self.on_color, qos(reliable))
            self.create_subscription(Image, "/camera/depth/image_raw", self.on_depth, qos(reliable))
            self.create_subscription(CameraInfo, "/camera/color/camera_info", self.on_color_info, qos(reliable))
            self.create_subscription(CameraInfo, "/camera/depth/camera_info", self.on_depth_info, qos(reliable))

    def on_color(self, message):
        if message.width > 0 and message.height > 0:
            self.color = message

    def on_depth(self, message):
        if message.width > 0 and message.height > 0:
            self.depth = message

    def on_color_info(self, message):
        self.color_info = message

    def on_depth_info(self, message):
        self.depth_info = message


def main():
    out = Path("evidence/P-008/R5-HAND-EYE-MOTION/W1/before")
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Grab()
    deadline = time.monotonic() + 40.0
    while time.monotonic() < deadline and (
        node.color is None or node.depth is None or node.color_info is None or node.depth_info is None
    ):
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.color is None or node.depth is None or node.color_info is None or node.depth_info is None:
        raise SystemExit("컬러, 깊이, camera_info 중 하나가 없다")

    saved = []
    next_save = time.monotonic()
    end = time.monotonic() + 12.0
    while len(saved) < 8 and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() < next_save or node.color is None or node.depth is None:
            continue
        color_t = stamp_sec(node.color.header.stamp)
        depth_t = stamp_sec(node.depth.header.stamp)
        if abs(color_t - depth_t) > 0.05:
            continue
        index = len(saved)
        write_png(out / f"{index:02d}-color.png", image_array(node.color))
        write_png(out / f"{index:02d}-depth.png", image_array(node.depth))
        saved.append(
            {
                "index": index,
                "color_stamp": color_t,
                "depth_stamp": depth_t,
                "stamp_delta_sec": abs(color_t - depth_t),
                "color_frame_id": node.color.header.frame_id,
                "depth_frame_id": node.depth.header.frame_id,
                "color_encoding": node.color.encoding,
                "depth_encoding": node.depth.encoding,
            }
        )
        next_save = time.monotonic() + 0.4
    (out / "color_camera_info.json").write_text(json.dumps(info_dict(node.color_info), indent=2) + "\n")
    (out / "depth_camera_info.json").write_text(json.dumps(info_dict(node.depth_info), indent=2) + "\n")
    (out / "timestamps.json").write_text(
        json.dumps(
            {
                "purpose": "MOTION_WITNESS_STATIC_BEFORE_WRITE",
                "dataset_v2": "UNCHANGED",
                "hand_eye_sample_count": 0,
                "frames": saved,
            },
            indent=2,
        )
        + "\n"
    )
    node.destroy_node()
    rclpy.shutdown()
    if len(saved) < 5:
        raise SystemExit(f"동기 프레임이 5장 미만이다: {len(saved)}")


if __name__ == "__main__":
    main()
