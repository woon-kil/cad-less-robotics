#!/usr/bin/env python3
"""Gemini 335L 컬러·깊이·camera_info·내부 TF를 읽기만 한다."""

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


def qos(reliable, transient=False):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE,
    )


def stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def write_png(path, image):
    height, width = image.shape[:2]
    if image.ndim == 2:
        if image.dtype == np.uint16:
            color_type = 0
            bit_depth = 16
            rows = image.astype(">u2")
        else:
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


class Grab(Node):
    def __init__(self):
        super().__init__("p006c_stream_grab")
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


def image_array(message):
    count = int(message.height) * int(message.width)
    if message.encoding in ("rgb8", "bgr8"):
        array = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width, 3)
        if message.encoding == "bgr8":
            array = array[:, :, ::-1]
        return np.ascontiguousarray(array)
    if message.encoding == "16UC1":
        return np.frombuffer(message.data, dtype=np.uint16).reshape(message.height, message.width).copy()
    if message.encoding == "mono8":
        return np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width).copy()
    raise SystemExit(f"지원하지 않는 encoding: {message.encoding} count={count}")


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


def main():
    out = Path("evidence/P-006/camera-target/frames")
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Grab()
    deadline = time.monotonic() + 40.0
    while time.monotonic() < deadline and (node.color is None or node.depth is None or node.color_info is None or node.depth_info is None):
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.color is None or node.depth is None or node.color_info is None or node.depth_info is None:
        raise SystemExit("컬러, 깊이, camera_info 중 하나가 없다")

    saved = []
    next_save = time.monotonic()
    end = time.monotonic() + 8.0
    while len(saved) < 12 and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() < next_save or node.color is None or node.depth is None:
            continue
        color_t = stamp_sec(node.color.header.stamp)
        depth_t = stamp_sec(node.depth.header.stamp)
        delta = abs(color_t - depth_t)
        if delta > 0.05:
            continue
        try:
            transform = node.buffer.lookup_transform(
                node.depth.header.frame_id,
                node.color.header.frame_id,
                rclpy.time.Time(),
            )
        except Exception:
            continue
        index = len(saved)
        color = image_array(node.color)
        depth = image_array(node.depth)
        write_png(out / f"{index:04d}-color.png", color)
        write_png(out / f"{index:04d}-depth.png", depth)
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        record = {
            "index": index,
            "pose_label": "fixed",
            "color_stamp": color_t,
            "depth_stamp": depth_t,
            "stamp_delta_sec": delta,
            "color_frame_id": node.color.header.frame_id,
            "depth_frame_id": node.depth.header.frame_id,
            "color_encoding": node.color.encoding,
            "depth_encoding": node.depth.encoding,
            "color_info": info_dict(node.color_info),
            "depth_info": info_dict(node.depth_info),
            "T_depth_color": {
                "parent": node.depth.header.frame_id,
                "child": node.color.header.frame_id,
                "translation_m": [translation.x, translation.y, translation.z],
                "quaternion_xyzw": [rotation.x, rotation.y, rotation.z, rotation.w],
            },
        }
        (out / f"{index:04d}.json").write_text(json.dumps(record, indent=2) + "\n")
        saved.append(record)
        next_save = time.monotonic() + 0.25
    (out / "capture-summary.json").write_text(json.dumps({"frames": len(saved)}, indent=2) + "\n")
    node.destroy_node()
    rclpy.shutdown()
    if len(saved) < 10:
        raise SystemExit(f"동기 프레임이 10장 미만이다: {len(saved)}")


if __name__ == "__main__":
    main()
