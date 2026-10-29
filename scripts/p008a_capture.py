#!/usr/bin/env python3
"""P-008A — 정지 운동화의 Gemini RGB, depth, point cloud를 읽기만 한다.

기하 필터, 평면 제거, clustering, boundary는 하지 않는다.
로봇 명령은 없다. 카메라와 물체는 움직이지 않는다.

사용: python3 scripts/p008a_capture.py <output_dir> [capture_count]
"""

import json
import struct
import sys
import time
import zlib
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

COLOR_TOPIC = "/camera/color/image_raw"
DEPTH_TOPIC = "/camera/depth/image_raw"
CLOUD_TOPIC = "/camera/depth/points"
COLOR_INFO_TOPIC = "/camera/color/camera_info"
DEPTH_INFO_TOPIC = "/camera/depth/camera_info"
PAIR_MAX_SEC = 0.05


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


def write_pcd(path, xyz):
    count = int(xyz.shape[0])
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as handle:
        handle.write(header.encode("ascii"))
        handle.write(np.ascontiguousarray(xyz, dtype="<f4").tobytes())


def image_array(message):
    if message.encoding in ("rgb8", "bgr8"):
        array = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width, 3)
        if message.encoding == "bgr8":
            array = array[:, :, ::-1]
        return np.ascontiguousarray(array)
    if message.encoding == "16UC1":
        return np.frombuffer(message.data, dtype=np.uint16).reshape(message.height, message.width).copy()
    if message.encoding == "mono8":
        return np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width).copy()
    raise SystemExit(f"지원하지 않는 encoding: {message.encoding}")


def cloud_xyz(message):
    names = {field.name: field for field in message.fields}
    if not all(name in names for name in ("x", "y", "z")):
        raise SystemExit("point cloud에 xyz가 없습니다.")
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [endian + "f4", endian + "f4", endian + "f4"],
            "offsets": [names[name].offset for name in ("x", "y", "z")],
            "itemsize": message.point_step,
        }
    )
    count = int(message.width) * int(message.height)
    points = np.frombuffer(message.data, dtype=dtype, count=count)
    xyz = np.stack((points["x"], points["y"], points["z"]), axis=1).astype("<f4", copy=False)
    return np.ascontiguousarray(xyz)


def info_dict(message):
    if message is None:
        return None
    return {
        "frame_id": message.header.frame_id,
        "stamp_sec": stamp_sec(message.header.stamp),
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
        super().__init__("p008a_static_grab")
        self.color = None
        self.depth = None
        self.cloud = None
        self.color_info = None
        self.depth_info = None
        for reliable in (True, False):
            self.create_subscription(Image, COLOR_TOPIC, self.on_color, qos(reliable))
            self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, qos(reliable))
            self.create_subscription(CameraInfo, COLOR_INFO_TOPIC, self.on_color_info, qos(reliable))
            self.create_subscription(CameraInfo, DEPTH_INFO_TOPIC, self.on_depth_info, qos(reliable))
            self.create_subscription(PointCloud2, CLOUD_TOPIC, self.on_cloud, qos(reliable))

    def on_color(self, message):
        if message.width > 0 and message.height > 0:
            self.color = message

    def on_depth(self, message):
        if message.width > 0 and message.height > 0:
            self.depth = message

    def on_cloud(self, message):
        if message.width * message.height > 0:
            self.cloud = message

    def on_color_info(self, message):
        self.color_info = message

    def on_depth_info(self, message):
        self.depth_info = message


def ready(node, after):
    if node.color is None or node.depth is None or node.cloud is None:
        return False
    color_t = stamp_sec(node.color.header.stamp)
    depth_t = stamp_sec(node.depth.header.stamp)
    cloud_t = stamp_sec(node.cloud.header.stamp)
    if min(color_t, depth_t, cloud_t) <= after:
        return False
    return abs(color_t - depth_t) <= PAIR_MAX_SEC and abs(cloud_t - depth_t) <= PAIR_MAX_SEC


def main():
    if len(sys.argv) < 2:
        raise SystemExit("사용: p008a_capture.py <output_dir> [capture_count]")
    out = Path(sys.argv[1])
    count = int(sys.argv[2]) if len(sys.argv) > 2 else 5
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Grab()
    deadline = time.monotonic() + 40.0
    while time.monotonic() < deadline and not ready(node, -1.0):
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.color is None or node.depth is None or node.cloud is None:
        raise SystemExit("컬러, 깊이, point cloud 중 하나가 없습니다.")
    info_dir = out / "camera_info"
    info_dir.mkdir(parents=True, exist_ok=True)
    color_info = info_dict(node.color_info)
    depth_info = info_dict(node.depth_info)
    (info_dir / "color_camera_info.json").write_text(json.dumps(color_info, indent=2) + "\n")
    (info_dir / "depth_camera_info.json").write_text(json.dumps(depth_info, indent=2) + "\n")
    saved = []
    after = -1.0
    for index in range(1, count + 1):
        frame_deadline = time.monotonic() + 20.0
        while time.monotonic() < frame_deadline and not ready(node, after):
            rclpy.spin_once(node, timeout_sec=0.05)
        if not ready(node, after):
            raise SystemExit(f"capture_{index:02d} 에서 새 RGB/depth/cloud 쌍을 받지 못했습니다.")
        color = image_array(node.color)
        depth = image_array(node.depth)
        xyz = cloud_xyz(node.cloud)
        finite = np.isfinite(xyz).all(axis=1)
        cap = out / f"capture_{index:02d}"
        cap.mkdir(parents=True, exist_ok=True)
        write_png(cap / "color.png", color)
        write_png(cap / "depth.png", depth)
        write_pcd(cap / "cloud.pcd", xyz)
        color_t = stamp_sec(node.color.header.stamp)
        depth_t = stamp_sec(node.depth.header.stamp)
        cloud_t = stamp_sec(node.cloud.header.stamp)
        record = {
            "capture": f"capture_{index:02d}",
            "hardware_synchronized_verified": False,
            "pairing_rule_sec": PAIR_MAX_SEC,
            "host_monotonic_sec": time.monotonic(),
            "color": {
                "topic": COLOR_TOPIC,
                "stamp_sec": color_t,
                "frame_id": node.color.header.frame_id,
                "width": int(node.color.width),
                "height": int(node.color.height),
                "encoding": node.color.encoding,
                "step": int(node.color.step),
            },
            "depth": {
                "topic": DEPTH_TOPIC,
                "stamp_sec": depth_t,
                "frame_id": node.depth.header.frame_id,
                "width": int(node.depth.width),
                "height": int(node.depth.height),
                "encoding": node.depth.encoding,
                "step": int(node.depth.step),
                "scale_in_message": None,
            },
            "cloud": {
                "topic": CLOUD_TOPIC,
                "stamp_sec": cloud_t,
                "frame_id": node.cloud.header.frame_id,
                "width": int(node.cloud.width),
                "height": int(node.cloud.height),
                "point_step": int(node.cloud.point_step),
                "is_dense": bool(node.cloud.is_dense),
                "is_bigendian": bool(node.cloud.is_bigendian),
                "point_count": int(xyz.shape[0]),
                "finite_xyz_count": int(np.count_nonzero(finite)),
                "coordinate_units": "meters",
                "organized": int(node.cloud.height) > 1,
            },
            "stamp_delta_color_depth_sec": abs(color_t - depth_t),
            "stamp_delta_cloud_depth_sec": abs(cloud_t - depth_t),
        }
        (cap / "metadata.json").write_text(json.dumps(record, indent=2) + "\n")
        saved.append({
            "capture": record["capture"],
            "color_stamp_sec": color_t,
            "depth_stamp_sec": depth_t,
            "cloud_stamp_sec": cloud_t,
        })
        after = min(color_t, depth_t, cloud_t)
        time.sleep(0.6)
    session = {
        "captures": len(saved),
        "requested": count,
        "hardware_synchronized_verified": False,
        "color_camera_info_captured": color_info is not None,
        "depth_camera_info_captured": depth_info is not None,
        "saved": saved,
    }
    (out / "session.json").write_text(json.dumps(session, indent=2) + "\n")
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(session))


if __name__ == "__main__":
    main()
