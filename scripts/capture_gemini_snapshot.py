#!/usr/bin/env python3
"""Gemini 335L PointCloud2 한 프레임을 binary PCD로 저장한다."""

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


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


class Capture(Node):
    def __init__(self, topic):
        super().__init__("gemini_snapshot_capture")
        self.message = None
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, topic, self.on_cloud, qos)

    def on_cloud(self, msg):
        if self.message is None and msg.width * msg.height > 0:
            self.message = msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/depth/points")
    parser.add_argument("--pcd", required=True)
    parser.add_argument("--meta", required=True)
    parser.add_argument("--timeout", type=float, default=40.0)
    args = parser.parse_args()
    rclpy.init()
    node = Capture(args.topic)
    started = time.monotonic()
    while rclpy.ok() and node.message is None and time.monotonic() - started < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.message is None:
        raise SystemExit("PointCloud2 스냅샷을 받지 못했습니다.")
    msg = node.message
    names = {field.name: field for field in msg.fields}
    if not all(name in names for name in ("x", "y", "z")):
        raise SystemExit("스냅샷에 xyz 필드가 없습니다.")
    endian = ">" if msg.is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [endian + "f4", endian + "f4", endian + "f4"],
            "offsets": [names[name].offset for name in ("x", "y", "z")],
            "itemsize": msg.point_step,
        }
    )
    count = int(msg.width * msg.height)
    points = np.frombuffer(msg.data, dtype=dtype, count=count)
    xyz = np.stack((points["x"], points["y"], points["z"]), axis=1).astype("<f4", copy=False)
    finite = np.isfinite(xyz).all(axis=1)
    write_pcd(args.pcd, xyz)
    fields = ",".join(f"{field.name}:{field.datatype}:{field.count}" for field in msg.fields)
    meta = "\n".join(
        [
            f"topic={args.topic}",
            f"frame_id={msg.header.frame_id}",
            f"stamp_sec={msg.header.stamp.sec}",
            f"stamp_nanosec={msg.header.stamp.nanosec}",
            f"width={msg.width}",
            f"height={msg.height}",
            f"point_step={msg.point_step}",
            f"is_dense={int(msg.is_dense)}",
            f"is_bigendian={int(msg.is_bigendian)}",
            f"fields={fields}",
            f"point_count={count}",
            f"finite_count={int(np.count_nonzero(finite))}",
            "",
        ]
    )
    with open(args.meta, "w", encoding="utf-8") as handle:
        handle.write(meta)
    print(meta, end="")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
