#!/usr/bin/env python3
"""실제 PointCloud2 토픽을 정해진 시간 동안 측정한다."""

import argparse
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def field_dtype(field):
    kind = {
        1: np.int8,
        2: np.uint8,
        3: np.int16,
        4: np.uint16,
        5: np.int32,
        6: np.uint32,
        7: np.float32,
        8: np.float64,
    }.get(field.datatype)
    if kind is None or field.count != 1:
        return None
    return np.dtype(kind)


class Probe(Node):
    def __init__(self, topic, reliability, duration):
        super().__init__("gemini_pointcloud_probe")
        self.topic = topic
        self.duration = duration
        self.started = time.monotonic()
        self.arrivals = []
        self.frame_ids = []
        self.stamps = []
        self.zero_size = 0
        self.finite_points = 0
        self.total_points = 0
        self.schema = None
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, topic, self.on_cloud, qos)

    def on_cloud(self, msg):
        now = time.monotonic()
        if now - self.started > self.duration:
            return
        self.arrivals.append(now)
        self.frame_ids.append(msg.header.frame_id)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.stamps.append(stamp)
        count = msg.width * msg.height
        if count == 0 or not msg.data:
            self.zero_size += 1
            return
        if self.schema is None:
            self.schema = {
                "width": msg.width,
                "height": msg.height,
                "is_bigendian": bool(msg.is_bigendian),
                "point_step": msg.point_step,
                "row_step": msg.row_step,
                "is_dense": bool(msg.is_dense),
                "fields": [
                    {"name": field.name, "offset": field.offset, "datatype": field.datatype, "count": field.count}
                    for field in msg.fields
                ],
            }
        names = {field.name: field for field in msg.fields}
        if not all(name in names and field_dtype(names[name]) is not None for name in ("x", "y", "z")):
            return
        endian = ">" if msg.is_bigendian else "<"
        dtype = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": [field_dtype(names[name]).str.replace("<", endian) for name in ("x", "y", "z")],
                "offsets": [names[name].offset for name in ("x", "y", "z")],
                "itemsize": msg.point_step,
            }
        )
        points = np.frombuffer(msg.data, dtype=dtype, count=count)
        finite = np.isfinite(points["x"]) & np.isfinite(points["y"]) & np.isfinite(points["z"])
        self.total_points += int(count)
        self.finite_points += int(np.count_nonzero(finite))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--duration", type=float, default=35.0)
    parser.add_argument("--reliability", choices=("reliable", "best_effort"), default="reliable")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    reliability = ReliabilityPolicy.RELIABLE if args.reliability == "reliable" else ReliabilityPolicy.BEST_EFFORT
    rclpy.init()
    node = Probe(args.topic, reliability, args.duration)
    while rclpy.ok() and time.monotonic() - node.started < args.duration:
        rclpy.spin_once(node, timeout_sec=0.2)
    arrivals = node.arrivals
    intervals = [arrivals[i] - arrivals[i - 1] for i in range(1, len(arrivals))]
    frame_set = sorted(set(node.frame_ids))
    stamp_backwards = sum(1 for i in range(1, len(node.stamps)) if node.stamps[i] + 1e-6 < node.stamps[i - 1])
    duration = arrivals[-1] - arrivals[0] if len(arrivals) >= 2 else 0.0
    result = {
        "device": "Gemini 335L",
        "topic": args.topic,
        "frame_id": frame_set[0] if len(frame_set) == 1 else None,
        "frame_ids": frame_set,
        "frame_id_changes": max(0, len(frame_set) - 1),
        "width": None if node.schema is None else node.schema["width"],
        "height": None if node.schema is None else node.schema["height"],
        "is_bigendian": None if node.schema is None else node.schema["is_bigendian"],
        "point_step": None if node.schema is None else node.schema["point_step"],
        "row_step": None if node.schema is None else node.schema["row_step"],
        "is_dense": None if node.schema is None else node.schema["is_dense"],
        "fields": [] if node.schema is None else node.schema["fields"],
        "finite_points": node.finite_points,
        "total_points": node.total_points,
        "finite_ratio": None if node.total_points == 0 else node.finite_points / node.total_points,
        "zero_size_messages": node.zero_size,
        "messages": len(arrivals),
        "duration_sec": duration,
        "mean_hz": None if duration <= 0 else (len(arrivals) - 1) / duration,
        "min_interval_sec": None if not intervals else min(intervals),
        "max_interval_sec": None if not intervals else max(intervals),
        "stddev_interval_sec": None
        if len(intervals) < 2
        else math.sqrt(sum((item - (sum(intervals) / len(intervals))) ** 2 for item in intervals) / (len(intervals) - 1)),
        "stamp_backwards": stamp_backwards,
        "disconnects": sum(1 for item in intervals if item > 2.0),
        "disconnect_gap_sec": 2.0,
        "qos_reliability": args.reliability,
    }
    with open(args.output, "w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2)
        handle.write("\n")
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    if result["messages"] < 2 or result["duration_sec"] < 30.0:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
