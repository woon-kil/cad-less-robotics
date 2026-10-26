#!/usr/bin/env python3
"""저장된 스냅샷을 path_extractor_node에 한 번 넣고 /vision/waypoints와 CSV를 비교한다."""

import argparse
import array
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def load_pcd(path):
    with open(path, "rb") as handle:
        raw = handle.read()
    marker = b"DATA binary\n"
    index = raw.find(marker)
    if index < 0:
        raise SystemExit("binary PCD가 아닙니다.")
    payload = raw[index + len(marker) :]
    xyz = np.frombuffer(payload, dtype="<f4")
    if xyz.size % 3 != 0:
        raise SystemExit("PCD 포인트 길이가 3의 배수가 아닙니다.")
    return xyz.reshape((-1, 3))


def load_csv(path):
    rows = []
    with open(path, encoding="utf-8") as handle:
        next(handle)
        for line in handle:
            parts = line.strip().split(",")
            if len(parts) < 8:
                continue
            rows.append([float(item) for item in parts[1:8]])
    return rows


class Check(Node):
    def __init__(self, xyz, frame_id, stamp_sec, stamp_nanosec):
        super().__init__("gemini_waypoint_topic_check")
        self.poses = None
        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(PointCloud2, "/vision/camera/points", cloud_qos)
        self.create_subscription(PoseArray, "/vision/waypoints", self.on_poses, pose_qos)
        self.message = self.make_cloud(xyz, frame_id, stamp_sec, stamp_nanosec)

    def on_poses(self, msg):
        if self.poses is None and msg.poses:
            self.poses = msg

    def make_cloud(self, xyz, frame_id, stamp_sec, stamp_nanosec):
        msg = PointCloud2()
        msg.header = Header()
        msg.header.frame_id = frame_id
        msg.header.stamp.sec = int(stamp_sec)
        msg.header.stamp.nanosec = int(stamp_nanosec)
        msg.height = 1
        msg.width = int(xyz.shape[0])
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = array.array("B", np.ascontiguousarray(xyz, dtype="<f4").tobytes())
        return msg

    def publish_until(self, timeout):
        started = time.monotonic()
        next_publish = 0.0
        self.subscription_count = 0
        while rclpy.ok() and time.monotonic() - started < timeout:
            now = time.monotonic()
            self.subscription_count = self.publisher.get_subscription_count()
            if now >= next_publish:
                self.publisher.publish(self.message)
                next_publish = now + 1.0
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.poses is not None:
                return True
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcd", required=True)
    parser.add_argument("--csv", required=True)
    parser.add_argument("--frame-id", required=True)
    parser.add_argument("--stamp-sec", type=int, required=True)
    parser.add_argument("--stamp-nanosec", type=int, required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    xyz = load_pcd(args.pcd)
    expected = load_csv(args.csv)
    rclpy.init()
    node = Check(xyz, args.frame_id, args.stamp_sec, args.stamp_nanosec)
    received = node.publish_until(120.0)
    lines = [
        "topic /vision/waypoints",
        "type geometry_msgs/msg/PoseArray",
        f"input_subscription_count {node.subscription_count}",
    ]
    status = "FAIL"
    if not received or node.poses is None:
        lines.append("received no")
    else:
        poses = node.poses.poses
        lines.append(f"frame_id {node.poses.header.frame_id}")
        lines.append(f"waypoint_count {len(poses)}")
        lines.append(f"csv_count {len(expected)}")
        mismatch = 0
        limit = min(len(poses), len(expected))
        for index in range(limit):
            pose = poses[index]
            got = [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
            if any(abs(got[item] - expected[index][item]) > 1e-5 for item in range(7)):
                mismatch += 1
        mismatch += abs(len(poses) - len(expected))
        lines.append(f"mismatched_poses {mismatch}")
        same_frame = node.poses.header.frame_id == args.frame_id
        lines.append(f"same_frame {str(same_frame).lower()}")
        if mismatch == 0 and same_frame and len(poses) == len(expected) and len(poses) > 1:
            status = "PASS"
    lines.append(f"comparison {status}")
    text = "\n".join(lines) + "\n"
    with open(args.output, "w", encoding="utf-8") as handle:
        handle.write(text)
    print(text, end="")
    node.destroy_node()
    rclpy.shutdown()
    if status != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
