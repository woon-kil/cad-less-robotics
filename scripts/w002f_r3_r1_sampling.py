#!/usr/bin/env python3
"""W-002F-R3-R1. 같은 검출기로 컬러만 다시 읽고 마커 크기와 ID를 기록한다.

solvePnP, 등록, TEST A~D, 검출 파라미터 변경은 하지 않는다.
"""

import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

OUT = Path("/ws/cad-less-robotics/evidence/W-002F-R3-R1")
BASELINE = Path("/ws/cad-less-robotics/evidence/W-002F-R3/frames/color/000.png")
PREVIEW = Path("/tmp/w002f_r3_r1_preview.png")
TARGET_FRAMES = 10
FRAME_GAP_SEC = 0.15
COLOR_FRAME = "camera_color_optical_frame"
COLOR_TOPIC = "/camera/color/image_raw"


def qos(reliable):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def image_array(message):
    if message.encoding != "rgb8":
        raise RuntimeError(message.encoding)
    return np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.width, 3
    ).copy()


def intrinsics(message):
    return {
        "width": int(message.width),
        "height": int(message.height),
        "distortion_model": message.distortion_model,
        "d": [float(value) for value in message.d],
        "k": [float(value) for value in message.k],
        "frame_id": message.header.frame_id,
    }


def camera_matrix(info):
    return np.array(info["k"], dtype=np.float64).reshape(3, 3)


def distortion(info):
    return np.array(info["d"], dtype=np.float64)


def parameters():
    detector = cv2.aruco.DetectorParameters_create()
    detector.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    return detector


def edge_lengths(points):
    lengths = []
    for index in range(4):
        delta = points[(index + 1) % 4] - points[index]
        lengths.append(float(np.linalg.norm(delta)))
    return lengths


def percentile(values, fraction):
    ordered = np.sort(np.asarray(values, dtype=np.float64))
    if len(ordered) == 0:
        return None
    if len(ordered) == 1:
        return float(ordered[0])
    position = (len(ordered) - 1) * fraction
    low = int(np.floor(position))
    high = int(np.ceil(position))
    weight = position - low
    return float(ordered[low] * (1.0 - weight) + ordered[high] * weight)


def edge_summary(quads):
    means = []
    for quad in quads:
        points = np.asarray(quad, dtype=np.float64).reshape(4, 2)
        means.append(float(np.mean(edge_lengths(points))))
    if not means:
        return {
            "quad_count": 0,
            "marker_edge_mean_px": None,
            "marker_edge_min_px": None,
            "marker_edge_max_px": None,
            "marker_edge_p50_px": None,
        }
    return {
        "quad_count": len(means),
        "marker_edge_mean_px": round(float(np.mean(means)), 3),
        "marker_edge_min_px": round(float(np.min(means)), 3),
        "marker_edge_max_px": round(float(np.max(means)), 3),
        "marker_edge_p50_px": round(percentile(means, 0.50), 3),
    }


def detect(image, dictionary):
    corners, ids, rejected = cv2.aruco.detectMarkers(
        image, dictionary, parameters=parameters()
    )
    marker_ids = [] if ids is None else [int(value) for value in ids.reshape(-1)]
    return corners or [], marker_ids, rejected or []


def interpolate(color, board, corners, marker_ids, info):
    if not marker_ids:
        return [], []
    count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners,
        np.array(marker_ids, dtype=np.int32).reshape(-1, 1),
        color,
        board,
        None,
        None,
        camera_matrix(info),
        distortion(info),
    )
    if count is None or int(count) <= 0 or charuco_ids is None:
        return [], []
    points = np.asarray(charuco_corners, dtype=np.float32).reshape(-1, 2)
    found = [int(value) for value in np.asarray(charuco_ids).reshape(-1)]
    return points.tolist(), found


class Grab(Node):
    def __init__(self):
        super().__init__("w002f_r3_r1_observe")
        self.color = None
        self.info = None
        self.stamp = None
        self.frame_id = None
        for reliable in (True, False):
            self.create_subscription(Image, COLOR_TOPIC, self.on_color, qos(reliable))
            self.create_subscription(
                CameraInfo, "/camera/color/camera_info", self.on_info, qos(reliable)
            )

    def on_color(self, message):
        if message.encoding != "rgb8" or message.width <= 0:
            return
        stamp = stamp_sec(message.header.stamp)
        if self.stamp == stamp:
            return
        self.color = image_array(message)
        self.stamp = stamp
        self.frame_id = message.header.frame_id

    def on_info(self, message):
        self.info = message

    def ready(self):
        return (
            self.color is not None
            and self.info is not None
            and self.frame_id == COLOR_FRAME
        )


def draw(color, corners, marker_ids, charuco_points, charuco_ids):
    canvas = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
    if corners:
        cv2.aruco.drawDetectedMarkers(
            canvas, corners, np.array(marker_ids, dtype=np.int32).reshape(-1, 1)
        )
    for point, corner_id in zip(charuco_points, charuco_ids):
        center = (int(round(point[0])), int(round(point[1])))
        cv2.circle(canvas, center, 4, (0, 255, 255), -1)
        cv2.putText(
            canvas, str(corner_id), center, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA
        )
    return canvas


def main():
    preview = "--preview" in sys.argv
    if cv2.__version__ != "4.5.4":
        raise RuntimeError(cv2.__version__)
    if hasattr(cv2.aruco, "CharucoDetector"):
        raise RuntimeError("CharucoDetector가 있다")
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard_create(7, 5, 0.030, 0.015, dictionary)
    if hasattr(board, "setLegacyPattern"):
        raise RuntimeError("setLegacyPattern이 있다")
    rclpy.init()
    node = Grab()
    deadline = time.monotonic() + 15.0
    while not node.ready() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.ready():
        raise RuntimeError("color frame이 없다")
    if preview:
        color = node.color.copy()
        cv2.imwrite(str(PREVIEW), cv2.cvtColor(color, cv2.COLOR_RGB2BGR))
        baseline = cv2.imread(str(BASELINE), cv2.IMREAD_COLOR)
        current = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
        difference = None
        if baseline is not None and baseline.shape == current.shape:
            difference = float(np.mean(np.abs(baseline.astype(np.int16) - current.astype(np.int16))))
        corners, marker_ids, rejected = detect(color, dictionary)
        print(json.dumps({
            "preview": True,
            "shape": [int(color.shape[1]), int(color.shape[0])],
            "mean_abs_diff_vs_r3_000": difference,
            "accepted": len(marker_ids),
            "rejected": len(rejected),
            "accepted_edges": edge_summary(corners),
            "rejected_edges": edge_summary(rejected),
        }))
        node.destroy_node()
        rclpy.shutdown()
        return
    for folder in ("frames/color", "figures", "metrics"):
        (OUT / folder).mkdir(parents=True, exist_ok=True)
    frames = []
    last_stamp = -1.0
    deadline = time.monotonic() + 40.0
    info = None
    while len(frames) < TARGET_FRAMES and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if not node.ready() or node.stamp <= last_stamp + FRAME_GAP_SEC:
            continue
        color = node.color.copy()
        info = intrinsics(node.info)
        last_stamp = node.stamp
        corners, marker_ids, rejected = detect(color, dictionary)
        charuco_points, charuco_ids = interpolate(color, board, corners, marker_ids, info)
        index = len(frames)
        cv2.imwrite(
            str(OUT / "frames/color" / f"{index:03d}.png"),
            cv2.cvtColor(color, cv2.COLOR_RGB2BGR),
        )
        accepted_edges = edge_summary(corners)
        rejected_edges = edge_summary(rejected)
        record = {
            "index": index,
            "timestamp_sec": node.stamp,
            "color_resolution": [int(color.shape[1]), int(color.shape[0])],
            "color_frame": node.frame_id,
            "detected_marker_count": len(marker_ids),
            "detected_marker_ids": marker_ids,
            "rejected_candidate_count": len(rejected),
            "charuco_corner_count": len(charuco_ids),
            "charuco_corner_ids": charuco_ids,
            "accepted_marker_edges": accepted_edges,
            "rejected_candidate_edges": rejected_edges,
            "solve_pnp": False,
        }
        frames.append(record)
        if index == 0:
            (OUT / "camera_info.json").write_text(json.dumps(info, indent=2) + "\n", encoding="utf-8")
        canvas = draw(color, corners, marker_ids, charuco_points, charuco_ids)
        cv2.imwrite(str(OUT / "figures" / f"detection_{index:03d}.png"), canvas)
        print(
            f"{index} markers={len(marker_ids)} rejected={len(rejected)} corners={len(charuco_ids)}",
            flush=True,
        )
    node.destroy_node()
    rclpy.shutdown()
    if len(frames) < TARGET_FRAMES:
        raise RuntimeError(f"frames {len(frames)}")
    best = max(frames, key=lambda item: item["detected_marker_count"])
    chosen = cv2.imread(str(OUT / "figures" / f"detection_{best['index']:03d}.png"))
    cv2.imwrite(str(OUT / "figures/recovery_detection.png"), chosen)
    baseline = cv2.imread(str(BASELINE), cv2.IMREAD_COLOR)
    current = cv2.imread(str(OUT / "frames/color" / f"{best['index']:03d}.png"), cv2.IMREAD_COLOR)
    pair = np.hstack([
        cv2.resize(baseline, (640, 360)),
        cv2.resize(current, (640, 360)),
    ])
    cv2.imwrite(str(OUT / "figures/baseline_vs_recovery.png"), pair)
    (OUT / "metrics/detection_comparison.json").write_text(
        json.dumps({
            "opencv": cv2.__version__,
            "dictionary": "DICT_5X5_100",
            "detector": "DetectorParameters_create with CORNER_REFINE_NONE",
            "color_topic": COLOR_TOPIC,
            "observation_count": len(frames),
            "baseline_marker_edge_p50_px": 23.486,
            "baseline_detected_marker_count": 0,
            "baseline_rejected_count": 14,
            "frames": frames,
            "solve_pnp_executed": False,
        }, indent=2) + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
