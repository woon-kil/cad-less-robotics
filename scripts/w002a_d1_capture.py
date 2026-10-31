#!/usr/bin/env python3
"""W-002A-D1. 현재 RGB 한 장과 기본 검출 결과만 저장한다. 파라미터는 바꾸지 않는다."""

import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from p008b_acquire import image_array

OUT = Path("/ws/cad-less-robotics/evidence/W-002A-D1")


def qos(reliable):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class Once(Node):
    def __init__(self):
        super().__init__("w002a_d1_capture")
        self.color = None
        self.frame = None
        for reliable in (True, False):
            self.create_subscription(Image, "/camera/color/image_raw", self.on_color, qos(reliable))

    def on_color(self, message):
        if message.encoding == "rgb8" and message.width > 0 and self.color is None:
            self.color = image_array(message)
            self.frame = message.header.frame_id
            self.encoding = message.encoding


def detect_default(image):
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) if image.ndim == 3 else image
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    found = []
    if ids is not None:
        for index, value in enumerate(ids.flatten()):
            found.append({"id": int(value), "corners": np.asarray(corners[index]).reshape(4, 2).round(2).tolist()})
    rejected_quads = []
    if rejected:
        for quad in rejected:
            rejected_quads.append(np.asarray(quad).reshape(4, 2).round(2).tolist())
    return found, rejected_quads


def main():
    source = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    OUT.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Once()
    deadline = time.monotonic() + 20.0
    while node.color is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.color is None:
        raise SystemExit("RGB 프레임이 없다.")
    color = node.color
    cv2.imwrite(str(OUT / "raw_color.png"), cv2.cvtColor(color, cv2.COLOR_RGB2BGR))
    found, rejected = detect_default(color)
    canvas = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
    for quad in rejected:
        polygon = np.asarray(quad, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(canvas, [polygon], True, (0, 220, 255), 2)
    cv2.imwrite(str(OUT / "rejected_candidates.png"), canvas)
    source_found = None
    if source is not None and source.exists():
        page = cv2.imread(str(source), cv2.IMREAD_COLOR)
        page_rgb = cv2.cvtColor(page, cv2.COLOR_BGR2RGB)
        source_found, _source_rejected = detect_default(page_rgb)
    report = {
        "image_width": int(color.shape[1]),
        "image_height": int(color.shape[0]),
        "channel_order": "RGB",
        "image_encoding": node.encoding,
        "frame_id": node.frame,
        "expected_dictionary": "DICT_4X4_50",
        "actual_dictionary": "DICT_4X4_50",
        "dictionary_enum": int(cv2.aruco.DICT_4X4_50),
        "opencv_version": cv2.__version__,
        "detector_parameters": "DetectorParameters_create default",
        "live_ids": [item["id"] for item in found],
        "live_markers": found,
        "rejected_candidates": rejected,
        "rejected_candidate_count": len(rejected),
        "source_marker_file": None if source is None else str(source),
        "source_ids": None if source_found is None else [item["id"] for item in source_found],
    }
    (OUT / "detector_diagnostic.json").write_text(json.dumps(report, indent=2) + "\n")
    node.destroy_node()
    rclpy.shutdown()
    print("saved", color.shape, "ids", report["live_ids"], "rejected", len(rejected), "source", report["source_ids"])


if __name__ == "__main__":
    main()
