#!/usr/bin/env python3
"""W-002C. 현재 RGB 한 장에서 DICT_4X4_50 ID를 모두 읽는다. PnP는 하지 않는다."""

import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from p008b_acquire import image_array

OUT = Path("/ws/cad-less-robotics/evidence/W-002C")


def qos(reliable):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class Once(Node):
    def __init__(self):
        super().__init__("w002c_capture")
        self.color = None
        self.frame = None
        self.encoding = None
        for reliable in (True, False):
            self.create_subscription(Image, "/camera/color/image_raw", self.on_color, qos(reliable))

    def on_color(self, message):
        if message.encoding == "rgb8" and message.width > 0 and self.color is None:
            self.color = image_array(message)
            self.frame = message.header.frame_id
            self.encoding = message.encoding


def side_lengths(corners):
    width = 0.5 * (np.linalg.norm(corners[1] - corners[0]) + np.linalg.norm(corners[2] - corners[3]))
    height = 0.5 * (np.linalg.norm(corners[3] - corners[0]) + np.linalg.norm(corners[2] - corners[1]))
    return float(width), float(height)


def main():
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
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()
    gray = cv2.cvtColor(color, cv2.COLOR_RGB2GRAY)
    corners, ids, _rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    found = []
    canvas = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
    if ids is not None:
        for index, value in enumerate(ids.flatten()):
            quad = np.asarray(corners[index], dtype=np.float64).reshape(4, 2)
            width, height = side_lengths(quad)
            center = quad.mean(axis=0)
            found.append({
                "id": int(value),
                "center_px": [round(float(center[0]), 2), round(float(center[1]), 2)],
                "corners_px": quad.round(2).tolist(),
                "pixel_width": round(width, 2),
                "pixel_height": round(height, 2),
            })
            polygon = quad.astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(canvas, [polygon], True, (0, 220, 0), 2)
            cv2.putText(
                canvas,
                str(int(value)),
                (int(round(center[0])), int(round(center[1]))),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 220, 0),
                2,
                cv2.LINE_AA,
            )
    cv2.imwrite(str(OUT / "detected_ids.png"), canvas)
    report = {
        "image_width": int(color.shape[1]),
        "image_height": int(color.shape[0]),
        "channel_order": "RGB",
        "image_encoding": node.encoding,
        "frame_id": node.frame,
        "dictionary": "DICT_4X4_50",
        "dictionary_enum": int(cv2.aruco.DICT_4X4_50),
        "opencv_version": cv2.__version__,
        "detector_parameters": "DetectorParameters_create default",
        "pnp_executed": False,
        "detected_ids": sorted(item["id"] for item in found),
        "detected_count": len(found),
        "markers": found,
    }
    (OUT / "detection.json").write_text(json.dumps(report, indent=2) + "\n")
    node.destroy_node()
    rclpy.shutdown()
    print("ids", report["detected_ids"], "n", report["detected_count"])


if __name__ == "__main__":
    main()
