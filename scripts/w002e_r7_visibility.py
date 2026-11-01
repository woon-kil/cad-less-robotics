#!/usr/bin/env python3
"""W-002E-R7 가시성만 확인한다. solvePnP와 재투영은 하지 않는다.

방향 일치는 검출 전에 고정한다. ID2의 corner0→corner1 방향이
ID0, ID1, ID3 중앙값과 20° 안이면 기존 설치 방향과 같다.
20°는 W-002E-R4의 코너 순서 일치 한계와 같다.
"""

import json
import math
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import w002e_register as reg

OUT = Path("/ws/cad-less-robotics/evidence/W-002E-R7")
ORIENTATION_LIMIT_DEG = 20.0


def wrap_deg(value):
    return (value + 180.0) % 360.0 - 180.0


def direction_deg(quad):
    vector = quad[1] - quad[0]
    return math.degrees(math.atan2(float(vector[1]), float(vector[0])))


class OneColor(Node):
    def __init__(self):
        super().__init__("w002e_r7_visibility")
        self.message = None
        self.create_subscription(Image, "/camera/color/image_raw", self.on_image, 10)

    def on_image(self, message):
        if self.message is None:
            self.message = message


def main():
    import cv2
    import numpy as np

    rclpy.init()
    node = OneColor()
    try:
        while rclpy.ok() and node.message is None:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    message = node.message
    if message is None or message.encoding != "rgb8":
        raise RuntimeError("color frame missing")
    color = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.width, 3
    )
    OUT.mkdir(parents=True, exist_ok=True)
    bgr = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
    raw_after = OUT / "id2_after_install.jpg"
    raw_visibility = OUT / "four_marker_visibility.jpg"
    if not cv2.imwrite(str(raw_after), bgr):
        raise RuntimeError("raw write failed")
    raw_visibility.write_bytes(raw_after.read_bytes())

    found = reg.detect_all(color)
    per_marker = {}
    canvas = bgr.copy()
    for marker_id in sorted(found):
        quad = found[marker_id]
        sides = [float(np.linalg.norm(quad[(i + 1) % 4] - quad[i])) for i in range(4)]
        inside = bool(
            np.all(quad[:, 0] >= 0)
            and np.all(quad[:, 0] <= message.width - 1)
            and np.all(quad[:, 1] >= 0)
            and np.all(quad[:, 1] <= message.height - 1)
            and min(sides) >= 8.0
        )
        per_marker[str(marker_id)] = {
            "full_visible": inside,
            "corner0_to_corner1_deg": direction_deg(quad),
            "min_side_px": min(sides),
        }
        polygon = quad.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(canvas, [polygon], True, (0, 220, 0), 2)
        origin = tuple(quad[0].astype(int))
        cv2.putText(
            canvas,
            f"ID{marker_id}",
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 220, 0),
            2,
        )
    cv2.imwrite(str(OUT / "detected_ids.png"), canvas)

    others = [
        per_marker[str(marker_id)]["corner0_to_corner1_deg"]
        for marker_id in (0, 1, 3)
        if str(marker_id) in per_marker
    ]
    orientation = "UNRESOLVED"
    orientation_delta_deg = None
    if "2" in per_marker and len(others) == 3:
        median = float(np.median(others))
        orientation_delta_deg = abs(wrap_deg(per_marker["2"]["corner0_to_corner1_deg"] - median))
        orientation = "YES" if orientation_delta_deg <= ORIENTATION_LIMIT_DEG else "NO"

    detected = sorted(found)
    payload = {
        "frame_id": message.header.frame_id,
        "width": int(message.width),
        "height": int(message.height),
        "detected_ids": detected,
        "detected_count": len(detected),
        "per_marker": per_marker,
        "all_markers_full_visible": reg.full_visible(found, message.width, message.height),
        "id2_orientation_contract_match": orientation,
        "id2_direction_delta_from_other_median_deg": orientation_delta_deg,
        "orientation_limit_deg": ORIENTATION_LIMIT_DEG,
        "corner_refinement": "CORNER_REFINE_NONE",
        "solve_pnp": False,
    }
    (OUT / "visibility.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload))
    return 0


if __name__ == "__main__":
    sys.exit(main())
