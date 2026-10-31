#!/usr/bin/env python3
"""W-002A. RGB에서 ArUco ID 0만 표시한다. 작업판 정합은 하지 않는다."""

import json
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from p008b_acquire import image_array

ROOT = Path("/ws/cad-less-robotics/evidence/W-002A/live")
STOP = Path("/ws/cad-less-robotics/evidence/W-002A/STOP")
DICTIONARY_ID = cv2.aruco.DICT_4X4_50


def qos(reliable):
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)


def detector():
    if hasattr(cv2.aruco, "ArucoDetector"):
        dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY_ID)
        return cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    dictionary = cv2.aruco.Dictionary_get(DICTIONARY_ID)
    parameters = cv2.aruco.DetectorParameters_create()
    return dictionary, parameters


def detect(image, model):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    if isinstance(model, tuple):
        corners, ids, _rejected = cv2.aruco.detectMarkers(gray, model[0], parameters=model[1])
    else:
        corners, ids, _rejected = model.detectMarkers(gray)
    if ids is None:
        return None
    for index, value in enumerate(ids.flatten()):
        if int(value) == 0:
            return np.asarray(corners[index], dtype=np.float64).reshape(4, 2)
    return None


def side_lengths(corners):
    width = 0.5 * (np.linalg.norm(corners[1] - corners[0]) + np.linalg.norm(corners[2] - corners[3]))
    height = 0.5 * (np.linalg.norm(corners[3] - corners[0]) + np.linalg.norm(corners[2] - corners[1]))
    return float(width), float(height)


class View(Node):
    def __init__(self):
        super().__init__("w002a_view")
        self.color = None
        self.depth = None
        self.color_frame = None
        self.depth_frame = None
        self.info = None
        for reliable in (True, False):
            self.create_subscription(Image, "/camera/color/image_raw", self.on_color, qos(reliable))
            self.create_subscription(Image, "/camera/depth/image_raw", self.on_depth, qos(reliable))
            self.create_subscription(CameraInfo, "/camera/color/camera_info", self.on_info, qos(reliable))

    def on_color(self, message):
        if message.encoding == "rgb8" and message.width > 0:
            self.color = image_array(message)
            self.color_frame = message.header.frame_id

    def on_depth(self, message):
        if message.encoding == "16UC1" and message.width > 0:
            self.depth = image_array(message)
            self.depth_frame = message.header.frame_id

    def on_info(self, message):
        self.info = message


def depth_at(view, u, v):
    if view.depth is None or view.info is None:
        return None
    if view.depth_frame != view.color_frame:
        return None
    if view.depth.shape[0] != view.color.shape[0] or view.depth.shape[1] != view.color.shape[1]:
        return None
    height, width = view.depth.shape
    x = int(round(u))
    y = int(round(v))
    if not (0 <= x < width and 0 <= y < height):
        return None
    raw = int(view.depth[y, x])
    if raw <= 0:
        return None
    z = raw / 1000.0
    fx, fy = view.info.k[0], view.info.k[4]
    cx, cy = view.info.k[2], view.info.k[5]
    return {
        "center_depth_m": z,
        "camera_xyz_m": [float((u - cx) * z / fx), float((v - cy) * z / fy), z],
    }


def draw(image, corners, depth):
    canvas = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    lines = []
    if corners is None:
        lines.append("NOT_DETECTED")
    else:
        polygon = corners.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(canvas, [polygon], True, (0, 220, 0), 2)
        for point in corners.astype(np.int32):
            cv2.circle(canvas, tuple(point), 5, (0, 220, 0), -1)
        center = corners.mean(axis=0)
        cv2.circle(canvas, (int(round(center[0])), int(round(center[1]))), 6, (0, 0, 255), -1)
        width, height = side_lengths(corners)
        lines.append("DETECTED")
        lines.append("marker ID 0")
        lines.append("corners %s" % np.round(corners, 1).tolist())
        lines.append("center pixel %.1f %.1f" % (center[0], center[1]))
        lines.append("marker pixel width %.1f" % width)
        lines.append("marker pixel height %.1f" % height)
        if depth is None:
            lines.append("center_depth_m unavailable")
        else:
            lines.append("center_depth_m %.4f" % depth["center_depth_m"])
            xyz = depth["camera_xyz_m"]
            lines.append("camera_xyz_m %.4f %.4f %.4f" % (xyz[0], xyz[1], xyz[2]))
    y = 28
    for line in lines:
        cv2.putText(canvas, line[:110], (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        y += 28
    return canvas


def write_outputs(canvas, status):
    ROOT.mkdir(parents=True, exist_ok=True)
    os.chmod(ROOT, 0o755)
    ok, encoded = cv2.imencode(".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    if ok:
        temporary = ROOT / "preview.jpg.tmp"
        temporary.write_bytes(encoded.tobytes())
        temporary.replace(ROOT / "preview.jpg")
        os.chmod(ROOT / "preview.jpg", 0o644)
    temporary = ROOT / "status.json.tmp"
    temporary.write_text(json.dumps(status, ensure_ascii=False))
    temporary.replace(ROOT / "status.json")
    os.chmod(ROOT / "status.json", 0o644)


def main():
    ROOT.mkdir(parents=True, exist_ok=True)
    if STOP.exists():
        STOP.unlink()
    rclpy.init()
    node = View()
    model = detector()
    last = 0.0
    while rclpy.ok() and not STOP.exists():
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if node.color is None or now - last < 0.1:
            continue
        last = now
        corners = detect(node.color, model)
        status = {
            "detected": corners is not None,
            "marker_id": 0 if corners is not None else None,
            "dictionary": "DICT_4X4_50",
            "marker_size_mm": 60,
            "color_frame": node.color_frame,
            "depth_frame": node.depth_frame,
            "corners_px": None if corners is None else corners.round(2).tolist(),
            "center_px": None,
            "pixel_width": None,
            "pixel_height": None,
            "center_depth_m": None,
            "camera_xyz_m": None,
            "workboard_frame_registered": False,
        }
        depth = None
        if corners is not None:
            center = corners.mean(axis=0)
            width, height = side_lengths(corners)
            depth = depth_at(node, float(center[0]), float(center[1]))
            status.update({
                "center_px": [round(float(center[0]), 2), round(float(center[1]), 2)],
                "pixel_width": round(width, 2),
                "pixel_height": round(height, 2),
            })
            if depth is not None:
                status["center_depth_m"] = depth["center_depth_m"]
                status["camera_xyz_m"] = depth["camera_xyz_m"]
        write_outputs(draw(node.color, corners, depth), status)
        print("DETECTED" if corners is not None else "NOT_DETECTED", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
