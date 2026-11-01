#!/usr/bin/env python3
"""W-002F-R3. ChArUco RGB-D 관측만 기록한다.

solvePnP, 등록, TEST A~D, 실패한 ArUco 변환은 사용하지 않는다.
"""

import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

OUT = Path("/ws/cad-less-robotics/evidence/W-002F-R3")
TARGET_FRAMES = 10
STAMP_MATCH_SEC = 0.05
FRAME_GAP_SEC = 0.15
COLOR_FRAME = "camera_color_optical_frame"
PACKAGE_XML = Path("/ws/cad-less-robotics/third_party/OrbbecSDK_ROS2/orbbec_camera/package.xml")
SERIAL_LOG = Path("/tmp/w002a_gemini.log")


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
    if message.encoding == "rgb8":
        return np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.width, 3
        ).copy()
    if message.encoding == "16UC1":
        return np.frombuffer(message.data, dtype=np.uint16).reshape(
            message.height, message.width
        ).copy()
    raise RuntimeError(f"지원하지 않는 encoding: {message.encoding}")


def read_driver_version():
    root = ET.parse(PACKAGE_XML).getroot()
    version = None
    for child in list(root):
        if child.tag.endswith("version"):
            version = child.text
    return version


def read_serial_log():
    serial = None
    firmware = None
    if not SERIAL_LOG.is_file():
        return {"serial": None, "firmware": None, "source": "LOG_ABSENT"}
    for line in SERIAL_LOG.read_text(errors="replace").splitlines():
        if "Serial number:" in line:
            serial = line.split("Serial number:", 1)[1].strip()
        if "Firmware version:" in line:
            firmware = line.split("Firmware version:", 1)[1].strip()
    return {"serial": serial, "firmware": firmware, "source": str(SERIAL_LOG)}


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


def detect_charuco(color, board, dictionary):
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    corners, ids, _rejected = cv2.aruco.detectMarkers(color, dictionary, parameters=parameters)
    marker_ids = [] if ids is None else [int(value) for value in ids.reshape(-1)]
    return corners, marker_ids


def interpolate_charuco(color, board, corners, marker_ids, info):
    if not marker_ids:
        return np.empty((0, 2), dtype=np.float32), []
    ids = np.array(marker_ids, dtype=np.int32).reshape(-1, 1)
    count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners,
        ids,
        color,
        board,
        None,
        None,
        camera_matrix(info),
        distortion(info),
    )
    if count is None or int(count) <= 0 or charuco_ids is None:
        return np.empty((0, 2), dtype=np.float32), []
    points = np.asarray(charuco_corners, dtype=np.float32).reshape(-1, 2)
    found_ids = [int(value) for value in np.asarray(charuco_ids).reshape(-1)]
    return points, found_ids


def printed_mask(marker_corners, shape):
    height, width = shape[:2]
    mask = np.zeros((height, width), dtype=np.uint8)
    if not marker_corners:
        return mask
    points = np.concatenate([np.asarray(quad, dtype=np.float32).reshape(-1, 2) for quad in marker_corners])
    hull = cv2.convexHull(points)
    cv2.fillConvexPoly(mask, np.round(hull).astype(np.int32), 255)
    bounds = cv2.boundingRect(hull)
    inset = max(5, int(round(0.02 * min(bounds[2], bounds[3]))))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (inset * 2 + 1, inset * 2 + 1))
    eroded = cv2.erode(mask, kernel)
    return eroded if int(eroded.sum()) > 0 else mask


def depth_ratio(depth, mask):
    selected = mask > 0
    total = int(selected.sum())
    if total == 0:
        return {"valid_pixel_count": 0, "total_pixel_count": 0, "valid_ratio": None}
    valid = int(np.count_nonzero(depth[selected] > 0))
    return {
        "valid_pixel_count": valid,
        "total_pixel_count": total,
        "valid_ratio": valid / total,
    }


def pattern_touches_border(marker_corners, width, height, margin_px=2):
    if not marker_corners:
        return True
    points = np.concatenate([np.asarray(quad, dtype=np.float32).reshape(-1, 2) for quad in marker_corners])
    return bool(
        np.any(points[:, 0] < margin_px)
        or np.any(points[:, 1] < margin_px)
        or np.any(points[:, 0] > width - 1 - margin_px)
        or np.any(points[:, 1] > height - 1 - margin_px)
    )


class Grab(Node):
    def __init__(self):
        super().__init__("w002f_r3_observe")
        self.color = None
        self.depth = None
        self.info = None
        self.color_stamp = None
        self.depth_stamp = None
        self.color_frame = None
        self.depth_frame = None
        for reliable in (True, False):
            self.create_subscription(Image, "/camera/color/image_raw", self.on_color, qos(reliable))
            self.create_subscription(Image, "/camera/depth/image_raw", self.on_depth, qos(reliable))
            self.create_subscription(CameraInfo, "/camera/color/camera_info", self.on_info, qos(reliable))

    def on_color(self, message):
        if message.encoding != "rgb8" or message.width <= 0:
            return
        stamp = stamp_sec(message.header.stamp)
        if self.color_stamp == stamp:
            return
        self.color = image_array(message)
        self.color_stamp = stamp
        self.color_frame = message.header.frame_id

    def on_depth(self, message):
        if message.encoding != "16UC1" or message.width <= 0:
            return
        stamp = stamp_sec(message.header.stamp)
        if self.depth_stamp == stamp:
            return
        self.depth = image_array(message)
        self.depth_stamp = stamp
        self.depth_frame = message.header.frame_id

    def on_info(self, message):
        self.info = message

    def ready_pair(self):
        if self.color is None or self.depth is None or self.info is None:
            return False
        if self.color_frame != COLOR_FRAME:
            return False
        if self.depth.shape[:2] != self.color.shape[:2]:
            return False
        return abs(self.color_stamp - self.depth_stamp) <= STAMP_MATCH_SEC


def draw_detection(color, marker_corners, marker_ids, charuco_points, charuco_ids):
    canvas = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
    if marker_corners:
        ids = np.array(marker_ids, dtype=np.int32).reshape(-1, 1)
        cv2.aruco.drawDetectedMarkers(canvas, marker_corners, ids)
    for point, corner_id in zip(charuco_points, charuco_ids):
        center = (int(round(point[0])), int(round(point[1])))
        cv2.circle(canvas, center, 4, (0, 255, 255), -1)
        cv2.putText(
            canvas, str(corner_id), center, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA
        )
    return canvas


def main():
    if cv2.__version__ != "4.5.4":
        raise RuntimeError(f"OpenCV {cv2.__version__}")
    if hasattr(cv2.aruco, "CharucoDetector") or hasattr(cv2.aruco.CharucoBoard_create(7, 5, 0.03, 0.015, cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)), "setLegacyPattern"):
        raise RuntimeError("이 런타임에 최신 ChArUco API가 있다")
    for folder in ("frames/color", "frames/depth", "figures", "metrics"):
        (OUT / folder).mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard_create(7, 5, 0.030, 0.015, dictionary)
    rclpy.init()
    node = Grab()
    frames = []
    last_stamp = -1.0
    deadline = time.monotonic() + 40.0
    while len(frames) < TARGET_FRAMES and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if not node.ready_pair():
            continue
        if node.color_stamp <= last_stamp + FRAME_GAP_SEC:
            continue
        color = node.color.copy()
        depth = node.depth.copy()
        info = intrinsics(node.info)
        last_stamp = node.color_stamp
        marker_corners, marker_ids = detect_charuco(color, board, dictionary)
        charuco_points, charuco_ids = interpolate_charuco(
            color, board, marker_corners, marker_ids, info
        )
        mask = printed_mask(marker_corners, color.shape)
        ratio = depth_ratio(depth, mask)
        index = len(frames)
        stem = f"{index:03d}"
        color_path = OUT / "frames/color" / f"{stem}.png"
        depth_path = OUT / "frames/depth" / f"{stem}.png"
        cv2.imwrite(str(color_path), cv2.cvtColor(color, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(depth_path), depth)
        record = {
            "index": index,
            "timestamp_sec": node.color_stamp,
            "depth_timestamp_sec": node.depth_stamp,
            "color_resolution": [int(color.shape[1]), int(color.shape[0])],
            "depth_resolution": [int(depth.shape[1]), int(depth.shape[0])],
            "color_frame": node.color_frame,
            "depth_frame": node.depth_frame,
            "distortion_model": info["distortion_model"],
            "detected_aruco_marker_count": len(marker_ids),
            "detected_marker_ids": marker_ids,
            "detected_charuco_corner_count": len(charuco_ids),
            "charuco_corner_ids": charuco_ids,
            "marker_corners_touch_image_border": pattern_touches_border(
                marker_corners, color.shape[1], color.shape[0]
            ),
            "charuco_depth_valid_pixel_count": ratio["valid_pixel_count"],
            "charuco_depth_total_pixel_count": ratio["total_pixel_count"],
            "charuco_depth_valid_ratio": ratio["valid_ratio"],
            "solve_pnp": False,
        }
        frames.append(record)
        if index == 0:
            cv2.imwrite(
                str(OUT / "figures/charuco_detection.png"),
                draw_detection(color, marker_corners, marker_ids, charuco_points, charuco_ids),
            )
            validity = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
            invalid = (mask > 0) & (depth == 0)
            valid = (mask > 0) & (depth > 0)
            validity[valid] = (0, 180, 0)
            validity[invalid] = (0, 0, 255)
            cv2.imwrite(str(OUT / "figures/depth_validity.png"), validity)
            (OUT / "camera_info.json").write_text(
                json.dumps(info, indent=2) + "\n", encoding="utf-8"
            )
        print(
            f"{index} markers={len(marker_ids)} corners={len(charuco_ids)} "
            f"depth_ratio={ratio['valid_ratio']}",
            flush=True,
        )
    node.destroy_node()
    rclpy.shutdown()
    summary = {
        "opencv_version": cv2.__version__,
        "driver_version": read_driver_version(),
        "device": read_serial_log(),
        "observation_count": len(frames),
        "color_topic": "/camera/color/image_raw",
        "depth_topic": "/camera/depth/image_raw",
        "color_camera_info_topic": "/camera/color/camera_info",
        "solve_pnp_executed": False,
        "failed_aruco_transform_used_as_ground_truth": False,
        "frames": frames,
    }
    (OUT / "metrics/observability.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    if len(frames) < TARGET_FRAMES:
        raise RuntimeError(f"frames {len(frames)}")


if __name__ == "__main__":
    main()
