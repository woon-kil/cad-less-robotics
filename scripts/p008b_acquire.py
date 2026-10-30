#!/usr/bin/env python3
"""P-008B — 계약과 preflight가 통과한 뒤에만 Dataset v2 다섯 장을 저장한다.

segmentation, PCA, boundary, robot 명령은 없다.
사용: python3 scripts/p008b_acquire.py <evidence/P-008B> <driver_log>
"""

import json
import re
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformListener

from orbbec_camera_msgs.msg import Extrinsics
from p008b_metrics import (
    PAIR_MAX_SEC,
    color_correspondence,
    depth_cloud_metrics,
    extrinsic_rejection,
    matrix_4x4,
    quaternion_xyzw,
    rotation_valid,
    sha256_file,
    unpack_rgb_little_endian,
    write_png,
    write_xyzrgb_pcd,
)

def qos(reliable, transient=False):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=30,
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE,
    )


def stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def info_dict(message):
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


def image_array(message):
    if message.encoding == "rgb8":
        return np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width, 3).copy()
    if message.encoding == "16UC1":
        return np.frombuffer(message.data, dtype=np.uint16).reshape(message.height, message.width).copy()
    raise RuntimeError(f"지원하지 않는 encoding: {message.encoding}")


def cloud_sample(message):
    fields = [
        {"name": field.name, "offset": int(field.offset), "datatype": int(field.datatype), "count": int(field.count)}
        for field in message.fields
    ]
    names = {field["name"]: field for field in fields}
    meta = {
        "topic": "/camera/depth_registered/points",
        "frame_id": message.header.frame_id,
        "stamp_sec": stamp_sec(message.header.stamp),
        "width": int(message.width),
        "height": int(message.height),
        "point_step": int(message.point_step),
        "is_dense": bool(message.is_dense),
        "is_bigendian": bool(message.is_bigendian),
        "fields": fields,
        "point_count": int(message.width) * int(message.height),
    }
    if message.is_bigendian or not all(name in names for name in ("x", "y", "z", "rgb")):
        return meta, None, None, None
    count = meta["point_count"]
    endian = "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [endian + "f4"] * 3,
            "offsets": [names[name]["offset"] for name in ("x", "y", "z")],
            "itemsize": message.point_step,
        }
    )
    points = np.frombuffer(message.data, dtype=dtype, count=count)
    xyz = np.column_stack((points["x"], points["y"], points["z"])).astype(np.float64)
    rgb = unpack_rgb_little_endian(message.data, names["rgb"]["offset"], message.point_step, count)
    rgb_float = np.frombuffer(message.data, dtype=endian + "f4", count=count * (message.point_step // 4))
    rgb_float = rgb_float.reshape(count, message.point_step // 4)[:, names["rgb"]["offset"] // 4].copy()
    finite = np.isfinite(xyz).all(axis=1)
    return meta, xyz[finite], rgb[finite], rgb_float[finite]


def quat_matrix(x, y, z, w):
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def parse_identity(log_text):
    def grab(pattern):
        match = re.search(pattern, log_text)
        return match.group(1).strip() if match else None

    return {
        "device_name": grab(r"Device (.+) connected"),
        "serial": grab(r"Serial number: (\S+)"),
        "firmware": grab(r"Firmware version: (\S+)"),
        "ros_wrapper_version": grab(r"ROS Wrapper version: (\S+)"),
        "sdk_version": grab(r"SDK version: (\S+)"),
        "align_log": "set align mode to SW" in log_text,
        "frame_sync_log": "Enable frame sync" in log_text,
    }


def write_json(path, payload):
    Path(path).write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")


class Grab(Node):
    def __init__(self):
        super().__init__("p008b_acquire")
        self.colors = []
        self.depths = []
        self.clouds = []
        self.color_info = None
        self.depth_info = None
        self.extrinsic_msg = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        for reliable in (True, False):
            self.create_subscription(Image, "/camera/color/image_raw", self.on_color, qos(reliable))
            self.create_subscription(Image, "/camera/depth/image_raw", self.on_depth, qos(reliable))
            self.create_subscription(PointCloud2, "/camera/depth_registered/points", self.on_cloud, qos(reliable))
            self.create_subscription(CameraInfo, "/camera/color/camera_info", self.on_color_info, qos(reliable))
            self.create_subscription(CameraInfo, "/camera/depth/camera_info", self.on_depth_info, qos(reliable))
        self.create_subscription(
            self._extrinsic_type(),
            "/camera/depth_to_color",
            self.on_extrinsic,
            qos(True, transient=True),
        )

    def _extrinsic_type(self):
        return Extrinsics

    def _push(self, bucket, item):
        bucket.append(item)
        del bucket[:-40]

    def on_color(self, message):
        if message.encoding == "rgb8" and message.width > 0:
            self._push(self.colors, (stamp_sec(message.header.stamp), message.header.frame_id, image_array(message)))

    def on_depth(self, message):
        if message.encoding == "16UC1" and message.width > 0:
            self._push(self.depths, (stamp_sec(message.header.stamp), message.header.frame_id, image_array(message)))

    def on_cloud(self, message):
        if message.width * message.height > 0:
            meta, xyz, rgb, rgb_float = cloud_sample(message)
            if xyz is not None and len(xyz) > 0:
                self._push(self.clouds, (meta, xyz, rgb, rgb_float))

    def on_color_info(self, message):
        self.color_info = info_dict(message)

    def on_depth_info(self, message):
        self.depth_info = info_dict(message)

    def on_extrinsic(self, message):
        self.extrinsic_msg = {
            "header_frame_id": message.header.frame_id,
            "stamp_sec": stamp_sec(message.header.stamp),
            "rotation": [float(value) for value in message.rotation],
            "translation_m": [float(value) for value in message.translation],
        }

    def lookup_optical(self):
        try:
            transform = self.buffer.lookup_transform(
                "camera_color_optical_frame",
                "camera_depth_optical_frame",
                rclpy.time.Time(),
            )
        except Exception:
            return None
        return transform_record(transform)


def transform_record(transform: TransformStamped):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    matrix = quat_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
    return {
        "source": "tf2 lookup_transform",
        "target_frame": transform.header.frame_id,
        "source_frame": transform.child_frame_id,
        "stamp_sec": stamp_sec(transform.header.stamp),
        "translation_m": [float(translation.x), float(translation.y), float(translation.z)],
        "quaternion_xyzw": [float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)],
        "rotation": matrix.reshape(-1).tolist(),
        "matrix_4x4": matrix_4x4(matrix, [translation.x, translation.y, translation.z]).reshape(-1).tolist(),
    }


def best_set(node):
    best = None
    best_span = None
    for color in node.colors:
        for depth in node.depths:
            for cloud in node.clouds:
                stamps = (color[0], depth[0], cloud[0]["stamp_sec"])
                span = max(stamps) - min(stamps)
                if span <= PAIR_MAX_SEC and (best_span is None or span < best_span):
                    best = (color, depth, cloud, stamps, span)
                    best_span = span
    return best


def intrinsics(info):
    return float(info["k"][0]), float(info["k"][4]), float(info["k"][2]), float(info["k"][5])


def measure_pair(color, depth, cloud, depth_info, color_info):
    _stamp, _frame, color_image = color
    _dstamp, depth_frame, depth_image = depth
    meta, xyz, rgb, _rgb_float = cloud
    fx, fy, cx, cy = intrinsics(depth_info)
    depth_metrics = depth_cloud_metrics(xyz, depth_image, fx, fy, cx, cy)
    color_fx, color_fy, color_cx, color_cy = intrinsics(color_info)
    color_metrics = color_correspondence(xyz, rgb, color_image, color_fx, color_fy, color_cx, color_cy)
    color_metrics["depth_frame"] = depth_frame
    color_metrics["cloud_frame"] = meta["frame_id"]
    color_metrics["color_shape"] = list(color_image.shape)
    color_metrics["depth_shape"] = list(depth_image.shape)
    color_metrics["same_resolution"] = color_image.shape[:2] == depth_image.shape[:2]
    return depth_metrics, color_metrics


def save_capture(directory, color, depth, cloud, color_info, depth_info, optical):
    directory.mkdir(parents=True, exist_ok=True)
    color_stamp, color_frame, color_image = color
    depth_stamp, depth_frame, depth_image = depth
    meta, xyz, rgb, rgb_float = cloud
    write_png(directory / "color.png", color_image)
    write_png(directory / "depth.png", depth_image)
    write_xyzrgb_pcd(directory / "cloud.pcd", xyz.astype(np.float32), rgb_float)
    write_json(directory / "color_camera_info.json", color_info)
    write_json(directory / "depth_camera_info.json", depth_info)
    write_json(directory / "pointcloud_metadata.json", meta)
    write_json(
        directory / "timestamps.json",
        {
            "color": color_stamp,
            "depth": depth_stamp,
            "cloud": meta["stamp_sec"],
            "color_depth_delta": abs(color_stamp - depth_stamp),
            "depth_cloud_delta": abs(depth_stamp - meta["stamp_sec"]),
            "color_cloud_delta": abs(color_stamp - meta["stamp_sec"]),
        },
    )
    write_json(directory / "extrinsic_reference.json", optical)
    write_json(
        directory / "alignment_metadata.json",
        {
            "mode": "SW_D2C",
            "color_frame": color_frame,
            "depth_frame": depth_frame,
            "cloud_frame": meta["frame_id"],
            "color_resolution": [int(color_image.shape[1]), int(color_image.shape[0])],
            "depth_resolution": [int(depth_image.shape[1]), int(depth_image.shape[0])],
        },
    )
    depth_metrics, color_metrics = measure_pair(color, depth, cloud, depth_info, color_info)
    write_json(directory / "depth_cloud.json", depth_metrics)
    write_json(directory / "color_correspondence.json", color_metrics)
    return depth_metrics, color_metrics


def positive_ratio(depth):
    return float(np.mean(depth > 0))


def main():
    if len(sys.argv) != 3:
        raise SystemExit("사용: p008b_acquire.py <evidence/P-008B> <driver_log>")
    output = Path(sys.argv[1])
    log_path = Path(sys.argv[2])
    if output.name != "P-008B":
        raise SystemExit("출력은 evidence/P-008B 만 허용한다.")
    contract_path = output / "acquisition_contract.json"
    contract_bytes = contract_path.read_bytes()
    contract = json.loads(contract_bytes)
    output.joinpath("preflight").mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Grab()
    deadline = time.monotonic() + 90.0
    chosen = None
    optical = None
    while time.monotonic() < deadline and chosen is None:
        rclpy.spin_once(node, timeout_sec=0.2)
        optical = node.lookup_optical()
        if node.color_info and node.depth_info and node.extrinsic_msg and optical:
            chosen = best_set(node)
    identity = parse_identity(log_path.read_text(errors="replace")) if log_path.exists() else {}
    write_json(output / "preflight" / "camera_identity.json", identity)
    write_json(output / "preflight" / "sdk_extrinsic_message.json", node.extrinsic_msg)
    write_json(output / "preflight" / "color_depth_extrinsic.json", optical)
    if contract_path.read_bytes() != contract_bytes:
        raise SystemExit("acquisition contract가 측정 중에 바뀌었다.")
    if chosen is None or optical is None or node.extrinsic_msg is None:
        write_json(output / "verdict.json", {"gate": "P-008B", "verdict": "FAIL_PREFLIGHT", "reason": "streams_or_extrinsic_missing"})
        print("FAIL_PREFLIGHT missing streams or extrinsic")
        return
    color, depth, cloud, stamps, span = chosen
    meta, xyz, rgb, rgb_float = cloud
    write_json(
        output / "preflight" / "pointcloud_metadata.json",
        {**meta, "finite_count": int(len(xyz)), "units": "meters", "units_basis": "ob_camera_node.cpp divides xyz by 1000"},
    )
    write_json(
        output / "preflight" / "alignment_verification.json",
        {
            "launch_argument_alone": False,
            "align_log": identity.get("align_log"),
            "frame_sync_log": identity.get("frame_sync_log"),
            "hardware_sync": "NOT_VERIFIED",
            "color_resolution": [int(color[2].shape[1]), int(color[2].shape[0])],
            "depth_resolution": [int(depth[2].shape[1]), int(depth[2].shape[0])],
            "color_frame": color[1],
            "depth_frame": depth[1],
            "cloud_frame": meta["frame_id"],
            "stamp_span_sec": span,
        },
    )
    depth_metrics, color_metrics = measure_pair(color, depth, cloud, node.depth_info, node.color_info)
    write_json(output / "preflight" / "depth_cloud_correspondence.json", depth_metrics)
    write_json(output / "preflight" / "color_correspondence.json", color_metrics)
    write_json(output / "preflight" / "driver_configuration.json", {"identity": identity, "contract_launch": contract["launch"]})
    rotation = np.array(optical["rotation"], dtype=np.float64).reshape(3, 3)
    checks = {
        "CAMERA_IDENTITY": identity.get("serial") == contract["serial"] and identity.get("firmware") == contract["firmware"] and identity.get("ros_wrapper_version") == "2.10.6" and bool(identity.get("device_name") and "335L" in identity["device_name"]),
        "COLOR_STREAM": list(color[2].shape[1::-1]) == contract["color_resolution"] and color[1] == contract["color_frame"],
        "DEPTH_STREAM": depth[2].dtype == np.uint16 and int(np.count_nonzero(depth[2])) > 0,
        "POINTCLOUD_STREAM": "rgb" in [field["name"] for field in meta["fields"]] and len(xyz) > 0,
        "COLOR_DEPTH_EXTRINSIC": extrinsic_rejection(rotation, optical["translation_m"]) is None and node.extrinsic_msg is not None,
        "FRAME_SEMANTICS": color[1] == contract["color_frame"] and depth[1] == contract["depth_frame"] and meta["frame_id"] == contract["cloud_frame"] and optical["target_frame"] == "camera_color_optical_frame" and optical["source_frame"] == "camera_depth_optical_frame",
        "ALIGNMENT_MODE": bool(identity.get("align_log")) and list(depth[2].shape[1::-1]) == contract["published_depth_resolution_after_d2c"] and depth[1] == contract["depth_frame"],
        "DEPTH_CLOUD_CORRESPONDENCE": bool(depth_metrics["passed"]),
        "COLOR_CORRESPONDENCE": bool(color_metrics["passed"] and color_metrics["same_resolution"]),
    }
    write_json(output / "preflight" / "checks.json", checks)
    if contract_path.read_bytes() != contract_bytes:
        raise SystemExit("acquisition contract가 측정 중에 바뀌었다.")
    if not all(checks.values()):
        failed = [name for name, ok in checks.items() if not ok]
        write_json(output / "verdict.json", {"gate": "P-008B", "verdict": "FAIL_PREFLIGHT", "failed": failed, "captures_written": 0})
        print("FAIL_PREFLIGHT " + ",".join(failed))
        return
    used_color_stamps = set()
    capture_results = []
    for index in range(1, 6):
        deadline = time.monotonic() + 20.0
        selected = None
        while time.monotonic() < deadline and selected is None:
            rclpy.spin_once(node, timeout_sec=0.2)
            candidate = best_set(node)
            if candidate and candidate[0][0] not in used_color_stamps:
                selected = candidate
        if selected is None:
            write_json(output / "verdict.json", {"gate": "P-008B", "verdict": "FAIL_CAPTURE_INTEGRITY", "saved": index - 1})
            print(f"FAIL_CAPTURE_INTEGRITY at capture_{index:02d}")
            return
        used_color_stamps.add(selected[0][0])
        capture_dir = output / f"capture_{index:02d}"
        depth_result, color_result = save_capture(capture_dir, selected[0], selected[1], selected[2], node.color_info, node.depth_info, optical)
        capture_results.append({"capture": f"capture_{index:02d}", "depth_passed": depth_result["passed"], "color_passed": color_result["passed"], "positive_depth_ratio": positive_ratio(selected[1][2])})
        print(f"saved capture_{index:02d} depth_pass={depth_result['passed']} color_pass={color_result['passed']}")
    if contract_path.read_bytes() != contract_bytes:
        raise SystemExit("acquisition contract가 측정 중에 바뀌었다.")
    print("CAPTURES_DONE")
    write_json(output / "preflight" / "capture_loop.json", capture_results)


if __name__ == "__main__":
    main()
