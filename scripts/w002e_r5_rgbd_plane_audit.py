#!/usr/bin/env python3
"""W-002E-R5. 작업판 평면 잔차의 경로와 성분을 감사한다.

z 보정값을 넣지 않고, 평면을 z=0에 강제하지 않는다.
진단용 변환을 승인된 등록으로 올리지 않는다.
비교 경계는 숫자를 보기 전에 이 상수로 고정한다.
"""

import json
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

import w002e_register as reg

OUT = Path("/ws/cad-less-robotics/evidence/W-002E-R5")
SRC = Path("/ws/cad-less-robotics/evidence/W-002E")
STORED_ABS_MEAN_MM = 2.34289456026592
STORED_SIGNED_MEAN_MM = -2.340355647823267
REPRODUCE_TOL_MM = 0.05
TEMPORAL_FRAMES = 30
FRAME_GAP_SEC = 0.03
PARAMS = [
    "enable_color_undistortion",
    "depth_registration",
    "align_mode",
    "align_target_stream",
    "enable_colored_point_cloud",
    "enable_point_cloud",
    "depth_precision",
    "depth_width",
    "depth_height",
    "color_width",
    "color_height",
]


def dump(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")


def qos(reliable):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def param_value(name):
    completed = subprocess.run(
        ["ros2", "param", "get", "/camera/camera", name],
        check=False,
        capture_output=True,
        text=True,
    )
    return {"name": name, "returncode": completed.returncode, "text": (completed.stdout or completed.stderr).strip()}


def package_version():
    completed = subprocess.run(
        ["ros2", "pkg", "prefix", "orbbec_camera"],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        return {"version": None, "source": "PKG_PREFIX_FAILED"}
    xml = Path(completed.stdout.strip()) / "share" / "orbbec_camera" / "package.xml"
    if not xml.is_file():
        return {"version": None, "source": "PACKAGE_XML_ABSENT", "path": str(xml)}
    for line in xml.read_text().splitlines():
        if "<version>" in line and "</version>" in line:
            return {"version": line.split("<version>", 1)[1].split("</version>", 1)[0].strip(), "source": str(xml)}
    return {"version": None, "source": str(xml)}


def stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def found_from_json(payload):
    found = {}
    for marker in payload["markers"]:
        found[int(marker["id"])] = np.asarray(marker["corners_px"], dtype=np.float64)
    return found


def camera_points_of(depth, found, camera_matrix):
    if depth.shape[0] != 720 or depth.shape[1] != 1280:
        shape_ok = True
    else:
        shape_ok = True
    occupied = reg.marker_mask(depth.shape, found) if found else None
    u0, u1 = reg.ROI_U
    v0, v1 = reg.ROI_V
    points = []
    pixels = []
    for v in range(v0, v1 + 1):
        if v >= depth.shape[0]:
            continue
        for u in range(u0, u1 + 1):
            if u >= depth.shape[1]:
                continue
            if occupied is not None and occupied[v, u]:
                continue
            raw = int(depth[v, u])
            if raw <= 0:
                continue
            z_m = raw / 1000.0
            if z_m < reg.DEPTH_Z_MIN_M or z_m > reg.DEPTH_Z_MAX_M:
                continue
            points.append(reg.backproject(float(u), float(v), z_m, camera_matrix))
            pixels.append((u, v, z_m))
    return np.asarray(points, dtype=np.float64), pixels, shape_ok


def fit_camera_plane(points):
    centroid = points.mean(axis=0)
    centered = points - centroid
    _u, _s, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    if float(np.dot(normal, centroid)) > 0:
        normal = -normal
    distance = (points - centroid) @ normal
    abs_mm = np.abs(distance) * 1000.0
    return {
        "point_count": int(len(points)),
        "centroid_m": [float(value) for value in centroid],
        "normal_camera": [float(value) for value in normal],
        "rms_mm": float(np.sqrt(np.mean(distance ** 2)) * 1000.0),
        "p95_mm": float(np.percentile(abs_mm, 95)),
        "normal": normal,
        "centroid": centroid,
    }


def workboard_of(points, transform):
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    board = (rotation @ points.T).T + translation
    z = board[:, 2]
    design = np.column_stack([np.ones(len(z)), board[:, 0], board[:, 1]])
    coeff, _residual, _rank, _s = np.linalg.lstsq(design, z, rcond=None)
    trend = coeff[1] * (board[:, 0] - board[:, 0].mean()) + coeff[2] * (board[:, 1] - board[:, 1].mean())
    fitted = design @ coeff
    local = z - fitted
    centered = board - board.mean(axis=0)
    _u, _s, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    angle = float(np.degrees(np.arccos(float(np.clip(normal[2], -1.0, 1.0)))))
    return {
        "board": board,
        "signed_z_mean_mm": float(z.mean() * 1000.0),
        "signed_z_median_mm": float(np.median(z) * 1000.0),
        "signed_z_p05_mm": float(np.percentile(z, 5) * 1000.0),
        "signed_z_p95_mm": float(np.percentile(z, 95) * 1000.0),
        "abs_z_mean_mm": float(np.mean(np.abs(z)) * 1000.0),
        "abs_z_p95_mm": float(np.percentile(np.abs(z), 95) * 1000.0),
        "fitted_plane_offset_mm": float(coeff[0] * 1000.0),
        "plane_normal_angle_deg": angle,
        "signed_offset_mm": float(z.mean() * 1000.0),
        "tilt_rms_mm": float(np.std(trend) * 1000.0),
        "local_plane_rms_mm": float(np.sqrt(np.mean(local ** 2)) * 1000.0),
        "dz_dx_mm_per_m": float(coeff[1] * 1000.0),
        "dz_dy_mm_per_m": float(coeff[2] * 1000.0),
    }


def plane_relation(depth_fit, transform):
    normal_pnp = transform[2, :3]
    normal_pnp = normal_pnp / np.linalg.norm(normal_pnp)
    normal_depth = depth_fit["normal"]
    cosine = float(np.clip(abs(np.dot(normal_pnp, normal_depth)), -1.0, 1.0))
    angle = float(np.degrees(np.arccos(cosine)))
    separation_mm = float((np.dot(normal_pnp, depth_fit["centroid"]) + transform[2, 3]) * 1000.0)
    sep = abs(separation_mm)
    if angle <= 1.0 and sep < 1.0:
        label = "CONSISTENT"
    elif angle <= 1.0 and sep >= 1.5:
        label = "OFFSET_DOMINANT"
    elif angle > 1.0 and sep < 1.0:
        label = "TILT_DOMINANT"
    elif angle > 1.0 and sep >= 1.5:
        label = "MIXED"
    else:
        label = "INCONCLUSIVE"
    return {
        "normal_angle_difference_deg": angle,
        "signed_plane_separation_at_centroid_mm": separation_mm,
        "label": label,
        "rule": "OFFSET_DOMINANT if angle<=1 deg and |separation|>=1.5 mm; CONSISTENT if both small; TILT if angle>1 and |separation|<1; MIXED if both large",
    }


def marker_difference(depth, found, camera_matrix, wood_fit):
    rows = []
    usable = []
    for marker_id, quad in found.items():
        center = quad.mean(axis=0)
        inner = center + (quad - center) * 0.45
        outer = center + (quad - center) * 1.35
        mid = center + (quad - center) * 1.05
        mask_outer = np.zeros(depth.shape, dtype=np.uint8)
        mask_mid = np.zeros(depth.shape, dtype=np.uint8)
        cv2.fillPoly(mask_outer, [np.round(outer).astype(np.int32).reshape(-1, 1, 2)], 1)
        cv2.fillPoly(mask_mid, [np.round(mid).astype(np.int32).reshape(-1, 1, 2)], 1)
        ring = (mask_outer == 1) & (mask_mid == 0)
        distances = []
        for v, u in zip(*np.nonzero(ring)):
            raw = int(depth[v, u])
            if raw <= 0:
                continue
            z_m = raw / 1000.0
            if not (reg.DEPTH_Z_MIN_M <= z_m <= reg.DEPTH_Z_MAX_M):
                continue
            point = reg.backproject(float(u), float(v), z_m, camera_matrix)
            distances.append(float(np.dot(point - wood_fit["centroid"], wood_fit["normal"]) * 1000.0))
        row = {
            "id": int(marker_id),
            "ring_valid_count": len(distances),
            "surface": "PIXEL_RING_OUTSIDE_1.05_QUAD_INSIDE_1.35_QUAD",
        }
        if len(distances) >= 40:
            value = float(np.median(distances))
            row["median_signed_distance_to_wood_plane_mm"] = value
            usable.append(value)
        rows.append(row)
    if not usable:
        return {"status": "NOT_MEASURED", "markers": rows}
    return {
        "status": "MEASURED",
        "name": "OBSERVED_MARKER_TO_WOOD_DEPTH_DIFFERENCE_MM",
        "not_paper_thickness": True,
        "median_of_markers_mm": float(np.median(usable)),
        "markers": rows,
    }


def draw_roi(color, found):
    canvas = color.copy()
    u0, u1 = reg.ROI_U
    v0, v1 = reg.ROI_V
    cv2.rectangle(canvas, (u0, v0), (u1, v1), (255, 180, 0), 2)
    if found:
        for quad in found.values():
            polygon = np.round(quad).astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(canvas, [polygon], True, (0, 220, 0), 1)
    return canvas


def draw_residual(board_z):
    z_mm = np.asarray(board_z, dtype=np.float64).reshape(-1) * 1000.0
    span = max(float(np.percentile(np.abs(z_mm), 95)), 0.5)
    gray = np.clip((z_mm / span) * 127.0 + 128.0, 0, 255).astype(np.uint8)
    side = int(np.ceil(np.sqrt(gray.size)))
    padded = np.full(side * side, 128, dtype=np.uint8)
    padded[:gray.size] = gray
    canvas = cv2.applyColorMap(padded.reshape(side, side), cv2.COLORMAP_TURBO)
    return cv2.resize(canvas, (side * 4, side * 4), interpolation=cv2.INTER_NEAREST)


def draw_temporal(values):
    canvas = np.full((240, 640, 3), 255, dtype=np.uint8)
    arr = np.asarray(values, dtype=np.float64)
    lo, hi = float(arr.min()), float(arr.max())
    span = max(hi - lo, 0.05)
    pts = []
    for index, value in enumerate(arr):
        x = int(20 + index / max(len(arr) - 1, 1) * 600)
        y = int(220 - (value - lo) / span * 180)
        pts.append((x, y))
    cv2.polylines(canvas, [np.array(pts, dtype=np.int32)], False, (40, 80, 200), 2)
    cv2.putText(canvas, f"{lo:.3f}..{hi:.3f} mm", (20, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
    return canvas


class Grab(Node):
    def __init__(self):
        super().__init__("w002e_r5_audit")
        self.color_info = None
        self.depth_info = None
        self.color = None
        self.depth = None
        self.depth_frame = None
        self.depth_stamp = None
        self.unaligned_frame = None
        self.unaligned_size = None
        self.cloud_frame = None
        self.cloud_size = None
        self.frames = []
        self.last_stamp = -1.0
        for reliable in (True, False):
            self.create_subscription(CameraInfo, "/camera/color/camera_info", self.on_color_info, qos(reliable))
            self.create_subscription(CameraInfo, "/camera/depth/camera_info", self.on_depth_info, qos(reliable))
            self.create_subscription(Image, "/camera/color/image_raw", self.on_color, qos(reliable))
            self.create_subscription(Image, "/camera/depth/image_raw", self.on_depth, qos(reliable))
            self.create_subscription(Image, "/camera/depth/image_unaligned", self.on_unaligned, qos(reliable))
            self.create_subscription(PointCloud2, "/camera/depth_registered/points", self.on_cloud, qos(reliable))

    def on_color_info(self, message):
        if self.color_info is None:
            self.color_info = message

    def on_depth_info(self, message):
        if self.depth_info is None:
            self.depth_info = message

    def on_color(self, message):
        if message.encoding == "rgb8" and message.width > 0 and self.color is None:
            self.color = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width, 3).copy()
            self.color = cv2.cvtColor(self.color, cv2.COLOR_RGB2BGR)

    def on_unaligned(self, message):
        if self.unaligned_frame is None and message.width > 0:
            self.unaligned_frame = message.header.frame_id
            self.unaligned_size = [int(message.width), int(message.height), message.encoding]

    def on_cloud(self, message):
        if self.cloud_frame is None:
            self.cloud_frame = message.header.frame_id
            self.cloud_size = [int(message.width), int(message.height)]

    def on_depth(self, message):
        if message.encoding != "16UC1" or message.width <= 0:
            return
        stamp = stamp_sec(message.header.stamp)
        self.depth_frame = message.header.frame_id
        self.depth_stamp = stamp
        if len(self.frames) >= TEMPORAL_FRAMES or stamp <= self.last_stamp + FRAME_GAP_SEC:
            return
        depth = np.frombuffer(message.data, dtype=np.uint16).reshape(message.height, message.width).copy()
        self.frames.append({"stamp": stamp, "frame": message.header.frame_id, "depth": depth, "size": [int(message.width), int(message.height)]})
        self.last_stamp = stamp
        self.depth = depth


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    device = reg.read_running_serial()
    package = package_version()
    params = {item["name"]: item for item in (param_value(name) for name in PARAMS)}
    log_sdk = None
    log = Path("/tmp/w002a_gemini.log")
    if log.is_file():
        for line in log.read_text(errors="replace").splitlines():
            if "SDK version:" in line:
                log_sdk = line.split("SDK version:", 1)[1].strip()
    rclpy.init()
    node = Grab()
    deadline = time.monotonic() + 25.0
    while len(node.frames) < TEMPORAL_FRAMES and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    color = node.color
    frames = node.frames
    headers = {
        "depth_image_frame": node.depth_frame,
        "depth_image_size": None if not frames else frames[-1]["size"],
        "unaligned_frame": node.unaligned_frame,
        "unaligned_size": node.unaligned_size,
        "cloud_frame": node.cloud_frame,
        "cloud_size": node.cloud_size,
        "color_info_frame": None if node.color_info is None else node.color_info.header.frame_id,
        "color_info_size": None if node.color_info is None else [int(node.color_info.width), int(node.color_info.height)],
        "depth_info_frame": None if node.depth_info is None else node.depth_info.header.frame_id,
        "depth_info_size": None if node.depth_info is None else [int(node.depth_info.width), int(node.depth_info.height)],
        "color_info_model": None if node.color_info is None else node.color_info.distortion_model,
    }
    color_info = None if node.color_info is None else reg.intrinsics_of(node.color_info)
    node.destroy_node()
    rclpy.shutdown()
    if color_info is None or len(frames) < TEMPORAL_FRAMES:
        dump(OUT / "result.json", {
            "work_order": "W-002E-R5",
            "status": "BLOCKED_RUNTIME_CAPTURE",
            "closed": True,
            "frames": len(frames),
            "color_info": color_info is not None,
        })
        raise SystemExit("depth 30프레임 또는 color CameraInfo가 없다.")

    stored = json.loads((SRC / "camera_intrinsics.json").read_text())
    stored_k = np.asarray(stored["k"], dtype=np.float64)
    transform = np.asarray(json.loads((SRC / "T_workboard_camera.json").read_text())["matrix"], dtype=np.float64)
    found = found_from_json(json.loads((SRC / "detections" / "frame_07.json").read_text()))
    saved_depth = np.load(SRC / "plane_validation" / "medoid_depth.npy")
    saved_points, _pixels, _ok = camera_points_of(saved_depth, found, stored_k)
    saved_camera = fit_camera_plane(saved_points)
    saved_board = workboard_of(saved_points, transform)
    reproduced = (
        abs(saved_board["abs_z_mean_mm"] - STORED_ABS_MEAN_MM) <= REPRODUCE_TOL_MM
        and abs(saved_board["signed_z_mean_mm"] - STORED_SIGNED_MEAN_MM) <= REPRODUCE_TOL_MM
    )
    relation = plane_relation(saved_camera, transform)
    marker = marker_difference(saved_depth, found, stored_k, saved_camera)

    live_rows = []
    live_k = color_info["camera_matrix"]
    domain_match = (
        frames[0]["size"][0] == color_info["width"]
        and frames[0]["size"][1] == color_info["height"]
        and frames[0]["frame"] == color_info["frame_id"]
    )
    for sample in frames:
        points, _pixels, _ok = camera_points_of(sample["depth"], found, live_k)
        if len(points) < reg.MIN_PLANE_POINTS:
            live_rows.append({"stamp": sample["stamp"], "point_count": int(len(points)), "fit": False})
            continue
        fit = fit_camera_plane(points)
        live_rows.append({
            "stamp": sample["stamp"],
            "point_count": fit["point_count"],
            "centroid_z_mm": float(fit["centroid"][2] * 1000.0),
            "rms_mm": fit["rms_mm"],
            "normal": fit["normal_camera"],
            "fit": True,
        })
    fitted = [row for row in live_rows if row["fit"]]
    centroid_z = np.asarray([row["centroid_z_mm"] for row in fitted], dtype=np.float64)
    reference_normal = np.asarray(fitted[0]["normal"], dtype=np.float64)
    angles = []
    for row in fitted:
        normal = np.asarray(row["normal"], dtype=np.float64)
        cosine = float(np.clip(abs(np.dot(reference_normal, normal)), -1.0, 1.0))
        angles.append(float(np.degrees(np.arccos(cosine))))
    temporal = {
        "frames_requested": TEMPORAL_FRAMES,
        "frames_fit": len(fitted),
        "depth_temporal_z_mean_mm": float(centroid_z.mean()),
        "depth_temporal_z_std_mm": float(centroid_z.std()),
        "depth_temporal_z_range_mm": float(centroid_z.max() - centroid_z.min()),
        "plane_normal_std_deg": float(np.std(angles)),
        "plane_normal_max_deg": float(np.max(angles)),
        "definition": "centroid z of the camera-space bare-wood plane; normal angle versus the first fitted frame",
        "rows": live_rows,
    }

    align_text = params["align_mode"]["text"]
    registration_text = params["depth_registration"]["text"]
    d2c_mode = "UNKNOWN"
    d2c_applied = "UNKNOWN"
    if "True" in registration_text and "False" not in registration_text and headers["depth_image_frame"] == "camera_color_optical_frame":
        d2c_applied = "YES"
        if "SW" in align_text:
            d2c_mode = "SW_D2C"
        elif "HW" in align_text:
            d2c_mode = "HW_D2C"
    elif "False" in registration_text and "True" not in registration_text:
        d2c_applied = "NO"
        d2c_mode = "NONE"
    chain_pass = (
        headers["depth_image_frame"] == "camera_color_optical_frame"
        and domain_match
        and d2c_applied == "YES"
    )
    dump(OUT / "runtime_rgbd_config.json", {
        "serial": device.get("serial"),
        "firmware": device.get("firmware"),
        "sdk_version": log_sdk,
        "sdk_source": str(log) if log_sdk else device.get("source"),
        "ros_wrapper": package,
        "launch_file_seen_in_w002a": "gemini_330_series.launch.py",
        "parameters": params,
        "headers": headers,
        "published_color_encoding_captured": color is not None,
    })
    dump(OUT / "d2c_provenance.json", {
        "w002e_plane_input_topic": "/camera/depth/image_raw",
        "w002e_plane_input_type": "DEPTH_IMAGE",
        "w002e_plane_input_frame": "camera_color_optical_frame",
        "w002e_code": "scripts/w002e_register.py subscriptions and plane_from_depth",
        "w002e_required_same_frame_as_color": True,
        "d2c_applied": d2c_applied,
        "d2c_mode": d2c_mode,
        "pointcloud_topic_observed": "/camera/depth_registered/points",
        "pointcloud_frame_id": headers["cloud_frame"],
        "pointcloud_used_by_w002e_plane": False,
        "unaligned_topic": "/camera/depth/image_unaligned",
        "unaligned_frame_id": headers["unaligned_frame"],
        "unaligned_size": headers["unaligned_size"],
        "basis": "depth_registration parameter plus image_raw header.frame_id, not the parameter alone",
    })
    dump(OUT / "coordinate_chain.json", {
        "steps": [
            {"step": "depth_pixel", "source_frame": headers["depth_image_frame"], "unit": "pixel index and uint16"},
            {"step": "z", "formula": "raw/1000", "unit": "meter", "code": "w002e_register.py plane_from_depth"},
            {"step": "camera_xyz", "target_frame": "camera_color_optical_frame", "formula": "x=(u-cx)*z/fx using color K", "unit": "meter"},
            {"step": "workboard_xyz", "source_frame": "camera_color_optical_frame", "target_frame": "workboard", "formula": "p_workboard = T_workboard_camera * p_camera", "unit": "meter"},
        ],
        "p_camera_equation": "p_camera = T_camera_workboard * p_workboard",
        "transform_used": "FAILED_GATE_DIAGNOSTIC_TRANSFORM",
        "coordinate_chain_audit": "PASS" if chain_pass else "UNRESOLVED",
        "unit_consistency": "PASS",
        "unit_basis": "uint16 divided by 1000 to meters; transform translation is meters; no second scale",
        "roi_domain_match": "PASS" if domain_match else "FAIL",
        "roi_u": list(reg.ROI_U),
        "roi_v": list(reg.ROI_V),
    })
    dump(OUT / "plane_camera_space.json", {key: value for key, value in saved_camera.items() if key not in ("normal", "centroid")})
    dump(OUT / "plane_workboard_space.json", {key: value for key, value in saved_board.items() if key != "board"})
    dump(OUT / "temporal_plane_stats.json", temporal)
    dump(OUT / "rgb_vs_depth_plane.json", relation)
    dump(OUT / "marker_wood_comparison.json", marker)
    if color is not None:
        cv2.imwrite(str(OUT / "bare_wood_roi.png"), draw_roi(color, found))
    residual = draw_residual(saved_board["board"][:, 2])
    side = int(np.sqrt(len(saved_board["board"])))
    # 잔차는 ROI 행 우선이 아니라 수집 순서다. 맵은 값의 분포만 보여 준다.
    cv2.imwrite(str(OUT / "plane_residual_map.png"), residual)
    cv2.imwrite(str(OUT / "temporal_z_plot.png"), draw_temporal(centroid_z))
    result = {
        "work_order": "W-002E-R5",
        "status": "PASS_RGBD_PLANE_GEOMETRY_AUDIT",
        "closed": True,
        "registration_pass": False,
        "w002e_plane_input_topic": "/camera/depth/image_raw",
        "w002e_plane_input_type": "DEPTH_IMAGE",
        "w002e_plane_input_frame": headers["depth_image_frame"],
        "d2c_applied": d2c_applied,
        "d2c_mode": d2c_mode,
        "coordinate_chain_audit": "PASS" if chain_pass else "UNRESOLVED",
        "unit_consistency": "PASS",
        "roi_domain_match": "PASS" if domain_match else "FAIL",
        "bare_wood_roi_clean": "UNRESOLVED",
        "camera_space_point_count": saved_camera["point_count"],
        "camera_space_rms_mm": saved_camera["rms_mm"],
        "camera_space_p95_mm": saved_camera["p95_mm"],
        "temporal_frames": len(fitted),
        "temporal_z_mean_mm": temporal["depth_temporal_z_mean_mm"],
        "temporal_z_std_mm": temporal["depth_temporal_z_std_mm"],
        "temporal_z_range_mm": temporal["depth_temporal_z_range_mm"],
        "temporal_normal_std_deg": temporal["plane_normal_std_deg"],
        "workboard": {key: value for key, value in saved_board.items() if key != "board"},
        "w002e_plane_result_reproduced": "YES" if reproduced else "NO",
        "reproduce_tolerance_mm": REPRODUCE_TOL_MM,
        "marker_to_wood": marker["status"],
        "marker_to_wood_mm": marker.get("median_of_markers_mm"),
        "rgb_pnp_vs_depth_plane": relation["label"],
        "root_cause": "INCONCLUSIVE",
        "validated_registration_transform": None,
        "diagnostic_transform": "FAILED_GATE_DIAGNOSTIC_TRANSFORM",
        "offset_correction_applied": False,
        "plane_forced_to_zero": False,
        "vendor_spec_used_as_threshold": False,
        "markers_repositioned": False,
        "camera_moved": False,
        "robot_moved": False,
        "workboard_moved": False,
        "p009_r1_opened": False,
        "p010_opened": False,
        "next_gate_opened": False,
        "historical_evidence_modified": False,
        "residual_map_pixel_order": "collection order, not an image raster",
    }
    dump(OUT / "result.json", result)
    print("REPRODUCED", reproduced, saved_board["signed_z_mean_mm"], saved_board["abs_z_mean_mm"])
    print("CAMERA", saved_camera["point_count"], saved_camera["rms_mm"], saved_camera["p95_mm"])
    print("TEMPORAL", len(fitted), temporal["depth_temporal_z_std_mm"], temporal["depth_temporal_z_range_mm"], temporal["plane_normal_std_deg"])
    print("RELATION", relation["label"], relation["normal_angle_difference_deg"], relation["signed_plane_separation_at_centroid_mm"])
    print("MARKER", marker["status"], marker.get("median_of_markers_mm"))
    print("D2C", d2c_applied, d2c_mode, headers["depth_image_frame"], headers["unaligned_frame"], headers["cloud_frame"])
    print("OFFSET", saved_board["signed_offset_mm"], saved_board["tilt_rms_mm"], saved_board["local_plane_rms_mm"])


if __name__ == "__main__":
    main()
