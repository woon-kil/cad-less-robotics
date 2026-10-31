#!/usr/bin/env python3
"""W-002E. 설치된 ArUco 네 장의 한 PnP로 T_camera_workboard를 구한다.

코너 순서는 detectMarkers와 같다. 마커마다 pose를 구해 평균내지 않는다.
임계값은 결과 뒤에 바꾸지 않는다.
"""

import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from p008b_acquire import image_array

OUT = Path("/ws/cad-less-robotics/evidence/W-002E")
DICTIONARY_ID = cv2.aruco.DICT_4X4_50
HALF_M = 0.030
# detectMarkers 0,1,2,3 = TL, TR, BR, BL. yaw 0이면 workboard 축과 같다.
LOCAL_CORNERS_M = np.array([
    [-HALF_M, HALF_M, 0.0],
    [HALF_M, HALF_M, 0.0],
    [HALF_M, -HALF_M, 0.0],
    [-HALF_M, -HALF_M, 0.0],
], dtype=np.float64)
CENTERS_M = {
    0: np.array([0.148, 0.071, 0.0], dtype=np.float64),
    1: np.array([0.500, 0.069, 0.0], dtype=np.float64),
    2: np.array([0.500, 0.336, 0.0], dtype=np.float64),
    3: np.array([0.149, 0.332, 0.0], dtype=np.float64),
}
REFERENCE_DISTANCE_MM = {
    "ID0_ID1": 352.006,
    "ID1_ID2": 267.000,
    "ID2_ID3": 351.023,
    "ID3_ID0": 261.002,
    "ID0_ID2": 440.601,
    "ID1_ID3": 438.600,
}
REFERENCE_PAIRS = [(0, 1), (1, 2), (2, 3), (3, 0), (0, 2), (1, 3)]
ROI_U = (480, 700)
ROI_V = (430, 560)
FRAME_GAP_SEC = 0.15
STAMP_MATCH_SEC = 0.05
TARGET_FRAMES = 10
# 대표 변환: translation mm와 rotation deg의 합이 다른 샘플에 대해 가장 작은 샘플.
# 동점이면 가장 작은 프레임 인덱스. 반복성 숫자는 그 대표에서의 최댓값이다.
MIN_PLANE_POINTS = 500
DEPTH_Z_MIN_M = 0.30
DEPTH_Z_MAX_M = 1.50


def qos(reliable):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def dump(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")


def matrix_list(matrix):
    return np.asarray(matrix, dtype=np.float64).round(12).tolist()


def pnp_flag():
    flag = getattr(cv2, "SOLVEPNP_SQPNP", None)
    if flag is None:
        raise SystemExit("SOLVEPNP_SQPNP가 없다. 다른 solver로 바꾸지 않고 중단한다.")
    return int(flag)


def read_running_serial():
    path = Path("/tmp/w002a_gemini.log")
    if not path.is_file():
        return {"serial": None, "source": "LOG_ABSENT"}
    serial = None
    firmware = None
    for line in path.read_text(errors="replace").splitlines():
        if "Serial number:" in line:
            serial = line.split("Serial number:", 1)[1].strip()
        if "Firmware version:" in line:
            firmware = line.split("Firmware version:", 1)[1].strip()
    return {
        "serial": serial,
        "firmware": firmware,
        "source": "RUNNING_DRIVER_LOG" if serial else "NOT_IN_RUNNING_LOG",
        "path": str(path),
    }


def object_points_for(marker_id):
    return LOCAL_CORNERS_M + CENTERS_M[marker_id]


def detect_all(color):
    dictionary = cv2.aruco.Dictionary_get(DICTIONARY_ID)
    parameters = cv2.aruco.DetectorParameters_create()
    gray = cv2.cvtColor(color, cv2.COLOR_RGB2GRAY)
    corners, ids, _rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    found = {}
    if ids is None:
        return found
    for index, value in enumerate(ids.flatten()):
        marker_id = int(value)
        quad = np.asarray(corners[index], dtype=np.float64).reshape(4, 2)
        found[marker_id] = quad
    return found


def full_visible(found, width, height):
    if sorted(found) != [0, 1, 2, 3]:
        return False
    for quad in found.values():
        if np.any(quad[:, 0] < 0) or np.any(quad[:, 0] > width - 1):
            return False
        if np.any(quad[:, 1] < 0) or np.any(quad[:, 1] > height - 1):
            return False
        sides = [np.linalg.norm(quad[(i + 1) % 4] - quad[i]) for i in range(4)]
        if min(sides) < 8.0:
            return False
    return True


def stack_correspondence(found):
    objects = []
    images = []
    labels = []
    for marker_id in (0, 1, 2, 3):
        obj = object_points_for(marker_id)
        img = found[marker_id]
        for corner_index in range(4):
            objects.append(obj[corner_index])
            images.append(img[corner_index])
            labels.append({"id": marker_id, "corner": corner_index})
    return (
        np.asarray(objects, dtype=np.float64),
        np.asarray(images, dtype=np.float64),
        labels,
    )


def homogeneous(rotation, translation):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def rotation_report(rotation):
    product = rotation.T @ rotation
    ortho = float(np.linalg.norm(product - np.eye(3)))
    determinant = float(np.linalg.det(rotation))
    return {
        "rtr_minus_i_frobenius": ortho,
        "det": determinant,
        "pass": bool(ortho < 1e-6 and abs(determinant - 1.0) < 1e-6),
    }


def inverse_report(transform, inverse):
    residual = float(np.linalg.norm(transform @ inverse - np.eye(4)))
    return {"t_times_inverse_minus_i_frobenius": residual, "pass": bool(residual < 1e-6)}


def so3_angle_deg(rotation_a, rotation_b):
    relative = rotation_a.T @ rotation_b
    cosine = (np.trace(relative) - 1.0) / 2.0
    cosine = float(np.clip(cosine, -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def solve_one(object_pts, image_pts, camera_matrix, distortion, flag):
    ok, rvec, tvec = cv2.solvePnP(
        object_pts,
        image_pts,
        camera_matrix,
        distortion,
        flags=flag,
    )
    if not ok:
        return None
    rotation, _jacobian = cv2.Rodrigues(rvec)
    translation = np.asarray(tvec, dtype=np.float64).reshape(3)
    rotation = np.asarray(rotation, dtype=np.float64)
    projected, _ = cv2.projectPoints(object_pts, rvec, tvec, camera_matrix, distortion)
    projected = projected.reshape(-1, 2)
    errors = np.linalg.norm(projected - image_pts, axis=1)
    camera_points = (rotation @ object_pts.T).T + translation
    board_normal_in_camera = rotation @ np.array([0.0, 0.0, 1.0])
    return {
        "rvec": np.asarray(rvec, dtype=np.float64).reshape(3),
        "tvec": translation,
        "rotation": rotation,
        "translation": translation,
        "projected": projected,
        "errors": errors,
        "camera_points": camera_points,
        "board_normal_in_camera": board_normal_in_camera,
    }


def reprojection_block(errors, labels):
    per_corner = []
    by_marker = {marker_id: [] for marker_id in (0, 1, 2, 3)}
    for error, label in zip(errors, labels):
        value = float(error)
        per_corner.append({
            "id": label["id"],
            "corner": label["corner"],
            "error_px": value,
        })
        by_marker[label["id"]].append(value)
    per_marker = {}
    for marker_id, values in by_marker.items():
        arr = np.asarray(values, dtype=np.float64)
        per_marker[str(marker_id)] = {
            "mean_px": float(arr.mean()),
            "max_px": float(arr.max()),
            "corners_px": [float(item) for item in arr],
        }
    return {
        "mean_px": float(np.mean(errors)),
        "median_px": float(np.median(errors)),
        "p95_px": float(np.percentile(errors, 95)),
        "max_px": float(np.max(errors)),
        "percentile_method": "numpy.percentile linear",
        "independent": False,
        "per_marker": per_marker,
        "per_corner": per_corner,
    }


def intrinsics_of(message):
    k = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(message.d, dtype=np.float64).reshape(-1)
    return {
        "frame_id": message.header.frame_id,
        "width": int(message.width),
        "height": int(message.height),
        "distortion_model": message.distortion_model,
        "fx": float(k[0, 0]),
        "fy": float(k[1, 1]),
        "cx": float(k[0, 2]),
        "cy": float(k[1, 2]),
        "k": matrix_list(k),
        "d": [float(value) for value in distortion],
        "camera_matrix": k,
        "distortion": distortion,
    }


def backproject(u, v, z_m, camera_matrix):
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    x = (u - cx) * z_m / fx
    y = (v - cy) * z_m / fy
    return np.array([x, y, z_m], dtype=np.float64)


def marker_mask(shape, found):
    mask = np.zeros(shape[:2], dtype=np.uint8)
    for quad in found.values():
        center = quad.mean(axis=0)
        expanded = center + (quad - center) * 1.8
        polygon = np.round(expanded).astype(np.int32).reshape(-1, 1, 2)
        cv2.fillPoly(mask, [polygon], 1)
    return mask


def plane_from_depth(depth, found, camera_matrix, transform_workboard_camera, color_shape):
    if depth.shape[0] != color_shape[0] or depth.shape[1] != color_shape[1]:
        return {"executed": False, "reason": "DEPTH_SIZE_MISMATCH"}
    occupied = marker_mask(color_shape, found)
    v0, v1 = ROI_V
    u0, u1 = ROI_U
    rows = []
    cols = []
    zs = []
    for v in range(v0, v1 + 1):
        for u in range(u0, u1 + 1):
            if occupied[v, u]:
                continue
            raw = int(depth[v, u])
            if raw <= 0:
                continue
            z_m = raw / 1000.0
            if z_m < DEPTH_Z_MIN_M or z_m > DEPTH_Z_MAX_M:
                continue
            rows.append(v)
            cols.append(u)
            zs.append(z_m)
    count = len(zs)
    if count < MIN_PLANE_POINTS:
        return {
            "executed": True,
            "point_count": count,
            "min_points": MIN_PLANE_POINTS,
            "reason": "TOO_FEW_VALID_DEPTH",
            "roi_u": [u0, u1],
            "roi_v": [v0, v1],
            "marker_pixels_excluded": True,
        }
    camera_points = np.vstack([
        backproject(float(u), float(v), z, camera_matrix)
        for u, v, z in zip(cols, rows, zs)
    ])
    rotation = transform_workboard_camera[:3, :3]
    translation = transform_workboard_camera[:3, 3]
    board = (rotation @ camera_points.T).T + translation
    abs_z_mm = np.abs(board[:, 2]) * 1000.0
    centered = board - board.mean(axis=0)
    _u, _s, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    cosine = float(np.clip(normal[2], -1.0, 1.0))
    angle = float(np.degrees(np.arccos(cosine)))
    return {
        "executed": True,
        "point_count": count,
        "min_points": MIN_PLANE_POINTS,
        "roi_u": [u0, u1],
        "roi_v": [v0, v1],
        "roi_source": "W-002C candidate, marker quads expanded 1.8x excluded",
        "depth_z_gate_m": [DEPTH_Z_MIN_M, DEPTH_Z_MAX_M],
        "marker_pixels_excluded": True,
        "abs_z_mean_mm": float(abs_z_mm.mean()),
        "abs_z_p95_mm": float(np.percentile(abs_z_mm, 95)),
        "abs_z_median_mm": float(np.median(abs_z_mm)),
        "normal_sign_rule": "flip so normal_z >= 0, then angle to +Z",
        "normal_workboard": matrix_list(normal),
        "normal_angle_deg": angle,
        "z_mm": abs_z_mm,
    }


def known_distance(depth, found, camera_matrix, transform_workboard_camera):
    centers_board = {}
    detail = {}
    reliable = True
    for marker_id, quad in found.items():
        center = quad.mean(axis=0)
        u = int(round(center[0]))
        v = int(round(center[1]))
        window = []
        for dv in range(-3, 4):
            for du in range(-3, 4):
                uu = u + du
                vv = v + dv
                if not (0 <= uu < depth.shape[1] and 0 <= vv < depth.shape[0]):
                    continue
                raw = int(depth[vv, uu])
                if raw <= 0:
                    continue
                z_m = raw / 1000.0
                if DEPTH_Z_MIN_M <= z_m <= DEPTH_Z_MAX_M:
                    window.append(backproject(float(uu), float(vv), z_m, camera_matrix))
        detail[str(marker_id)] = {"valid_pixels_in_7x7": len(window)}
        if len(window) < 4:
            reliable = False
            continue
        camera_point = np.median(np.vstack(window), axis=0)
        rotation = transform_workboard_camera[:3, :3]
        translation = transform_workboard_camera[:3, 3]
        centers_board[marker_id] = rotation @ camera_point + translation
    if not reliable or len(centers_board) < 4:
        return {
            "status": "NOT_RELIABLE",
            "source": "DERIVED_FROM_W002D_R1_MEASURED_CENTERS",
            "independent_physical_metrology": False,
            "rule": "7x7 center window needs >= 4 valid depth pixels on every marker",
            "markers": detail,
        }
    distances = {}
    for a, b in REFERENCE_PAIRS:
        key = f"ID{a}_ID{b}"
        measured = float(np.linalg.norm(centers_board[a] - centers_board[b]) * 1000.0)
        reference = REFERENCE_DISTANCE_MM[key]
        distances[key] = {
            "depth_derived_mm": measured,
            "layout_derived_mm": reference,
            "delta_mm": measured - reference,
        }
    return {
        "status": "COMPUTED_NOT_INDEPENDENT",
        "source": "DERIVED_FROM_W002D_R1_MEASURED_CENTERS",
        "independent_physical_metrology": False,
        "distances": distances,
        "markers": detail,
    }


def draw_overlay(color, found, roi):
    canvas = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
    u0, u1 = roi[0]
    v0, v1 = roi[1]
    cv2.rectangle(canvas, (u0, v0), (u1, v1), (255, 180, 0), 2)
    for marker_id, quad in found.items():
        polygon = np.round(quad).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(canvas, [polygon], True, (0, 220, 0), 2)
        cv2.circle(canvas, tuple(np.round(quad[0]).astype(int)), 6, (0, 0, 255), -1)
        center = quad.mean(axis=0)
        cv2.putText(
            canvas,
            str(marker_id),
            (int(round(center[0])), int(round(center[1]))),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 220, 0),
            2,
            cv2.LINE_AA,
        )
    return canvas


class Grab(Node):
    def __init__(self):
        super().__init__("w002e_register")
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
        if self.color_stamp is None or self.depth_stamp is None:
            return False
        return abs(self.color_stamp - self.depth_stamp) <= STAMP_MATCH_SEC


def select_medoid(samples):
    count = len(samples)
    scores = []
    for i in range(count):
        total = 0.0
        for j in range(count):
            if i == j:
                continue
            translation = np.linalg.norm(samples[i]["translation"] - samples[j]["translation"]) * 1000.0
            rotation = so3_angle_deg(samples[i]["rotation"], samples[j]["rotation"])
            total += translation + rotation
        scores.append(total)
    best = min(range(count), key=lambda index: (scores[index], index))
    return best, scores


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for name in ("detections", "transforms", "reprojection", "plane_validation", "repeatability"):
        (OUT / name).mkdir(parents=True, exist_ok=True)
    flag = pnp_flag()
    device = read_running_serial()
    rclpy.init()
    node = Grab()
    samples = []
    last_stamp = -1.0
    deadline = time.monotonic() + 90.0
    rejected = []
    while len(samples) < TARGET_FRAMES and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if not node.ready_pair():
            continue
        if node.color_stamp <= last_stamp + FRAME_GAP_SEC:
            continue
        color = node.color.copy()
        depth = node.depth.copy()
        info = intrinsics_of(node.info)
        found = detect_all(color)
        last_stamp = node.color_stamp
        visible = full_visible(found, color.shape[1], color.shape[0])
        frame_index = len(samples) + len(rejected)
        record = {
            "attempt": frame_index,
            "color_stamp_sec": node.color_stamp,
            "depth_stamp_sec": node.depth_stamp,
            "color_frame": node.color_frame,
            "depth_frame": node.depth_frame,
            "detected_ids": sorted(found),
            "detected_count": len(found),
            "full_visible_4_4": visible,
        }
        if not visible or node.color_frame != "camera_color_optical_frame":
            rejected.append(record)
            continue
        object_pts, image_pts, labels = stack_correspondence(found)
        solved = solve_one(object_pts, image_pts, info["camera_matrix"], info["distortion"], flag)
        if solved is None:
            record["pnp_success"] = False
            rejected.append(record)
            continue
        transform = homogeneous(solved["rotation"], solved["translation"])
        inverse = np.linalg.inv(transform)
        errors = reprojection_block(solved["errors"], labels)
        sample = {
            "index": len(samples),
            "color": color,
            "depth": depth,
            "found": found,
            "info": info,
            "labels": labels,
            "object_pts": object_pts,
            "rotation": solved["rotation"],
            "translation": solved["translation"],
            "transform": transform,
            "inverse": inverse,
            "errors": errors,
            "camera_points": solved["camera_points"],
            "board_normal_in_camera": solved["board_normal_in_camera"],
            "color_stamp": node.color_stamp,
            "depth_stamp": node.depth_stamp,
            "color_frame": node.color_frame,
            "depth_frame": node.depth_frame,
            "rotation_validity": rotation_report(solved["rotation"]),
            "inverse_consistency": inverse_report(transform, inverse),
        }
        samples.append(sample)
        print("frame", sample["index"], "mean_px", round(errors["mean_px"], 4), flush=True)
    node.destroy_node()
    rclpy.shutdown()

    dump(OUT / "installed_layout.json", {
        "center_xy_source": "W-002D-R1_MEASURED",
        "marker_size_source": "USER_ACCEPTED_NOMINAL",
        "z_source": "NOMINAL_PLANAR_INSTALLATION",
        "yaw_source": "INSTALLATION_CONTRACT",
        "black_square_mm": 60,
        "measurement_resolution_mm": 1,
        "measurement_resolution_is_accuracy_claim": False,
        "centers_m": {str(key): matrix_list(value) for key, value in CENTERS_M.items()},
        "local_corners_m": matrix_list(LOCAL_CORNERS_M),
        "corner_order": ["TOP_LEFT", "TOP_RIGHT", "BOTTOM_RIGHT", "BOTTOM_LEFT"],
        "corner_order_source": "scripts/p007f_07c_mount_geometry.py object_points",
        "yaw_deg": 0,
        "reference_distance_mm": REFERENCE_DISTANCE_MM,
        "reference_distances_are_separate_measurements": False,
    })

    if not samples:
        dump(OUT / "result.json", {
            "work_order": "W-002E",
            "status": "FAIL_MARKER_DETECTION",
            "closed": True,
            "pnp_success": False,
            "accepted_frames": 0,
            "rejected": rejected,
            "device": device,
            "registration_executed": True,
            "next_gate_opened": False,
        })
        print("NO_FRAMES")
        return

    info = samples[0]["info"]
    dump(OUT / "camera_intrinsics.json", {
        "topic": "/camera/color/camera_info",
        "image_topic": "/camera/color/image_raw",
        "depth_topic": "/camera/depth/image_raw",
        "frame_id": info["frame_id"],
        "image_width": info["width"],
        "image_height": info["height"],
        "fx": info["fx"],
        "fy": info["fy"],
        "cx": info["cx"],
        "cy": info["cy"],
        "distortion_model": info["distortion_model"],
        "distortion_coefficients": info["d"],
        "k": info["k"],
        "source": "this run CameraInfo, not a copied file",
        "device": device,
        "opencv_version": cv2.__version__,
        "pnp_flag": "SOLVEPNP_SQPNP",
        "pnp_flag_value": flag,
    })

    for sample in samples:
        index = sample["index"]
        markers = []
        for marker_id, quad in sorted(sample["found"].items()):
            markers.append({
                "id": marker_id,
                "corners_px": np.round(quad, 3).tolist(),
                "center_px": np.round(quad.mean(axis=0), 3).tolist(),
            })
        dump(OUT / "detections" / f"frame_{index:02d}.json", {
            "index": index,
            "color_stamp_sec": sample["color_stamp"],
            "depth_stamp_sec": sample["depth_stamp"],
            "color_frame": sample["color_frame"],
            "depth_frame": sample["depth_frame"],
            "detected_ids": [0, 1, 2, 3],
            "detected_count": 4,
            "full_visible_4_4": True,
            "markers": markers,
        })
        dump(OUT / "transforms" / f"frame_{index:02d}.json", {
            "index": index,
            "rvec_rodrigues": matrix_list(cv2.Rodrigues(sample["rotation"])[0].reshape(3)),
            "tvec_m": matrix_list(sample["translation"]),
            "T_camera_workboard": matrix_list(sample["transform"]),
            "T_workboard_camera": matrix_list(sample["inverse"]),
            "rotation_validity": sample["rotation_validity"],
            "inverse_consistency": sample["inverse_consistency"],
            "board_normal_in_camera": matrix_list(sample["board_normal_in_camera"]),
            "camera_point_z_m": matrix_list(sample["camera_points"][:, 2]),
        })
        dump(OUT / "reprojection" / f"frame_{index:02d}.json", sample["errors"])

    medoid, scores = select_medoid(samples)
    chosen = samples[medoid]
    translations = []
    rotations = []
    for sample in samples:
        translations.append(float(np.linalg.norm(sample["translation"] - chosen["translation"]) * 1000.0))
        rotations.append(so3_angle_deg(chosen["rotation"], sample["rotation"]))
    repeatability = {
        "frame_count": len(samples),
        "rejected_attempts": rejected,
        "method": "SE3_MEDOID_MIN_SUM_TRANSLATION_MM_PLUS_ROTATION_DEG",
        "tie_break": "LOWEST_INDEX",
        "medoid_index": medoid,
        "medoid_scores": scores,
        "translation_definition": "MAX_EUCLIDEAN_MM_FROM_MEDOID",
        "rotation_definition": "MAX_SO3_ANGLE_DEG_FROM_MEDOID",
        "translation_deviation_mm": translations,
        "rotation_deviation_deg": rotations,
        "translation_repeatability_mm": float(max(translations)),
        "rotation_repeatability_deg": float(max(rotations)),
        "thresholds": {"translation_mm": 2.0, "rotation_deg": 0.4, "n_min": 10},
    }
    dump(OUT / "repeatability" / "summary.json", repeatability)

    depth_same_frame = chosen["depth_frame"] == chosen["color_frame"]
    if depth_same_frame:
        plane = plane_from_depth(
            chosen["depth"],
            chosen["found"],
            chosen["info"]["camera_matrix"],
            chosen["inverse"],
            chosen["color"].shape,
        )
        z_mm = plane.pop("z_mm", None)
        if z_mm is not None:
            np.save(OUT / "plane_validation" / "abs_z_mm.npy", np.asarray(z_mm, dtype=np.float32))
        known = known_distance(
            chosen["depth"],
            chosen["found"],
            chosen["info"]["camera_matrix"],
            chosen["inverse"],
        )
    else:
        plane = {
            "executed": False,
            "reason": "DEPTH_FRAME_IS_NOT_COLOR_FRAME",
            "depth_frame": chosen["depth_frame"],
            "color_frame": chosen["color_frame"],
        }
        known = {
            "status": "NOT_RELIABLE",
            "reason": "DEPTH_FRAME_IS_NOT_COLOR_FRAME",
            "independent_physical_metrology": False,
        }
    dump(OUT / "plane_validation" / "summary.json", plane)
    dump(OUT / "plane_validation" / "known_distance.json", known)

    overlay = draw_overlay(chosen["color"], chosen["found"], (ROI_U, ROI_V))
    cv2.imwrite(str(OUT / "plane_validation" / "roi_overlay.png"), overlay)
    cv2.imwrite(str(OUT / "detections" / "medoid_color.png"), cv2.cvtColor(chosen["color"], cv2.COLOR_RGB2BGR))
    u0, u1 = ROI_U
    v0, v1 = ROI_V
    crop = chosen["color"][v0:v1 + 1, u0:u1 + 1]
    cv2.imwrite(str(OUT / "plane_validation" / "roi_crop.png"), cv2.cvtColor(crop, cv2.COLOR_RGB2BGR))
    np.save(OUT / "plane_validation" / "medoid_depth.npy", chosen["depth"])

    dump(OUT / "T_camera_workboard.json", {
        "name": "T_camera_workboard_FINAL",
        "equation": "p_camera = T_camera_workboard * p_workboard",
        "camera_frame": "camera_color_optical_frame",
        "workboard_frame": "workboard",
        "selection": "SE3_MEDOID",
        "medoid_index": medoid,
        "matrix": matrix_list(chosen["transform"]),
        "tvec_m": matrix_list(chosen["translation"]),
    })
    dump(OUT / "T_workboard_camera.json", {
        "name": "T_workboard_camera_FINAL",
        "equation": "p_workboard = T_workboard_camera * p_camera",
        "inverse_of": "T_camera_workboard_FINAL",
        "inverse_is_independent_calibration": False,
        "matrix": matrix_list(chosen["inverse"]),
    })

    reprojection_pass = (
        chosen["errors"]["mean_px"] <= 1.0
        and chosen["errors"]["p95_px"] <= 2.0
        and chosen["errors"]["max_px"] <= 3.0
    )
    plane_pass = (
        plane.get("executed") is True
        and plane.get("point_count", 0) >= MIN_PLANE_POINTS
        and plane.get("abs_z_mean_mm", 1e9) <= 2.0
        and plane.get("abs_z_p95_mm", 1e9) <= 4.0
        and plane.get("normal_angle_deg", 1e9) <= 1.0
    )
    repeat_pass = (
        len(samples) >= 10
        and repeatability["translation_repeatability_mm"] <= 2.0
        and repeatability["rotation_repeatability_deg"] <= 0.4
    )
    facing = bool(np.all(chosen["camera_points"][:, 2] > 0.0) and chosen["board_normal_in_camera"][2] < 0.0)
    sanity_pass = chosen["rotation_validity"]["pass"] and chosen["inverse_consistency"]["pass"] and facing
    failures = []
    if sorted(chosen["found"]) != [0, 1, 2, 3]:
        failures.append("FAIL_MARKER_DETECTION")
    if not chosen["rotation_validity"]["pass"] or not chosen["inverse_consistency"]["pass"] or not facing:
        failures.append("FAIL_TRANSFORM_SANITY")
    if not reprojection_pass:
        failures.append("FAIL_REPROJECTION")
    if not plane_pass:
        failures.append("FAIL_WORKBOARD_PLANE")
    if not repeat_pass:
        failures.append("FAIL_REPEATABILITY")
    status = "PASS_WORKBOARD_REGISTRATION" if not failures else failures[0]
    result = {
        "work_order": "W-002E",
        "status": status,
        "closed": True,
        "failure_labels": failures,
        "first_failure": failures[0] if failures else None,
        "date": "2026-10-31",
        "camera_frame": chosen["color_frame"],
        "depth_frame": chosen["depth_frame"],
        "detected_ids": [0, 1, 2, 3],
        "detected_count": 4,
        "pnp_method": "SOLVEPNP_SQPNP",
        "pnp_success": True,
        "correspondence_count": 16,
        "per_marker_pose_averaged": False,
        "reprojection_independent": False,
        "reprojection_mean_px": chosen["errors"]["mean_px"],
        "reprojection_p95_px": chosen["errors"]["p95_px"],
        "reprojection_max_px": chosen["errors"]["max_px"],
        "reprojection_median_px": chosen["errors"]["median_px"],
        "workboard_plane": {key: value for key, value in plane.items() if key != "z_mm"},
        "repeatability_frame_count": len(samples),
        "translation_repeatability_mm": repeatability["translation_repeatability_mm"],
        "rotation_repeatability_deg": repeatability["rotation_repeatability_deg"],
        "known_distance_validation": known["status"],
        "rotation_validity": chosen["rotation_validity"]["pass"],
        "inverse_consistency": chosen["inverse_consistency"]["pass"],
        "points_in_front_of_camera": bool(np.all(chosen["camera_points"][:, 2] > 0.0)),
        "board_normal_faces_camera": bool(chosen["board_normal_in_camera"][2] < 0.0),
        "final_transform_selected": True,
        "final_transform_method": repeatability["method"],
        "medoid_index": medoid,
        "T_camera_workboard_final": matrix_list(chosen["transform"]),
        "T_workboard_camera_final": matrix_list(chosen["inverse"]),
        "camera_moved": False,
        "robot_moved": False,
        "workboard_moved": False,
        "markers_repositioned": False,
        "thresholds_relaxed": False,
        "p009_r1_opened": False,
        "p010_opened": False,
        "next_gate_opened": False,
        "workpiece_instance_selected": False,
        "historical_evidence_modified": False,
        "registration_executed": True,
    }
    dump(OUT / "result.json", result)
    print("STATUS", status)
    print("REPROJ", result["reprojection_mean_px"], result["reprojection_p95_px"], result["reprojection_max_px"])
    print("PLANE", plane.get("point_count"), plane.get("abs_z_mean_mm"), plane.get("abs_z_p95_mm"), plane.get("normal_angle_deg"), plane.get("reason"))
    print("REPEAT", result["translation_repeatability_mm"], result["rotation_repeatability_deg"])
    print("KNOWN", known["status"])


if __name__ == "__main__":
    main()
