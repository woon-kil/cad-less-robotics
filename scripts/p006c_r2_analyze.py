#!/usr/bin/env python3
"""저장된 R2 프레임을 기존 P-006C 검출로 읽는다. workplane 좌표는 만들지 않는다."""

import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p006c_validate_metric as metric

ROOT = Path("evidence/P-006C-R2")
CAPTURES = ROOT / "captures"
THRESHOLDS = Path("evidence/P-006/camera-target/thresholds.yaml")
POSITIONS = ("CENTER", "LEFT", "RIGHT", "ROBOT_SIDE", "CAMERA_SIDE")


def is_identity(transform):
    if not transform:
        return False
    translation = np.array(transform["translation_m"], dtype=np.float64)
    quaternion = np.array(transform["quaternion_xyzw"], dtype=np.float64)
    return float(np.linalg.norm(translation)) < 1e-6 and abs(float(quaternion[3])) > 0.999999


def load_thresholds():
    thresholds = metric.load_yaml(THRESHOLDS)
    rules = json.loads((ROOT / "visibility_rules.json").read_text())
    if rules["thresholds_changed"] is not False:
        raise SystemExit("visibility rule이 threshold 변경을 허용한다")
    return thresholds


def depth_metrics(record, corners, cols, rows, rotation, translation, thresholds):
    empty = {
        "valid_depth_corner_count": 0,
        "valid_depth_corner_ratio": 0.0,
        "board_depth_valid_ratio": None,
        "plane_samples": 0,
        "plane_rmse_m": None,
        "rgb_depth_distance_m": None,
        "normal_angle_deg": None,
    }
    transform = record.get("T_depth_color")
    if not transform or is_identity(transform):
        return empty
    depth_image = metric.read_png(CAPTURES / f"{record['stem']}-depth.png")
    if depth_image.ndim != 2:
        return empty
    depth_rotation = metric.quaternion_to_matrix(transform["quaternion_xyzw"])
    depth_translation = np.array(transform["translation_m"], dtype=np.float64)
    obj = metric.object_points(cols, rows)
    board_in_color = metric.apply_transform(rotation, translation, obj)
    board_in_depth = metric.apply_transform(depth_rotation, depth_translation, board_in_color)
    depth_matrix = metric.camera_matrix(record["depth_info"])
    depth_dist = metric.dist_coeffs(record["depth_info"])
    projected, _ = cv2.projectPoints(board_in_depth, np.zeros(3), np.zeros(3), depth_matrix, depth_dist)
    uv = projected.reshape(-1, 2)
    height, width = depth_image.shape[:2]
    scale = float(thresholds["depth_unit"]["meters_per_count"])
    valid_corners = 0
    for u, v in uv:
        ui, vi = int(round(u)), int(round(v))
        if ui < 2 or vi < 2 or ui >= width - 2 or vi >= height - 2:
            continue
        window = depth_image[vi - 2:vi + 3, ui - 2:ui + 3].astype(np.float64)
        good = window[window > 0]
        if good.size < 5:
            continue
        depth_m = float(np.median(good)) * scale
        if 0.17 <= depth_m <= 20.0:
            valid_corners += 1
    polygon = np.round(uv[[0, cols - 1, cols * rows - 1, (rows - 1) * cols]]).astype(np.int32)
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, [polygon], 1)
    ys, xs = np.where(mask > 0)
    considered = 0
    valid_dense = 0
    dense = []
    for u, v in zip(xs[::2], ys[::2]):
        considered += 1
        raw = int(depth_image[v, u])
        if raw <= 0:
            continue
        depth_m = raw * scale
        if not (0.17 <= depth_m <= 20.0):
            continue
        valid_dense += 1
        if len(dense) < 4000:
            dense.append((float(u), float(v), depth_m))
    plane_rmse = None
    plane_distance = None
    normal_angle = None
    if len(dense) >= 3:
        dense_uv = np.array([[item[0], item[1]] for item in dense], dtype=np.float64)
        dense_z = np.array([item[2] for item in dense], dtype=np.float64)
        measured = metric.backproject(dense_uv, dense_z, depth_matrix, depth_dist)
        center, normal = metric.fit_plane(measured)
        signed = (measured - center) @ normal
        plane_rmse = float(np.sqrt(np.mean(signed ** 2)))
        rgb_normal = depth_rotation @ rotation[:, 2]
        rgb_point = depth_rotation @ translation + depth_translation
        if float(np.dot(normal, rgb_normal)) < 0.0:
            normal = -normal
        plane_distance = abs(float(np.dot(normal, rgb_point - center)))
        normal_angle = metric.angle_deg(normal, rgb_normal)
    return {
        "valid_depth_corner_count": valid_corners,
        "valid_depth_corner_ratio": valid_corners / 54.0,
        "board_depth_valid_ratio": None if considered == 0 else valid_dense / considered,
        "plane_samples": len(dense),
        "plane_rmse_m": plane_rmse,
        "rgb_depth_distance_m": plane_distance,
        "normal_angle_deg": normal_angle,
    }


def evaluate_frame(record, thresholds):
    minimum = int(thresholds["depth_plane"]["valid_corner_depths_min"])
    row = {
        "semantic_position": record["semantic_position"],
        "stem": record["stem"],
        "timestamp": record["color_stamp"],
        "color_frame_id": record["color_frame_id"],
        "depth_frame_id": record["depth_frame_id"],
        "checkerboard_detected": False,
        "internal_corner_count": 0,
        "full_board_detection": False,
        "valid_depth_corner_count": 0,
        "valid_depth_corner_ratio": 0.0,
        "board_depth_valid_ratio": None,
        "board_pose_camera": None,
        "tf_valid": bool(record.get("T_depth_color")) and not is_identity(record.get("T_depth_color")),
        "rejection_reason": "",
        "workplane_xy": "UNRESOLVED_PENDING_WORKPLANE_REGISTRATION",
    }
    if is_identity(record.get("T_depth_color")):
        row["rejection_reason"] = "identity_rejected"
        row["tf_valid"] = False
    elif not record.get("T_depth_color"):
        row["rejection_reason"] = "lookup_failed"
    color = metric.read_png(CAPTURES / f"{record['stem']}-color.png")
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY) if color.ndim == 3 else color
    found = metric.detect(gray, thresholds)
    if found is None:
        row["rejection_reason"] = "checkerboard_not_found"
        return row
    cols, rows, corners = found
    count = int(corners.shape[0])
    row["checkerboard_detected"] = True
    row["internal_corner_count"] = count
    row["full_board_detection"] = count == 54
    if count != 54:
        row["rejection_reason"] = "corner_count"
        return row
    obj = metric.object_points(cols, rows)
    ok, rvec, tvec = cv2.solvePnP(
        obj,
        corners,
        metric.camera_matrix(record["color_info"]),
        metric.dist_coeffs(record["color_info"]),
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok or float(tvec[2]) <= 0.0:
        row["rejection_reason"] = "pose_rejected"
        return row
    rotation, _ = cv2.Rodrigues(rvec)
    quaternion = metric.rotation_to_quaternion(rotation)
    translation = tvec.reshape(3)
    row["board_pose_camera"] = {
        "frame_id": record["color_frame_id"],
        "convention": "p_color = T_color_checkerboard * p_checkerboard",
        "not_workplane_frame": True,
        "not_base_link": True,
        "translation_m": [float(value) for value in translation],
        "quaternion_xyzw": [float(value) for value in quaternion],
    }
    depth = depth_metrics(record, corners, cols, rows, rotation, translation, thresholds)
    row.update(depth)
    if record.get("tf_reason") == "identity_rejected" or is_identity(record.get("T_depth_color")):
        row["rejection_reason"] = "identity_rejected"
        row["tf_valid"] = False
    elif not row["tf_valid"]:
        row["rejection_reason"] = "lookup_failed"
    elif row["valid_depth_corner_count"] < minimum:
        row["rejection_reason"] = "depth_corners"
    else:
        row["rejection_reason"] = ""
    return row


def observed(rows, thresholds):
    minimum = int(thresholds["depth_plane"]["valid_corner_depths_min"])
    return any(
        row["full_board_detection"] and row["board_pose_camera"] and row["valid_depth_corner_count"] >= minimum
        for row in rows
    )


def main():
    thresholds = load_thresholds()
    records = []
    for path in sorted(CAPTURES.glob("*.json")):
        records.append(json.loads(path.read_text()))
    rows = [evaluate_frame(record, thresholds) for record in records]
    by_position = {position: [row for row in rows if row["semantic_position"] == position] for position in POSITIONS}
    summary = {
        "capture_count": len(rows),
        "positions": {
            position: {
                "frames": len(items),
                "full_board": sum(1 for row in items if row["full_board_detection"]),
                "observed": observed(items, thresholds),
            }
            for position, items in by_position.items()
        },
        "camera_visible_roi": {
            "metric_boundary": "METRIC_BOUNDARY_UNRESOLVED",
            "reason": "UNRESOLVED_PENDING_WORKPLANE_REGISTRATION",
        },
        "robot_reachable_roi": "UNVERIFIED_P007_NOT_OPEN",
        "process_roi": "NOT_CONFIRMED",
        "optical_z_40mm_rule_used": False,
        "pixel_to_workplane": False,
        "threshold_change": 0,
    }
    (ROOT / "metrics.json").write_text(json.dumps({"frames": rows, "summary": summary}, indent=2) + "\n")
    with (ROOT / "results.csv").open("w", newline="") as handle:
        fields = [
            "semantic_position",
            "stem",
            "checkerboard_detected",
            "internal_corner_count",
            "full_board_detection",
            "valid_depth_corner_count",
            "valid_depth_corner_ratio",
            "board_depth_valid_ratio",
            "tf_valid",
            "rejection_reason",
            "workplane_xy",
        ]
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
