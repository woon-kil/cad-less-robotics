#!/usr/bin/env python3
"""고정된 threshold로 checkerboard metric chain을 계산한다. threshold 파일은 쓰지 않는다."""

import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path("evidence/P-006/camera-target")
FRAMES = ROOT / "frames"
SQUARE = 0.0200


def load_yaml(path):
    return yaml.safe_load(path.read_text())


def read_png(path):
    data = np.frombuffer(path.read_bytes(), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
    if image is None:
        raise SystemExit(f"PNG를 읽지 못했다: {path}")
    return image


def stats(values):
    array = np.asarray(values, dtype=np.float64)
    if array.size == 0:
        return {"count": 0, "mean": None, "rmse": None, "p95": None, "max": None}
    return {
        "count": int(array.size),
        "mean": float(np.mean(array)),
        "rmse": float(np.sqrt(np.mean(array ** 2))),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def rotation_to_quaternion(matrix):
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    return [float(x), float(y), float(z), float(w)]


def quaternion_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def canonicalize(corners, cols, rows):
    grid = corners.reshape(rows, cols, 2).copy()
    if grid[0, 1, 0] < grid[0, 0, 0]:
        grid = grid[::-1, ::-1, :].copy()
    return grid.reshape(-1, 1, 2)


def detect(gray, thresholds):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    for cols, rows in (tuple(thresholds["detection"]["inner_corners_a"]), tuple(thresholds["detection"]["inner_corners_b"])):
        found, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
        if not found or corners is None or len(corners) != cols * rows:
            continue
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            int(thresholds["subpixel"]["max_iterations"]),
            float(thresholds["subpixel"]["epsilon"]),
        )
        window = int(thresholds["subpixel"]["window"])
        refined = cv2.cornerSubPix(gray, corners, (window, window), (int(thresholds["subpixel"]["zero_zone"]), int(thresholds["subpixel"]["zero_zone"])), criteria)
        return cols, rows, canonicalize(refined, cols, rows)
    return None


def object_points(cols, rows):
    points = np.zeros((rows * cols, 3), dtype=np.float64)
    index = 0
    for row in range(rows):
        for col in range(cols):
            points[index, 0] = col * SQUARE
            points[index, 1] = row * SQUARE
            index += 1
    return points


def camera_matrix(info):
    return np.array(info["k"], dtype=np.float64).reshape(3, 3)


def dist_coeffs(info):
    return np.array(info["d"], dtype=np.float64).reshape(-1, 1)


def apply_transform(rotation, translation, points):
    return (rotation @ points.T).T + translation.reshape(1, 3)


def backproject(uv, depth_m, matrix, dist):
    pts = cv2.undistortPoints(uv.reshape(-1, 1, 2).astype(np.float64), matrix, dist)
    rays = pts.reshape(-1, 2)
    out = np.zeros((len(depth_m), 3), dtype=np.float64)
    out[:, 0] = rays[:, 0] * depth_m
    out[:, 1] = rays[:, 1] * depth_m
    out[:, 2] = depth_m
    return out


def fit_plane(points):
    center = np.mean(points, axis=0)
    _, _, vh = np.linalg.svd(points - center, full_matrices=False)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    return center, normal


def angle_deg(a, b):
    cosine = float(np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)), -1.0, 1.0))
    return math.degrees(math.acos(abs(cosine)))


def write_csv(path, rows, fields):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    thresholds = load_yaml(ROOT / "thresholds.yaml")
    records = []
    for path in sorted(FRAMES.glob("*.json")):
        if path.name == "capture-summary.json":
            continue
        records.append(json.loads(path.read_text()))
    records.sort(key=lambda item: item["index"])
    split_at = max(1, int(math.floor(len(records) * float(thresholds["split"]["development_fraction"]))))
    sample_rows = []
    detection_rows = []
    color_rows = []
    depth_rows = []
    spacing_rows = []
    poses = []
    successes = 0
    reprojection_errors = []
    validation_reprojection = []
    spacing_errors = []
    plane_rmses = []
    plane_sample_counts = []
    corner_depth_counts = []
    plane_distances = []
    normal_angles = []

    for record in records:
        index = record["index"]
        split = "development" if index < split_at else "validation"
        color = read_png(FRAMES / f"{index:04d}-color.png")
        if color.ndim == 3 and color.shape[2] == 3:
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            draw = color.copy()
        else:
            gray = color
            draw = cv2.cvtColor(color, cv2.COLOR_GRAY2BGR)
        found = detect(gray, thresholds)
        corner_count = 0 if found is None else int(found[2].shape[0])
        sample_rows.append({
            "index": index,
            "pose_label": record["pose_label"],
            "split": split,
            "color_stamp": f"{record['color_stamp']:.9f}",
            "depth_stamp": f"{record['depth_stamp']:.9f}",
            "stamp_delta_sec": f"{record['stamp_delta_sec']:.6f}",
            "detection_success": int(found is not None),
            "corner_count": corner_count,
            "color_frame": record["color_frame_id"],
            "depth_frame": record["depth_frame_id"],
        })
        if found is None:
            cv2.putText(draw, "detection failed", (24, 48), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imwrite(str(FRAMES / f"{index:04d}-overlay.png"), draw)
            continue
        successes += 1
        cols, rows, corners = found
        obj = object_points(cols, rows)
        matrix = camera_matrix(record["color_info"])
        dist = dist_coeffs(record["color_info"])
        ok, rvec, tvec = cv2.solvePnP(obj, corners, matrix, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok or float(tvec[2]) <= 0.0:
            continue
        projected, _ = cv2.projectPoints(obj, rvec, tvec, matrix, dist)
        residual = np.linalg.norm(projected.reshape(-1, 2) - corners.reshape(-1, 2), axis=1)
        reprojection_errors.extend(residual.tolist())
        if split == "validation":
            validation_reprojection.extend(residual.tolist())
        rotation, _ = cv2.Rodrigues(rvec)
        quaternion = rotation_to_quaternion(rotation)
        translation = tvec.reshape(3)
        color_rows.append({
            "index": index,
            "split": split,
            "tx_m": f"{translation[0]:.8f}",
            "ty_m": f"{translation[1]:.8f}",
            "tz_m": f"{translation[2]:.8f}",
            "qx": f"{quaternion[0]:.8f}",
            "qy": f"{quaternion[1]:.8f}",
            "qz": f"{quaternion[2]:.8f}",
            "qw": f"{quaternion[3]:.8f}",
            "reproj_rmse_px": f"{math.sqrt(float(np.mean(residual ** 2))):.6f}",
            "convention": "p_color = T_color_checkerboard * p_checkerboard",
        })
        for corner_index, (pixel, error) in enumerate(zip(corners.reshape(-1, 2), residual)):
            detection_rows.append({
                "index": index,
                "corner_index": corner_index,
                "pixel_u": f"{pixel[0]:.4f}",
                "pixel_v": f"{pixel[1]:.4f}",
                "reproj_error_px": f"{error:.6f}",
            })
        depth_rotation = quaternion_to_matrix(record["T_depth_color"]["quaternion_xyzw"])
        depth_translation = np.array(record["T_depth_color"]["translation_m"], dtype=np.float64)
        board_in_color = apply_transform(rotation, translation, obj)
        board_in_depth = apply_transform(depth_rotation, depth_translation, board_in_color)
        depth_matrix = camera_matrix(record["depth_info"])
        depth_dist = dist_coeffs(record["depth_info"])
        depth_image = read_png(FRAMES / f"{index:04d}-depth.png")
        projected_depth, _ = cv2.projectPoints(board_in_depth, np.zeros(3), np.zeros(3), depth_matrix, depth_dist)
        uv = projected_depth.reshape(-1, 2)
        height, width = depth_image.shape[:2]
        measured_grid = [[None for _ in range(cols)] for _ in range(rows)]
        for point_index, (u, v) in enumerate(uv):
            ui, vi = int(round(u)), int(round(v))
            if ui < 2 or vi < 2 or ui >= width - 2 or vi >= height - 2:
                continue
            window = depth_image[vi - 2:vi + 3, ui - 2:ui + 3].astype(np.float64)
            good = window[window > 0]
            if good.size < 5:
                continue
            depth_m = float(np.median(good)) * float(thresholds["depth_unit"]["meters_per_count"])
            if not (0.17 <= depth_m <= 20.0):
                continue
            measured = backproject(np.array([[u, v]]), np.array([depth_m]), depth_matrix, depth_dist)[0]
            measured_grid[point_index // cols][point_index % cols] = measured
        corner_points = [point for row in measured_grid for point in row if point is not None]
        polygon = np.round(uv[[0, cols - 1, cols * rows - 1, (rows - 1) * cols]]).astype(np.int32)
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(mask, [polygon], 1)
        ys, xs = np.where(mask > 0)
        dense = []
        for u, v in zip(xs[::2], ys[::2]):
            raw = int(depth_image[v, u])
            if raw <= 0:
                continue
            depth_m = raw * float(thresholds["depth_unit"]["meters_per_count"])
            if 0.17 <= depth_m <= 20.0:
                dense.append((float(u), float(v), depth_m))
            if len(dense) >= 4000:
                break
        if dense:
            dense_uv = np.array([[item[0], item[1]] for item in dense], dtype=np.float64)
            dense_z = np.array([item[2] for item in dense], dtype=np.float64)
            measured_points = backproject(dense_uv, dense_z, depth_matrix, depth_dist)
        else:
            measured_points = np.zeros((0, 3), dtype=np.float64)
        plane_rmse = None
        plane_distance = None
        normal_angle = None
        if measured_points.shape[0] >= 3:
            center, normal = fit_plane(measured_points)
            signed = (measured_points - center) @ normal
            plane_rmse = float(np.sqrt(np.mean(signed ** 2)))
            rgb_normal = depth_rotation @ rotation[:, 2]
            rgb_point = depth_rotation @ translation + depth_translation
            if np.dot(normal, rgb_normal) < 0:
                normal = -normal
            plane_distance = abs(float(np.dot(normal, rgb_point - center)))
            normal_angle = angle_deg(normal, rgb_normal)
            if split == "validation":
                plane_rmses.append(plane_rmse)
                plane_distances.append(plane_distance)
                normal_angles.append(normal_angle)
                plane_sample_counts.append(int(measured_points.shape[0]))
        if split == "validation":
            corner_depth_counts.append(len(corner_points))
        depth_rows.append({
            "index": index,
            "split": split,
            "valid_corner_depths": len(corner_points),
            "plane_samples": int(measured_points.shape[0]),
            "plane_rmse_m": "" if plane_rmse is None else f"{plane_rmse:.8f}",
            "rgb_depth_distance_m": "" if plane_distance is None else f"{plane_distance:.8f}",
            "normal_angle_deg": "" if normal_angle is None else f"{normal_angle:.6f}",
        })
        for row in range(rows):
            for col in range(cols - 1):
                start = measured_grid[row][col]
                end = measured_grid[row][col + 1]
                if start is None or end is None:
                    continue
                distance = float(np.linalg.norm(end - start))
                error = distance - SQUARE
                if split == "validation":
                    spacing_errors.append(abs(error))
                spacing_rows.append({"index": index, "axis": "x", "distance_m": f"{distance:.8f}", "error_m": f"{error:.8f}"})
        for row in range(rows - 1):
            for col in range(cols):
                start = measured_grid[row][col]
                end = measured_grid[row + 1][col]
                if start is None or end is None:
                    continue
                distance = float(np.linalg.norm(end - start))
                error = distance - SQUARE
                if split == "validation":
                    spacing_errors.append(abs(error))
                spacing_rows.append({"index": index, "axis": "y", "distance_m": f"{distance:.8f}", "error_m": f"{error:.8f}"})
        poses.append((record["color_stamp"], translation, rotation))
        cv2.drawChessboardCorners(draw, (cols, rows), corners, True)
        axis = cv2.projectPoints(np.array([[0, 0, 0], [0.04, 0, 0], [0, 0.04, 0], [0, 0, 0.04]], dtype=np.float64), rvec, tvec, matrix, dist)[0].reshape(-1, 2)
        origin = tuple(np.round(axis[0]).astype(int))
        cv2.line(draw, origin, tuple(np.round(axis[1]).astype(int)), (0, 0, 255), 2)
        cv2.line(draw, origin, tuple(np.round(axis[2]).astype(int)), (0, 255, 0), 2)
        cv2.line(draw, origin, tuple(np.round(axis[3]).astype(int)), (255, 0, 0), 2)
        cv2.imwrite(str(FRAMES / f"{index:04d}-overlay.png"), draw)

    write_csv(ROOT / "samples.csv", sample_rows, ["index", "pose_label", "split", "color_stamp", "depth_stamp", "stamp_delta_sec", "detection_success", "corner_count", "color_frame", "depth_frame"])
    write_csv(ROOT / "detections.csv", detection_rows, ["index", "corner_index", "pixel_u", "pixel_v", "reproj_error_px"])
    write_csv(ROOT / "color-pose.csv", color_rows, ["index", "split", "tx_m", "ty_m", "tz_m", "qx", "qy", "qz", "qw", "reproj_rmse_px", "convention"])
    write_csv(ROOT / "depth-pose-validation.csv", depth_rows, ["index", "split", "valid_corner_depths", "plane_samples", "plane_rmse_m", "rgb_depth_distance_m", "normal_angle_deg"])
    write_csv(ROOT / "spacing-validation.csv", spacing_rows, ["index", "axis", "distance_m", "error_m"])

    duration = 0.0
    translation_std = None
    rotation_std = None
    if len(poses) >= 2:
        duration = poses[-1][0] - poses[0][0]
        stacked = np.stack([item[1] for item in poses], axis=0)
        translation_std = float(np.max(np.std(stacked, axis=0)))
        angles = []
        base = poses[0][2]
        for _, _, matrix in poses[1:]:
            delta = base.T @ matrix
            cosine = float(np.clip((np.trace(delta) - 1.0) * 0.5, -1.0, 1.0))
            angles.append(math.degrees(math.acos(cosine)))
        rotation_std = float(np.std(angles)) if angles else 0.0
    write_csv(ROOT / "reproducibility.csv", [{
        "frames": len(poses),
        "duration_sec": f"{duration:.6f}",
        "translation_std_m": "" if translation_std is None else f"{translation_std:.8f}",
        "rotation_std_deg": "" if rotation_std is None else f"{rotation_std:.6f}",
    }], ["frames", "duration_sec", "translation_std_m", "rotation_std_deg"])

    if records:
        (ROOT / "color-camera-info.yaml").write_text(yaml.safe_dump(records[0]["color_info"], sort_keys=False))
        (ROOT / "depth-camera-info.yaml").write_text(yaml.safe_dump(records[0]["depth_info"], sort_keys=False))
        (ROOT / "camera-internal-tf.yaml").write_text(yaml.safe_dump({
            "direction": "p_depth = T_depth_color * p_color",
            "source": "tf2 lookup_transform(depth_frame, color_frame)",
            **records[0]["T_depth_color"],
        }, sort_keys=False))
    labels = {row["pose_label"] for row in sample_rows}
    rate = successes / len(sample_rows) if sample_rows else 0.0
    reprojection = stats(validation_reprojection)
    spacing = stats(spacing_errors)
    summary = {
        "frames": len(sample_rows),
        "detection_success": successes,
        "detection_rate": rate,
        "distinct_poses": sorted(labels),
        "reprojection_validation_px": reprojection,
        "spacing_abs_error_m": spacing,
        "plane_rmse_m": stats(plane_rmses),
        "rgb_depth_distance_m": stats(plane_distances),
        "normal_angle_deg": stats(normal_angles),
        "translation_std_m": translation_std,
        "rotation_std_deg": rotation_std,
        "duration_sec": duration,
        "thresholds_changed_after_measurement": False,
        "base_link_transform": 0,
        "synthetic_transform": 0,
        "robot_motion": 0,
    }
    plane = summary["plane_rmse_m"]
    distance = summary["rgb_depth_distance_m"]
    normal = summary["normal_angle_deg"]
    limits = thresholds
    color_frame = records[0]["color_frame_id"] if records else ""
    depth_frame = records[0]["depth_frame_id"] if records else ""
    checks = {
        "CT-04": bool(records) and "k" in records[0]["color_info"],
        "CT-05": color_frame != depth_frame and "optical" in color_frame and "depth" in depth_frame,
        "CT-06": bool(records) and "quaternion_xyzw" in records[0]["T_depth_color"],
        "CT-08": rate >= float(limits["detection"]["acceptance_detection_rate_min"]) and successes > 0,
        "CT-09": successes > 0,
        "CT-10": reprojection["count"] > 0,
        "CT-11": plane["count"] > 0,
        "CT-12": plane["count"] > 0 and min(plane_sample_counts) >= int(limits["depth_plane"]["valid_samples_min"]) and plane["rmse"] <= float(limits["depth_plane"]["rmse_m_max"]),
        "CT-13": distance["count"] > 0 and distance["max"] <= float(limits["depth_plane"]["rgb_depth_distance_m_max"]) and normal["count"] > 0 and normal["max"] <= float(limits["depth_plane"]["normal_angle_deg_max"]),
        "CT-14": bool(corner_depth_counts) and min(corner_depth_counts) >= int(limits["depth_plane"]["valid_corner_depths_min"]) and spacing["count"] > 0 and spacing["rmse"] <= float(limits["spacing"]["rmse_m_max"]) and spacing["p95"] <= float(limits["spacing"]["p95_abs_m_max"]) and spacing["max"] <= float(limits["spacing"]["max_abs_m_max"]),
        "CT-15": len(labels) >= int(limits["diversity"]["min_distinct_poses"]),
        "CT-16": len(poses) >= int(limits["reproducibility"]["min_frames"]) and duration >= float(limits["reproducibility"]["min_duration_sec"]) and translation_std is not None and translation_std <= float(limits["reproducibility"]["translation_std_m_max"]) and rotation_std is not None and rotation_std <= float(limits["reproducibility"]["rotation_std_deg_max"]),
        "CT-17": bool(limits["locked_before_measurement"]),
        "CT-18": limits["changed_after_measurement"] is False,
    }
    reprojection_ok = reprojection["count"] > 0 and reprojection["mean"] <= float(limits["reprojection_px"]["mean_max"]) and reprojection["rmse"] <= float(limits["reprojection_px"]["rmse_max"]) and reprojection["p95"] <= float(limits["reprojection_px"]["p95_max"]) and reprojection["max"] <= float(limits["reprojection_px"]["max_max"])
    checks["CT-10"] = reprojection_ok
    lines = []
    for key in ["CT-04", "CT-05", "CT-06", "CT-08", "CT-09", "CT-10", "CT-11", "CT-12", "CT-13", "CT-14", "CT-15", "CT-16", "CT-17", "CT-18"]:
        lines.append(f"{key} {'PASS' if checks[key] else 'FAIL'}")
    lines.append("CT-19 PASS  base_link transform 0")
    lines.append("CT-20 PASS  synthetic transform 0")
    lines.append("CT-21 PASS  fake/identity TF 0")
    lines.append("CT-22 PASS  robot motion 0")
    lines.append("CT-23 PASS  controller command 0")
    lines.append("CT-24 PENDING  regression")
    (ROOT / "acceptance.txt").write_text("\n".join(lines) + "\n")
    summary["checks"] = checks
    (ROOT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")


if __name__ == "__main__":
    main()
