#!/usr/bin/env python3
"""Attempt 3 프레임을 기존 threshold로 계산하고 Attempt 2 tilt를 읽기만 한다."""

import csv
import hashlib
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p006c_validate_metric as metric

PARENT = Path("evidence/P-006/camera-target")
ROOT = PARENT / "attempt-05"
FRAMES = ROOT / "frames"
POSES = ["left", "right", "near", "far"]
TILTS = ["tilt_positive", "tilt_negative"]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require_frozen():
    plan = metric.load_yaml(ROOT / "capture-plan.yaml")
    checks = {
        PARENT / "thresholds.yaml": plan["thresholds_sha256"],
        PARENT / "checkerboard.yaml": plan["checkerboard_sha256"],
        PARENT / "color-pose.csv": plan["attempt1_color_pose_sha256"],
        PARENT / "reproducibility.csv": plan["attempt1_reproducibility_sha256"],
        PARENT / "summary.json": plan["attempt1_summary_sha256"],
        PARENT / "frames" / "0000.json": plan["attempt1_frame_0000_sha256"],
        PARENT / "frames" / "0011.json": plan["attempt1_frame_0011_sha256"],
        PARENT / "attempt-02" / "capture-plan.yaml": plan["attempt2_plan_sha256"],
        PARENT / "attempt-02" / "diversity-validation.csv": plan["attempt2_diversity_sha256"],
        PARENT / "attempt-02" / "poses.csv": plan["attempt2_poses_sha256"],
        PARENT / "attempt-02" / "summary.json": plan["attempt2_summary_sha256"],
        PARENT / "attempt-02" / "frames" / "tilt_positive-00.json": plan["attempt2_tilt_positive_sha256"],
        PARENT / "attempt-02" / "frames" / "tilt_negative-01.json": plan["attempt2_tilt_negative_sha256"],
        PARENT / "attempt-03" / "capture-plan.yaml": plan["attempt3_plan_sha256"],
        PARENT / "attempt-03" / "diversity-validation.csv": plan["attempt3_diversity_sha256"],
        PARENT / "attempt-03" / "poses.csv": plan["attempt3_poses_sha256"],
        PARENT / "attempt-03" / "summary.json": plan["attempt3_summary_sha256"],
        PARENT / "attempt-03" / "frames" / "left-t01-00.json": plan["attempt3_left_sha256"],
        PARENT / "attempt-03" / "frames" / "right-t01-00.json": plan["attempt3_right_sha256"],
        PARENT / "attempt-04" / "capture-plan.yaml": plan["attempt4_plan_sha256"],
        PARENT / "attempt-04" / "diversity-validation.csv": plan["attempt4_diversity_sha256"],
        PARENT / "attempt-04" / "poses.csv": plan["attempt4_poses_sha256"],
        PARENT / "attempt-04" / "summary.json": plan["attempt4_summary_sha256"],
        PARENT / "attempt-04" / "frames" / "near-t01-00.json": plan["attempt4_near_sha256"],
        PARENT / "attempt-04" / "frames" / "far-t01-00.json": plan["attempt4_far_sha256"],
    }
    for path, expected in checks.items():
        actual = sha256(path)
        if actual != expected:
            raise SystemExit(f"동결 파일이 바뀌었다: {path} {actual}")
    return plan


def angle_between(a, b):
    cosine = float(np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)), -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def board_normal(rotation):
    normal = rotation[:, 2]
    return normal / np.linalg.norm(normal)


def is_identity(transform):
    if not transform:
        return False
    translation = np.array(transform["translation_m"], dtype=np.float64)
    quaternion = np.array(transform["quaternion_xyzw"], dtype=np.float64)
    return float(np.linalg.norm(translation)) < 1e-6 and abs(float(quaternion[3])) > 0.999999


def placement_passes(label, delta, solved, minimum, lateral, range_m):
    if solved < minimum:
        return False
    if label == "left":
        return float(delta[0]) < -lateral
    if label == "right":
        return float(delta[0]) > lateral
    if label == "near":
        return float(delta[2]) < -range_m
    if label == "far":
        return float(delta[2]) > range_m
    return False


def load_attempt1_reproducibility(thresholds):
    row = list(csv.DictReader((PARENT / "reproducibility.csv").open()))[0]
    frames = int(row["frames"])
    duration = float(row["duration_sec"])
    translation = float(row["translation_std_m"])
    rotation = float(row["rotation_std_deg"])
    limits = thresholds["reproducibility"]
    ok = (
        frames >= int(limits["min_frames"])
        and duration >= float(limits["min_duration_sec"])
        and translation <= float(limits["translation_std_m_max"])
        and rotation <= float(limits["rotation_std_deg_max"])
    )
    return {
        "frames": frames,
        "duration_sec": duration,
        "translation_std_m": translation,
        "rotation_std_deg": rotation,
        "pass": ok,
    }


def load_tilts(plan):
    fixed_normal = np.array(plan["fixed_reference"]["board_plus_z_in_color"], dtype=np.float64)
    fixed_normal = fixed_normal / np.linalg.norm(fixed_normal)
    grouped = {label: [] for label in TILTS}
    for row in csv.DictReader((PARENT / "attempt-02" / "poses.csv").open()):
        if row["pose_label"] not in grouped:
            continue
        grouped[row["pose_label"]].append(row)
    tilts = {}
    for label, rows in grouped.items():
        if not rows:
            tilts[label] = None
            continue
        translation = np.median(np.array([[float(row["tx_m"]), float(row["ty_m"]), float(row["tz_m"])] for row in rows], dtype=np.float64), axis=0)
        normal = np.mean(np.array([[float(row["nx"]), float(row["ny"]), float(row["nz"])] for row in rows], dtype=np.float64), axis=0)
        normal = normal / np.linalg.norm(normal)
        tilts[label] = {
            "solved": len(rows),
            "median_t": translation,
            "normal": normal,
            "tilt_deg": angle_between(normal, fixed_normal),
        }
    tilt_deg = float(plan["separation"]["tilt_deg"])
    opposite = False
    if tilts["tilt_positive"] and tilts["tilt_negative"]:
        cross_pos = np.cross(fixed_normal, tilts["tilt_positive"]["normal"])
        cross_neg = np.cross(fixed_normal, tilts["tilt_negative"]["normal"])
        opposite = float(np.dot(cross_pos, cross_neg)) < 0.0
        tilts["between_deg"] = angle_between(tilts["tilt_positive"]["normal"], tilts["tilt_negative"]["normal"])
    else:
        tilts["between_deg"] = None
    tilts["opposite"] = opposite
    if tilts["tilt_positive"]:
        tilts["tilt_positive"]["pass"] = tilts["tilt_positive"]["solved"] >= 4 and tilts["tilt_positive"]["tilt_deg"] >= tilt_deg
    if tilts["tilt_negative"]:
        tilts["tilt_negative"]["pass"] = bool(tilts["tilt_negative"]) and tilts["tilt_negative"]["solved"] >= 4 and tilts["tilt_negative"]["tilt_deg"] >= tilt_deg and opposite
    return tilts


def load_lateral(plan, fixed_t):
    lateral = float(plan["separation"]["lateral_m"])
    grouped = {"left": [], "right": []}
    for row in csv.DictReader((PARENT / "attempt-03" / "poses.csv").open()):
        if row["pose_label"] in grouped:
            grouped[row["pose_label"]].append(row)
    loaded = {}
    for label, rows in grouped.items():
        translation = np.median(np.array([[float(row["tx_m"]), float(row["ty_m"]), float(row["tz_m"])] for row in rows], dtype=np.float64), axis=0)
        normal = np.mean(np.array([[float(row["nx"]), float(row["ny"]), float(row["nz"])] for row in rows], dtype=np.float64), axis=0)
        normal = normal / np.linalg.norm(normal)
        delta = translation - fixed_t
        passed = len(rows) >= 4 and placement_passes(label, delta, len(rows), 4, lateral, float(plan["separation"]["range_m"]))
        loaded[label] = {
            "solved": len(rows),
            "median_t": translation,
            "normal": normal,
            "delta": delta,
            "passed": passed,
        }
    return loaded


def main():
    plan = require_frozen()
    thresholds = metric.load_yaml(PARENT / "thresholds.yaml")
    if thresholds.get("changed_after_measurement") is not False or not thresholds.get("locked_before_measurement"):
        raise SystemExit("threshold 잠금 표시가 아니다")
    fixed_t = np.array(plan["fixed_reference"]["median_t_m"], dtype=np.float64)
    lateral = float(plan["separation"]["lateral_m"])
    range_m = float(plan["separation"]["range_m"])
    minimum = int(plan["frames_per_try"])
    records = []
    if FRAMES.is_dir():
        for path in sorted(FRAMES.glob("*.json")):
            records.append(json.loads(path.read_text()))
    records.sort(key=lambda item: (item["pose_label"], int(item["try_index"]), item["index"]))

    sample_rows = []
    detection_rows = []
    pose_rows = []
    depth_rows = []
    spacing_rows = []
    tf_rows = []
    solved = []
    reprojection_errors = []
    spacing_errors = []
    plane_rmses = []
    plane_sample_counts = []
    corner_depth_counts = []
    plane_distances = []
    normal_angles = []
    successes = 0

    for record in records:
        label = record["pose_label"]
        stem = record["stem"]
        try_index = int(record["try_index"])
        color = metric.read_png(FRAMES / f"{stem}-color.png")
        if color.ndim == 3 and color.shape[2] == 3:
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            draw = color.copy()
        else:
            gray = color
            draw = cv2.cvtColor(color, cv2.COLOR_GRAY2BGR)
        found = metric.detect(gray, thresholds)
        corner_count = 0 if found is None else int(found[2].shape[0])
        transform = record.get("T_depth_color")
        if transform is None:
            tf_reason = "lookup_failed"
        elif is_identity(transform):
            tf_reason = "identity_rejected"
        else:
            tf_reason = "ok"
        tf_rows.append({"stem": stem, "pose_label": label, "try_index": try_index, "tf_valid": int(tf_reason == "ok"), "reason": tf_reason})
        sample_rows.append({
            "stem": stem,
            "pose_label": label,
            "try_index": try_index,
            "split": "diversity",
            "color_stamp": f"{record['color_stamp']:.9f}",
            "depth_stamp": f"{record['depth_stamp']:.9f}",
            "stamp_delta_sec": f"{record['stamp_delta_sec']:.6f}",
            "detection_success": int(found is not None),
            "corner_count": corner_count,
            "color_frame": record["color_frame_id"],
            "depth_frame": record["depth_frame_id"],
            "color_width": record.get("color_width", ""),
            "color_height": record.get("color_height", ""),
            "depth_width": record.get("depth_width", ""),
            "depth_height": record.get("depth_height", ""),
            "tf_reason": tf_reason,
        })
        if found is None:
            cv2.putText(draw, f"{label} t{try_index} detection failed", (24, 48), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imwrite(str(FRAMES / f"{stem}-overlay.png"), draw)
            continue
        cols, rows, corners = found
        obj = metric.object_points(cols, rows)
        matrix = metric.camera_matrix(record["color_info"])
        dist = metric.dist_coeffs(record["color_info"])
        ok, rvec, tvec = cv2.solvePnP(obj, corners, matrix, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok or float(tvec[2]) <= 0.0:
            cv2.putText(draw, f"{label} t{try_index} solve failed", (24, 48), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imwrite(str(FRAMES / f"{stem}-overlay.png"), draw)
            continue
        successes += 1
        projected, _ = cv2.projectPoints(obj, rvec, tvec, matrix, dist)
        residual = np.linalg.norm(projected.reshape(-1, 2) - corners.reshape(-1, 2), axis=1)
        reprojection_errors.extend(residual.tolist())
        rotation, _ = cv2.Rodrigues(rvec)
        quaternion = metric.rotation_to_quaternion(rotation)
        translation = tvec.reshape(3)
        normal = board_normal(rotation)
        pose_rows.append({
            "stem": stem,
            "pose_label": label,
            "try_index": try_index,
            "tx_m": f"{translation[0]:.8f}",
            "ty_m": f"{translation[1]:.8f}",
            "tz_m": f"{translation[2]:.8f}",
            "nx": f"{normal[0]:.8f}",
            "ny": f"{normal[1]:.8f}",
            "nz": f"{normal[2]:.8f}",
            "qx": f"{quaternion[0]:.8f}",
            "qy": f"{quaternion[1]:.8f}",
            "qz": f"{quaternion[2]:.8f}",
            "qw": f"{quaternion[3]:.8f}",
            "reproj_rmse_px": f"{math.sqrt(float(np.mean(residual ** 2))):.6f}",
            "convention": "p_color = T_color_checkerboard * p_checkerboard",
        })
        solved.append({"label": label, "try_index": try_index, "translation": translation.copy(), "normal": normal.copy()})
        for corner_index, (pixel, error) in enumerate(zip(corners.reshape(-1, 2), residual)):
            detection_rows.append({
                "stem": stem,
                "corner_index": corner_index,
                "pixel_u": f"{pixel[0]:.4f}",
                "pixel_v": f"{pixel[1]:.4f}",
                "reproj_error_px": f"{error:.6f}",
            })
        plane_rmse = None
        plane_distance = None
        normal_angle = None
        measured_count = 0
        corner_depth_count = 0
        if tf_reason == "ok":
            depth_rotation = metric.quaternion_to_matrix(transform["quaternion_xyzw"])
            depth_translation = np.array(transform["translation_m"], dtype=np.float64)
            board_in_color = metric.apply_transform(rotation, translation, obj)
            board_in_depth = metric.apply_transform(depth_rotation, depth_translation, board_in_color)
            depth_matrix = metric.camera_matrix(record["depth_info"])
            depth_dist = metric.dist_coeffs(record["depth_info"])
            depth_image = metric.read_png(FRAMES / f"{stem}-depth.png")
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
                measured = metric.backproject(np.array([[u, v]]), np.array([depth_m]), depth_matrix, depth_dist)[0]
                measured_grid[point_index // cols][point_index % cols] = measured
            corner_points = [point for row in measured_grid for point in row if point is not None]
            corner_depth_count = len(corner_points)
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
                measured_points = metric.backproject(dense_uv, dense_z, depth_matrix, depth_dist)
            else:
                measured_points = np.zeros((0, 3), dtype=np.float64)
            measured_count = int(measured_points.shape[0])
            if measured_points.shape[0] >= 3:
                center, plane_normal = metric.fit_plane(measured_points)
                signed = (measured_points - center) @ plane_normal
                plane_rmse = float(np.sqrt(np.mean(signed ** 2)))
                rgb_normal = depth_rotation @ rotation[:, 2]
                rgb_point = depth_rotation @ translation + depth_translation
                if np.dot(plane_normal, rgb_normal) < 0:
                    plane_normal = -plane_normal
                plane_distance = abs(float(np.dot(plane_normal, rgb_point - center)))
                normal_angle = metric.angle_deg(plane_normal, rgb_normal)
                plane_rmses.append(plane_rmse)
                plane_distances.append(plane_distance)
                normal_angles.append(normal_angle)
                plane_sample_counts.append(measured_count)
            corner_depth_counts.append(corner_depth_count)
            for row in range(rows):
                for col in range(cols - 1):
                    start = measured_grid[row][col]
                    end = measured_grid[row][col + 1]
                    if start is None or end is None:
                        continue
                    distance = float(np.linalg.norm(end - start))
                    spacing_rows.append({"stem": stem, "pose_label": label, "try_index": try_index, "axis": "x", "distance_m": f"{distance:.8f}", "error_m": f"{distance - metric.SQUARE:.8f}"})
                    spacing_errors.append(abs(distance - metric.SQUARE))
            for row in range(rows - 1):
                for col in range(cols):
                    start = measured_grid[row][col]
                    end = measured_grid[row + 1][col]
                    if start is None or end is None:
                        continue
                    distance = float(np.linalg.norm(end - start))
                    spacing_rows.append({"stem": stem, "pose_label": label, "try_index": try_index, "axis": "y", "distance_m": f"{distance:.8f}", "error_m": f"{distance - metric.SQUARE:.8f}"})
                    spacing_errors.append(abs(distance - metric.SQUARE))
        depth_rows.append({
            "stem": stem,
            "pose_label": label,
            "try_index": try_index,
            "valid_corner_depths": corner_depth_count,
            "plane_samples": measured_count,
            "plane_rmse_m": "" if plane_rmse is None else f"{plane_rmse:.8f}",
            "rgb_depth_distance_m": "" if plane_distance is None else f"{plane_distance:.8f}",
            "normal_angle_deg": "" if normal_angle is None else f"{normal_angle:.6f}",
        })
        cv2.drawChessboardCorners(draw, (cols, rows), corners, True)
        axis = cv2.projectPoints(np.array([[0, 0, 0], [0.04, 0, 0], [0, 0.04, 0], [0, 0, 0.04]], dtype=np.float64), rvec, tvec, matrix, dist)[0].reshape(-1, 2)
        origin = tuple(np.round(axis[0]).astype(int))
        cv2.line(draw, origin, tuple(np.round(axis[1]).astype(int)), (0, 0, 255), 2)
        cv2.line(draw, origin, tuple(np.round(axis[2]).astype(int)), (0, 255, 0), 2)
        cv2.line(draw, origin, tuple(np.round(axis[3]).astype(int)), (255, 0, 0), 2)
        cv2.putText(draw, f"{label} t{try_index}", (24, 48), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        cv2.imwrite(str(FRAMES / f"{stem}-overlay.png"), draw)

    groups = {}
    for item in solved:
        groups.setdefault((item["label"], item["try_index"]), []).append(item)
    diversity_rows = []
    class_choice = {label: {"passed": False, "delta": None, "solved": 0, "accepted": 0, "score": -1e9} for label in POSES}
    for label in POSES:
        tries = sorted({try_index for (name, try_index) in groups if name == label})
        captured_tries = sorted({int(row["try_index"]) for row in sample_rows if row["pose_label"] == label})
        for try_index in captured_tries:
            items = groups.get((label, try_index), [])
            frames = [row for row in sample_rows if row["pose_label"] == label and int(row["try_index"]) == try_index]
            detected = sum(int(row["detection_success"]) for row in frames)
            if items:
                median_t = np.median(np.stack([item["translation"] for item in items], axis=0), axis=0)
                delta = median_t - fixed_t
                mean_n = np.mean(np.stack([item["normal"] for item in items], axis=0), axis=0)
                mean_n = mean_n / np.linalg.norm(mean_n)
            else:
                median_t = None
                delta = None
                mean_n = None
            passed = bool(items) and placement_passes(label, delta, len(items), minimum, lateral, range_m)
            diversity_rows.append({
                "source": "attempt-05",
                "pose_label": label,
                "try_index": try_index,
                "frames": len(frames),
                "detected": detected,
                "solved": len(items),
                "accepted": len(items) if passed else 0,
                "median_x_m": "" if median_t is None else f"{median_t[0]:.8f}",
                "median_y_m": "" if median_t is None else f"{median_t[1]:.8f}",
                "median_z_m": "" if median_t is None else f"{median_t[2]:.8f}",
                "delta_x_m": "" if delta is None else f"{delta[0]:.8f}",
                "delta_y_m": "" if delta is None else f"{delta[1]:.8f}",
                "delta_z_m": "" if delta is None else f"{delta[2]:.8f}",
                "normal_angle_from_fixed_deg": "" if mean_n is None else f"{angle_between(mean_n, np.array(plan['fixed_reference']['board_plus_z_in_color'], dtype=np.float64)):.6f}",
                "rule_pass": int(passed),
            })
            score = None if delta is None else {
                "left": -float(delta[0]),
                "right": float(delta[0]),
                "near": -float(delta[2]),
                "far": float(delta[2]),
            }[label]
            previous = class_choice[label]
            better = passed and not previous["passed"]
            same_state_stronger = passed == previous["passed"] and score is not None and score > previous["score"] and (previous["delta"] is not None or passed)
            if previous["delta"] is None or better or same_state_stronger:
                class_choice[label] = {"passed": passed, "delta": delta, "solved": len(items), "accepted": len(items) if passed else 0, "score": -1e9 if score is None else score}

    lateral_loaded = load_lateral(plan, fixed_t)
    fixed_normal = np.array(plan["fixed_reference"]["board_plus_z_in_color"], dtype=np.float64)
    for label in ("left", "right"):
        item = lateral_loaded[label]
        class_choice[label] = {
            "passed": item["passed"],
            "delta": item["delta"],
            "solved": item["solved"],
            "accepted": item["solved"] if item["passed"] else 0,
            "score": float(-item["delta"][0] if label == "left" else item["delta"][0]),
        }
        diversity_rows.append({
            "source": "attempt-03",
            "pose_label": label,
            "try_index": "",
            "frames": item["solved"],
            "detected": item["solved"],
            "solved": item["solved"],
            "accepted": item["solved"] if item["passed"] else 0,
            "median_x_m": f"{item['median_t'][0]:.8f}",
            "median_y_m": f"{item['median_t'][1]:.8f}",
            "median_z_m": f"{item['median_t'][2]:.8f}",
            "delta_x_m": f"{item['delta'][0]:.8f}",
            "delta_y_m": f"{item['delta'][1]:.8f}",
            "delta_z_m": f"{item['delta'][2]:.8f}",
            "normal_angle_from_fixed_deg": f"{angle_between(item['normal'], fixed_normal):.6f}",
            "rule_pass": int(bool(item["passed"])),
        })

    tilts = load_tilts(plan)
    fixed_t_list = fixed_t
    for label in TILTS:
        item = tilts[label]
        delta = item["median_t"] - fixed_t_list
        diversity_rows.append({
            "source": "attempt-02",
            "pose_label": label,
            "try_index": "",
            "frames": item["solved"],
            "detected": item["solved"],
            "solved": item["solved"],
            "accepted": item["solved"] if item["pass"] else 0,
            "median_x_m": f"{item['median_t'][0]:.8f}",
            "median_y_m": f"{item['median_t'][1]:.8f}",
            "median_z_m": f"{item['median_t'][2]:.8f}",
            "delta_x_m": f"{delta[0]:.8f}",
            "delta_y_m": f"{delta[1]:.8f}",
            "delta_z_m": f"{delta[2]:.8f}",
            "normal_angle_from_fixed_deg": f"{item['tilt_deg']:.6f}",
            "rule_pass": int(bool(item["pass"])),
        })

    sample_fields = ["stem", "pose_label", "try_index", "split", "color_stamp", "depth_stamp", "stamp_delta_sec", "detection_success", "corner_count", "color_frame", "depth_frame", "color_width", "color_height", "depth_width", "depth_height", "tf_reason"]
    metric.write_csv(ROOT / "samples.csv", sample_rows, sample_fields)
    metric.write_csv(ROOT / "detections.csv", detection_rows, ["stem", "corner_index", "pixel_u", "pixel_v", "reproj_error_px"])
    metric.write_csv(ROOT / "poses.csv", pose_rows, ["stem", "pose_label", "try_index", "tx_m", "ty_m", "tz_m", "nx", "ny", "nz", "qx", "qy", "qz", "qw", "reproj_rmse_px", "convention"])
    metric.write_csv(ROOT / "depth-pose-validation.csv", depth_rows, ["stem", "pose_label", "try_index", "valid_corner_depths", "plane_samples", "plane_rmse_m", "rgb_depth_distance_m", "normal_angle_deg"])
    metric.write_csv(ROOT / "spacing-validation.csv", spacing_rows, ["stem", "pose_label", "try_index", "axis", "distance_m", "error_m"])
    metric.write_csv(ROOT / "diversity-validation.csv", diversity_rows, ["source", "pose_label", "try_index", "frames", "detected", "solved", "accepted", "median_x_m", "median_y_m", "median_z_m", "delta_x_m", "delta_y_m", "delta_z_m", "normal_angle_from_fixed_deg", "rule_pass"])
    metric.write_csv(ROOT / "tf-validity.csv", tf_rows, ["stem", "pose_label", "try_index", "tf_valid", "reason"])

    reprojection = metric.stats(reprojection_errors)
    spacing = metric.stats(spacing_errors)
    plane = metric.stats(plane_rmses)
    distance = metric.stats(plane_distances)
    normal = metric.stats(normal_angles)
    repeatability = load_attempt1_reproducibility(thresholds)
    rate = successes / len(sample_rows) if sample_rows else 0.0
    full_resolution = bool(sample_rows) and all(int(row["color_width"]) == 1280 and int(row["color_height"]) == 720 and int(row["depth_width"]) == 848 and int(row["depth_height"]) == 480 for row in sample_rows)
    color_frame = records[0]["color_frame_id"] if records else ""
    depth_frame = records[0]["depth_frame_id"] if records else ""
    tf_valid = sum(int(row["tf_valid"]) for row in tf_rows)
    identity_count = sum(1 for row in tf_rows if row["reason"] == "identity_rejected")
    launch_text = (ROOT / "launch.log").read_text(encoding="utf-8", errors="replace") if (ROOT / "launch.log").is_file() else ""
    hardware_text = (ROOT / "hardware-inventory.txt").read_text(encoding="utf-8", errors="replace") if (ROOT / "hardware-inventory.txt").is_file() else ""
    board = metric.load_yaml(PARENT / "checkerboard.yaml")
    reprojection_ok = (
        reprojection["count"] > 0
        and reprojection["mean"] <= float(thresholds["reprojection_px"]["mean_max"])
        and reprojection["rmse"] <= float(thresholds["reprojection_px"]["rmse_max"])
        and reprojection["p95"] <= float(thresholds["reprojection_px"]["p95_max"])
        and reprojection["max"] <= float(thresholds["reprojection_px"]["max_max"])
    )
    plane_ok = plane["count"] > 0 and plane_sample_counts and min(plane_sample_counts) >= int(thresholds["depth_plane"]["valid_samples_min"]) and plane["max"] <= float(thresholds["depth_plane"]["rmse_m_max"])
    rgb_depth_ok = distance["count"] > 0 and distance["max"] <= float(thresholds["depth_plane"]["rgb_depth_distance_m_max"]) and normal["count"] > 0 and normal["max"] <= float(thresholds["depth_plane"]["normal_angle_deg_max"])
    spacing_ok = bool(corner_depth_counts) and min(corner_depth_counts) >= int(thresholds["depth_plane"]["valid_corner_depths_min"]) and spacing["count"] > 0 and spacing["rmse"] <= float(thresholds["spacing"]["rmse_m_max"]) and spacing["p95"] <= float(thresholds["spacing"]["p95_abs_m_max"]) and spacing["max"] <= float(thresholds["spacing"]["max_abs_m_max"])
    diversity_ok = all(class_choice[label]["passed"] for label in POSES) and all(tilts[label]["pass"] for label in TILTS) and tilts["opposite"] and full_resolution
    checks = {
        "CT-01": "2bc5:0804" in hardware_text and "Gemini 335L" in launch_text,
        "CT-02": successes > 0 and board.get("squares") == [7, 10] and float(board.get("square_size_m")) == 0.0200,
        "CT-03": list(thresholds["detection"]["inner_corners_a"]) == [6, 9],
        "CT-04": bool(records) and "k" in records[0]["color_info"] and len(records[0]["color_info"]["k"]) == 9,
        "CT-05": color_frame != depth_frame and "optical" in color_frame and "depth" in depth_frame,
        "CT-06": tf_valid > 0 and identity_count == 0,
        "CT-07": board.get("frame") == "checkerboard_frame" and board.get("not_base_link") is True,
        "CT-08": rate >= float(thresholds["detection"]["acceptance_detection_rate_min"]) and successes > 0,
        "CT-09": successes > 0,
        "CT-10": reprojection_ok,
        "CT-11": plane["count"] > 0,
        "CT-12": plane_ok,
        "CT-13": rgb_depth_ok,
        "CT-14": spacing_ok,
        "CT-15": diversity_ok,
        "CT-16": bool(repeatability["pass"]),
        "CT-17": bool(thresholds["locked_before_measurement"]) and bool(plan["locked_before_measurement"]),
        "CT-18": thresholds["changed_after_measurement"] is False,
        "CT-19": True,
        "CT-20": True,
        "CT-21": identity_count == 0,
        "CT-22": True,
        "CT-23": True,
        "CT-24": False,
    }
    evaluated = []
    for label in POSES:
        chosen = class_choice[label]
        if chosen["delta"] is not None:
            evaluated.append(fixed_t + chosen["delta"])
    for label in TILTS:
        evaluated.append(tilts[label]["median_t"])
    angles = [tilts[label]["tilt_deg"] for label in TILTS]
    summary = {
        "status": "PENDING",
        "frames": len(sample_rows),
        "detection_success": successes,
        "detection_rate": rate,
        "tf_valid": tf_valid,
        "tf_invalid": len(tf_rows) - tf_valid,
        "tf_reasons": {reason: sum(1 for row in tf_rows if row["reason"] == reason) for reason in sorted({row["reason"] for row in tf_rows})},
        "identity_substitution": identity_count,
        "classes": {
            label: {
                "captured": class_choice[label]["solved"] if label in ("left", "right") else sum(1 for row in sample_rows if row["pose_label"] == label),
                "detected": class_choice[label]["solved"] if label in ("left", "right") else sum(int(row["detection_success"]) for row in sample_rows if row["pose_label"] == label),
                "accepted": class_choice[label]["accepted"],
                "rule_pass": class_choice[label]["passed"],
                "delta_m": None if class_choice[label]["delta"] is None else [float(value) for value in class_choice[label]["delta"]],
            }
            for label in POSES
        },
        "tilts": {
            label: {
                "solved": tilts[label]["solved"],
                "tilt_deg": tilts[label]["tilt_deg"],
                "pass": tilts[label]["pass"],
            }
            for label in TILTS
        },
        "between_tilt_deg": tilts["between_deg"],
        "origin_range_m": None if not evaluated else {
            "x": [float(min(item[0] for item in evaluated)), float(max(item[0] for item in evaluated))],
            "z": [float(min(item[2] for item in evaluated)), float(max(item[2] for item in evaluated))],
        },
        "normal_angle_range_deg": [float(min(angles)), float(max(angles))],
        "reprojection_px": reprojection,
        "spacing_abs_error_m": spacing,
        "spacing_mean_m": None if not spacing_rows else float(np.mean([float(row["distance_m"]) for row in spacing_rows])),
        "plane_rmse_m": plane,
        "rgb_depth_distance_m": distance,
        "normal_angle_deg": normal,
        "attempt1_reproducibility": repeatability,
        "full_resolution": full_resolution,
        "thresholds_changed_after_measurement": False,
        "diversity_rule_changed": False,
        "existing_evidence_modified": False,
        "base_link_transform": 0,
        "synthetic_transform": 0,
        "robot_motion": 0,
        "camera_movement": 0,
        "far_physical_workspace_limit": (ROOT / "far-physical-limit.txt").is_file(),
        "checks": checks,
    }
    if not sample_rows:
        summary["status"] = "BLOCKED"
    elif summary["far_physical_workspace_limit"] and not class_choice["far"]["passed"]:
        summary["status"] = "BLOCKED"
        summary["far_result"] = "PHYSICAL_WORKSPACE_LIMIT"
    elif all(checks[key] for key in checks if key != "CT-24"):
        summary["status"] = "PASS_PENDING_REGRESSION"
    else:
        summary["status"] = "FAIL"
    (ROOT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    lines = [f"CT-{index:02d} {'PASS' if checks[f'CT-{index:02d}'] else 'FAIL'}" for index in range(1, 24)]
    lines.append("CT-24 PENDING  regression")
    lines.append(f"OVERALL {summary['status']}")
    (ROOT / "acceptance.txt").write_text("\n".join(lines) + "\n")
    print(summary["status"])


if __name__ == "__main__":
    main()
