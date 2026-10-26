#!/usr/bin/env python3
"""Attempt 1–5의 실측 포즈만으로 작업면과 optical Z 변화율을 계산한다. 새 촬영은 하지 않는다."""

import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path("evidence/P-006/camera-target")
OUT = ROOT / "attempt-06-diagnosis"
FIXED = np.array([-0.04281625, -0.14894789, 0.46428018], dtype=np.float64)
TARGET_Z_M = 0.040
TILT_LABELS = {"tilt_positive", "tilt_negative"}
TF_INVALID = {
    ("attempt-02", "tilt_negative-00"),
    ("attempt-03", "near-t01-00"),
    ("attempt-04", "far-t01-00"),
    ("attempt-04", "near-t03-00"),
    ("attempt-05", "near-t01-00"),
}


def quaternion_to_matrix(quaternion):
    x, y, z, w = (float(value) for value in quaternion)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def project_unit(axis, normal):
    tangent = axis - float(np.dot(axis, normal)) * normal
    return tangent / np.linalg.norm(tangent)


def load_attempt1():
    rows = []
    for record in csv.DictReader((ROOT / "color-pose.csv").open()):
        rotation = quaternion_to_matrix((record["qx"], record["qy"], record["qz"], record["qw"]))
        normal = rotation[:, 2]
        rows.append(
            {
                "attempt": "attempt-01",
                "stem": f"{int(record['index']):04d}",
                "pose_label": "fixed",
                "tx_m": float(record["tx_m"]),
                "ty_m": float(record["ty_m"]),
                "tz_m": float(record["tz_m"]),
                "nx": float(normal[0]),
                "ny": float(normal[1]),
                "nz": float(normal[2]),
            }
        )
    return rows


def load_poses(attempt, path):
    rows = []
    for record in csv.DictReader(path.open()):
        if "nx" in record and record["nx"] != "":
            normal = (float(record["nx"]), float(record["ny"]), float(record["nz"]))
        else:
            rotation = quaternion_to_matrix((record["qx"], record["qy"], record["qz"], record["qw"]))
            normal = tuple(float(value) for value in rotation[:, 2])
        rows.append(
            {
                "attempt": attempt,
                "stem": record["stem"],
                "pose_label": record["pose_label"],
                "tx_m": float(record["tx_m"]),
                "ty_m": float(record["ty_m"]),
                "tz_m": float(record["tz_m"]),
                "nx": normal[0],
                "ny": normal[1],
                "nz": normal[2],
            }
        )
    return rows


def write_csv(path, rows, fields):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    poses = load_attempt1()
    poses.extend(load_poses("attempt-02", ROOT / "attempt-02" / "poses.csv"))
    poses.extend(load_poses("attempt-03", ROOT / "attempt-03" / "poses.csv"))
    poses.extend(load_poses("attempt-04", ROOT / "attempt-04" / "poses.csv"))
    poses.extend(load_poses("attempt-05", ROOT / "attempt-05" / "poses.csv"))

    valid = []
    tf_invalid_with_pose = 0
    for pose in poses:
        pose["delta_x_m"] = pose["tx_m"] - float(FIXED[0])
        pose["delta_y_m"] = pose["ty_m"] - float(FIXED[1])
        pose["delta_z_m"] = pose["tz_m"] - float(FIXED[2])
        excluded = (pose["attempt"], pose["stem"]) in TF_INVALID
        pose["tf_valid"] = int(not excluded)
        pose["workplane_sample"] = int((not excluded) and pose["pose_label"] not in TILT_LABELS)
        if excluded:
            tf_invalid_with_pose += 1
            continue
        valid.append(pose)

    plane_rows = [pose for pose in valid if pose["workplane_sample"]]
    points = np.array([[pose["tx_m"], pose["ty_m"], pose["tz_m"]] for pose in plane_rows], dtype=np.float64)
    board_normals = np.array([[pose["nx"], pose["ny"], pose["nz"]] for pose in plane_rows], dtype=np.float64)
    centroid = points.mean(axis=0)
    _, singular, vt = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vt[-1].copy()
    mean_board = board_normals.mean(axis=0)
    mean_board = mean_board / np.linalg.norm(mean_board)
    if float(np.dot(normal, mean_board)) < 0.0:
        normal = -normal
    normal = normal / np.linalg.norm(normal)
    plane_d = -float(np.dot(normal, centroid))
    residuals = points @ normal + plane_d
    residual_rmse = float(math.sqrt(np.mean(residuals ** 2)))
    residual_max = float(np.max(np.abs(residuals)))

    image_right = project_unit(np.array([1.0, 0.0, 0.0]), normal)
    image_down = project_unit(np.array([0.0, 1.0, 0.0]), normal)
    optical = np.array([0.0, 0.0, 1.0])
    gradient = optical - float(np.dot(optical, normal)) * normal
    gradient_unit = gradient / np.linalg.norm(gradient)
    per_m = {
        "image_right": float(image_right[2]),
        "image_down": float(image_down[2]),
        "steepest_increase": float(gradient_unit[2]),
    }
    offsets = points - FIXED
    s_right = offsets @ image_right
    s_down = offsets @ image_down
    delta_z = points[:, 2] - float(FIXED[2])
    design = np.column_stack([s_right, s_down])
    slope, _, _, _ = np.linalg.lstsq(design, delta_z, rcond=None)
    predicted = design @ slope
    empirical_rmse = float(math.sqrt(np.mean((delta_z - predicted) ** 2)))

    def travel(target_z, rate):
        if abs(rate) < 1e-9:
            return None
        return float(target_z / rate)

    near_along_down = travel(-TARGET_Z_M, per_m["image_down"])
    far_along_up = travel(TARGET_Z_M, -per_m["image_down"])
    near_steepest = travel(-TARGET_Z_M, -per_m["steepest_increase"])
    far_steepest = travel(TARGET_Z_M, per_m["steepest_increase"])

    for pose, right, down, residual in zip(plane_rows, s_right, s_down, residuals):
        pose["s_image_right_m"] = float(right)
        pose["s_image_down_m"] = float(down)
        pose["plane_residual_m"] = float(residual)
    for pose in valid:
        pose.setdefault("s_image_right_m", "")
        pose.setdefault("s_image_down_m", "")
        pose.setdefault("plane_residual_m", "")

    near_index = int(np.argmin(points[:, 2]))
    far_index = int(np.argmax(points[:, 2]))
    observed = {
        "most_negative_z": {
            "attempt": plane_rows[near_index]["attempt"],
            "stem": plane_rows[near_index]["stem"],
            "pose_label": plane_rows[near_index]["pose_label"],
            "delta_z_m": float(delta_z[near_index]),
            "s_image_down_m": float(s_down[near_index]),
            "s_image_right_m": float(s_right[near_index]),
        },
        "most_positive_z": {
            "attempt": plane_rows[far_index]["attempt"],
            "stem": plane_rows[far_index]["stem"],
            "pose_label": plane_rows[far_index]["pose_label"],
            "delta_z_m": float(delta_z[far_index]),
            "s_image_down_m": float(s_down[far_index]),
            "s_image_right_m": float(s_right[far_index]),
        },
        "s_image_down_m": {"min": float(np.min(s_down)), "max": float(np.max(s_down))},
        "s_image_right_m": {"min": float(np.min(s_right)), "max": float(np.max(s_right))},
        "delta_z_m": {"min": float(np.min(delta_z)), "max": float(np.max(delta_z))},
        "attempt5_near_try2_median_delta_z_m": float(np.median([pose["delta_z_m"] for pose in plane_rows if pose["attempt"] == "attempt-05" and pose["stem"].startswith("near-t02-")])),
        "far_physical_workspace_limit": True,
        "far_limit_has_valid_pose": False,
    }

    failed_frames = []
    for attempt, relative in (
        ("attempt-01", ROOT / "samples.csv"),
        ("attempt-02", ROOT / "attempt-02" / "samples.csv"),
        ("attempt-03", ROOT / "attempt-03" / "samples.csv"),
        ("attempt-04", ROOT / "attempt-04" / "samples.csv"),
        ("attempt-05", ROOT / "attempt-05" / "samples.csv"),
    ):
        for record in csv.DictReader(relative.open()):
            if int(record["detection_success"]) == 0:
                stem = record["stem"] if "stem" in record else f"{int(record['index']):04d}"
                failed_frames.append(f"{attempt}/{stem}")

    detected = [pose for pose in valid if pose["workplane_sample"] or pose["pose_label"] in TILT_LABELS]
    detected_points = np.array([[pose["tx_m"], pose["ty_m"], pose["tz_m"]] for pose in detected], dtype=np.float64)
    envelope = {
        "successful_pose_count": len(detected),
        "successful_x_m": [float(np.min(detected_points[:, 0])), float(np.max(detected_points[:, 0]))],
        "successful_y_m": [float(np.min(detected_points[:, 1])), float(np.max(detected_points[:, 1]))],
        "successful_z_m": [float(np.min(detected_points[:, 2])), float(np.max(detected_points[:, 2]))],
        "failed_detection_frames": failed_frames,
        "pixel_to_metric": "not_used",
        "qualitative_fov": {
            "attempt-05/far-t01-00": "컬러 프레임 안에 체크보드 용지가 있고 이미지 경계에 잘리지 않았다. 용지와 화면 위쪽 카메라 받침 사이에 작업면이 보인다. 픽셀은 거리로 환산하지 않았다.",
            "attempt-05/near-t01-00": "컬러 프레임 안에 체크보드 용지가 있고 이미지 경계에 잘리지 않았다. 화면 아래·오른쪽으로 치우쳐 있다. 픽셀은 거리로 환산하지 않았다.",
        },
    }

    normal_angle = math.degrees(math.acos(float(np.clip(np.dot(normal, optical), -1.0, 1.0))))
    workplane = {
        "sample_count": len(plane_rows),
        "tilt_excluded": sum(1 for pose in valid if pose["pose_label"] in TILT_LABELS),
        "tf_invalid_excluded_with_pose": tf_invalid_with_pose,
        "tf_invalid_frames_recorded": len(TF_INVALID),
        "centroid_m": [float(value) for value in centroid],
        "normal_camera": [float(value) for value in normal],
        "d_m": plane_d,
        "equation": "n · p + d = 0",
        "residual_rmse_m": residual_rmse,
        "residual_max_m": residual_max,
        "singular_values_m": [float(value) for value in singular],
        "normal_angle_from_optical_axis_deg": normal_angle,
        "mean_board_plus_z": [float(value) for value in mean_board],
    }
    gradient_doc = {
        "prediction_not_measurement": True,
        "image_down_is_camera_plus_y": True,
        "optical_z_change_per_m": per_m,
        "optical_z_change_per_100mm": {key: value * 0.1 for key, value in per_m.items()},
        "image_right_unit_camera": [float(value) for value in image_right],
        "image_down_unit_camera": [float(value) for value in image_down],
        "steepest_increase_unit_camera": [float(value) for value in gradient_unit],
        "empirical_dz_per_m_image_right": float(slope[0]),
        "empirical_dz_per_m_image_down": float(slope[1]),
        "empirical_rmse_m": empirical_rmse,
        "partial_dz_per_camera_x": float(-normal[0] / normal[2]),
        "partial_dz_per_camera_y": float(-normal[1] / normal[2]),
    }
    required = {
        "prediction_not_measurement": True,
        "target_abs_delta_z_m": TARGET_Z_M,
        "locked_rule": "near median tz < fixed tz - 0.040 m; far median tz > fixed tz + 0.040 m",
        "threshold_unchanged": True,
        "attempt5_minus_0_034581_m_remains_fail": True,
        "near_along_image_down_m": near_along_down,
        "far_along_image_up_m": far_along_up,
        "near_along_steepest_m": near_steepest,
        "far_along_steepest_m": far_steepest,
        "observed": observed,
    }

    OUT.mkdir(parents=True, exist_ok=True)
    input_files = [
        ROOT / "color-pose.csv",
        ROOT / "samples.csv",
        ROOT / "attempt-02" / "poses.csv",
        ROOT / "attempt-02" / "samples.csv",
        ROOT / "attempt-03" / "poses.csv",
        ROOT / "attempt-03" / "samples.csv",
        ROOT / "attempt-03" / "diversity-validation.csv",
        ROOT / "attempt-04" / "poses.csv",
        ROOT / "attempt-04" / "samples.csv",
        ROOT / "attempt-05" / "poses.csv",
        ROOT / "attempt-05" / "samples.csv",
        ROOT / "attempt-05" / "diversity-validation.csv",
        ROOT / "attempt-05" / "far-physical-limit.txt",
    ]
    input_doc = ["inputs:"]
    for path in input_files:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        input_doc.append(f"  - path: {path.as_posix()}")
        input_doc.append(f"    sha256: {digest}")
    input_doc.append("fixed_reference_m: [-0.04281625, -0.14894789, 0.46428018]")
    input_doc.append("new_capture: 0")
    (OUT / "input-evidence.yaml").write_text("\n".join(input_doc) + "\n")
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        figure, axes = plt.subplots(1, 2, figsize=(9.5, 4.2))
        down = np.array([pose["s_image_down_m"] for pose in plane_rows], dtype=np.float64)
        right = np.array([pose["s_image_right_m"] for pose in plane_rows], dtype=np.float64)
        z_mm = delta_z * 1000.0
        axes[0].scatter(down * 1000.0, z_mm, s=12)
        axes[0].axhline(-40.0, color="0.4", linewidth=0.8)
        axes[0].axhline(40.0, color="0.4", linewidth=0.8)
        axes[0].set_xlabel("image down on workplane (mm)")
        axes[0].set_ylabel("delta Z (mm)")
        axes[1].scatter(right * 1000.0, z_mm, s=12)
        axes[1].axhline(-40.0, color="0.4", linewidth=0.8)
        axes[1].axhline(40.0, color="0.4", linewidth=0.8)
        axes[1].set_xlabel("image right on workplane (mm)")
        figure.tight_layout()
        figure.savefig(OUT / "optical-z-scatter.png", dpi=120)
        plt.close(figure)
    except ImportError:
        pass
    fields = [
        "attempt",
        "stem",
        "pose_label",
        "tx_m",
        "ty_m",
        "tz_m",
        "nx",
        "ny",
        "nz",
        "delta_x_m",
        "delta_y_m",
        "delta_z_m",
        "tf_valid",
        "workplane_sample",
        "s_image_right_m",
        "s_image_down_m",
        "plane_residual_m",
    ]
    write_csv(OUT / "valid-poses.csv", valid, fields)
    (OUT / "workplane-fit.json").write_text(json.dumps(workplane, indent=2) + "\n")
    (OUT / "optical-z-gradient.json").write_text(json.dumps(gradient_doc, indent=2) + "\n")
    (OUT / "required-displacement.json").write_text(json.dumps(required, indent=2) + "\n")
    (OUT / "detection-envelope.json").write_text(json.dumps(envelope, indent=2) + "\n")
    print(json.dumps({"workplane": workplane, "gradient": gradient_doc, "required": required}, indent=2))


if __name__ == "__main__":
    main()
