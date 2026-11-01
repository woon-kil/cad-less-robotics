#!/usr/bin/env python3
"""W-002E-R8. W-002E와 같은 NONE + SQPnP + 60 mm로 다시 등록한다.

ID2 중심만 W-002E-R7-R1 실측 0.450, 0.336 m이다.
SUBPIX, 마커별 크기, 임계값 변경은 하지 않는다.
효과 분류는 샘플을 보기 전에 고정한다.
재투영 mean/p95/max와 |z| mean/p95, |signed z|가 모두 줄면 IMPROVED.
모두 늘면 DEGRADED. 절대값이 재투영 0.05 px, 평면 0.05 mm 안이면 NO_MATERIAL_CHANGE.
나머지는 MIXED.
"""

import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy

import w002e_register as reg

OUT = Path("/ws/cad-less-robotics/evidence/W-002E-R8")
HISTORICAL = Path("/ws/cad-less-robotics/evidence/W-002E/camera_intrinsics.json")
W002E_MEAN = 1.8437682288188695
W002E_P95 = 2.751381026732307
W002E_MAX = 2.7562668054797266
W002E_ABS_MEAN = 2.34289456026592
W002E_ABS_P95 = 4.018965442925282
W002E_ANGLE = 0.5915848148131425
W002E_SIGNED = -2.340355647823267
MATERIAL_PX = 0.05
MATERIAL_MM = 0.05
reg.CENTERS_M[2] = np.array([0.450, 0.336, 0.0], dtype=np.float64)


def effect_label(reproj_deltas, plane_deltas):
    values = list(reproj_deltas) + list(plane_deltas)
    if all(abs(item) < (MATERIAL_PX if index < 3 else MATERIAL_MM) for index, item in enumerate(values)):
        return "NO_MATERIAL_CHANGE"
    if all(item < 0.0 for item in values):
        return "IMPROVED"
    if all(item > 0.0 for item in values):
        return "DEGRADED"
    return "MIXED"


def board_points(depth, found, camera_matrix, inverse, color_shape):
    if depth.shape[0] != color_shape[0] or depth.shape[1] != color_shape[1]:
        return None
    occupied = reg.marker_mask(color_shape, found)
    v0, v1 = reg.ROI_V
    u0, u1 = reg.ROI_U
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
            if z_m < reg.DEPTH_Z_MIN_M or z_m > reg.DEPTH_Z_MAX_M:
                continue
            rows.append(v)
            cols.append(u)
            zs.append(z_m)
    if len(zs) < reg.MIN_PLANE_POINTS:
        return None
    camera_points = np.vstack([
        reg.backproject(float(u), float(v), z, camera_matrix)
        for u, v, z in zip(cols, rows, zs)
    ])
    rotation = inverse[:3, :3]
    translation = inverse[:3, 3]
    return (rotation @ camera_points.T).T + translation


def decompose(board):
    z = board[:, 2]
    x = board[:, 0] - board[:, 0].mean()
    y = board[:, 1] - board[:, 1].mean()
    design = np.column_stack([np.ones(len(z)), x, y])
    coef, _residuals, _rank, _sv = np.linalg.lstsq(design, z, rcond=None)
    fitted = design @ coef
    tilt = fitted - coef[0]
    local = z - fitted
    return {
        "signed_z_mean_mm": float(z.mean() * 1000.0),
        "signed_z_median_mm": float(np.median(z) * 1000.0),
        "signed_offset_mm": float(coef[0] * 1000.0),
        "tilt_rms_mm": float(np.sqrt(np.mean(tilt ** 2)) * 1000.0),
        "local_plane_rms_mm": float(np.sqrt(np.mean(local ** 2)) * 1000.0),
    }


def marker_blocks(errors, labels, image_pts, projected):
    residual = image_pts - projected
    blocks = {}
    for marker_id in (0, 1, 2, 3):
        index = [i for i, label in enumerate(labels) if label["id"] == marker_id]
        res = residual[index]
        err = errors[index]
        centroid = res.mean(axis=0)
        internal = res - centroid
        blocks[str(marker_id)] = {
            "mean_px": float(err.mean()),
            "max_px": float(err.max()),
            "centroid_residual_px": [float(centroid[0]), float(centroid[1])],
            "centroid_norm_px": float(np.linalg.norm(centroid)),
            "internal_rms_px": float(np.sqrt(np.mean(np.sum(internal ** 2, axis=1)))),
        }
    return blocks


def geometry_match(live, stored):
    live_k = np.asarray(live["camera_matrix"], dtype=np.float64)
    stored_k = np.asarray(stored["k"], dtype=np.float64)
    live_d = np.asarray(live["distortion"], dtype=np.float64)
    stored_d = np.asarray(stored["distortion_coefficients"], dtype=np.float64)
    k_ok = bool(np.allclose(live_k, stored_k, rtol=0.0, atol=1e-4))
    d_ok = bool(np.allclose(live_d, stored_d, rtol=0.0, atol=1e-8))
    frame_ok = live["frame_id"] == stored["frame_id"] == "camera_color_optical_frame"
    size_ok = live["width"] == stored["image_width"] and live["height"] == stored["image_height"]
    return {
        "k_match": k_ok,
        "d_match": d_ok,
        "frame_match": frame_ok,
        "size_match": size_ok,
        "pass": bool(k_ok and d_ok and frame_ok and size_ok),
    }


def first_failure(reproj, plane, repeatability, rotation_ok, inverse_ok):
    checks = [
        ("DETECTED_COUNT", True),
        ("REPROJECTION_MEAN_PX", reproj["mean_px"] <= 1.0),
        ("REPROJECTION_P95_PX", reproj["p95_px"] <= 2.0),
        ("REPROJECTION_MAX_PX", reproj["max_px"] <= 3.0),
        ("WORKBOARD_PLANE_ABS_Z_MEAN_MM", plane["abs_z_mean_mm"] <= 2.0),
        ("WORKBOARD_PLANE_ABS_Z_P95_MM", plane["abs_z_p95_mm"] <= 4.0),
        ("WORKBOARD_PLANE_NORMAL_ANGLE_DEG", plane["normal_angle_deg"] <= 1.0),
        ("REPEATABILITY_TRANSLATION_MM", repeatability["translation_repeatability_mm"] <= 2.0),
        ("REPEATABILITY_ROTATION_DEG", repeatability["rotation_repeatability_deg"] <= 0.4),
        ("ROTATION_VALIDITY", rotation_ok),
        ("INVERSE_CONSISTENCY", inverse_ok),
    ]
    for name, ok in checks:
        if not ok:
            return name
    return None


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    flag = reg.pnp_flag()
    if flag != int(cv2.SOLVEPNP_SQPNP):
        raise SystemExit("SQPnP flag mismatch")
    stored = json.loads(HISTORICAL.read_text(encoding="utf-8"))
    rclpy.init()
    node = reg.Grab()
    samples = []
    last_stamp = -1.0
    deadline = time.monotonic() + 90.0
    rejected = []
    while len(samples) < reg.TARGET_FRAMES and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if not node.ready_pair():
            continue
        if node.color_stamp <= last_stamp + reg.FRAME_GAP_SEC:
            continue
        color = node.color.copy()
        depth = node.depth.copy()
        info = reg.intrinsics_of(node.info)
        found = reg.detect_all(color)
        last_stamp = node.color_stamp
        visible = reg.full_visible(found, color.shape[1], color.shape[0])
        if not visible or node.color_frame != "camera_color_optical_frame":
            rejected.append({
                "detected_ids": sorted(found),
                "color_frame": node.color_frame,
                "depth_frame": node.depth_frame,
            })
            continue
        if node.depth_frame != node.color_frame:
            rejected.append({"reason": "DEPTH_FRAME_IS_NOT_COLOR_FRAME", "depth_frame": node.depth_frame})
            continue
        object_pts, image_pts, labels = reg.stack_correspondence(found)
        solved = reg.solve_one(object_pts, image_pts, info["camera_matrix"], info["distortion"], flag)
        if solved is None:
            rejected.append({"pnp_success": False})
            continue
        transform = reg.homogeneous(solved["rotation"], solved["translation"])
        inverse = np.linalg.inv(transform)
        errors = reg.reprojection_block(solved["errors"], labels)
        samples.append({
            "color": color,
            "depth": depth,
            "found": found,
            "info": info,
            "labels": labels,
            "image_pts": image_pts,
            "projected": solved["projected"],
            "rotation": solved["rotation"],
            "translation": solved["translation"],
            "transform": transform,
            "inverse": inverse,
            "errors": errors,
            "error_values": solved["errors"],
            "rotation_validity": reg.rotation_report(solved["rotation"]),
            "inverse_consistency": reg.inverse_report(transform, inverse),
            "color_frame": node.color_frame,
            "depth_frame": node.depth_frame,
        })
        print("frame", len(samples) - 1, "mean_px", round(errors["mean_px"], 4), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    if len(samples) < reg.TARGET_FRAMES:
        reg.dump(OUT / "result.json", {
            "status": "BLOCKED_ACQUISITION",
            "closed": True,
            "accepted_frames": len(samples),
            "rejected": rejected,
            "validated_registration_transform": "NONE",
        })
        print("BLOCKED", len(samples))
        return 2

    medoid, scores = reg.select_medoid(samples)
    chosen = samples[medoid]
    translations = [
        float(np.linalg.norm(sample["translation"] - chosen["translation"]) * 1000.0)
        for sample in samples
    ]
    rotations = [
        reg.so3_angle_deg(chosen["rotation"], sample["rotation"])
        for sample in samples
    ]
    repeatability = {
        "frame_count": len(samples),
        "method": "SE3_MEDOID_MIN_SUM_TRANSLATION_MM_PLUS_ROTATION_DEG",
        "tie_break": "LOWEST_INDEX",
        "medoid_index": medoid,
        "medoid_scores": scores,
        "translation_repeatability_mm": float(max(translations)),
        "rotation_repeatability_deg": float(max(rotations)),
    }
    match = geometry_match(chosen["info"], stored)
    plane = reg.plane_from_depth(
        chosen["depth"],
        chosen["found"],
        chosen["info"]["camera_matrix"],
        chosen["inverse"],
        chosen["color"].shape,
    )
    plane.pop("z_mm", None)
    board = board_points(
        chosen["depth"],
        chosen["found"],
        chosen["info"]["camera_matrix"],
        chosen["inverse"],
        chosen["color"].shape,
    )
    extra = decompose(board) if board is not None else {}
    plane.update(extra)
    reproj = chosen["errors"]
    blocks = marker_blocks(
        chosen["error_values"],
        chosen["labels"],
        chosen["image_pts"],
        chosen["projected"],
    )
    delta_mean = reproj["mean_px"] - W002E_MEAN
    delta_p95 = reproj["p95_px"] - W002E_P95
    delta_max = reproj["max_px"] - W002E_MAX
    delta_abs_mean = plane.get("abs_z_mean_mm", 0.0) - W002E_ABS_MEAN
    delta_abs_p95 = plane.get("abs_z_p95_mm", 0.0) - W002E_ABS_P95
    delta_signed_abs = abs(plane.get("signed_z_mean_mm", 0.0)) - abs(W002E_SIGNED)
    package = effect_label(
        [delta_mean, delta_p95, delta_max],
        [delta_abs_mean, delta_abs_p95, delta_signed_abs],
    )
    failure = None
    status = "FAIL_REGISTRATION_RECOVERY"
    validated = "NONE"
    if not match["pass"] or not plane.get("executed"):
        status = "BLOCKED_CAMERA_OR_DEPTH_CHAIN"
    else:
        failure = first_failure(
            reproj,
            plane,
            repeatability,
            chosen["rotation_validity"]["pass"],
            chosen["inverse_consistency"]["pass"],
        )
        if failure is None:
            status = "PASS_REGISTRATION_RECOVERY"
            validated = "T_camera_workboard_R8"
    bgr = cv2.cvtColor(chosen["color"], cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(OUT / "raw_color.png"), bgr)
    overlay = reg.draw_overlay(chosen["color"], chosen["found"], (reg.ROI_U, reg.ROI_V))
    cv2.imwrite(str(OUT / "detected_ids.png"), overlay)
    for label, point in zip(chosen["labels"], chosen["projected"]):
        cv2.circle(overlay, tuple(np.round(point).astype(int)), 3, (0, 0, 255), -1)
    cv2.imwrite(str(OUT / "reprojection_overlay.png"), overlay)
    u0, u1 = reg.ROI_U
    v0, v1 = reg.ROI_V
    cv2.imwrite(str(OUT / "bare_wood_roi.png"), bgr[v0:v1 + 1, u0:u1 + 1])
    reg.dump(OUT / "camera_info.json", {
        "frame_id": chosen["info"]["frame_id"],
        "width": chosen["info"]["width"],
        "height": chosen["info"]["height"],
        "k": chosen["info"]["k"],
        "d": chosen["info"]["d"],
        "match_to_w002e": match,
        "color_image_domain": "RAW_DISTORTED",
        "image_pre_undistorted": False,
        "undistort_points_used": False,
        "corner_refinement": "NONE",
    })
    reg.dump(OUT / "reprojection.json", reproj)
    reg.dump(OUT / "per_marker_residual.json", blocks)
    reg.dump(OUT / "plane_metrics.json", plane)
    reg.dump(OUT / "repeatability.json", repeatability)
    reg.dump(OUT / "transform.json", {
        "T_camera_workboard_R8": reg.matrix_list(chosen["transform"]),
        "T_workboard_camera_R8": reg.matrix_list(chosen["inverse"]),
        "rotation_validity": chosen["rotation_validity"],
        "inverse_consistency": chosen["inverse_consistency"],
        "validated": validated == "T_camera_workboard_R8",
    })
    reg.dump(OUT / "comparison_w002e.json", {
        "delta_mean_px": delta_mean,
        "delta_p95_px": delta_p95,
        "delta_max_px": delta_max,
        "delta_abs_z_mean_mm": delta_abs_mean,
        "delta_abs_z_p95_mm": delta_abs_p95,
        "delta_abs_signed_z_mm": delta_signed_abs,
        "w002e_normal_angle_deg": W002E_ANGLE,
        "r8_normal_angle_deg": plane.get("normal_angle_deg"),
        "effect_rule": "all six deltas negative IMPROVED; all positive DEGRADED; within 0.05 NO_MATERIAL_CHANGE; else MIXED",
        "fiducial_recovery_package_effect": package,
        "id2_nonplanarity_was_sole_root_cause": "NOT_ESTABLISHED",
    })
    reg.dump(OUT / "result.json", {
        "status": status,
        "closed": True,
        "first_failure": failure,
        "centers_mm": {"0": [148, 71, 0], "1": [500, 69, 0], "2": [450, 336, 0], "3": [149, 332, 0]},
        "marker_black_square_model_mm": 60,
        "marker_size_model_changed_from_w002e": False,
        "corner_refinement": "NONE",
        "pnp_method": "SOLVEPNP_SQPNP",
        "detected_ids": [0, 1, 2, 3],
        "detected_count": 4,
        "reprojection_mean_px": reproj["mean_px"],
        "reprojection_p95_px": reproj["p95_px"],
        "reprojection_max_px": reproj["max_px"],
        "delta_vs_w002e_mean_px": delta_mean,
        "delta_vs_w002e_p95_px": delta_p95,
        "delta_vs_w002e_max_px": delta_max,
        "workboard_plane_abs_z_mean_mm": plane.get("abs_z_mean_mm"),
        "workboard_plane_abs_z_p95_mm": plane.get("abs_z_p95_mm"),
        "workboard_plane_normal_angle_deg": plane.get("normal_angle_deg"),
        "signed_z_mean_mm": plane.get("signed_z_mean_mm"),
        "repeatability_translation_mm": repeatability["translation_repeatability_mm"],
        "repeatability_rotation_deg": repeatability["rotation_repeatability_deg"],
        "rotation_validity": chosen["rotation_validity"]["pass"],
        "inverse_consistency": chosen["inverse_consistency"]["pass"],
        "fiducial_recovery_package_effect": package,
        "id2_nonplanarity_was_sole_root_cause": "NOT_ESTABLISHED",
        "validated_registration_transform": validated,
        "w002e_historical_status": "FAIL_REPROJECTION",
        "adr010_status": "ACCEPTED",
        "thresholds_relaxed": False,
        "p009_r1_opened": False,
        "p010_opened": False,
        "w002f_opened": False,
        "next_gate_opened": False,
    })
    print("STATUS", status)
    print("REPROJ", reproj["mean_px"], reproj["p95_px"], reproj["max_px"])
    print("PLANE", plane.get("abs_z_mean_mm"), plane.get("abs_z_p95_mm"), plane.get("normal_angle_deg"), plane.get("signed_z_mean_mm"))
    print("REPEAT", repeatability["translation_repeatability_mm"], repeatability["rotation_repeatability_deg"])
    print("EFFECT", package, "FAIL", failure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
