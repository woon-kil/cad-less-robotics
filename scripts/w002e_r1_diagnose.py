#!/usr/bin/env python3
"""W-002E-R1. W-002E 저장 프레임의 residual만 본다.

geometry, threshold, solver를 바꾸지 않는다.
민감도 계산은 진단이며 새 등록값을 채택하지 않는다.
판정 경계는 숫자를 보기 전에 이 상수로 고정한다.
"""

import json
from pathlib import Path

import cv2
import numpy as np

SRC = Path("/ws/cad-less-robotics/evidence/W-002E")
OUT = Path("/ws/cad-less-robotics/evidence/W-002E-R1")
STORED_MEAN_PX = 1.8437682288188695
HALF_NOMINAL_M = 0.030
MEASURED_M = {
    0: np.array([0.148, 0.071, 0.0]),
    1: np.array([0.500, 0.069, 0.0]),
    2: np.array([0.500, 0.336, 0.0]),
    3: np.array([0.149, 0.332, 0.0]),
}
TARGET_M = {
    0: np.array([0.150, 0.070, 0.0]),
    1: np.array([0.500, 0.070, 0.0]),
    2: np.array([0.500, 0.330, 0.0]),
    3: np.array([0.150, 0.330, 0.0]),
}
SIZE_MM = (58.0, 59.0, 60.0, 61.0, 62.0)
ROI_U = (480, 700)
ROI_V = (430, 560)
ARROW_SCALE = 20.0
# 평균 재투영이 0.30 px 이상 움직이면 MATERIAL, 0.15 px 미만이면 LOW.
SENSITIVITY_MATERIAL_PX = 0.30
SENSITIVITY_LOW_PX = 0.15
# 평면: |mean z|가 선형 tilt RMS의 2배 이상이면 OFFSET, 반대면 TILT.
PLANE_RATIO = 2.0
PLANE_MIXED_MM = 1.0


def dump(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")


def load_json(path):
    return json.loads(path.read_text())


def local_corners(half_m):
    return np.array([
        [-half_m, half_m, 0.0],
        [half_m, half_m, 0.0],
        [half_m, -half_m, 0.0],
        [-half_m, -half_m, 0.0],
    ], dtype=np.float64)


def stack(found, centers, half_m, ids):
    local = local_corners(half_m)
    objects = []
    images = []
    labels = []
    for marker_id in ids:
        for corner_index in range(4):
            objects.append(centers[marker_id] + local[corner_index])
            images.append(found[marker_id][corner_index])
            labels.append({"id": marker_id, "corner": corner_index})
    return (
        np.asarray(objects, dtype=np.float64),
        np.asarray(images, dtype=np.float64),
        labels,
    )


def solve(object_pts, image_pts, camera_matrix, distortion):
    flag = cv2.SOLVEPNP_SQPNP
    ok, rvec, tvec = cv2.solvePnP(
        object_pts, image_pts, camera_matrix, distortion, flags=flag)
    if not ok:
        return None
    projected, _ = cv2.projectPoints(object_pts, rvec, tvec, camera_matrix, distortion)
    projected = projected.reshape(-1, 2)
    residual = projected - image_pts
    errors = np.linalg.norm(residual, axis=1)
    rotation, _ = cv2.Rodrigues(rvec)
    translation = np.asarray(tvec, dtype=np.float64).reshape(3)
    return {
        "projected": projected,
        "residual": residual,
        "errors": errors,
        "rotation": np.asarray(rotation, dtype=np.float64),
        "translation": translation,
        "rvec": np.asarray(rvec, dtype=np.float64).reshape(3),
    }


def error_stats(errors):
    return {
        "mean_px": float(np.mean(errors)),
        "median_px": float(np.median(errors)),
        "p95_px": float(np.percentile(errors, 95)),
        "max_px": float(np.max(errors)),
        "count": int(errors.size),
    }


def detect(image):
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    found = {}
    if ids is None:
        return found
    for index, value in enumerate(ids.flatten()):
        found[int(value)] = np.asarray(corners[index], dtype=np.float64).reshape(4, 2)
    return found


def found_from_json(payload):
    found = {}
    for marker in payload["markers"]:
        found[int(marker["id"])] = np.asarray(marker["corners_px"], dtype=np.float64)
    return found


def sensitivity_label(baseline_mean, other_mean):
    delta = abs(other_mean - baseline_mean)
    improvement = baseline_mean - other_mean
    if improvement >= SENSITIVITY_MATERIAL_PX or delta >= SENSITIVITY_MATERIAL_PX and improvement > 0:
        return "MATERIAL", float(delta), float(improvement)
    # MATERIAL은 어느 쪽이든 0.30 px 이상 차이가 날 때.
    if delta >= SENSITIVITY_MATERIAL_PX:
        return "MATERIAL", float(delta), float(improvement)
    if delta < SENSITIVITY_LOW_PX:
        return "LOW", float(delta), float(improvement)
    return "INCONCLUSIVE", float(delta), float(improvement)


def size_label(means):
    nominal = means[60.0]
    deltas = [abs(value - nominal) for value in means.values()]
    best_drop = nominal - min(means.values())
    if best_drop >= SENSITIVITY_MATERIAL_PX or max(deltas) >= SENSITIVITY_MATERIAL_PX:
        return "MATERIAL", float(best_drop), float(max(deltas))
    if max(deltas) < SENSITIVITY_LOW_PX:
        return "LOW", float(best_drop), float(max(deltas))
    return "INCONCLUSIVE", float(best_drop), float(max(deltas))


def marker_block(residual, errors, labels, image_pts, projected):
    blocks = {}
    for marker_id in (0, 1, 2, 3):
        index = [i for i, label in enumerate(labels) if label["id"] == marker_id]
        res = residual[index]
        err = errors[index]
        obs = image_pts[index]
        proj = projected[index]
        centroid = res.mean(axis=0)
        internal = res - centroid
        obs_sides = [np.linalg.norm(obs[(i + 1) % 4] - obs[i]) for i in range(4)]
        proj_sides = [np.linalg.norm(proj[(i + 1) % 4] - proj[i]) for i in range(4)]
        scale_ratio = float(np.mean(obs_sides) / np.mean(proj_sides))
        centroid_norm = float(np.linalg.norm(centroid))
        internal_rms = float(np.sqrt(np.mean(np.sum(internal ** 2, axis=1))))
        if centroid_norm > 2.0 * internal_rms:
            dominant = "CENTROID"
        elif internal_rms > 2.0 * centroid_norm:
            dominant = "INTERNAL"
        else:
            dominant = "MIXED"
        blocks[str(marker_id)] = {
            "mean_du_px": float(centroid[0]),
            "mean_dv_px": float(centroid[1]),
            "mean_error_px": float(err.mean()),
            "max_error_px": float(err.max()),
            "centroid_residual_px": [float(centroid[0]), float(centroid[1])],
            "centroid_norm_px": centroid_norm,
            "internal_rms_px": internal_rms,
            "corner_residual_spread_px": float(np.std(err)),
            "observed_over_projected_side": scale_ratio,
            "scale_residual": scale_ratio - 1.0,
            "dominant": dominant,
        }
    return blocks


def pattern_label(blocks, errors, image_pts, principal):
    flags = []
    angles = []
    for block in blocks.values():
        vector = np.array(block["centroid_residual_px"], dtype=np.float64)
        if np.linalg.norm(vector) > 1e-6:
            angles.append(float(np.degrees(np.arctan2(vector[1], vector[0]))))
    if len(angles) == 4:
        spread = float(np.max(angles) - np.min(angles))
        # 각도 wrap. 네 방향의 최대 쌍별 차.
        pair = []
        for i in range(4):
            for j in range(i + 1, 4):
                diff = abs(angles[i] - angles[j]) % 360.0
                pair.append(min(diff, 360.0 - diff))
        spread = float(max(pair))
        if spread < 30.0 and all(block["dominant"] == "CENTROID" for block in blocks.values()):
            flags.append("SIMILAR_MARKER_SHIFT")
    scales = [block["scale_residual"] for block in blocks.values()]
    if all(value > 0.01 for value in scales) or all(value < -0.01 for value in scales):
        flags.append("COMMON_SCALE")
    radius = np.linalg.norm(image_pts - principal, axis=1)
    if radius.std() > 0 and errors.std() > 0:
        correlation = float(np.corrcoef(radius, errors)[0, 1])
    else:
        correlation = 0.0
    if correlation > 0.5:
        flags.append("PERIPHERAL_GROWTH")
    if all(block["dominant"] == "INTERNAL" for block in blocks.values()):
        flags.append("INTERNAL_CORNER")
    means = [block["mean_error_px"] for block in blocks.values()]
    median_mean = float(np.median(means))
    if any(block["mean_error_px"] > 1.5 * median_mean and block["dominant"] == "CENTROID" for block in blocks.values()):
        flags.append("ONE_MARKER_SHIFT")
    return flags, correlation, angles


def leave_one_out(found, camera_matrix, distortion, all_mean):
    rows = []
    isolated = []
    for excluded in (0, 1, 2, 3):
        fit_ids = [marker_id for marker_id in (0, 1, 2, 3) if marker_id != excluded]
        fit_obj, fit_img, _fit_labels = stack(found, MEASURED_M, HALF_NOMINAL_M, fit_ids)
        solved = solve(fit_obj, fit_img, camera_matrix, distortion)
        if solved is None:
            rows.append({"excluded_id": excluded, "pnp_success": False})
            continue
        hold_obj, hold_img, _hold_labels = stack(found, MEASURED_M, HALF_NOMINAL_M, [excluded])
        projected, _ = cv2.projectPoints(
            hold_obj, solved["rvec"], solved["translation"], camera_matrix, distortion)
        projected = projected.reshape(-1, 2)
        held_errors = np.linalg.norm(projected - hold_img, axis=1)
        fit_stats = error_stats(solved["errors"])
        held_stats = error_stats(held_errors)
        row = {
            "excluded_id": excluded,
            "pnp_success": True,
            "fit_reprojection": fit_stats,
            "held_out_marker_reprojection": held_stats,
            "adopted": False,
        }
        rows.append(row)
        if held_stats["mean_px"] > 2.0 * fit_stats["mean_px"] and held_stats["mean_px"] > 1.5 * all_mean:
            isolated.append(excluded)
    return rows, isolated


def plane_points(depth, found, camera_matrix, transform_workboard_camera):
    mask = np.zeros(depth.shape, dtype=np.uint8)
    for quad in found.values():
        center = quad.mean(axis=0)
        expanded = center + (quad - center) * 1.8
        polygon = np.round(expanded).astype(np.int32).reshape(-1, 1, 2)
        cv2.fillPoly(mask, [polygon], 1)
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    rotation = transform_workboard_camera[:3, :3]
    translation = transform_workboard_camera[:3, 3]
    points = []
    u0, u1 = ROI_U
    v0, v1 = ROI_V
    for v in range(v0, v1 + 1):
        for u in range(u0, u1 + 1):
            if mask[v, u]:
                continue
            raw = int(depth[v, u])
            if raw <= 0:
                continue
            z_m = raw / 1000.0
            if z_m < 0.30 or z_m > 1.50:
                continue
            camera = np.array([(u - cx) * z_m / fx, (v - cy) * z_m / fy, z_m])
            points.append(rotation @ camera + translation)
    return np.asarray(points, dtype=np.float64)


def plane_component(points):
    z = points[:, 2]
    design = np.column_stack([np.ones(len(points)), points[:, 0], points[:, 1]])
    coeff, _residual, _rank, _s = np.linalg.lstsq(design, z, rcond=None)
    trend = coeff[1] * (points[:, 0] - points[:, 0].mean()) + coeff[2] * (points[:, 1] - points[:, 1].mean())
    offset_mm = abs(float(z.mean()) * 1000.0)
    tilt_rms_mm = float(np.std(trend) * 1000.0)
    if offset_mm >= PLANE_RATIO * tilt_rms_mm:
        dominant = "OFFSET"
    elif tilt_rms_mm >= PLANE_RATIO * offset_mm:
        dominant = "TILT"
    elif offset_mm >= PLANE_MIXED_MM and tilt_rms_mm >= PLANE_MIXED_MM:
        dominant = "MIXED"
    else:
        dominant = "INCONCLUSIVE"
    x_bins = binned(points[:, 0], z)
    y_bins = binned(points[:, 1], z)
    return {
        "point_count": int(len(points)),
        "mean_z_mm": float(z.mean() * 1000.0),
        "abs_z_mean_mm": float(np.mean(np.abs(z)) * 1000.0),
        "abs_z_p95_mm": float(np.percentile(np.abs(z), 95) * 1000.0),
        "linear_model": "z = a + b x + c y",
        "a_m": float(coeff[0]),
        "dz_dx": float(coeff[1]),
        "dz_dy": float(coeff[2]),
        "dz_dx_mm_per_m": float(coeff[1] * 1000.0),
        "dz_dy_mm_per_m": float(coeff[2] * 1000.0),
        "offset_abs_mean_mm": offset_mm,
        "tilt_rms_mm": tilt_rms_mm,
        "dominant_component": dominant,
        "rule": "OFFSET if |mean z| >= 2 * tilt RMS; TILT if tilt RMS >= 2 * |mean z|; MIXED if both >= 1 mm",
        "z_vs_x_bins": x_bins,
        "z_vs_y_bins": y_bins,
        "threshold_changed": False,
    }


def binned(axis, z, count=8):
    edges = np.linspace(axis.min(), axis.max(), count + 1)
    rows = []
    for start, stop in zip(edges[:-1], edges[1:]):
        choose = (axis >= start) & (axis <= stop if stop == edges[-1] else axis < stop)
        if not np.any(choose):
            continue
        rows.append({
            "axis_mean_m": float(axis[choose].mean()),
            "z_mean_mm": float(z[choose].mean() * 1000.0),
            "count": int(choose.sum()),
        })
    return rows


def draw_residuals(image, image_pts, projected):
    canvas = image.copy()
    for observed, point in zip(image_pts, projected):
        start = tuple(np.round(observed).astype(int))
        delta = (point - observed) * ARROW_SCALE
        end = tuple(np.round(observed + delta).astype(int))
        cv2.arrowedLine(canvas, start, end, (0, 200, 255), 1, tipLength=0.25)
        cv2.circle(canvas, start, 3, (0, 220, 0), -1)
        cv2.circle(canvas, tuple(np.round(point).astype(int)), 2, (0, 0, 255), -1)
    cv2.putText(
        canvas,
        "green observed, red projected, arrow x20",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return canvas


def draw_trends(plane):
    canvas = np.full((360, 900, 3), 255, dtype=np.uint8)
    draw_series(canvas, plane["z_vs_x_bins"], (40, 40, 420, 300), "z vs x")
    draw_series(canvas, plane["z_vs_y_bins"], (480, 40, 860, 300), "z vs y")
    return canvas


def draw_series(canvas, rows, box, title):
    x0, y0, x1, y1 = box
    cv2.rectangle(canvas, (x0, y0), (x1, y1), (0, 0, 0), 1)
    cv2.putText(canvas, title, (x0, y0 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
    if len(rows) < 2:
        return
    xs = np.array([row["axis_mean_m"] for row in rows])
    zs = np.array([row["z_mean_mm"] for row in rows])
    span_x = max(float(xs.max() - xs.min()), 1e-6)
    span_z = max(float(zs.max() - zs.min()), 1.0)
    pts = []
    for x_value, z_value in zip(xs, zs):
        px = int(x0 + 10 + (x_value - xs.min()) / span_x * (x1 - x0 - 20))
        py = int(y1 - 10 - (z_value - zs.min()) / span_z * (y1 - y0 - 20))
        pts.append((px, py))
    cv2.polylines(canvas, [np.array(pts, dtype=np.int32)], False, (40, 80, 200), 2)
    cv2.putText(
        canvas,
        f"{zs.min():.2f}..{zs.max():.2f} mm",
        (x0 + 8, y1 - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 0, 0),
        1,
        cv2.LINE_AA,
    )


def suspect(center_label, size_label_value, isolated):
    hits = []
    if center_label == "MATERIAL":
        hits.append("MEASURED_CENTER_METROLOGY")
    if size_label_value == "MATERIAL":
        hits.append("NOMINAL_MARKER_SIZE")
    if len(isolated) == 1:
        hits.append(f"MARKER_ID_{isolated[0]}_GEOMETRY")
    if len(hits) == 1:
        return hits[0], "INCONCLUSIVE", [
            "corner_localization",
            "intrinsic_distortion",
            "paper_flatness",
            "rgb_depth_correspondence",
            "physical_board_or_paper_height",
            "depth_bias",
        ]
    return "INCONCLUSIVE", "INCONCLUSIVE", [
        "installed_center_metrology",
        "nominal_print_size",
        "corner_localization",
        "intrinsic_distortion",
        "correspondence_convention",
        "rgb_depth_correspondence",
        "depth_bias",
        "roi_contamination",
        "physical_board_or_paper_height",
        "transform_error",
    ]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    intrinsics = load_json(SRC / "camera_intrinsics.json")
    camera_matrix = np.asarray(intrinsics["k"], dtype=np.float64)
    distortion = np.asarray(intrinsics["distortion_coefficients"], dtype=np.float64)
    image = cv2.imread(str(SRC / "detections" / "medoid_color.png"), cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit("medoid color가 없다.")
    detected = detect(image)
    saved = found_from_json(load_json(SRC / "detections" / "frame_07.json"))
    if sorted(detected) != [0, 1, 2, 3]:
        raise SystemExit("저장 이미지에서 ID 0~3이 다시 읽히지 않는다.")
    object_pts, image_pts, labels = stack(detected, MEASURED_M, HALF_NOMINAL_M, [0, 1, 2, 3])
    solved = solve(object_pts, image_pts, camera_matrix, distortion)
    if solved is None:
        raise SystemExit("진단 PnP가 실패했다.")
    mean_px = float(solved["errors"].mean())
    corner_source = "REDETECT_MEDOID_PNG"
    if abs(mean_px - STORED_MEAN_PX) > 0.02:
        object_pts, image_pts, labels = stack(saved, MEASURED_M, HALF_NOMINAL_M, [0, 1, 2, 3])
        solved = solve(object_pts, image_pts, camera_matrix, distortion)
        detected = saved
        corner_source = "SAVED_ROUNDED_CORNERS"
        mean_px = float(solved["errors"].mean())
    per_corner = []
    for label, observed, point, residual, error in zip(
        labels, image_pts, solved["projected"], solved["residual"], solved["errors"]
    ):
        per_corner.append({
            "marker_id": label["id"],
            "corner_index": label["corner"],
            "observed_u": float(observed[0]),
            "observed_v": float(observed[1]),
            "projected_u": float(point[0]),
            "projected_v": float(point[1]),
            "du": float(residual[0]),
            "dv": float(residual[1]),
            "error_px": float(error),
        })
    dump(OUT / "reprojection" / "per_corner.json", {
        "residual_definition": "projected - observed",
        "arrow": "observed to projected",
        "corner_source": corner_source,
        "mean_px_vs_w002e": {"recomputed": mean_px, "stored": STORED_MEAN_PX},
        "corners": per_corner,
    })
    blocks = marker_block(solved["residual"], solved["errors"], labels, image_pts, solved["projected"])
    principal = np.array([intrinsics["cx"], intrinsics["cy"]], dtype=np.float64)
    flags, correlation, angles = pattern_label(blocks, solved["errors"], image_pts, principal)
    pattern = "+".join(flags) if flags else "NO_SINGLE_PATTERN"
    dump(OUT / "reprojection" / "per_marker.json", {
        "markers": blocks,
        "pattern_flags": flags,
        "pattern": pattern,
        "radius_error_correlation": correlation,
        "centroid_angles_deg": angles,
        "hypothesis_map_not_a_verdict": {
            "A": "marker-wide same direction -> installed center",
            "B": "internal radial/scale -> marker size",
            "C": "peripheral growth -> intrinsic/distortion",
            "D": "irregular corners -> localization or flatness",
            "E": "similar global structure -> camera model or planar convention",
        },
    })
    overlay = draw_residuals(image, image_pts, solved["projected"])
    cv2.imwrite(str(OUT / "reprojection" / "residual_vectors.png"), overlay)

    rows, isolated = leave_one_out(detected, camera_matrix, distortion, mean_px)
    dump(OUT / "leave_one_marker_out.json", {
        "rows": rows,
        "isolated_ids": isolated,
        "removed_from_registration": False,
        "new_registration_candidate": False,
    })

    target_obj, target_img, _target_labels = stack(detected, TARGET_M, HALF_NOMINAL_M, [0, 1, 2, 3])
    target_solved = solve(target_obj, target_img, camera_matrix, distortion)
    measured_stats = error_stats(solved["errors"])
    target_stats = error_stats(target_solved["errors"])
    center_label, center_delta, center_improvement = sensitivity_label(
        measured_stats["mean_px"], target_stats["mean_px"])
    dump(OUT / "center_sensitivity.json", {
        "case_a": "W-002D-R1 measured centers",
        "case_b": "W-002B target centers",
        "case_a_reprojection": measured_stats,
        "case_b_reprojection": target_stats,
        "abs_mean_delta_px": center_delta,
        "case_b_improvement_px": center_improvement,
        "label": center_label,
        "rule": "MATERIAL if |mean difference| >= 0.30 px; LOW if < 0.15 px; else INCONCLUSIVE",
        "target_coordinates_adopted": False,
    })

    size_means = {}
    size_rows = []
    for size_mm in SIZE_MM:
        size_obj, size_img, _size_labels = stack(
            detected, MEASURED_M, size_mm / 2000.0, [0, 1, 2, 3])
        size_solved = solve(size_obj, size_img, camera_matrix, distortion)
        stats = error_stats(size_solved["errors"])
        size_means[size_mm] = stats["mean_px"]
        size_rows.append({"size_mm": size_mm, "reprojection": stats, "adopted": False})
    size_name, size_drop, size_span = size_label(size_means)
    dump(OUT / "marker_size_sensitivity.json", {
        "sizes_mm": size_rows,
        "nominal_mm": 60.0,
        "best_drop_from_60_px": size_drop,
        "max_abs_delta_from_60_px": size_span,
        "label": size_name,
        "rule": "MATERIAL if any size moves the mean by >= 0.30 px; LOW if all stay within 0.15 px",
        "print_size_measured": False,
        "size_adopted": False,
    })

    transform = np.asarray(
        load_json(SRC / "T_workboard_camera.json")["matrix"], dtype=np.float64)
    depth = np.load(SRC / "plane_validation" / "medoid_depth.npy")
    points = plane_points(depth, detected, camera_matrix, transform)
    plane = plane_component(points)
    (OUT / "plane_validation").mkdir(parents=True, exist_ok=True)
    np.save(OUT / "plane_validation" / "workboard_xyz.npy", points.astype(np.float32))
    dump(OUT / "plane_validation" / "structure.json", plane)
    cv2.imwrite(str(OUT / "plane_validation" / "z_trends.png"), draw_trends(plane))

    primary, secondary, unresolved = suspect(center_label, size_name, isolated)
    result = {
        "work_order": "W-002E-R1",
        "status": "PASS_DIAGNOSIS",
        "closed": True,
        "diagnosis_only": True,
        "w002e_status_immutable": "FAIL_REPROJECTION",
        "validated_registration_transform": None,
        "diagnostic_transform": "FAILED_GATE_DIAGNOSTIC_TRANSFORM",
        "diagnostic_transform_source": "W-002E medoid index 7",
        "reprojection_residual_pattern": pattern,
        "pattern_flags": flags,
        "radius_error_correlation": correlation,
        "per_marker": blocks,
        "leave_one_marker_out_isolated_ids": isolated,
        "center_geometry_sensitivity": center_label,
        "marker_size_sensitivity": size_name,
        "plane_residual_dominant_component": plane["dominant_component"],
        "rgb_failure_root_cause": "INCONCLUSIVE",
        "depth_failure_root_cause": "INCONCLUSIVE",
        "common_root_cause": False,
        "root_cause": "INCONCLUSIVE",
        "primary_suspect": primary,
        "secondary_suspect": secondary,
        "unresolved": unresolved,
        "print_size_measured": False,
        "markers_repositioned": False,
        "camera_moved": False,
        "robot_moved": False,
        "workboard_moved": False,
        "thresholds_changed": False,
        "solver_changed": False,
        "p009_r1_opened": False,
        "p010_opened": False,
        "next_gate_opened": False,
        "historical_evidence_modified": False,
        "corner_source": corner_source,
        "recomputed_mean_px": mean_px,
    }
    dump(OUT / "result.json", result)
    print("PATTERN", pattern)
    print("CENTER", center_label, measured_stats["mean_px"], target_stats["mean_px"])
    print("SIZE", size_name, size_means)
    print("PLANE", plane["dominant_component"], plane["mean_z_mm"], plane["tilt_rms_mm"], plane["abs_z_mean_mm"])
    print("ISOLATED", isolated)
    print("PRIMARY", primary)
    print("CORNER", corner_source, mean_px)


if __name__ == "__main__":
    main()
