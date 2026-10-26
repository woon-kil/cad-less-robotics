#!/usr/bin/env python3
"""REG 샘플로 T_workplane_camera_depth를 구하고 VAL 샘플로만 검증한다."""

import csv
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p006c_r25_geometry as geometry

def vision():
    import cv2
    import p006c_validate_metric as metric
    return cv2, metric


ROOT = Path("evidence/P-006C-R2.5")
CAPTURES = ROOT / "captures"
SENSOR = Path("evidence/P-006/camera-target/thresholds.yaml")
UP = np.array([0.0, 0.0, 1.0], dtype=np.float64)


def load_json(name):
    return json.loads((ROOT / name).read_text())


def write_json(name, payload):
    (ROOT / name).write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")


def is_identity(transform):
    if not transform:
        return False
    translation = np.array(transform["translation_m"], dtype=np.float64)
    quaternion = np.array(transform["quaternion_xyzw"], dtype=np.float64)
    return float(np.linalg.norm(translation)) < 1e-6 and abs(float(quaternion[3])) > 0.999999


def depth_transform(record):
    _, metric = vision()
    transform = record.get("T_depth_color")
    if not transform or is_identity(transform) or record.get("tf_reason") != "ok":
        return None
    if transform.get("parent") != "camera_depth_optical_frame":
        return None
    if transform.get("child") != "camera_color_optical_frame":
        return None
    return geometry.se3(
        metric.quaternion_to_matrix(transform["quaternion_xyzw"]),
        transform["translation_m"],
    )


def valid_corner_count(record, board_in_depth, thresholds):
    cv2, metric = vision()
    depth_path = CAPTURES / f"{record['stem']}-depth.png"
    if not depth_path.is_file():
        return 0
    depth_image = metric.read_png(depth_path)
    if depth_image.ndim != 2:
        return 0
    matrix = metric.camera_matrix(record["depth_info"])
    distortion = metric.dist_coeffs(record["depth_info"])
    projected, _ = cv2.projectPoints(board_in_depth, np.zeros(3), np.zeros(3), matrix, distortion)
    height, width = depth_image.shape[:2]
    scale = float(thresholds["depth_unit"]["meters_per_count"])
    count = 0
    for u, v in projected.reshape(-1, 2):
        ui, vi = int(round(float(u))), int(round(float(v)))
        if ui < 2 or vi < 2 or ui >= width - 2 or vi >= height - 2:
            continue
        window = depth_image[vi - 2:vi + 3, ui - 2:ui + 3].astype(np.float64)
        good = window[window > 0]
        if good.size < 5:
            continue
        depth_m = float(np.median(good)) * scale
        if 0.17 <= depth_m <= 20.0:
            count += 1
    return count


def board_depth_z(record, board_in_depth, workplane_depth, expected_z, thresholds):
    cv2, metric = vision()
    depth_image = metric.read_png(CAPTURES / f"{record['stem']}-depth.png")
    if depth_image.ndim != 2:
        return []
    matrix = metric.camera_matrix(record["depth_info"])
    distortion = metric.dist_coeffs(record["depth_info"])
    projected, _ = cv2.projectPoints(board_in_depth, np.zeros(3), np.zeros(3), matrix, distortion)
    uv = np.round(projected.reshape(-1, 2)).astype(np.int32)
    height, width = depth_image.shape[:2]
    polygon = uv[[0, record["cols"] - 1, record["cols"] * record["rows"] - 1, (record["rows"] - 1) * record["cols"]]]
    mask = np.zeros((height, width), dtype=np.uint8)
    if not np.all((polygon[:, 0] >= 0) & (polygon[:, 0] < width) & (polygon[:, 1] >= 0) & (polygon[:, 1] < height)):
        return []
    cv2.fillPoly(mask, [polygon], 1)
    ys, xs = np.where(mask > 0)
    scale = float(thresholds["depth_unit"]["meters_per_count"])
    errors = []
    for u, v in zip(xs[::2], ys[::2]):
        raw = int(depth_image[v, u])
        if raw <= 0:
            continue
        depth_m = raw * scale
        if not (0.17 <= depth_m <= 20.0):
            continue
        point = metric.backproject(np.array([[u, v]], dtype=np.float64), np.array([depth_m]), matrix, distortion)[0]
        workplane = (workplane_depth @ np.array([point[0], point[1], point[2], 1.0]))[:3]
        errors.append(float(workplane[2] - expected_z))
    return errors


def frame_pose(record, sensor, anchor_xyz, yaw_deg):
    cv2, metric = vision()
    row = {
        "stem": record["stem"],
        "semantic_position": record["semantic_position"],
        "rejection_reason": "",
        "color_frame_id": record.get("color_frame_id"),
        "depth_frame_id": record.get("depth_frame_id"),
        "tf_reason": record.get("tf_reason"),
    }
    if record.get("depth_frame_id") != "camera_depth_optical_frame":
        row["rejection_reason"] = "depth_frame"
        return row
    color = metric.read_png(CAPTURES / f"{record['stem']}-color.png")
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY) if color.ndim == 3 else color
    found = metric.detect(gray, sensor)
    if found is None:
        row["rejection_reason"] = "checkerboard_not_found"
        return row
    cols, rows, corners = found
    count = int(corners.shape[0])
    row["internal_corner_count"] = count
    if count != 54:
        row["rejection_reason"] = "corner_count"
        return row
    ok, rvec, tvec = cv2.solvePnP(
        metric.object_points(cols, rows),
        corners,
        metric.camera_matrix(record["color_info"]),
        metric.dist_coeffs(record["color_info"]),
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok or float(tvec.reshape(3)[2]) <= 0.0:
        row["rejection_reason"] = "pose_rejected"
        return row
    rotation, _ = cv2.Rodrigues(rvec)
    if float(np.linalg.det(rotation)) <= 0.0:
        row["rejection_reason"] = "reflection"
        return row
    color_board = geometry.se3(rotation, tvec.reshape(3))
    depth_color = depth_transform(record)
    if record.get("tf_reason") == "identity_rejected" or is_identity(record.get("T_depth_color")):
        row["rejection_reason"] = "identity_rejected"
        return row
    if depth_color is None:
        row["rejection_reason"] = "lookup_failed"
        return row
    depth_board = depth_color @ color_board
    record["cols"] = cols
    record["rows"] = rows
    corners_in_depth = (depth_board @ np.concatenate([metric.object_points(cols, rows), np.ones((54, 1))], axis=1).T).T[:, :3]
    valid_corners = valid_corner_count(record, corners_in_depth, sensor)
    row["valid_depth_corner_count"] = valid_corners
    row["depth_info"] = record["depth_info"]
    if valid_corners < int(sensor["depth_plane"]["valid_corner_depths_min"]):
        row["rejection_reason"] = "depth_corners"
        return row
    workplane_board = geometry.workplane_from_board(anchor_xyz, yaw_deg)
    transform = geometry.workplane_from_depth(workplane_board, depth_board)
    row["transform"] = transform
    row["depth_board"] = depth_board
    row["board_in_depth"] = corners_in_depth
    row["corners"] = corners
    row["cols"] = cols
    row["rows"] = rows
    return row


def placement_transform(frames):
    rotations = [frame["transform"][:3, :3] for frame in frames]
    translations = [frame["transform"][:3, 3] for frame in frames]
    rotation = geometry.mean_rotation(rotations)
    translation = geometry.mean_translation(translations)
    return geometry.se3(rotation, translation)


def transform_document(transform, reg_count, val_count, freeze_commit, source_hashes):
    _, metric = vision()
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    quaternion = metric.rotation_to_quaternion(rotation)
    return {
        "status": "COMPUTED",
        "name": "T_workplane_camera_depth",
        "source_frame": "camera_depth_optical_frame",
        "target_frame": "workplane_roi_frame",
        "direction": "p_workplane = T_workplane_camera_depth * p_camera_depth",
        "matrix": transform.tolist(),
        "translation_m": [float(value) for value in translation],
        "quaternion_xyzw": quaternion,
        "rotation_matrix": rotation.tolist(),
        "determinant": float(np.linalg.det(rotation)),
        "orthogonality_error": geometry.orthogonality_error(rotation),
        "inverse_consistency_error": geometry.inverse_consistency_error(transform),
        "scale_policy": "rigid_fixed_1.0",
        "similarity_used": False,
        "reflection_flipped": False,
        "registration_sample_count": reg_count,
        "validation_sample_count": val_count,
        "threshold_freeze_commit": freeze_commit,
        "source_evidence_hashes": source_hashes,
        "T_camera_depth_workplane": geometry.invert(transform).tolist(),
        "T_base_camera": None,
    }


def draw_overlay(color, corners, cols, rows):
    cv2, _ = vision()
    draw = color.copy()
    grid = corners.reshape(rows, cols, 2)
    origin = tuple(np.round(grid[0, 0]).astype(int))
    plus_x = tuple(np.round(grid[0, cols - 1]).astype(int))
    plus_y = tuple(np.round(grid[rows - 1, 0]).astype(int))
    cv2.arrowedLine(draw, origin, plus_x, (0, 0, 255), 2, tipLength=0.08)
    cv2.arrowedLine(draw, origin, plus_y, (0, 255, 0), 2, tipLength=0.05)
    cv2.circle(draw, origin, 10, (0, 0, 255), 2)
    cv2.putText(draw, "origin +X", (origin[0] + 14, max(24, origin[1] - 14)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.putText(draw, "+Y", (plus_y[0] + 8, plus_y[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 180, 0), 2)
    return draw


def save_overlay(record, row):
    cv2, metric = vision()
    color = metric.read_png(CAPTURES / f"{record['stem']}-color.png")
    image = draw_overlay(color, row["corners"], row["cols"], row["rows"])
    out = ROOT / "overlays" / f"{record['stem']}-axes.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imencode(".png", image)[1].tofile(str(out))


def summarize_errors(values):
    array = np.asarray(values, dtype=np.float64)
    if array.size == 0:
        return {"count": 0, "rmse": None, "p95": None, "max_abs": None, "mean": None}
    return {
        "count": int(array.size),
        "rmse": float(np.sqrt(np.mean(array ** 2))),
        "p95": float(np.percentile(np.abs(array), 95)),
        "max_abs": float(np.max(np.abs(array))),
        "mean": float(np.mean(array)),
    }


def evaluate():
    _, metric = vision()
    if not geometry.sensor_thresholds_unchanged():
        raise SystemExit("sensor threshold가 바뀌었다")
    sensor = metric.load_yaml(SENSOR)
    rules = load_json("thresholds.json")
    anchors = load_json("anchor_definition.json")
    freeze = load_json("threshold_freeze.json")
    problems = geometry.anchor_problems(anchors, rules["placement_preconditions"])
    if problems or anchors.get("coordinates_recorded") is not True or not freeze.get("commit"):
        raise SystemExit("앵커 또는 threshold freeze가 없다")
    yaw = float(anchors["alignment"]["yaw_deg"])
    records = [json.loads(path.read_text()) for path in sorted(CAPTURES.glob("*.json"))]
    grouped = {name: [] for name in geometry.ALL_IDS}
    rejected = []
    for record in records:
        name = record.get("semantic_position")
        anchor = geometry.point_xyz((anchors.get("anchors") or {}).get(name))
        if name not in grouped or anchor is None:
            rejected.append({"stem": record.get("stem"), "rejection_reason": "unknown_position"})
            continue
        row = frame_pose(record, sensor, anchor, yaw)
        if row["rejection_reason"]:
            rejected.append(row)
            continue
        row["anchor_m"] = anchor
        grouped[name].append(row)
        save_overlay(record, row)
    minimum = int(rules["placement_preconditions"]["min_accepted_frames"])
    reg_transforms = []
    for name in geometry.REG_IDS:
        if len(grouped[name]) < minimum:
            rejected.append({"semantic_position": name, "rejection_reason": "too_few_frames", "accepted": len(grouped[name])})
            continue
        reg_transforms.append(placement_transform(grouped[name]))
    if len(reg_transforms) != 3:
        write_json("transform.json", {"status": "NOT_COMPUTED", "T_base_camera": None, "reason": "registration_incomplete"})
        print("BLOCKED")
        return
    published = placement_transform([{"transform": item} for item in reg_transforms])
    rotation = published[:3, :3]
    reflection = float(np.linalg.det(rotation)) < 0.0
    source_hashes = load_json("threshold_freeze.json").get("source_evidence_hashes")
    val_count = sum(len(grouped[name]) for name in geometry.VAL_REQUIRED + geometry.VAL_OPTIONAL)
    document = transform_document(published, 3, val_count, freeze["commit"], source_hashes)
    document["reflection_detected"] = reflection
    spreads_t = [float(np.linalg.norm(item[:3, 3] - published[:3, 3])) for item in reg_transforms]
    spreads_r = [geometry.geodesic_deg(rotation, item[:3, :3]) for item in reg_transforms]
    position_errors = []
    z_errors = []
    angles = []
    csv_rows = []
    for name in geometry.VAL_REQUIRED + geometry.VAL_OPTIONAL:
        for row in grouped[name]:
            origin_depth = row["depth_board"][:3, 3]
            measured = (published @ np.array([origin_depth[0], origin_depth[1], origin_depth[2], 1.0]))[:3]
            position_error = float(np.linalg.norm(measured - np.asarray(row["anchor_m"])))
            normal = (published @ row["depth_board"])[:3, 2]
            angle = geometry.directed_angle_deg(normal, UP)
            depth_errors = board_depth_z(row, row["board_in_depth"], published, row["anchor_m"][2], sensor)
            position_errors.append(position_error)
            z_errors.extend(depth_errors)
            angles.append(angle)
            csv_rows.append({
                "split": "VAL",
                "semantic_position": name,
                "stem": row["stem"],
                "rejection_reason": "",
                "position_error_m": position_error,
                "plane_z_samples": len(depth_errors),
                "normal_error_deg": angle,
            })
    for item in rejected:
        csv_rows.append({
            "split": "REJECTED",
            "semantic_position": item.get("semantic_position", ""),
            "stem": item.get("stem", ""),
            "rejection_reason": item.get("rejection_reason", ""),
            "position_error_m": "",
            "plane_z_samples": "",
            "normal_error_deg": "",
        })
    position = summarize_errors(position_errors)
    plane = summarize_errors(z_errors)
    orientation = {
        "count": len(angles),
        "mean": None if not angles else float(np.mean(angles)),
        "max": None if not angles else float(np.max(angles)),
    }
    numerical = rules["numerical"]
    consistency = rules["registration_consistency"]
    validation = rules["validation"]
    checks = {
        "rigid": document["scale_policy"] == "rigid_fixed_1.0" and document["similarity_used"] is False,
        "scale": True,
        "reflection": not reflection and document["determinant"] > 0.0,
        "determinant": abs(document["determinant"] - 1.0) <= float(numerical["determinant_abs_error_max"]),
        "orthogonality": document["orthogonality_error"] <= float(numerical["orthogonality_max_abs"]),
        "inverse": document["inverse_consistency_error"] <= float(numerical["inverse_consistency_max_abs"]),
        "finite": geometry.finite_vector(document["translation_m"]) and geometry.finite_vector(document["quaternion_xyzw"]),
        "translation_spread": max(spreads_t) <= float(consistency["translation_spread_m_max"]),
        "rotation_spread": max(spreads_r) <= float(consistency["rotation_spread_deg_max"]),
        "position_rmse": position["rmse"] is not None and position["rmse"] <= float(validation["position_rmse_m_max"]),
        "position_p95": position["p95"] is not None and position["p95"] <= float(validation["position_p95_m_max"]),
        "position_max": position["max_abs"] is not None and position["max_abs"] <= float(validation["position_max_m_max"]),
        "plane_rmse": plane["rmse"] is not None and plane["rmse"] <= float(validation["plane_z_rmse_m_max"]),
        "plane_max": plane["max_abs"] is not None and plane["max_abs"] <= float(validation["plane_z_abs_max_m"]),
        "orientation_mean": orientation["mean"] is not None and orientation["mean"] <= float(validation["normal_mean_deg_max"]),
        "orientation_max": orientation["max"] is not None and orientation["max"] <= float(validation["normal_max_deg_max"]),
        "val_coverage": all(len(grouped[name]) >= minimum for name in geometry.VAL_REQUIRED),
    }
    status = "PASS" if all(checks.values()) else "BLOCKED"
    document["registration_translation_spread_m"] = spreads_t
    document["registration_rotation_spread_deg"] = spreads_r
    def public_frame(row):
        return {
            "stem": row["stem"],
            "semantic_position": row["semantic_position"],
            "valid_depth_corner_count": row.get("valid_depth_corner_count"),
            "anchor_m": row.get("anchor_m"),
        }
    write_json("registration_samples.json", {
        "status": "CAPTURED",
        "used_in_fit": True,
        "samples": {name: [public_frame(row) for row in grouped[name]] for name in geometry.REG_IDS},
    })
    write_json("validation_samples.json", {
        "status": "CAPTURED",
        "used_in_fit": False,
        "samples": {name: [public_frame(row) for row in grouped[name]] for name in geometry.VAL_REQUIRED + geometry.VAL_OPTIONAL},
    })
    write_json("transform.json", document)
    write_json("metrics.json", {"status": status, "checks": checks, "position": position, "plane": plane, "orientation": orientation, "rejected": rejected})
    with (ROOT / "results.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["split", "semantic_position", "stem", "rejection_reason", "position_error_m", "plane_z_samples", "normal_error_deg"])
        writer.writeheader()
        writer.writerows(csv_rows)
    print(status)


def illustrate(source):
    """기존 이미지를 읽기만 해서 코너 원점을 그린다. workplane 좌표는 만들지 않는다."""
    cv2, metric = vision()
    sensor = metric.load_yaml(SENSOR)
    color = metric.read_png(Path(source))
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY) if color.ndim == 3 else color
    found = metric.detect(gray, sensor)
    out_dir = ROOT / "overlays"
    out_dir.mkdir(parents=True, exist_ok=True)
    note = {
        "source": str(source),
        "role": "CONVENTION_ILLUSTRATION_NOT_AN_ANCHOR",
        "workplane_xy": "UNRESOLVED",
        "r2_evidence_modified": False,
    }
    if found is None:
        note["detected"] = False
        write_json_path(out_dir / "convention-illustration.json", note)
        print("ILLUSTRATION_NOT_DETECTED")
        return
    cols, rows, corners = found
    image = draw_overlay(color, corners, cols, rows)
    cv2.imencode(".png", image)[1].tofile(str(out_dir / "convention-illustration.png"))
    note.update({"detected": True, "cols": cols, "rows": rows, "origin": "corner 0 after canonicalize"})
    write_json_path(out_dir / "convention-illustration.json", note)
    print("ILLUSTRATION_WRITTEN")


def write_json_path(path, payload):
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--self-check":
        geometry.self_check()
        return
    if len(sys.argv) > 2 and sys.argv[1] == "--illustrate":
        illustrate(sys.argv[2])
        return
    if not geometry.sensor_thresholds_unchanged():
        raise SystemExit("sensor threshold가 바뀌었다")
    rules = load_json("thresholds.json")
    anchors = load_json("anchor_definition.json")
    problems = geometry.anchor_problems(anchors, rules["placement_preconditions"])
    if problems or anchors.get("coordinates_recorded") is not True:
        print("OPERATOR_ANCHOR_REQUIRED")
        for item in problems:
            print(item)
        raise SystemExit(2)
    freeze = load_json("threshold_freeze.json")
    if not freeze.get("commit"):
        raise SystemExit("threshold freeze commit이 없다")
    if not list(CAPTURES.glob("*.json")):
        print("IMPLEMENTED_NOT_VERIFIED")
        return
    evaluate()


if __name__ == "__main__":
    main()
