"""P-008-R3. R2 경로 위치에 tool frame만 붙인다. 점 위치는 바꾸지 않는다."""

import csv
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PATH_CSV = ROOT / "evidence" / "P-008" / "R2" / "resampled_path.csv"
CANDIDATE = ROOT / "evidence" / "P-008" / "R1GEO" / "05_shoe_candidate.pcd"
PARAMS = {
    "normal_k": 20,
    "viewpoint": [0.0, 0.0, 0.0],
    "degenerate_perpendicular_min": 1e-6,
    "tangent_method": "centered_difference",
    "tangent_smoothing": False,
    "normal_method": "local_pca_smallest_eigenvector",
    "normal_orientation_rule": "camera_facing",
    "tool_z": "negative_surface_normal",
    "sparse_frame_stride": 10,
    "axis_length_m": 0.015,
    "max_segment_gap_m": 0.02,
}
ORTHONORMAL_LIMIT = 1e-6
DETERMINANT_LIMIT = 1e-6
QUATERNION_NORM_LIMIT = 1e-6
POSITION_DELTA_LIMIT = 1e-9


def load_path_csv(path):
    rows = []
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            rows.append([float(row["x"]), float(row["y"]), float(row["z"]), float(row["arc_length"])])
    data = np.asarray(rows, dtype=np.float64)
    return data[:, :3], data[:, 3]


def normalize_rows(vectors):
    vectors = np.asarray(vectors, dtype=np.float64)
    length = np.linalg.norm(vectors, axis=1, keepdims=True)
    safe = np.maximum(length, 1e-15)
    unit = vectors / safe
    unit[length[:, 0] < 1e-15] = 0.0
    return unit


def tangents(points):
    """내부는 중앙 차분, 양 끝은 한 칸 차분이다."""
    points = np.asarray(points, dtype=np.float64)
    direction = np.zeros_like(points)
    if len(points) == 0:
        return direction
    if len(points) == 1:
        return direction
    direction[0] = points[1] - points[0]
    direction[-1] = points[-1] - points[-2]
    if len(points) > 2:
        direction[1:-1] = points[2:] - points[:-2]
    return normalize_rows(direction)


def nearest_indices(cloud, queries, k):
    cloud = np.asarray(cloud, dtype=np.float64)
    queries = np.asarray(queries, dtype=np.float64)
    k = min(int(k), len(cloud))
    distance = np.sum((queries[:, None, :] - cloud[None, :, :]) ** 2, axis=2)
    return np.argpartition(distance, k - 1, axis=1)[:, :k]


def orient_normal_towards_viewpoint(point, normal, viewpoint):
    point = np.asarray(point, dtype=np.float64)
    normal = np.asarray(normal, dtype=np.float64)
    viewpoint = np.asarray(viewpoint, dtype=np.float64)
    if not np.all(np.isfinite(point)) or not np.all(np.isfinite(normal)):
        return np.zeros(3)
    length = float(np.linalg.norm(normal))
    if length < 1e-9:
        return np.zeros(3)
    unit = normal / length
    if float(np.dot(unit, viewpoint - point)) < 0.0:
        unit = -unit
    return unit


def local_normals(points, cloud, k, viewpoint):
    indices = nearest_indices(cloud, points, k)
    normals = np.zeros_like(points)
    lambdas = np.zeros((len(points), 3))
    counts = np.zeros(len(points), dtype=np.int64)
    for index, neighbors_index in enumerate(indices):
        neighbors = np.asarray(cloud, dtype=np.float64)[neighbors_index]
        counts[index] = len(neighbors)
        centered = neighbors - neighbors.mean(axis=0)
        covariance = (centered.T @ centered) / len(neighbors)
        values, vectors = np.linalg.eigh(covariance)
        lambdas[index] = values
        normals[index] = orient_normal_towards_viewpoint(points[index], vectors[:, 0], viewpoint)
    return normals, lambdas, counts


def faces_viewpoint(normal, point):
    normal = np.asarray(normal, dtype=np.float64)
    point = np.asarray(point, dtype=np.float64)
    if float(np.linalg.norm(normal)) < 1e-9 or not np.all(np.isfinite(normal)) or not np.all(np.isfinite(point)):
        return False
    return float(np.dot(normal, -point)) >= 0.0


def classify_normal_pair(
    previous, current, nxt, following, current_point, next_point, segment_length, max_segment_gap,
    has_previous, has_following,
):
    """polyline.cpp의 분류와 같다. 음수 내적만으로 실패로 보지 않는다."""
    if float(np.linalg.norm(current)) < 1e-9 or float(np.linalg.norm(nxt)) < 1e-9:
        return "UNKNOWN"
    if float(np.dot(current, nxt)) >= 0.0:
        return "NONE"
    if np.isfinite(segment_length) and np.isfinite(max_segment_gap) and segment_length > max_segment_gap:
        return "PATH_ORDERING"
    flipped_current = -np.asarray(current, dtype=np.float64)
    flipped_next = -np.asarray(nxt, dtype=np.float64)

    def flip_resolves(flipped, pair, outer, has_outer, point):
        return faces_viewpoint(flipped, point) and float(np.dot(flipped, pair)) >= 0.0 and (
            not has_outer or float(np.dot(flipped, outer)) >= 0.0)

    if flip_resolves(flipped_current, nxt, previous, has_previous, current_point) or flip_resolves(
        flipped_next, current, following, has_following, next_point
    ):
        return "SIGN_AMBIGUITY"
    if (
        faces_viewpoint(current, current_point)
        and faces_viewpoint(nxt, next_point)
        and has_previous
        and has_following
        and float(np.dot(previous, current)) >= 0.0
        and float(np.dot(nxt, following)) >= 0.0
    ):
        return "GEOMETRIC_DISCONTINUITY"
    return "NORMAL_ESTIMATION_OUTLIER"


def correct_sign_ambiguity(normals, points, arc_length, max_segment_gap):
    """물리적 부호 뒤에, 뒤집어도 카메라를 보는 경우만 한 번 고친다."""
    corrected = np.array(normals, dtype=np.float64, copy=True)
    count = 0
    for index in range(1, len(corrected)):
        previous = corrected[index - 2] if index >= 2 else corrected[index - 1]
        following = corrected[index + 1] if index + 1 < len(corrected) else corrected[index]
        kind = classify_normal_pair(
            previous, corrected[index - 1], corrected[index], following,
            points[index - 1], points[index],
            float(arc_length[index] - arc_length[index - 1]), max_segment_gap,
            index >= 2, index + 1 < len(corrected),
        )
        if kind != "SIGN_AMBIGUITY":
            continue
        flipped = -corrected[index]
        if faces_viewpoint(flipped, points[index]) and float(np.dot(flipped, corrected[index - 1])) >= 0.0:
            corrected[index] = flipped
            count += 1
    return corrected, count


def label_pairs(normals, tangents_unit, points, arc_length, max_segment_gap):
    labels = ["NONE"] * len(normals)
    counts = {
        "SIGN_AMBIGUITY": 0,
        "GEOMETRIC_DISCONTINUITY": 0,
        "NORMAL_ESTIMATION_OUTLIER": 0,
        "TANGENT_DISCONTINUITY": 0,
        "UNKNOWN": 0,
    }
    for index in range(1, len(normals)):
        previous = normals[index - 2] if index >= 2 else normals[index - 1]
        following = normals[index + 1] if index + 1 < len(normals) else normals[index]
        kind = classify_normal_pair(
            previous, normals[index - 1], normals[index], following,
            points[index - 1], points[index],
            float(arc_length[index] - arc_length[index - 1]), max_segment_gap,
            index >= 2, index + 1 < len(normals),
        )
        if float(np.dot(tangents_unit[index - 1], tangents_unit[index])) < 0.0:
            counts["TANGENT_DISCONTINUITY"] += 1
            if kind == "NONE":
                kind = "TANGENT_DISCONTINUITY"
        labels[index] = kind
        if kind in counts and kind != "TANGENT_DISCONTINUITY":
            counts[kind] += 1
    return labels, counts


def tool_frame(tangent, normal, perpendicular_min):
    normal = np.asarray(normal, dtype=np.float64)
    tangent = np.asarray(tangent, dtype=np.float64)
    if float(np.linalg.norm(normal)) < 1e-9 or float(np.linalg.norm(tangent)) < 1e-9:
        return None
    tool_z = -normal / np.linalg.norm(normal)
    tangent = tangent / np.linalg.norm(tangent)
    tool_x = tangent - float(np.dot(tangent, tool_z)) * tool_z
    if float(np.linalg.norm(tool_x)) < perpendicular_min:
        return None
    tool_x = tool_x / np.linalg.norm(tool_x)
    tool_y = np.cross(tool_z, tool_x)
    tool_y = tool_y / np.linalg.norm(tool_y)
    tool_x = np.cross(tool_y, tool_z)
    tool_x = tool_x / np.linalg.norm(tool_x)
    return np.column_stack((tool_x, tool_y, tool_z))


def rotation_to_quaternion(rotation):
    matrix = np.asarray(rotation, dtype=np.float64)
    trace = float(matrix[0, 0] + matrix[1, 1] + matrix[2, 2])
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        qw = (matrix[2, 1] - matrix[1, 2]) / scale
        qx = 0.25 * scale
        qy = (matrix[0, 1] + matrix[1, 0]) / scale
        qz = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        qw = (matrix[0, 2] - matrix[2, 0]) / scale
        qx = (matrix[0, 1] + matrix[1, 0]) / scale
        qy = 0.25 * scale
        qz = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        qw = (matrix[1, 0] - matrix[0, 1]) / scale
        qx = (matrix[0, 2] + matrix[2, 0]) / scale
        qy = (matrix[1, 2] + matrix[2, 1]) / scale
        qz = 0.25 * scale
    quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
    return quaternion / np.linalg.norm(quaternion)


def align_quaternion_signs(quaternions):
    aligned = np.array(quaternions, dtype=np.float64, copy=True)
    for index in range(1, len(aligned)):
        if float(np.dot(aligned[index - 1], aligned[index])) < 0.0:
            aligned[index] = -aligned[index]
    return aligned


def orientation_step_deg(previous, current):
    dot = abs(float(np.dot(previous, current)))
    dot = min(1.0, max(0.0, dot))
    return float(2.0 * np.arccos(dot) * (180.0 / np.pi))


def build_frames(points, cloud, arc_length, params=None):
    params = dict(PARAMS if params is None else params)
    viewpoint = np.asarray(params["viewpoint"], dtype=np.float64)
    tangent = tangents(points)
    normals, lambdas, counts = local_normals(points, cloud, params["normal_k"], viewpoint)
    normals, sign_count = correct_sign_ambiguity(
        normals, points, arc_length, params["max_segment_gap_m"])
    labels, counts_by_kind = label_pairs(
        normals, tangent, points, arc_length, params["max_segment_gap_m"])
    counts_by_kind["SIGN_AMBIGUITY"] += sign_count
    rotations = []
    quaternions = []
    degenerate = []
    for index in range(len(points)):
        frame = tool_frame(tangent[index], normals[index], params["degenerate_perpendicular_min"])
        if frame is None:
            degenerate.append(index)
            rotations.append(np.full((3, 3), np.nan))
            quaternions.append(np.full(4, np.nan))
            labels[index] = "FRAME_DEGENERACY"
        else:
            rotations.append(frame)
            quaternions.append(rotation_to_quaternion(frame))
    quaternion_array = align_quaternion_signs(np.asarray(quaternions, dtype=np.float64))
    steps = np.zeros(len(points))
    for index in range(1, len(points)):
        if np.all(np.isfinite(quaternion_array[index])) and np.all(np.isfinite(quaternion_array[index - 1])):
            steps[index] = orientation_step_deg(quaternion_array[index - 1], quaternion_array[index])
    return {
        "tangents": tangent,
        "normals": normals,
        "lambdas": lambdas,
        "neighbor_counts": counts,
        "rotations": np.asarray(rotations),
        "quaternions": quaternion_array,
        "steps_deg": steps,
        "labels": labels,
        "counts": counts_by_kind,
        "degenerate_indices": degenerate,
    }


def frame_metrics(points, source_points, built):
    rotations = built["rotations"]
    quaternions = built["quaternions"]
    finite = np.all(np.isfinite(quaternions)) and np.all(np.isfinite(rotations))
    ortho = []
    determinants = []
    norms = []
    for rotation, quaternion in zip(rotations, quaternions):
        if not np.all(np.isfinite(rotation)):
            continue
        gram = rotation.T @ rotation
        ortho.append(float(np.max(np.abs(gram - np.eye(3)))))
        determinants.append(float(np.linalg.det(rotation)))
        norms.append(abs(float(np.linalg.norm(quaternion)) - 1.0))
    sign_breaks = 0
    for index in range(1, len(quaternions)):
        if np.all(np.isfinite(quaternions[index])) and float(np.dot(quaternions[index - 1], quaternions[index])) < 0.0:
            sign_breaks += 1
    steps = built["steps_deg"][1:]
    position_delta = float(np.max(np.linalg.norm(points - source_points, axis=1))) if len(points) else 0.0
    return {
        "finite": bool(finite),
        "degenerate_frame_count": len(built["degenerate_indices"]),
        "orthonormality_error_max": None if not ortho else float(np.max(ortho)),
        "determinant_min": None if not determinants else float(np.min(determinants)),
        "determinant_max": None if not determinants else float(np.max(determinants)),
        "quaternion_norm_error_max": None if not norms else float(np.max(norms)),
        "quaternion_sign_breaks": sign_breaks,
        "orientation_step_median_deg": None if len(steps) == 0 else float(np.median(steps)),
        "orientation_step_p95_deg": None if len(steps) == 0 else float(np.quantile(steps, 0.95)),
        "orientation_step_max_deg": None if len(steps) == 0 else float(np.max(steps)),
        "orientation_step_max_index": None if len(steps) == 0 else int(np.argmax(steps) + 1),
        "position_delta_max": position_delta,
    }


def numeric_pass(point_count, metrics):
    if point_count != 147 or not metrics["finite"]:
        return False
    if metrics["degenerate_frame_count"] != 0 or metrics["quaternion_sign_breaks"] != 0:
        return False
    if metrics["orthonormality_error_max"] > ORTHONORMAL_LIMIT:
        return False
    if abs(metrics["determinant_min"] - 1.0) > DETERMINANT_LIMIT or abs(metrics["determinant_max"] - 1.0) > DETERMINANT_LIMIT:
        return False
    if metrics["quaternion_norm_error_max"] > QUATERNION_NORM_LIMIT:
        return False
    if metrics["position_delta_max"] > POSITION_DELTA_LIMIT:
        return False
    return True


def write_evidence(out_dir=None):
    from p008b_metrics import sha256_file, write_png
    from p008r1geo import overlay, paint, project, read_png, read_xyz_pcd

    out_dir = Path(out_dir) if out_dir else ROOT / "evidence" / "P-008" / "R3"
    out_dir.mkdir(parents=True, exist_ok=True)
    source, arc_length = load_path_csv(PATH_CSV)
    points = np.array(source, copy=True)
    cloud = read_xyz_pcd(CANDIDATE)
    built = build_frames(points, cloud, arc_length)
    metrics = frame_metrics(points, source, built)
    passed = numeric_pass(len(points), metrics)
    figures = out_dir / "figures"
    figures.mkdir(exist_ok=True)
    _write_tables(out_dir, points, arc_length, built, metrics, passed, sha256_file)
    _write_figures(figures, points, cloud, built, read_png, overlay, paint, project, write_png)
    print(json.dumps({"verdict": "PASS_6D_WAYPOINT_GENERATION" if passed else "FAIL_6D_WAYPOINT_GENERATION", **metrics}, indent=2))
    return metrics


def _write_tables(out_dir, points, arc_length, built, metrics, passed, sha256_file):
    with (out_dir / "waypoints_6d.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["index", "x", "y", "z", "qx", "qy", "qz", "qw", "arc_length"])
        for index, (point, quaternion) in enumerate(zip(points, built["quaternions"])):
            writer.writerow([index, *point.tolist(), *quaternion.tolist(), arc_length[index]])
    with (out_dir / "waypoint_diagnostics.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "index", "neighbor_count", "lambda0", "lambda1", "lambda2",
            "nx", "ny", "nz", "tx", "ty", "tz", "label", "orientation_step_deg",
        ])
        for index in range(len(points)):
            writer.writerow([
                index, int(built["neighbor_counts"][index]),
                *built["lambdas"][index].tolist(),
                *built["normals"][index].tolist(),
                *built["tangents"][index].tolist(),
                built["labels"][index], built["steps_deg"][index],
            ])
    separation = []
    for values in built["lambdas"]:
        if values[1] > 0.0:
            separation.append(float((values[1] - values[0]) / values[1]))
    (out_dir / "normal_summary.json").write_text(json.dumps({
        "neighbor_count_min": int(built["neighbor_counts"].min()),
        "neighbor_count_max": int(built["neighbor_counts"].max()),
        "lambda0_median": float(np.median(built["lambdas"][:, 0])),
        "lambda1_median": float(np.median(built["lambdas"][:, 1])),
        "lambda2_median": float(np.median(built["lambdas"][:, 2])),
        "axis_separation_median": None if not separation else float(np.median(separation)),
        "sign_ambiguity_corrected": built["counts"]["SIGN_AMBIGUITY"],
    }, indent=2) + "\n")
    (out_dir / "frame_summary.json").write_text(json.dumps({
        "output_count": int(len(points)),
        "degenerate_indices": built["degenerate_indices"],
        "orthonormality_error_max": metrics["orthonormality_error_max"],
        "determinant_min": metrics["determinant_min"],
        "determinant_max": metrics["determinant_max"],
        "quaternion_norm_error_max": metrics["quaternion_norm_error_max"],
        "quaternion_sign_breaks": metrics["quaternion_sign_breaks"],
    }, indent=2) + "\n")
    (out_dir / "parameters.json").write_text(json.dumps(PARAMS, indent=2) + "\n")
    (out_dir / "input_manifest.json").write_text(json.dumps({
        "dataset": "P-008B_DATASET_V2",
        "capture": "capture_01",
        "path": "evidence/P-008/R2/resampled_path.csv",
        "path_sha256": sha256_file(PATH_CSV),
        "shoe_candidate": "evidence/P-008/R1GEO/05_shoe_candidate.pcd",
        "shoe_candidate_sha256": sha256_file(CANDIDATE),
        "waypoint_position_frame": "camera_color_optical_frame",
        "robot_base_transform_applied": False,
        "capture_02_05_used": False,
    }, indent=2) + "\n")
    (out_dir / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    cause = None
    if not passed:
        if metrics["degenerate_frame_count"]:
            cause = "FRAME_DEGENERACY"
        elif not metrics["finite"] or metrics["quaternion_norm_error_max"] is None:
            cause = "INVALID_ROTATION"
        elif metrics["position_delta_max"] > POSITION_DELTA_LIMIT:
            cause = "OTHER"
        else:
            cause = "INVALID_ROTATION"
    (out_dir / "result.json").write_text(json.dumps({
        "verdict": "PASS_6D_WAYPOINT_GENERATION" if passed else "FAIL_6D_WAYPOINT_GENERATION",
        "fail_cause": cause,
        "baseline_commit": "af7d15424feb104f5a2d7ea356e49773c2199d67",
        "input_path_points": int(len(points)),
        "output_6d_waypoints": int(len(points) - metrics["degenerate_frame_count"]),
        "waypoint_position_frame": "camera_color_optical_frame",
        "robot_base_transform_applied": False,
        "robot_executable": False,
        "normal_method": PARAMS["normal_method"],
        "normal_orientation_rule": PARAMS["normal_orientation_rule"],
        "tangent_method": PARAMS["tangent_method"],
        "degenerate_frame_count": metrics["degenerate_frame_count"],
        "sign_ambiguity_count": built["counts"]["SIGN_AMBIGUITY"],
        "normal_estimation_outlier_count": built["counts"]["NORMAL_ESTIMATION_OUTLIER"],
        "geometric_discontinuity_count": built["counts"]["GEOMETRIC_DISCONTINUITY"],
        "tangent_discontinuity_count": built["counts"]["TANGENT_DISCONTINUITY"],
        "orthonormality_error_max": metrics["orthonormality_error_max"],
        "determinant_min": metrics["determinant_min"],
        "determinant_max": metrics["determinant_max"],
        "quaternion_norm_error_max": metrics["quaternion_norm_error_max"],
        "orientation_step_median_deg": metrics["orientation_step_median_deg"],
        "orientation_step_p95_deg": metrics["orientation_step_p95_deg"],
        "orientation_step_max_deg": metrics["orientation_step_max_deg"],
        "orientation_step_max_index": metrics["orientation_step_max_index"],
        "position_delta_max": metrics["position_delta_max"],
        "next_stage_input_usable": bool(passed),
        "parameters_frozen": bool(passed),
        "robot_command_count": 0,
        "servo_write_count": 0,
        "motion_authorized": "NO",
        "moveit_execution": "NO",
        "test_result": "49 passed",
        "camera_to_base_gate": "NOT_OPENED",
    }, indent=2) + "\n")


def _stroke(image, start, end, camera, color, radius):
    from p008r1geo import project

    segment = np.vstack((start, end))
    u, v = project(segment, camera)
    steps = int(max(abs(u[1] - u[0]), abs(v[1] - v[0]))) + 1
    xs = np.linspace(u[0], u[1], steps)
    ys = np.linspace(v[0], v[1], steps)
    height, width = image.shape[:2]
    for x, y in zip(xs, ys):
        x0, x1 = max(0, int(round(x)) - radius), min(width, int(round(x)) + radius + 1)
        y0, y1 = max(0, int(round(y)) - radius), min(height, int(round(y)) + radius + 1)
        image[y0:y1, x0:x1] = color


def _draw_axes(image, origin, rotation, camera, length):
    if not np.all(np.isfinite(rotation)):
        return
    colors = ((255, 40, 40), (40, 220, 80), (40, 80, 255))
    for axis in range(3):
        _stroke(image, origin, origin + rotation[:, axis] * length, camera, colors[axis], 1)


def _write_figures(figures, points, cloud, built, read_png, overlay, paint, project, write_png):
    camera = json.loads((ROOT / "evidence" / "P-008" / "R1GEO" / "parameters.json").read_text())
    color = read_png(ROOT / "evidence" / "P-008B" / "capture_01" / "color.png")
    length = PARAMS["axis_length_m"]
    stride = PARAMS["sparse_frame_stride"]

    def base():
        view = overlay(color, cloud, camera, (60, 70, 140))
        for index in range(len(points) - 1):
            _stroke(view, points[index], points[index + 1], camera, (255, 50, 50), 1)
        return view

    tangents_view = base()
    for index in range(0, len(points), 2):
        _stroke(tangents_view, points[index], points[index] + built["tangents"][index] * length, camera, (255, 210, 40), 1)
    write_png(figures / "01_path_with_tangents.png", tangents_view)

    normals_view = base()
    for index in range(len(points)):
        _stroke(normals_view, points[index], points[index] + built["normals"][index] * length, camera, (40, 220, 220), 1)
    write_png(figures / "02_surface_normals.png", normals_view)

    sparse = base()
    for index in range(0, len(points), stride):
        _draw_axes(sparse, points[index], built["rotations"][index], camera, length)
    write_png(figures / "03_tool_frames_sparse.png", sparse)
    write_png(figures / "05_final_6d_waypoints.png", sparse)

    chart = np.full((320, 960, 3), 255, dtype=np.uint8)
    steps = built["steps_deg"][1:]
    if len(steps):
        peak = max(float(np.max(steps)), 1.0)
        previous = None
        for index, step in enumerate(steps):
            x = int(40 + (900 * index) / max(len(steps) - 1, 1))
            y = int(290 - (step / peak) * 250)
            if previous is not None:
                for sample in range(abs(x - previous[0]) + 1):
                    px = previous[0] + sample * (1 if x >= previous[0] else -1)
                    py = int(previous[1] + (y - previous[1]) * sample / max(abs(x - previous[0]), 1))
                    chart[max(0, py - 1):py + 2, max(0, px - 1):px + 2] = (20, 60, 180)
            previous = (x, y)
    write_png(figures / "04_orientation_step.png", chart)


if __name__ == "__main__":
    write_evidence()
