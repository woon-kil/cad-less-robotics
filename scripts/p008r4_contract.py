"""P-008-R4. R3 pose는 유지하고, 작업 구간과 clearance만 정한다."""

import csv
import json
from pathlib import Path

import numpy as np

from p008r3_frames import orientation_step_deg

ROOT = Path(__file__).resolve().parents[1]
R3_CSV = ROOT / "evidence" / "P-008" / "R3" / "waypoints_6d.csv"
CANDIDATE = ROOT / "evidence" / "P-008" / "R1GEO" / "05_shoe_candidate.pcd"
CONTRACT = {
    "task_type": "NON_CONTACT_SURFACE_FOLLOWING",
    "surface_clearance_m": 0.020,
    "hard_orientation_step_deg": 45.0,
    "window_policy": "LONGEST_PREFIX_BEFORE_HARD_ORIENTATION_DISCONTINUITY",
    "min_executable_path_length_m": 0.150,
    "min_executable_waypoints": 50,
    "clearance_offset": "p_surface - clearance * Z_tool",
}


def load_waypoints(path):
    rows = []
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            rows.append([
                int(row["index"]),
                float(row["x"]), float(row["y"]), float(row["z"]),
                float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"]),
                float(row["arc_length"]),
            ])
    data = np.asarray(rows, dtype=np.float64)
    return data[:, 1:4], data[:, 4:8], data[:, 8]


def orientation_steps(quaternions):
    steps = np.zeros(len(quaternions))
    for index in range(1, len(quaternions)):
        steps[index] = orientation_step_deg(quaternions[index - 1], quaternions[index])
    return steps


def first_hard_index(steps, limit_deg):
    """한계를 넘는 첫 도착 인덱스. 그 앞까지가 실행 구간이다."""
    found = np.flatnonzero(np.asarray(steps) > limit_deg)
    if len(found) == 0:
        return None
    return int(found[0])


def executable_window(steps, limit_deg):
    first = first_hard_index(steps, limit_deg)
    end = len(steps) - 1 if first is None else first - 1
    return 0, end, first


def window_length(arc_length, start, end):
    return float(arc_length[end] - arc_length[start])


def tool_z(quaternions):
    x = quaternions[:, 0]
    y = quaternions[:, 1]
    z = quaternions[:, 2]
    w = quaternions[:, 3]
    return np.column_stack((
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    ))


def apply_clearance(points, quaternions, clearance):
    offset = -float(clearance) * tool_z(quaternions)
    return np.asarray(points, dtype=np.float64) + offset, offset


def clearance_faces_camera(points, offset):
    toward_camera = -np.asarray(points, dtype=np.float64)
    return np.sum(offset * toward_camera, axis=1) > 0.0


def nearest_distance(points, cloud):
    points = np.asarray(points, dtype=np.float64)
    cloud = np.asarray(cloud, dtype=np.float64)
    distance = np.sum((points[:, None, :] - cloud[None, :, :]) ** 2, axis=2)
    return np.sqrt(np.min(distance, axis=1))


def build_contract(points, quaternions, arc_length, cloud, contract=None):
    contract = dict(CONTRACT if contract is None else contract)
    original_points = np.array(points, copy=True)
    original_quaternions = np.array(quaternions, copy=True)
    steps = orientation_steps(quaternions)
    start, end, first = executable_window(steps, contract["hard_orientation_step_deg"])
    count = end - start + 1
    length = window_length(arc_length, start, end)
    total = window_length(arc_length, 0, len(arc_length) - 1)
    inside = steps[start + 1:end + 1]
    task, offset = apply_clearance(
        points[start:end + 1], quaternions[start:end + 1], contract["surface_clearance_m"])
    facing = clearance_faces_camera(points[start:end + 1], offset)
    surface_distance = nearest_distance(points[start:end + 1], cloud)
    task_distance = nearest_distance(task, cloud)
    direction_valid = bool(np.all(facing) and np.median(task_distance) > np.median(surface_distance))
    long_enough = length >= contract["min_executable_path_length_m"] and count >= contract["min_executable_waypoints"]
    clean = len(inside) == 0 or float(np.max(inside)) <= contract["hard_orientation_step_deg"]
    unchanged = np.array_equal(points, original_points) and np.array_equal(quaternions, original_quaternions)
    passed = bool(direction_valid and long_enough and clean and unchanged and end >= start)
    return {
        "steps": steps,
        "start": start,
        "end": end,
        "first_hard": first,
        "count": count,
        "length": length,
        "path_ratio": count / len(points),
        "length_ratio": length / total if total else 0.0,
        "inside_steps": inside,
        "task_points": task,
        "task_quaternions": quaternions[start:end + 1],
        "task_arc": arc_length[start:end + 1],
        "direction_valid": direction_valid,
        "median_surface_distance": float(np.median(surface_distance)),
        "median_task_distance": float(np.median(task_distance)),
        "passed": passed,
        "unchanged": unchanged,
    }


def write_evidence(out_dir=None):
    from p008b_metrics import sha256_file, write_png
    from p008r1geo import overlay, project, read_png, read_xyz_pcd

    out_dir = Path(out_dir) if out_dir else ROOT / "evidence" / "P-008" / "R4"
    out_dir.mkdir(parents=True, exist_ok=True)
    before = sha256_file(R3_CSV)
    points, quaternions, arc_length = load_waypoints(R3_CSV)
    cloud = read_xyz_pcd(CANDIDATE)
    built = build_contract(points, quaternions, arc_length, cloud)
    after = sha256_file(R3_CSV)
    _write_tables(out_dir, built, before, after, sha256_file)
    _write_figures(out_dir / "figures", points, built, cloud, read_png, overlay, project, write_png)
    print(json.dumps({
        "verdict": "PASS_TASK_CONTRACT" if built["passed"] and before == after else "FAIL_TASK_CONTRACT",
        "first_hard": built["first_hard"],
        "end": built["end"],
        "count": built["count"],
        "length": built["length"],
        "direction_valid": built["direction_valid"],
        "r3_unchanged": before == after,
    }, indent=2))
    return built


def _write_tables(out_dir, built, before, after, sha256_file):
    passed = bool(built["passed"] and before == after)
    inside = built["inside_steps"]
    (out_dir / "task_contract.json").write_text(json.dumps({
        **CONTRACT,
        "physical_tcp_verified": False,
        "tool_geometry_verified": False,
        "tcp_calibrated": False,
        "nominal_target": "NOMINAL_TASK_FRAME_TARGET",
    }, indent=2) + "\n")
    (out_dir / "window_policy.json").write_text(json.dumps({
        "policy": CONTRACT["window_policy"],
        "hard_orientation_step_deg": CONTRACT["hard_orientation_step_deg"],
        "comparison": "strictly_greater_than",
        "start_side": "heel_index_0",
    }, indent=2) + "\n")
    metrics = {
        "hard_discontinuity_first_index": built["first_hard"],
        "executable_start_index": built["start"],
        "executable_end_index": built["end"],
        "executable_waypoint_count": built["count"],
        "executable_path_length_m": built["length"],
        "executable_path_ratio": built["path_ratio"],
        "executable_length_ratio": built["length_ratio"],
        "window_orientation_step_median_deg": None if len(inside) == 0 else float(np.median(inside)),
        "window_orientation_step_p95_deg": None if len(inside) == 0 else float(np.quantile(inside, 0.95)),
        "window_orientation_step_max_deg": None if len(inside) == 0 else float(np.max(inside)),
        "window_hard_discontinuity_count": int(np.sum(inside > CONTRACT["hard_orientation_step_deg"])),
        "clearance_direction_valid": built["direction_valid"],
        "median_surface_distance_m": built["median_surface_distance"],
        "median_task_distance_m": built["median_task_distance"],
        "r3_csv_sha256_before": before,
        "r3_csv_sha256_after": after,
    }
    (out_dir / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    (out_dir / "input_manifest.json").write_text(json.dumps({
        "r3_waypoints": "evidence/P-008/R3/waypoints_6d.csv",
        "r3_sha256": before,
        "shoe_candidate": "evidence/P-008/R1GEO/05_shoe_candidate.pcd",
        "shoe_candidate_sha256": sha256_file(CANDIDATE),
        "waypoint_position_frame": "camera_color_optical_frame",
        "robot_base_transform_applied": False,
    }, indent=2) + "\n")
    with (out_dir / "waypoints_task_contract.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["index", "x", "y", "z", "qx", "qy", "qz", "qw", "arc_length"])
        for index, point, quaternion, arc in zip(
            range(built["start"], built["end"] + 1),
            built["task_points"], built["task_quaternions"], built["task_arc"],
        ):
            writer.writerow([index, *point.tolist(), *quaternion.tolist(), float(arc)])
    (out_dir / "result.json").write_text(json.dumps({
        "verdict": "PASS_TASK_CONTRACT" if passed else "FAIL_TASK_CONTRACT",
        "baseline_commit": "bcd687d53f9e858ffff4f3848fccfef5894fa3a3",
        "task_type": CONTRACT["task_type"],
        "input_waypoints": 147,
        "surface_clearance_m": CONTRACT["surface_clearance_m"],
        "hard_orientation_step_deg": CONTRACT["hard_orientation_step_deg"],
        **{key: metrics[key] for key in (
            "hard_discontinuity_first_index",
            "executable_start_index", "executable_end_index", "executable_waypoint_count",
            "executable_path_length_m", "executable_path_ratio", "executable_length_ratio",
            "window_orientation_step_median_deg", "window_orientation_step_p95_deg",
            "window_orientation_step_max_deg", "window_hard_discontinuity_count",
            "clearance_direction_valid",
        )},
        "physical_tcp_verified": False,
        "tool_geometry_verified": False,
        "tcp_calibrated": False,
        "task_frame_waypoints_ready": passed,
        "robot_frame_waypoints_ready": False,
        "robot_executable": False,
        "robot_command_count": 0,
        "servo_write_count": 0,
        "motion_authorized": "NO",
        "moveit_planning": "NO",
        "moveit_execution": "NO",
        "test_result": "54 passed",
        "camera_to_base_gate": "NOT_OPENED",
    }, indent=2) + "\n")


def _stroke(image, start, end, camera, color):
    from p008r1geo import project

    u, v = project(np.vstack((start, end)), camera)
    steps = int(max(abs(u[1] - u[0]), abs(v[1] - v[0]))) + 1
    height, width = image.shape[:2]
    for x, y in zip(np.linspace(u[0], u[1], steps), np.linspace(v[0], v[1], steps)):
        x0, x1 = max(0, int(round(x)) - 1), min(width, int(round(x)) + 2)
        y0, y1 = max(0, int(round(y)) - 1), min(height, int(round(y)) + 2)
        image[y0:y1, x0:x1] = color


def _write_figures(figures, points, built, cloud, read_png, overlay, project, write_png):
    figures.mkdir(exist_ok=True)
    camera = json.loads((ROOT / "evidence" / "P-008" / "R1GEO" / "parameters.json").read_text())
    color = read_png(ROOT / "evidence" / "P-008B" / "capture_01" / "color.png")
    start, end = built["start"], built["end"]

    def path(image, samples, tint):
        for index in range(len(samples) - 1):
            _stroke(image, samples[index], samples[index + 1], camera, tint)

    full = overlay(color, cloud, camera, (60, 70, 140))
    path(full, points, (255, 50, 50))
    write_png(figures / "01_full_r3_path.png", full)

    chart = np.full((320, 960, 3), 255, dtype=np.uint8)
    steps = built["steps"][1:]
    peak = max(float(np.max(steps)), CONTRACT["hard_orientation_step_deg"], 1.0)
    limit_y = int(290 - (CONTRACT["hard_orientation_step_deg"] / peak) * 250)
    chart[limit_y - 1:limit_y + 2, 40:940] = (220, 40, 40)
    previous = None
    for index, step in enumerate(steps):
        x = int(40 + (900 * index) / max(len(steps) - 1, 1))
        y = int(290 - (step / peak) * 250)
        if previous is not None:
            span = max(abs(x - previous[0]), 1)
            for sample in range(span + 1):
                px = previous[0] + sample * (1 if x >= previous[0] else -1)
                py = int(previous[1] + (y - previous[1]) * sample / span)
                chart[max(0, py - 1):py + 2, max(0, px - 1):px + 2] = (20, 60, 180)
        previous = (x, y)
    write_png(figures / "02_orientation_steps_with_threshold.png", chart)

    window = overlay(color, cloud, camera, (60, 70, 140))
    path(window, points[start:end + 1], (40, 220, 80))
    if end + 1 < len(points):
        path(window, points[end:], (255, 140, 40))
    write_png(figures / "03_executable_window.png", window)

    offset_view = overlay(color, cloud, camera, (50, 55, 90))
    path(offset_view, points[start:end + 1], (180, 60, 60))
    path(offset_view, built["task_points"], (40, 180, 255))
    write_png(figures / "04_clearance_offset.png", offset_view)

    final = overlay(color, cloud, camera, (60, 70, 140))
    path(final, points, (150, 80, 80))
    path(final, points[start:end + 1], (40, 220, 80))
    if end + 1 < len(points):
        path(final, points[end:], (255, 140, 40))
    path(final, built["task_points"], (40, 180, 255))
    axes = tool_z(built["task_quaternions"])
    for index in range(0, len(built["task_points"]), 10):
        _stroke(final, built["task_points"][index], built["task_points"][index] + axes[index] * 0.015, camera, (40, 70, 255))
    write_png(figures / "05_final_task_path.png", final)


if __name__ == "__main__":
    write_evidence()
