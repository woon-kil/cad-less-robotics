#!/usr/bin/env python3
"""P-013 ProcessPath를 P-022 RobotPlanningInput으로 바꾼다.

동결된 단위 변환과 simulation datum만 적용한다. MoveIt은 호출하지 않는다.
"""

import json
import math
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
INPUT_PATH = ROOT / "evidence" / "P-013" / "cad_process_path.json"
OUT_DIR = Path(os.environ.get("P022_OUT", ROOT / "evidence" / "P-022"))

TRANSLATION_M = (0.3244333936553, -0.0499999998932, 0.11133404082435)
REQUIRED = (
    "surface_anchor",
    "command_position",
    "orientation_quaternion",
    "surface_normal",
    "traversal_tangent",
    "tool_tangent",
    "process_offset_mm",
    "segment_type",
    "sequence_id",
)


def finite_numbers(values):
    return all(isinstance(value, (int, float)) and math.isfinite(value) for value in values)


def add_translation(position_mm):
    return [position_mm[index] * 0.001 + TRANSLATION_M[index] for index in range(3)]


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    data = json.loads(INPUT_PATH.read_text())
    waypoints = data["waypoints"]
    failures = []
    if data.get("frame_id") != "CAD_MODEL_FRAME":
        failures.append("frame_id")
    if data.get("position_unit") != "mm":
        failures.append("position_unit")
    if data.get("quaternion_order") != ["x", "y", "z", "w"]:
        failures.append("quaternion_order")
    if len(waypoints) != 741:
        failures.append("waypoint_count")

    poses = []
    for waypoint in waypoints:
        missing = [name for name in REQUIRED if name not in waypoint]
        if missing:
            failures.append("missing_field")
            break
        anchor = waypoint["surface_anchor"]
        command = waypoint["command_position"]
        quaternion = waypoint["orientation_quaternion"]
        normal = waypoint["surface_normal"]
        traversal = waypoint["traversal_tangent"]
        tool = waypoint["tool_tangent"]
        vectors = (anchor, command, quaternion, normal, traversal, tool)
        if any(not finite_numbers(values) for values in vectors):
            failures.append("nonfinite")
            break
        if any(len(values) != 3 for values in (anchor, command, normal, traversal, tool)) or len(quaternion) != 4:
            failures.append("dimension")
            break
        norm = math.sqrt(sum(value * value for value in quaternion))
        if abs(norm - 1.0) > 1.0e-6:
            failures.append("quaternion_norm")
            break
        poses.append({
            "sequence_id": waypoint["sequence_id"],
            "pass_index": waypoint["pass_index"],
            "segment_type": waypoint["segment_type"],
            "surface_anchor_m": add_translation(anchor),
            "command_position_m": add_translation(command),
            "process_offset_m": waypoint["process_offset_mm"] * 0.001,
            "orientation_quaternion": quaternion,
            "surface_normal": normal,
            "traversal_tangent": traversal,
            "tool_tangent": tool,
        })

    runs = []
    current = []
    transition_ids = []
    for pose in poses:
        if pose["segment_type"] == "PROCESS":
            current.append(pose["sequence_id"])
        elif pose["segment_type"] == "TRANSITION":
            transition_ids.append(pose["sequence_id"])
            if current:
                runs.append(current)
                current = []
        else:
            failures.append("segment_type")
            break
    if current:
        runs.append(current)

    process_edges = 0
    transition_edges = 0
    for previous, current_waypoint in zip(waypoints, waypoints[1:]):
        del previous
        if current_waypoint["segment_type"] == "TRANSITION":
            transition_edges += 1
        elif current_waypoint["segment_type"] == "PROCESS":
            process_edges += 1

    validation = {
        "input_artifact": "evidence/P-013/cad_process_path.json",
        "input_waypoint_count": len(waypoints),
        "failures": sorted(set(failures)),
        "nonfinite_count": 0 if "nonfinite" not in failures else 1,
        "unit_conversion": "position_m = position_mm * 0.001",
        "length_conversion_excluded": [
            "orientation_quaternion",
            "surface_normal",
            "traversal_tangent",
            "tool_tangent",
        ],
        "transform_source": "CONFIGURED_SIMULATION_DATUM",
        "transform_role": "SIMULATION_PLACEMENT",
        "rotation": "identity",
        "translation_m": list(TRANSLATION_M),
        "measured_transform": False,
        "calibration_result": False,
        "ground_truth": "NONE",
        "output_pose_count": len(poses),
        "process_run_count": len(runs),
        "process_waypoint_count": sum(len(run) for run in runs),
        "process_segment_count": process_edges,
        "transition_segment_count": transition_edges,
        "transition_sequence_ids": transition_ids,
        "transition_motion_generation": "OUT_OF_SCOPE",
    }
    (OUT_DIR / "input_validation.json").write_text(json.dumps(validation, indent=2) + "\n")
    if failures:
        raise SystemExit(2)

    planning_input = {
        "contract": "RobotPlanningInput",
        "robot_trajectory": False,
        "frame_id": "base_link",
        "position_unit": "m",
        "quaternion_order": ["x", "y", "z", "w"],
        "source_process_path": "evidence/P-013/cad_process_path.json",
        "reference_robot": "Universal Robots UR5e",
        "robot_role": "PLANNING_REFERENCE_ROBOT",
        "planning_group": "ur_manipulator",
        "base_frame": "base_link",
        "tcp_frame": "tool0",
        "tcp_role": "PLANNING_REFERENCE_TCP",
        "physical_process_tcp": False,
        "transform_source": "CONFIGURED_SIMULATION_DATUM",
        "transform_role": "SIMULATION_PLACEMENT",
        "rotation": "identity",
        "translation_m": list(TRANSLATION_M),
        "measured_transform": False,
        "calibration_result": False,
        "ground_truth": "NONE",
        "process_runs": [{"run_id": index, "sequence_ids": run} for index, run in enumerate(runs)],
        "transition_sequence_ids": transition_ids,
        "poses": poses,
    }
    (OUT_DIR / "robot_planning_input.json").write_text(json.dumps(planning_input) + "\n")

    lines = []
    by_id = {pose["sequence_id"]: pose for pose in poses}
    for index, run in enumerate(runs):
        lines.append(f"RUN {index} {len(run)}")
        for sequence_id in run:
            pose = by_id[sequence_id]
            position = pose["command_position_m"]
            quaternion = pose["orientation_quaternion"]
            values = position + quaternion
            lines.append(" ".join(format(value, ".17g") for value in values))
    (OUT_DIR / "process_runs.txt").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
