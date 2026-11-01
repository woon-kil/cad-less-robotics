#!/usr/bin/env python3
"""P-025 Evidence Adapter.

Frozen evidence의 저장 값을 Visualization Model로 옮긴다.
normal 재추정, path 재생성, datum 재적용, IK, fraction 재집계는 하지 않는다.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT_JSON = ROOT / "evidence" / "P-025" / "visualization_model.json"
OUT_JS = ROOT / "evidence" / "P-025" / "viewer" / "model.js"

SCOPE_TEXT = (
    "동결된 P-022 계약과 검토 환경 내에서 구현을 정당화할 대안을 확보하지 못함. "
    "계획 해법의 부재를 의미하지 않음."
)

PROHIBITED = [
    "RobotTrajectory generated",
    "trajectory valid",
    "collision-free",
    "all ProcessPath poses reachable",
    "UR5e can physically execute the path",
    "physical calibration completed",
    "physical accuracy validated",
    "production-ready",
    "ground truth established",
]


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def require(text: str, needle: str, source: str) -> None:
    if needle not in text:
        raise SystemExit(f"mapping source missing: {source}: {needle}")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def upstream_manifest() -> dict[str, str]:
    files: dict[str, str] = {}
    for gate in ("P-011", "P-012", "P-013", "P-021", "P-022"):
        for path in sorted((ROOT / "evidence" / gate).rglob("*")):
            if path.is_file():
                files[str(path.relative_to(ROOT))] = sha256_file(path)
    roadmap = read(ROOT / "evidence" / "PROJECT-ROADMAP.md")
    start = roadmap.index("PRIMARY STORY:\n")
    end = roadmap.index("### P-026 Portfolio / Demo Packaging")
    body = roadmap[start:end]
    files["evidence/PROJECT-ROADMAP.md#P025_HISTORICAL_BODY"] = hashlib.sha256(
        body.encode("utf-8")
    ).hexdigest()
    return files


def main() -> None:
    p011 = read(ROOT / "evidence" / "P-011" / "P-011_REPORT.md")
    p012 = read(ROOT / "evidence" / "P-012" / "P-012_REPORT.md")
    p013 = json.loads(read(ROOT / "evidence" / "P-013" / "cad_process_path.json"))
    p021 = read(ROOT / "evidence" / "P-021" / "P-021_REPORT.md")
    p022 = read(ROOT / "evidence" / "P-022" / "P-022_REPORT.md")
    diagnosis = read(ROOT / "evidence" / "P-022" / "failure_diagnosis.md")
    runs_doc = json.loads(read(ROOT / "evidence" / "P-022" / "cartesian_runs.json"))
    runs = runs_doc["runs"]

    for needle in (
        "FINAL_GATE = PASS_COMPLEX_SURFACE_CAD_TEST_COUPON",
        "UNIT = mm",
        "SOLID_COUNT = 1",
        "FACE_COUNT = 20",
        "EDGE_COUNT = 40",
        "X = 180.0",
        "Y = 100.0",
        "Z = 41.252357465702",
    ):
        require(p011, needle, "P-011_REPORT.md")
    for needle in (
        "FINAL_GATE = PASS_CAD_GEOMETRY_ENGINE",
        "FACE_COUNT = 20",
        "SURFACE_TYPES = BSpline 14, Plane 5, Cylinder 1",
        "TOTAL_SAMPLE_COUNT = 1620",
        "VALID_SAMPLE_COUNT = 1585",
        "INVALID_SAMPLE_COUNT = 35",
        "NORMAL_FAILURE_COUNT = 0",
        "K_MIN = -0.0011310794",
        "K_MAX = 0.0017847765",
        "INPUT_SHA256 = 2a791f83b37439ef59c425422599765c26d273b30d56f624dedfbd9a8d3bb3fa",
    ):
        require(p012, needle, "P-012_REPORT.md")
    require(
        p021,
        "1.000017846308765e-09 mm",
        "P-021_REPORT.md",
    )
    require(p021, "실제 변환은 하지 않았다", "P-021_REPORT.md")
    require(p021, "741-entry 데이터 파일은 없다", "P-021_REPORT.md")
    require(
        p022,
        "FRAME_TRANSFORM = identity rotation, translation_m [0.3244333936553, -0.0499999998932, 0.11133404082435]",
        "P-022_REPORT.md",
    )
    require(p022, "TRANSFORM_SOURCE = CONFIGURED_SIMULATION_DATUM", "P-022_REPORT.md")
    require(p022, "ROBOTTRAJECTORY_CREATED = NO", "P-022_REPORT.md")
    require(diagnosis, "FULL_FRACTION_RUN_COUNT = 17", "failure_diagnosis.md")
    require(diagnosis, "fraction = 0.606061", "failure_diagnosis.md")
    require(diagnosis, "20 / 33", "failure_diagnosis.md")
    require(diagnosis, "BASELINE_FINAL_GATE = FAIL_CARTESIAN_API", "failure_diagnosis.md")

    if p013["frame_id"] != "CAD_MODEL_FRAME":
        raise SystemExit("frame_id")
    if p013["position_unit"] != "mm":
        raise SystemExit("position_unit")
    if p013["quaternion_order"] != ["x", "y", "z", "w"]:
        raise SystemExit("quaternion_order")
    if p013["geometry_role"] != "REFERENCE_GEOMETRY":
        raise SystemExit("geometry_role")
    if p013["ground_truth"] != "NONE":
        raise SystemExit("ground_truth")
    waypoints = p013["waypoints"]
    if len(waypoints) != 741:
        raise SystemExit("waypoint count")
    if any("tool_standoff_mm" in item for item in waypoints):
        raise SystemExit("tool_standoff_mm present")
    if any(item["process_offset_mm"] != 20.0 for item in waypoints):
        raise SystemExit("process_offset_mm")

    display_runs = []
    for row in runs:
        fraction = row["fraction"]
        if fraction is None:
            fraction_display = "N/A"
        elif fraction == 1:
            fraction_display = "1"
        elif row["run_id"] == 20:
            fraction_display = "0.606061"
        else:
            raise SystemExit(f"unmapped fraction run {row['run_id']}")
        if "trajectory_written" in row:
            trajectory_written = row["trajectory_written"]
            trajectory_written_display = "false" if trajectory_written is False else "true"
        else:
            trajectory_written = None
            trajectory_written_display = "absent"
        display_runs.append(
            {
                "run_id": row["run_id"],
                "fraction": fraction,
                "fraction_display": fraction_display,
                "reason": row.get("reason"),
                "api_status": row["api_status"],
                "trajectory_written": trajectory_written,
                "trajectory_written_display": trajectory_written_display,
                "source": "evidence/P-022/cartesian_runs.json",
            }
        )

    view_waypoints = []
    for item in waypoints:
        view_waypoints.append(
            {
                "sequence_id": item["sequence_id"],
                "pass_index": item["pass_index"],
                "surface_anchor": item["surface_anchor"],
                "command_position": item["command_position"],
                "surface_normal": item["surface_normal"],
                "orientation_quaternion": item["orientation_quaternion"],
                "process_offset_mm": item["process_offset_mm"],
            }
        )

    model = {
        "schema": "p025_visualization_model",
        "transformation": "SOURCE_TO_DISPLAY_MAPPING",
        "recomputation": "NOT_PERFORMED",
        "derived_analytics": "NOT_CREATED",
        "p025_pass": "NOT_EVALUATED",
        "upstream_evidence": "READ_ONLY",
        "overview": {
            "stages": [
                "REFERENCE CAD",
                "Geometry Engine",
                "6D ProcessPath",
                "Robot Adapter",
                "MoveIt Integration",
            ],
            "verified_boundary": "P-022: CLOSED_FAIL",
            "robot_trajectory": "NOT ACHIEVED",
            "geometry_roles": {
                "CAD": "REFERENCE_GEOMETRY",
                "Scan": "MEASURED_GEOMETRY",
                "GROUND_TRUTH": "NONE",
            },
        },
        "scene1": {
            "source": [
                "evidence/P-011/P-011_REPORT.md",
                "evidence/P-012/P-012_REPORT.md",
            ],
            "role": "CONTROLLED_TEST_COUPON",
            "geometry_role": "REFERENCE_GEOMETRY",
            "ground_truth": "NONE",
            "final_gate": "PASS_COMPLEX_SURFACE_CAD_TEST_COUPON",
            "unit": "mm",
            "solid_count": 1,
            "face_count": 20,
            "edge_count": 40,
            "bounding_box_mm": {"X": "180.0", "Y": "100.0", "Z": "41.252357465702"},
            "step_sha256": "2a791f83b37439ef59c425422599765c26d273b30d56f624dedfbd9a8d3bb3fa",
            "step_sha256_source": "evidence/P-012/P-012_REPORT.md INPUT_SHA256",
            "prohibited": ["footwear CAD", "ground truth"],
        },
        "scene2": {
            "source": "evidence/P-012/P-012_REPORT.md",
            "final_gate": "PASS_CAD_GEOMETRY_ENGINE",
            "face_count": 20,
            "surface_types": "BSpline 14, Plane 5, Cylinder 1",
            "total_sample_count": 1620,
            "valid_sample_count": 1585,
            "invalid_sample_count": 35,
            "normal_failure_count": 0,
            "k_min": "-0.0011310794",
            "k_max": "0.0017847765",
            "ground_truth": "NONE",
            "prohibited": ["new geometry calculation", "physical accuracy"],
        },
        "scene3": {
            "source": [
                "evidence/P-013/cad_process_path.json",
                "evidence/P-021/P-021_REPORT.md",
            ],
            "waypoint_count": 741,
            "frame": "CAD_MODEL_FRAME",
            "position_unit": "mm",
            "quaternion_order": ["x", "y", "z", "w"],
            "geometry_role": "REFERENCE_GEOMETRY",
            "ground_truth": "NONE",
            "standoff": "surface normal 방향 20 mm",
            "process_offset_mm": 20.0,
            "tool_standoff_mm_key": "absent",
            "residual_max_mm": "1.000017846308765e-09",
            "residual_meaning": "계약식과 저장 수치의 일치. 물리 정확도, 로봇 위치 오차, ground truth가 아니다.",
            "distinctions": {
                "surface_anchor": "표면 위의 점. TCP가 아니다.",
                "command_position": "anchor에서 법선 방향 standoff를 더한 점.",
                "tool_z": "-surface_normal. IK goal도 trajectory point도 아니다.",
            },
            "waypoints": view_waypoints,
        },
        "scene4": {
            "p021": {
                "label": "RobotPlanningInput contract",
                "also": "unit/frame conversion requirement",
                "input_frame": "CAD_MODEL_FRAME",
                "unit": "mm",
                "formula": "x_m = x_mm * 0.001",
                "transform_executed": "NO",
                "robot_planning_input_file_written": "NO",
                "source": "evidence/P-021/P-021_REPORT.md",
            },
            "p022": {
                "label": "configured simulation datum applied",
                "also": ["CAD_MODEL_FRAME → base_link", "MoveIt integration"],
                "frame_transform": "identity rotation",
                "translation_m": [
                    "0.3244333936553",
                    "-0.0499999998932",
                    "0.11133404082435",
                ],
                "datum_role": "CONFIGURED_SIMULATION_DATUM",
                "planning_group": "ur_manipulator",
                "tcp_frame": "tool0",
                "tcp_role": "PLANNING_REFERENCE_TCP",
                "source": "evidence/P-022/P-022_REPORT.md",
            },
        },
        "scene5": {
            "source": [
                "evidence/P-022/cartesian_runs.json",
                "evidence/P-022/failure_diagnosis.md",
            ],
            "process_runs": 21,
            "run_id_label": "run_id",
            "run_id_range": [0, 20],
            "full_fraction_runs": 17,
            "start_ik_failed_runs": [5, 7, 12],
            "partial_run": 20,
            "partial_fraction": "0.606061",
            "completed_intervals": 20,
            "total_intervals": 33,
            "final_gate": "FAIL_CARTESIAN_API",
            "p022_status": "CLOSED_FAIL",
            "robottrajectory_created": "NO",
            "trajectory_written_note": "키가 있는 행은 false다. START_IK_FAILED 행에는 키가 없다.",
            "scope_text": SCOPE_TEXT,
            "kdl_scalar_callback": "RECONSIDER",
            "kdl_scalar_callback_meaning": "구현되었다는 뜻이 아니다.",
            "p023": "NOT_OPENED",
            "p024": "NOT_OPENED",
            "runs": display_runs,
            "legend": {
                "fraction_1": "fraction = 1",
                "start_ik_failed": "START_IK_FAILED, fraction = N/A",
                "partial": "run_id 20, fraction = 0.606061, 20/33",
            },
        },
        "claim_boundary": {
            "prohibited": PROHIBITED,
            "failure_handling": "Visualization mismatch → fix adapter or visualization. Do not modify frozen upstream evidence.",
        },
        "provenance": {
            "upstream_sha256": upstream_manifest(),
        },
    }

    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUT_JS.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(model, ensure_ascii=False, indent=2)
    OUT_JSON.write_text(payload + "\n", encoding="utf-8")
    OUT_JS.write_text("window.P025_MODEL = " + payload + ";\n", encoding="utf-8")
    print(f"wrote {OUT_JSON}")
    print(f"wrote {OUT_JS}")


if __name__ == "__main__":
    main()
