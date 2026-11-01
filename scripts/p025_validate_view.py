#!/usr/bin/env python3
"""P-025 V-01..V-10.

Visualization Model과 frozen evidence를 비교한다.
불일치는 화면 쪽의 실패다. upstream 파일은 고치지 않는다.
이 스크립트는 P025_PASS를 정하지 않는다.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "evidence" / "P-025" / "visualization_model.json"
JS_PATH = ROOT / "evidence" / "P-025" / "viewer" / "model.js"
HTML_PATH = ROOT / "evidence" / "P-025" / "viewer" / "index.html"
REPORT_PATH = ROOT / "evidence" / "P-025" / "validation_v01_v10.md"

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

SCOPE_TEXT = (
    "동결된 P-022 계약과 검토 환경 내에서 구현을 정당화할 대안을 확보하지 못함. "
    "계획 해법의 부재를 의미하지 않음."
)


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
    roadmap = (ROOT / "evidence" / "PROJECT-ROADMAP.md").read_text(encoding="utf-8")
    start = roadmap.index("PRIMARY STORY:\n")
    end = roadmap.index("### P-026 Portfolio / Demo Packaging")
    body = roadmap[start:end]
    files["evidence/PROJECT-ROADMAP.md#P025_HISTORICAL_BODY"] = hashlib.sha256(
        body.encode("utf-8")
    ).hexdigest()
    return files


def main() -> None:
    before = upstream_manifest()
    model = json.loads(MODEL_PATH.read_text(encoding="utf-8"))
    js = JS_PATH.read_text(encoding="utf-8")
    prefix = "window.P025_MODEL = "
    if not js.startswith(prefix) or not js.endswith(";\n"):
        raise SystemExit("model.js wrapper")
    js_model = json.loads(js[len(prefix):-2])
    if js_model != model:
        raise SystemExit("model.js differs from visualization_model.json")
    html = HTML_PATH.read_text(encoding="utf-8")
    p011 = (ROOT / "evidence" / "P-011" / "P-011_REPORT.md").read_text(encoding="utf-8")
    p012 = (ROOT / "evidence" / "P-012" / "P-012_REPORT.md").read_text(encoding="utf-8")
    p013 = json.loads(
        (ROOT / "evidence" / "P-013" / "cad_process_path.json").read_text(encoding="utf-8")
    )
    p021 = (ROOT / "evidence" / "P-021" / "P-021_REPORT.md").read_text(encoding="utf-8")
    p022 = (ROOT / "evidence" / "P-022" / "P-022_REPORT.md").read_text(encoding="utf-8")
    diagnosis = (ROOT / "evidence" / "P-022" / "failure_diagnosis.md").read_text(encoding="utf-8")
    runs = json.loads(
        (ROOT / "evidence" / "P-022" / "cartesian_runs.json").read_text(encoding="utf-8")
    )["runs"]

    results: list[tuple[str, str]] = []

    def check(name: str, ok: bool, detail: str) -> None:
        results.append((name, "PASS" if ok else "FAIL", detail))
        if not ok:
            raise SystemExit(f"{name} FAIL: {detail}")

    scene1 = model["scene1"]
    check(
        "V-01",
        scene1["final_gate"] == "PASS_COMPLEX_SURFACE_CAD_TEST_COUPON"
        and "FINAL_GATE = PASS_COMPLEX_SURFACE_CAD_TEST_COUPON" in p011
        and scene1["face_count"] == 20
        and "FACE_COUNT = 20" in p011
        and scene1["edge_count"] == 40
        and "EDGE_COUNT = 40" in p011
        and scene1["solid_count"] == 1
        and scene1["bounding_box_mm"] == {"X": "180.0", "Y": "100.0", "Z": "41.252357465702"}
        and "Z = 41.252357465702" in p011
        and scene1["step_sha256"] in p012
        and scene1["ground_truth"] == "NONE"
        and scene1["geometry_role"] == "REFERENCE_GEOMETRY"
        and scene1["role"] == "CONTROLLED_TEST_COUPON",
        "Scene 1 fields",
    )

    scene2 = model["scene2"]
    check(
        "V-02",
        scene2["final_gate"] == "PASS_CAD_GEOMETRY_ENGINE"
        and "FINAL_GATE = PASS_CAD_GEOMETRY_ENGINE" in p012
        and scene2["surface_types"] == "BSpline 14, Plane 5, Cylinder 1"
        and scene2["total_sample_count"] == 1620
        and "TOTAL_SAMPLE_COUNT = 1620" in p012
        and scene2["valid_sample_count"] == 1585
        and scene2["invalid_sample_count"] == 35
        and scene2["normal_failure_count"] == 0
        and scene2["k_min"] == "-0.0011310794"
        and scene2["k_max"] == "0.0017847765"
        and scene2["face_count"] == 20,
        "Scene 2 fields",
    )

    scene3 = model["scene3"]
    stored = [
        {
            "sequence_id": item["sequence_id"],
            "pass_index": item["pass_index"],
            "surface_anchor": item["surface_anchor"],
            "command_position": item["command_position"],
            "surface_normal": item["surface_normal"],
            "orientation_quaternion": item["orientation_quaternion"],
            "process_offset_mm": item["process_offset_mm"],
        }
        for item in p013["waypoints"]
    ]
    check(
        "V-03",
        scene3["waypoint_count"] == 741
        and len(p013["waypoints"]) == 741
        and scene3["waypoints"] == stored
        and scene3["frame"] == p013["frame_id"] == "CAD_MODEL_FRAME"
        and scene3["position_unit"] == "mm"
        and scene3["quaternion_order"] == ["x", "y", "z", "w"]
        and scene3["residual_max_mm"] == "1.000017846308765e-09"
        and "1.000017846308765e-09 mm" in p021
        and scene3["process_offset_mm"] == 20.0
        and scene3["tool_standoff_mm_key"] == "absent"
        and all("tool_standoff_mm" not in item for item in p013["waypoints"])
        and p013["ground_truth"] == "NONE",
        "Scene 3 stored waypoints and residual string",
    )

    scene4 = model["scene4"]
    check(
        "V-04",
        scene4["p021"]["transform_executed"] == "NO"
        and scene4["p021"]["robot_planning_input_file_written"] == "NO"
        and scene4["p021"]["formula"] == "x_m = x_mm * 0.001"
        and "실제 변환은 하지 않았다" in p021
        and "741-entry 데이터 파일은 없다" in p021
        and scene4["p022"]["translation_m"]
        == ["0.3244333936553", "-0.0499999998932", "0.11133404082435"]
        and "translation_m [0.3244333936553, -0.0499999998932, 0.11133404082435]" in p022
        and scene4["p022"]["datum_role"] == "CONFIGURED_SIMULATION_DATUM"
        and "TRANSFORM_SOURCE = CONFIGURED_SIMULATION_DATUM" in p022
        and scene4["p022"]["tcp_role"] == "PLANNING_REFERENCE_TCP",
        "Scene 4 provenance split",
    )

    scene5 = model["scene5"]
    by_id = {row["run_id"]: row for row in runs}
    mapped = {row["run_id"]: row for row in scene5["runs"]}
    failed = [row["run_id"] for row in runs if row.get("reason") == "START_IK_FAILED"]
    full = [row["run_id"] for row in runs if row["fraction"] == 1]
    nulls_ok = all(mapped[i]["fraction"] is None and mapped[i]["fraction_display"] == "N/A" for i in (5, 7, 12))
    check(
        "V-05",
        scene5["process_runs"] == 21
        and list(mapped) == list(range(21))
        and failed == [5, 7, 12]
        and scene5["start_ik_failed_runs"] == [5, 7, 12]
        and len(full) == 17
        and scene5["full_fraction_runs"] == 17
        and "FULL_FRACTION_RUN_COUNT = 17" in diagnosis
        and by_id[20]["fraction"] == 0.606061
        and scene5["partial_fraction"] == "0.606061"
        and scene5["completed_intervals"] == 20
        and scene5["total_intervals"] == 33
        and "20 / 33" in diagnosis
        and nulls_ok
        and all(row.get("trajectory_written") is False for row in runs if "trajectory_written" in row)
        and all(
            mapped[i]["trajectory_written"] is None
            and mapped[i]["trajectory_written_display"] == "absent"
            for i in (5, 7, 12)
        )
        and all(
            mapped[i]["trajectory_written"] is False
            for i in range(21)
            if i not in (5, 7, 12)
        )
        and scene5["robottrajectory_created"] == "NO"
        and "ROBOTTRAJECTORY_CREATED = NO" in p022
        and "80.95" not in json.dumps(model)
        and "0%" not in json.dumps(model),
        "Scene 5 run_id 0-20, null fraction, no derived percent",
    )

    check(
        "V-06",
        model["overview"]["verified_boundary"] == "P-022: CLOSED_FAIL"
        and scene5["p022_status"] == "CLOSED_FAIL"
        and "CLOSED_FAIL" in html
        and "P-022: CLOSED_FAIL" in html,
        "CLOSED_FAIL visible",
    )

    check(
        "V-07",
        scene5["scope_text"] == SCOPE_TEXT and SCOPE_TEXT in html,
        "scoped NO_JUSTIFIABLE_ALTERNATIVE text",
    )

    check(
        "V-08",
        model["overview"]["robot_trajectory"] == "NOT ACHIEVED"
        and "RobotTrajectory" in html
        and "NOT ACHIEVED" in html
        and "6D ProcessPath" in html
        and html.index("6D ProcessPath") < html.index("NOT ACHIEVED"),
        "ProcessPath and RobotTrajectory are distinct",
    )

    outside = html
    leaked = [phrase for phrase in PROHIBITED if phrase in outside]
    check(
        "V-09",
        leaked == []
        and model["claim_boundary"]["prohibited"] == PROHIBITED
        and "주장하지 않음" in html,
        "prohibited claims are not success statements in the page",
    )

    after = upstream_manifest()
    check(
        "V-10",
        before == after and model["provenance"]["upstream_sha256"] == before,
        "upstream evidence and historical P-025 body unchanged",
    )

    lines = [
        "# P-025 V-01..V-10",
        "",
        "```text",
        "P025_VALIDATION = V01_V10_EXECUTED",
        "P025_PASS = NOT_EVALUATED",
        "UPSTREAM_EVIDENCE = READ_ONLY",
        "```",
        "",
        "비교 대상은 Visualization Model과 frozen evidence다. 실패하면 adapter 또는 화면을 고친다. upstream evidence는 고치지 않는다.",
        "",
        "| Check | Result |",
        "| --- | --- |",
    ]
    for name, status, detail in results:
        lines.append(f"| {name} | {status} | {detail} |")
    lines.extend(
        [
            "",
            "## AC 대응",
            "",
            "이 표는 검사 결과의 대응이다. Gate PASS는 reviewer 판정 전이다.",
            "",
            "| AC | 이 실행에서 확인한 것 | 판정 |",
            "| --- | --- | --- |",
            "| AC-01 | V-01..V-05. 정량 필드는 출처 파일의 저장 값이다 | PENDING_REVIEWER |",
            "| AC-02 | Scene별 FINAL_GATE만 성공으로 표시한다 | PENDING_REVIEWER |",
            "| AC-03 | V-06. overview와 Scene 5에 CLOSED_FAIL | PENDING_REVIEWER |",
            "| AC-04 | V-05. 17, run_id 5·7·12, run_id 20, 0.606061, 20/33 | PENDING_REVIEWER |",
            "| AC-05 | V-07. 고정 문구 | PENDING_REVIEWER |",
            "| AC-06 | V-08. ProcessPath와 RobotTrajectory NOT ACHIEVED | PENDING_REVIEWER |",
            "| AC-07 | CAD = REFERENCE_GEOMETRY, Scan = MEASURED_GEOMETRY, GROUND_TRUTH = NONE | PENDING_REVIEWER |",
            "| AC-08 | Scene 5의 P-023, P-024는 NOT_OPENED | PENDING_REVIEWER |",
            "| AC-09 | overview에 VERIFIED BOUNDARY와 NOT ACHIEVED가 있다 | PENDING_REVIEWER |",
            "| AC-10 | V-10. P-011..P-022와 역사 본문 해시가 같다 | PENDING_REVIEWER |",
            "",
        ]
    )
    REPORT_PATH.write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(f"{name} {status}" for name, status, _detail in results))
    print(f"wrote {REPORT_PATH}")


if __name__ == "__main__":
    main()
