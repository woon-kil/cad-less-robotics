#!/usr/bin/env python3
"""채택된 21개 경로를 검사 입력으로 옮긴다. 원본과 manifest는 읽기만 한다."""

import hashlib
import json
import math
import os
import struct
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUN_LENGTHS = [35, 34, 34, 34, 34, 34, 34, 35, 35, 35, 35, 35, 35, 34, 34, 34, 34, 34, 34, 34, 34]


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def selected_sha256(selected):
    raw = json.dumps(selected, ensure_ascii=False, separators=(",", ":")).encode()
    return hashlib.sha256(raw).hexdigest()


def finite_numbers(values, width):
    return (
        isinstance(values, list)
        and len(values) == width
        and all(isinstance(item, (int, float)) and math.isfinite(item) for item in values)
    )


def main():
    if len(sys.argv) != 3:
        print("usage: ur_analytic_ik_sampled_path_check_prepare.py manifest.json out.bin", file=sys.stderr)
        return 2
    manifest_path, out_path = sys.argv[1:]
    manifest = json.load(open(manifest_path, "r", encoding="utf-8"))
    planning = json.load(open(os.path.join(ROOT, "evidence/P-022/robot_planning_input.json"), "r", encoding="utf-8"))
    search = json.load(
        open(os.path.join(ROOT, "evidence/UR-ANALYTIC-IK-WAYPOINT-SURVEY-R1/search_candidate_set.json"), "r", encoding="utf-8")
    )
    if search.get("rotation_input") != "orientation_quaternion_normalized_xyzw" or search.get("revision") != "R1":
        print("BLOCKED_INPUT_INTEGRITY search", file=sys.stderr)
        return 2
    pose_by_id = {pose["sequence_id"]: pose for pose in planning["poses"]}
    search_by_id = {waypoint["sequence_id"]: waypoint for waypoint in search["waypoints"]}
    runs = manifest["runs"]
    if [item["run_id"] for item in runs] != list(range(21)):
        print("BLOCKED_INPUT_INTEGRITY manifest_runs", file=sys.stderr)
        return 2

    encoded = []
    report = []
    for item in runs:
        run_id = item["run_id"]
        sequence_ids = planning["process_runs"][run_id]["sequence_ids"]
        if item["selected_sha256"] != selected_sha256(item["selected"]):
            print("CHECK_HASH_MISMATCH run %d" % run_id, file=sys.stderr)
            return 2
        if len(sequence_ids) != RUN_LENGTHS[run_id]:
            print("BLOCKED_INPUT_INTEGRITY run_length %d" % run_id, file=sys.stderr)
            return 2
        selected = item["selected"]
        piecewise = item["verification"]["procedure"] == "MIDPOINT_T_0_5_THEN_SUBEDGE_AND_ORIGINAL_SAMPLE"
        points = []
        if piecewise:
            if len(selected) != 2 * len(sequence_ids) - 1:
                print("CHECK_PATH_SHAPE run %d" % run_id, file=sys.stderr)
                return 2
            for index, sequence_id in enumerate(sequence_ids):
                waypoint = selected[2 * index]
                if waypoint.get("kind") != "WAYPOINT" or waypoint.get("sequence_id") != sequence_id:
                    print("CHECK_PATH_SHAPE run %d" % run_id, file=sys.stderr)
                    return 2
                if not finite_numbers(waypoint.get("joints_rad"), 6):
                    print("CHECK_PATH_SHAPE run %d" % run_id, file=sys.stderr)
                    return 2
                points.append(waypoint["joints_rad"])
                if index + 1 < len(sequence_ids):
                    insert = selected[2 * index + 1]
                    if (
                        insert.get("kind") != "INSERT"
                        or insert.get("from_sequence_id") != sequence_id
                        or insert.get("to_sequence_id") != sequence_ids[index + 1]
                        or insert.get("t") != 0.5
                        or not finite_numbers(insert.get("joints_rad"), 6)
                    ):
                        print("CHECK_PATH_SHAPE run %d" % run_id, file=sys.stderr)
                        return 2
                    points.append(insert["joints_rad"])
        else:
            if len(selected) != len(sequence_ids):
                print("CHECK_PATH_SHAPE run %d" % run_id, file=sys.stderr)
                return 2
            for sequence_id, waypoint in zip(sequence_ids, selected):
                if waypoint.get("sequence_id") != sequence_id or not finite_numbers(waypoint.get("joints_rad"), 6):
                    print("CHECK_PATH_SHAPE run %d" % run_id, file=sys.stderr)
                    return 2
                points.append(waypoint["joints_rad"])
        segments = []
        for index in range(len(sequence_ids) - 1):
            start_id = sequence_ids[index]
            end_id = sequence_ids[index + 1]
            start_pose = pose_by_id[start_id]
            end_pose = pose_by_id[end_id]
            start_quat = search_by_id[start_id]["orientation_quaternion_normalized_xyzw"]
            end_quat = search_by_id[end_id]["orientation_quaternion_normalized_xyzw"]
            if not finite_numbers(start_pose["command_position_m"], 3) or not finite_numbers(end_pose["command_position_m"], 3):
                print("BLOCKED_INPUT_INTEGRITY position", file=sys.stderr)
                return 2
            if not finite_numbers(start_quat, 4) or not finite_numbers(end_quat, 4):
                print("BLOCKED_INPUT_INTEGRITY quaternion", file=sys.stderr)
                return 2
            segment = {
                "start_id": start_id,
                "end_id": end_id,
                "start_position": start_pose["command_position_m"],
                "end_position": end_pose["command_position_m"],
                "start_quaternion": start_quat,
                "end_quaternion": end_quat,
                "start_joints": points[2 * index] if piecewise else points[index],
                "end_joints": points[2 * index + 2] if piecewise else points[index + 1],
            }
            if piecewise:
                segment["mid_joints"] = points[2 * index + 1]
            segments.append(segment)
        encoded.append((run_id, piecewise, segments))
        report.append({
            "run_id": run_id,
            "piecewise": piecewise,
            "segment_count": len(segments),
            "expected_check_count": len(segments) * 101,
            "selected_sha256": item["selected_sha256"],
            "adopted_file": item["adopted_file"],
            "adopted_file_sha256": item["adopted_file_sha256"],
        })

    with open(out_path, "wb") as handle:
        handle.write(b"CHK1")
        handle.write(struct.pack("<I", len(encoded)))
        for run_id, piecewise, segments in encoded:
            handle.write(struct.pack("<iBI", run_id, 1 if piecewise else 0, len(segments)))
            for segment in segments:
                handle.write(struct.pack("<ii", segment["start_id"], segment["end_id"]))
                handle.write(struct.pack("<3d", *segment["start_position"]))
                handle.write(struct.pack("<3d", *segment["end_position"]))
                handle.write(struct.pack("<4d", *segment["start_quaternion"]))
                handle.write(struct.pack("<4d", *segment["end_quaternion"]))
                handle.write(struct.pack("<6d", *segment["start_joints"]))
                handle.write(struct.pack("<6d", *segment["end_joints"]))
                if piecewise:
                    handle.write(struct.pack("<6d", *segment["mid_joints"]))
    sidecar = os.path.splitext(out_path)[0] + ".targets.json"
    with open(sidecar, "w", encoding="utf-8") as handle:
        json.dump({
            "manifest_sha256": sha256(manifest_path),
            "planning_sha256": sha256(os.path.join(ROOT, "evidence/P-022/robot_planning_input.json")),
            "search_sha256": sha256(os.path.join(ROOT, "evidence/UR-ANALYTIC-IK-WAYPOINT-SURVEY-R1/search_candidate_set.json")),
            "input_sha256": sha256(out_path),
            "runs": report,
        }, handle, indent=2)
        handle.write("\n")
    print("PREPARE_OK", sha256(out_path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
