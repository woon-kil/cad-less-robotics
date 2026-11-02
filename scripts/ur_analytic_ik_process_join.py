#!/usr/bin/env python3
"""R1 탐색 후보와 원본 command_position_m을 sequence_id로 결합한다.

탐색은 하지 않는다. 해시나 소속이 어긋나면 출력 파일을 만들지 않는다.
"""

import hashlib
import json
import math
import struct
import sys

EXPECTED = {
    "cad_process_path.json": "04dcd594f09a18d2d996166e9feb1bfe9f20c0282758e732e94ccf59faf28114",
    "robot_planning_input.json": "23266fbcd5da565955378585fd38a893670465ca446c2ce10d0a8362bd0dd436",
    "search_candidate_set.json": "4398eae94fb305380dea8cba3bb6485ca5a70452f1a26804b2288acd54016f62",
    "ur5e.urdf": "a21bf5fb70b3a1745bf2e4816e0f43654539737ba7f2585939ec773d159778d8",
    "ur5e.srdf": "ee0d940b486bca52534d5b112c1020e0eaa542130c1301af8dc2fda28c757dbb",
}
RUN_LENGTHS = [35, 34, 34, 34, 34, 34, 34, 35, 35, 35, 35, 35, 35, 34, 34, 34, 34, 34, 34, 34, 34]


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def fail(message):
    print("BLOCKED_INPUT_INTEGRITY " + message, file=sys.stderr)
    return 2


def same_components(left, right):
    return len(left) == len(right) and all(item == other for item, other in zip(left, right))


def main():
    if len(sys.argv) != 7:
        print(
            "usage: ur_analytic_ik_process_join.py cad input search urdf srdf out.bin",
            file=sys.stderr,
        )
        return 2
    cad_path, input_path, search_path, urdf_path, srdf_path, out_path = sys.argv[1:]
    actual = {
        "cad_process_path.json": sha256(cad_path),
        "robot_planning_input.json": sha256(input_path),
        "search_candidate_set.json": sha256(search_path),
        "ur5e.urdf": sha256(urdf_path),
        "ur5e.srdf": sha256(srdf_path),
    }
    for name, digest in EXPECTED.items():
        if actual[name] != digest:
            return fail(name)

    planning = json.load(open(input_path, "r", encoding="utf-8"))
    search = json.load(open(search_path, "r", encoding="utf-8"))
    if search.get("rotation_input") != "orientation_quaternion_normalized_xyzw":
        return fail("rotation_input")
    if search.get("revision") != "R1":
        return fail("revision")

    poses = planning["poses"]
    if len(poses) != 741:
        return fail("pose_count")
    pose_by_id = {}
    for pose in poses:
        sequence_id = pose["sequence_id"]
        if sequence_id in pose_by_id:
            return fail("duplicate_input_sequence")
        pose_by_id[sequence_id] = pose
    if set(pose_by_id) != set(range(741)):
        return fail("input_sequence_range")

    search_by_id = {}
    for waypoint in search["waypoints"]:
        sequence_id = waypoint["sequence_id"]
        if sequence_id in search_by_id:
            return fail("duplicate_search_sequence")
        search_by_id[sequence_id] = waypoint
    if set(search_by_id) != set(range(741)):
        return fail("search_sequence_range")

    runs = planning["process_runs"]
    if len(runs) != 21:
        return fail("run_count")
    assigned = {}
    payload = []
    for index, run in enumerate(runs):
        if run["run_id"] != index or len(run["sequence_ids"]) != RUN_LENGTHS[index]:
            return fail("run_inventory")
        encoded = []
        for sequence_id in run["sequence_ids"]:
            if sequence_id in assigned:
                return fail("duplicate_run_member")
            pose = pose_by_id[sequence_id]
            waypoint = search_by_id[sequence_id]
            if pose["segment_type"] != "PROCESS" or waypoint["segment_type"] != "PROCESS":
                return fail("segment_type")
            if waypoint["process_run_id"] != index:
                return fail("process_run_id")
            if not same_components(
                waypoint["orientation_quaternion_xyzw"], pose["orientation_quaternion"]
            ):
                return fail("quaternion " + str(sequence_id))
            normalized = waypoint["orientation_quaternion_normalized_xyzw"]
            if normalized is None or len(normalized) != 4:
                return fail("normalized_quaternion")
            if any(not math.isfinite(component) for component in normalized):
                return fail("normalized_nan")
            candidates = waypoint["candidates"]
            if len(candidates) != 256:
                return fail("candidate_count")
            encoded_candidates = []
            for candidate in candidates:
                source = candidate["source"]
                if source == "solver":
                    source_code = 0
                elif source == "redundant_2pi":
                    source_code = 1
                else:
                    return fail("source")
                joints = candidate["joints_rad"]
                if len(joints) != 6 or any(not math.isfinite(value) for value in joints):
                    return fail("joints")
                encoded_candidates.append((source_code, int(candidate["kinematic_class_id"]), joints))
            position = pose["command_position_m"]
            if len(position) != 3 or any(not math.isfinite(value) for value in position):
                return fail("position")
            encoded.append((sequence_id, position, normalized, encoded_candidates))
            assigned[sequence_id] = index
        payload.append(encoded)

    transitions = set(planning["transition_sequence_ids"])
    if len(transitions) != 20 or transitions & set(assigned):
        return fail("transition_separation")
    if len(assigned) != 721:
        return fail("process_count")

    with open(out_path, "wb") as handle:
        handle.write(b"URPC1")
        handle.write(struct.pack("<I", 21))
        for run_id, waypoints in enumerate(payload):
            handle.write(struct.pack("<iI", run_id, len(waypoints)))
            for sequence_id, position, normalized, candidates in waypoints:
                handle.write(struct.pack("<i", sequence_id))
                handle.write(struct.pack("<3d", *position))
                handle.write(struct.pack("<4d", *normalized))
                handle.write(struct.pack("<I", len(candidates)))
                for source_code, class_id, joints in candidates:
                    handle.write(struct.pack("<iB", class_id, source_code))
                    handle.write(struct.pack("<6d", *joints))
    print("JOIN_OK runs 21 waypoints 721")
    return 0


if __name__ == "__main__":
    sys.exit(main())
