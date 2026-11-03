#!/usr/bin/env python3
"""전체 layered graph 1회. 부모 벽시계 757초, 주소 8 GiB, 재시도 0."""

import hashlib
import json
import os
import resource
import struct
import subprocess
import time
from pathlib import Path

WALL_S = 757
LIMIT = 8589934592
JSON_DIR = Path("/tmp/full-graph/json")
OUT = Path("/tmp/full-graph/out")
HASHES = {
    6: "cda4368e5f6eb8fe1b289ffd509eb37b7e2f4933b9503780097bdf42e179bcb8",
    7: "973fa70e38412fe4e2bd952a898e2e35eda3186e207b08c4740f2641c024314a",
    8: "1e47b542549f3e40b9ee2b7202999047850dfc39d581cb08575be0bc2fdcd709",
    9: "ced6759a4b784877274b23a89b34442e485910347089d732555c8e2aa1cd9a86",
    10: "721fec3b096d0ae0c53d1890e4288b97319619f73bd506831d1aa7b9041ea975",
    11: "d513e81a72dd1acd7afae1bc73b12cfc4154c95cb61fa3f319badae85f8c6539",
}
COUNTS = {6: 33, 7: 34, 8: 34, 9: 34, 10: 34, 11: 34}
SPANS = {6: (211, 244), 7: (246, 280), 8: (282, 316), 9: (318, 352), 10: (354, 388), 11: (390, 424)}


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def source_code(name):
    if name == "solver":
        return 0
    if name == "redundant_2pi":
        return 1
    raise SystemExit("bad source " + name)


def write_inserts():
    blob = bytearray()
    blob += struct.pack("<4sI", b"R6IN", 6)
    total = 0
    for run_id in (6, 7, 8, 9, 10, 11):
        path = JSON_DIR / f"run_{run_id:02d}.json"
        if sha256(path) != HASHES[run_id]:
            raise SystemExit("hash mismatch " + str(run_id))
        data = json.loads(path.read_text())
        inserts = data["inserts"]
        if len(inserts) != COUNTS[run_id]:
            raise SystemExit("count " + str(run_id))
        blob += struct.pack("<ii", run_id, len(inserts))
        first, last = SPANS[run_id]
        for offset, insert in enumerate(inserts):
            from_id = insert["from_sequence_id"]
            to_id = insert["to_sequence_id"]
            if from_id != first + offset or to_id != from_id + 1 or to_id > last or insert["t"] != 0.5:
                raise SystemExit("span " + str(run_id))
            if insert["valid_count"] != 256 or len(insert["valid_candidates"]) != 256:
                raise SystemExit("candidates " + str(run_id))
            blob += struct.pack("<ii3d4dI", from_id, to_id, *insert["position_m"], *insert["orientation_quaternion_normalized_xyzw"], 256)
            for index, candidate in enumerate(insert["valid_candidates"]):
                if candidate["candidate_index"] != index:
                    raise SystemExit("index order")
                blob += struct.pack("<iB6d", candidate["kinematic_class_id"], source_code(candidate["source"]), *candidate["joints_rad"])
            total += 1
    if total != 203:
        raise SystemExit("total")
    destination = OUT / "inserts.bin"
    destination.write_bytes(blob)
    return sha256(destination), len(blob)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    digest, size = write_inserts()
    resource.setrlimit(resource.RLIMIT_AS, (LIMIT, LIMIT))
    soft, hard = resource.getrlimit(resource.RLIMIT_AS)
    (OUT / "parent_limit.json").write_text(
        json.dumps({
            "confirmed_before_exec": soft == LIMIT and hard == LIMIT,
            "address_limit_bytes": LIMIT,
            "overall_wall_s": WALL_S,
            "retry": 0,
            "leftover_transfer": False,
            "inserts_sha256": digest,
            "inserts_bytes": size,
            "json_hash_checked": True,
        }) + "\n")
    command = [
        "/tmp/full-graph/build/ur_analytic_ik_process_connect_collision",
        "fullgraph",
        "/tmp/connect-collision/src/ur5e.urdf",
        "/tmp/connect-collision/src/ur5e.srdf",
        "/tmp/descartes-light/runs.bin",
        "/tmp/connect-collision/src/mesh.bin",
        "/tmp/connect-collision/src/mesh_acceptance.json",
        "/tmp/transition-connect/out/transitions.bin",
        str(OUT / "inserts.bin"),
        str(OUT / "fullgraph.json"),
    ]
    started = time.monotonic()
    process = subprocess.Popen(command, start_new_session=True)
    wall_kill = False
    try:
        code = process.wait(timeout=WALL_S)
    except subprocess.TimeoutExpired:
        wall_kill = True
        os.killpg(process.pid, 9)
        code = process.wait()
    elapsed = time.monotonic() - started
    (OUT / "summary.json").write_text(
        json.dumps({
            "exit": code,
            "parent_elapsed_s": elapsed,
            "parent_wall_kill": wall_kill,
            "overall_wall_s": WALL_S,
            "complete": True,
        }) + "\n")
    print("PARENT_DONE", code, elapsed, wall_kill)


if __name__ == "__main__":
    main()
