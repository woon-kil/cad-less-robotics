#!/usr/bin/env python3
"""R3 자식 프로세스를 run 6–11만 한 번씩 실행하고 부모 벽시계로 끊는다."""

import hashlib
import json
import os
import signal
import subprocess
import sys
import time

WALL_S = 1800
PRECHECK_WALL_S = 120
RUNS = [6, 7, 8, 9, 10, 11]
EXPECTED_INSERTS = 203
EXPECTED = {
    "cad_process_path.json": "04dcd594f09a18d2d996166e9feb1bfe9f20c0282758e732e94ccf59faf28114",
    "robot_planning_input.json": "23266fbcd5da565955378585fd38a893670465ca446c2ce10d0a8362bd0dd436",
    "search_candidate_set.json": "4398eae94fb305380dea8cba3bb6485ca5a70452f1a26804b2288acd54016f62",
    "ur5e.urdf": "a21bf5fb70b3a1745bf2e4816e0f43654539737ba7f2585939ec773d159778d8",
    "ur5e.srdf": "ee0d940b486bca52534d5b112c1020e0eaa542130c1301af8dc2fda28c757dbb",
}


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def launch(args, wall_s):
    started = time.monotonic()
    process = subprocess.Popen(args, start_new_session=True)
    timed_out = False
    try:
        code = process.wait(timeout=wall_s)
    except subprocess.TimeoutExpired:
        timed_out = True
        os.killpg(process.pid, signal.SIGKILL)
        code = process.wait()
    return code, time.monotonic() - started, timed_out


def rejected(reason, elapsed):
    return {
        "search": reason,
        "process_accuracy": "NOT_EVALUATED",
        "termination_reason": reason,
        "result_role": "NONE",
        "parent_elapsed_s": elapsed,
        "partial_used_as_pass": False,
        "complete": True,
    }


def accept_output(path, code, elapsed):
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
    except (OSError, json.JSONDecodeError):
        return rejected("EXECUTION_ERROR", elapsed)
    if data.get("complete") is not True:
        return rejected("EXECUTION_ERROR", elapsed)
    status = data.get("search")
    allowed = code == 0 and status in ("FOUND", "NO_PATH", "NO_INSERT_CANDIDATE")
    if allowed and status == "FOUND" and data.get("process_accuracy") != "SAMPLED_ACCURACY_PASS":
        return rejected("EXECUTION_ERROR", elapsed)
    if allowed or (code == 6 and status == "CHECK_MISMATCH"):
        data["parent_elapsed_s"] = elapsed
        data["partial_used_as_pass"] = False
        if status != "FOUND":
            data["process_accuracy"] = "NOT_EVALUATED"
        return data
    return rejected("EXECUTION_ERROR", elapsed)


def compact(record):
    inserts = record.get("inserts") or []
    valid_counts = [item.get("valid_count", 0) for item in inserts]
    return {
        "run_id": record.get("run_id"),
        "search": record.get("search"),
        "process_accuracy": record.get("process_accuracy"),
        "termination_reason": record.get("termination_reason"),
        "result_role": record.get("result_role"),
        "insert_count": record.get("insert_count"),
        "valid_count_min": min(valid_counts) if valid_counts else None,
        "in_limit_fk_violation_total": sum(
            item.get("in_limit_fk_violation_count", 0) for item in inserts
        ),
        "edge_evaluation_count": record.get("edge_evaluation_count"),
        "early_reject_count": record.get("early_reject_count"),
        "fk_check_count": record.get("fk_check_count"),
        "verification_fk_count": record.get("verification_fk_count"),
        "nonfinite_reject_count": record.get("nonfinite_reject_count"),
        "elapsed_s": record.get("elapsed_s"),
        "parent_elapsed_s": record.get("parent_elapsed_s"),
        "ladder_cost": record.get("ladder_cost"),
        "max_position_error_m": record.get("max_position_error_m"),
        "max_position_sequence_ids": record.get("max_position_sequence_ids"),
        "max_position_t": record.get("max_position_t"),
        "max_orientation_error_rad": record.get("max_orientation_error_rad"),
        "max_orientation_sequence_ids": record.get("max_orientation_sequence_ids"),
        "max_orientation_t": record.get("max_orientation_t"),
        "selected_count": len(record.get("selected") or []),
        "partial_used_as_pass": False,
    }


def main():
    if len(sys.argv) != 9:
        print(
            "usage: parent binary cad input search urdf srdf runs.bin out_dir",
            file=sys.stderr,
        )
        return 2
    binary, cad, planning, search, urdf, srdf, runs_bin, out_dir = sys.argv[1:]
    actual = {
        "cad_process_path.json": sha256(cad),
        "robot_planning_input.json": sha256(planning),
        "search_candidate_set.json": sha256(search),
        "ur5e.urdf": sha256(urdf),
        "ur5e.srdf": sha256(srdf),
    }
    for name, digest in EXPECTED.items():
        if actual[name] != digest:
            print("BLOCKED_INPUT_INTEGRITY " + name, file=sys.stderr)
            return 2
    os.makedirs(out_dir, exist_ok=True)
    precheck_path = os.path.join(out_dir, "precheck.json")
    code, elapsed, timed_out = launch(
        [binary, "precheck", urdf, srdf, runs_bin, precheck_path], PRECHECK_WALL_S
    )
    if timed_out or code != 0:
        print("PRECHECK_STOP", "TIMEOUT" if timed_out else code, file=sys.stderr)
        return 5
    precheck = json.load(open(precheck_path, "r", encoding="utf-8"))
    evaluated = int(precheck.get("valid_edge_count", 0)) + int(precheck.get("invalid_edge_count", 0))
    if precheck.get("precheck_match") is not True or precheck.get("complete") is not True or evaluated == 0:
        print("PRECHECK_MISMATCH", file=sys.stderr)
        return 5
    print("PRECHECK_MATCH", precheck.get("precheck_scope"), elapsed)
    results = []
    for run_id in RUNS:
        out_path = os.path.join(out_dir, "run_%02d.json" % run_id)
        code, elapsed, timed_out = launch(
            [binary, "run", str(run_id), urdf, srdf, runs_bin, out_path], WALL_S
        )
        if timed_out:
            record = rejected("TIMEOUT", elapsed)
            record["run_id"] = run_id
        elif code == 3:
            record = rejected("RESOURCE_LIMIT", elapsed)
            record["run_id"] = run_id
        else:
            record = accept_output(out_path, code, elapsed)
            record["run_id"] = run_id
        results.append(compact(record))
        print(
            "PARENT",
            run_id,
            results[-1].get("search"),
            results[-1].get("termination_reason"),
            results[-1].get("parent_elapsed_s"),
        )
    insert_total = sum(item.get("insert_count") or 0 for item in results)
    summary = {
        "gate": "UR-ANALYTIC-IK-PROCESS-CONNECT-R3",
        "precheck_scope": precheck.get("precheck_scope"),
        "precheck_valid_edge_count": precheck.get("valid_edge_count"),
        "precheck_invalid_edge_count": precheck.get("invalid_edge_count"),
        "precheck_absent_edge_count": precheck.get("absent_edge_count"),
        "insert_total": insert_total,
        "insert_total_matches_203": insert_total == EXPECTED_INSERTS,
        "runs": results,
        "complete": True,
    }
    with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
        handle.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
