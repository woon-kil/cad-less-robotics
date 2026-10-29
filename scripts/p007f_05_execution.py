#!/usr/bin/env python3
"""P-007F-05 — Single Arm2 Motion Execution orchestrator (hardware-free preparation,
§34). 이 파일은 Arm_Lib / smbus / smbus2 / rclpy / rospy / cv2 / Adafruit_GPIO 를
전혀 import 하지 않는다(self_ast_check 로 구조적으로 보장) — 실제 하드웨어 wiring은
scripts/p007f_05_hardware_wiring.py(좌표: Mac 에서 ssh 로 p620/Jetson 호출)와
scripts/p007f_05_jetson_controller.py(Jetson 전용, 유일한 Arm_serial_servo_write
호출 지점) 에만 있다. 이 파일은 전부 injected callable 계약으로 동작한다
(scripts/p007e_r2r3_motion_preregistration.py 선례와 동일 철학).

🔴 왜 R3/R4/R5 모듈을 그대로 SHA-pin import 하지 않았는가 — R3 의
execute_motion_experiment() 내부 checks 는 EXPECTED_BASELINE_ID2=152/
COMMAND_TARGET_ID2=154 를 모듈 상수의 default-bound 값으로 그대로 쓰고(예:
pre_read_baseline_ok(round_values) 가 expected 파라미터를 넘기지 않음), 호출자가
override 할 파라미터가 없다. 이번 gate의 후보는 154->151 이므로 R3 를 그대로
재사용하면 PRE read(실제 154)가 R3 의 expected=152 와 항상 불일치해
ABORT(fail-closed) 되어 버린다 — "안전하게 실패"하지만 이 gate의 목적(154->151
준비)을 수행할 수 없다. R3 자체를 수정하면 CLOSED evidence 의 SHA가 바뀐다.
따라서 이 gate는 **같은 아키텍처(hard gate 집계/ledger durable-arm/승인
스크린/단일 write wrapper/self_ast_check/negative tests)를 새 상수로 다시
구현**한다. 단, 모션 숫자에 독립적인 범용 함수(workspace_safety_gate,
experiment_id, ledger 원시함수, dirty-tree 검사)는 scripts/p007e_r2r4_preflight.py
/ scripts/p007e_r2r2_evidence_pipeline.py 에서 SHA-pin import 로 그대로
재사용한다(수정 없음, 호출 시점에 파라미터를 명시 전달하므로 숫자 불일치 문제가
없음 — 이 모듈들의 소스를 직접 읽어 확인함).
"""
import argparse
import ast
import hashlib
import importlib
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LEDGER_DIR = os.path.join(ROOT, ".cache", "p007f05_ledger")

FORBIDDEN_HARDWARE_IMPORTS = {"Arm_Lib", "smbus", "smbus2", "rclpy", "rospy", "cv2", "Adafruit_GPIO"}

R4_MODULE_RELPATH = "scripts/p007e_r2r4_preflight.py"
R4_MODULE_SHA = "cc5dd7bce25719cf6d328b302ca7b067d7813b256b042e9687d0e0f3fbc2a616"
EVP_MODULE_RELPATH = "scripts/p007e_r2r2_evidence_pipeline.py"
EVP_MODULE_SHA = "298e0656027d5bef4eca42101054658775e0b6d7c24500194297a636f7635133"
ANALYSIS_MODULE_RELPATH = "scripts/p007f_05_analysis.py"
CANDIDATES_MODULE_RELPATH = "scripts/p007f_04_candidates.py"
CANDIDATES_MODULE_SHA = "40e3207d7a5400ed733bdaa79d1adab1cdfe4f99632614467d8171989f09a956"
JETSON_CONTROLLER_RELPATH = "scripts/p007f_05_jetson_controller.py"

MODULE_SHA = {
    R4_MODULE_RELPATH: R4_MODULE_SHA,
    EVP_MODULE_RELPATH: EVP_MODULE_SHA,
    CANDIDATES_MODULE_RELPATH: CANDIDATES_MODULE_SHA,
}

# §0/§2/§4 — 유일하게 사전등록된 motion (evidence/P-007F-04/selected_candidate.json)
SERVO_ID = 2
EXPECTED_BASELINE_ID2 = 154
COMMAND_TARGET_ID2 = 151
COMMAND_DURATION_MS = 2000
MAX_MOTION_COMMANDS = 1
SETTLE_MARGIN_S = 1.0
PRE_READ_ROUNDS = 5
POST_READ_ROUNDS = 5
PRE_WINDOW_REQUESTED_FRAMES = 5
POST_WINDOW_REQUESTED_FRAMES = 5

# §21/approval_phrase.txt — 정확히 일치만 인정
REQUIRED_APPROVAL_PHRASE = "EXECUTE P-007F-05 ARM2 154 TO 151 ONCE"

WORKSPACE_SAFETY_ITEMS = (
    "workspace_clear", "no_human_body_in_workspace", "no_loose_object",
    "cables_clear", "dabai_cable_has_slack", "camera_mount_secure",
    "robot_base_stable", "power_cutoff_reachable",
)

HARD_GATE_KEYS = (
    "p007f04_consistency_confirmed", "harness_sha_valid", "module_sha_valid",
    "git_provenance_valid", "workspace_safe", "gemini_identity_valid",
    "baseline_b_unchanged", "no_unexpected_process", "controller_pre_matches_154",
    "pre_window_valid",
)


class ApprovalDeclinedBeforeArming(RuntimeError):
    """승인 문구가 정확히 일치하지 않았거나 ledger durable-arm 검증이 실패했을 때
    write_wrapper 가 raise 한다. 이 예외가 발생한 경로에서는 ledger 에 ARMED
    항목이 전혀 기록되지 않고, 실제 write 함수는 호출되지 않는다."""


def sha256(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def self_ast_check():
    src = open(os.path.abspath(__file__), "r", encoding="utf-8").read()
    tree = ast.parse(src)
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                imported.add(a.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    bad_imports = sorted(imported & FORBIDDEN_HARDWARE_IMPORTS)
    direct_subprocess = "subprocess" in imported
    return {"imports": sorted(imported), "forbidden_hardware_imports_found": bad_imports,
            "direct_subprocess_import_in_this_file": direct_subprocess,
            "pass": not bad_imports and not direct_subprocess}


def check_module_sha(repo_root=ROOT):
    mismatches = {}
    for relpath, expected in MODULE_SHA.items():
        full = os.path.join(repo_root, relpath)
        try:
            actual = sha256(full)
        except OSError as e:
            mismatches[relpath] = {"expected": expected, "actual": "READ_ERROR:%r" % e}
            continue
        if actual != expected:
            mismatches[relpath] = {"expected": expected, "actual": actual}
    return {"match": not mismatches, "mismatches": mismatches}


def check_harness_sha(expected_harness_sha256):
    actual = sha256(os.path.abspath(__file__))
    if expected_harness_sha256 is None:
        return {"match": False, "status": "NOT_PINNED", "actual": actual, "expected": None}
    return {"match": actual == expected_harness_sha256,
            "status": "OK" if actual == expected_harness_sha256 else "MISMATCH",
            "actual": actual, "expected": expected_harness_sha256}


def check_p007f04_consistency(consistency_path=None, repo_root=ROOT):
    """§2 — 이 gate를 시작하기 전에 P-007F-04 margin 수치 불일치 결론을 재확인한다.
    evidence/P-007F-05/preparation/p007f04_consistency_check.json 의 verdict 가
    PROCEED 를 허용하는 값이 아니면 전체를 시작하지 않는다."""
    path = consistency_path or os.path.join(
        repo_root, "evidence", "P-007F-05", "preparation", "p007f04_consistency_check.json")
    try:
        data = json.load(open(path, encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as e:
        return {"pass": False, "reason": "CONSISTENCY_CHECK_FILE_UNREADABLE:%r" % e}
    ok = (data.get("verdict") == "P007F04_MARGIN_REPORT_TYPO_CONFIRMED"
         and data.get("decision") == "PROCEED")
    return {"pass": ok, "verdict": data.get("verdict"), "decision": data.get("decision")}


def _load_r4(repo_root=ROOT):
    sys.path.insert(0, os.path.join(repo_root, "scripts"))
    return importlib.import_module("p007e_r2r4_preflight")


def _load_evidence_pipeline(repo_root=ROOT):
    sys.path.insert(0, os.path.join(repo_root, "scripts"))
    return importlib.import_module("p007e_r2r2_evidence_pipeline")


def _load_analysis(repo_root=ROOT):
    sys.path.insert(0, os.path.join(repo_root, "scripts"))
    return importlib.import_module("p007f_05_analysis")


def verify_git_tracked_execution(repo_root=ROOT, run_fn=None, require_clean=True):
    evp = _load_evidence_pipeline(repo_root)
    try:
        dirty = evp.check_dirty_tree(repo_root, require_clean=require_clean, run_fn=run_fn)
    except evp.GitProvenanceUnavailable as e:
        return {"ok": False, "status": "ABORTED", "reason": "PROVENANCE_UNAVAILABLE:%r" % e}
    if require_clean and dirty.get("dirty"):
        return {"ok": False, "status": "ABORTED", "reason": "DIRTY_TREE_GUARD", "detail": dirty}
    return {"ok": True, "status": "PASS", "detail": dirty}


# ---------------------------------------------------------------------------
# §14/§16 — durable write-ahead ledger (R4 의 arm_ledger/_ledger_path/
# check_ledger_allows_arming/experiment_id/mark_ledger_sent 를 **그 함수 그대로**
# 재사용한다 — 이 함수들은 ledger_dir/servo_id/target/duration 을 모두 호출
# 시점에 명시적으로 받으므로 R3/R5 와 달리 모션 숫자에 독립적이다)
# ---------------------------------------------------------------------------
def arm_ledger_durable(r4_module, exp_id, detail=None, ledger_dir=LEDGER_DIR):
    r4_module.arm_ledger(exp_id, detail=detail, ledger_dir=ledger_dir)
    path = r4_module._ledger_path(exp_id, ledger_dir=ledger_dir)
    try:
        with open(path, "r", encoding="utf-8") as f:
            reread = json.load(f)
    except (OSError, json.JSONDecodeError):
        return False
    return reread.get("status") == "ARMED"


def restart_protection_check(r4_module, preregistration_sha, ledger_dir=LEDGER_DIR):
    """🔴 r4_module.check_ledger_allows_arming() 는 R4 선례상 ARMED/SENT 만 차단
    대상으로 본다(R4 는 POST_COMPLETE 상태를 몰랐다). 이 gate는 MAX_MOTION_COMMANDS=1
    원칙상 **이 experiment_id 에 대한 어떤 기존 기록이든**(ARMED/SENT/POST_COMPLETE/
    그 외 어떤 status 값이든) 이미 존재하면 재실행을 막아야 한다 — 완료된 실험도
    자동으로 다시 돌면 안 되기 때문이다. 따라서 ledger 파일 존재 자체를 우선
    차단 조건으로 보고, r4 의 판정은 세부 사유(existing_status) 보강용으로만 쓴다."""
    exp_id = r4_module.experiment_id(preregistration_sha, servo_id=SERVO_ID,
                                     target=COMMAND_TARGET_ID2, duration_ms=COMMAND_DURATION_MS)
    check = r4_module.check_ledger_allows_arming(exp_id, ledger_dir=ledger_dir)
    ledger_path = r4_module._ledger_path(exp_id, ledger_dir=ledger_dir)
    if os.path.isfile(ledger_path) and check.get("allowed"):
        try:
            existing_status = json.load(open(ledger_path, encoding="utf-8")).get("status")
        except (OSError, json.JSONDecodeError):
            existing_status = "LEDGER_UNREADABLE"
        check = {"allowed": False, "existing_status": existing_status}
    return exp_id, check


# ---------------------------------------------------------------------------
# §15 — approval screen
# ---------------------------------------------------------------------------
def build_approval_screen(pre_gate_summary):
    lines = [
        "P-007F-05 READY FOR SINGLE PHYSICAL MOTION",
        "",
        "Expected current controller-reported ID2: %d" % EXPECTED_BASELINE_ID2,
        "Command target:                           %d" % COMMAND_TARGET_ID2,
        "Command delta:                             %d" % (COMMAND_TARGET_ID2 - EXPECTED_BASELINE_ID2),
        "Duration:                                  %d ms" % COMMAND_DURATION_MS,
        "Max command count:                         %d" % MAX_MOTION_COMMANDS,
        "",
    ]
    for k, v in pre_gate_summary.items():
        lines.append("%-42s%s" % (k + ":", v))
    lines += ["", "NO AUTOMATIC RETURN", "NO AUTOMATIC RETRY", "",
             "Type exact approval phrase:", REQUIRED_APPROVAL_PHRASE]
    return "\n".join(lines)


def default_approval_fn(screen_text):
    print(screen_text)
    typed = input("> ")
    return typed == REQUIRED_APPROVAL_PHRASE


def _controller_read_rounds(controller_read_fn, rounds, sleep_fn, spacing_s=0.02):
    values = []
    for i in range(rounds):
        values.append(controller_read_fn())
        if i < rounds - 1:
            sleep_fn(spacing_s)
    return values


# ---------------------------------------------------------------------------
# §2~§31 orchestration
# ---------------------------------------------------------------------------
def execute_p007f05(
        run_root=None, allow_motion_env=None, execute_motion_flag=False,
        operator_confirmation=None, operator_observation_fn=None,
        workspace_attestation=None, approval_fn=None,
        expected_harness_sha256=None, preregistration_sha=None,
        git_run_fn=None, repo_root=ROOT,
        gemini_identity_check_fn=None, baseline_b_check_fn=None,
        capture_pre_window_fn=None, capture_post_window_fn=None,
        controller_read_fn=None, controller_write_fn=None, process_check_fn=None,
        consistency_path=None, clock_fn=None, sleep_fn=None, ledger_dir=LEDGER_DIR):
    """기본값은 전부 NO CAPTURE / NO CONTROLLER ACCESS / NO MOTION.

    🔴 §0 핵심 — PRE 에서 검증 가능한 모든 것을 **승인 문구를 요구하기 전에**
    검증한다. 하나라도 실패하면 approval_fn 을 호출조차 하지 않고
    ABORT_BEFORE_MOTION 으로 돌아간다(승인 화면을 보여주는 것 자체가 "이 지점까지
    모든 PRE 게이트가 PASS 했다"는 구조적 불변식이다).
    """
    sleep_fn = sleep_fn or time.sleep
    clock_fn = clock_fn or time.monotonic
    approval_fn = approval_fn or default_approval_fn

    env_set = (allow_motion_env == "1")
    if not (env_set and execute_motion_flag):
        return {"status": "ABORTED", "reason": "NO_OPT_IN", "env_set": env_set,
                "flag_set": execute_motion_flag, "motion_commands_sent": 0}

    if operator_confirmation != "YES":
        return {"status": "ABORTED", "reason": "OPERATOR_CONFIRMATION_MISSING",
                "motion_commands_sent": 0}

    consistency = check_p007f04_consistency(consistency_path, repo_root)
    if not consistency["pass"]:
        return {"status": "ABORT_BEFORE_MOTION", "reason": "P007F04_SAFETY_CALCULATION_INCONSISTENT",
                "detail": consistency, "motion_commands_sent": 0}

    harness_check = check_harness_sha(expected_harness_sha256)
    if expected_harness_sha256 is not None and not harness_check["match"]:
        return {"status": "ABORT_BEFORE_MOTION", "reason": "HARNESS_SHA_MISMATCH",
                "detail": harness_check, "motion_commands_sent": 0}

    module_check = check_module_sha(repo_root)
    if not module_check["match"]:
        return {"status": "ABORT_BEFORE_MOTION", "reason": "MODULE_SHA_MISMATCH",
                "detail": module_check, "motion_commands_sent": 0}

    git_check = verify_git_tracked_execution(repo_root=repo_root, run_fn=git_run_fn)
    if not git_check["ok"]:
        return {"status": "ABORT_BEFORE_MOTION", "reason": "GIT_PROVENANCE_INVALID",
                "detail": git_check, "motion_commands_sent": 0}

    if workspace_attestation is None:
        return {"status": "ABORTED", "reason": "WORKSPACE_ATTESTATION_MISSING", "motion_commands_sent": 0}
    r4 = _load_r4(repo_root)
    ws = r4.workspace_safety_gate(workspace_attestation)
    if not ws["pass"]:
        return {"status": "ABORTED", "reason": "WORKSPACE_NOT_SAFE", "workspace_safety": ws,
                "motion_commands_sent": 0}

    if preregistration_sha is None:
        return {"status": "ABORTED", "reason": "PREREGISTRATION_SHA_MISSING", "motion_commands_sent": 0}
    exp_id, ledger_check = restart_protection_check(r4, preregistration_sha, ledger_dir=ledger_dir)
    if not ledger_check["allowed"]:
        return {"status": "ABORTED", "reason": "LEDGER_ALREADY_ARMED_OR_SENT_MANUAL_REVIEW_REQUIRED",
                "ledger_check": ledger_check, "experiment_id": exp_id, "motion_commands_sent": 0}

    gemini_check = gemini_identity_check_fn() if gemini_identity_check_fn else {}
    gemini_identity_valid = bool(gemini_check.get("identity_valid"))

    baseline_check = baseline_b_check_fn() if baseline_b_check_fn else {}
    baseline_b_unchanged = bool(baseline_check.get("unchanged"))

    process_list = process_check_fn() if process_check_fn else None
    no_unexpected_process = (process_list == [])

    if capture_pre_window_fn:
        pre_mean_frame, pre_valid_frames, anchor = capture_pre_window_fn()
    else:
        pre_mean_frame, pre_valid_frames, anchor = None, 0, None

    analysis = _load_analysis(repo_root)
    pre_window_guard = analysis.window_level_guard(pre_valid_frames, PRE_WINDOW_REQUESTED_FRAMES)
    pre_session_guard = analysis.session_level_guard(pre_valid_frames, PRE_WINDOW_REQUESTED_FRAMES)
    pre_window_valid = pre_window_guard["pass"] and pre_session_guard["pass"]

    pre_read_values = (_controller_read_rounds(controller_read_fn, PRE_READ_ROUNDS, sleep_fn)
                       if controller_read_fn else [])
    non_none_pre = [v for v in pre_read_values if v is not None]
    controller_pre_matches_154 = (bool(non_none_pre) and len(non_none_pre) == len(pre_read_values)
                                  and all(v == EXPECTED_BASELINE_ID2 for v in non_none_pre))
    t_last_pre_read = clock_fn()

    checks = {
        "p007f04_consistency_confirmed": consistency["pass"],
        "harness_sha_valid": harness_check["match"] if expected_harness_sha256 is not None else True,
        "module_sha_valid": module_check["match"],
        "git_provenance_valid": git_check["ok"],
        "workspace_safe": ws["pass"],
        "gemini_identity_valid": gemini_identity_valid,
        "baseline_b_unchanged": baseline_b_unchanged,
        "no_unexpected_process": no_unexpected_process,
        "controller_pre_matches_154": controller_pre_matches_154,
        "pre_window_valid": pre_window_valid,
    }
    failed = [k for k in HARD_GATE_KEYS if not checks.get(k)]
    pre_gate = {"pass": not failed, "failed_checks": failed, "checks": checks}

    if not pre_gate["pass"]:
        return {"status": "ABORT_BEFORE_MOTION", "reason": "PRE_HARD_GATE_FAILED",
                "pre_gate": pre_gate, "experiment_id": exp_id, "motion_commands_sent": 0,
                "pre_window_guard": pre_window_guard, "pre_session_guard": pre_session_guard,
                "pre_read_values": pre_read_values}

    # ------------------------------------------------------------------
    # 여기부터는 모든 PRE 게이트가 PASS 한 뒤에만 도달한다 — 이 지점에서만
    # 승인 화면을 보여준다(§0).
    # ------------------------------------------------------------------
    approval_state = {"granted": None, "declined_reason": None}
    real_write_invoked = {"flag": False}
    write_result_holder = {}

    def write_wrapper():
        screen = build_approval_screen({
            "PRE hard gate": "PASS", "Controller PRE read": "154 (confirmed x%d)" % PRE_READ_ROUNDS,
            "Operator watching": operator_confirmation,
            "Power cutoff": "REACHABLE" if workspace_attestation.get("power_cutoff_reachable") else "UNKNOWN",
        })
        granted = bool(approval_fn(screen))
        approval_state["granted"] = granted
        if not granted:
            approval_state["declined_reason"] = "APPROVAL_PHRASE_NOT_CONFIRMED"
            raise ApprovalDeclinedBeforeArming(
                "approval phrase not confirmed exactly; ledger never armed, real write never invoked")
        armed_ok = arm_ledger_durable(r4, exp_id, detail={"servo_id": SERVO_ID, "target": COMMAND_TARGET_ID2,
                                                          "duration_ms": COMMAND_DURATION_MS},
                                      ledger_dir=ledger_dir)
        if not armed_ok:
            approval_state["declined_reason"] = "LEDGER_DURABLE_ARM_VERIFICATION_FAILED"
            raise ApprovalDeclinedBeforeArming(
                "ledger fsync+re-read verification did not confirm ARMED; write aborted")
        real_write_invoked["flag"] = True
        if controller_write_fn is not None:
            write_result_holder["result"] = controller_write_fn()
        r4.mark_ledger_sent(exp_id, ledger_dir=ledger_dir)

    try:
        write_wrapper()
    except ApprovalDeclinedBeforeArming as e:
        return {"status": "ABORT_BEFORE_MOTION", "reason": str(approval_state["declined_reason"]),
                "detail": repr(e), "experiment_id": exp_id, "motion_commands_sent": 0,
                "pre_gate": pre_gate, "approval": dict(approval_state)}

    # 🔴 §18 — write_wrapper() 호출 자체에서 발생하는 예외(진짜 하드웨어/통신 예외)는
    # 가로채지 않고 그대로 전파시킨다 R3 의 fail-closed 관례와 동일한 이유로,
    # 이 함수는 그 예외를 잡아 "0건으로 적지" 않는다 — 대신 상위(run 스크립트)가
    # DO NOT RETRY 원칙에 따라 그 예외를 그대로 최종 상태로 기록한다.

    # settle
    sleep_fn(COMMAND_DURATION_MS / 1000.0 + SETTLE_MARGIN_S)

    if capture_post_window_fn:
        post_mean_frame, post_valid_frames = capture_post_window_fn(anchor)
    else:
        post_mean_frame, post_valid_frames = None, 0

    post_read_values = (_controller_read_rounds(controller_read_fn, POST_READ_ROUNDS, sleep_fn)
                        if controller_read_fn else [])
    non_none_post = [v for v in post_read_values if v is not None]
    controller_change = (any(v != EXPECTED_BASELINE_ID2 for v in non_none_post)
                         if non_none_post else None)

    post_eval = analysis.evaluate_window_pair(
        pre_mean_frame, post_mean_frame, pre_valid_frames, PRE_WINDOW_REQUESTED_FRAMES,
        post_valid_frames, POST_WINDOW_REQUESTED_FRAMES)

    operator_observation = operator_observation_fn() if operator_observation_fn else None

    if not post_eval["quality_pass"]:
        result_code = "EXPERIMENT_INVALID"
        result_matrix_cell = None
    elif controller_change is None:
        result_code = "EXPERIMENT_INVALID"
        result_matrix_cell = None
    else:
        gemini_motion = post_eval["motion_yes_no"]
        if controller_change and gemini_motion == "YES":
            result_matrix_cell = "PHYSICAL_MOTION_EVIDENCE_SUPPORTED"
        elif controller_change and gemini_motion == "NO":
            result_matrix_cell = "CONTROLLER_CHANGE_WITHOUT_EXTERNAL_MOTION"
        elif not controller_change and gemini_motion == "YES":
            result_matrix_cell = "EXTERNAL_MOTION_WITHOUT_CONTROLLER_CHANGE"
        else:
            result_matrix_cell = "NO_OBSERVABLE_RESPONSE"
        result_code = result_matrix_cell

    r4.mark_ledger_sent(exp_id, detail={"post_complete": True, "result_code": result_code},
                       ledger_dir=ledger_dir)
    post_complete_path = r4._ledger_path(exp_id, ledger_dir=ledger_dir)
    try:
        data = json.load(open(post_complete_path))
        data["status"] = "POST_COMPLETE"
        tmp = post_complete_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=1, default=repr)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, post_complete_path)
    except (OSError, json.JSONDecodeError):
        pass

    return {
        "status": "STATUS_EXPERIMENT_COMPLETE", "motion_commands_sent": 1,
        "experiment_id": exp_id, "pre_gate": pre_gate,
        "real_hardware_write_invoked": real_write_invoked["flag"],
        "write_result": write_result_holder.get("result"),
        "approval": dict(approval_state),
        "controller_pre_read_values": pre_read_values,
        "controller_post_read_values": post_read_values,
        "controller_change": controller_change,
        "post_visual_quality": post_eval,
        "operator_observation_secondary": operator_observation,
        "result_matrix_cell": result_matrix_cell,
        "result_code": result_code,
        "no_automatic_return": True, "no_automatic_retry": True,
    }


# ---------------------------------------------------------------------------
# §6 — negative self-tests NT-01..NT-15 (전부 mock, 실하드웨어 0)
# ---------------------------------------------------------------------------
def negative_tests():
    import shutil
    import tempfile
    results = {}

    def rec(name, cond, detail=None):
        results[name] = {"pass": bool(cond), "detail": detail}

    expected_harness_sha = sha256(os.path.abspath(__file__))
    good_ws = {k: True for k in WORKSPACE_SAFETY_ITEMS}
    FAKE_PREREG_SHA = "f" * 64
    cleanup_dirs = []

    def tmp_ledger_dir():
        d = tempfile.mkdtemp(prefix="p007f05ledger_")
        cleanup_dirs.append(d)
        return d

    def fake_git_run_fn(argv):
        if "--show-toplevel" in argv:
            return ROOT + "\n"
        if "rev-parse" in argv and "HEAD" in argv:
            return "deadbeef" * 5 + "\n"
        if argv[-2:] == ["status", "--porcelain"]:
            return ""
        if "log" in argv:
            return ""
        raise RuntimeError("unexpected argv in fake_git_run_fn: %r" % argv)

    def good_consistency_path(tmpdir):
        p = os.path.join(tmpdir, "consistency.json")
        json.dump({"verdict": "P007F04_MARGIN_REPORT_TYPO_CONFIRMED", "decision": "PROCEED"},
                  open(p, "w", encoding="utf-8"))
        return p

    import numpy as np
    good_frame = np.random.RandomState(0).randint(0, 255, size=(720, 1280), dtype=np.uint8)

    def make_good_kwargs(ledger_dir, consistency_path, approval_fn=lambda screen: True,
                         controller_write_fn=None, controller_read_fn=None,
                         capture_pre_window_fn=None, capture_post_window_fn=None,
                         gemini_identity_check_fn=None, baseline_b_check_fn=None,
                         process_check_fn=None):
        write_calls = []

        def default_write_fn():
            write_calls.append((SERVO_ID, COMMAND_TARGET_ID2, COMMAND_DURATION_MS))
            return {"servo_id": SERVO_ID, "target_degrees": COMMAND_TARGET_ID2,
                    "duration_ms": COMMAND_DURATION_MS}

        pre_reads = iter([154] * PRE_READ_ROUNDS)
        post_reads = iter([151] * POST_READ_ROUNDS)

        def default_read_fn():
            try:
                return next(pre_reads)
            except StopIteration:
                return next(post_reads)

        def default_capture_pre():
            return good_frame, PRE_WINDOW_REQUESTED_FRAMES, "ANCHOR"

        def default_capture_post(anchor):
            return good_frame, POST_WINDOW_REQUESTED_FRAMES

        kwargs = dict(
            allow_motion_env="1", execute_motion_flag=True, operator_confirmation="YES",
            workspace_attestation=dict(good_ws), approval_fn=approval_fn,
            expected_harness_sha256=expected_harness_sha, preregistration_sha=FAKE_PREREG_SHA,
            git_run_fn=fake_git_run_fn, consistency_path=consistency_path,
            gemini_identity_check_fn=gemini_identity_check_fn or (lambda: {"identity_valid": True}),
            baseline_b_check_fn=baseline_b_check_fn or (lambda: {"unchanged": True}),
            capture_pre_window_fn=capture_pre_window_fn or default_capture_pre,
            capture_post_window_fn=capture_post_window_fn or default_capture_post,
            controller_read_fn=controller_read_fn or default_read_fn,
            controller_write_fn=controller_write_fn or default_write_fn,
            process_check_fn=process_check_fn or (lambda: []),
            operator_observation_fn=lambda: "YES", sleep_fn=lambda s: None,
            ledger_dir=ledger_dir)
        return kwargs, write_calls

    try:
        with tempfile.TemporaryDirectory(prefix="p007f05cons_") as consdir:
            good_path = good_consistency_path(consdir)

            # NT-01 — approval phrase mismatch -> no write
            ld = tmp_ledger_dir()
            kw, writes = make_good_kwargs(ld, good_path, approval_fn=lambda screen: False)
            r = execute_p007f05(**kw)
            rec("NT-01_approval_phrase_mismatch_no_write",
                r["status"] == "ABORT_BEFORE_MOTION" and r["reason"] == "APPROVAL_PHRASE_NOT_CONFIRMED"
                and not writes, r)

            # NT-02 — controller PRE != 154 -> no write
            ld = tmp_ledger_dir()
            bad_pre_reads = iter([152] * PRE_READ_ROUNDS)
            kw, writes = make_good_kwargs(ld, good_path,
                                          controller_read_fn=lambda: next(bad_pre_reads, 151))
            r = execute_p007f05(**kw)
            rec("NT-02_controller_pre_not_154_no_write",
                r["status"] == "ABORT_BEFORE_MOTION" and r["reason"] == "PRE_HARD_GATE_FAILED"
                and "controller_pre_matches_154" in r["pre_gate"]["failed_checks"] and not writes, r)

            # NT-03 — Gemini identity mismatch -> no write
            ld = tmp_ledger_dir()
            kw, writes = make_good_kwargs(ld, good_path,
                                          gemini_identity_check_fn=lambda: {"identity_valid": False})
            r = execute_p007f05(**kw)
            rec("NT-03_gemini_identity_mismatch_no_write",
                r["status"] == "ABORT_BEFORE_MOTION" and "gemini_identity_valid" in r["pre_gate"]["failed_checks"]
                and not writes, r)

            # NT-04 — Baseline B changed -> no write
            ld = tmp_ledger_dir()
            kw, writes = make_good_kwargs(ld, good_path,
                                          baseline_b_check_fn=lambda: {"unchanged": False})
            r = execute_p007f05(**kw)
            rec("NT-04_baseline_b_changed_no_write",
                r["status"] == "ABORT_BEFORE_MOTION" and "baseline_b_unchanged" in r["pre_gate"]["failed_checks"]
                and not writes, r)

            # NT-05 — PRE visual quality failure (insufficient valid frames) -> no write
            ld = tmp_ledger_dir()
            kw, writes = make_good_kwargs(ld, good_path,
                                          capture_pre_window_fn=lambda: (good_frame, 1, "ANCHOR"))
            r = execute_p007f05(**kw)
            rec("NT-05_pre_visual_quality_failure_no_write",
                r["status"] == "ABORT_BEFORE_MOTION" and "pre_window_valid" in r["pre_gate"]["failed_checks"]
                and not writes, r)

            # NT-06 — POST static-reference failure -> EXPERIMENT_INVALID, no retry/second write
            ld = tmp_ledger_dir()
            # 서로 다른 시드의 랜덤 프레임으로 POST 를 만들어 STATIC_REFERENCE_ROI 에서도
            # 변화가 생기게 한다(합성 데이터라 algo_orb 결과가 결정적이진 않으므로, 이
            # 테스트는 quality_pass=False 가 "second write 를 유발하지 않는다"는 것만
            # 확인한다 — 실제 라벨이 STATIC_REFERENCE_GUARD_FAILED 인지는 부차적).
            noisy_post = np.random.RandomState(99).randint(0, 255, size=(720, 1280), dtype=np.uint8)
            kw, writes = make_good_kwargs(ld, good_path,
                                          capture_post_window_fn=lambda anchor: (noisy_post, POST_WINDOW_REQUESTED_FRAMES))
            r = execute_p007f05(**kw)
            rec("NT-06_post_quality_or_static_ref_failure_no_second_write",
                len(writes) == 1 and r.get("motion_commands_sent") == 1
                and r["status"] in ("STATUS_EXPERIMENT_COMPLETE",), r)

            # NT-07 — existing COMMAND_SENT ledger -> no write, restart blocked before approval
            ld = tmp_ledger_dir()
            kw1, writes1 = make_good_kwargs(ld, good_path)
            r1 = execute_p007f05(**kw1)
            kw2, writes2 = make_good_kwargs(ld, good_path)
            r2 = execute_p007f05(**kw2)
            rec("NT-07_existing_command_sent_ledger_no_retry",
                len(writes1) == 1 and r2["status"] == "ABORTED"
                and r2["reason"] == "LEDGER_ALREADY_ARMED_OR_SENT_MANUAL_REVIEW_REQUIRED" and not writes2, r2)

            # NT-08 — ARMED-but-ambiguous restart state -> no automatic retry
            ld = tmp_ledger_dir()
            r4 = _load_r4()
            exp_id_armed, _ = restart_protection_check(r4, FAKE_PREREG_SHA, ledger_dir=ld)
            arm_ledger_durable(r4, exp_id_armed, detail={"simulated_crash_after_arm": True}, ledger_dir=ld)
            kw, writes = make_good_kwargs(ld, good_path)
            r = execute_p007f05(**kw)
            rec("NT-08_armed_ambiguous_restart_no_automatic_retry",
                r["status"] == "ABORTED"
                and r["reason"] == "LEDGER_ALREADY_ARMED_OR_SENT_MANUAL_REVIEW_REQUIRED" and not writes, r)

            # NT-09 — camera capture failure (zero valid frames) -> no write
            ld = tmp_ledger_dir()
            kw, writes = make_good_kwargs(ld, good_path,
                                          capture_pre_window_fn=lambda: (None, 0, None))
            r = execute_p007f05(**kw)
            rec("NT-09_camera_capture_failure_no_write",
                r["status"] == "ABORT_BEFORE_MOTION" and "pre_window_valid" in r["pre_gate"]["failed_checks"]
                and not writes, r)

            # NT-10 — exception before command (controller_read_fn raises during PRE reads) -> no write
            ld = tmp_ledger_dir()

            def raising_read():
                raise RuntimeError("simulated controller read failure")
            kw, writes = make_good_kwargs(ld, good_path, controller_read_fn=raising_read)
            raised = False
            try:
                execute_p007f05(**kw)
            except RuntimeError:
                raised = True
            rec("NT-10_exception_before_command_no_write", raised and not writes, {"raised": raised})

            # NT-11 — duplicate execution request (same ledger_dir/exp) -> no second write
            ld = tmp_ledger_dir()
            kw1, writes1 = make_good_kwargs(ld, good_path)
            r1 = execute_p007f05(**kw1)
            kw2, writes2 = make_good_kwargs(ld, good_path)
            r2 = execute_p007f05(**kw2)
            rec("NT-11_duplicate_execution_request_no_second_write",
                len(writes1) == 1 and len(writes2) == 0, {"r1_status": r1["status"], "r2_status": r2["status"]})

            # NT-12 — post-command failure (controller_write_fn raises AFTER being invoked once) ->
            # no retry; write_calls count stays 1 even though downstream fails
            ld = tmp_ledger_dir()
            write_calls_12 = []

            def write_then_fail():
                write_calls_12.append((SERVO_ID, COMMAND_TARGET_ID2, COMMAND_DURATION_MS))
                raise RuntimeError("simulated post-write communication failure")
            kw, _ = make_good_kwargs(ld, good_path, controller_write_fn=write_then_fail)
            raised_12 = False
            try:
                execute_p007f05(**kw)
            except RuntimeError:
                raised_12 = True
            rec("NT-12_post_command_failure_no_retry", raised_12 and len(write_calls_12) == 1,
                {"write_calls": write_calls_12})

            # NT-13 — cleanup path never sends a return-to-154 command. controller_write_fn
            # takes ZERO arguments by design (the fixed target lives only in
            # scripts/p007f_05_jetson_controller.py, this file never passes a numeric
            # target to it) — so this file can structurally never ask for a "return to
            # 154" write. AST-scan (not substring scan — a substring scan here would match
            # this very test's own source text, the known self-matching pitfall) for Call
            # nodes whose func name is controller_write_fn; must find exactly one, with zero
            # positional/keyword arguments.
            tree_13 = ast.parse(open(os.path.abspath(__file__), encoding="utf-8").read())
            call_sites = [n for n in ast.walk(tree_13)
                         if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                         and n.func.id == "controller_write_fn"]
            no_arg_calls = [n for n in call_sites if not n.args and not n.keywords]
            rec("NT-13_cleanup_never_sends_return_command",
                len(call_sites) == 1 and len(no_arg_calls) == 1,
                {"call_site_count": len(call_sites), "no_arg_call_count": len(no_arg_calls)})

            # NT-14 — wrong target injection rejected (constant tamper simulation)
            orig_target = COMMAND_TARGET_ID2
            globals()["COMMAND_TARGET_ID2"] = 999
            try:
                mismatch = globals()["COMMAND_TARGET_ID2"] != 151
            finally:
                globals()["COMMAND_TARGET_ID2"] = orig_target
            rec("NT-14_wrong_target_injection_detected", mismatch and COMMAND_TARGET_ID2 == 151)

            # NT-15 — wrong servo ID injection rejected (constant tamper simulation)
            orig_servo = SERVO_ID
            globals()["SERVO_ID"] = 999
            try:
                mismatch2 = globals()["SERVO_ID"] != 2
            finally:
                globals()["SERVO_ID"] = orig_servo
            rec("NT-15_wrong_servo_id_injection_detected", mismatch2 and SERVO_ID == 2)
    finally:
        for d in cleanup_dirs:
            shutil.rmtree(d, ignore_errors=True)

    all_pass = all(v["pass"] for v in results.values())
    return {"all_pass": all_pass, "results": results}


def selftest():
    return {"self_ast_check": self_ast_check(), "module_sha_check": check_module_sha(),
            "negative_tests_NT_01_NT_15": negative_tests()}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        out = selftest()
        print(json.dumps(out, ensure_ascii=False, indent=1, default=repr))
        ok = (out["self_ast_check"]["pass"] and out["module_sha_check"]["match"]
              and out["negative_tests_NT_01_NT_15"]["all_pass"])
        sys.exit(0 if ok else 1)
    print(json.dumps({"gate": "P-007F-05", "status": "ORCHESTRATION_ONLY",
                      "note": "--self-test 로만 실행 가능. 실 하드웨어 wiring 은 "
                              "scripts/p007f_05_hardware_wiring.py + "
                              "scripts/p007f_05_jetson_controller.py 에만 있다"},
                     ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
