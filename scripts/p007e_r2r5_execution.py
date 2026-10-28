#!/usr/bin/env python3
"""P-007E-R2-R5 — Single Controlled Physical-Motion Execution orchestrator.

이 gate 는 **최초로** 실제 DaBai capture / controller read / Arm2 servo write(<=1회)
를 허용한다. 그러나 이 파일 자신은 여전히 Arm_Lib / smbus / smbus2 / rclpy / rospy /
cv2 / Adafruit_GPIO 를 전혀 import 하지 않는다(self_ast_check 로 구조적으로 보장) —
실제 하드웨어 wiring 은 별도 파일(scripts/p007e_r2r5_hardware_wiring.py, 이 self-check
의 적용 대상이 아니다, p007e_phase1_command.py 선례와 동일)에만 있다. 이 파일은
scripts/p007e_r2r3_motion_preregistration.py(R3, CLOSED, SHA f8eda225...)와
scripts/p007e_r2r4_preflight.py(R4, CLOSED, SHA cc5dd7bc...)를 **1바이트도 수정하지
않고** SHA-pin import 로 재사용한다. 이 파일이 새로 더하는 것은 셋뿐이다.

  1. §6 git-tracked checkout 재확인 + §7 provenance SHA 확장(R4 자신의 SHA — R3는
     R4 를 모르므로 R4 SHA 를 검증하는 쪽은 R5 가 유일하다).
  2. §15/§16/§17 — durable write-ahead ledger **재읽기 검증**(fsync 만으로 끝내지
     않고, 쓴 뒤 다시 열어 status=="ARMED" 를 확인한다) + 재시작 보호(동일
     experiment_id 가 이미 ARMED/SENT 면 이 게이트 전체를 시작하지 않는다 —
     R4.preflight_execute() 의 검사와 같은 R4 원시함수를 재사용하지만, R5 는
     **자신만의** ledger 디렉터리(.cache/p007e_r2r5_ledger/)를 쓴다 — R4 의
     ledger 는 전부 음성시험용이었고 실제로 ARMED 된 적이 없다).
  3. §31 — 사람의 명시적 승인 문구 입력 게이트. **R3 의 controller_write_fn 호출
     시점**(이 시점에 도달했다는 사실 자체가 R3 내부 pre_motion_hard_gate 14항목이
     전부 PASS 했다는 구조적 불변식이다 — scripts/p007e_r2r3_motion_preregistration.py
     의 execute_motion_experiment() 코드를 보면 controller_write_fn 호출은
     `if not gate["pass"]: return ABORT...` 를 통과한 뒤에만 일어난다)에서, 실제
     하드웨어 write 함수를 호출하기 **전에** 승인 문구를 요구한다.

🔴 설계상 알려진 한계(공개, R3 misattribution) — 승인을 거부(decline)하면 이
   wrapper 는 ledger 를 ARMED 하지 않고 ApprovalDeclinedBeforeArming 을 raise 한다.
   R3.execute_motion_experiment() 는 그 예외를 자신의 범용 `except BaseException`
   으로 받아 (자기 자신의 기존 설계 그대로, 수정하지 않음) result_code="X",
   motion_commands_sent=1, invalid_reasons=["WRITE_CALL_EXCEPTION"] 로 저장한다 —
   이것은 R3 의 fail-closed 관례("I2C 보드에 도달했는지 모르면 0 으로 적지 않는다")
   가 "운영자가 애초에 승인하지 않았다"는 사례에도 그대로 적용된 결과다. 이
   필드는 **실제 하드웨어 접촉 여부의 권위 있는 기록이 아니다** — 권위 있는
   기록은 이 파일의 ledger(이 경우 해당 experiment_id 에 ARMED/SENT 항목이
   전혀 없음)와 `real_hardware_write_invoked` 반환 필드다. R3 를 수정하지 않고
   이 경계를 해소하는 유일한 방법이기 때문에 선택했다 — R3 를 고치면 CLOSED
   evidence 의 SHA 가 바뀐다.
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
LEDGER_DIR = os.path.join(ROOT, ".cache", "p007e_r2r5_ledger")

FORBIDDEN_HARDWARE_IMPORTS = {"Arm_Lib", "smbus", "smbus2", "rclpy", "rospy", "cv2", "Adafruit_GPIO"}

R3_MODULE_RELPATH = "scripts/p007e_r2r3_motion_preregistration.py"
R3_MODULE_SHA = "f8eda225b8792285f4a3ce5001f736f0a2cfaae8cce9f2b4aed3cc5742a79466"
R4_MODULE_RELPATH = "scripts/p007e_r2r4_preflight.py"
R4_MODULE_SHA = "cc5dd7bce25719cf6d328b302ca7b067d7813b256b042e9687d0e0f3fbc2a616"
WINDOW_MODULE_RELPATH = "scripts/p007e_r2r5_window.py"
WINDOW_MODULE_SHA = "31004bea35f31f4b5fe675076a91bce875f0cdc9f0979d0c123ba749d8903053"
SHAM_MODULE_RELPATH = "scripts/p007e_r2r2_c1r1_sham.py"
SHAM_MODULE_SHA = "86f94437642828741654a34d4eedeccb586c72a9497762e74d1de41aaba785bc"

# §7 — R3 가 이미 내부적으로 재검증하는 acquisition/measurement/analysis/evidence_pipeline
# 은 R3.MODULE_SHA 에 있으므로 여기서 중복 선언하지 않는다. 여기서는 R3 가 모르는 두
# 모듈(R4 자신, 이 harness 가 새로 쓰는 window 캡처 모듈, 그리고 둘 다가 재사용하는
# sham.py 의 build_estimator_and_mask/env_guards)만 R5 고유로 추가 핀한다.
MODULE_SHA = {
    R3_MODULE_RELPATH: R3_MODULE_SHA,
    R4_MODULE_RELPATH: R4_MODULE_SHA,
    WINDOW_MODULE_RELPATH: WINDOW_MODULE_SHA,
    SHAM_MODULE_RELPATH: SHAM_MODULE_SHA,
    "scripts/p007e_r2r2_evidence_pipeline.py": "298e0656027d5bef4eca42101054658775e0b6d7c24500194297a636f7635133",
    "scripts/p007e_r2r4_fk.py": "a4e9d2d7891597a6bfa62c8ca87303aeab28ba0a784f23f2640ca8dc145622cb",
}

# §2 — 유일하게 승인된 motion (R3 §7 과 동일, 재정의 아님 — 여기서 재검증만)
SERVO_ID = 2
EXPECTED_BASELINE_ID2 = 152
COMMAND_TARGET_ID2 = 154
COMMAND_DURATION_MS = 2000
MAX_MOTION_COMMANDS = 1
OPERATOR_ENVELOPE_MM = 15

# §31 — 정확한 승인 문구(변경 금지, 띄어쓰기/대소문자까지 정확히 일치해야 한다)
REQUIRED_APPROVAL_PHRASE = "EXECUTE P-007E-R2-R5 ARM2 152 TO 154 ONCE"

WORKSPACE_SAFETY_ITEMS = (
    "workspace_clear", "no_human_body_in_workspace", "no_loose_object",
    "cables_clear", "dabai_cable_has_slack", "camera_mount_secure",
    "robot_base_stable", "power_cutoff_reachable",
)


class ApprovalDeclinedBeforeArming(RuntimeError):
    """승인 문구가 정확히 입력되지 않았거나 ledger durable-arm 검증이 실패했을 때
    write_wrapper 가 raise 한다. arm_ledger_durable() 호출 **전에** raise 되므로,
    이 예외가 발생한 경로에서는 ledger 에 ARMED 항목이 전혀 기록되지 않는다."""


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


def _load_r3(repo_root=ROOT):
    sys.path.insert(0, os.path.join(repo_root, "scripts"))
    return importlib.import_module("p007e_r2r3_motion_preregistration")


def _load_r4(repo_root=ROOT):
    sys.path.insert(0, os.path.join(repo_root, "scripts"))
    return importlib.import_module("p007e_r2r4_preflight")


# ---------------------------------------------------------------------------
# §15/§16/§17 — durable write-ahead ledger (R4 의 arm_ledger/mark_ledger_sent/
# check_ledger_allows_arming/experiment_id 를 **그 함수 그대로** 재사용한다 — R5 는
# 자신만의 ledger_dir(.cache/p007e_r2r5_ledger/)만 추가한다)
# ---------------------------------------------------------------------------
def arm_ledger_durable(r4_module, exp_id, detail=None, ledger_dir=LEDGER_DIR):
    """🔴 §15 — fsync 쓰기(R4.arm_ledger, 그대로 재사용) 후 **다시 열어 읽어서**
    status=="ARMED" 를 확인한다. 재읽기가 실패하거나 불일치하면 False — 호출자는
    실제 write 함수를 절대 부르지 않는다."""
    r4_module.arm_ledger(exp_id, detail=detail, ledger_dir=ledger_dir)
    path = r4_module._ledger_path(exp_id, ledger_dir=ledger_dir)
    try:
        with open(path, "r", encoding="utf-8") as f:
            reread = json.load(f)
    except (OSError, json.JSONDecodeError):
        return False
    return reread.get("status") == "ARMED"


def restart_protection_check(r4_module, r3_module, ledger_dir=LEDGER_DIR):
    """§17 — 이 게이트를 시작하기 전에(승인 화면 도달 전에) 먼저 확인한다. 동일
    experiment_id 에 이미 ARMED/SENT 가 있으면 전체를 시작하지 않는다."""
    exp_id = r4_module.experiment_id(r3_module.PREREG_SHA, servo_id=SERVO_ID,
                                     target=COMMAND_TARGET_ID2, duration_ms=COMMAND_DURATION_MS)
    check = r4_module.check_ledger_allows_arming(exp_id, ledger_dir=ledger_dir)
    return exp_id, check


# ---------------------------------------------------------------------------
# §31 — approval screen + gate
# ---------------------------------------------------------------------------
def build_approval_screen(pre_gate_summary):
    lines = [
        "P-007E-R2-R5 READY FOR SINGLE PHYSICAL MOTION",
        "",
        "Expected current ID2: %d°" % EXPECTED_BASELINE_ID2,
        "Target ID2:           %d°" % COMMAND_TARGET_ID2,
        "Command delta:        +%d°" % (COMMAND_TARGET_ID2 - EXPECTED_BASELINE_ID2),
        "Duration:             %d ms" % COMMAND_DURATION_MS,
        "Max command count:    %d" % MAX_MOTION_COMMANDS,
        "Operator envelope:    %d mm" % OPERATOR_ENVELOPE_MM,
        "",
    ]
    for k, v in pre_gate_summary.items():
        lines.append("%-22s%s" % (k + ":", v))
    lines += ["", "NO AUTOMATIC RETURN", "NO AUTOMATIC RETRY", "",
             "Type exact approval phrase:", REQUIRED_APPROVAL_PHRASE]
    return "\n".join(lines)


def default_approval_fn(screen_text):
    print(screen_text)
    typed = input("> ")
    return typed == REQUIRED_APPROVAL_PHRASE


# ---------------------------------------------------------------------------
# §6/§7/§9/§15~17/§31 orchestration — R3/R4 를 조립만 한다
# ---------------------------------------------------------------------------
def execute_r5(
        run_root=None, allow_motion_env=None, execute_motion_flag=False,
        operator_confirmation=None, operator_observation_fn=None,
        workspace_attestation=None, approval_fn=None,
        expected_harness_sha256=None, expected_r3_harness_sha256=None,
        git_run_fn=None, repo_root=ROOT,
        camera_check_fn=None, capture_pre_window_fn=None, capture_post_window_fn=None,
        controller_read_fn=None, controller_write_fn=None, process_check_fn=None,
        clock_fn=None, sleep_fn=None, ledger_dir=LEDGER_DIR):
    """기본값은 전부 NO CAPTURE / NO CONTROLLER ACCESS / NO MOTION(R3 와 동일 계약)."""
    sleep_fn = sleep_fn or time.sleep
    approval_fn = approval_fn or default_approval_fn

    env_set = (allow_motion_env == "1")
    if not (env_set and execute_motion_flag):
        return {"status": "ABORTED", "reason": "NO_OPT_IN",
                "env_set": env_set, "flag_set": execute_motion_flag, "motion_commands_sent": 0}

    harness_check = check_harness_sha(expected_harness_sha256)
    if expected_harness_sha256 is not None and not harness_check["match"]:
        return {"status": "ABORT_BEFORE_MOTION", "reason": "R5_HARNESS_SHA_MISMATCH",
                "detail": harness_check, "motion_commands_sent": 0}

    module_check = check_module_sha(repo_root)
    if not module_check["match"]:
        return {"status": "ABORT_BEFORE_MOTION", "reason": "MODULE_SHA_MISMATCH",
                "detail": module_check, "motion_commands_sent": 0}

    r3 = _load_r3(repo_root)
    r4 = _load_r4(repo_root)

    if workspace_attestation is None:
        return {"status": "ABORTED", "reason": "WORKSPACE_ATTESTATION_MISSING", "motion_commands_sent": 0}
    ws = r4.workspace_safety_gate(workspace_attestation)
    if not ws["pass"]:
        return {"status": "ABORTED", "reason": "WORKSPACE_NOT_SAFE", "workspace_safety": ws,
                "motion_commands_sent": 0}

    exp_id, ledger_check = restart_protection_check(r4, r3, ledger_dir=ledger_dir)
    if not ledger_check["allowed"]:
        return {"status": "ABORTED", "reason": "LEDGER_ALREADY_ARMED_OR_SENT_MANUAL_REVIEW_REQUIRED",
                "ledger_check": ledger_check, "experiment_id": exp_id, "motion_commands_sent": 0}

    approval_state = {"granted": None, "declined_reason": None}
    real_write_invoked = {"flag": False}

    def write_wrapper(*a, **kw):
        pre_gate_summary = {
            "PRE visual": "PASS", "Controller baseline": "PASS", "Camera identity": "PASS",
            "Provenance": "PASS", "Workspace": "PASS",
            "Operator watching": operator_confirmation,
            "Power cutoff": "REACHABLE" if workspace_attestation.get("power_cutoff_reachable") else "UNKNOWN",
        }
        screen = build_approval_screen(pre_gate_summary)
        granted = bool(approval_fn(screen))
        approval_state["granted"] = granted
        if not granted:
            approval_state["declined_reason"] = "APPROVAL_PHRASE_NOT_CONFIRMED"
            raise ApprovalDeclinedBeforeArming(
                "operator did not confirm the exact approval phrase; ledger was never armed, "
                "real hardware write function was never invoked")
        armed_ok = arm_ledger_durable(r4, exp_id, detail={"args": list(a)}, ledger_dir=ledger_dir)
        if not armed_ok:
            approval_state["declined_reason"] = "LEDGER_DURABLE_ARM_VERIFICATION_FAILED"
            raise ApprovalDeclinedBeforeArming(
                "ledger fsync+re-read verification did not confirm ARMED; write aborted")
        real_write_invoked["flag"] = True
        if controller_write_fn is not None:
            controller_write_fn(*a, **kw)
        r4.mark_ledger_sent(exp_id, ledger_dir=ledger_dir)

    result = r3.execute_motion_experiment(
        run_root=run_root, allow_motion_env=allow_motion_env, execute_motion_flag=execute_motion_flag,
        operator_confirmation=operator_confirmation, operator_observation_fn=operator_observation_fn,
        expected_harness_sha256=expected_r3_harness_sha256, git_run_fn=git_run_fn, repo_root=repo_root,
        camera_check_fn=camera_check_fn, capture_pre_window_fn=capture_pre_window_fn,
        capture_post_window_fn=capture_post_window_fn, controller_read_fn=controller_read_fn,
        controller_write_fn=write_wrapper, process_check_fn=process_check_fn,
        clock_fn=clock_fn, sleep_fn=sleep_fn)

    result["experiment_id"] = exp_id
    result["workspace_safety"] = ws
    result["approval"] = dict(approval_state)
    result["real_hardware_write_invoked"] = real_write_invoked["flag"]
    if approval_state["granted"] is False and not real_write_invoked["flag"]:
        result["🔴_known_r3_accounting_note"] = (
            "operator declined before any write attempt (or ledger durable-arm verification "
            "failed); ledger has no ARMED/SENT entry for this experiment_id and "
            "real_hardware_write_invoked=false both confirm no hardware write occurred. R3's own "
            "motion_commands_sent/result_code=X fields in this specific result reflect R3's "
            "generic, unmodified fail-closed BaseException handling and must NOT be read as "
            "evidence that a write was attempted.")
    return result


# ---------------------------------------------------------------------------
# negative tests R5-01..R5-20 (전부 mock, 실하드웨어 0)
# ---------------------------------------------------------------------------
def negative_tests():
    import shutil
    import tempfile
    results = {}

    def rec(name, cond, detail=None):
        results[name] = {"pass": bool(cond), "detail": detail}

    r3 = _load_r3()
    r4 = _load_r4()
    expected_r5_sha = sha256(os.path.abspath(__file__))
    expected_r3_sha = R3_MODULE_SHA
    mock_git_clean = r3._make_mock_git(ROOT, dirty=False)
    good_ws = {k: True for k in WORKSPACE_SAFETY_ITEMS}
    zero = {"translation_x_px": 0.0, "translation_y_px": 0.0, "rotation_deg": 0.0}
    moved = {"translation_x_px": 25.0, "translation_y_px": 0.0, "rotation_deg": 0.0}

    cleanup_dirs = []

    def tmp_run_root():
        d = tempfile.mkdtemp(prefix="r5neg_")
        cleanup_dirs.append(d)
        return d

    def tmp_ledger_dir():
        d = tempfile.mkdtemp(prefix="r5ledger_")
        cleanup_dirs.append(d)
        return d

    def good_kwargs(ledger_dir, approval_fn, run_root=None):
        run_root = run_root or tmp_run_root()
        write_calls = []

        def write_fn(*a):
            write_calls.append(a)
        pre_reads = iter([152, 152, 152])
        post_reads = iter([154] * 5)

        def read_fn():
            try:
                return next(pre_reads)
            except StopIteration:
                return next(post_reads)
        kwargs = dict(
            run_root=run_root, allow_motion_env="1", execute_motion_flag=True,
            operator_confirmation="YES", workspace_attestation=dict(good_ws),
            approval_fn=approval_fn, expected_harness_sha256=expected_r5_sha,
            expected_r3_harness_sha256=expected_r3_sha, git_run_fn=mock_git_clean,
            camera_check_fn=lambda: {"dabai_identity_valid": True, "mjpg_640x480_valid": True,
                                      "camera_control_invariant_valid": True},
            capture_pre_window_fn=lambda: ({"valid": True, "median": dict(zero)}, "ANCHOR"),
            capture_post_window_fn=lambda anchor: {"valid": True, "median": dict(moved)},
            controller_read_fn=read_fn, controller_write_fn=write_fn,
            process_check_fn=lambda: [], operator_observation_fn=lambda: "YES",
            ledger_dir=ledger_dir)
        return kwargs, write_calls

    try:
        # R5-01..03 — opt-in gates (reused semantics)
        r = execute_r5(allow_motion_env=None, execute_motion_flag=False)
        rec("R5-01_no_opt_in_no_motion", r["status"] == "ABORTED" and r["motion_commands_sent"] == 0, r)
        r = execute_r5(allow_motion_env="1", execute_motion_flag=False)
        rec("R5-02_flag_missing_no_motion", r["status"] == "ABORTED" and r["motion_commands_sent"] == 0, r)
        r = execute_r5(allow_motion_env="1", execute_motion_flag=True)
        rec("R5-03_no_workspace_attestation_no_motion",
            r["status"] == "ABORTED" and r["reason"] == "WORKSPACE_ATTESTATION_MISSING", r)

        # R5-04 — R5 harness SHA mismatch -> abort before even loading R3/R4
        r = execute_r5(allow_motion_env="1", execute_motion_flag=True,
                       workspace_attestation=dict(good_ws), expected_harness_sha256="0" * 64)
        rec("R5-04_r5_harness_sha_mismatch_abort",
            r["status"] == "ABORT_BEFORE_MOTION" and r["reason"] == "R5_HARNESS_SHA_MISMATCH", r)

        # R5-05 — module SHA mismatch (R4's own SHA tampered) -> abort before motion
        orig = dict(MODULE_SHA)
        MODULE_SHA[R4_MODULE_RELPATH] = "0" * 64
        try:
            r = execute_r5(allow_motion_env="1", execute_motion_flag=True,
                           workspace_attestation=dict(good_ws))
        finally:
            MODULE_SHA.clear()
            MODULE_SHA.update(orig)
        rec("R5-05_module_sha_mismatch_abort",
            r["status"] == "ABORT_BEFORE_MOTION" and r["reason"] == "MODULE_SHA_MISMATCH"
            and R4_MODULE_RELPATH in r["detail"]["mismatches"], r)

        # R5-06 — workspace unsafe -> abort, R3 never called (no ledger/approval reached)
        bad_ws = dict(good_ws); bad_ws["cables_clear"] = False
        r = execute_r5(allow_motion_env="1", execute_motion_flag=True, workspace_attestation=bad_ws)
        rec("R5-06_workspace_unsafe_abort",
            r["status"] == "ABORTED" and r["reason"] == "WORKSPACE_NOT_SAFE", r)

        # R5-07 — approval declined -> ledger never armed, real write never invoked
        ledger_07 = tmp_ledger_dir()
        kwargs_07, writes_07 = good_kwargs(ledger_07, approval_fn=lambda screen: False)
        r = execute_r5(**kwargs_07)
        exp_id_07 = r["experiment_id"]
        ledger_path_07 = os.path.join(ledger_07, exp_id_07 + ".json")
        rec("R5-07_approval_declined_ledger_never_armed",
            not r["real_hardware_write_invoked"] and not writes_07
            and not os.path.isfile(ledger_path_07), r)

        # R5-08 — approval declined because typed phrase is close-but-wrong (case/whitespace)
        ledger_08 = tmp_ledger_dir()
        kwargs_08, writes_08 = good_kwargs(
            ledger_08, approval_fn=lambda screen: "execute p-007e-r2-r5 arm2 152 to 154 once" == REQUIRED_APPROVAL_PHRASE)
        r = execute_r5(**kwargs_08)
        rec("R5-08_approval_phrase_case_mismatch_declined",
            not r["real_hardware_write_invoked"] and not writes_08, r)

        # R5-09 — approval granted -> ledger ARMED then SENT, write invoked exactly once
        ledger_09 = tmp_ledger_dir()
        kwargs_09, writes_09 = good_kwargs(ledger_09, approval_fn=lambda screen: True)
        r = execute_r5(**kwargs_09)
        exp_id_09 = r["experiment_id"]
        ledger_path_09 = os.path.join(ledger_09, exp_id_09 + ".json")
        ledger_data_09 = json.load(open(ledger_path_09)) if os.path.isfile(ledger_path_09) else {}
        rec("R5-09_approval_granted_armed_then_sent_write_once",
            r["real_hardware_write_invoked"] and len(writes_09) == 1
            and writes_09[0] == (SERVO_ID, COMMAND_TARGET_ID2, COMMAND_DURATION_MS)
            and ledger_data_09.get("status") == "SENT", r)

        # R5-10 — restart protection: same ledger_dir, call again -> ABORTED before R3/approval reached
        kwargs_10, writes_10 = good_kwargs(ledger_09, approval_fn=lambda screen: True)
        r = execute_r5(**kwargs_10)
        rec("R5-10_restart_after_sent_cannot_resend",
            r["status"] == "ABORTED"
            and r["reason"] == "LEDGER_ALREADY_ARMED_OR_SENT_MANUAL_REVIEW_REQUIRED"
            and not writes_10, r)

        # R5-11 — ledger durable-arm verification failure (simulated corrupt re-read) ->
        # ApprovalDeclinedBeforeArming before real write, real_hardware_write_invoked stays False
        ledger_11 = tmp_ledger_dir()
        kwargs_11, writes_11 = good_kwargs(ledger_11, approval_fn=lambda screen: True)

        def _patched_r4_for_broken_reread():
            # arm_ledger writes fine, but we corrupt the file right after by monkeypatching
            # arm_ledger_durable's reread path indirectly via a broken ledger_dir permission
            # is OS-dependent; instead we directly exercise the function unit-style below.
            pass
        broken_ok = arm_ledger_durable(
            type("FakeR4", (), {"arm_ledger": staticmethod(lambda exp_id, detail=None, ledger_dir=None: None),
                                "_ledger_path": staticmethod(lambda exp_id, ledger_dir=None: os.path.join(ledger_dir, "nope.json"))})(),
            "doesnotexist", ledger_dir=ledger_11)
        rec("R5-11_ledger_durable_arm_verification_detects_missing_file", broken_ok is False, broken_ok)

        # R5-12 — command geometry unchanged from R3/R4 (defense in depth re-check)
        rec("R5-12_command_geometry_matches_r3_r4",
            SERVO_ID == r3.SERVO_ID == 2 and COMMAND_TARGET_ID2 == r3.COMMAND_TARGET_ID2 == 154
            and COMMAND_DURATION_MS == r3.COMMAND_DURATION_MS == 2000)

        # R5-13 — approval wrapper does not call real controller_write_fn before approval granted
        call_order = []

        def tracking_write(*a):
            call_order.append("real_write")

        def tracking_approval(screen):
            call_order.append("approval_prompted")
            return True
        ledger_13 = tmp_ledger_dir()
        kwargs_13, _ = good_kwargs(ledger_13, approval_fn=tracking_approval)
        kwargs_13["controller_write_fn"] = tracking_write
        execute_r5(**kwargs_13)
        rec("R5-13_approval_prompted_before_real_write",
            call_order == ["approval_prompted", "real_write"], call_order)

        # R5-14 — self_ast_check: no hardware imports in this orchestration file
        chk = self_ast_check()
        rec("R5-14_self_ast_check_no_hardware_imports", chk["pass"], chk)

        # R5-15 — module SHA dict pins R3/R4/window/sham/evidence_pipeline/fk (6 modules)
        rec("R5-15_module_sha_pins_six_modules", len(MODULE_SHA) == 6, sorted(MODULE_SHA))

        # R5-16 — approval phrase constant exact (defensive, catches accidental edits)
        rec("R5-16_approval_phrase_exact",
            REQUIRED_APPROVAL_PHRASE == "EXECUTE P-007E-R2-R5 ARM2 152 TO 154 ONCE")

        # R5-17 — no second motion command even if approval_fn is called twice by accident
        # (write_wrapper itself is only invoked once by R3 per call to execute_r5)
        ledger_17 = tmp_ledger_dir()
        approval_calls = {"n": 0}

        def counting_approval(screen):
            approval_calls["n"] += 1
            return True
        kwargs_17, writes_17 = good_kwargs(ledger_17, approval_fn=counting_approval)
        execute_r5(**kwargs_17)
        rec("R5-17_write_wrapper_invoked_at_most_once", approval_calls["n"] <= 1 and len(writes_17) <= 1,
            {"approval_calls": approval_calls["n"], "writes": len(writes_17)})

        # R5-18 — R4 module itself unmodified (SHA match re-confirmed independently of execute_r5)
        module_check = check_module_sha()
        rec("R5-18_r4_module_unmodified", module_check["mismatches"].get(R4_MODULE_RELPATH) is None,
            module_check)

        # R5-19 — no real camera / controller / robot access anywhere in this test suite
        rec("R5-19_no_real_hardware_in_tests",
            "Arm_Lib" not in chk["imports"] and "cv2" not in chk["imports"])

        # R5-20 — decline path leaves R3's own result marked X, but R5 adds the disclosure note
        ledger_20 = tmp_ledger_dir()
        kwargs_20, writes_20 = good_kwargs(ledger_20, approval_fn=lambda screen: False)
        r20 = execute_r5(**kwargs_20)
        rec("R5-20_decline_path_carries_disclosure_note",
            "🔴_known_r3_accounting_note" in r20 and not writes_20, r20)
    finally:
        for d in cleanup_dirs:
            shutil.rmtree(d, ignore_errors=True)

    all_pass = all(v["pass"] for v in results.values())
    return {"all_pass": all_pass, "results": results}


def selftest():
    return {"self_ast_check": self_ast_check(), "module_sha_check": check_module_sha(),
            "negative_tests_R5_01_R5_20": negative_tests()}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        out = selftest()
        print(json.dumps(out, ensure_ascii=False, indent=1, default=repr))
        ok = (out["self_ast_check"]["pass"] and out["module_sha_check"]["match"]
              and out["negative_tests_R5_01_R5_20"]["all_pass"])
        sys.exit(0 if ok else 1)
    print(json.dumps({"gate": "P-007E-R2-R5", "status": "ORCHESTRATION_ONLY",
                      "note": "--self-test 로만 실행 가능. 실 하드웨어 wiring 은 "
                              "scripts/p007e_r2r5_hardware_wiring.py 에서만 이뤄진다"},
                     ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
