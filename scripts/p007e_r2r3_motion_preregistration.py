#!/usr/bin/env python3
"""P-007E-R2-R3 — Controlled Physical-Motion Observation Preregistration harness.

이 gate 는 PREREGISTRATION ONLY 다. 이 파일은 **직접** Arm_Lib / smbus / smbus2 /
rclpy / rospy / cv2 / Adafruit_GPIO 를 import 하지 않고, **직접** subprocess 도
호출하지 않는다(self_ast_check() 가 이 파일의 AST 만으로 보장). 카메라 프레임
캡처·추정기(estimator) 구성·실제 서보 I/O는 전부 **호출자 주입형**(camera_check_fn /
capture_pre_window_fn / capture_post_window_fn / controller_read_fn /
controller_write_fn / process_check_fn)이며 기본값 None 이면 그 항목은 실패로
취급되어 ABORT_BEFORE_MOTION 으로 끝난다 — 즉 opt-in 환경변수·플래그·운영자 확인을
전부 참으로 둬도, 이 파일 자체는 실제 로봇/카메라에 닿을 방법이 없다.

🔴 run_trial()(scripts/p007e_r2r2_c1r1_sham.py)을 그대로 재사용하지 않는 이유 —
   §20 은 "PRE window 가 불량하면 motion 을 보내기 전에 abort" 를 요구하는데, run_trial()
   은 reference→(quiet+command+settle)→comparison 을 한 함수 안에서 끝까지 진행해 그
   사이에 결정점을 둘 수 없다(수정하면 이미 CLOSED 된 C1-R1/C1-R1-R1 의 harness SHA가
   바뀐다 — 불변 evidence 를 건드리지 않기 위해 sham.py 는 1바이트도 고치지 않았다).
   그래서 이 파일은 PRE window 캡처와 POST window 캡처를 **분리된 주입형 콜백**으로
   요구한다 — 그 콜백의 실제 구현(측정 루프 자체)은 다음 EXECUTION gate 가 추가한다.
   이 파일이 직접 재사용하는 것은 §28/§29 의 순수 함수(결과행렬/claim mapping)와
   §25/§26 의 evidence_pipeline(git provenance/SHA/전송무결성) 뿐이다.

🔴 advisor 2차 재검토(BLOCKING 4건) 반영:
   1) anchor 동결 — run_trial() 은 reference window 의 첫 프레임을 anchor 로 고정하고
      comparison window 도 **같은 anchor** 로 추정한다(anchor_gray 가 윈도우 루프 바깥에서
      한 번만 초기화됨). 이 harness 도 동일 계약을 쓴다: capture_pre_window_fn() 이
      (window_dict, anchor) 를 반환하고, capture_post_window_fn(anchor) 가 그 anchor 를
      받는다. anchor 를 넘기지 않으면 PRE/POST 가 각자 자기 첫 프레임을 기준으로 추정해
      Δ≈0 으로 수렴하는 치명적 결함이 생긴다(advisor 가 직접 지적).
   2) write 이후 crash-safety + 증분 저장 — write 호출 자체와 그 이후 구간을 각각
      try/except BaseException 으로 감싸고, write 직후 즉시 1회·최종 1회 저장한다.
      "명령을 보냈다는 사실"이 디스크에 남지 않는 경로를 없앤다.
   3) quiet interval 측정 순서 — controller pre-read 를 **motion 직전으로 재배치**해
      quiet_interval_ms 가 실제 "마지막 read 종료 -> write 시작" 간격을 측정하게 한다
      (이전 버전은 PRE window/process check/writable check 가 다 끝난 뒤의 오래된
      read 를 기준으로 삼아 수십 초 묵은 값을 60ms 라고 거짓 기록할 위험이 있었다).
   4) pre-read None 규칙을 코드(엄격: 라운드 전부 non-None)에 맞춰 preregistration
      문구를 통일했다.
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
PREREG = os.path.join(ROOT, "evidence/P-007E-R2-R3/stage_r3_preregistration.json")

# §6 — 이 preregistration 문서 자신의 sha256 (evidence/P-007E-R2-R3/stage_r3_preregistration.json)
PREREG_SHA = "291dc16acf0cf53db521fae722a8553f321aa99b39f4aa82c98162705599c868"

# §26 — future execution 이 preregistered SHA 와 일치해야 하는 모듈들
MODULE_SHA = {
    "scripts/p007e_r2r2_acquisition.py": "dc4cf383d61ddfd96fb322a5bd8522ecb9cb97b9e3c636592f9d2b4f0e7400e1",
    "scripts/p007e_r2r2_measurement.py": "024b7b9d3d6a37aca79afe90b2fa65ced6f5885c89fdbd82b85b1c776b7d526c",
    "scripts/p007e_r2r2_analysis.py": "fd3910ff9d4759244cb3ce1837c2dab099059af006276bee56d5dc8951da4f4f",
    "scripts/p007e_r2r2_evidence_pipeline.py": "298e0656027d5bef4eca42101054658775e0b6d7c24500194297a636f7635133",
}

FORBIDDEN_HARDWARE_IMPORTS = {"Arm_Lib", "smbus", "smbus2", "rclpy", "rospy", "cv2", "Adafruit_GPIO"}

ALLOW_ENV = "P007E_R2_R3_ALLOW_MOTION"
OPERATOR_CONFIRM_VALUE = "YES"

# §7 — command geometry (frozen)
SERVO_ID = 2
EXPECTED_BASELINE_ID2 = 152
COMMAND_DELTA = 2
COMMAND_TARGET_ID2 = 154
COMMAND_DURATION_MS = 2000
MAX_MOTION_COMMANDS = 1
ALLOWED_API = "Arm_serial_servo_write"

# §9 — forbidden motion APIs (documentation/negative-test data only — this file
# never calls any servo API itself, so this list is reference data, not a self-scan target)
FORBIDDEN_MOTION_APIS = ("write6", "write6_array", "write_any", "PWM",
                          "action group", "MoveIt", "trajectory command")

# §10 — host-control quiet interval (이 host 프로세스의 제어호출 간격. "I2C 버스 전체가
# 조용하다"는 주장이 아니다 — yahboom_oled 가 같은 버스를 쓸 수 있기 때문)
HOST_CONTROL_QUIET_INTERVAL_MIN_MS = 60.0
HOST_CONTROL_QUIET_SLEEP_S = 0.060   # P-007E-R1 phase_b_acceptance 실측 60.099ms 채택

# 🔴 controller read 프로토콜 — P-007E-R1 선례 그대로 전재(phase_b_decision_rules.json,
# phase_b_preread.json/phase_b_postread.json 실측): 라운드 간 20ms 간격(<10ms 는 ced8f52
# 실측상 ~50% None), pre 3라운드/post 5라운드, 반복 read 는 100% 안정(0 분산) 관측됨.
# 이 spacing 은 HOST-CONTROL QUIET INTERVAL(§10, read 종료→write 시작 간격)과는 다른
# 개념이다 — 혼동하지 않는다.
READ_SPACING_S = 0.020
PRE_READ_ROUNDS = 3
POST_READ_ROUNDS = 5

# §19
COMMAND_DURATION_S = 2.0
POST_COMMAND_SETTLE_S = 3.0

# §20
PRE_WINDOW_M = 60

# §15/§17 — frozen publication thresholds (floor-dominated, unchanged since C1-R1)
PUBLICATION_THRESHOLD_TRANSLATION_PX = 4.950
PUBLICATION_THRESHOLD_ROTATION_DEG = 0.300

# §16
PREDICTION_ONLY_PX = 21

# §21 — pre-motion hard gate: every key must be True. write 직전 단일 결정점에서만 평가한다
PRE_MOTION_HARD_GATE_KEYS = (
    "git_provenance_valid", "preregistration_sha_valid", "harness_sha_valid",
    "module_sha_valid", "dabai_identity_valid", "mjpg_640x480_valid",
    "camera_control_invariant_valid", "pre_visual_window_valid",
    "roi_feature_guards_pass", "controller_baseline_valid",
    "no_unexpected_motion_capable_process", "host_control_quiet_interval_satisfied",
    "evidence_package_writable", "artifact_persistence_path_valid",
)

# §23 — not machine-verifiable; operator confirms manually, recorded in the report
WORKSPACE_SAFETY_CHECKLIST = (
    "workspace clear", "no human body in robot workspace",
    "no loose object interfering with Arm2", "cables clear",
    "camera mount secure", "robot base stable",
    "emergency physical power cutoff reachable",
)

# §4x — post-motion EXPERIMENT_INVALID(X) trigger list (사전 고정, 데이터를 보고 추가하지 않음)
# 🔴 advisor 2차 재검토로 WRITE_CALL_EXCEPTION/POST_WRITE_EXCEPTION 추가 — 명령을
# 보낸 뒤 어떤 예외가 나도 "기록이 아예 안 남는" 경로를 없앤다
X_TRIGGER_REASONS = (
    "POST_WINDOW_QUALITY_GUARD_FAILED",
    "CONTROLLER_POST_READ_UNKNOWN",
    "UNEXPECTED_MOTION_CAPABLE_PROCESS_AFTER_WRITE",
    "EVIDENCE_PERSISTENCE_FAILED",
    "WRITE_CALL_EXCEPTION",
    "POST_WRITE_EXCEPTION",
)

R3_RESULT_REQUIRED_KEYS = (
    "pre_visual", "post_visual", "controller_pre_read_rounds", "controller_post_read_rounds",
    "result_code", "physical_motion_claim", "operator_observed_physical_motion",
    "quiet_interval_ms", "invalid_reasons", "motion_commands_sent",
)
STATUS_COMMAND_SENT_IN_PROGRESS = "COMMAND_SENT_IN_PROGRESS"
STATUS_EXPERIMENT_COMPLETE = "EXPERIMENT_COMPLETE"


def sha256(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


# ---------------------------------------------------------------------------
# self-safety check — structural guarantee that this file touches no hardware
# directly. 🔴 evidence_pipeline(별도 pinned 모듈)은 git_run_fn=None 실경로에서
# 자체 subprocess 를 쓴다 — 이 파일 자신이 쓰는 것은 아니다. "subprocess 0" 이라는
# 주장은 **이 파일 한정**이다(decision.json 에도 이 경계를 명시한다).
# ---------------------------------------------------------------------------
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
    direct_subprocess_import = "subprocess" in imported
    return {
        "imports": sorted(imported),
        "forbidden_hardware_imports_found": bad_imports,
        "direct_subprocess_import_in_this_file": direct_subprocess_import,
        "pass": not bad_imports and not direct_subprocess_import,
    }


# ---------------------------------------------------------------------------
# §28 — motion result matrix (pure function)
# ---------------------------------------------------------------------------
def motion_result_matrix(controller_changed, visual_changed, experiment_valid):
    if not experiment_valid:
        return "X"
    if controller_changed and visual_changed:
        return "A"
    if controller_changed and not visual_changed:
        return "B"
    if not controller_changed and visual_changed:
        return "C"
    return "D"


# ---------------------------------------------------------------------------
# §29 — physical-motion claim mapping (pure function)
# ---------------------------------------------------------------------------
_CLAIM_MAP = {
    "A": "PHYSICAL_MOTION_EVIDENCE_SUPPORTED",
    "B": "PHYSICAL_MOTION_NOT_ESTABLISHED",
    "C": "UNEXPECTED_VISUAL_MOTION",
    "D": "NO_OBSERVABLE_RESPONSE",
    "X": "EXPERIMENT_INVALID",
}


def physical_motion_claim(result):
    if result not in _CLAIM_MAP:
        raise ValueError("unknown result code: %r" % result)
    return _CLAIM_MAP[result]


# ---------------------------------------------------------------------------
# command / safety validators (pure functions)
# ---------------------------------------------------------------------------
def validate_command_request(servo_id, target_degrees, duration_ms, command_count, api=ALLOWED_API):
    violations = []
    if api != ALLOWED_API:
        violations.append("FORBIDDEN_API:%r" % api)
    if servo_id != SERVO_ID:
        violations.append("WRONG_SERVO_ID:%r" % servo_id)
    if target_degrees != COMMAND_TARGET_ID2:
        violations.append("WRONG_TARGET:%r" % target_degrees)
    if duration_ms != COMMAND_DURATION_MS:
        violations.append("WRONG_DURATION:%r" % duration_ms)
    if command_count > MAX_MOTION_COMMANDS:
        violations.append("COMMAND_BUDGET_EXCEEDED:%r" % command_count)
    return {"accepted": not violations, "violations": violations}


def reject_automatic_return(command_sequence):
    """command_sequence: 이번 실행에서 전송(예정)된 target 각도 리스트.
    길이 1 초과거나, 알려진 복귀 패턴(154->152, 152->150)을 포함하면 거부한다."""
    if len(command_sequence) > MAX_MOTION_COMMANDS:
        return {"rejected": True, "reason": "SEQUENCE_LENGTH_EXCEEDS_BUDGET"}
    pairs = list(zip(command_sequence, command_sequence[1:]))
    known_return_pairs = {(154, 152), (152, 150)}
    if any(p in known_return_pairs for p in pairs):
        return {"rejected": True, "reason": "AUTOMATIC_RETURN_PATTERN_DETECTED"}
    return {"rejected": False, "reason": None}


def quiet_interval_ms(t_last_read_end_ns, t_write_start_ns):
    """🔴 순수 함수 — p007e_r1_timing_controlled.py 의 동일 정의를 전재한다."""
    return (t_write_start_ns - t_last_read_end_ns) / 1_000_000.0


def host_control_quiet_interval_ok(quiet_ms, minimum_ms=HOST_CONTROL_QUIET_INTERVAL_MIN_MS):
    return quiet_ms >= minimum_ms


def operator_presence_gate(confirmation_value):
    return confirmation_value == OPERATOR_CONFIRM_VALUE


def visual_changed(delta_norm_px, delta_rotation_deg,
                    thr_norm=PUBLICATION_THRESHOLD_TRANSLATION_PX,
                    thr_rot=PUBLICATION_THRESHOLD_ROTATION_DEG):
    return (delta_norm_px > thr_norm) or (abs(delta_rotation_deg) > thr_rot)


def compute_visual_delta(pre_median, post_median):
    """§17 — run_trial() 의 trial['delta'] 구성과 동일한 공식(elementwise 차 + norm).
    🔴 pre_median/post_median 은 **같은 anchor** 로 추정된 median 이어야 의미가 있다
    (advisor 2차 지적 — anchor 계약은 capture_pre_window_fn/capture_post_window_fn 에 있다)."""
    delta = {k: post_median[k] - pre_median[k] for k in pre_median}
    delta["norm"] = (delta["translation_x_px"] ** 2 + delta["translation_y_px"] ** 2) ** 0.5
    return delta


# ---------------------------------------------------------------------------
# controller read-round protocol (P-007E-R1 선례 그대로 전재, 베이스라인만 150->152)
# ---------------------------------------------------------------------------
def controller_read_rounds(controller_read_fn, n_rounds, sleep_fn=None):
    sleep_fn = sleep_fn or time.sleep
    values = []
    for i in range(n_rounds):
        values.append(controller_read_fn())
        if i < n_rounds - 1:
            sleep_fn(READ_SPACING_S)
    return values


def pre_read_baseline_ok(round_values, expected=EXPECTED_BASELINE_ID2):
    """🔴 엄격 규칙(코드-prereg 불일치를 이 쪽으로 통일, advisor 2차 지적):
    라운드 전부가 non-None 이고 전부 expected 와 같아야 PASS. 하나라도 None 이거나
    하나라도 expected 가 아니면 FAIL."""
    if not round_values:
        return False
    non_none = [v for v in round_values if v is not None]
    if len(non_none) != len(round_values):
        return False
    return all(v == expected for v in non_none)


def controller_changed_from_rounds(round_values, baseline=EXPECTED_BASELINE_ID2):
    """🔴 P-007E-R1 phase_b_decision_rules.json 의 readback_change_criterion 을 그대로
    전재한다: "None 이 아닌 값 중 하나라도 baseline 이 아니면 CHANGE". 전부 None 이면
    None(UNKNOWN) 을 반환한다 — 호출자가 experiment_valid=False(X) 로 처리한다."""
    non_none = [v for v in round_values if v is not None]
    if not non_none:
        return None
    return any(v != baseline for v in non_none)


# ---------------------------------------------------------------------------
# §21 — pre-motion hard gate aggregate
# ---------------------------------------------------------------------------
def pre_motion_hard_gate(checks):
    missing_keys = [k for k in PRE_MOTION_HARD_GATE_KEYS if k not in checks]
    if missing_keys:
        return {"pass": False, "status": "ABORT_BEFORE_MOTION",
                "reason": "MISSING_GATE_KEYS", "missing_keys": missing_keys, "failed_checks": missing_keys}
    failed = [k for k in PRE_MOTION_HARD_GATE_KEYS if not checks[k]]
    if failed:
        return {"pass": False, "status": "ABORT_BEFORE_MOTION",
                "reason": "GATE_CHECKS_FAILED", "failed_checks": failed}
    return {"pass": True, "status": "PASS", "reason": None, "failed_checks": []}


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


def check_preregistration_sha(prereg_path=PREREG, expected=None):
    # 🔴 expected=PREREG_SHA 를 기본 파라미터로 두면 모듈 로드 시점에 바인딩되어
    # negative test 가 globals()["PREREG_SHA"] 를 patch 해도 반영되지 않는다
    # (self-discovered, Python late-binding 함정) — 호출 시점에 전역을 다시 읽는다
    if expected is None:
        expected = PREREG_SHA
    try:
        actual = sha256(prereg_path)
    except OSError as e:
        return {"match": False, "expected": expected, "actual": "READ_ERROR:%r" % e}
    return {"match": actual == expected, "expected": expected, "actual": actual}


def check_harness_sha(expected_harness_sha256):
    actual = sha256(os.path.abspath(__file__))
    if expected_harness_sha256 is None:
        return {"match": False, "status": "NOT_PINNED", "actual": actual, "expected": None}
    return {"match": actual == expected_harness_sha256,
            "status": "OK" if actual == expected_harness_sha256 else "MISMATCH",
            "actual": actual, "expected": expected_harness_sha256}


# ---------------------------------------------------------------------------
# git-tracked execution requirement (§25/§26) — reuses evidence_pipeline, no duplication
# ---------------------------------------------------------------------------
def _load_evidence_pipeline():
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    return importlib.import_module("p007e_r2r2_evidence_pipeline")


def verify_git_tracked_execution(repo_root=ROOT, run_fn=None, require_clean=True):
    evp = _load_evidence_pipeline()
    try:
        dirty = evp.check_dirty_tree(repo_root, require_clean=require_clean, run_fn=run_fn)
    except evp.GitProvenanceUnavailable as e:
        return {"ok": False, "status": "ABORTED", "reason": "PROVENANCE_UNAVAILABLE:%r" % e}
    if require_clean and dirty.get("dirty"):
        return {"ok": False, "status": "ABORTED", "reason": "DIRTY_TREE_GUARD", "detail": dirty}
    return {"ok": True, "status": "PASS", "detail": dirty}


def _make_mock_git(expected_root, head=None, dirty=False):
    head = head or ("mock" + "head" + "0" * 36)

    def run_fn(argv):
        if argv[:3] == ["git", "-C", expected_root] or (len(argv) > 2 and argv[1] == "-C"):
            if "--show-toplevel" in argv:
                return expected_root + "\n"
            if "rev-parse" in argv and "HEAD" in argv:
                return head + "\n"
            if "status" in argv and "--porcelain" in argv:
                return (" M fake_dirty_file.py\n" if dirty else "")
            if "log" in argv:
                return head + "\n"
        raise RuntimeError("unexpected git invocation in mock: %r" % argv)
    return run_fn


def _check_writable(run_root):
    if not run_root:
        return False
    try:
        os.makedirs(run_root, exist_ok=True)
        probe = os.path.join(run_root, ".write_probe")
        with open(probe, "w") as f:
            f.write("ok")
        os.remove(probe)
        return True
    except OSError:
        return False


def _atomic_write_json(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=1, default=repr)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def _atomic_write_json_best_effort(path, obj):
    """§21 pre-motion record 등 — 저장 실패가 abort 사유가 되지 않는 보조 기록.
    🔴 OSError 뿐 아니라 ValueError(예: 경로에 embedded null byte)도 잡는다 —
    best-effort 기록이 비정상 경로 입력으로 harness 자체를 죽여선 안 된다."""
    try:
        _atomic_write_json(path, obj)
        return True
    except (OSError, ValueError) as e:
        return repr(e)


def _persist_r3_result(run_root, out):
    """🔴 advisor 2차 BLOCKING 2 — write 직후부터, 그리고 최종적으로 다시, 디스크에
    남긴다. 저장 자체가 실패하면 예외를 던지지 않고 실패 사유 문자열을 반환한다
    (호출자가 EVIDENCE_PERSISTENCE_FAILED 로 처리)."""
    if not run_root:
        return None
    try:
        result_path = os.path.join(run_root, "r3_result.json")
        _atomic_write_json(result_path, out)
        evp = _load_evidence_pipeline()
        manifest = evp.build_artifact_sha_manifest(run_root, relative_paths=["r3_result.json"])
        _atomic_write_json(os.path.join(run_root, "artifact_sha256.json"), manifest)
        return True
    except Exception as e:  # noqa: BLE001 — 저장 실패 사유를 전부 포착해 X 로 되돌린다
        return repr(e)


def check_r3_artifact_completeness(root_dir):
    """R3 실험 고유 artifact contract — sham/trial 포맷을 재사용하지 않는다(§35 와
    무관한 다른 모양이라 evidence_pipeline.check_artifact_completeness 를 그대로
    쓰면 R3 shape 을 검증하지 못한다 — advisor 지적으로 전용 함수를 둔다)."""
    expected_files = ("r3_result.json", "artifact_sha256.json")
    missing_files = [f for f in expected_files if not os.path.isfile(os.path.join(root_dir, f))]
    if missing_files:
        return {"status": "EVIDENCE_PACKAGE_INCOMPLETE", "missing": missing_files}
    try:
        result = json.load(open(os.path.join(root_dir, "r3_result.json")))
    except (OSError, json.JSONDecodeError) as e:
        return {"status": "EVIDENCE_PACKAGE_INCOMPLETE", "missing": [], "parse_error": repr(e)}
    missing_keys = [k for k in R3_RESULT_REQUIRED_KEYS if k not in result]
    if missing_keys:
        return {"status": "EVIDENCE_PACKAGE_INCOMPLETE", "missing": [], "missing_keys": missing_keys}
    # 🔴 advisor 3차 BLOCKING 1 — 필수 키가 다 있어도 result["status"] 가
    # STATUS_EXPERIMENT_COMPLETE 가 아니면(= 중간저장 상태에서 프로세스가 죽은 경우) 완전하지 않다
    if result.get("status") != STATUS_EXPERIMENT_COMPLETE:
        return {"status": "EVIDENCE_PACKAGE_INCOMPLETE", "missing": [],
                "missing_keys": [], "reason": "RESULT_STATUS_NOT_COMPLETE",
                "actual_status": result.get("status")}
    return {"status": "COMPLETE", "missing": [], "missing_keys": []}


# ---------------------------------------------------------------------------
# §31 — future execution orchestration (NO hardware wiring in this file)
# ---------------------------------------------------------------------------
def execute_motion_experiment(
        run_root=None,
        allow_motion_env=None, execute_motion_flag=False,
        operator_confirmation=None, operator_observation_fn=None,
        expected_harness_sha256=None,
        git_run_fn=None, repo_root=ROOT,
        camera_check_fn=None, capture_pre_window_fn=None, capture_post_window_fn=None,
        controller_read_fn=None, controller_write_fn=None, process_check_fn=None,
        clock_fn=None, sleep_fn=None):
    """기본값은 전부 NO CAPTURE / NO CONTROLLER ACCESS / NO MOTION.

    🔴 advisor 2차 재검토 순서 고정(BLOCKING 3) — camera check -> PRE window(+anchor)
    -> process check -> writable check -> controller pre-read(가장 신선한 값) ->
    quiet sleep -> hard gate 평가 -> write. 이렇게 해야 host_control_quiet_interval_ms
    가 실제로 "마지막 controller read 종료 -> write 시작" 간격을 측정한다.

    🔴 capture_pre_window_fn() 은 (window_dict, anchor) 를 반환하고,
    capture_post_window_fn(anchor) 는 그 anchor 를 받는다 — PRE/POST 가 같은 anchor
    로 추정되어야 Δ 가 의미를 가진다(advisor 2차 BLOCKING 1).

    write 호출 자체와 그 이후 구간은 각각 try/except BaseException 으로 감싼다
    (advisor 2차 BLOCKING 2) — 명령을 보낸 사실은 write 직후 즉시 저장한다."""
    sleep_fn = sleep_fn or time.sleep
    clock_fn = clock_fn or time.monotonic_ns

    env_set = (allow_motion_env == "1")
    if not (env_set and execute_motion_flag):
        return {"status": "ABORTED", "reason": "NO_OPT_IN",
                "env_set": env_set, "flag_set": execute_motion_flag, "motion_commands_sent": 0}

    if not operator_presence_gate(operator_confirmation):
        return {"status": "ABORTED", "reason": "OPERATOR_CONFIRMATION_MISSING", "motion_commands_sent": 0}

    harness_sha_check = check_harness_sha(expected_harness_sha256)
    git_check = verify_git_tracked_execution(repo_root=repo_root, run_fn=git_run_fn)
    prereg_check = check_preregistration_sha()
    module_check = check_module_sha(repo_root=repo_root)

    camera_check = camera_check_fn() if camera_check_fn else {}

    if capture_pre_window_fn:
        pre_visual, anchor = capture_pre_window_fn()
    else:
        pre_visual, anchor = {"valid": False}, None

    process_list_before = process_check_fn() if process_check_fn else None
    no_unexpected_process_before = (process_list_before == [])

    writable = _check_writable(run_root)

    # 🔴 controller pre-read 는 quiet interval 측정의 기준점이므로 여기(motion 직전)
    # 에서 수행한다 — PRE window/process check/writable check 보다 먼저 하면 그만큼
    # 묵은 값이 되어 quiet_interval_ms 가 거짓말을 한다
    pre_read_values = (controller_read_rounds(controller_read_fn, PRE_READ_ROUNDS, sleep_fn)
                       if controller_read_fn else [])
    controller_baseline_valid = pre_read_baseline_ok(pre_read_values) if pre_read_values else False
    t_last_read_end = clock_fn()

    checks = {
        "git_provenance_valid": git_check["ok"],
        "preregistration_sha_valid": prereg_check["match"],
        "harness_sha_valid": harness_sha_check["match"],
        "module_sha_valid": module_check["match"],
        "dabai_identity_valid": bool(camera_check.get("dabai_identity_valid")),
        "mjpg_640x480_valid": bool(camera_check.get("mjpg_640x480_valid")),
        "camera_control_invariant_valid": bool(camera_check.get("camera_control_invariant_valid")),
        "pre_visual_window_valid": bool(pre_visual.get("valid")),
        # 🔴 ROI feature guard 통과율은 window capture 자체의 >=80% valid-frame 요건과
        # 같은 신호다(estimator 가 프레임별로 그 guard 를 이미 평가한다) — 별도 센서가
        # 없으므로 두 키를 같은 값으로 묻는다. 다음 gate 가 별도 세부 breakdown 을 주입하면
        # 그때 분리한다
        "roi_feature_guards_pass": bool(pre_visual.get("valid")),
        "controller_baseline_valid": controller_baseline_valid,
        "no_unexpected_motion_capable_process": no_unexpected_process_before,
        "host_control_quiet_interval_satisfied": None,   # 아래서 측정 후 채운다
        "evidence_package_writable": writable,
        "artifact_persistence_path_valid": writable,
    }

    sleep_fn(HOST_CONTROL_QUIET_SLEEP_S)
    t_write_start = clock_fn()
    quiet_ms = quiet_interval_ms(t_last_read_end, t_write_start)
    checks["host_control_quiet_interval_satisfied"] = host_control_quiet_interval_ok(quiet_ms)

    gate = pre_motion_hard_gate(checks)

    if run_root:
        _atomic_write_json_best_effort(os.path.join(run_root, "r3_pre_motion_record.json"), {
            "hard_gate_checks": checks, "quiet_interval_ms": quiet_ms,
            "controller_pre_read_rounds": pre_read_values, "pre_visual": pre_visual,
            "gate_pass": gate["pass"],
        })

    if not gate["pass"]:
        return {"status": "ABORT_BEFORE_MOTION", "reason": gate["reason"],
                "failed_checks": gate["failed_checks"], "hard_gate_checks": checks,
                "quiet_interval_ms": quiet_ms, "motion_commands_sent": 0}

    cmd = validate_command_request(SERVO_ID, COMMAND_TARGET_ID2, COMMAND_DURATION_MS, 1)
    if not cmd["accepted"]:
        return {"status": "ABORTED", "reason": "COMMAND_VALIDATION_FAILED", "detail": cmd,
                "hard_gate_checks": checks, "motion_commands_sent": 0}

    if controller_write_fn is None:
        return {"status": "ABORTED", "reason": "HARDWARE_WIRING_NOT_IMPLEMENTED_IN_THIS_STAGE",
                "hard_gate_checks": checks, "motion_commands_sent": 0}

    # 🔴 advisor 3차 BLOCKING 1 — 결과 키를 전부 None 으로 미리 채워두고, 중간 저장은
    # status=STATUS_COMMAND_SENT_IN_PROGRESS 로만 남긴다. "다 끝났다"는 라벨
    # (STATUS_EXPERIMENT_COMPLETE)은 더 진행할 단계가 없는 **최종 저장 시점에만** 붙인다
    # — write 직후 프로세스가 죽어도(SIGKILL/전원차단/SSH 끊김) 디스크에는 끝나지 않은
    # 실험이 정직하게 IN_PROGRESS 로 남아야 한다.
    out = {"status": STATUS_COMMAND_SENT_IN_PROGRESS, "motion_commands_sent": 0,
           "hard_gate_checks": checks, "quiet_interval_ms": quiet_ms,
           "controller_pre_read_rounds": pre_read_values, "pre_visual": pre_visual,
           "controller_post_read_rounds": None, "post_visual": None, "visual_delta": None,
           "controller_changed": None, "visual_changed": None, "experiment_valid": None,
           "invalid_reasons": [], "result_code": None, "physical_motion_claim": None,
           "operator_observed_physical_motion": None}

    # ---- 이 지점부터 물리 명령 전송을 시도한다 — 이후 어떤 실패도 "명령을 보냈다는
    # 사실"을 디스크에서 지우지 않는다(advisor 2차 BLOCKING 2) ----
    try:
        controller_write_fn(SERVO_ID, COMMAND_TARGET_ID2, COMMAND_DURATION_MS)
    except BaseException as e:  # noqa: BLE001 — Arm_Lib 는 I2C 예외를 bare except 로
        # 삼킨다는 선례가 있다(p007e_phase1_command.py) — 보드에 도달했는지 모른다고
        # 해서 0 으로 적지 않는다. fail-closed 로 "보냈다"고 기록한다. 이 분기는 더
        # 이어질 단계가 없으므로 바로 최종 상태(EXPERIMENT_COMPLETE)로 저장한다
        out["status"] = STATUS_EXPERIMENT_COMPLETE
        out["motion_commands_sent"] = 1
        out["invalid_reasons"] = ["WRITE_CALL_EXCEPTION"]
        out["experiment_valid"] = False
        out["result_code"] = "X"
        out["physical_motion_claim"] = physical_motion_claim("X")
        out["write_call_exception"] = repr(e)
        _persist_r3_result(run_root, out)
        return out

    out["motion_commands_sent"] = 1
    _persist_r3_result(run_root, out)   # 🔴 write 직후 즉시 저장(status 는 여전히
                                        # IN_PROGRESS) — 이 시점부터는 디스크에 "명령
                                        # 1회 전송됨"이 남는다

    try:
        sleep_fn(COMMAND_DURATION_S + POST_COMMAND_SETTLE_S)
        operator_observed = operator_observation_fn() if operator_observation_fn else "UNCERTAIN"
        post_read_values = (controller_read_rounds(controller_read_fn, POST_READ_ROUNDS, sleep_fn)
                           if controller_read_fn else [])
        post_visual = capture_post_window_fn(anchor) if capture_post_window_fn else {"valid": False}
        process_list_after = process_check_fn() if process_check_fn else None

        invalid_reasons = []
        if not post_visual.get("valid"):
            invalid_reasons.append("POST_WINDOW_QUALITY_GUARD_FAILED")
        non_none_post = [v for v in post_read_values if v is not None]
        if not non_none_post:
            invalid_reasons.append("CONTROLLER_POST_READ_UNKNOWN")
        if process_list_after:
            invalid_reasons.append("UNEXPECTED_MOTION_CAPABLE_PROCESS_AFTER_WRITE")

        experiment_valid = not invalid_reasons
        controller_changed_val = None
        visual_delta = None
        vis_changed_val = None
        if experiment_valid:
            controller_changed_val = controller_changed_from_rounds(post_read_values)
            visual_delta = compute_visual_delta(pre_visual["median"], post_visual["median"])
            vis_changed_val = visual_changed(visual_delta["norm"], visual_delta["rotation_deg"])

        result_code = motion_result_matrix(bool(controller_changed_val), bool(vis_changed_val), experiment_valid)
        claim = physical_motion_claim(result_code)

        out.update({
            "status": STATUS_EXPERIMENT_COMPLETE,
            "controller_post_read_rounds": post_read_values,
            "post_visual": post_visual,
            "visual_delta": visual_delta,
            "controller_changed": controller_changed_val,
            "visual_changed": vis_changed_val,
            "experiment_valid": experiment_valid,
            "invalid_reasons": invalid_reasons,
            "result_code": result_code,
            "physical_motion_claim": claim,
            "operator_observed_physical_motion": operator_observed,
        })
    except BaseException as e:  # noqa: BLE001 — write 이후 구간 전체를 감싼다. 더 이어질
        # 단계가 없으므로(이 분기에서 함수가 끝난다) 바로 최종 상태로 저장한다
        out["status"] = STATUS_EXPERIMENT_COMPLETE
        out["invalid_reasons"] = out.get("invalid_reasons", []) + ["POST_WRITE_EXCEPTION"]
        out["experiment_valid"] = False
        out["result_code"] = "X"
        out["physical_motion_claim"] = physical_motion_claim("X")
        out["post_write_exception"] = repr(e)

    persisted = _persist_r3_result(run_root, out)
    if persisted not in (None, True):
        out["invalid_reasons"] = out.get("invalid_reasons", []) + ["EVIDENCE_PERSISTENCE_FAILED"]
        out["experiment_valid"] = False
        out["result_code"] = "X"
        out["physical_motion_claim"] = physical_motion_claim("X")
        out["evidence_persistence_error"] = persisted

    return out


# ---------------------------------------------------------------------------
# §32 — negative tests (R3-01..R3-22), all mock/synthetic
# ---------------------------------------------------------------------------
def negative_tests():
    results = {}

    def rec(name, cond, detail=None):
        results[name] = {"pass": bool(cond), "detail": detail}

    mock_git_dirty = _make_mock_git(ROOT, dirty=True)
    mock_git_clean = _make_mock_git(ROOT, dirty=False)

    def pre_window_ok():
        return {"valid": True, "median": {"translation_x_px": 0.0, "translation_y_px": 0.0, "rotation_deg": 0.0}}, "ANCHOR"

    # R3-01..04 — opt-in / operator gates
    r = execute_motion_experiment(allow_motion_env=None, execute_motion_flag=False)
    rec("R3-01_no_opt_in_no_motion", r["status"] == "ABORTED" and r["motion_commands_sent"] == 0, r)

    r = execute_motion_experiment(allow_motion_env="1", execute_motion_flag=False)
    rec("R3-02_env_only_no_motion", r["status"] == "ABORTED" and r["motion_commands_sent"] == 0, r)

    r = execute_motion_experiment(allow_motion_env=None, execute_motion_flag=True)
    rec("R3-03_flag_only_no_motion", r["status"] == "ABORTED" and r["motion_commands_sent"] == 0, r)

    r = execute_motion_experiment(allow_motion_env="1", execute_motion_flag=True, operator_confirmation="NO")
    rec("R3-04_operator_confirmation_missing_no_motion",
        r["status"] == "ABORTED" and r["reason"] == "OPERATOR_CONFIRMATION_MISSING", r)

    import tempfile

    # 🔴 advisor 3차 BLOCKING 2 — run_root=None 이면 evidence_package_writable/
    # artifact_persistence_path_valid 가 항상 같이 실패해 "각 항목 단독 실패" 를 증명하지
    # 못한다. 전부 쓰기 가능한 tempdir + write 추적 콜백을 주고 failed_checks 를
    # **정확히** 비교한다.
    def isolated_ok_kwargs(write_calls):
        return dict(
            run_root=tempfile.mkdtemp(prefix="r3neg_"),
            operator_confirmation="YES", expected_harness_sha256=sha256(os.path.abspath(__file__)),
            camera_check_fn=lambda: {"dabai_identity_valid": True, "mjpg_640x480_valid": True,
                                      "camera_control_invariant_valid": True},
            controller_read_fn=lambda: 152, process_check_fn=lambda: [],
            capture_pre_window_fn=pre_window_ok,
            controller_write_fn=lambda *a: write_calls.append(a))

    # R3-05 — dirty git tree -> abort before motion (gate failure, git_provenance_valid 단독)
    write_calls_05 = []
    r = execute_motion_experiment(allow_motion_env="1", execute_motion_flag=True, git_run_fn=mock_git_dirty,
                                  **isolated_ok_kwargs(write_calls_05))
    rec("R3-05_dirty_git_tree_abort",
        r["status"] == "ABORT_BEFORE_MOTION" and r.get("failed_checks") == ["git_provenance_valid"]
        and not write_calls_05, r)

    # R3-06 — preregistration SHA mismatch -> abort before motion (preregistration_sha_valid 단독)
    write_calls_06 = []
    orig_prereg_sha = globals()["PREREG_SHA"]
    globals()["PREREG_SHA"] = "0" * 64
    try:
        r = execute_motion_experiment(allow_motion_env="1", execute_motion_flag=True, git_run_fn=mock_git_clean,
                                      **isolated_ok_kwargs(write_calls_06))
    finally:
        globals()["PREREG_SHA"] = orig_prereg_sha
    rec("R3-06_preregistration_sha_mismatch_abort",
        r["status"] == "ABORT_BEFORE_MOTION" and r.get("failed_checks") == ["preregistration_sha_valid"]
        and not write_calls_06, r)

    # R3-07 — module SHA mismatch -> abort before motion (module_sha_valid 단독)
    write_calls_07 = []
    orig_module_sha = dict(MODULE_SHA)
    MODULE_SHA[list(MODULE_SHA.keys())[0]] = "0" * 64
    try:
        r = execute_motion_experiment(allow_motion_env="1", execute_motion_flag=True, git_run_fn=mock_git_clean,
                                      **isolated_ok_kwargs(write_calls_07))
    finally:
        MODULE_SHA.clear()
        MODULE_SHA.update(orig_module_sha)
    rec("R3-07_module_sha_mismatch_abort",
        r["status"] == "ABORT_BEFORE_MOTION" and r.get("failed_checks") == ["module_sha_valid"]
        and not write_calls_07, r)

    # R3-08 — wrong camera format -> abort before motion (mjpg_640x480_valid 단독)
    write_calls_08 = []
    kwargs08 = isolated_ok_kwargs(write_calls_08)
    kwargs08["camera_check_fn"] = lambda: {"dabai_identity_valid": True, "mjpg_640x480_valid": False,
                                           "camera_control_invariant_valid": True}
    r = execute_motion_experiment(allow_motion_env="1", execute_motion_flag=True, git_run_fn=mock_git_clean, **kwargs08)
    rec("R3-08_wrong_camera_format_abort",
        r["status"] == "ABORT_BEFORE_MOTION" and r.get("failed_checks") == ["mjpg_640x480_valid"]
        and not write_calls_08, r)

    # R3-09 — PRE visual invalid -> abort before motion (pre_visual_window_valid +
    # roi_feature_guards_pass 는 같은 신호를 공유해 둘 다 실패한다 — 설계상 의도된 쌍)
    write_calls_09 = []
    kwargs09 = isolated_ok_kwargs(write_calls_09)
    kwargs09["capture_pre_window_fn"] = lambda: ({"valid": False, "median": None}, None)
    r = execute_motion_experiment(allow_motion_env="1", execute_motion_flag=True, git_run_fn=mock_git_clean, **kwargs09)
    rec("R3-09_pre_visual_invalid_abort_before_motion",
        r["status"] == "ABORT_BEFORE_MOTION"
        and set(r.get("failed_checks", [])) == {"pre_visual_window_valid", "roi_feature_guards_pass"}
        and not write_calls_09, r)

    # R3-10 — controller baseline != 152 -> abort before motion (controller_baseline_valid 단독)
    write_calls_10 = []
    kwargs10 = isolated_ok_kwargs(write_calls_10)
    kwargs10["controller_read_fn"] = lambda: 150
    r = execute_motion_experiment(allow_motion_env="1", execute_motion_flag=True, git_run_fn=mock_git_clean, **kwargs10)
    rec("R3-10_controller_baseline_mismatch_abort",
        r["status"] == "ABORT_BEFORE_MOTION" and r.get("failed_checks") == ["controller_baseline_valid"]
        and not write_calls_10, r)

    # R3-11..15 — result mapping (pure function)
    rec("R3-11_result_A_mapping",
        motion_result_matrix(True, True, True) == "A"
        and physical_motion_claim("A") == "PHYSICAL_MOTION_EVIDENCE_SUPPORTED")
    rec("R3-12_result_B_mapping",
        motion_result_matrix(True, False, True) == "B"
        and physical_motion_claim("B") == "PHYSICAL_MOTION_NOT_ESTABLISHED")
    rec("R3-13_result_C_mapping",
        motion_result_matrix(False, True, True) == "C"
        and physical_motion_claim("C") == "UNEXPECTED_VISUAL_MOTION")
    rec("R3-14_result_D_mapping",
        motion_result_matrix(False, False, True) == "D"
        and physical_motion_claim("D") == "NO_OBSERVABLE_RESPONSE")
    rec("R3-15_result_X_mapping",
        motion_result_matrix(True, True, False) == "X"
        and physical_motion_claim("X") == "EXPERIMENT_INVALID")

    # R3-16..19 — command validator
    rec("R3-16_command_budget_exceeded_rejected",
        not validate_command_request(SERVO_ID, COMMAND_TARGET_ID2, COMMAND_DURATION_MS, 2)["accepted"])
    rec("R3-17_wrong_servo_id_rejected",
        not validate_command_request(1, COMMAND_TARGET_ID2, COMMAND_DURATION_MS, 1)["accepted"])
    rec("R3-18_wrong_target_rejected",
        not validate_command_request(SERVO_ID, 160, COMMAND_DURATION_MS, 1)["accepted"])
    rec("R3-19_wrong_duration_rejected",
        not validate_command_request(SERVO_ID, COMMAND_TARGET_ID2, 500, 1)["accepted"])

    # R3-20 — automatic return attempt rejected
    rec("R3-20_automatic_return_rejected",
        reject_automatic_return([154, 152])["rejected"]
        and reject_automatic_return([152, 150])["rejected"]
        and not reject_automatic_return([154])["rejected"])

    # R3-21 — evidence_pipeline module SHA enforced specifically
    orig_evp_sha = MODULE_SHA["scripts/p007e_r2r2_evidence_pipeline.py"]
    MODULE_SHA["scripts/p007e_r2r2_evidence_pipeline.py"] = "0" * 64
    try:
        chk = check_module_sha()
    finally:
        MODULE_SHA["scripts/p007e_r2r2_evidence_pipeline.py"] = orig_evp_sha
    rec("R3-21_evidence_pipeline_module_sha_enforced",
        not chk["match"] and "scripts/p007e_r2r2_evidence_pipeline.py" in chk["mismatches"], chk)

    # R3-22 — incomplete artifact package detected (R3 자체 artifact contract)
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        completeness = check_r3_artifact_completeness(tmp)
    rec("R3-22_incomplete_artifact_package_detected", completeness["status"] != "COMPLETE", completeness)

    all_pass = all(v["pass"] for v in results.values())
    return {"all_pass": all_pass, "results": results}


# ---------------------------------------------------------------------------
# R3X — 추가 시험(의뢰서 R3-01..22 밖, advisor 재검토로 추가된 긍정경로/하드게이트/
# anchor계약/crash-safety 시험, C1-R1-R1 의 E16/E17 선례와 같은 방식으로 별도 표기)
# ---------------------------------------------------------------------------
def extended_tests():
    import shutil
    import tempfile
    results = {}
    cleanup_dirs = []

    def rec(name, cond, detail=None):
        results[name] = {"pass": bool(cond), "detail": detail}

    def new_run_root(prefix):
        d = tempfile.mkdtemp(prefix=prefix)
        cleanup_dirs.append(d)
        return d

    mock_git_clean = _make_mock_git(ROOT, dirty=False)
    expected_sha = sha256(os.path.abspath(__file__))
    zero = {"translation_x_px": 0.0, "translation_y_px": 0.0, "rotation_deg": 0.0}
    moved = {"translation_x_px": 25.0, "translation_y_px": 0.0, "rotation_deg": 0.0}

    def good_kwargs(pre_median, post_median, post_read_value, run_root=None, process_after=None):
        if run_root is None:
            run_root = new_run_root("r3x_")
        write_calls = []

        def write_fn(*a):
            write_calls.append(a)
        pre_reads = iter([152, 152, 152])
        post_reads = iter([post_read_value] * POST_READ_ROUNDS)

        def read_fn():
            try:
                return next(pre_reads)
            except StopIteration:
                return next(post_reads)
        kwargs = dict(
            run_root=run_root,
            allow_motion_env="1", execute_motion_flag=True, operator_confirmation="YES",
            expected_harness_sha256=expected_sha, git_run_fn=mock_git_clean,
            camera_check_fn=lambda: {"dabai_identity_valid": True, "mjpg_640x480_valid": True,
                                      "camera_control_invariant_valid": True},
            capture_pre_window_fn=lambda: ({"valid": True, "median": dict(pre_median)}, "ANCHOR_OBJ"),
            capture_post_window_fn=lambda anchor: {"valid": True, "median": dict(post_median)},
            controller_read_fn=read_fn, controller_write_fn=write_fn,
            process_check_fn=(lambda: []) if process_after is None else _toggle_process_check(process_after),
            operator_observation_fn=lambda: "YES",
        )
        return kwargs, write_calls, run_root

    try:
        # R3X-01 — 전체 happy path -> result A, write 정확히 1회, 인자 (2,154,2000)
        kwargs, writes, _ = good_kwargs(zero, moved, post_read_value=154)
        r = execute_motion_experiment(**kwargs)
        rec("R3X-01_happy_path_result_A",
            r.get("status") == "EXPERIMENT_COMPLETE" and r["result_code"] == "A"
            and r["physical_motion_claim"] == "PHYSICAL_MOTION_EVIDENCE_SUPPORTED"
            and writes == [(SERVO_ID, COMMAND_TARGET_ID2, COMMAND_DURATION_MS)], r)

        # R3X-02 — result B: controller changed, visual not changed
        kwargs, writes, _ = good_kwargs(zero, zero, post_read_value=154)
        r = execute_motion_experiment(**kwargs)
        rec("R3X-02_result_B", r.get("status") == "EXPERIMENT_COMPLETE" and r["result_code"] == "B"
            and r["physical_motion_claim"] == "PHYSICAL_MOTION_NOT_ESTABLISHED", r)

        # R3X-03 — result C: controller not changed, visual changed
        kwargs, writes, _ = good_kwargs(zero, moved, post_read_value=152)
        r = execute_motion_experiment(**kwargs)
        rec("R3X-03_result_C", r.get("status") == "EXPERIMENT_COMPLETE" and r["result_code"] == "C"
            and r["physical_motion_claim"] == "UNEXPECTED_VISUAL_MOTION", r)

        # R3X-04 — result D: neither changed
        kwargs, writes, _ = good_kwargs(zero, zero, post_read_value=152)
        r = execute_motion_experiment(**kwargs)
        rec("R3X-04_result_D", r.get("status") == "EXPERIMENT_COMPLETE" and r["result_code"] == "D"
            and r["physical_motion_claim"] == "NO_OBSERVABLE_RESPONSE", r)

        # R3X-05 — result X: 명령은 보내졌으나 POST window 불량(experiment_valid=False)
        write_calls_05 = []

        def write_fn_05(*a):
            write_calls_05.append(a)
        run_root_05 = new_run_root("r3x05_")
        r = execute_motion_experiment(
            run_root=run_root_05,
            allow_motion_env="1", execute_motion_flag=True, operator_confirmation="YES",
            expected_harness_sha256=expected_sha, git_run_fn=mock_git_clean,
            camera_check_fn=lambda: {"dabai_identity_valid": True, "mjpg_640x480_valid": True,
                                      "camera_control_invariant_valid": True},
            capture_pre_window_fn=lambda: ({"valid": True, "median": dict(zero)}, "ANCHOR"),
            capture_post_window_fn=lambda anchor: {"valid": False, "median": None},
            controller_read_fn=lambda: 152, controller_write_fn=write_fn_05,
            process_check_fn=lambda: [])
        rec("R3X-05_result_X_post_window_invalid_but_write_already_sent",
            r.get("status") == "EXPERIMENT_COMPLETE" and r["result_code"] == "X" and r["motion_commands_sent"] == 1
            and "POST_WINDOW_QUALITY_GUARD_FAILED" in r["invalid_reasons"]
            and write_calls_05 == [(SERVO_ID, COMMAND_TARGET_ID2, COMMAND_DURATION_MS)], r)

        def isolated_ok_kwargs_ext(write_calls, controller_read_fn=None):
            # 🔴 advisor 3차 BLOCKING 2 — run_root 를 쓰기 가능하게 둬서 각 음성시험이
            # 의도한 항목 **단독**으로만 실패하는지 정확히 비교한다
            return dict(
                run_root=new_run_root("r3xneg_"),
                allow_motion_env="1", execute_motion_flag=True, operator_confirmation="YES",
                expected_harness_sha256=expected_sha, git_run_fn=mock_git_clean,
                camera_check_fn=lambda: {"dabai_identity_valid": True, "mjpg_640x480_valid": True,
                                          "camera_control_invariant_valid": True},
                capture_pre_window_fn=lambda: ({"valid": True, "median": dict(zero)}, "ANCHOR"),
                controller_read_fn=controller_read_fn or (lambda: 152),
                controller_write_fn=lambda *a: write_calls.append(a),
                process_check_fn=lambda: [])

        # R3X-06 — harness SHA mismatch -> abort before motion (harness_sha_valid 단독)
        write_calls_06 = []
        kwargs06 = isolated_ok_kwargs_ext(write_calls_06)
        kwargs06["expected_harness_sha256"] = "0" * 64
        r = execute_motion_experiment(**kwargs06)
        rec("R3X-06_harness_sha_mismatch_abort",
            r["status"] == "ABORT_BEFORE_MOTION" and r["failed_checks"] == ["harness_sha_valid"]
            and not write_calls_06, r)

        # R3X-07 — quiet interval violation (clock_fn 이 간격 0 을 보고) -> abort before motion
        # (host_control_quiet_interval_satisfied 단독)
        write_calls_07 = []
        kwargs07 = isolated_ok_kwargs_ext(write_calls_07)
        kwargs07["clock_fn"] = lambda: 0
        kwargs07["sleep_fn"] = lambda s: None
        r = execute_motion_experiment(**kwargs07)
        rec("R3X-07_quiet_interval_violation_abort",
            r["status"] == "ABORT_BEFORE_MOTION"
            and r["failed_checks"] == ["host_control_quiet_interval_satisfied"]
            and not write_calls_07, r)

        # R3X-08 — unexpected motion-capable process pre-motion -> abort before motion
        # (no_unexpected_motion_capable_process 단독)
        write_calls_08x = []
        kwargs08x = isolated_ok_kwargs_ext(write_calls_08x)
        kwargs08x["process_check_fn"] = lambda: ["moveit_fake_pid_1234"]
        r = execute_motion_experiment(**kwargs08x)
        rec("R3X-08_unexpected_process_abort",
            r["status"] == "ABORT_BEFORE_MOTION"
            and r["failed_checks"] == ["no_unexpected_motion_capable_process"]
            and not write_calls_08x, r)

        # R3X-09 — run_root 가 쓰기 불가(상위 경로가 디렉터리가 아님) -> abort before motion (write 미호출)
        write_calls_09x = []
        blocking_file = os.path.join(new_run_root("r3x09_blocker_"), "not_a_directory")
        open(blocking_file, "w").close()
        bad_run_root = os.path.join(blocking_file, "cannot_be_created_under_a_file")
        r = execute_motion_experiment(
            run_root=bad_run_root,
            allow_motion_env="1", execute_motion_flag=True, operator_confirmation="YES",
            expected_harness_sha256=expected_sha, git_run_fn=mock_git_clean,
            camera_check_fn=lambda: {"dabai_identity_valid": True, "mjpg_640x480_valid": True,
                                      "camera_control_invariant_valid": True},
            capture_pre_window_fn=lambda: ({"valid": True, "median": dict(zero)}, "ANCHOR"),
            controller_read_fn=lambda: 152, controller_write_fn=lambda *a: write_calls_09x.append(a),
            process_check_fn=lambda: [])
        rec("R3X-09_unwritable_run_root_abort_before_motion",
            r["status"] == "ABORT_BEFORE_MOTION"
            and set(r["failed_checks"]) == {"evidence_package_writable", "artifact_persistence_path_valid"}
            and not write_calls_09x, r)

        # R3X-10 — 부분 None pre-read([152,None,152]) -> abort before motion
        # (controller_baseline_valid 단독, 쓰기 가능한 run_root 로 격리)
        write_calls_10x = []
        pre_vals_10 = iter([152, None, 152])
        kwargs10x = isolated_ok_kwargs_ext(write_calls_10x, controller_read_fn=lambda: next(pre_vals_10))
        r = execute_motion_experiment(**kwargs10x)
        rec("R3X-10_partial_none_preread_abort",
            r["status"] == "ABORT_BEFORE_MOTION" and r["failed_checks"] == ["controller_baseline_valid"]
            and not write_calls_10x, r)

        # R3X-11 — anchor identity: PRE 가 반환한 anchor 객체가 POST 콜백에 그대로 전달되는지
        received_anchor = []
        anchor_token = object()
        kwargs, writes, _ = good_kwargs(zero, moved, post_read_value=154)
        kwargs["capture_pre_window_fn"] = lambda: ({"valid": True, "median": dict(zero)}, anchor_token)

        def capture_post_check_anchor(a):
            received_anchor.append(a)
            return {"valid": True, "median": dict(moved)}
        kwargs["capture_post_window_fn"] = capture_post_check_anchor
        r = execute_motion_experiment(**kwargs)
        rec("R3X-11_anchor_identity_passed_to_post_window",
            r.get("status") == "EXPERIMENT_COMPLETE" and len(received_anchor) == 1
            and received_anchor[0] is anchor_token, {"received_is_same_object": received_anchor and received_anchor[0] is anchor_token})

        # R3X-12 — POST_WRITE_EXCEPTION: POST 콜백이 raise 해도 디스크에 motion_commands_sent=1 + X 가 남는다
        run_root_12 = new_run_root("r3x12_")

        def raising_post_window(anchor):
            raise RuntimeError("simulated POST window capture failure")
        r = execute_motion_experiment(
            run_root=run_root_12,
            allow_motion_env="1", execute_motion_flag=True, operator_confirmation="YES",
            expected_harness_sha256=expected_sha, git_run_fn=mock_git_clean,
            camera_check_fn=lambda: {"dabai_identity_valid": True, "mjpg_640x480_valid": True,
                                      "camera_control_invariant_valid": True},
            capture_pre_window_fn=lambda: ({"valid": True, "median": dict(zero)}, "ANCHOR"),
            capture_post_window_fn=raising_post_window,
            controller_read_fn=lambda: 152, controller_write_fn=lambda *a: None,
            process_check_fn=lambda: [])
        on_disk = json.load(open(os.path.join(run_root_12, "r3_result.json")))
        rec("R3X-12_post_write_exception_persisted_on_disk",
            r["result_code"] == "X" and r["motion_commands_sent"] == 1
            and "POST_WRITE_EXCEPTION" in r["invalid_reasons"]
            and on_disk["motion_commands_sent"] == 1 and on_disk["result_code"] == "X", r)

        # R3X-13 — EVIDENCE_PERSISTENCE_FAILED: POST 콜백이 run_root 자체를 지워 최종 저장이 실패
        run_root_13 = new_run_root("r3x13_")

        def destructive_post_window(anchor):
            shutil.rmtree(run_root_13, ignore_errors=True)
            return {"valid": True, "median": dict(moved)}
        r = execute_motion_experiment(
            run_root=run_root_13,
            allow_motion_env="1", execute_motion_flag=True, operator_confirmation="YES",
            expected_harness_sha256=expected_sha, git_run_fn=mock_git_clean,
            camera_check_fn=lambda: {"dabai_identity_valid": True, "mjpg_640x480_valid": True,
                                      "camera_control_invariant_valid": True},
            capture_pre_window_fn=lambda: ({"valid": True, "median": dict(zero)}, "ANCHOR"),
            capture_post_window_fn=destructive_post_window,
            controller_read_fn=lambda: 152, controller_write_fn=lambda *a: None,
            process_check_fn=lambda: [])
        rec("R3X-13_evidence_persistence_failed_forces_X",
            r["result_code"] == "X" and r["motion_commands_sent"] == 1
            and "EVIDENCE_PERSISTENCE_FAILED" in r["invalid_reasons"], r)

        # R3X-14 — 정상 완주한 run_root 는 check_r3_artifact_completeness() 가 COMPLETE 를 반환해야 한다
        # (지금까지 COMPLETE 분기를 실제로 행사한 시험이 없었다 — advisor 3차 지적)
        kwargs14, writes14, run_root_14 = good_kwargs(zero, moved, post_read_value=154)
        r14 = execute_motion_experiment(**kwargs14)
        completeness_14 = check_r3_artifact_completeness(run_root_14)
        rec("R3X-14_completeness_complete_on_normal_finish",
            r14.get("status") == "EXPERIMENT_COMPLETE" and completeness_14["status"] == "COMPLETE",
            completeness_14)

        # R3X-15 — "write 직후 프로세스가 죽는다"를 흉내낸다: POST window 콜백 안에서 디스크의
        # r3_result.json 을 읽어, 그 시점 기록이 IN_PROGRESS + motion_commands_sent=1 인지,
        # 그리고 completeness 가 INCOMPLETE 인지 확인한다(advisor 3차 BLOCKING 1)
        run_root_15 = new_run_root("r3x15_")
        mid_flight_snapshot = {}

        def capture_mid_flight_post_window(anchor):
            with open(os.path.join(run_root_15, "r3_result.json")) as f:
                mid_flight_snapshot["disk"] = json.load(f)
            mid_flight_snapshot["completeness"] = check_r3_artifact_completeness(run_root_15)
            return {"valid": True, "median": dict(moved)}
        r15 = execute_motion_experiment(
            run_root=run_root_15,
            allow_motion_env="1", execute_motion_flag=True, operator_confirmation="YES",
            expected_harness_sha256=expected_sha, git_run_fn=mock_git_clean,
            camera_check_fn=lambda: {"dabai_identity_valid": True, "mjpg_640x480_valid": True,
                                      "camera_control_invariant_valid": True},
            capture_pre_window_fn=lambda: ({"valid": True, "median": dict(zero)}, "ANCHOR"),
            capture_post_window_fn=capture_mid_flight_post_window,
            controller_read_fn=lambda: 152, controller_write_fn=lambda *a: None,
            process_check_fn=lambda: [])
        rec("R3X-15_mid_flight_snapshot_is_in_progress_and_incomplete",
            mid_flight_snapshot.get("disk", {}).get("status") == STATUS_COMMAND_SENT_IN_PROGRESS
            and mid_flight_snapshot.get("disk", {}).get("motion_commands_sent") == 1
            and mid_flight_snapshot.get("completeness", {}).get("status") == "EVIDENCE_PACKAGE_INCOMPLETE"
            and r15.get("status") == STATUS_EXPERIMENT_COMPLETE, mid_flight_snapshot)
    finally:
        for d in cleanup_dirs:
            shutil.rmtree(d, ignore_errors=True)

    all_pass = all(v["pass"] for v in results.values())
    return {"all_pass": all_pass, "results": results}


def _toggle_process_check(process_after):
    state = {"calls": 0}

    def fn():
        state["calls"] += 1
        return [] if state["calls"] == 1 else process_after
    return fn


def selftest():
    return {"self_ast_check": self_ast_check(),
            "negative_tests_R3_01_R3_22": negative_tests(),
            "extended_tests_R3X": extended_tests()}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--self-test", action="store_true",
                    help="self_ast_check + negative_tests(R3-01..R3-22) + extended_tests(R3X) 실행, "
                         "카메라/로봇 접촉 없음")
    ap.add_argument("--execute-motion", action="store_true",
                    help="§31 opt-in 플래그. env P007E_R2_R3_ALLOW_MOTION=1 와 함께여야 한다. "
                         "이 harness 는 capture_*_fn/controller_*_fn 이 주입되지 않으면 "
                         "ABORT_BEFORE_MOTION 또는 HARDWARE_WIRING_NOT_IMPLEMENTED_IN_THIS_STAGE 로 abort 한다")
    args = ap.parse_args()

    if args.self_test:
        out = selftest()
        print(json.dumps(out, ensure_ascii=False, indent=1))
        ok = (out["self_ast_check"]["pass"] and out["negative_tests_R3_01_R3_22"]["all_pass"]
              and out["extended_tests_R3X"]["all_pass"])
        sys.exit(0 if ok else 1)

    env_val = os.environ.get(ALLOW_ENV)
    result = execute_motion_experiment(allow_motion_env=env_val, execute_motion_flag=args.execute_motion)
    print(json.dumps(result, ensure_ascii=False, indent=1))
    sys.exit(0 if result.get("status") not in ("ABORTED", "ABORT_BEFORE_MOTION") else 1)


if __name__ == "__main__":
    main()
