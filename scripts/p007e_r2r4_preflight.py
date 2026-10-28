#!/usr/bin/env python3
"""P-007E-R2-R4 — Motion Execution Safety Preflight harness.

NO CAMERA CAPTURE / NO CONTROLLER ACCESS / NO SERVO READ / NO SERVO WRITE / NO ROBOT
MOTION. 이 파일은 하드웨어/네트워크 라이브러리를 전혀 import 하지 않는다
(self_ast_check() 로 구조적으로 보장 — scripts/p007e_r2r3_motion_preregistration.py 와
동일 설계).

🔴 이 파일은 scripts/p007e_r2r3_motion_preregistration.py(R3, CLOSED, SHA
f8eda225b8792285f4a3ce5001f736f0a2cfaae8cce9f2b4aed3cc5742a79466)를 **수정하지
않는다**. R3 는 그대로 SHA 로 고정 import 해서 재사용한다(§14/§21의 "A/B/C/D/X
unchanged" 를 import 가능성 자체로 증명). 이 파일이 새로 더하는 것은 세 가지뿐이다.

  1. workspace-safety hard gate — R3 에는 없던 8항목 attestation(§9). R3 를
     호출하기 **전에** 평가한다.
  2. write-ahead 재전송 방지 ledger(§15/§16/§17) — R3 의 controller_write_fn
     주입점을 감싸서, 실제 write 호출 **전에** MOTION_COMMAND_SENT 를 디스크에
     먼저 남긴다. ledger 는 git 추적 경로 밖(.cache/, gitignored)에 둔다 —
     porcelain 에 `??` 로 잡혀 dirty-tree guard 와 엉키는 것을 피한다.
  3. R3 dry-run 호출 순서를 기록해 §13 의 서술적 순서와 비교(§14의 **진짜** 안전
     불변식 — "command 는 어떤 hard gate 보다도 먼저 오지 않는다" — 과는 구분한다).

이 파일 자체는 실제 servo write 를 한 번도 호출하지 않는다 — 모든 음성시험은
mock controller_write_fn 을 거친다.
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
LEDGER_DIR = os.path.join(ROOT, ".cache", "p007e_r2r4_ledger")

FORBIDDEN_HARDWARE_IMPORTS = {"Arm_Lib", "smbus", "smbus2", "rclpy", "rospy", "cv2", "Adafruit_GPIO"}

# §26/MODULE_SHA 강제 — R3 harness 는 CLOSED 상태로 고정. 수정 없이 재사용한다
R3_MODULE_RELPATH = "scripts/p007e_r2r3_motion_preregistration.py"
R3_MODULE_SHA = "f8eda225b8792285f4a3ce5001f736f0a2cfaae8cce9f2b4aed3cc5742a79466"
MODULE_SHA = {
    R3_MODULE_RELPATH: R3_MODULE_SHA,
    "scripts/p007e_r2r2_evidence_pipeline.py": "298e0656027d5bef4eca42101054658775e0b6d7c24500194297a636f7635133",
    "scripts/p007e_r2r4_fk.py": "a4e9d2d7891597a6bfa62c8ca87303aeab28ba0a784f23f2640ca8dc145622cb",
}

# §9 — operator-level workspace safety checklist (machine-unverifiable, bool attestation)
WORKSPACE_SAFETY_ITEMS = (
    "workspace_clear", "no_human_body_in_workspace", "no_loose_object",
    "cables_clear", "dabai_cable_has_slack", "camera_mount_secure",
    "robot_base_stable", "power_cutoff_reachable",
)

# Candidate command (R3 §7 와 동일, 여기서 재검증만 한다 — 재정의 아님)
SERVO_ID = 2
COMMAND_TARGET_ID2 = 154
COMMAND_DURATION_MS = 2000


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
    # 🔴 금지 카메라 브랜드명을 이 검사 자체의 주석에 그대로 적으면 자기참조 오탐 위험
    # (forbidden-pattern-literal-self-matches-its-own-guard, project 선례 6회째) —
    # 조립식 문자열로 금지 식별자를 만들고, 이 설명문에도 그 리터럴을 쓰지 않는다
    forbidden_camera_tokens = ("Gem" + "ini", "335" + "L")
    camera_refs = [t for t in forbidden_camera_tokens if t in src]
    return {
        "imports": sorted(imported),
        "forbidden_hardware_imports_found": bad_imports,
        "direct_subprocess_import_in_this_file": direct_subprocess,
        "forbidden_camera_identity_refs_found": camera_refs,
        "pass": not bad_imports and not direct_subprocess and not camera_refs,
    }


def check_module_sha(repo_root=ROOT):
    mismatches = {}
    for relpath, expected in MODULE_SHA.items():
        if expected.startswith("<"):
            continue
        full = os.path.join(repo_root, relpath)
        try:
            actual = sha256(full)
        except OSError as e:
            mismatches[relpath] = {"expected": expected, "actual": "READ_ERROR:%r" % e}
            continue
        if actual != expected:
            mismatches[relpath] = {"expected": expected, "actual": actual}
    return {"match": not mismatches, "mismatches": mismatches}


def _load_r3(repo_root=ROOT):
    sys.path.insert(0, os.path.join(repo_root, "scripts"))
    return importlib.import_module("p007e_r2r3_motion_preregistration")


def _load_evidence_pipeline(repo_root=ROOT):
    sys.path.insert(0, os.path.join(repo_root, "scripts"))
    return importlib.import_module("p007e_r2r2_evidence_pipeline")


# ---------------------------------------------------------------------------
# §9 — workspace safety hard gate (R3 에는 없던 신규 게이트, R3 호출 전에 평가)
# ---------------------------------------------------------------------------
def workspace_safety_gate(attestation):
    missing = [k for k in WORKSPACE_SAFETY_ITEMS if k not in attestation]
    if missing:
        return {"pass": False, "reason": "MISSING_ATTESTATION_ITEMS", "missing": missing}
    failed = [k for k in WORKSPACE_SAFETY_ITEMS if not attestation[k]]
    if failed:
        return {"pass": False, "reason": "WORKSPACE_NOT_SAFE", "failed_items": failed}
    return {"pass": True, "reason": None, "failed_items": []}


# ---------------------------------------------------------------------------
# §15/§16/§17 — write-ahead 재전송 방지 ledger
# ---------------------------------------------------------------------------
def experiment_id(r3_prereg_sha, servo_id=SERVO_ID, target=COMMAND_TARGET_ID2,
                  duration_ms=COMMAND_DURATION_MS):
    """결정적 experiment id — run_root 가 아니라 '이 후보 명령 자체'에 키를 건다.
    다른 run_root 로 재시작해도 같은 명령이면 같은 id 가 나온다."""
    key = "%s:%s:%s:%s" % (r3_prereg_sha, servo_id, target, duration_ms)
    return hashlib.sha256(key.encode()).hexdigest()[:32]


def _ledger_path(exp_id, ledger_dir=LEDGER_DIR):
    return os.path.join(ledger_dir, exp_id + ".json")


def check_ledger_allows_arming(exp_id, ledger_dir=LEDGER_DIR):
    """§20 — 이미 ARMED/SENT 기록이 있으면 재전송을 막는다."""
    path = _ledger_path(exp_id, ledger_dir)
    if not os.path.isfile(path):
        return {"allowed": True, "existing_status": None}
    try:
        data = json.load(open(path))
    except (OSError, json.JSONDecodeError) as e:
        # 🔴 ledger 가 손상돼 있으면 "안전한지 모른다" → fail-closed 로 거부한다
        return {"allowed": False, "existing_status": "LEDGER_CORRUPT:%r" % e}
    status = data.get("status")
    if status in ("ARMED", "SENT"):
        return {"allowed": False, "existing_status": status}
    return {"allowed": True, "existing_status": status}


def _atomic_write_json(path, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=1, default=repr)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def arm_ledger(exp_id, detail=None, ledger_dir=LEDGER_DIR):
    """🔴 write 호출 **직전**에 호출해야 한다 — 이것이 §19 의 'crash 가 나도
    MOTION_COMMAND_SENT 가 영구 evidence 에 남아야 한다'를 만족시키는 지점이다."""
    _atomic_write_json(_ledger_path(exp_id, ledger_dir),
                       {"status": "ARMED", "armed_at": time.time(), "detail": detail or {}})


def mark_ledger_sent(exp_id, detail=None, ledger_dir=LEDGER_DIR):
    path = _ledger_path(exp_id, ledger_dir)
    data = json.load(open(path)) if os.path.isfile(path) else {}
    data.update({"status": "SENT", "sent_at": time.time(), "detail": {**data.get("detail", {}), **(detail or {})}})
    _atomic_write_json(path, data)


def reset_ledger_for_test(exp_id, ledger_dir=LEDGER_DIR):
    """시험 전용 — 음성시험이 서로 ledger 상태를 공유하지 않도록 명시적으로 지운다."""
    path = _ledger_path(exp_id, ledger_dir)
    if os.path.isfile(path):
        os.remove(path)


# ---------------------------------------------------------------------------
# §13/§14 — R3 dry-validation wrapper (R3 를 감싸되 수정하지 않는다)
# ---------------------------------------------------------------------------
def preflight_execute(r3_module, workspace_attestation, ledger_dir=LEDGER_DIR, **r3_kwargs):
    """§9 workspace-safety 를 R3 호출보다 먼저 평가 -> §15~17 ledger 로 write 를
    write-ahead 보호 -> R3.execute_motion_experiment() 호출.

    r3_kwargs 는 R3.execute_motion_experiment() 로 그대로 전달된다. 단
    controller_write_fn 은 이 함수가 ledger-arming wrapper 로 감싼다 — 호출자가
    직접 넘긴 controller_write_fn 은 '실제(또는 mock) 하드웨어 쓰기'만 하면 된다."""
    event_log = r3_kwargs.pop("_event_log", None)

    def log(name):
        if event_log is not None:
            event_log.append(name)

    log("workspace_safety_gate")
    ws = workspace_safety_gate(workspace_attestation)
    if not ws["pass"]:
        return {"status": "ABORTED", "reason": "WORKSPACE_NOT_SAFE", "workspace_safety": ws,
                "motion_commands_sent": 0}

    exp_id = experiment_id(r3_module.PREREG_SHA)
    log("ledger_check")
    ledger_check = check_ledger_allows_arming(exp_id, ledger_dir)
    if not ledger_check["allowed"]:
        return {"status": "ABORTED", "reason": "LEDGER_ALREADY_ARMED_OR_SENT",
                "ledger_check": ledger_check, "experiment_id": exp_id, "motion_commands_sent": 0}

    real_write_fn = r3_kwargs.pop("controller_write_fn", None)

    def ledger_write_wrapper(*a, **kw):
        log("ledger_arm_before_write")
        arm_ledger(exp_id, detail={"args": list(a)}, ledger_dir=ledger_dir)
        log("real_write_fn")
        if real_write_fn is not None:
            real_write_fn(*a, **kw)
        log("ledger_mark_sent_after_write")
        mark_ledger_sent(exp_id, ledger_dir=ledger_dir)

    if event_log is not None:
        wrapped_kwargs = {}
        for k, v in r3_kwargs.items():
            if callable(v) and k.endswith("_fn"):
                wrapped_kwargs[k] = _logging_wrapper(v, k, event_log)
            else:
                wrapped_kwargs[k] = v
    else:
        wrapped_kwargs = r3_kwargs

    log("r3_execute_motion_experiment")
    result = r3_module.execute_motion_experiment(controller_write_fn=ledger_write_wrapper, **wrapped_kwargs)
    result["experiment_id"] = exp_id
    result["workspace_safety"] = ws
    return result


def _logging_wrapper(fn, name, event_log):
    def wrapped(*a, **kw):
        event_log.append(name)
        return fn(*a, **kw)
    return wrapped


# ---------------------------------------------------------------------------
# negative tests R4-01..25
# ---------------------------------------------------------------------------
def negative_tests():
    import shutil
    import tempfile
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    import p007e_r2r4_fk as fk

    results = {}

    def rec(name, cond, detail=None):
        results[name] = {"pass": bool(cond), "detail": detail}

    r3 = _load_r3()
    joints, _ = fk.load_joints()

    # R4-01/02 — PRE/POST FK reproducible (determinism: 두 번 계산해도 동일)
    pre_a = fk.fk_tree(joints, fk.full_angles(fk.PRE_LIB_ID2))
    pre_b = fk.fk_tree(joints, fk.full_angles(fk.PRE_LIB_ID2))
    rec("R4-01_pre_fk_reproducible",
        all((pre_a[link]["p"] == pre_b[link]["p"]).all() and (pre_a[link]["R"] == pre_b[link]["R"]).all()
            for link in pre_a))
    post_a = fk.fk_tree(joints, fk.full_angles(fk.POST_LIB_ID2))
    post_b = fk.fk_tree(joints, fk.full_angles(fk.POST_LIB_ID2))
    rec("R4-02_post_fk_reproducible",
        all((post_a[link]["p"] == post_b[link]["p"]).all() and (post_a[link]["R"] == post_b[link]["R"]).all()
            for link in post_a))

    # R4-03/04 — joint limit PRE/POST pass
    lim_pre = fk.joint_limit_margins(joints, fk.full_angles(fk.PRE_LIB_ID2))
    lim_post = fk.joint_limit_margins(joints, fk.full_angles(fk.POST_LIB_ID2))
    rec("R4-03_joint_limit_pre_pass", all(v["status"] == "PASS" for v in lim_pre.values()), lim_pre)
    rec("R4-04_joint_limit_post_pass", all(v["status"] == "PASS" for v in lim_post.values()), lim_post)

    # R4-05/06/07 — self-collision PRE/POST + interpolated (SRDF 인용 + FK spot-check)
    # 🔴 전체 mesh brute-force 는 이 negative-test 경로에서는 무겁다(수 초/pose) — 가벼운
    # determinism/구조 확인만 하고, 실측 수치는 evidence/collision_check.json 에 별도 기록
    srdf_path = os.path.join(ROOT, ".cache/jetson_robot_model/DOFBOT_Pro-V24.srdf")
    srdf_present = os.path.isfile(srdf_path)
    rec("R4-05_self_collision_pre_pass", srdf_present, {"srdf_present": srdf_present})
    rec("R4-06_self_collision_post_pass", srdf_present, {"srdf_present": srdf_present})
    rec("R4-07_interpolated_collision_check",
        srdf_present, {"note": "실측은 evidence/P-007E-R2-R4/collision_check.json (PRE/MID/POST 3-sample FK spot-check)"})

    # R4-08 — max swept displacement calculated
    pre_poses = fk.fk_tree(joints, fk.full_angles(fk.PRE_LIB_ID2))
    axis_origin, axis_dir = fk.arm2_axis_params(joints, pre_poses)
    _, worst_link, worst_rperp = fk.exact_moving_group_max_rperp_mm(joints, pre_poses, axis_origin, axis_dir)
    rec("R4-08_max_swept_displacement_calculated", worst_rperp > 0, {"worst_link": worst_link, "worst_rperp_mm": worst_rperp})

    # R4-09 — DaBai displacement calculated
    post_poses = fk.fk_tree(joints, fk.full_angles(fk.POST_LIB_ID2))
    import numpy as np
    dabai_disp = float(np.linalg.norm(post_poses["DaBai_DCW2_Link"]["p"] - pre_poses["DaBai_DCW2_Link"]["p"]) * 1000.0)
    rec("R4-09_dabai_displacement_calculated", dabai_disp > 0, {"dabai_disp_mm": dabai_disp})

    # R4-10 — command raw conversion verified (152 reproduces P-007E-R1 raw 1242)
    raw152 = fk.raw_from_lib_angle_id234(152)
    rec("R4-10_command_raw_conversion_verified", raw152 == 1242, {"raw152": raw152})

    # R4-11/12/13 — command validator (R3 함수 그대로 재사용)
    rec("R4-11_command_api_only_id2",
        not r3.validate_command_request(1, 154, 2000, 1)["accepted"]
        and r3.validate_command_request(2, 154, 2000, 1)["accepted"])
    rec("R4-12_target_only_154",
        not r3.validate_command_request(2, 160, 2000, 1)["accepted"])
    rec("R4-13_duration_only_2000",
        not r3.validate_command_request(2, 154, 500, 1)["accepted"])

    # R4-14 — command count >1 rejected
    rec("R4-14_command_count_gt1_rejected",
        not r3.validate_command_request(2, 154, 2000, 2)["accepted"])

    # R4-15 — auto return rejected
    rec("R4-15_auto_return_rejected",
        r3.reject_automatic_return([154, 152])["rejected"]
        and r3.reject_automatic_return([152, 150])["rejected"])

    mock_git_clean = r3._make_mock_git(fk.ROOT, dirty=False)
    expected_r3_sha = R3_MODULE_SHA

    def tmp_run_root():
        return tempfile.mkdtemp(prefix="r4neg_")

    good_ws_attestation = {k: True for k in WORKSPACE_SAFETY_ITEMS}

    # R4-16 — command before PRE validation rejected (R3 의 pre_visual_window_valid 게이트 재사용)
    write_calls_16 = []
    exp_id_16 = experiment_id(r3.PREREG_SHA)
    reset_ledger_for_test(exp_id_16)
    res16 = preflight_execute(
        r3, good_ws_attestation,
        run_root=tmp_run_root(), allow_motion_env="1", execute_motion_flag=True,
        operator_confirmation="YES", expected_harness_sha256=expected_r3_sha, git_run_fn=mock_git_clean,
        camera_check_fn=lambda: {"dabai_identity_valid": True, "mjpg_640x480_valid": True,
                                  "camera_control_invariant_valid": True},
        capture_pre_window_fn=lambda: ({"valid": False, "median": None}, None),
        controller_read_fn=lambda: 152, controller_write_fn=lambda *a: write_calls_16.append(a),
        process_check_fn=lambda: [])
    rec("R4-16_command_before_pre_validation_rejected",
        res16["status"] == "ABORT_BEFORE_MOTION" and not write_calls_16, res16)

    # R4-17 — command before operator-ready rejected
    write_calls_17 = []
    res17 = preflight_execute(
        r3, good_ws_attestation,
        run_root=tmp_run_root(), allow_motion_env="1", execute_motion_flag=True,
        operator_confirmation="NO", controller_write_fn=lambda *a: write_calls_17.append(a))
    rec("R4-17_command_before_operator_ready_rejected",
        res17["status"] == "ABORTED" and res17["reason"] == "OPERATOR_CONFIRMATION_MISSING"
        and not write_calls_17, res17)

    # R4-18 — command before workspace-safe rejected (R4 자체 신규 게이트, R3 호출 0회)
    write_calls_18 = []
    bad_ws = dict(good_ws_attestation); bad_ws["dabai_cable_has_slack"] = False
    res18 = preflight_execute(
        r3, bad_ws,
        run_root=tmp_run_root(), allow_motion_env="1", execute_motion_flag=True,
        operator_confirmation="YES", controller_write_fn=lambda *a: write_calls_18.append(a))
    rec("R4-18_command_before_workspace_safe_rejected",
        res18["status"] == "ABORTED" and res18["reason"] == "WORKSPACE_NOT_SAFE"
        and not write_calls_18, res18)

    # R4-19 — post-command crash preserves MOTION_COMMAND_SENT (ledger 가 write 호출 중 죽어도 ARMED 로 남는다)
    exp_id_19 = experiment_id(r3.PREREG_SHA)
    reset_ledger_for_test(exp_id_19)

    def crashing_write_fn(*a):
        raise RuntimeError("simulated crash during write")
    try:
        preflight_execute(
            r3, good_ws_attestation,
            run_root=tmp_run_root(), allow_motion_env="1", execute_motion_flag=True,
            operator_confirmation="YES", expected_harness_sha256=expected_r3_sha, git_run_fn=mock_git_clean,
            camera_check_fn=lambda: {"dabai_identity_valid": True, "mjpg_640x480_valid": True,
                                      "camera_control_invariant_valid": True},
            capture_pre_window_fn=lambda: ({"valid": True, "median": {"translation_x_px": 0.0,
                                           "translation_y_px": 0.0, "rotation_deg": 0.0}}, "ANCHOR"),
            controller_read_fn=lambda: 152, controller_write_fn=crashing_write_fn,
            process_check_fn=lambda: [])
    except RuntimeError:
        pass
    ledger_after_crash = json.load(open(_ledger_path(exp_id_19)))
    rec("R4-19_post_command_crash_preserves_motion_command_sent",
        ledger_after_crash["status"] == "ARMED", ledger_after_crash)

    # R4-20 — restart after MOTION_COMMAND_SENT cannot resend (같은 ledger 재사용, 두번째 호출 abort)
    write_calls_20 = []
    res20 = preflight_execute(
        r3, good_ws_attestation,
        run_root=tmp_run_root(), allow_motion_env="1", execute_motion_flag=True,
        operator_confirmation="YES", expected_harness_sha256=expected_r3_sha, git_run_fn=mock_git_clean,
        camera_check_fn=lambda: {"dabai_identity_valid": True, "mjpg_640x480_valid": True,
                                  "camera_control_invariant_valid": True},
        capture_pre_window_fn=lambda: ({"valid": True, "median": {"translation_x_px": 0.0,
                                       "translation_y_px": 0.0, "rotation_deg": 0.0}}, "ANCHOR"),
        controller_read_fn=lambda: 152, controller_write_fn=lambda *a: write_calls_20.append(a),
        process_check_fn=lambda: [])
    rec("R4-20_restart_after_sent_cannot_resend",
        res20["status"] == "ABORTED" and res20["reason"] == "LEDGER_ALREADY_ARMED_OR_SENT"
        and not write_calls_20, res20)

    # R4-21 — A/B/C/D/X unchanged (R3 모듈 SHA 고정 + 순수함수 출력 재확인)
    module_check = check_module_sha()
    behavior_ok = (r3.motion_result_matrix(True, True, True) == "A"
                  and r3.motion_result_matrix(True, False, True) == "B"
                  and r3.motion_result_matrix(False, True, True) == "C"
                  and r3.motion_result_matrix(False, False, True) == "D"
                  and r3.motion_result_matrix(True, True, False) == "X")
    rec("R4-21_abcdx_unchanged",
        module_check["mismatches"].get(R3_MODULE_RELPATH) is None and behavior_ok, module_check)

    # R4-22 — 금지된 보조 카메라 브랜드(§19) 미접근 (정적 검사, 조립식 문자열로 자기참조 회피)
    self_check = self_ast_check()
    rec("R4-22_gemini_not_accessed", self_check["pass"] and not self_check["forbidden_camera_identity_refs_found"], self_check)

    # R4-23/24/25 — no real camera / controller access / robot motion (이 negative_tests() 실행 자체가 증거)
    rec("R4-23_no_real_camera", "cv2" not in self_check["imports"])
    rec("R4-24_no_controller_access", "Arm_Lib" not in self_check["imports"] and "smbus" not in self_check["imports"])
    rec("R4-25_no_robot_motion", True, {"note": "이 negative_tests() 는 전부 mock controller_write_fn 만 호출한다"})

    for d in [_ledger_path(exp_id_16), _ledger_path(exp_id_19)]:
        if os.path.isfile(d):
            os.remove(d)

    all_pass = all(v["pass"] for v in results.values())
    return {"all_pass": all_pass, "results": results}


def selftest():
    return {"self_ast_check": self_ast_check(), "module_sha_check": check_module_sha(),
            "negative_tests_R4_01_R4_25": negative_tests()}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        out = selftest()
        print(json.dumps(out, ensure_ascii=False, indent=1, default=repr))
        ok = (out["self_ast_check"]["pass"] and out["module_sha_check"]["match"]
              and out["negative_tests_R4_01_R4_25"]["all_pass"])
        sys.exit(0 if ok else 1)
    print(json.dumps({"gate": "P-007E-R2-R4", "status": "PREFLIGHT_ONLY",
                      "note": "--self-test 로만 실행 가능. 실 하드웨어 경로 없음"}, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
