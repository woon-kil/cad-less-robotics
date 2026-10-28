#!/usr/bin/env python3
"""P-007E-R2-R5 — 실제 실행 CLI. Jetson 의 git-tracked checkout 안에서만 실행한다.

사용법:
  python3 scripts/p007e_r2r5_run.py --check-only
      opt-in 0. Arm_Lib 무결성/DaBai 식별/git tree/module SHA 만 확인하고 끝낸다.
      서보 read 도 하지 않는다(Arm_Device() 자체를 생성하지 않는다).

  P007E_R2_R5_ALLOW_MOTION=1 python3 scripts/p007e_r2r5_run.py --execute
      §31 의 전체 흐름(운영자 확인 -> workspace 체크리스트 -> PRE 캡처 -> ... ->
      승인 문구 입력 화면)을 실제로 돈다. 승인 문구가 정확히 일치하지 않으면
      실제 write 는 호출되지 않는다(scripts/p007e_r2r5_execution.py 의 설계).
"""
import argparse
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

# 🔴 scripts/p007e_r2r5_execution.py 의 SHA 를 **고정 리터럴**로 핀한다 — 런타임에
# 그 파일을 다시 읽어 계산한 값을 "expected" 로 되돌려주면(self-referential) 변조를
# 전혀 잡지 못하는 타당성 없는 검사가 된다(self-discovered — R3/R4 가 서로의 SHA를
# 상수로 고정하는 것과 같은 이유). 이 값이 바뀌면(execution.py 를 고쳤으면) 이 상수도
# 함께 갱신해야 한다 — 그 갱신 자체가 git diff 로 드러난다.
R5_EXEC_SHA = "2fc8f6390bb366518df4da838060aa7b5db11b5db08ddcb7ff62ee1be2fafae9"

WORKSPACE_SAFETY_ITEMS = (
    ("workspace_clear", "workspace clear"),
    ("no_human_body_in_workspace", "no human body in robot workspace"),
    ("no_loose_object", "no loose object within 15 mm conservative envelope"),
    ("cables_clear", "cables clear"),
    ("dabai_cable_has_slack", "DaBai cable slack sufficient"),
    ("camera_mount_secure", "camera mount secure"),
    ("robot_base_stable", "robot base stable"),
    ("power_cutoff_reachable", "physical power cutoff reachable"),
)


def ask_yes_no(prompt):
    v = input(prompt + " [yes/no]: ").strip().lower()
    return v == "yes"


def do_check_only():
    import p007e_r2r5_execution as r5
    import p007e_r2r5_hardware_wiring as hw

    out = {"gate": "P-007E-R2-R5", "mode": "CHECK_ONLY"}
    out["self_ast_check_execution"] = r5.self_ast_check()
    out["module_sha_check"] = r5.check_module_sha()
    out["hardware_wiring_selftest"] = hw.selftest()
    try:
        out["arm_lib_integrity"] = hw.check_arm_lib_integrity()
    except Exception as e:  # noqa: BLE001
        out["arm_lib_integrity"] = {"pass": False, "error": repr(e)}
    try:
        out["camera_identity_check"] = hw.camera_identity_check()
    except Exception as e:  # noqa: BLE001
        out["camera_identity_check"] = {"pass": False, "error": repr(e)}
    out["i2c7_holder_diagnostic"] = hw.i2c7_holder_diagnostic()

    import importlib
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    evp = importlib.import_module("p007e_r2r2_evidence_pipeline")
    try:
        out["git_dirty_tree_check"] = evp.check_dirty_tree(ROOT, require_clean=True, run_fn=None)
    except Exception as e:  # noqa: BLE001
        out["git_dirty_tree_check"] = {"error": repr(e)}

    ok = (out["self_ast_check_execution"]["pass"] and out["module_sha_check"]["match"]
          and out["hardware_wiring_selftest"]["negative_tests_HW_01_HW_04"]["all_pass"]
          and out.get("arm_lib_integrity", {}).get("pass")
          and out.get("camera_identity_check", {}).get("dabai_identity_valid")
          and out.get("camera_identity_check", {}).get("mjpg_640x480_valid")
          and not out.get("git_dirty_tree_check", {}).get("dirty", True))
    out["overall_check_only_pass"] = bool(ok)
    print(json.dumps(out, ensure_ascii=False, indent=1, default=repr))
    return 0 if ok else 1


def do_execute(run_root):
    import p007e_r2r5_execution as r5
    import p007e_r2r5_hardware_wiring as hw

    allow_env = os.environ.get("P007E_R2_R5_ALLOW_MOTION")
    if allow_env != "1":
        print(json.dumps({"status": "ABORTED", "reason": "NO_OPT_IN_ENV"}))
        return 1

    print("=" * 70)
    print("P-007E-R2-R5 — SINGLE CONTROLLED PHYSICAL-MOTION EXECUTION")
    print("=" * 70)
    print("Arm2 ID2: expected %d -> target %d (+%d deg, %d ms, <=1 command)" % (
        r5.EXPECTED_BASELINE_ID2, r5.COMMAND_TARGET_ID2,
        r5.COMMAND_TARGET_ID2 - r5.EXPECTED_BASELINE_ID2, r5.COMMAND_DURATION_MS))
    print()

    if not ask_yes_no("Are you physically watching the robot right now (OPERATOR_VISUAL_READY)?"):
        print(json.dumps({"status": "ABORTED", "reason": "OPERATOR_NOT_WATCHING"}))
        return 1
    operator_confirmation = "YES"

    print()
    print("Workspace safety checklist — answer for each item:")
    attestation = {}
    for key, label in WORKSPACE_SAFETY_ITEMS:
        attestation[key] = ask_yes_no("  " + label)
    failed = [label for key, label in WORKSPACE_SAFETY_ITEMS if not attestation[key]]
    if failed:
        print(json.dumps({"status": "ABORTED", "reason": "WORKSPACE_NOT_SAFE", "failed_items": failed}))
        return 1

    print()
    print("Loading Arm_Lib and verifying integrity...")
    integrity = hw.check_arm_lib_integrity()
    print(json.dumps(integrity, ensure_ascii=False, indent=1))
    if not integrity["pass"]:
        print(json.dumps({"status": "ABORT_BEFORE_MOTION", "reason": "ARM_LIB_INTEGRITY_FAILED"}))
        return 1

    import Arm_Lib
    arm = Arm_Lib.Arm_Device()
    time.sleep(0.1)

    callables = hw.build_real_callables(run_root, arm)

    def operator_observation_fn():
        v = input("Did you observe physical motion of Arm2? [YES/NO/UNCERTAIN]: ").strip().upper()
        return v if v in ("YES", "NO", "UNCERTAIN") else "UNCERTAIN"

    result = r5.execute_r5(
        run_root=run_root, allow_motion_env="1", execute_motion_flag=True,
        operator_confirmation=operator_confirmation,
        operator_observation_fn=operator_observation_fn,
        workspace_attestation=attestation,
        expected_harness_sha256=R5_EXEC_SHA,
        expected_r3_harness_sha256=r5.R3_MODULE_SHA,
        git_run_fn=None, repo_root=ROOT,
        camera_check_fn=callables["camera_check_fn"],
        capture_pre_window_fn=callables["capture_pre_window_fn"],
        capture_post_window_fn=callables["capture_post_window_fn"],
        controller_read_fn=callables["controller_read_fn"],
        controller_write_fn=callables["controller_write_fn"],
        process_check_fn=callables["process_check_fn"])

    out_path = os.path.join(run_root, "r5_final_result.json")
    os.makedirs(run_root, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=1, default=repr)
    print(json.dumps(result, ensure_ascii=False, indent=1, default=repr))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check-only", action="store_true")
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--run-root", default=None)
    args = ap.parse_args()

    if args.check_only:
        sys.exit(do_check_only())
    if args.execute:
        run_root = args.run_root or os.path.join(
            ROOT, "evidence", "P-007E-R2-R5", "run_%s" % time.strftime("%Y%m%dT%H%M%S"))
        sys.exit(do_execute(run_root))
    print("specify --check-only or --execute")
    sys.exit(2)


if __name__ == "__main__":
    main()
