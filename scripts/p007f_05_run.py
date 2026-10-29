#!/usr/bin/env python3
"""P-007F-05 — CLI entrypoint. scripts/p007f_05_hardware_wiring.py 의 real
callables 를 scripts/p007f_05_execution.py.execute_p007f05() 에 연결한다.

🔴 이 스크립트를 실행한다고 로봇이 움직이지 않는다 — execute_p007f05() 자체가
`--allow-motion` 플래그 **AND** `P007F05_ALLOW_MOTION=1` 환경변수를 **둘 다**
요구하고(§0 opt-in 2중 게이트, R3/R5 선례 재사용), 그 이후에도 PRE hard gate
전체가 PASS해야 승인 화면이 뜨며, 승인 화면에서 정확한 문구를 타이핑해야만
실제 write 가 일어난다. §34 hardware-free preparation 단계에서는 이 스크립트를
`--dry-run-prep`으로만 호출한다(아래) — 이 모드는 ARM 상태까지 가지 않고
하드웨어 identity/controller read-only/캡처만 수행한다.
"""
import argparse
import importlib.util
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--allow-motion", action="store_true", help="§0 opt-in flag (REQUIRED for real motion)")
    ap.add_argument("--dry-run-prep", action="store_true",
                    help="§34 전용 — gemini identity + baseline B + controller PRE read만 실측, write 없음")
    ap.add_argument("--run-root", default=None)
    args = ap.parse_args()

    execution = importlib.import_module("p007f_05_execution")
    wiring = importlib.import_module("p007f_05_hardware_wiring")

    sync_rc = wiring.sync_jetson_controller_script()
    if sync_rc[0] != 0:
        print(json.dumps({"status": "ABORT_BEFORE_MOTION", "reason": "JETSON_CONTROLLER_SYNC_FAILED",
                          "detail": sync_rc}, ensure_ascii=False))
        sys.exit(1)

    if args.dry_run_prep:
        import time as _time
        wiring.start_gemini_driver()
        _time.sleep(4.0)  # driver 가 serial/firmware 를 로그에 쓸 때까지 settle
        identity = wiring.gemini_identity_check()
        pre_value = wiring.controller_read_once()
        wiring.stop_gemini_driver()
        print(json.dumps({"mode": "dry_run_prep", "gemini_identity": identity,
                          "controller_pre_read": pre_value, "write_invoked": False},
                         ensure_ascii=False, indent=1))
        return

    allow_motion_env = os.environ.get("P007F05_ALLOW_MOTION")
    run_root = args.run_root or os.path.join(ROOT, "evidence", "P-007F-05",
                                             "run_%d" % __import__("time").time())
    callables = wiring.build_real_callables(run_root)

    expected_harness_sha = execution.sha256(os.path.abspath(
        os.path.join(ROOT, "scripts", "p007f_05_execution.py")))
    preregistration_sha = execution.sha256(os.path.join(
        ROOT, "evidence", "P-007F-04", "preregistration.json"))

    result = execution.execute_p007f05(
        run_root=run_root, allow_motion_env=allow_motion_env, execute_motion_flag=args.allow_motion,
        operator_confirmation=input("Are you physically present and watching the robot? YES/NO\n> "),
        workspace_attestation=None,  # §9 운영자 attestation은 run_root 준비 단계에서 별도 수집
        expected_harness_sha256=expected_harness_sha, preregistration_sha=preregistration_sha,
        **callables)
    os.makedirs(run_root, exist_ok=True)
    with open(os.path.join(run_root, "execution_result.json"), "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=1, default=repr)
    print(json.dumps(result, ensure_ascii=False, indent=1, default=repr))


if __name__ == "__main__":
    main()
