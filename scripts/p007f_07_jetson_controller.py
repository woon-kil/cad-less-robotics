#!/usr/bin/env python3
"""P-007F-07 Phase B (future) — Jetson 측 controller read/write skeleton.

🔴 Phase A에서는 이 파일의 `do_write()` 가 **절대 호출되지 않는다** -- 이 파일
자체가 import 되거나 `--read` 모드로 실행되는 것은 허용되지만(read-only),
`--write` 인자를 주더라도 Phase A 코드 경로 어디에서도 이 모드로 진입하지
않는다(§1/§19 — Phase B 실제 target/duration은 이 gate에서 확정하지 않음).

§23 one-write architecture: 이 파일 전체에서 `Arm_serial_servo_write` 를
호출하는 지점은 `do_write()` 단 한 곳이며, 루프/재시도 없이 정확히 1회만
실행되도록 작성한다(인자는 호출자가 Phase B preregistration에서 넘기는
servo_id/target_degrees/duration_ms — 이번 gate에서는 숫자를 고정하지 않는다,
§19). `scripts/p007f_07_hardware_free_check.py` 가 AST로 call-site 개수/위치를
검증한다.
"""
import argparse
import hashlib
import json
import sys
import time

ARM_LIB_SRC = "/home/jetson/software/Arm_Lib/Arm_Lib/Arm_Lib.py"
ARM_LIB_DISK_SHA = "edfcae726658ce9340208de03d836aabfc5ebc1754d14ce8720a7ba36e13ed5c"
ARM_LIB_EGG = "/usr/local/lib/python3.10/dist-packages/Arm_Lib-1.0.3-py3.10.egg"
ARM_LIB_EGG_SHA = "3aba35832587428b83a231a94856fe89e36f93507c911fcf39c1b8ad27f13371"

READ_SPACING_S = 0.020
DEFAULT_SERVO_ID = 2


def _sha256(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def check_arm_lib_integrity():
    import Arm_Lib
    actual_disk = _sha256(ARM_LIB_SRC)
    actual_egg = _sha256(ARM_LIB_EGG)
    runtime_is_egg = ".egg" in Arm_Lib.__file__
    return {"disk_sha_match": actual_disk == ARM_LIB_DISK_SHA, "disk_sha_actual": actual_disk,
            "egg_sha_match": actual_egg == ARM_LIB_EGG_SHA, "egg_sha_actual": actual_egg,
            "runtime_is_egg": runtime_is_egg, "runtime_file": Arm_Lib.__file__,
            "pass": (actual_disk == ARM_LIB_DISK_SHA and actual_egg == ARM_LIB_EGG_SHA
                     and runtime_is_egg)}


def do_read(rounds, servo_id=DEFAULT_SERVO_ID):
    import Arm_Lib
    arm = Arm_Lib.Arm_Device()
    values = []
    for i in range(rounds):
        values.append(arm.Arm_serial_servo_read(servo_id))
        if i < rounds - 1:
            time.sleep(READ_SPACING_S)
    return values


def do_write(servo_id, target_degrees, duration_ms,
            command_prepare_timestamp_ns=None):
    """🔴 이 함수가 이 파일 전체에서 `Arm_serial_servo_write` 를 호출하는
    유일한 지점이다. 정확히 1회만 호출한다 -- 루프/retry/except 안 재호출
    없음. §10 요구대로 호출 직전/직후 monotonic timestamp 를 기록한다."""
    import Arm_Lib
    arm = Arm_Lib.Arm_Device()
    command_call_enter_timestamp_ns = time.monotonic_ns()
    arm.Arm_serial_servo_write(servo_id, target_degrees, duration_ms)
    command_call_exit_timestamp_ns = time.monotonic_ns()
    return {"servo_id": servo_id, "target_degrees": target_degrees, "duration_ms": duration_ms,
            "command_prepare_timestamp_ns": command_prepare_timestamp_ns,
            "command_call_enter_timestamp_ns": command_call_enter_timestamp_ns,
            "command_call_exit_timestamp_ns": command_call_exit_timestamp_ns,
            "write_call_count": 1}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--read", type=int, metavar="N")
    g.add_argument("--write", nargs=3, type=float, metavar=("SERVO_ID", "TARGET_DEG", "DURATION_MS"),
                   help="Phase B ONLY -- not reachable from any Phase A code path")
    ap.add_argument("--integrity-only", action="store_true")
    args = ap.parse_args()

    integrity = check_arm_lib_integrity()
    if args.integrity_only:
        print(json.dumps({"integrity": integrity}, ensure_ascii=False))
        sys.exit(0 if integrity["pass"] else 1)
    if not integrity["pass"]:
        print(json.dumps({"status": "ABORT_ARM_LIB_INTEGRITY_MISMATCH", "integrity": integrity},
                         ensure_ascii=False))
        sys.exit(1)

    if args.read is not None:
        values = do_read(args.read)
        print(json.dumps({"mode": "read", "rounds": args.read, "values": values,
                          "integrity": integrity}, ensure_ascii=False))
        return
    servo_id, target_deg, duration_ms = args.write
    result = do_write(int(servo_id), target_deg, duration_ms)
    print(json.dumps({"mode": "write", "result": result, "integrity": integrity},
                     ensure_ascii=False))


if __name__ == "__main__":
    main()
