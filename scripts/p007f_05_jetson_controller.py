#!/usr/bin/env python3
"""P-007F-05 — Jetson 측 controller read/write. 이 파일만 실제 Arm_Lib 를 import 한다.

🔴 이 파일은 scripts/p007f_05_execution.py 의 self_ast_check(하드웨어 import 금지)
대상이 **아니다** — p007e_phase1_command.py/p007e_r2r5_hardware_wiring.py 선례와
동일하게, 실제 servo write 를 수행하는 최종 실행 파일만 Arm_Lib 를 직접 import
한다. 안전 로직(게이트/ledger/승인)은 전부 scripts/p007f_05_execution.py 에 있고,
이 파일은 오직 두 가지 모드만 제공한다: `--read N`(read-only, N회) 와
`--write`(유일하게 허용된 단 1회 write). 이 두 모드 외의 다른 Arm_Device 메서드는
전혀 호출하지 않는다(scripts/p007f_01_readonly_probe.py 와 동일한 FORBIDDEN_METHODS
원칙 — 여기서는 호출 자체를 소스에서 아예 두 함수로만 제한해 AST 검사로 증명한다).

🔴 §5 hardware-write reachability 요구사항을 만족하기 위해 `do_write()` 는 이
파일 전체에서 `Arm_serial_servo_write` 를 호출하는 **유일한 지점**이다. 호출은
정확히 1회, 인자는 하드코딩된 상수(SERVO_ID=2, COMMAND_TARGET_ID2=151,
COMMAND_DURATION_MS=2000)만 쓴다. 루프 없음, retry 없음, except/finally 안에서
호출 없음, reconnect 경로 없음.
"""
import argparse
import hashlib
import json
import sys
import time

# scripts/p007f_01_readonly_probe.py / p007e_r2r5_hardware_wiring.py 와 동일한
# Arm_Lib 무결성 고정값(2026-10-28 실측, 변경 없음을 재확인)
ARM_LIB_SRC = "/home/jetson/software/Arm_Lib/Arm_Lib/Arm_Lib.py"
ARM_LIB_DISK_SHA = "edfcae726658ce9340208de03d836aabfc5ebc1754d14ce8720a7ba36e13ed5c"
ARM_LIB_EGG = "/usr/local/lib/python3.10/dist-packages/Arm_Lib-1.0.3-py3.10.egg"
ARM_LIB_EGG_SHA = "3aba35832587428b83a231a94856fe89e36f93507c911fcf39c1b8ad27f13371"

SERVO_ID = 2
COMMAND_TARGET_ID2 = 151
COMMAND_DURATION_MS = 2000
READ_SPACING_S = 0.020


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


def do_read(rounds):
    """read-only. Arm_serial_servo_read(2) 를 rounds 회 호출한다. write 없음."""
    import Arm_Lib
    arm = Arm_Lib.Arm_Device()
    values = []
    for i in range(rounds):
        values.append(arm.Arm_serial_servo_read(SERVO_ID))
        if i < rounds - 1:
            time.sleep(READ_SPACING_S)
    return values


def do_write():
    """🔴 이 함수가 이 파일에서 Arm_serial_servo_write 를 호출하는 유일한 지점이다.
    정확히 1회만 호출한다 — 루프/retry/except 안 재호출 없음. 호출 자체에서 발생하는
    예외는 그대로 전파한다(가로채서 재시도하지 않는다)."""
    import Arm_Lib
    arm = Arm_Lib.Arm_Device()
    t0 = time.monotonic()
    arm.Arm_serial_servo_write(SERVO_ID, COMMAND_TARGET_ID2, COMMAND_DURATION_MS)
    elapsed = time.monotonic() - t0
    return {"servo_id": SERVO_ID, "target_degrees": COMMAND_TARGET_ID2,
            "duration_ms": COMMAND_DURATION_MS, "elapsed_s": round(elapsed, 6),
            "write_call_count": 1}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--read", type=int, metavar="N", help="read-only, N rounds, no write")
    g.add_argument("--write", action="store_true",
                   help="the ONE authorized write: Arm_serial_servo_write(2, 151, 2000)")
    ap.add_argument("--integrity-only", action="store_true",
                     help="print Arm_Lib integrity check and exit, no read/write")
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
    result = do_write()
    print(json.dumps({"mode": "write", "result": result, "integrity": integrity},
                     ensure_ascii=False))


if __name__ == "__main__":
    main()
