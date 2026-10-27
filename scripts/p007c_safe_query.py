#!/usr/bin/env python3
"""P-007C 안전 조회. Jetson에서 `ssh ... python3 -` 로 돈다.

allowlist 밖의 Arm_Lib 함수는 호출하지 않는다. 허용은 세 개뿐이다.
  Arm_serial_servo_read · Arm_get_hardversion · Arm_ping_servo

위치 명령 0, torque 변경 0, reset 0, calibration 0, offset 0, i2c probe 0.
조회 전후로 ID1~6 을 5라운드씩 읽어 state mutation 을 확인한다.

사전 검증 실패 시 조회하지 않는다.
  rc=2  Arm_Lib 해시 불일치 또는 생성자에 쓰기 호출
stdout: JSON 한 덩어리
"""
import ast, hashlib, json, sys, time

ARM_LIB = "/home/jetson/dofbot_pro/Arm_Lib/Arm_Lib/Arm_Lib.py"
EXPECTED_SHA = "edfcae726658ce9340208de03d836aabfc5ebc1754d14ce8720a7ba36e13ed5c"
ALLOWED = ("Arm_serial_servo_read", "Arm_get_hardversion", "Arm_ping_servo")
WRITE_TOKENS = ("write", "torque", "offset", "reset", "init_pose")


def die(rc, msg):
    sys.stderr.write("ABORT rc=%d %s\n" % (rc, msg))
    sys.exit(rc)


def constructor_calls(path):
    tree = ast.parse(open(path, encoding="utf-8", errors="replace").read())
    found = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "Arm_Device":
            for fn in node.body:
                if isinstance(fn, ast.FunctionDef) and fn.name in ("__init__", "__del__"):
                    calls = []
                    for sub in ast.walk(fn):
                        if isinstance(sub, ast.Call):
                            name = sub.func.attr if isinstance(sub.func, ast.Attribute) else (
                                sub.func.id if isinstance(sub.func, ast.Name) else "")
                            if any(t in name.lower() for t in WRITE_TOKENS):
                                calls.append(name)
                    found[fn.name] = calls
    return found


def read_rounds(arm, rounds=5):
    out = []
    for r in range(rounds):
        for sid in range(1, 7):
            t = time.time()
            err, value = "", None
            try:
                value = arm.Arm_serial_servo_read(sid)
            except Exception as e:  # noqa: BLE001
                err = "%s: %s" % (type(e).__name__, e)
            out.append({"round": r, "servo_id": sid, "deg": value, "error": err,
                        "at": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(t)),
                        "latency_s": round(time.time() - t, 6)})
    return out


def main():
    digest = hashlib.sha256(open(ARM_LIB, "rb").read()).hexdigest()
    if digest != EXPECTED_SHA:
        die(2, "Arm_Lib sha256 %s != %s" % (digest, EXPECTED_SHA))
    for fn, calls in constructor_calls(ARM_LIB).items():
        if calls:
            die(2, "%s calls write-like api %s" % (fn, calls))

    from Arm_Lib import Arm_Device
    arm = Arm_Device()

    result = {"gate": "P-007C", "allowlist": list(ALLOWED),
              "arm_lib_sha256": digest,
              "position_commands": 0, "torque_changes": 0, "resets": 0,
              "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")}

    result["pre_reads"] = read_rounds(arm)

    queries = []
    # 1) 보드 버전 — register 0x01
    t = time.time()
    rec = {"order": 1, "call": "Arm_get_hardversion()", "register": "0x01"}
    try:
        rec["value"] = arm.Arm_get_hardversion()
        rec["exception"] = None
    except Exception as e:  # noqa: BLE001
        rec["value"] = None
        rec["exception"] = "%s: %s" % (type(e).__name__, e)
    rec["elapsed_s"] = round(time.time() - t, 6)
    queries.append(rec)

    # 2) 서보 상태 — register 0x38, ID 1..6 (문서화된 범위만)
    for sid in range(1, 7):
        t = time.time()
        rec = {"order": 2, "call": "Arm_ping_servo(%d)" % sid, "register": "0x38", "servo_id": sid}
        try:
            v = arm.Arm_ping_servo(sid)
            rec["value"] = v
            rec["value_hex"] = ("0x%02x" % v) if isinstance(v, int) else None
            rec["interpretation"] = ("NORMAL_0xDA" if v == 0xDA else
                                     "NO_RESPONSE_0x00" if v == 0 else
                                     "NONE_AFTER_RETRIES" if v is None else "SERVO_ERROR_OTHER")
            rec["exception"] = None
        except Exception as e:  # noqa: BLE001
            rec["value"] = None
            rec["interpretation"] = "EXCEPTION"
            rec["exception"] = "%s: %s" % (type(e).__name__, e)
        rec["elapsed_s"] = round(time.time() - t, 6)
        queries.append(rec)

    result["queries"] = queries
    result["post_reads"] = read_rounds(arm)
    result["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    json.dump(result, sys.stdout, ensure_ascii=False, indent=1)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
