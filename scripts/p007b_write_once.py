#!/usr/bin/env python3
"""P-007B 단 1회 서보 쓰기. Jetson에서 `ssh ... python3 - <target> <dur> <expect_base>` 로 돈다.

이 스크립트는 ID2에 Arm_serial_servo_write 를 **정확히 한 번** 호출한다.
재시도하지 않는다. 예외가 나도 다시 쓰지 않는다 — 전송 여부를 알 수 없기 때문이다.

사전 검증에 실패하면 쓰지 않고 종료한다.
  rc=2  Arm_Lib 해시 불일치
  rc=3  쓰기 직전 ID2 readback이 기대값과 다름
  rc=4  인자 검증 실패 (target != base+2 등)
stdout: `#META|{json}` 한 줄 + post-command 읽기 CSV
"""
import ast, contextlib, csv, hashlib, io, json, sys, time

ARM_LIB = "/home/jetson/dofbot_pro/Arm_Lib/Arm_Lib/Arm_Lib.py"
EXPECTED_SHA = "edfcae726658ce9340208de03d836aabfc5ebc1754d14ce8720a7ba36e13ed5c"
SERVO_ID = 2
WRITE_TOKENS = ("write", "torque", "offset", "reset", "init_pose")


def die(rc, msg):
    sys.stderr.write("ABORT rc=%d %s\n" % (rc, msg))
    sys.exit(rc)


def constructor_calls(path):
    """Arm_Device.__init__ / __del__ 이 쓰기 API를 부르지 않는지 AST로 확인한다."""
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


def main():
    target, duration, expect_base = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])

    if target != expect_base + 2:
        die(4, "target %d != base %d + 2" % (target, expect_base))
    if not (0 <= target <= 180):
        die(4, "target %d out of 0..180" % target)
    if duration <= 0 or duration > 65535:
        die(4, "duration %d out of 1..65535" % duration)

    digest = hashlib.sha256(open(ARM_LIB, "rb").read()).hexdigest()
    if digest != EXPECTED_SHA:
        die(2, "Arm_Lib sha256 %s != %s" % (digest, EXPECTED_SHA))
    for fn, calls in constructor_calls(ARM_LIB).items():
        if calls:
            die(2, "%s calls write-like api %s" % (fn, calls))

    from Arm_Lib import Arm_Device
    arm = Arm_Device()

    # 쓰기 직전 마지막 확인. 기대값과 다르면 쓰지 않는다. target을 다시 계산하지 않는다.
    pre = arm.Arm_serial_servo_read(SERVO_ID)
    if pre != expect_base:
        die(3, "pre-write ID2 readback %r != expected %d" % (pre, expect_base))

    assert SERVO_ID == 2, "이 스크립트는 ID2 전용이다"
    meta = {"servo_id": SERVO_ID, "target_library_deg": target, "duration_ms": duration,
            "pre_write_read": pre, "api": "Arm_Device.Arm_serial_servo_write",
            "call_count": 0, "retried": False}

    buf = io.StringIO()
    t0 = time.time()
    try:
        with contextlib.redirect_stdout(buf):
            arm.Arm_serial_servo_write(SERVO_ID, target, duration)   # ← 단 1회
        meta["call_count"] = 1
        meta["exception"] = None
    except Exception as e:  # noqa: BLE001
        meta["call_count"] = 1          # 전송됐을 수 있다. 호출은 1회로 센다
        meta["exception"] = "%s: %s" % (type(e).__name__, e)
    meta["elapsed_s"] = round(time.time() - t0, 6)
    meta["library_stdout"] = buf.getvalue().strip()
    meta["i2c_error_printed"] = "I2C error" in meta["library_stdout"]
    meta["commanded_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(t0))

    time.sleep(duration / 1000.0 + 1.5)   # 보간 시간 + 여유

    out = csv.writer(sys.stdout)
    print("#META|" + json.dumps(meta, ensure_ascii=False))
    out.writerow(["wall_timestamp", "monotonic_unix", "round_index", "servo_id",
                  "library_reported_degree", "error", "latency_s"])
    errors = 0
    for r in range(5):
        for sid in range(1, 7):
            t = time.time()
            err, value = "", None
            try:
                value = arm.Arm_serial_servo_read(sid)
            except Exception as e:  # noqa: BLE001
                err = "%s: %s" % (type(e).__name__, e)
                errors += 1
            stamp = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(t))
            stamp += ".%03d" % int((t % 1) * 1000) + time.strftime("%z", time.localtime(t))
            out.writerow([stamp[:-2] + ":" + stamp[-2:], "%.6f" % t, r, sid,
                          "" if value is None else value, err, "%.6f" % (time.time() - t)])
    sys.stderr.write("post_read_rounds=5 errors=%d\n" % errors)


if __name__ == "__main__":
    main()
