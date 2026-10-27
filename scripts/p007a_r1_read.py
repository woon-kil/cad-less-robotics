#!/usr/bin/env python3
# P-007A-R1 컨트롤러 읽기. Jetson에서 `ssh jetson@HOST python3 - <rounds> < 이 파일`로 돈다.
# Arm_serial_servo_read 외의 API를 부르지 않는다. 위치·토크·offset·reset 쓰기 0.
# 사전 검증에 실패하면 읽지 않고 rc=2로 끝낸다.
import ast, csv, hashlib, io, os, sys, time

ARM_LIB = "/home/jetson/dofbot_pro/Arm_Lib/Arm_Lib/Arm_Lib.py"
EXPECTED_SHA = "edfcae726658ce9340208de03d836aabfc5ebc1754d14ce8720a7ba36e13ed5c"  # P-007A source_integrity.json
WRITE_TOKENS = ("write", "torque", "offset", "reset", "init_pose")


def fail(msg):
    sys.stderr.write("PREFLIGHT_FAIL %s\n" % msg)
    sys.exit(2)


def constructor_is_read_only(path):
    """Arm_Device.__init__과 __del__이 쓰기 호출을 하지 않는지 AST로 확인한다."""
    tree = ast.parse(open(path, encoding="utf-8", errors="replace").read())
    found = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "Arm_Device":
            for fn in node.body:
                if isinstance(fn, ast.FunctionDef) and fn.name in ("__init__", "__del__"):
                    calls = []
                    for sub in ast.walk(fn):
                        if isinstance(sub, ast.Call):
                            name = ""
                            if isinstance(sub.func, ast.Attribute):
                                name = sub.func.attr
                            elif isinstance(sub.func, ast.Name):
                                name = sub.func.id
                            if any(t in name.lower() for t in WRITE_TOKENS):
                                calls.append(name)
                    found[fn.name] = calls
    return found


def main():
    rounds = int(sys.argv[1]) if len(sys.argv) > 1 else 100

    digest = hashlib.sha256(open(ARM_LIB, "rb").read()).hexdigest()
    if digest != EXPECTED_SHA:
        fail("Arm_Lib sha256 %s != expected %s" % (digest, EXPECTED_SHA))

    ctor = constructor_is_read_only(ARM_LIB)
    if "__init__" not in ctor:
        fail("Arm_Device.__init__ not found")
    for fn, calls in ctor.items():
        if calls:
            fail("%s calls write-like api %s" % (fn, calls))

    from Arm_Lib import Arm_Device

    arm = Arm_Device()
    out = csv.writer(sys.stdout)
    out.writerow(["wall_timestamp", "monotonic_unix", "round_index", "servo_id",
                  "library_reported_degree", "error", "latency_s"])
    errors = 0
    for r in range(rounds):
        for sid in range(1, 7):
            t0 = time.time()
            err, value = "", None
            try:
                value = arm.Arm_serial_servo_read(sid)
            except Exception as e:  # noqa: BLE001
                err = "%s: %s" % (type(e).__name__, e)
                errors += 1
            dt = time.time() - t0
            stamp = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(t0))
            stamp += ".%03d" % int((t0 % 1) * 1000) + time.strftime("%z", time.localtime(t0))
            stamp = stamp[:-2] + ":" + stamp[-2:]
            out.writerow([stamp, "%.6f" % t0, r, sid, "" if value is None else value, err, "%.6f" % dt])
    sys.stderr.write("rounds=%d errors=%d\n" % (rounds, errors))
    sys.exit(1 if errors else 0)


if __name__ == "__main__":
    main()
