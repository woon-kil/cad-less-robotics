#!/usr/bin/env python3
"""P-007E phase 1 — 승인된 서보 명령 1회. **readback 하지 않는다.**

readback 을 여기서 하면 controller 값이 운영자 육안 판정을 오염시킨다.
읽기는 운영자 관찰을 받은 뒤 phase 2 에서 별도로 한다.

허용: Arm_serial_servo_read (명령 직전 baseline 재확인) + Arm_serial_servo_write 1회.
금지: 그 외 전부. retry 0 · 자동 return 0 · 두 번째 명령 0.
"""
import ast, contextlib, hashlib, io, json, os, subprocess, sys, time

DISK_SHA = "edfcae726658ce9340208de03d836aabfc5ebc1754d14ce8720a7ba36e13ed5c"
EGG_SHA = "3aba35832587428b83a231a94856fe89e36f93507c911fcf39c1b8ad27f13371"
EGG = "/usr/local/lib/python3.10/dist-packages/Arm_Lib-1.0.3-py3.10.egg"
SRC = "/home/jetson/software/Arm_Lib/Arm_Lib/Arm_Lib.py"

SERVO_ID = 2
EXPECTED_BASELINE = 150
DELTA = 5
TARGET = EXPECTED_BASELINE + DELTA        # 155
DURATION_MS = 2000

rec = {"gate": "P-007E", "phase": "1-COMMAND", "guards": [], "aborted": None,
       "bus_write_attempted": False, "servo_writes": 0,
       "torque_changes": 0, "rgb_commands": 0, "readback_in_this_phase": 0}


def emit():
    print("P007E1_JSON_BEGIN"); print(json.dumps(rec, ensure_ascii=False)); print("P007E1_JSON_END")


def die(w):
    rec["aborted"] = w; emit(); sys.exit(1)


rec["disk_sha256"] = hashlib.sha256(open(SRC, "rb").read()).hexdigest()
if rec["disk_sha256"] != DISK_SHA:
    die("DISK_SHA_MISMATCH")
tree = ast.parse(open(SRC, encoding="utf-8").read())
cls = [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == "Arm_Device"][0]
dd = {f.name: f for f in cls.body if isinstance(f, ast.FunctionDef) and f.name.startswith("__")}
bad = [f"{n}:{x.func.attr}" for n, fn in dd.items() for x in ast.walk(fn)
       if isinstance(x, ast.Call) and isinstance(x.func, ast.Attribute)
       and x.func.attr.startswith(("write", "read"))]
if bad:
    rec["dunder_bus_calls"] = bad; die("DUNDER_BUS_SIDE_EFFECT")
rec["guards"].append("sha256 + __init__/__del__ 부작용 0")


def _anc():
    s, p = set(), os.getpid()
    while p and p not in s:
        s.add(p)
        try:
            p = int(open("/proc/%d/stat" % p).read().rsplit(")", 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            break
    return s


SELF = _anc()
ps = subprocess.run(["ps", "-eo", "pid,comm,args"], capture_output=True, text=True).stdout
FORB = ["YahboomArm", "joystick", "arm_driver", "move_group", "moveit", "start_app"]
hits = [l.strip() for l in ps.splitlines()[1:]
        if l.split(None, 1) and l.split(None, 1)[0].isdigit()
        and int(l.split(None, 1)[0]) not in SELF
        and any(f.lower() in l.lower() for f in FORB)]
rec["exclusive_control_violations"] = hits
if hits:
    die("EXCLUSIVE_CONTROL_LOST")
rec["guards"].append("배타적 제어 유지")

import Arm_Lib                                                      # noqa: E402
rec["arm_lib_file"] = Arm_Lib.__file__
if ".egg" not in Arm_Lib.__file__:
    die("RUNTIME_MODULE_NOT_EGG")
rec["egg_sha256"] = hashlib.sha256(open(EGG, "rb").read()).hexdigest()
if rec["egg_sha256"] != EGG_SHA:
    die("EGG_SHA_MISMATCH")
rec["guards"].append("런타임 egg sha256 일치")

arm = Arm_Lib.Arm_Device()
time.sleep(.1)

# 명령 직전 baseline 재확인 — preflight 이후 상태가 바뀌었으면 중단
pre = [arm.Arm_serial_servo_read(SERVO_ID) for _ in range(3)]
rec["pre_write_read_id2"] = pre
rec["readback_in_this_phase"] = 3           # baseline 확인용. 명령 '후' 읽기는 하지 않는다
if len(set(pre)) != 1 or pre[0] != EXPECTED_BASELINE:
    die("BASELINE_CHANGED:%s" % pre)
rec["guards"].append("명령 직전 baseline == %d 재확인" % EXPECTED_BASELINE)

assert SERVO_ID == 2, "ID2 only"
assert TARGET == pre[0] + DELTA == 155, "target must be baseline+5"
assert 0 <= TARGET <= 180, "ID2 범위"
rec["guards"].append("id==2 · target==baseline+5==155 · 범위 검사")

rec["command"] = {"api": "Arm_serial_servo_write", "id": SERVO_ID,
                  "angle": TARGET, "time_ms": DURATION_MS,
                  "baseline_deg": pre[0], "delta_deg": DELTA}
rec["commanded_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")

buf = io.StringIO()
rec["bus_write_attempted"] = True
rec["servo_writes"] = 1                     # 시도 기준. 호출 '앞'에서 센다
t0 = time.perf_counter()
try:
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        arm.Arm_serial_servo_write(SERVO_ID, TARGET, DURATION_MS)
    rec["exception"] = None
except BaseException as e:                  # noqa: BLE001
    rec["exception"] = "%s: %s" % (type(e).__name__, e)
rec["elapsed_s"] = round(time.perf_counter() - t0, 6)

# 🔴 Arm_serial_servo_write 는 bare except 로 I2C 예외를 삼킨다. stdout 이 유일한 오류 신호다
rec["library_stdout"] = buf.getvalue()
rec["i2c_error_printed"] = "I2C error" in buf.getvalue()

rec["retry"] = 0
rec["automatic_return"] = False
rec["second_command"] = 0
rec["post_write_read_performed"] = False    # 🔴 phase 2 에서만 읽는다
emit()
