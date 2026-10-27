#!/usr/bin/env python3
"""P-007E-R1 — Timing-Controlled Servo Command Experiment.

P-007B 와 **단 하나의 변수만** 다르다: 마지막 baseline read 완료 ~ servo write 시작
사이의 explicit quiet interval (>= 50 ms).

  Joint=Arm2/ID2 · Baseline=150 · Target=152 · Delta=+2 · Duration=2000ms
  API=Arm_serial_servo_write · Command=exactly one · Auto return=none

🔴 기본값은 **NO WRITE** 다. 쓰기는 explicit opt-in 없이는 불가능하다.
   환경변수 P007E_R1_ALLOW_SERVO_WRITE=1 이어야만 쓰기 경로에 도달한다.

모드
  (기본)        dry-run — baseline read 까지만. servo write 0
  --guard-test  하드웨어 없이 timing guard 음성 시험. Arm_Lib import 안 함
"""
import json, os, sys, time

SERVO_ID = 2
EXPECTED_BASELINE = [86, 150, 0, 18, 93, 178]
EXPECTED_ID2 = 150
DELTA = 2
TARGET = EXPECTED_ID2 + DELTA                 # 152
DURATION_MS = 2000
READ_SPACING_S = 0.020                        # >= 20 ms (ced8f52: <10ms 는 ~50% 실패)
QUIET_INTERVAL_MIN_MS = 50.0
QUIET_SLEEP_S = 0.060                         # 여유를 둔 목표. 실측은 따로 기록한다

DISK_SHA = "edfcae726658ce9340208de03d836aabfc5ebc1754d14ce8720a7ba36e13ed5c"
EGG_SHA = "3aba35832587428b83a231a94856fe89e36f93507c911fcf39c1b8ad27f13371"
EGG = "/usr/local/lib/python3.10/dist-packages/Arm_Lib-1.0.3-py3.10.egg"
SRC = "/home/jetson/software/Arm_Lib/Arm_Lib/Arm_Lib.py"
URDF = "/home/jetson/dofbot_pro_ws/src/dofbot_pro_description/urdf/DOFBOT_Pro-V24.urdf"


def quiet_interval_ms(t_last_read_end_ns, t_write_start_ns):
    """🔴 순수 함수. 하드웨어 없이 시험 가능하도록 분리했다."""
    return (t_write_start_ns - t_last_read_end_ns) / 1_000_000.0


def timing_guard_ok(q_ms):
    return q_ms >= QUIET_INTERVAL_MIN_MS


# ─── guard 음성 시험: 하드웨어 0, Arm_Lib import 0 ─────────────────────
if "--guard-test" in sys.argv:
    r = {"gate": "P-007E-R1", "mode": "GUARD-NEGATIVE-TEST",
         "servo_writes": 0, "bus_write_attempted": False, "cases": []}
    for label, gap_ns in [("0ms", 0), ("10ms", 10_000_000), ("49.9ms", 49_900_000),
                          ("50.0ms", 50_000_000), ("60ms", 60_000_000)]:
        base = 1_000_000_000
        q = quiet_interval_ms(base, base + gap_ns)
        ok = timing_guard_ok(q)
        r["cases"].append({"label": label, "quiet_interval_ms": round(q, 4),
                           "guard_ok": ok,
                           "action": "WRITE PATH REACHABLE" if ok else "HARNESS_TIMING_ABORT"})
    r["aborts"] = sum(1 for c in r["cases"] if not c["guard_ok"])
    r["passes"] = sum(1 for c in r["cases"] if c["guard_ok"])
    r["expected"] = "0/10/49.9ms → ABORT (3건) · 50.0/60ms → 통과 (2건)"
    r["verdict"] = "PASS" if (r["aborts"] == 3 and r["passes"] == 2) else "FAIL"
    print("P007E_R1_JSON_BEGIN"); print(json.dumps(r, ensure_ascii=False)); print("P007E_R1_JSON_END")
    sys.exit(0 if r["verdict"] == "PASS" else 1)

# ─── 실행 모드 (Jetson) ────────────────────────────────────────────────
import ast, contextlib, hashlib, io, subprocess                      # noqa: E402

ALLOW_WRITE = os.environ.get("P007E_R1_ALLOW_SERVO_WRITE", "0") == "1"
rec = {"gate": "P-007E-R1", "mode": "EXECUTE" if ALLOW_WRITE else "DRY-RUN (기본값)",
       "allow_servo_write_env": ALLOW_WRITE, "guards": [], "aborted": None,
       "servo_writes": 0, "motion_capable_commands": 0, "bus_write_attempted": False,
       "torque_writes": 0, "rgb_writes": 0, "raw_i2c_writes": 0}


def emit():
    print("P007E_R1_JSON_BEGIN"); print(json.dumps(rec, ensure_ascii=False)); print("P007E_R1_JSON_END")


def die(w):
    rec["aborted"] = w; emit(); sys.exit(1)


rec["arm_lib_disk_sha256"] = hashlib.sha256(open(SRC, "rb").read()).hexdigest()
if rec["arm_lib_disk_sha256"] != DISK_SHA:
    die("DISK_SHA_MISMATCH")
rec["urdf_sha256"] = hashlib.sha256(open(URDF, "rb").read()).hexdigest()

tree = ast.parse(open(SRC, encoding="utf-8").read())
cls = [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == "Arm_Device"][0]
dd = {f.name: f for f in cls.body if isinstance(f, ast.FunctionDef) and f.name.startswith("__")}
bad = [f"{n}:{x.func.attr}" for n, fn in dd.items() for x in ast.walk(fn)
       if isinstance(x, ast.Call) and isinstance(x.func, ast.Attribute)
       and x.func.attr.startswith(("write", "read"))]
rec["dunder_methods"] = sorted(dd); rec["dunder_bus_calls"] = bad
if bad:
    die("DUNDER_BUS_SIDE_EFFECT")
rec["guards"].append("disk sha256 + __init__/__del__ 버스 부작용 0")


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
ad = os.path.expanduser("~/.config/autostart")
rec["autostart_entries"] = sorted(os.listdir(ad)) if os.path.isdir(ad) else []
rec["jupyterlab"] = subprocess.run(["systemctl", "is-active", "jupyterlab"],
                                   capture_output=True, text=True).stdout.strip()
rec["guards"].append("배타적 제어 · autostart · jupyterlab 확인")

import Arm_Lib                                                       # noqa: E402
rec["arm_lib_runtime_file"] = Arm_Lib.__file__
if ".egg" not in Arm_Lib.__file__:
    die("RUNTIME_MODULE_NOT_EGG")
rec["arm_lib_runtime_sha256"] = hashlib.sha256(open(EGG, "rb").read()).hexdigest()
if rec["arm_lib_runtime_sha256"] != EGG_SHA:
    die("EGG_SHA_MISMATCH")
rec["guards"].append("런타임 egg sha256 일치")

arm = Arm_Lib.Arm_Device()
time.sleep(.1)

# ─── baseline: ID1~6, 각 read 사이 >= 20 ms ───────────────────────────
reads, t_last_end = [], None
for i in range(1, 7):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        v = arm.Arm_serial_servo_read(i)
    t_last_end = time.monotonic_ns()           # 🔴 이 read 가 끝난 시각
    reads.append({"id": i, "value": v, "is_none": v is None,
                  "i2c_error_printed": "I2C error" in buf.getvalue()})
    if i < 6:
        time.sleep(READ_SPACING_S)
rec["baseline_reads"] = reads
rec["baseline_values"] = [r["value"] for r in reads]
rec["baseline_read_spacing_ms"] = READ_SPACING_S * 1000
rec["read_calls"] = 6

# 🔴 ID3 의 정상값 0 을 None 과 혼동하지 않는다 (is None 으로만 판정)
id3 = reads[2]
rec["id3_note"] = {"value": id3["value"], "is_none": id3["is_none"],
                   "expected_normal_zero": 0,
                   "distinction": "라이브러리는 raw word==0 일 때만 None 을 낸다. ID3 의 0 은 "
                                  "raw 900 이 0° 로 변환된 **정상값**이다. `is None` 으로만 판정한다"}

rec["expected_baseline"] = EXPECTED_BASELINE
rec["baseline_matches_expected"] = rec["baseline_values"] == EXPECTED_BASELINE
id2 = reads[1]["value"]
rec["id2_baseline"] = id2
if id2 is None or id2 != EXPECTED_ID2:
    die("PRE_COMMAND_BASELINE_ABORT:ID2=%r" % id2)
if not rec["baseline_matches_expected"]:
    die("PRE_COMMAND_BASELINE_ABORT:%r" % rec["baseline_values"])
rec["guards"].append("baseline == %r · ID2 == %d" % (EXPECTED_BASELINE, EXPECTED_ID2))

rec["target"] = TARGET; rec["delta"] = DELTA; rec["duration_ms"] = DURATION_MS
assert SERVO_ID == 2 and TARGET == id2 + DELTA == 152

# ─── quiet interval: 이 구간에 controller/I2C 트랜잭션 0 ──────────────
rec["quiet_interval_spec"] = {
    "definition": "마지막 baseline I2C 트랜잭션 완료 ~ servo write 시작",
    "clock": "time.monotonic_ns()", "threshold_ms": QUIET_INTERVAL_MIN_MS,
    "target_sleep_s": QUIET_SLEEP_S,
    "forbidden_in_window": ["servo read", "다른 ID read", "ping", "hardversion",
                            "RGB", "torque query/write", "Button_Mode", "raw SMBus"],
    "allowed_in_window": ["monotonic clock", "로컬 변수 검증"]}
time.sleep(QUIET_SLEEP_S)                      # ← 이 사이에 버스 접근 없음
t_write_start = time.monotonic_ns()
q_ms = quiet_interval_ms(t_last_end, t_write_start)
rec["t_last_read_end_ns"] = t_last_end
rec["t_write_start_ns"] = t_write_start
rec["quiet_interval_ms"] = round(q_ms, 4)

# ─── HARD GUARD: 미달이면 write 함수에 **도달하지 않는다** ────────────
if not timing_guard_ok(q_ms):
    die("HARNESS_TIMING_ABORT:quiet_interval_ms=%.4f < %.1f" % (q_ms, QUIET_INTERVAL_MIN_MS))
rec["guards"].append("HARD GUARD 통과 — quiet_interval %.3f ms >= %.1f ms" % (q_ms, QUIET_INTERVAL_MIN_MS))

if not ALLOW_WRITE:
    rec["dry_run_stop"] = ("🔴 DRY-RUN — servo write 경로에 진입하지 않았다. "
                           "P007E_R1_ALLOW_SERVO_WRITE=1 이어야 쓰기가 가능하다")
    emit(); sys.exit(0)

# ─── Phase B 전용. 승인 없이 도달하지 않는다 ─────────────────────────
rec["command"] = {"api": "Arm_serial_servo_write", "id": SERVO_ID,
                  "angle": TARGET, "time_ms": DURATION_MS, "baseline": id2}
rec["commanded_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
buf = io.StringIO()
rec["bus_write_attempted"] = True
rec["servo_writes"] = 1; rec["motion_capable_commands"] = 1
t0 = time.perf_counter()
try:
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        arm.Arm_serial_servo_write(SERVO_ID, TARGET, DURATION_MS)
    rec["exception"] = None
except BaseException as e:                      # noqa: BLE001
    rec["exception"] = "%s: %s" % (type(e).__name__, e)
rec["elapsed_s"] = round(time.perf_counter() - t0, 6)
rec["library_stdout"] = buf.getvalue()
rec["i2c_error_printed"] = "I2C error" in buf.getvalue()
rec["retry"] = 0; rec["automatic_return"] = False; rec["second_command"] = 0
rec["post_write_read_performed"] = False        # 🔴 물리 관찰 먼저. readback 은 별도 단계
emit()
