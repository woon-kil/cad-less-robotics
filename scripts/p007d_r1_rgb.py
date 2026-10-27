#!/usr/bin/env python3
"""P-007D-R1 — RGB block-write 제어 실험. Jetson 에서 `ssh ... python3 -` 로 돈다.

허용: Arm_RGB_set 정확히 1회. 그 외 Arm_Lib 함수 호출 0.
금지: servo write / torque / Button_Mode / reset / buzzer / raw register.

🔴 Arm_RGB_set 은 본문에 bare `except:` 가 있어 I2C 예외를 삼키고
   'Arm_RGB_set I2C error' 만 출력한다. 따라서 예외 유무가 아니라
   **stdout 캡처**로 I2C 오류를 판별해야 한다. 그러지 않으면 실험 오류(C)가
   진단 신호(B)로 잘못 분류된다.
"""
import ast, contextlib, hashlib, io, json, os, subprocess, sys, time

DISK_SHA = "edfcae726658ce9340208de03d836aabfc5ebc1754d14ce8720a7ba36e13ed5c"
EGG_SHA  = "3aba35832587428b83a231a94856fe89e36f93507c911fcf39c1b8ad27f13371"
EGG_PATH = "/usr/local/lib/python3.10/dist-packages/Arm_Lib-1.0.3-py3.10.egg"
SRC = "/home/jetson/software/Arm_Lib/Arm_Lib/Arm_Lib.py"
RGB = (0, 0, 255)                       # 플랜에서 사전 고정. 채널 매핑은 미검증
rec = {"gate": "P-007D-R1", "guards": [], "aborted": None, "bus_write_attempted": False}


def emit():
    print("P007D_R1_JSON_BEGIN"); print(json.dumps(rec, ensure_ascii=False)); print("P007D_R1_JSON_END")


def die(why):
    rec["aborted"] = why
    emit()
    sys.exit(1)


# ── 가드 1: 디스크 소스 무결성 (정적 분석 대상과 동일한가) ─────────────
rec["disk_sha256"] = hashlib.sha256(open(SRC, "rb").read()).hexdigest()
if rec["disk_sha256"] != DISK_SHA:
    die("DISK_SHA_MISMATCH")
rec["guards"].append("디스크 소스 sha256 일치")

# ── 가드 2: 생성/소멸 부작용 AST 검사 ──────────────────────────────────
tree = ast.parse(open(SRC, encoding="utf-8").read())
cls = [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == "Arm_Device"][0]
dunder = {f.name: f for f in cls.body if isinstance(f, ast.FunctionDef) and f.name.startswith("__")}
rec["dunder_methods"] = sorted(dunder)
bad = [f"{n}:{x.func.attr}" for n, fn in dunder.items() for x in ast.walk(fn)
       if isinstance(x, ast.Call) and isinstance(x.func, ast.Attribute)
       and x.func.attr.startswith(("write", "read"))]
rec["dunder_bus_calls"] = bad
if bad:
    die("DUNDER_BUS_SIDE_EFFECT")
rec["guards"].append("__init__/__del__ 버스 부작용 0")


# ── 가드 3: 배타적 제어 (자기 조상 체인 제외) ─────────────────────────
def _ancestors():
    seen, pid = set(), os.getpid()
    while pid and pid not in seen:
        seen.add(pid)
        try:
            pid = int(open("/proc/%d/stat" % pid).read().rsplit(")", 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            break
    return seen


SELF = _ancestors()
rec["self_pid_chain"] = sorted(SELF)
ps = subprocess.run(["ps", "-eo", "pid,comm,args"], capture_output=True, text=True).stdout
FORBIDDEN = ["YahboomArm", "joystick", "arm_driver", "move_group", "moveit", "start_app"]
hits = []
for line in ps.splitlines()[1:]:
    parts = line.split(None, 1)
    if not parts or not parts[0].isdigit() or int(parts[0]) in SELF:
        continue
    if any(f.lower() in line.lower() for f in FORBIDDEN):
        hits.append(line.strip())
rec["exclusive_control_violations"] = hits
if hits:
    die("EXCLUSIVE_CONTROL_LOST")
rec["guards"].append("APP/joystick/arm_driver/MoveIt 미실행")

# ── 가드 4: /dev/i2c-7 보유자가 승인 집합뿐인가 ───────────────────────
fds = []
for pid in os.listdir("/proc"):
    if not pid.isdigit():
        continue
    try:
        for fd in os.listdir("/proc/%s/fd" % pid):
            if "i2c-7" in os.readlink("/proc/%s/fd/%s" % (pid, fd)):
                fds.append(pid)
    except OSError:
        pass
rec["other_i2c7_holders"] = sorted(set(fds))
KNOWN = ("yahboom_oled.py", "kinemarics_dofbot")
unknown = []
for pid in sorted(set(fds)):
    if int(pid) in SELF:
        continue
    try:
        cl = open("/proc/%s/cmdline" % pid).read().replace("\0", " ").strip()
    except OSError:
        cl = "<사라짐>"
    if not any(k in cl for k in KNOWN):
        unknown.append({"pid": pid, "cmdline": cl})
rec["unknown_i2c7_holders"] = unknown
if unknown:
    die("UNKNOWN_I2C7_HOLDER")
rec["guards"].append("i2c-7 보유자가 P-007A-R1 승인 집합(OLED 0x3C · ros2 래퍼)뿐")

# ── 가드 5: 🔴 실제로 import 되는 것은 egg 다 (P-007C-R1 발견) ────────
# import 자체는 버스 부작용이 없다 — 패키지 __init__.py 가 한 줄짜리 re-export 다.
import Arm_Lib                                                      # noqa: E402

rec["arm_lib_file"] = Arm_Lib.__file__
if ".egg" not in Arm_Lib.__file__:
    die("RUNTIME_MODULE_NOT_EGG:%s" % Arm_Lib.__file__)
rec["egg_sha256"] = hashlib.sha256(open(EGG_PATH, "rb").read()).hexdigest()
if rec["egg_sha256"] != EGG_SHA:
    die("EGG_SHA_MISMATCH")
rec["guards"].append("런타임 모듈이 egg 이고 egg sha256 이 P-007C-R1 기록과 일치")

# ── 실행: Arm_RGB_set 1회 ─────────────────────────────────────────────
arm = Arm_Lib.Arm_Device()
time.sleep(.1)                                      # 공식 예제와 동일
rec["constructor"] = "Arm_Device() + sleep(.1)"
rec["rgb_value"] = {"R": RGB[0], "G": RGB[1], "B": RGB[2], "intended": "BLUE (채널 매핑 미검증)"}
rec["commanded_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")

buf = io.StringIO()
rec["bus_write_attempted"] = True                   # 🔴 시도 횟수. 호출 '앞'에서 센다
rec["command_count"] = {"Arm_RGB_set": 1}
t0 = time.perf_counter()
try:
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        arm.Arm_RGB_set(*RGB)
    rec["exception"] = None
except BaseException as e:                          # noqa: BLE001
    rec["exception"] = "%s: %s" % (type(e).__name__, e)
rec["elapsed_s"] = round(time.perf_counter() - t0, 6)

# 🔴 라이브러리가 예외를 삼키므로 stdout 이 유일한 I2C 오류 신호다
rec["library_stdout"] = buf.getvalue()
rec["i2c_error_printed"] = "I2C error" in buf.getvalue()

rec["servo_commands"] = 0
rec["torque_commands"] = 0
rec["other_armlib_calls"] = 0
rec["recovery_command_sent"] = False                # 플랜: OFF 복구 금지
emit()
