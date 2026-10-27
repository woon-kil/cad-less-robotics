#!/usr/bin/env python3
"""P-007E preflight — 읽기 전용. Jetson 에서 `ssh ... python3 -` 로 돈다.

허용: Arm_serial_servo_read 만 (0x30+id, write_byte_data + read_word_data).
금지: 서보 쓰기 · torque · Button_Mode · RGB · reset · raw register.
"""
import ast, hashlib, json, os, subprocess, sys, time

DISK_SHA = "edfcae726658ce9340208de03d836aabfc5ebc1754d14ce8720a7ba36e13ed5c"
EGG_SHA = "3aba35832587428b83a231a94856fe89e36f93507c911fcf39c1b8ad27f13371"
EGG_PATH = "/usr/local/lib/python3.10/dist-packages/Arm_Lib-1.0.3-py3.10.egg"
SRC = "/home/jetson/software/Arm_Lib/Arm_Lib/Arm_Lib.py"
URDF = "/home/jetson/dofbot_pro_ws/src/dofbot_pro_description/urdf/DOFBOT_Pro-V24.urdf"
rec = {"gate": "P-007E", "phase": "PREFLIGHT", "guards": [], "aborted": None,
       "servo_writes": 0, "torque_changes": 0, "rgb_commands": 0}


def emit():
    print("P007E_JSON_BEGIN"); print(json.dumps(rec, ensure_ascii=False)); print("P007E_JSON_END")


def die(why):
    rec["aborted"] = why; emit(); sys.exit(1)


# 1. provenance
rec["disk_sha256"] = hashlib.sha256(open(SRC, "rb").read()).hexdigest()
if rec["disk_sha256"] != DISK_SHA:
    die("DISK_SHA_MISMATCH")
rec["urdf_sha256"] = hashlib.sha256(open(URDF, "rb").read()).hexdigest()
rec["guards"].append("Arm_Lib 디스크 sha256 일치")

# 2. 생성/소멸 부작용
tree = ast.parse(open(SRC, encoding="utf-8").read())
cls = [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == "Arm_Device"][0]
dunder = {f.name: f for f in cls.body if isinstance(f, ast.FunctionDef) and f.name.startswith("__")}
bad = [f"{n}:{x.func.attr}" for n, fn in dunder.items() for x in ast.walk(fn)
       if isinstance(x, ast.Call) and isinstance(x.func, ast.Attribute)
       and x.func.attr.startswith(("write", "read"))]
rec["dunder_methods"] = sorted(dunder); rec["dunder_bus_calls"] = bad
if bad:
    die("DUNDER_BUS_SIDE_EFFECT")
rec["guards"].append("__init__/__del__ 버스 부작용 0")


# 3. 배타적 제어 (자기 조상 제외)
def _anc():
    seen, pid = set(), os.getpid()
    while pid and pid not in seen:
        seen.add(pid)
        try:
            pid = int(open("/proc/%d/stat" % pid).read().rsplit(")", 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            break
    return seen


SELF = _anc()
ps = subprocess.run(["ps", "-eo", "pid,comm,args"], capture_output=True, text=True).stdout
FORB = ["YahboomArm", "joystick", "arm_driver", "move_group", "moveit", "start_app"]
hits = []
for line in ps.splitlines()[1:]:
    pa = line.split(None, 1)
    if not pa or not pa[0].isdigit() or int(pa[0]) in SELF:
        continue
    if any(f.lower() in line.lower() for f in FORB):
        hits.append(line.strip())
rec["exclusive_control_violations"] = hits
if hits:
    die("EXCLUSIVE_CONTROL_LOST")
rec["guards"].append("APP/joystick/arm_driver/MoveIt 미실행")

# 4. i2c-7 보유자
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
KNOWN = ("yahboom_oled.py", "kinemarics_dofbot")
unk = []
for pid in sorted(set(fds)):
    if int(pid) in SELF:
        continue
    try:
        cl = open("/proc/%s/cmdline" % pid).read().replace("\0", " ").strip()
    except OSError:
        cl = "<사라짐>"
    if not any(k in cl for k in KNOWN):
        unk.append({"pid": pid, "cmdline": cl})
rec["i2c7_holders"] = sorted(set(fds)); rec["unknown_i2c7_holders"] = unk
if unk:
    die("UNKNOWN_I2C7_HOLDER")
rec["guards"].append("i2c-7 보유자가 승인 집합뿐")

# 5. autostart 격리 유지
ad = os.path.expanduser("~/.config/autostart")
rec["autostart_entries"] = sorted(os.listdir(ad)) if os.path.isdir(ad) else []
rec["jupyterlab"] = subprocess.run(["systemctl", "is-active", "jupyterlab"],
                                   capture_output=True, text=True).stdout.strip()
rec["guards"].append("autostart·jupyterlab 상태 기록")

# 6. 런타임 egg
import Arm_Lib                                                   # noqa: E402
rec["arm_lib_file"] = Arm_Lib.__file__
if ".egg" not in Arm_Lib.__file__:
    die("RUNTIME_MODULE_NOT_EGG")
rec["egg_sha256"] = hashlib.sha256(open(EGG_PATH, "rb").read()).hexdigest()
if rec["egg_sha256"] != EGG_SHA:
    die("EGG_SHA_MISMATCH")
rec["guards"].append("런타임 egg sha256 일치")

# 7. baseline read — ID1~6 x5 라운드. 읽기 전용
arm = Arm_Lib.Arm_Device()
time.sleep(.1)
rounds = []
for r in range(5):
    row = {}
    for i in range(1, 7):
        t0 = time.perf_counter()
        v = arm.Arm_serial_servo_read(i)
        row["ID%d" % i] = {"deg": v, "elapsed_s": round(time.perf_counter() - t0, 6)}
        time.sleep(.01)
    row["at"] = time.strftime("%H:%M:%S")
    rounds.append(row)
    time.sleep(.2)
rec["baseline_rounds"] = rounds
per_id = {"ID%d" % i: [r["ID%d" % i]["deg"] for r in rounds] for i in range(1, 7)}
rec["baseline_per_id"] = per_id
rec["baseline_stable"] = {k: (len(set(v)) == 1) for k, v in per_id.items()}
rec["arm2_baseline_deg"] = per_id["ID2"][-1]
rec["read_calls"] = 30
emit()
