#!/usr/bin/env python3
"""P-007E-R1 — read-only servo readback (pre/post snapshot).

🔴 읽기 전용이다. Arm_serial_servo_read 외의 호출은 없다 (AST 로 확인한다).
   기존 scripts/p007e_phase2_readback.py 는 한 라운드 안에서 ID1~6 을
   **간격 없이** 읽는다. ced8f52 실측에서 0~8ms 간격은 None 이 ~50% 였다.
   그래서 이 스크립트는 **모든 read 사이에 >= 20 ms** 를 둔다.

사용: ssh jetson@... 'python3 - <label> [rounds]' < scripts/p007e_r1_postread.py
"""
import contextlib, hashlib, io, json, sys, time

READ_SPACING_S = 0.020
DISK_SHA = "edfcae726658ce9340208de03d836aabfc5ebc1754d14ce8720a7ba36e13ed5c"
EGG_SHA = "3aba35832587428b83a231a94856fe89e36f93507c911fcf39c1b8ad27f13371"
EGG = "/usr/local/lib/python3.10/dist-packages/Arm_Lib-1.0.3-py3.10.egg"
SRC = "/home/jetson/software/Arm_Lib/Arm_Lib/Arm_Lib.py"
BASELINE = [86, 150, 0, 18, 93, 178]

label = sys.argv[1] if len(sys.argv) > 1 else "unlabeled"
rounds_n = int(sys.argv[2]) if len(sys.argv) > 2 else 5

rec = {"gate": "P-007E-R1", "phase": "READ-ONLY SNAPSHOT", "label": label,
       "servo_writes": 0, "motion_capable_commands": 0, "bus_write_attempted": False,
       "torque_writes": 0, "rgb_writes": 0, "raw_i2c_writes": 0,
       "read_spacing_ms": READ_SPACING_S * 1000, "rounds_requested": rounds_n,
       "aborted": None}


def emit():
    print("P007E_R1_READ_JSON_BEGIN"); print(json.dumps(rec, ensure_ascii=False))
    print("P007E_R1_READ_JSON_END")


def die(w):
    rec["aborted"] = w; emit(); sys.exit(1)


rec["arm_lib_disk_sha256"] = hashlib.sha256(open(SRC, "rb").read()).hexdigest()
if rec["arm_lib_disk_sha256"] != DISK_SHA:
    die("DISK_SHA_MISMATCH")

import Arm_Lib                                                       # noqa: E402
rec["arm_lib_runtime_file"] = Arm_Lib.__file__
rec["arm_lib_runtime_sha256"] = hashlib.sha256(open(EGG, "rb").read()).hexdigest()
if rec["arm_lib_runtime_sha256"] != EGG_SHA:
    die("EGG_SHA_MISMATCH")

arm = Arm_Lib.Arm_Device()
time.sleep(.1)

rounds, read_calls, i2c_errors = [], 0, 0
rec["started_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
for r in range(rounds_n):
    row = {"round": r + 1, "at": time.strftime("%H:%M:%S"), "reads": []}
    for i in range(1, 7):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            v = arm.Arm_serial_servo_read(i)
        read_calls += 1
        err = "I2C error" in buf.getvalue()
        i2c_errors += int(err)
        row["reads"].append({"id": i, "value": v, "is_none": v is None,
                             "i2c_error_printed": err})
        time.sleep(READ_SPACING_S)             # 🔴 모든 read 사이 >= 20 ms
    row["values"] = [x["value"] for x in row["reads"]]
    rounds.append(row)
rec["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
rec["rounds"] = rounds
rec["read_calls"] = read_calls
rec["i2c_error_count"] = i2c_errors
rec["per_id"] = {"ID%d" % i: [row["reads"][i - 1]["value"] for row in rounds]
                 for i in range(1, 7)}
rec["none_count_per_id"] = {k: sum(1 for v in vs if v is None)
                            for k, vs in rec["per_id"].items()}
rec["stable_per_id"] = {k: len(set(vs)) == 1 for k, vs in rec["per_id"].items()}

id2 = [v for v in rec["per_id"]["ID2"] if v is not None]
rec["id2_non_none"] = id2
rec["id2_all_none"] = len(id2) == 0
rec["id2_changed_from_150"] = (None if not id2 else any(v != 150 for v in id2))
rec["id2_equals_152"] = (None if not id2 else all(v == 152 for v in id2))
rec["expected_baseline"] = BASELINE
rec["matches_baseline_all_rounds"] = all(row["values"] == BASELINE for row in rounds)
emit()
