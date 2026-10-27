#!/usr/bin/env python3
"""P-007E phase 2 — settling 후 readback. 읽기 전용.

🔴 운영자 물리 관찰을 **먼저 확정한 뒤에만** 실행한다.
허용: Arm_serial_servo_read 만. 서보 쓰기 0.
"""
import hashlib, json, time

SRC = "/home/jetson/software/Arm_Lib/Arm_Lib/Arm_Lib.py"
rec = {"gate": "P-007E", "phase": "2-READBACK", "servo_writes": 0, "torque_changes": 0}
rec["disk_sha256"] = hashlib.sha256(open(SRC, "rb").read()).hexdigest()

import Arm_Lib                                                      # noqa: E402
arm = Arm_Lib.Arm_Device()
time.sleep(.1)
rounds = []
for _ in range(5):
    row = {"ID%d" % i: arm.Arm_serial_servo_read(i) for i in range(1, 7)}
    row["at"] = time.strftime("%H:%M:%S")
    rounds.append(row); time.sleep(.2)
rec["post_rounds"] = rounds
rec["post_per_id"] = {"ID%d" % i: [r["ID%d" % i] for r in rounds] for i in range(1, 7)}
rec["stable"] = {k: len(set(v)) == 1 for k, v in rec["post_per_id"].items()}
rec["read_calls"] = 30
print("P007E2_JSON_BEGIN"); print(json.dumps(rec, ensure_ascii=False)); print("P007E2_JSON_END")
