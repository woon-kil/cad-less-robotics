#!/usr/bin/env python3
"""P-007F-01 — Read-only Arm_Lib/controller diagnostic probe. Jetson 전용.

목적: R5에서 관측된 "write 1회 -> readback 152->154 -> 운영자 관찰상 물리 동작 없음"
현상의 원인을, **추가 motion 명령 없이** 얼마나 좁힐 수 있는지 조사한다.

🔴 이 파일은 다음 외의 Arm_Device 메서드를 호출하지 않는다(self_ast_check 로
   AST 수준에서 구조적으로 보장):
     Arm_serial_servo_read, Arm_serial_servo_read_any, Arm_ping_servo,
     Arm_get_hardversion, get_i2c_bus_num
   actuator 상태를 바꿀 수 있는 모든 메서드(Arm_serial_servo_write*,
   Arm_serial_set_torque, Arm_serial_set_id, Arm_reset, Arm_PWM_servo_write,
   Arm_Clear_Action, Arm_Action_Study, Arm_Action_Mode, Arm_RGB_set,
   Arm_Product_Select, Arm_Button_Mode, Arm_Buzzer_On/Off,
   bus_servo_control, bus_servo_control_array6,
   Arm_serial_servo_write_offset_switch, Arm_serial_servo_write_offset_state)
   는 self_ast_check() 가 FORBIDDEN_METHODS 로 걸러 자기 자신을 즉시 실패시킨다.

🔴 Arm_serial_servo_write_offset_state() 와 Arm_Read_Action_Num() 은 source상
   read 처럼 보이지만(값을 반환) 각각 offset-switch 캘리브레이션/액션그룹
   서브시스템에 묶인 trigger-write(0x1b, 0x22)를 동반한다 — "side effect 가능성이
   조금이라도 있으면 실행하지 않는다"는 지시에 따라 **호출하지 않는다**
   (evidence/P-007F-01/arm_lib_source_forensics.md 에 사유 기록, Task E/A 참조).
"""
import ast
import contextlib
import hashlib
import io
import json
import os
import sys
import time

DISK_SHA = "edfcae726658ce9340208de03d836aabfc5ebc1754d14ce8720a7ba36e13ed5c"
EGG_SHA = "3aba35832587428b83a231a94856fe89e36f93507c911fcf39c1b8ad27f13371"
EGG = "/usr/local/lib/python3.10/dist-packages/Arm_Lib-1.0.3-py3.10.egg"
SRC = "/home/jetson/software/Arm_Lib/Arm_Lib/Arm_Lib.py"

READ_SPACING_S = 0.020
READ_ROUNDS = 5

ALLOWED_METHODS = {
    "Arm_serial_servo_read", "Arm_serial_servo_read_any", "Arm_ping_servo",
    "Arm_get_hardversion", "get_i2c_bus_num", "Arm_Device",
}
FORBIDDEN_METHODS = {
    "Arm_serial_servo_write", "Arm_serial_servo_write6", "Arm_serial_servo_write6_array",
    "Arm_serial_servo_write_any", "Arm_PWM_servo_write", "Arm_serial_set_torque",
    "Arm_serial_set_id", "Arm_reset", "Arm_Clear_Action", "Arm_Action_Study",
    "Arm_Action_Mode", "Arm_RGB_set", "Arm_Product_Select", "Arm_Button_Mode",
    "Arm_Buzzer_On", "Arm_Buzzer_Off", "bus_servo_control", "bus_servo_control_array6",
    "Arm_serial_servo_write_offset_switch", "Arm_serial_servo_write_offset_state",
    "Arm_Read_Action_Num",
}


def sha256(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def _scan_referenced_attrs(src):
    """순수 함수 — 임의의 python 소스 텍스트에서 참조되는 모든 attribute 이름을
    모은다. self_ast_check() 와 negative test(PF01-10, 금지 호출이 실제로 섞인
    합성 소스에 대해 탐지가 작동하는지 — true positive 증명) 양쪽이 공유한다."""
    tree = ast.parse(src)
    referenced_attrs = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute):
            referenced_attrs.add(node.attr)
    return referenced_attrs


def _classify_referenced_attrs(referenced_attrs):
    forbidden_found = sorted(referenced_attrs & FORBIDDEN_METHODS)
    not_allowlisted = sorted(a for a in referenced_attrs
                             if a in FORBIDDEN_METHODS or (a.startswith("Arm_") and a not in ALLOWED_METHODS))
    return {
        "referenced_attrs": sorted(referenced_attrs),
        "forbidden_methods_found": forbidden_found,
        "not_allowlisted_arm_calls": not_allowlisted,
        "pass": not forbidden_found and not not_allowlisted,
    }


def self_ast_check():
    """이 파일 자신이 FORBIDDEN_METHODS 를 호출/참조하지 않는지, 그리고 실제로
    쓰는 Arm_* 메서드 이름이 ALLOWED_METHODS 의 부분집합인지 AST 로 확인한다.

    🔴 self-discovered 버그(이 함수 자신의 1차 초안) — 직접 호출 패턴
    (`x.method(...)` == `ast.Call(func=ast.Attribute(...))`)만 보면, 이 파일이
    실제로 쓰는 `call_capturing(arm.Arm_serial_servo_read, sid)` 같은
    "메서드를 변수/인자로 참조해 나중에 호출" 패턴을 놓친다 — `arm.Arm_serial_servo_read`
    는 그 자체로는 `ast.Call` 이 아니라 인자 위치의 `ast.Attribute` 라서 탐지되지
    않았다(1차 self-test 에서 referenced_attrs 에 Arm_serial_servo_read/read_any/
    ping_servo/get_hardversion 가 전혀 안 잡혀 발견). **모든** `ast.Attribute`
    노드(직접 호출이든 참조든)를 스캔하도록 수정했다 — 금지 메서드를 "호출하지
    않고 참조만 하는" 우회도 이 검사로 잡힌다. PF01-10 이 이 탐지력 자체를
    (합성 금지호출 소스에 대해) true positive 로 증명한다 — "금지 호출이 없다"는
    결과만으로는 탐지기가 애초에 고장나 아무것도 못 보는 경우와 구별되지 않는다."""
    src = open(os.path.abspath(__file__), "r", encoding="utf-8").read()
    return _classify_referenced_attrs(_scan_referenced_attrs(src))


def negative_tests():
    results = {}

    def rec(name, cond, detail=None):
        results[name] = {"pass": bool(cond), "detail": detail}

    chk = self_ast_check()
    rec("PF01-01_forbidden_motion_apis_zero_calls",
        "Arm_serial_servo_write" not in chk["referenced_attrs"]
        and "Arm_serial_servo_write6" not in chk["referenced_attrs"]
        and "Arm_serial_servo_write6_array" not in chk["referenced_attrs"]
        and "Arm_serial_servo_write_any" not in chk["referenced_attrs"], chk)
    rec("PF01-02_torque_write_zero_calls", "Arm_serial_set_torque" not in chk["referenced_attrs"])
    rec("PF01-03_reset_zero_calls", "Arm_reset" not in chk["referenced_attrs"])
    rec("PF01-04_calibration_write_zero_calls",
        "Arm_serial_servo_write_offset_switch" not in chk["referenced_attrs"]
        and "Arm_serial_servo_write_offset_state" not in chk["referenced_attrs"])
    rec("PF01-05_action_playback_zero_calls",
        "Arm_Action_Study" not in chk["referenced_attrs"] and "Arm_Action_Mode" not in chk["referenced_attrs"]
        and "Arm_Clear_Action" not in chk["referenced_attrs"])
    rec("PF01-06_self_ast_check_pass", chk["pass"], chk)
    rec("PF01-07_bus_servo_control_zero_calls",
        "bus_servo_control" not in chk["referenced_attrs"]
        and "bus_servo_control_array6" not in chk["referenced_attrs"])
    rec("PF01-08_pwm_write_zero_calls", "Arm_PWM_servo_write" not in chk["referenced_attrs"])
    rec("PF01-09_set_id_zero_calls", "Arm_serial_set_id" not in chk["referenced_attrs"])

    # PF01-10 — true positive 증명: 탐지기가 "금지 호출이 섞인 소스"를 실제로
    # FAIL 로 잡아내는지, 두 가지 우회 패턴(직접 호출 / 메서드-참조-후-호출) 둘 다로 확인
    synthetic_direct_call = "arm.Arm_serial_servo_write(2, 154, 2000)\n"
    synthetic_reference_only = "fn = arm.Arm_serial_set_torque\nfn(1)\n"
    direct_result = _classify_referenced_attrs(_scan_referenced_attrs(synthetic_direct_call))
    reference_result = _classify_referenced_attrs(_scan_referenced_attrs(synthetic_reference_only))
    rec("PF01-10_detector_true_positive_on_synthetic_forbidden_calls",
        not direct_result["pass"] and "Arm_serial_servo_write" in direct_result["forbidden_methods_found"]
        and not reference_result["pass"]
        and "Arm_serial_set_torque" in reference_result["forbidden_methods_found"],
        {"direct": direct_result, "reference_only": reference_result})

    all_pass = all(v["pass"] for v in results.values())
    return {"all_pass": all_pass, "results": results}


def run_probe():
    out = {"gate": "P-007F-01", "stage": "read-only runtime probe",
          "servo_writes": 0, "torque_writes": 0, "reset_calls": 0, "calibration_writes": 0,
          "action_playback_calls": 0}
    out["self_ast_check"] = self_ast_check()
    if not out["self_ast_check"]["pass"]:
        out["aborted"] = "SELF_AST_CHECK_FAILED"
        return out

    out["arm_lib_disk_sha256"] = sha256(SRC)
    out["arm_lib_disk_sha_match"] = (out["arm_lib_disk_sha256"] == DISK_SHA)
    if not out["arm_lib_disk_sha_match"]:
        out["aborted"] = "DISK_SHA_MISMATCH"
        return out
    out["arm_lib_egg_sha256"] = sha256(EGG)
    out["arm_lib_egg_sha_match"] = (out["arm_lib_egg_sha256"] == EGG_SHA)
    if not out["arm_lib_egg_sha_match"]:
        out["aborted"] = "EGG_SHA_MISMATCH"
        return out

    import Arm_Lib  # noqa: E402
    out["arm_lib_runtime_file"] = Arm_Lib.__file__
    arm = Arm_Lib.Arm_Device()
    time.sleep(0.1)

    def call_capturing(fn, *a):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
            v = fn(*a)
        return v, buf.getvalue(), ("I2C error" in buf.getvalue())

    # --- Arm_serial_servo_read(1..6), READ_ROUNDS times, spaced ---
    rounds = []
    for r in range(READ_ROUNDS):
        row = {"round": r + 1, "at": time.strftime("%H:%M:%S"), "reads": []}
        for sid in range(1, 7):
            v, stdout_text, err = call_capturing(arm.Arm_serial_servo_read, sid)
            row["reads"].append({"id": sid, "value": v, "is_none": v is None, "i2c_error_printed": err})
            time.sleep(READ_SPACING_S)
        rounds.append(row)
    out["servo_read_rounds"] = rounds
    out["servo_reads_total"] = READ_ROUNDS * 6

    # --- Arm_serial_servo_read_any(2) — different register path (0x37 vs 0x32) ---
    read_any_rounds = []
    for r in range(READ_ROUNDS):
        v, stdout_text, err = call_capturing(arm.Arm_serial_servo_read_any, 2)
        read_any_rounds.append({"round": r + 1, "value": v, "is_none": v is None,
                                "i2c_error_printed": err})
        time.sleep(READ_SPACING_S)
    out["servo_read_any_id2_rounds"] = read_any_rounds

    # --- Arm_ping_servo(1..6) — status/communication check, register 0x38 ---
    ping_results = {}
    for sid in range(1, 7):
        v, stdout_text, err = call_capturing(arm.Arm_ping_servo, sid)
        ping_results[str(sid)] = {"value": v, "value_hex": (hex(v) if isinstance(v, int) else None),
                                  "is_0xda": (v == 0xda), "i2c_error_printed": err}
        time.sleep(READ_SPACING_S)
    out["ping_servo_results"] = ping_results

    # --- Arm_get_hardversion() ---
    v, stdout_text, err = call_capturing(arm.Arm_get_hardversion)
    out["hardversion"] = {"value": v, "i2c_error_printed": err}

    out["i2c_bus_num"] = arm.get_i2c_bus_num()
    out["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    return out


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--self-test":
        nt = negative_tests()
        print(json.dumps({"negative_tests_PF01_01_09": nt}, ensure_ascii=False, indent=1))
        sys.exit(0 if nt["all_pass"] else 1)
    out = run_probe()
    print(json.dumps(out, ensure_ascii=False, indent=1, default=repr))


if __name__ == "__main__":
    main()
