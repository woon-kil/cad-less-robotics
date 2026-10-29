#!/usr/bin/env python3
"""P-007F-06 — §28 Hardware-free verification. scripts/p007f_06_forensics.py 가
Arm_Lib/서보 write/torque/카메라 capture API 를 전혀 참조하지 않음을 AST로
증명한다.

🔴 forbidden-pattern-literal-self-matches-its-own-guard 교훈 재적용 -- 이 검사
대상 파일(p007f_06_forensics.py)의 docstring/주석에 "Arm_Lib" 같은 금지어가
설명문으로 등장하므로, substring 스캔이 아니라 AST Import/Call 노드만 검사한다.
"""
import ast
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TARGET = os.path.join(ROOT, "scripts", "p007f_06_forensics.py")

FORBIDDEN_IMPORTS = {"Arm_Lib", "smbus", "smbus2", "rclpy", "rospy", "serial", "Adafruit_GPIO"}
FORBIDDEN_CALL_NAMES = {"Arm_serial_servo_write", "Arm_serial_set_torque", "Arm_PWM_servo_write",
                        "Arm_reset", "Arm_Clear_Action", "VideoCapture"}


def check(path):
    src = open(path, encoding="utf-8").read()
    tree = ast.parse(src)
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                imported.add(a.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    bad_imports = sorted(imported & FORBIDDEN_IMPORTS)

    bad_calls = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            func = node.func
            name = None
            if isinstance(func, ast.Attribute):
                name = func.attr
            elif isinstance(func, ast.Name):
                name = func.id
            if name in FORBIDDEN_CALL_NAMES:
                bad_calls.append({"name": name, "line": node.lineno})

    return {"imports": sorted(imported), "forbidden_imports_found": bad_imports,
           "forbidden_calls_found": bad_calls,
           "pass": not bad_imports and not bad_calls}


def main():
    result = check(TARGET)
    result["robot_command_count"] = 0 if result["pass"] else None
    import json
    print(json.dumps(result, ensure_ascii=False, indent=1))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
