#!/usr/bin/env python3
"""P-007F-05 — §5/§34 hardware-write reachability static/AST check.

확인 대상 둘:
  1. scripts/p007f_05_execution.py — Arm_Lib/smbus/smbus2/rclpy/rospy/cv2/
     Adafruit_GPIO/직접 subprocess 를 전혀 import 하지 않는다(이미 자체
     self_ast_check() 가 있지만, 이 파일은 독립적으로 재확인한다 — P-007F-03/04
     선례와 동일 원칙, 단일 소스만 신뢰하지 않는다).
  2. scripts/p007f_05_jetson_controller.py — Arm_serial_servo_write 호출이
     정확히 1곳(ast.Call 노드 기준)이고, 그 인자가 리터럴 상수
     SERVO_ID(=2)/COMMAND_TARGET_ID2(=151)/COMMAND_DURATION_MS(=2000) 뿐이며,
     for/while 루프 안에 있지 않고, except/finally 블록 안에 있지 않다.

🔴 2026-10-29 교훈 재적용(forbidden-pattern-literal-self-matches-its-own-guard,
   반복 패턴) — 이 파일 자신의 docstring/주석/코드 리터럴이 검사 대상 키워드를
   그대로 담고 있으므로, substring 스캔이 아니라 AST 노드 구조로만 판정한다.
   또한 이 파일을 대상으로 자체 검사를 돌릴 때는(true-positive 증명용) 합성
   코드 문자열을 별도로 만들어 비교하지, 이 파일 자신의 소스를 "금지 패턴을
   포함하는 악성 샘플"로 스캔하지 않는다.
"""
import ast
import hashlib
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXECUTION_PATH = os.path.join(ROOT, "scripts", "p007f_05_execution.py")
JETSON_CONTROLLER_PATH = os.path.join(ROOT, "scripts", "p007f_05_jetson_controller.py")

FORBIDDEN_HARDWARE_IMPORTS = {"Arm_Lib", "smbus", "smbus2", "rclpy", "rospy", "cv2", "Adafruit_GPIO"}
EXPECTED_SERVO_ID = 2
EXPECTED_TARGET = 151
EXPECTED_DURATION_MS = 2000


def sha256(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def check_execution_no_hardware_imports(src):
    tree = ast.parse(src)
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                imported.add(a.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    bad = sorted(imported & FORBIDDEN_HARDWARE_IMPORTS)
    direct_subprocess = "subprocess" in imported
    return {"imports": sorted(imported), "forbidden_found": bad,
            "direct_subprocess_import": direct_subprocess,
            "pass": not bad and not direct_subprocess}


def _resolve_const_value(node, module_consts):
    """ast.Constant(숫자) 또는 모듈 전역에 바인딩된 ast.Name 상수를 resolve."""
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.Name) and node.id in module_consts:
        return module_consts[node.id]
    return "UNRESOLVED:%r" % ast.dump(node)


def _module_level_int_consts(tree):
    consts = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 \
                and isinstance(node.targets[0], ast.Name) and isinstance(node.value, ast.Constant):
            consts[node.targets[0].id] = node.value.value
    return consts


def _in_loop_or_exception_handler(tree, target_node):
    """target_node 가 For/While/ExceptHandler/Try(finalbody) 안에 있는지 확인."""
    parents = {}

    def walk(node, parent=None):
        parents[node] = parent
        for child in ast.iter_child_nodes(node):
            walk(child, node)
    walk(tree)
    node = target_node
    in_loop, in_except, in_finally = False, False, False
    while node is not None:
        p = parents.get(node)
        if isinstance(p, (ast.For, ast.While)):
            in_loop = True
        if isinstance(p, ast.ExceptHandler):
            in_except = True
        if isinstance(p, ast.Try) and node in p.finalbody:
            in_finally = True
        node = p
    return {"in_loop": in_loop, "in_except_handler": in_except, "in_finally": in_finally}


def check_jetson_controller_single_write(src):
    tree = ast.parse(src)
    consts = _module_level_int_consts(tree)
    call_sites = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            func = node.func
            is_target_call = (
                (isinstance(func, ast.Attribute) and func.attr == "Arm_serial_servo_write")
                or (isinstance(func, ast.Name) and func.id == "Arm_serial_servo_write"))
            if is_target_call:
                call_sites.append(node)

    details = []
    all_ok = len(call_sites) == 1
    for node in call_sites:
        args = [_resolve_const_value(a, consts) for a in node.args]
        placement = _in_loop_or_exception_handler(tree, node)
        args_ok = (args == [EXPECTED_SERVO_ID, EXPECTED_TARGET, EXPECTED_DURATION_MS])
        placement_ok = not any(placement.values())
        details.append({"line": node.lineno, "resolved_args": args, "args_ok": args_ok,
                        "placement": placement, "placement_ok": placement_ok})
        all_ok = all_ok and args_ok and placement_ok

    return {"call_site_count": len(call_sites), "call_sites": details,
            "module_level_consts_checked": {"SERVO_ID": consts.get("SERVO_ID"),
                                            "COMMAND_TARGET_ID2": consts.get("COMMAND_TARGET_ID2"),
                                            "COMMAND_DURATION_MS": consts.get("COMMAND_DURATION_MS")},
            "pass": all_ok}


def run_checks():
    exec_src = open(EXECUTION_PATH, encoding="utf-8").read()
    jc_src = open(JETSON_CONTROLLER_PATH, encoding="utf-8").read()
    r1 = check_execution_no_hardware_imports(exec_src)
    r2 = check_jetson_controller_single_write(jc_src)
    return {"execution_no_hardware_imports": r1, "jetson_controller_single_write": r2,
            "pass": r1["pass"] and r2["pass"]}


# ---------------------------------------------------------------------------
# true-positive 증명 — 합성 코드 문자열(이 파일의 소스가 아니다)로 검사기 자체가
# 실제로 위반을 탐지하는지 확인한다(forbidden-pattern-literal-self-matches 교훈
# 재적용 — 이 파일 자신을 "위반 샘플"로 스캔하지 않는다).
# ---------------------------------------------------------------------------
_SYNTHETIC_BAD_IMPORT = "import cv2\nimport Arm_Lib\n\ndef f():\n    pass\n"
_SYNTHETIC_RETRY_LOOP = (
    "SERVO_ID = 2\nCOMMAND_TARGET_ID2 = 151\nCOMMAND_DURATION_MS = 2000\n"
    "def write():\n    for _ in range(3):\n"
    "        arm.Arm_serial_servo_write(SERVO_ID, COMMAND_TARGET_ID2, COMMAND_DURATION_MS)\n")
_SYNTHETIC_FINALLY_RETRY = (
    "SERVO_ID = 2\nCOMMAND_TARGET_ID2 = 151\nCOMMAND_DURATION_MS = 2000\n"
    "def write():\n    try:\n        pass\n    finally:\n"
    "        arm.Arm_serial_servo_write(SERVO_ID, COMMAND_TARGET_ID2, COMMAND_DURATION_MS)\n")
_SYNTHETIC_WRONG_TARGET = (
    "SERVO_ID = 2\nCOMMAND_TARGET_ID2 = 151\nCOMMAND_DURATION_MS = 2000\n"
    "def write():\n    arm.Arm_serial_servo_write(2, 154, 2000)\n")
_SYNTHETIC_TWO_CALLS = (
    "SERVO_ID = 2\nCOMMAND_TARGET_ID2 = 151\nCOMMAND_DURATION_MS = 2000\n"
    "def write():\n    arm.Arm_serial_servo_write(SERVO_ID, COMMAND_TARGET_ID2, COMMAND_DURATION_MS)\n"
    "def retry():\n    arm.Arm_serial_servo_write(SERVO_ID, COMMAND_TARGET_ID2, COMMAND_DURATION_MS)\n")
_SYNTHETIC_GOOD = (
    "SERVO_ID = 2\nCOMMAND_TARGET_ID2 = 151\nCOMMAND_DURATION_MS = 2000\n"
    "def write():\n    arm.Arm_serial_servo_write(SERVO_ID, COMMAND_TARGET_ID2, COMMAND_DURATION_MS)\n")


def selftest():
    results = {}

    def rec(name, cond, detail=None):
        results[name] = {"pass": bool(cond), "detail": detail}

    r = check_execution_no_hardware_imports(_SYNTHETIC_BAD_IMPORT)
    rec("TP-01_detects_cv2_and_arm_lib_import", not r["pass"] and set(r["forbidden_found"]) == {"Arm_Lib", "cv2"}, r)

    rec("TP-02_real_execution_file_passes", check_execution_no_hardware_imports(
        open(EXECUTION_PATH, encoding="utf-8").read())["pass"])

    r = check_jetson_controller_single_write(_SYNTHETIC_RETRY_LOOP)
    rec("TP-03_detects_write_inside_for_loop", not r["pass"] and r["call_sites"][0]["placement"]["in_loop"], r)

    r = check_jetson_controller_single_write(_SYNTHETIC_FINALLY_RETRY)
    rec("TP-04_detects_write_inside_finally", not r["pass"] and r["call_sites"][0]["placement"]["in_finally"], r)

    r = check_jetson_controller_single_write(_SYNTHETIC_WRONG_TARGET)
    rec("TP-05_detects_wrong_target_literal", not r["pass"] and not r["call_sites"][0]["args_ok"], r)

    r = check_jetson_controller_single_write(_SYNTHETIC_TWO_CALLS)
    rec("TP-06_detects_second_call_site", not r["pass"] and r["call_site_count"] == 2, r)

    r = check_jetson_controller_single_write(_SYNTHETIC_GOOD)
    rec("TP-07_good_synthetic_sample_passes", r["pass"], r)

    r = check_jetson_controller_single_write(open(JETSON_CONTROLLER_PATH, encoding="utf-8").read())
    rec("TP-08_real_jetson_controller_file_passes", r["pass"], r)

    all_pass = all(v["pass"] for v in results.values())
    return {"all_pass": all_pass, "results": results}


if __name__ == "__main__":
    import json
    report = run_checks()
    tp = selftest()
    out = {"static_check": report, "true_positive_selftest": tp,
          "overall_pass": report["pass"] and tp["all_pass"]}
    print(json.dumps(out, ensure_ascii=False, indent=1, default=repr))
    import sys
    sys.exit(0 if out["overall_pass"] else 1)
