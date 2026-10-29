#!/usr/bin/env python3
"""P-007F-07 Phase A — §28 hardware-free verification.

확인 대상:
  1. scripts/p007f_07_execution.py -- 하드웨어 import 0, p007f_07_jetson_controller
     모듈조차 import 하지 않음(즉 Phase A 코드 경로 어디서도 실제 write 함수에
     도달할 수 없음을 구조적으로 증명).
  2. scripts/p007f_07_marker_detector.py / trajectory_classifier.py /
     synthetic_tests.py -- 하드웨어 import 0(cv2.aruco/numpy 는 허용, VideoCapture
     호출은 금지).
  3. scripts/p007f_07_jetson_controller.py -- `Arm_serial_servo_write` 호출이
     정확히 1곳(`do_write()` 안)이고 루프/except/finally 밖에 있음(§23).
  4. scripts/p007f_07_continuous_capture.py -- Arm_serial_servo_write 등 서보
     API를 전혀 참조하지 않음(카메라 wiring 계층이라 rclpy/cv2 import 는 허용).

🔴 forbidden-pattern-literal-self-matches-its-own-guard 교훈 재적용 -- AST 노드만
사용, substring 스캔 안 함(이 파일 자신의 docstring에 "Arm_serial_servo_write" 등
금지어가 설명문으로 등장하므로).
"""
import ast
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(ROOT, "scripts")

FORBIDDEN_HARDWARE_IMPORTS = {"Arm_Lib", "smbus", "smbus2", "rospy", "Adafruit_GPIO"}
FORBIDDEN_CALL_NAMES = {"Arm_serial_servo_write", "Arm_serial_set_torque", "Arm_PWM_servo_write",
                        "Arm_reset", "Arm_Clear_Action", "VideoCapture"}


def _imports(tree):
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                imported.add(a.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    return imported


def _calls(tree, names):
    out = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            func = node.func
            name = func.attr if isinstance(func, ast.Attribute) else (func.id if isinstance(func, ast.Name) else None)
            if name in names:
                out.append({"name": name, "line": node.lineno})
    return out


def check_no_hardware_file(path, extra_forbidden_imports=None):
    src = open(path, encoding="utf-8").read()
    tree = ast.parse(src)
    forbidden_imports = FORBIDDEN_HARDWARE_IMPORTS | (extra_forbidden_imports or set())
    imported = _imports(tree)
    bad_imports = sorted(imported & forbidden_imports)
    bad_calls = _calls(tree, FORBIDDEN_CALL_NAMES)
    return {"path": os.path.relpath(path, ROOT), "imports": sorted(imported),
           "forbidden_imports_found": bad_imports, "forbidden_calls_found": bad_calls,
           "pass": not bad_imports and not bad_calls}


def check_execution_no_write_module(path):
    """p007f_07_execution.py 는 p007f_07_jetson_controller 를 import 조차
    하면 안 된다(실제 write 함수에 도달할 import 경로 자체가 없어야 함)."""
    r = check_no_hardware_file(path, extra_forbidden_imports={"p007f_07_jetson_controller"})
    return r


def _module_level_names(tree):
    names = set()
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            continue
    return names


def check_jetson_controller_single_write(path):
    src = open(path, encoding="utf-8").read()
    tree = ast.parse(src)
    call_sites = []
    do_write_func = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "do_write":
            do_write_func = node
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            func = node.func
            name = func.attr if isinstance(func, ast.Attribute) else (func.id if isinstance(func, ast.Name) else None)
            if name == "Arm_serial_servo_write":
                call_sites.append(node)

    details = []
    all_ok = len(call_sites) == 1
    for node in call_sites:
        inside_do_write = (do_write_func is not None
                           and do_write_func.lineno <= node.lineno <= (do_write_func.end_lineno or 10 ** 9))
        in_loop, in_except, in_finally = _placement(tree, node)
        ok = inside_do_write and not in_loop and not in_except and not in_finally
        details.append({"line": node.lineno, "inside_do_write": inside_do_write,
                        "in_loop": in_loop, "in_except": in_except, "in_finally": in_finally, "ok": ok})
        all_ok = all_ok and ok
    return {"call_site_count": len(call_sites), "call_sites": details, "pass": all_ok}


def _placement(tree, target_node):
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
    return in_loop, in_except, in_finally


def run_checks():
    results = {
        "execution_py": check_execution_no_write_module(os.path.join(SCRIPTS, "p007f_07_execution.py")),
        "marker_detector_py": check_no_hardware_file(os.path.join(SCRIPTS, "p007f_07_marker_detector.py")),
        "trajectory_classifier_py": check_no_hardware_file(os.path.join(SCRIPTS, "p007f_07_trajectory_classifier.py")),
        "synthetic_tests_py": check_no_hardware_file(os.path.join(SCRIPTS, "p007f_07_synthetic_tests.py")),
        "continuous_capture_py": check_no_hardware_file(os.path.join(SCRIPTS, "p007f_07_continuous_capture.py")),
        "jetson_controller_single_write": check_jetson_controller_single_write(
            os.path.join(SCRIPTS, "p007f_07_jetson_controller.py")),
    }
    overall = all(r["pass"] for r in results.values())
    return {"results": results, "pass": overall, "robot_command_count": 0 if overall else None}


if __name__ == "__main__":
    import json
    import sys
    out = run_checks()
    print(json.dumps(out, ensure_ascii=False, indent=1))
    sys.exit(0 if out["pass"] else 1)
