#!/usr/bin/env python3
"""P-007F-04 — candidate 생성 코드(scripts/p007f_04_candidates.py)에 서보 write/torque/
reset/calibration/action/PWM/position write API 가 전혀 참조되지 않음을 AST로 검증한다.

p007f_04_candidates.py 는 FK/joint-limit/mesh 수학만 하는 순수 numpy 코드다 — Arm_Lib
자체를 import 하지 않으므로 기대값은 "Arm_* 참조 0건"(P-007F-03의 camera-read-only
기준보다 더 엄격: 읽기조차 하지 않는다). §4 의 read-only controller probe 는 별도로
P-007F-01 의 scripts/p007f_01_readonly_probe.py 를 **1바이트도 수정하지 않고** sha-pin
재사용했다(이 파일의 검사 대상이 아니다 — 그 파일 자체 self_ast_check/negative_tests 가
이미 P-007F-01 에서 검증됐고, 이번 gate에서 재실행한 self-test 결과를 그대로 인용한다).
"""
import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TARGET_FILES = [
    ROOT / "scripts" / "p007f_04_candidates.py",
]

_FORBIDDEN_PARTS = [
    ("Arm_", "Device"),
    ("Arm_serial_servo_", "write"),
    ("Arm_serial_servo_", "write6"),
    ("Arm_serial_servo_", "write_any"),
    ("Arm_serial_servo_", "read"),
    ("Arm_serial_servo_read_", "any"),
    ("Arm_", "ping_servo"),
    ("Arm_", "get_hardversion"),
    ("Arm_", "Run"),
    ("Arm_", "reset"),
    ("Arm_", "PWM_servo_write"),
    ("Arm_", "Clear_Action"),
    ("Arm_", "Action_Study"),
    ("Arm_", "Action_Mode"),
    ("Arm_", "RGB_set"),
    ("Arm_serial_servo_write_offset_", "switch"),
    ("Arm_serial_servo_write_offset_", "state"),
    ("torque_", "enable"),
    ("set_", "torque"),
    ("Arm_serial_set_", "torque"),
    ("bus_servo_", "control"),
]
FORBIDDEN_NAMES = {a + b for a, b in _FORBIDDEN_PARTS}
FORBIDDEN_SUBSTRINGS = ["i2c", "I2C", "Arm_Lib", "serial_servo", "torque", "pwm", "PWM", "get_i2c_bus_num"]


def scan_names_and_attrs(src):
    tree = ast.parse(src)
    hits = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute):
            if node.attr in FORBIDDEN_NAMES:
                hits.append(("attr", node.attr, node.lineno))
        if isinstance(node, ast.Name):
            if node.id in FORBIDDEN_NAMES:
                hits.append(("name", node.id, node.lineno))
    return hits


def _docstring_line_span(src):
    """모듈 docstring(설명 prose)의 줄 범위를 ast.get_docstring 으로 찾는다 — 산문으로
    금지 API 이름을 설명하는 것(실제 참조가 아님)을 substring 스캔에서 제외하기 위함.
    AST attr/name 스캔은 docstring 과 무관하게(파이썬 Expr 문자열일 뿐이라 애초에
    ast.Attribute/ast.Name 로 안 잡힘) 이미 안전하다 — 이 함수는 2차 휴리스틱(substring)
    레이어의 자기오탐만 줄인다."""
    tree = ast.parse(src)
    if not (tree.body and isinstance(tree.body[0], ast.Expr)
            and isinstance(getattr(tree.body[0], "value", None), ast.Constant)):
        return None
    node = tree.body[0]
    return node.lineno, node.end_lineno


def scan_substrings(src):
    hits = []
    doc_span = _docstring_line_span(src)
    for lineno, line in enumerate(src.splitlines(), start=1):
        stripped = line.strip()
        if stripped.startswith("#"):
            continue
        if doc_span and doc_span[0] <= lineno <= doc_span[1]:
            continue  # 모듈 docstring 산문 — 실제 참조가 아니다(위 docstring: 참고)
        if '"api":' in line:
            continue  # command_encoding 메타데이터 문자열 리터럴 — 호출이 아니라 향후
            # gate 승인문구/ledger 에 기록할 API 이름표를 JSON 산출물에 남기기 위한 값
        for token in FORBIDDEN_SUBSTRINGS:
            if token in line:
                hits.append((token, lineno, line.strip()))
    return hits


def negative_tests():
    results = []

    synth_direct = "arm.Arm_serial_servo_write(2, 151, 2000)\n"
    synth_ref = "fn = arm.Arm_serial_set_torque\nfn(1)\n"
    for label, src in (("direct_call", synth_direct), ("ref_then_call", synth_ref)):
        hits = scan_names_and_attrs(src)
        sub = scan_substrings(src)
        results.append((f"PF04-TP-{label}-attr", len(hits) > 0, f"hits={hits}"))
        results.append((f"PF04-TP-{label}-substr", len(sub) > 0, f"hits={sub}"))

    for path in TARGET_FILES:
        src = path.read_text()
        tree_ok = True
        try:
            ast.parse(src)
        except SyntaxError as exc:
            tree_ok = False
            results.append((f"PF04-REAL-{path.name}-parses", False, str(exc)))
        if tree_ok:
            name_hits = scan_names_and_attrs(src)
            sub_hits = scan_substrings(src)
            results.append((f"PF04-REAL-{path.name}-forbidden_api", len(name_hits) == 0, f"hits={name_hits}"))
            results.append((f"PF04-REAL-{path.name}-forbidden_substring", len(sub_hits) == 0, f"hits={sub_hits}"))
            # 더 엄격한 기준: Arm_Lib/hardware 와 관련된 어떤 import 도 없어야 한다
            imports = [n.names[0].name for n in ast.walk(ast.parse(src))
                       if isinstance(n, (ast.Import, ast.ImportFrom)) and n.names]
            forbidden_imports = [i for i in imports if "Arm_Lib" in i or "rclpy" in i or "serial" in i.lower()]
            results.append((f"PF04-REAL-{path.name}-no_hardware_imports", len(forbidden_imports) == 0,
                             f"imports={imports}"))

    return results


def main():
    results = negative_tests()
    failed = [r for r in results if not r[1]]
    for name, ok, detail in results:
        print(("PASS" if ok else "FAIL"), name, detail)
    if failed:
        raise SystemExit(f"{len(failed)}건 실패")
    print(f"전체 {len(results)}건 PASS — p007f_04_candidates.py 는 하드웨어 API 참조 0건(읽기조차 없음)")


if __name__ == "__main__":
    main()
