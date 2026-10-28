#!/usr/bin/env python3
"""P-007F-03 — 로봇 하드웨어 API가 이 gate 코드에 전혀 참조되지 않음을 AST로 검증한다.

P-007F-03은 카메라(읽기 전용)만 다루고 서보 read/write 포함 로봇 API는
일절 호출하지 않는다(§2 — read 포함 전부 금지). 모든 ast.Attribute/ast.Name
노드를 스캔해 직접호출·참조-후-호출 패턴 양쪽을 다 잡는다
(forbidden-pattern-literal-self-matches-its-own-guard 메모리 교훈 반영 —
금지 토큰을 문자열 조립으로 비교해 이 파일 자신의 주석이 자기 오탐을
일으키지 않게 한다).
"""

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TARGET_FILES = [
    ROOT / "scripts" / "p007f_03_capture_static.py",
    ROOT / "scripts" / "p007f_03_visual_analysis.py",
]

_FORBIDDEN_PARTS = [
    ("Arm_", "Device"),
    ("Arm_serial_servo_", "write"),
    ("Arm_serial_servo_", "read"),
    ("Arm_serial_servo_read_", "any"),
    ("Arm_", "ping_servo"),
    ("Arm_", "get_hardversion"),
    ("Arm_", "Run"),
    ("torque_", "enable"),
    ("set_", "torque"),
    ("PWM_", "write"),
    ("reset_", "arm"),
]
FORBIDDEN_NAMES = {a + b for a, b in _FORBIDDEN_PARTS}
FORBIDDEN_SUBSTRINGS = ["i2c", "I2C", "serial_servo", "Arm_Lib", "torque", "pwm", "PWM"]


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


def scan_substrings(src):
    hits = []
    for lineno, line in enumerate(src.splitlines(), start=1):
        if line.strip().startswith("#"):
            continue
        for token in FORBIDDEN_SUBSTRINGS:
            if token in line:
                hits.append((token, lineno, line.strip()))
    return hits


def negative_tests():
    results = []

    # true-positive 증명 — 탐지기가 실제로 작동하는지(죽은 탐지기가 아닌지) 확인
    synth_direct = "arm.Arm_serial_servo_write(2, 150, 500)\n"
    synth_ref = "fn = arm.Arm_serial_servo_write\nfn(2, 150, 500)\n"
    for label, src in (("direct_call", synth_direct), ("ref_then_call", synth_ref)):
        hits = scan_names_and_attrs(src)
        results.append((f"PF03-TP-{label}", len(hits) > 0, f"hits={hits}"))

    # 실제 대상 파일 검사
    for path in TARGET_FILES:
        src = path.read_text()
        name_hits = scan_names_and_attrs(src)
        sub_hits = scan_substrings(src)
        results.append((f"PF03-REAL-{path.name}-forbidden_api", len(name_hits) == 0, f"hits={name_hits}"))
        results.append((f"PF03-REAL-{path.name}-forbidden_substring", len(sub_hits) == 0, f"hits={sub_hits}"))

    return results


def main():
    results = negative_tests()
    failed = [r for r in results if not r[1]]
    for name, ok, detail in results:
        print(("PASS" if ok else "FAIL"), name, detail)
    if failed:
        raise SystemExit(f"{len(failed)}건 실패")
    print(f"전체 {len(results)}건 PASS — hardware API 참조 0건 확인")


if __name__ == "__main__":
    main()
