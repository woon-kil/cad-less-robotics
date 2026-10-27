#!/usr/bin/env python3
"""P-007D — 설치 Arm_Lib 의 SMBus write/read call-site 전수 인벤토리.

정적 분석 전용. Arm_Lib 를 import 하지 않는다. I2C 접근 0.
소스 경로를 인자로 받는다 (스크립트는 소스 디렉터리 밖에 둔다).

사용: python3 -I scripts/p007d_smbus_inventory.py <Arm_Lib.py 경로>
"""
import ast, json, sys

SMBUS = {"write_byte", "write_byte_data", "write_word_data", "write_i2c_block_data",
         "read_byte", "read_byte_data", "read_word_data", "read_i2c_block_data"}
WRITES = {"write_byte", "write_byte_data", "write_word_data", "write_i2c_block_data"}


def seg(src_lines, node):
    """노드의 소스 텍스트를 한 줄로 되돌린다."""
    try:
        return ast.unparse(node)
    except Exception:
        return "<unparse 실패 line %d>" % getattr(node, "lineno", -1)


def main(path):
    src = open(path, encoding="utf-8").read()
    lines = src.splitlines()
    tree = ast.parse(src)

    calls = []
    for fn in [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)]:
        for node in ast.walk(fn):
            if not isinstance(node, ast.Call):
                continue
            f = node.func
            if not (isinstance(f, ast.Attribute) and f.attr in SMBUS):
                continue
            # self.bus.<method> 인지 확인
            recv = seg(lines, f.value)
            args = [seg(lines, a) for a in node.args]
            payload_len = None
            if args and isinstance(node.args[-1], ast.List):
                payload_len = len(node.args[-1].elts)
            calls.append({
                "function": fn.name,
                "def_line": fn.lineno,
                "call_line": node.lineno,
                "receiver": recv,
                "primitive": f.attr,
                "is_write": f.attr in WRITES,
                "args": args,
                "addr_arg": args[0] if len(args) > 0 else None,
                "register_expr": args[1] if len(args) > 1 else None,
                "payload_expr": args[2] if len(args) > 2 else None,
                "payload_len": payload_len,
            })

    funcs = sorted({c["function"] for c in calls})
    by_prim = {}
    for c in calls:
        by_prim.setdefault(c["primitive"], []).append(c["function"])

    out = {
        "source": path,
        "source_lines": len(lines),
        "total_functions_in_class": len([n for n in ast.walk(tree)
                                         if isinstance(n, ast.FunctionDef)]),
        "smbus_call_sites": len(calls),
        "functions_touching_bus": funcs,
        "by_primitive": {k: sorted(set(v)) for k, v in sorted(by_prim.items())},
        "counts_by_primitive": {k: len(v) for k, v in sorted(by_prim.items())},
        "calls": calls,
    }
    json.dump(out, sys.stdout, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main(sys.argv[1])
