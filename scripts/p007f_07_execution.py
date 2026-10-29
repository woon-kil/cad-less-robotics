#!/usr/bin/env python3
"""P-007F-07 Phase A — §20~§24 orchestration skeleton. 순수 설계/검증 코드 --
Arm_Lib/smbus/rclpy/cv2 VideoCapture 등을 전혀 import 하지 않는다
(self_ast_check 로 구조적 보장). 실제 하드웨어 wiring 은
scripts/p007f_07_jetson_controller.py(Jetson, 유일한 write 지점) /
향후 작성될 hardware_wiring 모듈에만 있다 -- 이 파일은 그 함수들을 **호출하지
않는다**(Phase A 전체에서 실제 write 함수가 import조차 되지 않는다 -- 아래
self_ast_check 가 이 파일이 p007f_07_jetson_controller 를 import 하지 않음도
함께 증명한다).

§20 구조:
```
PHASE A(unattended): build -> self tests -> negative tests -> camera startup
  -> marker detector startup -> continuous recorder ready -> controller
  read-only -> provenance -> safety -> ledger -> PRE static qualification
  -> READY_AND_ARMED
```//
이 파일은 그 뒤에 오는 "승인 이후" 단계(§21~§24)의 **구조**만 구현한다 --
quick_invariant_checks() 는 Phase A에서 전부 주입 콜백(mock)으로만 테스트되고,
실제 hardware 콜백은 이 gate에서 연결되지 않는다.
"""
import ast
import hashlib
import json
import os
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LEDGER_DIR = os.path.join(ROOT, ".cache", "p007f07_ledger")

FORBIDDEN_HARDWARE_IMPORTS = {"Arm_Lib", "smbus", "smbus2", "rclpy", "rospy", "cv2", "Adafruit_GPIO"}
FORBIDDEN_MODULE_IMPORTS = FORBIDDEN_HARDWARE_IMPORTS | {"p007f_07_jetson_controller"}

LEDGER_STATES = ("PREPARED", "READY_AND_ARMED", "APPROVAL_RECEIVED", "COMMAND_SENT",
                 "POST_CAPTURE_COMPLETE", "COMPLETE")
# §24 -- COMMAND_SENT 이후(및 그 이후 모든 상태)에는 재시도를 막는다
NO_RETRY_STATES = ("COMMAND_SENT", "POST_CAPTURE_COMPLETE", "COMPLETE")

QUICK_INVARIANT_KEYS = ("controller_state_unchanged", "camera_stream_alive",
                        "marker_visible", "background_reference_valid", "ledger_not_already_sent")


def sha256(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def self_ast_check():
    src = open(os.path.abspath(__file__), "r", encoding="utf-8").read()
    tree = ast.parse(src)
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                imported.add(a.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    bad = sorted(imported & FORBIDDEN_MODULE_IMPORTS)
    direct_subprocess = "subprocess" in imported
    return {"imports": sorted(imported), "forbidden_found": bad,
            "direct_subprocess_import": direct_subprocess,
            "pass": not bad and not direct_subprocess}


def _ledger_path(exp_id, ledger_dir=LEDGER_DIR):
    return os.path.join(ledger_dir, exp_id + ".json")


def _atomic_write_json(path, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=1, default=repr)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def write_ledger_state(exp_id, state, detail=None, ledger_dir=LEDGER_DIR):
    """§24 -- durable write-ahead. 매 상태 전이마다 fsync 쓰기 + 재읽기 검증."""
    if state not in LEDGER_STATES:
        raise ValueError("unknown ledger state: %r" % state)
    path = _ledger_path(exp_id, ledger_dir)
    existing = {}
    if os.path.isfile(path):
        try:
            existing = json.load(open(path, encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            existing = {}
    history = existing.get("history", [])
    history.append({"state": state, "at": time.time(), "detail": detail or {}})
    _atomic_write_json(path, {"status": state, "history": history})
    reread = json.load(open(path, encoding="utf-8"))
    return reread.get("status") == state


def check_ledger_allows_restart(exp_id, ledger_dir=LEDGER_DIR):
    """§24 -- COMMAND_SENT 이후(그 이후 어떤 상태든) 존재하면 자동 재시도 금지."""
    path = _ledger_path(exp_id, ledger_dir)
    if not os.path.isfile(path):
        return {"allowed": True, "existing_status": None}
    try:
        data = json.load(open(path, encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as e:
        return {"allowed": False, "existing_status": "LEDGER_CORRUPT:%r" % e}
    status = data.get("status")
    if status in NO_RETRY_STATES:
        return {"allowed": False, "existing_status": status}
    return {"allowed": True, "existing_status": status}


def experiment_id(preregistration_sha, servo_id, target_degrees, duration_ms):
    key = "%s:%s:%s:%s" % (preregistration_sha, servo_id, target_degrees, duration_ms)
    return hashlib.sha256(key.encode()).hexdigest()[:32]


# ---------------------------------------------------------------------------
# §21/§22 -- approval -> command fast path. 승인 후 허용되는 유일한 작업은
# quick_invariant_checks() 뿐이다(build/test/analysis/camera init/SSH 설정/
# 파일 전송 없음). 전부 injected callable -- Phase A 에서는 mock 으로만 호출된다.
# ---------------------------------------------------------------------------
def quick_invariant_checks(checks_fn_map):
    """checks_fn_map: {key: zero-arg callable returning bool} for each of
    QUICK_INVARIANT_KEYS. 하나라도 False/예외면 즉시 ABORT_BEFORE_MOTION,
    자동 재시도 없음."""
    results = {}
    for key in QUICK_INVARIANT_KEYS:
        fn = checks_fn_map.get(key)
        if fn is None:
            results[key] = {"ok": False, "reason": "CHECK_NOT_PROVIDED"}
            continue
        try:
            ok = bool(fn())
            results[key] = {"ok": ok, "reason": None if ok else "CHECK_FAILED"}
        except Exception as e:  # noqa: BLE001 -- 어떤 예외든 fail-closed
            results[key] = {"ok": False, "reason": "EXCEPTION:%r" % e}
    all_ok = all(v["ok"] for v in results.values())
    return {"pass": all_ok, "checks": results,
           "status": "PASS" if all_ok else "ABORT_BEFORE_MOTION"}


def approval_phrase_matches(typed_phrase, required_phrase):
    return typed_phrase == required_phrase


# ---------------------------------------------------------------------------
# negative tests -- 전부 mock, 실하드웨어 0. p007f_07_jetson_controller 는
# import 조차 하지 않는다(self_ast_check 로 증명).
# ---------------------------------------------------------------------------
def negative_tests():
    import shutil
    import tempfile
    results = {}

    def rec(name, cond, detail=None):
        results[name] = {"pass": bool(cond), "detail": detail}

    with tempfile.TemporaryDirectory(prefix="p007f07ledger_") as ld:
        exp_id = experiment_id("fakeprereg", 2, 151, 2000)

        # NT-01 -- fresh exp_id allows start
        check1 = check_ledger_allows_restart(exp_id, ledger_dir=ld)
        rec("NT-01_fresh_experiment_allows_start", check1["allowed"], check1)

        # NT-02 -- state transitions write+reread correctly
        for s in ("PREPARED", "READY_AND_ARMED", "APPROVAL_RECEIVED"):
            ok = write_ledger_state(exp_id, s, ledger_dir=ld)
            rec("NT-02_state_%s_durable_write_verified" % s, ok)

        # NT-03 -- after COMMAND_SENT, restart is blocked
        write_ledger_state(exp_id, "COMMAND_SENT", ledger_dir=ld)
        check3 = check_ledger_allows_restart(exp_id, ledger_dir=ld)
        rec("NT-03_command_sent_blocks_restart", not check3["allowed"]
            and check3["existing_status"] == "COMMAND_SENT", check3)

        # NT-04 -- after POST_CAPTURE_COMPLETE/COMPLETE, still blocked (not just COMMAND_SENT)
        write_ledger_state(exp_id, "POST_CAPTURE_COMPLETE", ledger_dir=ld)
        write_ledger_state(exp_id, "COMPLETE", ledger_dir=ld)
        check4 = check_ledger_allows_restart(exp_id, ledger_dir=ld)
        rec("NT-04_complete_still_blocks_restart", not check4["allowed"], check4)

        # NT-05 -- quick_invariant_checks: all pass -> PASS
        all_true = {k: (lambda: True) for k in QUICK_INVARIANT_KEYS}
        r5 = quick_invariant_checks(all_true)
        rec("NT-05_all_invariants_pass", r5["pass"], r5)

        # NT-06 -- one failing invariant -> ABORT_BEFORE_MOTION, no exception propagates
        one_false = dict(all_true)
        one_false["marker_visible"] = lambda: False
        r6 = quick_invariant_checks(one_false)
        rec("NT-06_one_invariant_fails_aborts", not r6["pass"]
            and r6["status"] == "ABORT_BEFORE_MOTION", r6)

        # NT-07 -- exception in a check is treated as failure, not propagated
        def raiser():
            raise RuntimeError("simulated camera failure")
        with_exception = dict(all_true)
        with_exception["camera_stream_alive"] = raiser
        r7 = quick_invariant_checks(with_exception)
        rec("NT-07_exception_in_check_treated_as_failure", not r7["pass"], r7)

        # NT-08 -- missing check key -> failure (fail-closed, not silently skipped)
        missing = dict(all_true)
        del missing["ledger_not_already_sent"]
        r8 = quick_invariant_checks(missing)
        rec("NT-08_missing_check_key_fails_closed", not r8["pass"], r8)

        # NT-09 -- approval phrase exact match only
        rec("NT-09_approval_phrase_exact_match",
            approval_phrase_matches("EXECUTE X", "EXECUTE X")
            and not approval_phrase_matches("execute x", "EXECUTE X")
            and not approval_phrase_matches("EXECUTE X ", "EXECUTE X"))

        # NT-10 -- self_ast_check: no hardware imports, and jetson_controller module
        # (the one real write function) is not imported by this orchestration file
        chk = self_ast_check()
        rec("NT-10_self_ast_check_no_hardware_or_write_module_imports", chk["pass"], chk)

    all_pass = all(v["pass"] for v in results.values())
    return {"all_pass": all_pass, "results": results}


def selftest():
    return {"self_ast_check": self_ast_check(), "negative_tests": negative_tests()}


if __name__ == "__main__":
    import sys
    out = selftest()
    print(json.dumps(out, ensure_ascii=False, indent=1, default=repr))
    ok = out["self_ast_check"]["pass"] and out["negative_tests"]["all_pass"]
    sys.exit(0 if ok else 1)
