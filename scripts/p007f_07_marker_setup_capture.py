#!/usr/bin/env python3
"""P-007F-07 — Marker Setup Reference Capture (사용자 지시, 2026-10-29).

목적: 현재 Frozen Gemini 335L Baseline B 시점에서 실제 DOFBOT Arm2 marker
부착 위치를 결정할 reference RGB image 를 확보한다.

제한 (work order 원문):
    ROBOT COMMANDS    = 0
    SERVO WRITES      = 0
    TORQUE COMMANDS   = 0
    ROBOT MOVEMENT    = 0
    GEMINI MOVEMENT   = 0

이 스크립트는 scripts/p007f_05_hardware_wiring.py 의 기존 함수(start/stop
driver, identity check, baseline B check, capture_window_on_p620,
fetch_capture_dir)를 SHA-pin import 로 **수정 없이** 재사용한다 -- Jetson
controller 쪽은 import 하지 않는다(robot_command_count=0 을 import 자체로
구조적으로 보장, grep self-check 로 재확인).
"""
import hashlib
import importlib.util
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EVIDENCE_DIR = os.path.join(ROOT, "evidence", "P-007F-07", "marker_setup_capture")
RAW_FRAMES_DIR = os.path.join(EVIDENCE_DIR, "raw_frames")

WIRING_PATH = os.path.join(ROOT, "scripts", "p007f_05_hardware_wiring.py")


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        h.update(f.read())
    return h.hexdigest()


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# 자기검증: 이 파일 전체에 Jetson/Arm_Lib/servo 관련 토큰이 없는지 확인
# (조립식 문자열이 아니라 이 파일 자신의 소스를 읽어 검사 -- 자기 자신이
# 금지 리터럴을 포함하지 않게 substring 이 아니라 import 문만 검사한다).
def self_check_no_jetson_import():
    import ast
    with open(os.path.abspath(__file__), "r", encoding="utf-8") as f:
        tree = ast.parse(f.read())
    forbidden_modules = {"p007f_05_jetson_controller", "p007f_07_jetson_controller", "Arm_Lib"}
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imported.add(alias.name)
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                imported.add(node.module)
    return forbidden_modules.intersection(imported)


def sharpness_score(cv2, path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return -1.0
    return float(cv2.Laplacian(img, cv2.CV_64F).var())


def mean_brightness(cv2, path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    return float(img.mean()) if img is not None else None


def draw_overlay(cv2, in_path, out_path):
    """§6/§7 marker mounting instruction 의 구조적 위치를 기존 ROI 좌표 체계
    (scripts/p007f_06_forensics.py ROIS, 동일 1280x720 Gemini 프레임 기준)
    위에 근사 박스로 표시한다. 픽셀 단위로 정밀하지 않음을 라벨에 명시한다
    (marker_mount_instruction.md 의 불확실성 공개와 동일 입장)."""
    img = cv2.imread(in_path)
    candidates = {
        # (x, y, w, h), color(BGR), label
        "PRIMARY_A": ((655, 225, 70, 50), (0, 255, 0),
                     "PRIMARY_A: white disc (ARM2_ROI), offset from axis"),
        "PRIMARY_B": ((650, 195, 90, 30), (0, 200, 255),
                     "PRIMARY_B: diagonal link (ARM2<->ARM3 boundary)"),
        "SECONDARY_A": ((600, 180, 90, 50), (255, 0, 255),
                        "SECONDARY_A: wrist housing (END_EFFECTOR_ROI edge)"),
    }
    for name, ((x, y, w, h), color, label) in candidates.items():
        cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
        ty = y - 8 if y - 8 > 10 else y + h + 18
        cv2.putText(img, label, (max(2, x - 20), ty), cv2.FONT_HERSHEY_SIMPLEX,
                   0.45, color, 1, cv2.LINE_AA)
    cv2.putText(img, "CANDIDATE_ONLY -- APPROXIMATE, NOT FINAL (see marker_mount_instruction.md)",
               (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(out_path, img)
    return candidates


def main():
    forbidden_found = self_check_no_jetson_import()
    if forbidden_found:
        print(json.dumps({"status": "ABORT", "reason": "FORBIDDEN_IMPORT", "found": list(forbidden_found)}))
        sys.exit(1)

    wiring_sha = sha256(WIRING_PATH)
    wiring = _load("p007f_05_hardware_wiring_pin", WIRING_PATH)
    print(json.dumps({"status": "WIRING_MODULE_LOADED", "sha256": wiring_sha}, ensure_ascii=False))

    wiring.start_gemini_driver()
    time.sleep(4.0)
    identity = wiring.gemini_identity_check()
    if not identity.get("identity_valid"):
        wiring.stop_gemini_driver()
        print(json.dumps({"status": "ABORT", "reason": "GEMINI_IDENTITY_CHECK_FAILED", "identity": identity},
                         ensure_ascii=False))
        sys.exit(1)

    out_dir_name = "marker_setup_capture_%d" % time.time()
    rc, out, err, remote_out = wiring.capture_window_on_p620(out_dir_name, frame_count=5)
    fetch_rc, fetch_out, fetch_err = wiring.fetch_capture_dir(remote_out, RAW_FRAMES_DIR)
    fetched_local_dir = os.path.join(RAW_FRAMES_DIR, out_dir_name)

    summary_path = os.path.join(fetched_local_dir, "capture-summary.json")
    frames_ok = os.path.isfile(summary_path)
    if not frames_ok:
        wiring.stop_gemini_driver()
        print(json.dumps({"status": "ABORT", "reason": "CAPTURE_SUMMARY_MISSING",
                          "capture_rc": rc, "capture_stdout": out, "capture_stderr": err,
                          "fetch_rc": fetch_rc, "fetch_err": fetch_err}, ensure_ascii=False))
        sys.exit(1)

    import importlib
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    va = importlib.import_module("p007f_03_visual_analysis")
    cv2 = va.cv2

    color_frames = sorted(f for f in os.listdir(fetched_local_dir) if f.endswith("-color.png"))
    scored = [(f, sharpness_score(cv2, os.path.join(fetched_local_dir, f))) for f in color_frames]
    scored.sort(key=lambda t: t[1], reverse=True)
    best_frame_name = scored[0][0] if scored else None
    best_frame_path = os.path.join(fetched_local_dir, best_frame_name) if best_frame_name else None

    baseline_check = None
    if best_frame_path:
        baseline_check = wiring.baseline_b_check(best_frame_path)

    wiring.stop_gemini_driver()

    if not best_frame_path:
        print(json.dumps({"status": "ABORT", "reason": "NO_COLOR_FRAMES_FETCHED",
                          "fetched_local_dir": fetched_local_dir}, ensure_ascii=False))
        sys.exit(1)

    rgb_reference_path = os.path.join(EVIDENCE_DIR, "rgb_reference.png")
    img = cv2.imread(best_frame_path)
    cv2.imwrite(rgb_reference_path, img)
    height, width = img.shape[:2]

    overlay_path = os.path.join(EVIDENCE_DIR, "marker_candidate_overlay.png")
    candidates = draw_overlay(cv2, best_frame_path, overlay_path)

    metadata = {
        "camera": "Gemini 335L",
        "serial": wiring.GEMINI_EXPECTED_SERIAL,
        "firmware": wiring.GEMINI_EXPECTED_FIRMWARE,
        "resolution": {"width": width, "height": height},
        "timestamp_unix": time.time(),
        "baseline_b_status": baseline_check,
        "robot_command_count": 0,
        "servo_write_count": 0,
        "frames_requested": 5,
        "frames_captured": len(color_frames),
        "frame_scores_laplacian_var": [{"frame": f, "sharpness": s, "mean_brightness": mean_brightness(cv2, os.path.join(fetched_local_dir, f))} for f, s in scored],
        "selected_reference_frame": best_frame_name,
        "selection_criterion": "max Laplacian variance (sharpest) among 5 captured frames",
        "identity_check": identity,
        "marker_candidates": {k: {"box_xywh": v[0], "label": v[2]} for k, v in candidates.items()},
        "marker_location_status": "CANDIDATE_ONLY",
        "wiring_module_sha256": wiring_sha,
    }
    with open(os.path.join(EVIDENCE_DIR, "capture_metadata.json"), "w", encoding="utf-8") as f:
        json.dump(metadata, f, ensure_ascii=False, indent=1, default=repr)

    print(json.dumps({"status": "DONE", "metadata": metadata}, ensure_ascii=False, indent=1, default=repr))


if __name__ == "__main__":
    main()
