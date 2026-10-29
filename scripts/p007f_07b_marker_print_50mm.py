#!/usr/bin/env python3
"""P-007F-07B Phase B1 -- 50mm ArUco Print Sheet.

실제 motion 0, robot/servo 0. 30mm 버전(scripts/p007f_07_marker_print_sheet.py,
수정 없이 보존)의 Dictionary/ID 선택과 레이아웃/검증 로직을 **그대로 재사용**하고
물리 크기만 50.0mm로 바꾼다 -- Dictionary/ID를 새로 정하지 않는다(사용자 지시:
"ID를 새로 정하지 않는다 -- 기존 PRIMARY_MARKER_ID=0과 반드시 일치시킨다").

재사용 방법: 원본 모듈을 SHA-pin import 한 뒤 **그 모듈의 함수(build_figure 등)를
그대로 호출**하면서 모듈 전역 상수(BLACK_SQUARE_MM/MARKER_PX)만 이 스크립트에서
override 한다(원본 파일은 1바이트도 수정하지 않는다 -- 원본은 30mm 산출물의
재현성을 위해 그대로 보존).
"""
import importlib.util
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(ROOT, "evidence", "P-007F-07", "marker_print_50mm")
PRINT30_MODULE_PATH = os.path.join(ROOT, "scripts", "p007f_07_marker_print_sheet.py")

BLACK_SQUARE_MM = 50.0
MARKER_PX = 600  # 100px/module, multiple of 6 -- 30mm 버전과 동일 density(12px/mm)


def self_check_no_hardware_import():
    import ast
    with open(os.path.abspath(__file__), "r", encoding="utf-8") as f:
        tree = ast.parse(f.read())
    forbidden = {"rclpy", "Arm_Lib", "p007f_05_hardware_wiring", "p007f_05_jetson_controller",
                "p007f_07_jetson_controller", "subprocess"}
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imported.add(alias.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                imported.add(node.module.split(".")[0])
    return forbidden.intersection(imported)


def sha256(path):
    import hashlib
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def load_print30_module():
    spec = importlib.util.spec_from_file_location("p007f_07_marker_print_sheet_pin", PRINT30_MODULE_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    forbidden = self_check_no_hardware_import()
    if forbidden:
        print(json.dumps({"status": "ABORT", "reason": "FORBIDDEN_IMPORT", "found": list(forbidden)}))
        sys.exit(1)

    print30_sha_before = sha256(PRINT30_MODULE_PATH)
    m = load_print30_module()

    # Dictionary/ID -- 새로 정하지 않는다. detector 모듈(Phase A 사전등록)에서
    # 그대로 재확인(이 값은 p007f_07_marker_print_sheet.py 안에서 이미 한 번
    # assert 되지만, 여기서도 다시 한번 명시 재확인한다).
    det = m.load_detector_module()
    assert det.DEFAULT_DICT == det.cv2.aruco.DICT_4X4_50, "DICT changed unexpectedly"
    dict_name = "DICT_4X4_50"
    marker_id = det.PRIMARY_MARKER_ID
    assert marker_id == 0, "PRIMARY_MARKER_ID changed from the value used in P-007F-07 30mm print"

    marker_img = det.generate_marker_image(marker_id, MARKER_PX)

    os.makedirs(OUT_DIR, exist_ok=True)

    # 🔴 원본 파일은 수정하지 않는다 -- 로드된 모듈 인스턴스(m)의 전역만 override.
    m.BLACK_SQUARE_MM = BLACK_SQUARE_MM
    m.MARKER_PX = MARKER_PX
    m.OUT_DIR = OUT_DIR

    fig = m.build_figure(marker_img, dict_name, marker_id)
    pdf_path = os.path.join(OUT_DIR, "aruco_4x4_50_id%d_50mm_A4.pdf" % marker_id)
    fig.savefig(pdf_path)
    import matplotlib.pyplot as plt
    plt.close(fig)

    fig2 = m.build_figure(marker_img, dict_name, marker_id)
    png_path = os.path.join(OUT_DIR, "aruco_4x4_50_id%d_50mm.png" % marker_id)
    fig2.savefig(png_path, dpi=m.VERIFY_PNG_DPI)
    plt.close(fig2)

    # -- geometry verification (30mm 버전과 동일 2단계 방법, 크기만 50mm로) --
    with open(pdf_path, "rb") as f:
        pdf_bytes = f.read()
    mb_matches = re.findall(rb"/MediaBox\s*\[\s*([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s*\]", pdf_bytes)
    mediabox_pt = [tuple(float(v) for v in mm_) for mm_ in mb_matches]
    expected_w_pt = m.PAGE_W_MM / 25.4 * 72.0
    expected_h_pt = m.PAGE_H_MM / 25.4 * 72.0
    mediabox_ok = bool(mediabox_pt) and all(
        abs(mb[2] - mb[0] - expected_w_pt) < 0.5 and abs(mb[3] - mb[1] - expected_h_pt) < 0.5
        for mb in mediabox_pt)

    import numpy as np
    from PIL import Image
    png_img = np.array(Image.open(png_path).convert("L"))
    px_per_mm = m.VERIFY_PNG_DPI / 25.4
    dark = png_img < 128
    marker_bottom_mm = 170.0
    marker_left_mm = (m.PAGE_W_MM - BLACK_SQUARE_MM) / 2.0
    top_mm = m.PAGE_H_MM - (marker_bottom_mm + BLACK_SQUARE_MM)
    bottom_mm = m.PAGE_H_MM - marker_bottom_mm
    row0 = int((top_mm - 2) * px_per_mm)
    row1 = int((bottom_mm + 2) * px_per_mm)
    col0 = int((marker_left_mm - 2) * px_per_mm)
    col1 = int((marker_left_mm + BLACK_SQUARE_MM + 2) * px_per_mm)
    region = dark[max(0, row0):row1, max(0, col0):col1]
    r_rows = np.where(region.any(axis=1))[0]
    r_cols = np.where(region.any(axis=0))[0]
    measured_w_mm = measured_h_mm = None
    if r_rows.size and r_cols.size:
        measured_w_px = r_cols.max() - r_cols.min() + 1
        measured_h_px = r_rows.max() - r_rows.min() + 1
        measured_w_mm = measured_w_px / px_per_mm
        measured_h_mm = measured_h_px / px_per_mm

    geometry_ok = bool(mediabox_ok and measured_w_mm is not None and
                       abs(measured_w_mm - BLACK_SQUARE_MM) < 0.5 and abs(measured_h_mm - BLACK_SQUARE_MM) < 0.5)

    print30_sha_after = sha256(PRINT30_MODULE_PATH)
    report = {
        "gate": "P-007F-07B Phase B1",
        "dictionary": dict_name,
        "marker_id": marker_id,
        "marker_id_source": "scripts/p007f_07_marker_detector.py PRIMARY_MARKER_ID (reused unmodified, unchanged from P-007F-07 30mm print)",
        "black_square_physical_size_mm": BLACK_SQUARE_MM,
        "pdf_path": pdf_path,
        "png_path": png_path,
        "pdf_mediabox_pt": mediabox_pt,
        "pdf_mediabox_expected_pt": [0.0, 0.0, round(expected_w_pt, 2), round(expected_h_pt, 2)],
        "pdf_mediabox_matches_a4": mediabox_ok,
        "png_verify_dpi": m.VERIFY_PNG_DPI,
        "png_measured_marker_width_mm": round(measured_w_mm, 3) if measured_w_mm else None,
        "png_measured_marker_height_mm": round(measured_h_mm, 3) if measured_h_mm else None,
        "geometry_verification_passed": geometry_ok,
        "authoritative_geometry_source": "PDF vector MediaBox + axes-fraction placement (dpi-independent); PNG raster measurement is a supplementary cross-check only",
        "print30_module_sha256_before": print30_sha_before,
        "print30_module_sha256_after": print30_sha_after,
        "print30_module_unmodified": print30_sha_before == print30_sha_after,
        "print_ready": geometry_ok,
        "robot_command_count": 0,
        "servo_write_count": 0,
    }
    with open(os.path.join(OUT_DIR, "geometry_report.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=1, default=repr)
    print(json.dumps(report, ensure_ascii=False, indent=1, default=repr))
    if not geometry_ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
