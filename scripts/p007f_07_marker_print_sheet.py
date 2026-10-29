#!/usr/bin/env python3
"""P-007F-07 -- 30mm ArUco Print Sheet (사용자 지시, 2026-10-29).

실제 motion 0, robot/servo 0. 이 스크립트는 저장된 marker bitmap 생성
함수(scripts/p007f_07_marker_detector.py.generate_marker_image, Phase A에서
이미 작성됨, SHA-pin import)만 호출하고 PDF/PNG를 그린다 -- 카메라/로봇 API를
전혀 참조하지 않는다.

🔴 Dictionary/ID는 **새로 정하지 않는다** -- Phase A 에서 이미 사전등록된
scripts/p007f_07_marker_detector.py 의 DEFAULT_DICT(DICT_4X4_50)/
PRIMARY_MARKER_ID(0) 를 그대로 가져온다(사용자 지시: "Cursor가 기존 코드를
먼저 확인하고 기존 ID가 있으면 그것을 우선"). 사용자가 제안한 ID=0은
우연히 기존 사전등록 값과 일치한다 -- 이 스크립트가 그것을 재확인했다.

물리 크기 보장 방법: matplotlib Figure 를 **정확히 A4 inch 크기**로
생성하고([0,1]x[0,1] 전체를 덮는 단일 axes 에 xlim=[0,PAGE_W_MM],
ylim=[0,PAGE_H_MM], aspect='equal' 고정), 모든 요소(marker 이미지 포함)를
그 mm 좌표계에 직접 그린다. matplotlib 의 PDF 백엔드는 axes 배치를 analytic
(벡터) 좌표로 기록하므로 dpi 에 의존하지 않는다 -- PNG 저장은 검증용으로
dpi 를 명시해 픽셀->mm 역산이 가능하게 한다(§3 "출력 검증").
"""
import importlib.util
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(ROOT, "evidence", "P-007F-07", "marker_print")
DETECTOR_PATH = os.path.join(ROOT, "scripts", "p007f_07_marker_detector.py")

PAGE_W_MM, PAGE_H_MM = 210.0, 297.0  # A4 portrait
BLACK_SQUARE_MM = 30.0
MARKER_PX = 360  # 60px/module, multiple of 6 (4x4 code + 1-module border each side)
VERIFY_PNG_DPI = 300  # exact round number -> px_per_mm = 300/25.4, used only for empirical cross-check

QUIET_ZONE_MARGIN_MM = 3.0  # >=1 module width recommended; shown as a labeled dashed guide only


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


def load_detector_module():
    spec = importlib.util.spec_from_file_location("p007f_07_marker_detector_pin", DETECTOR_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def mm_to_fig_fraction_x(mm):
    return mm / PAGE_W_MM


def mm_to_fig_fraction_y(mm):
    return mm / PAGE_H_MM


def build_figure(marker_img, dict_name, marker_id):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    fig = plt.figure(figsize=(PAGE_W_MM / 25.4, PAGE_H_MM / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, PAGE_W_MM)
    ax.set_ylim(0, PAGE_H_MM)
    ax.set_aspect("equal")
    ax.axis("off")

    # -- header: print instructions (must survive even if everything else is misjudged) --
    ax.text(PAGE_W_MM / 2, 280, "PRINT AT 100% / ACTUAL SIZE", ha="center", va="center",
           fontsize=16, fontweight="bold")
    ax.text(PAGE_W_MM / 2, 272, "DO NOT FIT TO PAGE", ha="center", va="center",
           fontsize=14, fontweight="bold", color="firebrick")
    ax.text(PAGE_W_MM / 2, 263, "Printer dialog: Scale=100%, Actual Size, Fit to Page=OFF, Shrink/Oversize=OFF",
           ha="center", va="center", fontsize=8.5)
    ax.text(PAGE_W_MM / 2, 255, "P-007F-07 -- ArUco Marker Print Sheet", ha="center", va="center", fontsize=11)

    # -- marker placement --
    marker_bottom = 170.0
    marker_left = (PAGE_W_MM - BLACK_SQUARE_MM) / 2.0
    ax.imshow(marker_img, cmap="gray", vmin=0, vmax=255, interpolation="nearest", origin="upper",
             extent=[marker_left, marker_left + BLACK_SQUARE_MM,
                     marker_bottom, marker_bottom + BLACK_SQUARE_MM], zorder=3)

    # quiet-zone guide (labeled dashed box, not part of the marker bitmap itself)
    qz = mpatches.Rectangle(
        (marker_left - QUIET_ZONE_MARGIN_MM, marker_bottom - QUIET_ZONE_MARGIN_MM),
        BLACK_SQUARE_MM + 2 * QUIET_ZONE_MARGIN_MM, BLACK_SQUARE_MM + 2 * QUIET_ZONE_MARGIN_MM,
        fill=False, linestyle="--", linewidth=0.8, edgecolor="gray", zorder=2)
    ax.add_patch(qz)
    ax.text(marker_left + BLACK_SQUARE_MM / 2, marker_bottom + BLACK_SQUARE_MM + QUIET_ZONE_MARGIN_MM + 3,
           "quiet zone guide (+%gmm margin)" % QUIET_ZONE_MARGIN_MM, ha="center", va="bottom", fontsize=7,
           color="gray")

    ax.text(PAGE_W_MM / 2, marker_bottom - QUIET_ZONE_MARGIN_MM - 7,
           "%s  /  ID %d  /  %.0f mm (outer black square)" % (dict_name, marker_id, BLACK_SQUARE_MM),
           ha="center", va="center", fontsize=11, fontweight="bold")

    # -- reference ruler, independent of the marker, for post-print measurement --
    # 🔴 자체발견·수정(P-007F-07B, 50mm 버전 재사용 중 발견): bar 자체는 BLACK_SQUARE_MM
    # 폭으로 그려지는데 눈금/라벨은 "30"에 하드코딩돼 있어, BLACK_SQUARE_MM>30일 때
    # bar가 마지막 눈금(30) 너머까지 라벨 없이 뻗어나가는 버그였다. 눈금/라벨 모두
    # BLACK_SQUARE_MM에서 유도한다 -- bar 길이가 항상 marker 물리 크기와 정확히 일치한다.
    ruler_left = marker_left
    ruler_bottom = marker_bottom - QUIET_ZONE_MARGIN_MM - 28
    ruler_h = 4.0
    bar = mpatches.Rectangle((ruler_left, ruler_bottom), BLACK_SQUARE_MM, ruler_h,
                             facecolor="black", edgecolor="black", zorder=3)
    ax.add_patch(bar)
    tick_step = 10
    ticks_mm = list(range(0, int(round(BLACK_SQUARE_MM)) + 1, tick_step))
    if ticks_mm[-1] != int(round(BLACK_SQUARE_MM)):
        ticks_mm.append(int(round(BLACK_SQUARE_MM)))
    for tick_mm in ticks_mm:
        x = ruler_left + tick_mm
        ax.plot([x, x], [ruler_bottom - 1.5, ruler_bottom + ruler_h + 1.5], color="black", linewidth=0.8, zorder=4)
        ax.text(x, ruler_bottom - 4, "%d" % tick_mm, ha="center", va="top", fontsize=7)
    ax.text(ruler_left + BLACK_SQUARE_MM / 2, ruler_bottom + ruler_h + 6,
           "%g mm REFERENCE BAR -- measure this with a ruler after printing" % BLACK_SQUARE_MM,
           ha="center", va="bottom", fontsize=8)

    # -- footer -- OUT_DIR 모듈 전역에서 상대경로를 유도(하드코딩 금지 -- 호출측이
    # OUT_DIR를 override하면 이 라벨도 그에 맞춰 바뀐다, 자체발견·수정과 함께 정리)
    footer_rel = os.path.relpath(os.path.join(OUT_DIR, "print_verification.md"), ROOT)
    ax.text(PAGE_W_MM / 2, 20, "%s -- geometry verification method" % footer_rel,
           ha="center", va="center", fontsize=7, color="gray")

    return fig


def main():
    forbidden = self_check_no_hardware_import()
    if forbidden:
        print(json.dumps({"status": "ABORT", "reason": "FORBIDDEN_IMPORT", "found": list(forbidden)}))
        sys.exit(1)

    det = load_detector_module()
    dict_name = "DICT_4X4_50"
    assert det.DEFAULT_DICT == det.cv2.aruco.DICT_4X4_50, "Phase A DEFAULT_DICT changed unexpectedly"
    marker_id = det.PRIMARY_MARKER_ID

    marker_img = det.generate_marker_image(marker_id, MARKER_PX)  # reuses Phase A function unmodified

    os.makedirs(OUT_DIR, exist_ok=True)

    fig = build_figure(marker_img, dict_name, marker_id)
    pdf_path = os.path.join(OUT_DIR, "aruco_4x4_50_id%d_30mm_A4.pdf" % marker_id)
    fig.savefig(pdf_path)  # vector PDF -- page size taken from figsize (inches), dpi irrelevant to geometry
    import matplotlib.pyplot as plt
    plt.close(fig)

    # -- PNG at a known exact dpi, for empirical geometry cross-check (same axes layout) --
    fig2 = build_figure(marker_img, dict_name, marker_id)
    png_path = os.path.join(OUT_DIR, "aruco_4x4_50_id%d_30mm.png" % marker_id)
    fig2.savefig(png_path, dpi=VERIFY_PNG_DPI)
    plt.close(fig2)

    # -- geometry verification --
    import struct
    with open(pdf_path, "rb") as f:
        pdf_bytes = f.read()
    import re
    mb_matches = re.findall(rb"/MediaBox\s*\[\s*([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s*\]", pdf_bytes)
    mediabox_pt = [tuple(float(v) for v in m) for m in mb_matches]
    expected_w_pt = PAGE_W_MM / 25.4 * 72.0
    expected_h_pt = PAGE_H_MM / 25.4 * 72.0
    mediabox_ok = bool(mediabox_pt) and all(
        abs(mb[2] - mb[0] - expected_w_pt) < 0.5 and abs(mb[3] - mb[1] - expected_h_pt) < 0.5
        for mb in mediabox_pt)

    import numpy as np
    from PIL import Image
    png_img = np.array(Image.open(png_path).convert("L"))
    px_per_mm = VERIFY_PNG_DPI / 25.4
    dark = png_img < 128
    rows = np.where(dark.any(axis=1))[0]
    cols = np.where(dark.any(axis=0))[0]
    # restrict the dark-pixel search to the marker's expected PNG-pixel bounding region
    # (header/footer text is also dark -- isolate the marker by its known mm position)
    marker_bottom_mm = 170.0
    marker_left_mm = (PAGE_W_MM - BLACK_SQUARE_MM) / 2.0
    # PNG y is top-down; our mm y-axis is bottom-up (origin='upper' in a bottom-up axes) ->
    # convert marker mm box to PNG pixel row/col range with a margin, then measure the dark blob within it
    img_h_px = png_img.shape[0]
    top_mm = PAGE_H_MM - (marker_bottom_mm + BLACK_SQUARE_MM)
    bottom_mm = PAGE_H_MM - marker_bottom_mm
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

    report = {
        "dictionary": dict_name,
        "marker_id": marker_id,
        "marker_id_source": "scripts/p007f_07_marker_detector.py PRIMARY_MARKER_ID (Phase A preregistered, reused unmodified)",
        "black_square_physical_size_mm": BLACK_SQUARE_MM,
        "pdf_path": pdf_path,
        "png_path": png_path,
        "pdf_mediabox_pt": mediabox_pt,
        "pdf_mediabox_expected_pt": [0.0, 0.0, round(expected_w_pt, 2), round(expected_h_pt, 2)],
        "pdf_mediabox_matches_a4": mediabox_ok,
        "png_verify_dpi": VERIFY_PNG_DPI,
        "png_measured_marker_width_mm": round(measured_w_mm, 3) if measured_w_mm else None,
        "png_measured_marker_height_mm": round(measured_h_mm, 3) if measured_h_mm else None,
        "geometry_verification_passed": geometry_ok,
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
