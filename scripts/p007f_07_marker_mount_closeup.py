#!/usr/bin/env python3
"""P-007F-07 -- Arm2 Marker Mount Close-up (사용자 지시, 2026-10-29).

목적: 이미 확보된 marker_setup_capture 의 reference frame(재촬영 없음)을
이용해 Arm2 주변을 확대하고, 실제 marker를 직접 부착할 수 있는 rigid flat
surface 후보(M1/M2/M3)를 사람이 한눈에 이해할 수 있는 이미지로 제시한다.

ROBOT COMMANDS = 0, SERVO WRITES = 0, ROBOT/GEMINI MOVEMENT = 0,
MARKER ATTACHMENT = 0 -- 이 스크립트는 저장된 PNG/depth 파일만 읽는다.
카메라/로봇 API를 전혀 참조하지 않는다(AST 자기검증 통과 대상 -- 아래
self_check_no_hardware_import()).

후보 좌표(M1/M2/M3 bounding box)는 §격자 오버레이를 이용해 사람이(이번에는
Claude가) 1280x720 원본에서 직접 픽셀 단위로 반복 판독한 결과다 -- 자동
검출 알고리즘이 아니다. 깊이 추정은 color->depth 픽셀 근사 매핑(초점거리
비율만 사용, P-007F-06에서 공개된 RGB/Depth 23.8mm baseline parallax는
보정하지 않음 -- ESTIMATED로 명시)을 사용한다.
"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_COLOR = os.path.join(ROOT, "evidence", "P-007F-07", "marker_setup_capture",
                         "raw_frames", "marker_setup_capture_1793278747", "0004-color.png")
SRC_DEPTH = os.path.join(ROOT, "evidence", "P-007F-07", "marker_setup_capture",
                         "raw_frames", "marker_setup_capture_1793278747", "0004-depth.png")
SRC_JSON = os.path.join(ROOT, "evidence", "P-007F-07", "marker_setup_capture",
                        "raw_frames", "marker_setup_capture_1793278747", "0004.json")
OUT_DIR = os.path.join(ROOT, "evidence", "P-007F-07", "marker_mount_closeup")

CLOSEUP_BOX = (640, 80, 820, 300)  # x0,y0,x1,y1 in original 1280x720 coords
CLOSEUP_SCALE = 6

# Legacy Phase A candidate boxes (scripts/p007f_07_marker_setup_capture.py), kept for reference overlay only
PRIMARY_A_LEGACY = (655, 225, 70, 50)
PRIMARY_B_LEGACY = (650, 195, 90, 30)

ARM2_ROTATION_AXIS_X = 712  # vertical axis through visible disc center, grid-read
ARM3_BOUNDARY_Y = 155       # approx seam between disc/fork assembly and next joint body, grid-read

# M1/M2/M3 candidate boxes: (x, y, w, h) in original 1280x720 coords, grid-read
CANDIDATES = {
    "M1": {
        "box": (675, 255, 65, 30),
        "physical_part": "Joint1 rotating disc/horn top surface (established ARM2_ROI region, P-007F-03~06 witness)",
        "rigid_with_arm2": "YES",
        "rigid_with_arm2_note": "이 영역 자체가 ARM2_ROI로 정의된 구조물이다 -- 정의상 YES",
        "surface_shape": "FLAT",
        "sample_color_px": (707, 268),
    },
    "M2": {
        "box": (745, 95, 55, 55),
        "physical_part": "cream/white rectangular bracket panel above the shoulder-joint cylinder (DaBai 방향 상단 브래킷)",
        "rigid_with_arm2": "UNCERTAIN",
        "rigid_with_arm2_note": ("M1보다 최소 1개 이상 상위 joint seam(ARM3_BOUNDARY_Y 선) 너머에 있다 -- "
                                 "Arm2 단독 명령 실험(§single-motion, MAX_MOTION_COMMANDS=1)에서는 다른 servo가 "
                                 "움직이지 않으므로 witness로 유효하지만, DOFBOT 자체 joint 번호 체계와 이 "
                                 "프로젝트의 'Arm2'(servo_id=2) 대응관계는 P-007F-01 READBACK_SEMANTICS_UNRESOLVED "
                                 "로 미해결 상태라 구조적으로 Arm2 '자신의 몸체'인지는 단정하지 않는다."),
        "surface_shape": "FLAT",
        "sample_color_px": (772, 122),
    },
    "M3": {
        "box": (697, 123, 28, 17),
        "physical_part": "shoulder-joint cylindrical servo body의 라벨 스티커 부착면(밀링된 평평한 facet으로 추정)",
        "rigid_with_arm2": "UNCERTAIN",
        "rigid_with_arm2_note": "M2와 동일한 이유로 UNCERTAIN. 또한 몸체 자체는 원통형(CURVED)이라 marker 부착면은 스티커가 붙은 평평한 facet 부분으로 한정된다.",
        "surface_shape": "FLAT",
        "sample_color_px": (711, 131),
    },
}

MARKER_TEST_SIZES_MM = [20, 25, 30, 40, 50]


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


def load_cv2():
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    import importlib
    va = importlib.import_module("p007f_03_visual_analysis")
    return va.cv2, va.np


def color_to_depth_px(cx, cy, ci, di):
    cx_c, cy_c, fx_c, fy_c = ci["k"][2], ci["k"][5], ci["k"][0], ci["k"][4]
    cx_d, cy_d, fx_d, fy_d = di["k"][2], di["k"][5], di["k"][0], di["k"][4]
    dx = (cx - cx_c) * (fx_d / fx_c) + cx_d
    dy = (cy - cy_c) * (fy_d / fy_c) + cy_d
    return dx, dy, fx_c


def sample_local_depth_mm(cv2, np, depth, cx, cy, ci, di, win=10):
    dx, dy, fx_c = color_to_depth_px(cx, cy, ci, di)
    dx, dy = int(round(dx)), int(round(dy))
    x0, x1 = max(0, dx - win), min(depth.shape[1], dx + win + 1)
    y0, y1 = max(0, dy - win), min(depth.shape[0], dy + win + 1)
    patch = depth[y0:y1, x0:x1]
    valid = patch[patch > 0]
    if valid.size == 0:
        return None, 0, fx_c
    return float(np.median(valid)), int(valid.size), fx_c


def draw_crop_box_on_full(cv2, img):
    out = img.copy()
    x0, y0, x1, y1 = CLOSEUP_BOX
    cv2.rectangle(out, (x0, y0), (x1, y1), (0, 255, 255), 2)
    cv2.putText(out, "Arm2 close-up region (see 02_arm2_closeup.png)", (x0, max(20, y0 - 10)),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
    return out


def to_crop_px(x, y):
    x0, y0, _, _ = CLOSEUP_BOX
    return int((x - x0) * CLOSEUP_SCALE), int((y - y0) * CLOSEUP_SCALE)


def make_closeup_base(cv2, img):
    x0, y0, x1, y1 = CLOSEUP_BOX
    crop = img[y0:y1, x0:x1]
    return cv2.resize(crop, (crop.shape[1] * CLOSEUP_SCALE, crop.shape[0] * CLOSEUP_SCALE),
                      interpolation=cv2.INTER_CUBIC)


def draw_annotated_closeup(cv2, img):
    out = make_closeup_base(cv2, img)
    h = out.shape[0]

    ax_x, _ = to_crop_px(ARM2_ROTATION_AXIS_X, 0)
    cv2.line(out, (ax_x, 0), (ax_x, h), (0, 0, 255), 2)
    cv2.putText(out, "Arm2 rotation axis (vertical, through disc center)", (max(5, ax_x - 280), 24),
               cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)

    _, by = to_crop_px(0, ARM3_BOUNDARY_Y)
    cv2.line(out, (0, by), (out.shape[1], by), (255, 255, 0), 2)
    cv2.putText(out, "next-joint seam (approx Arm2/Arm3 boundary, anatomical numbering UNCERTAIN)",
               (10, max(20, by - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2, cv2.LINE_AA)

    for name, box, color in (("PRIMARY_A (legacy)", PRIMARY_A_LEGACY, (0, 255, 0)),
                             ("PRIMARY_B (legacy)", PRIMARY_B_LEGACY, (0, 180, 255))):
        x, y, w, hh = box
        p0 = to_crop_px(x, y)
        p1 = to_crop_px(x + w, y + hh)
        cv2.rectangle(out, p0, p1, color, 1)
        cv2.putText(out, name, (p0[0], max(15, p0[1] - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)

    cv2.putText(out, "Arm2 rigid link = everything below the yellow line (disc + fork), per ARM2_ROI convention",
               (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def draw_candidates(cv2, img):
    out = make_closeup_base(cv2, img)
    colors = {"M1": (0, 255, 0), "M2": (0, 165, 255), "M3": (255, 0, 255)}
    legend_h = 26 * len(CANDIDATES) + 44
    overlay = out.copy()
    cv2.rectangle(overlay, (6, out.shape[0] - legend_h - 6), (out.shape[1] - 6, out.shape[0] - 6), (15, 15, 15), -1)
    out = cv2.addWeighted(overlay, 0.75, out, 0.25, 0)

    cv2.putText(out, "DIRECT MARKER ATTACHMENT CANDIDATES -- CANDIDATE ONLY, not final", (10, 24),
               cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)

    base_y = out.shape[0] - legend_h + 18
    for i, (name, spec) in enumerate(CANDIDATES.items()):
        color = colors[name]
        x, y, w, hh = spec["box"]
        p0 = to_crop_px(x, y)
        p1 = to_crop_px(x + w, y + hh)
        cv2.rectangle(out, p0, p1, color, 3)
        cx, cy = (p0[0] + p1[0]) // 2, (p0[1] + p1[1]) // 2
        label_pt = (min(out.shape[1] - 260, cx + 40), max(40, p0[1] - 30 - 10 * i))
        cv2.arrowedLine(out, label_pt, (cx, cy), color, 2, tipLength=0.25)
        cv2.putText(out, name, (label_pt[0] - 10, label_pt[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

        ly = base_y + i * 26
        cv2.rectangle(out, (20, ly - 14), (40, ly + 2), color, -1)
        cv2.putText(out, "%s: %s (%s, rigid_with_arm2=%s)" % (
            name, spec["physical_part"], spec["surface_shape"], spec["rigid_with_arm2"]),
                   (48, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def draw_size_comparison(cv2, img, depth_info):
    out = draw_candidates(cv2, img)
    colors = {"M1": (0, 255, 0), "M2": (0, 165, 255), "M3": (255, 0, 255)}
    panel_w, panel_h = 260, 160
    for i, (name, spec) in enumerate(CANDIDATES.items()):
        color = colors[name]
        mm_per_px = depth_info[name]["mm_per_px"]
        px0, py0 = 10 + i * (panel_w + 10), 40
        overlay = out.copy()
        cv2.rectangle(overlay, (px0, py0), (px0 + panel_w, py0 + panel_h), (10, 10, 10), -1)
        out = cv2.addWeighted(overlay, 0.78, out, 0.22, 0)
        cv2.rectangle(out, (px0, py0), (px0 + panel_w, py0 + panel_h), color, 2)
        cv2.putText(out, "%s size fit (mm_per_px=%s)" % (
            name, ("%.3f" % mm_per_px) if mm_per_px else "UNKNOWN"),
                   (px0 + 8, py0 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        if mm_per_px:
            box_w_mm = spec["box"][2] * mm_per_px
            box_h_mm = spec["box"][3] * mm_per_px
            cv2.putText(out, "avail ~%.0fx%.0fmm (ESTIMATED)" % (box_w_mm, box_h_mm),
                       (px0 + 8, py0 + 34), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
            origin_x, origin_y = px0 + 20, py0 + panel_h - 14
            for mm in MARKER_TEST_SIZES_MM:
                side_px = int(mm / mm_per_px)
                fits = (mm <= box_w_mm) and (mm <= box_h_mm)
                c = (0, 255, 0) if fits else (0, 0, 255)
                cv2.rectangle(out, (origin_x, origin_y - side_px), (origin_x + side_px, origin_y), c, 1)
                cv2.putText(out, "%dmm" % mm, (origin_x + side_px + 3, origin_y - side_px + 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.32, c, 1, cv2.LINE_AA)
        else:
            cv2.putText(out, "depth UNKNOWN -- size fit not estimated", (px0 + 8, py0 + 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 255), 1, cv2.LINE_AA)
    return out


def main():
    forbidden = self_check_no_hardware_import()
    if forbidden:
        print(json.dumps({"status": "ABORT", "reason": "FORBIDDEN_IMPORT", "found": list(forbidden)}))
        sys.exit(1)

    cv2, np = load_cv2()
    img = cv2.imread(SRC_COLOR)
    depth = cv2.imread(SRC_DEPTH, cv2.IMREAD_UNCHANGED)
    meta = json.load(open(SRC_JSON, encoding="utf-8"))
    ci, di = meta["color_info"], meta["depth_info"]

    os.makedirs(OUT_DIR, exist_ok=True)

    cv2.imwrite(os.path.join(OUT_DIR, "01_arm2_original_context.png"), draw_crop_box_on_full(cv2, img))
    cv2.imwrite(os.path.join(OUT_DIR, "02_arm2_closeup.png"), draw_annotated_closeup(cv2, img))
    cv2.imwrite(os.path.join(OUT_DIR, "03_marker_surface_candidates.png"), draw_candidates(cv2, img))

    depth_info = {}
    for name, spec in CANDIDATES.items():
        cx, cy = spec["sample_color_px"]
        med_mm, n_valid, fx_c = sample_local_depth_mm(cv2, np, depth, cx, cy, ci, di, win=10)
        mm_per_px = (med_mm / fx_c) if med_mm else None
        depth_info[name] = {"sample_color_px": [cx, cy], "local_depth_mm_median": med_mm,
                            "local_depth_valid_samples": n_valid, "mm_per_px": mm_per_px}

    cv2.imwrite(os.path.join(OUT_DIR, "04_candidate_size_comparison.png"),
               draw_size_comparison(cv2, img, depth_info))

    report = {"source_frame": SRC_COLOR, "closeup_box_xyxy": CLOSEUP_BOX, "closeup_scale": CLOSEUP_SCALE,
              "arm2_rotation_axis_x": ARM2_ROTATION_AXIS_X, "arm3_boundary_y": ARM3_BOUNDARY_Y,
              "candidates": {}, "robot_command_count": 0, "servo_write_count": 0}
    for name, spec in CANDIDATES.items():
        di_ = depth_info[name]
        box_w_mm = di_["mm_per_px"] * spec["box"][2] if di_["mm_per_px"] else None
        box_h_mm = di_["mm_per_px"] * spec["box"][3] if di_["mm_per_px"] else None
        largest_fit = None
        if box_w_mm and box_h_mm:
            fits = [mm for mm in MARKER_TEST_SIZES_MM if mm <= box_w_mm and mm <= box_h_mm]
            largest_fit = max(fits) if fits else None
        report["candidates"][name] = {
            "box_xywh": spec["box"], "physical_part": spec["physical_part"],
            "rigid_with_arm2": spec["rigid_with_arm2"], "rigid_with_arm2_note": spec["rigid_with_arm2_note"],
            "surface_shape": spec["surface_shape"],
            "approx_available_width_mm": round(box_w_mm, 1) if box_w_mm else "UNKNOWN",
            "approx_available_height_mm": round(box_h_mm, 1) if box_h_mm else "UNKNOWN",
            "camera_view_angle": "steep downward oblique (Baseline B fixed workcell view, not corrected for foreshortening)",
            "expected_occlusion": ("cables/center hub visible in some frames" if name == "M1" else
                                   "minimal in this frame, not verified across full range of motion"),
            "direct_marker_attachment": "YES" if (box_w_mm and box_w_mm >= 20 and box_h_mm and box_h_mm >= 20) else
                                        ("UNKNOWN" if box_w_mm is None else "NO"),
            "largest_directly_attachable_marker_mm": largest_fit if largest_fit else "NONE_OF_TESTED_SIZES_FIT",
            "depth_sample": di_,
        }
    with open(os.path.join(OUT_DIR, "candidate_report.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=1, default=repr)

    print(json.dumps(report, ensure_ascii=False, indent=1, default=repr))


if __name__ == "__main__":
    main()
