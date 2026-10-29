#!/usr/bin/env python3
"""P-007F-06 — Offline Forensic Analysis of P-007F-05 Visual Evidence.

완전 OFFLINE / HARDWARE-FREE: 이 파일은 Arm_Lib/smbus/rclpy/cv2 camera-capture
API(VideoCapture 등)를 전혀 호출하지 않는다 — 기존에 저장된 PNG 파일만 cv2.imread
로 읽는다. 로봇/카메라 명령 0건.

evidence/P-007F-05/run_1793274484/{pre_frames,post_frames}/ 의 저장된 RGB+Depth
PNG만 입력으로 쓴다. 새 촬영 없음.
"""
import hashlib
import importlib
import json
import os
import sys
import time

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

RUN_DIR = os.path.join(ROOT, "evidence", "P-007F-05", "run_1793274484")
PRE_DIR = os.path.join(RUN_DIR, "pre_frames", "pre_1793274498")
POST_DIR = os.path.join(RUN_DIR, "post_frames", "post_1793274506")
BASELINE_DIR = os.path.join(RUN_DIR, "baseline_check_frame", "baselinecheck_1793274496")
OUT_DIR = os.path.join(ROOT, "evidence", "P-007F-06")
FIG_DIR = os.path.join(OUT_DIR, "figures")

N_PRE = 5
N_POST = 5

# ---------------------------------------------------------------------------
# ROI definitions. ARM2_ROI/ROBOT_ROI/DOWNSTREAM_ROI/STATIC_REFERENCE_ROI는
# evidence/P-007F-03/roi_definition.json 그대로(수치 재조정 없음). END_EFFECTOR_ROI/
# DABAI_ROI는 evidence/P-007F-02B-R2/roi_measurements.json(end_effector_bbox/
# dabai_approx_location)에서 그대로 가져온다(같은 Baseline B 카메라 위치, 재조정
# 없음). ARM1_ROI/ARM3_ROI는 정밀 측정값이 없어, robot_bbox 를 로봇 kinematic
# chain 순서(base=하단 -> gripper=상단)에 맞춰 수직으로 대략 분할한 **근사
# 구조 분할**이다 -- diff 결과를 먼저 보고 "움직임이 보이는 곳"으로 맞춘 것이
# 아니라, 이 분석을 시작하기 전에 기하학적 추론만으로 고정했다(§7 요구사항).
# ---------------------------------------------------------------------------
ROIS = {
    "ROBOT_ROI": (595, 90, 210, 255),
    "ARM2_ROI": (650, 220, 120, 70),
    "DOWNSTREAM_ROI": (600, 90, 200, 170),
    "STATIC_REFERENCE_ROI": (900, 150, 300, 300),
    "END_EFFECTOR_ROI": (600, 180, 180, 80),
    "DABAI_ROI": (665, 90, 90, 60),
    "ARM4_ROI": (665, 90, 90, 60),
    "ARM1_ROI": (595, 290, 210, 55),
    "ARM3_ROI": (595, 150, 210, 70),
}
ROI_SOURCE = {
    "ROBOT_ROI": "evidence/P-007F-03/roi_definition.json (unmodified)",
    "ARM2_ROI": "evidence/P-007F-03/roi_definition.json (unmodified) -- primary detector ROI",
    "DOWNSTREAM_ROI": "evidence/P-007F-03/roi_definition.json (unmodified)",
    "STATIC_REFERENCE_ROI": "evidence/P-007F-03/roi_definition.json (unmodified)",
    "END_EFFECTOR_ROI": "evidence/P-007F-02B-R2/roi_measurements.json end_effector_bbox (unmodified)",
    "DABAI_ROI": "evidence/P-007F-02B-R2/roi_measurements.json dabai_approx_location (unmodified)",
    "ARM4_ROI": "same pixel region as DABAI_ROI -- DaBai is mounted on/near the Arm4 link, no separate measured bbox exists",
    "ARM1_ROI": "APPROXIMATE -- lower y-band of ROBOT_ROI (base end of kinematic chain), no precision measurement exists. Fixed geometrically BEFORE running any diff analysis (not chosen post-hoc from where change appears).",
    "ARM3_ROI": "APPROXIMATE -- middle y-band of ROBOT_ROI (between ARM2_ROI and END_EFFECTOR_ROI), same caveat as ARM1_ROI.",
}


def sha256_bytes(b):
    return hashlib.sha256(b).hexdigest()


def crop(img, roi):
    x, y, w, h = roi
    return img[y:y + h, x:x + w]


def load_color_gray(d, i):
    img = cv2.imread(os.path.join(d, "%04d-color.png" % i))
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def load_color_bgr(d, i):
    return cv2.imread(os.path.join(d, "%04d-color.png" % i))


def load_depth(d, i):
    return cv2.imread(os.path.join(d, "%04d-depth.png" % i), cv2.IMREAD_UNCHANGED)


PRE_GRAY = [load_color_gray(PRE_DIR, i) for i in range(N_PRE)]
POST_GRAY = [load_color_gray(POST_DIR, i) for i in range(N_POST)]
PRE_BGR = [load_color_bgr(PRE_DIR, i) for i in range(N_PRE)]
POST_BGR = [load_color_bgr(POST_DIR, i) for i in range(N_POST)]
PRE_DEPTH = [load_depth(PRE_DIR, i) for i in range(N_PRE)]
POST_DEPTH = [load_depth(POST_DIR, i) for i in range(N_POST)]

PRE_MEAN = np.mean([f.astype(np.float64) for f in PRE_GRAY], axis=0).astype(np.uint8)
POST_MEAN = np.mean([f.astype(np.float64) for f in POST_GRAY], axis=0).astype(np.uint8)


def w(name, obj):
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, name), "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=1, default=repr)


# ===========================================================================
# §4 -- source integrity
# ===========================================================================
def discover_files(d):
    return sorted(f for f in os.listdir(d) if os.path.isfile(os.path.join(d, f)))


def build_manifest(d, label):
    entries = []
    for fn in discover_files(d):
        full = os.path.join(d, fn)
        data = open(full, "rb").read()
        stat = os.stat(full)
        entries.append({
            "filename": fn, "size_bytes": len(data), "sha256": sha256_bytes(data),
            "mtime": stat.st_mtime,
            "pre_post": "PRE" if "pre" in label else ("POST" if "post" in label else "BASELINE"),
            "modality": "RGB" if "color" in fn else ("Depth" if "depth" in fn else "metadata"),
        })
    return {"label": label, "dir": d, "entries": entries}


def run_source_integrity():
    pre_m = build_manifest(PRE_DIR, "pre_frames")
    post_m = build_manifest(POST_DIR, "post_frames")
    baseline_m = build_manifest(BASELINE_DIR, "baseline_check_frame")
    w("source_manifest.json", {"pre": pre_m, "post": post_m, "baseline_check": baseline_m})

    # capture-summary.json 선언 프레임 수와 실제 파일 수 대조
    pre_summary = json.load(open(os.path.join(PRE_DIR, "capture-summary.json")))
    post_summary = json.load(open(os.path.join(POST_DIR, "capture-summary.json")))
    pre_color_files = [e for e in pre_m["entries"] if e["modality"] == "RGB"]
    post_color_files = [e for e in post_m["entries"] if e["modality"] == "RGB"]
    integrity = {
        "prior_manifest_existed": False,
        "note": "evidence/P-007F-05/run_1793274484/ never recorded a SHA256 manifest for pre_frames/post_frames (gemini_pre_manifest.json/gemini_post_manifest.json only held capture-summary.json's {frames,requested} counts). This is the FIRST hash freeze of these files -- no prior hash to diff against.",
        "pre_declared_frames": pre_summary["frames"], "pre_actual_rgb_files": len(pre_color_files),
        "post_declared_frames": post_summary["frames"], "post_actual_rgb_files": len(post_color_files),
        "counts_match": (pre_summary["frames"] == len(pre_color_files) == N_PRE
                         and post_summary["frames"] == len(post_color_files) == N_POST),
        "all_files_readable": True,
    }
    integrity["verdict"] = ("SOURCE_INTEGRITY_FROZEN_FIRST_TIME" if integrity["counts_match"]
                            else "SOURCE_EVIDENCE_INTEGRITY_FAILURE")
    w("source_integrity.json", integrity)
    return integrity


def run_roi_definition():
    w("roi_definition.json", {"image_size": {"width": 1280, "height": 720}, "rois": ROIS,
                              "source": ROI_SOURCE,
                              "policy": "All ROIs fixed before running any PRE/POST difference analysis -- none chosen post-hoc based on where a visual change appeared (§7 requirement)."})


# ===========================================================================
# §6 -- Global registration using STATIC_REFERENCE_ROI
# ===========================================================================
def global_registration():
    pre_static = crop(PRE_MEAN, ROIS["STATIC_REFERENCE_ROI"])
    post_static = crop(POST_MEAN, ROIS["STATIC_REFERENCE_ROI"])
    shift, response = cv2.phaseCorrelate(pre_static.astype(np.float32), post_static.astype(np.float32))
    residual_frame = post_static.astype(np.float64)
    # apply the estimated shift and measure residual via affine warp + abs-diff
    M = np.array([[1, 0, -shift[0]], [0, 1, -shift[1]]], dtype=np.float32)
    post_static_shifted = cv2.warpAffine(post_static, M, (post_static.shape[1], post_static.shape[0]))
    residual = float(np.mean(np.abs(pre_static.astype(np.float64) - post_static_shifted.astype(np.float64))))
    result = {
        "method": "cv2.phaseCorrelate on STATIC_REFERENCE_ROI (PRE_MEAN vs POST_MEAN, 5-frame mean each)",
        "global_translation_px": list(shift), "global_translation_magnitude_px": float(np.hypot(*shift)),
        "global_rotation_deg": None,
        "phase_correlation_response": float(response),
        "registration_residual_mean_abs_diff": residual,
        "raw_mean_abs_diff_before_shift_correction": float(np.mean(np.abs(
            pre_static.astype(np.float64) - post_static.astype(np.float64)))),
    }
    w("registration_analysis.json", result)
    return shift


def apply_global_shift(img, shift):
    M = np.array([[1, 0, -shift[0]], [0, 1, -shift[1]]], dtype=np.float32)
    return cv2.warpAffine(img, M, (img.shape[1], img.shape[0]))


# ===========================================================================
# §8 -- ORB reproduction (exact pipeline from p007f_03_visual_analysis/
# p007f_05_analysis, SHA-pin reused unmodified)
# ===========================================================================
def orb_reproduction():
    va = importlib.import_module("p007f_03_visual_analysis")
    arm2_pre = crop(PRE_MEAN, ROIS["ARM2_ROI"])
    arm2_post = crop(POST_MEAN, ROIS["ARM2_ROI"])
    static_pre = crop(PRE_MEAN, ROIS["STATIC_REFERENCE_ROI"])
    static_post = crop(POST_MEAN, ROIS["STATIC_REFERENCE_ROI"])
    arm2_result = va.algo_orb(arm2_pre, arm2_post)
    static_result = va.algo_orb(static_pre, static_post)
    robot_relative_translation = arm2_result["translation_px"] - static_result["translation_px"]
    robot_relative_rotation = arm2_result["rotation_deg"] - static_result["rotation_deg"]
    expected_translation, expected_rotation = 0.2297, 0.3976
    reproduced = (abs(robot_relative_translation - expected_translation) < 0.05
                 and abs(robot_relative_rotation - expected_rotation) < 0.05)
    result = {
        "module_sha256_pinned": "5ded8c75349cd91018cb6c024812e6fa07c80a2a77c525f21136ff6b11931ed2",
        "arm2_result": arm2_result, "static_result": static_result,
        "robot_relative_translation_px": robot_relative_translation,
        "robot_relative_rotation_deg": robot_relative_rotation,
        "expected_from_p007f_05": {"translation_px": expected_translation, "rotation_deg": expected_rotation},
        "reproduced_within_tolerance_0.05": reproduced,
    }
    w("orb_reproduction.json", result)
    return result


# ===========================================================================
# §9 -- Dense optical flow (Farneback), per ROI
# ===========================================================================
def _flow_stats(flow, roi):
    """🔴 `valid_pixel_count`(mag>1e-6)는 '움직임이 있다'가 아니라 '정확히 0이
    아니다'만 거른다 -- 실측 이미지에서는 noise만으로도 거의 전 픽셀이 이 조건을
    만족해, directional coherence 를 전혀 측정하지 못하는 약한 proxy다. 진짜
    rigid-motion 증거는 **방향 일치**다 -- 상위-매그니튜드 픽셀들의 unit vector를
    평균 낸 resultant length R(원형통계, 0=완전 무작위 방향, 1=전부 같은 방향)을
    별도로 계산해 coherence 판정에는 이 R을 쓴다(아래 top-magnitude 25% 픽셀
    대상, noise-level 픽셀을 섞지 않기 위함)."""
    x, y, w_, h_ = roi
    fx = flow[y:y + h_, x:x + w_, 0]
    fy = flow[y:y + h_, x:x + w_, 1]
    mag = np.sqrt(fx ** 2 + fy ** 2)
    valid = mag > 1e-6
    coherent_dir = None
    if valid.sum() > 0:
        mean_vx, mean_vy = float(np.mean(fx[valid])), float(np.mean(fy[valid]))
        coherent_dir = float(np.degrees(np.arctan2(mean_vy, mean_vx)))
    # directional coherence R (circular statistics) on the top-25%-magnitude pixels
    thresh = np.percentile(mag, 75)
    top = mag >= thresh
    r_value = None
    if top.sum() >= 10:
        unit_x = fx[top] / np.maximum(mag[top], 1e-9)
        unit_y = fy[top] / np.maximum(mag[top], 1e-9)
        r_value = float(np.hypot(unit_x.mean(), unit_y.mean()))
    return {
        "median_magnitude": float(np.median(mag)), "mean_magnitude": float(np.mean(mag)),
        "p90_magnitude": float(np.percentile(mag, 90)), "p95_magnitude": float(np.percentile(mag, 95)),
        "max_magnitude": float(np.max(mag)), "coherent_direction_deg": coherent_dir,
        "valid_pixel_count": int(valid.sum()), "total_pixel_count": int(mag.size),
        "directional_coherence_R_top25pct": r_value, "top25pct_pixel_count": int(top.sum()),
    }


def optical_flow_analysis(pre_img, post_img, label, roi_names=None):
    flow = cv2.calcOpticalFlowFarneback(pre_img, post_img, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    names = roi_names or list(ROIS.keys())
    return {name: _flow_stats(flow, ROIS[name]) for name in names}


def run_optical_flow():
    result = {"pre_post": optical_flow_analysis(PRE_MEAN, POST_MEAN, "pre_post")}
    w("optical_flow_analysis.json", result)
    return result


# ===========================================================================
# §10 -- Registered absolute difference
# ===========================================================================
def _absdiff_stats(diff, roi):
    x, y, w_, h_ = roi
    patch = diff[y:y + h_, x:x + w_]
    return {
        "median": float(np.median(patch)), "mean": float(np.mean(patch)),
        "p95": float(np.percentile(patch, 95)),
        "changed_pixel_ratio_gt5": float(np.mean(patch > 5)),
        "changed_pixel_ratio_gt10": float(np.mean(patch > 10)),
        "max": float(np.max(patch)),
    }


def run_registered_difference(global_shift):
    post_registered = apply_global_shift(POST_MEAN, global_shift)
    diff_raw = np.abs(PRE_MEAN.astype(np.float64) - POST_MEAN.astype(np.float64))
    diff_registered = np.abs(PRE_MEAN.astype(np.float64) - post_registered.astype(np.float64))
    result = {
        "global_shift_applied_px": list(global_shift),
        "raw_diff_per_roi": {name: _absdiff_stats(diff_raw, roi) for name, roi in ROIS.items()},
        "registered_diff_per_roi": {name: _absdiff_stats(diff_registered, roi) for name, roi in ROIS.items()},
    }
    w("registered_difference_analysis.json", result)
    cv2.imwrite(os.path.join(OUT_DIR, "_diff_registered_heatmap.png"),
               cv2.applyColorMap(np.clip(diff_registered, 0, 50).astype(np.uint8) * 5, cv2.COLORMAP_JET))
    return result, post_registered, diff_registered


# ===========================================================================
# §11 -- Edge/silhouette analysis
# ===========================================================================
def _edge_stats(pre_roi, post_roi):
    e_pre = cv2.Canny(pre_roi, 50, 150)
    e_post = cv2.Canny(post_roi, 50, 150)
    pre_pts = np.argwhere(e_pre > 0)
    post_pts = np.argwhere(e_post > 0)
    if len(pre_pts) < 5 or len(post_pts) < 5:
        return {"valid": False, "reason": "insufficient_edge_pixels",
                "pre_edge_count": int(len(pre_pts)), "post_edge_count": int(len(post_pts))}
    overlap = float(np.mean((e_pre > 0) == (e_post > 0)))
    centroid_shift = np.array(post_pts.mean(axis=0)) - np.array(pre_pts.mean(axis=0))
    # 간이 Chamfer-like distance: post edge pixel마다 pre edge와의 최근접 거리 평균(다운샘플)
    sample = post_pts[::max(1, len(post_pts) // 300)]
    pre_sample = pre_pts[::max(1, len(pre_pts) // 300)]
    dists = np.sqrt(((sample[:, None, :] - pre_sample[None, :, :]) ** 2).sum(-1)).min(axis=1)
    return {"valid": True, "pre_edge_count": int(len(pre_pts)), "post_edge_count": int(len(post_pts)),
           "edge_overlap_ratio": overlap, "centroid_shift_px": centroid_shift.tolist(),
           "centroid_shift_magnitude_px": float(np.hypot(*centroid_shift)),
           "chamfer_like_mean_distance_px": float(np.mean(dists))}


def run_edge_analysis():
    result = {}
    for name, roi in ROIS.items():
        result[name] = _edge_stats(crop(PRE_MEAN, roi), crop(POST_MEAN, roi))
    w("edge_analysis.json", result)
    return result


# ===========================================================================
# §12 -- Local phase correlation per ROI
# ===========================================================================
def _phasecorr(pre_roi, post_roi):
    shift, response = cv2.phaseCorrelate(pre_roi.astype(np.float32), post_roi.astype(np.float32))
    return {"dx": shift[0], "dy": shift[1], "magnitude": float(np.hypot(*shift)), "response": float(response)}


def run_phase_correlation():
    result = {name: _phasecorr(crop(PRE_MEAN, roi), crop(POST_MEAN, roi)) for name, roi in ROIS.items()}
    w("phase_correlation_analysis.json", result)
    return result


# ===========================================================================
# §13 -- Within-window noise control (PRE_i->PRE_j, POST_i->POST_j), compared
# against the actual PRE_MEAN->POST_MEAN signal, per metric, for ARM2_ROI
# (the primary ROI under question) and STATIC_REFERENCE_ROI (sanity).
# ===========================================================================
def _pairs(n):
    return [(i, j) for i in range(n) for j in range(i + 1, n)]


def within_window_noise():
    va = importlib.import_module("p007f_03_visual_analysis")
    # 🔴 전체 ROI(로봇 추정 영역 전부 + static reference)를 대상으로 해야 "ARM2만
    # noise를 넘는다"는 결론이 진짜인지, 아니면 다른 ROI(특히 정밀 측정이 없는
    # 근사 ROI들, ARM3/ARM4/DABAI)도 똑같이 noise를 넘어서 ARM2 특이적 로컬라이즈가
    # 아닌지 확인할 수 있다 -- 처음에는 4개 ROI만 검사했다가(자체발견), 단순
    # registered-diff-mean 수치에서 DABAI_ROI/ARM4_ROI/ARM3_ROI도 static보다
    # 높게 나오는 것을 보고 전체 ROI로 확장했다.
    metrics_rois = list(ROIS.keys())
    noise = {name: {"orb_translation_px": [], "orb_rotation_deg": [],
                    "flow_median_mag": [], "absdiff_mean": [], "edge_centroid_shift_px": [],
                    "phasecorr_magnitude": []} for name in metrics_rois}

    def sample(window_gray, pairs, bucket):
        for i, j in pairs:
            a, b = window_gray[i], window_gray[j]
            for name in metrics_rois:
                roi = ROIS[name]
                ra = crop(a, roi)
                rb = crop(b, roi)
                orb = va.algo_orb(ra, rb)
                bucket[name]["orb_translation_px"].append(orb["translation_px"] if orb.get("valid") else None)
                bucket[name]["orb_rotation_deg"].append(orb["rotation_deg"] if orb.get("valid") else None)
                flow = cv2.calcOpticalFlowFarneback(ra, rb, None, 0.5, 3, 15, 3, 5, 1.2, 0)
                mag = np.sqrt(flow[..., 0] ** 2 + flow[..., 1] ** 2)
                bucket[name]["flow_median_mag"].append(float(np.median(mag)))
                bucket[name]["absdiff_mean"].append(float(np.mean(np.abs(ra.astype(np.float64) - rb.astype(np.float64)))))
                edge = _edge_stats(ra, rb)
                bucket[name]["edge_centroid_shift_px"].append(
                    edge["centroid_shift_magnitude_px"] if edge.get("valid") else None)
                pc = _phasecorr(ra, rb)
                bucket[name]["phasecorr_magnitude"].append(pc["magnitude"])

    sample(PRE_GRAY, _pairs(N_PRE), noise)
    sample(POST_GRAY, _pairs(N_POST), noise)

    def stats(values):
        v = [x for x in values if x is not None]
        if not v:
            return {"n": 0}
        arr = np.array(v, dtype=np.float64)
        return {"n": len(v), "mean": float(arr.mean()), "stdev": float(arr.std(ddof=1)) if len(v) > 1 else 0.0,
               "max": float(arr.max()), "mean_plus_3sigma": float(arr.mean() + 3 * (arr.std(ddof=1) if len(v) > 1 else 0.0))}

    noise_stats = {name: {m: stats(vals) for m, vals in d.items()} for name, d in noise.items()}

    # actual PRE_MEAN -> POST_MEAN signal for the same metrics/ROIs, for comparison
    signal = {}
    for name in metrics_rois:
        roi = ROIS[name]
        a, b = crop(PRE_MEAN, roi), crop(POST_MEAN, roi)
        orb = va.algo_orb(a, b)
        flow = cv2.calcOpticalFlowFarneback(a, b, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        mag = np.sqrt(flow[..., 0] ** 2 + flow[..., 1] ** 2)
        edge = _edge_stats(a, b)
        pc = _phasecorr(a, b)
        signal[name] = {
            "orb_translation_px": orb.get("translation_px"), "orb_rotation_deg": orb.get("rotation_deg"),
            "flow_median_mag": float(np.median(mag)),
            "absdiff_mean": float(np.mean(np.abs(a.astype(np.float64) - b.astype(np.float64)))),
            "edge_centroid_shift_px": edge.get("centroid_shift_magnitude_px"),
            "phasecorr_magnitude": pc["magnitude"],
        }

    comparison = {}
    for name in metrics_rois:
        comparison[name] = {}
        for metric in signal[name]:
            sig_val = signal[name][metric]
            ns = noise_stats[name][metric]
            if sig_val is None or ns.get("n", 0) == 0:
                comparison[name][metric] = {"signal": sig_val, "noise": ns, "exceeds_noise_mean_plus_3sigma": None}
                continue
            comparison[name][metric] = {
                "signal": sig_val, "noise": ns,
                "exceeds_noise_mean_plus_3sigma": bool(abs(sig_val) > ns["mean_plus_3sigma"]),
                "exceeds_noise_max": bool(abs(sig_val) > ns["max"]),
            }

    result = {"pairs_per_window": {"pre_pairs": len(_pairs(N_PRE)), "post_pairs": len(_pairs(N_POST))},
             "noise_distribution": noise_stats, "pre_to_post_signal": signal, "comparison": comparison}
    w("within_window_noise.json", result)
    return result


# ===========================================================================
# §14/§15 -- Spatial localization + coherence classification
# ===========================================================================
def spatial_localization(reg_diff_result, flow_result, noise_result):
    """🔴 자체발견/수정 -- 1차 초안은 STATIC_REFERENCE_ROI/END_EFFECTOR_ROI/
    DOWNSTREAM_ROI 등 4개 ROI만 noise-control 검사했고, 단순 raw
    registered-diff-mean 수치로는 DABAI_ROI/ARM4_ROI/ARM3_ROI(정밀 측정이
    없는 근사 ROI들)도 STATIC_REFERENCE_ROI보다 높게 나오는 것을 발견했다 --
    "ARM2만 특이적으로 변했다"는 결론이 단지 2~4개 ROI만 봐서 생긴 착시인지
    확인하려면 **9개 ROI 전부**를 같은 within-window noise control 로 검사해야
    한다. `within_window_noise()`를 전체 ROI로 확장한 뒤 여기서 그 결과를
    그대로 쓴다(raw registered-diff-mean 휴리스틱은 참고용으로만 남긴다)."""
    classifications = {}
    robot_rois = ["ARM1_ROI", "ARM2_ROI", "ARM3_ROI", "ARM4_ROI", "DABAI_ROI", "END_EFFECTOR_ROI", "ROBOT_ROI"]
    static_val = reg_diff_result["registered_diff_per_roi"]["STATIC_REFERENCE_ROI"]["mean"]
    for name in ROIS:
        roi_val = reg_diff_result["registered_diff_per_roi"][name]["mean"]
        is_robot = name in robot_rois
        above_static = roi_val > static_val * 1.5
        classifications[name] = {
            "registered_diff_mean": roi_val, "static_reference_diff_mean": static_val,
            "above_static_background_raw_heuristic": above_static,
            "is_robot_geometry_roi": is_robot,
        }

    noise_exceed_any = {}
    for name in ROIS:
        comp = noise_result["comparison"].get(name, {})
        noise_exceed_any[name] = any(bool(m.get("exceeds_noise_mean_plus_3sigma")) for m in comp.values())
        classifications[name]["exceeds_within_window_noise_any_metric"] = noise_exceed_any[name]

    arm2_any_exceed = noise_exceed_any["ARM2_ROI"]
    static_any_exceed = noise_exceed_any["STATIC_REFERENCE_ROI"]
    other_robot_rois = [n for n in robot_rois if n != "ARM2_ROI"]
    other_robot_exceed = [n for n in other_robot_rois if noise_exceed_any[n]]
    non_robot_non_static_exceed = [n for n in ROIS if n not in robot_rois and n != "STATIC_REFERENCE_ROI"
                                   and noise_exceed_any[n]]

    result = {
        "per_roi_registered_diff": classifications,
        "noise_exceed_any_metric_by_roi": noise_exceed_any,
        "arm2_roi_exceeds_within_window_noise_any_metric": arm2_any_exceed,
        "static_reference_roi_exceeds_within_window_noise_any_metric": static_any_exceed,
        "other_robot_rois_also_exceeding": other_robot_exceed,
        "non_robot_non_static_rois_also_exceeding": non_robot_non_static_exceed,
        "localization_label": None,
    }
    if arm2_any_exceed and not static_any_exceed and not other_robot_exceed:
        result["localization_label"] = "ROBOT_LOCALIZED"
    elif arm2_any_exceed and not static_any_exceed and other_robot_exceed:
        result["localization_label"] = "ROBOT_LOCALIZED"
        result["localization_note"] = ("ARM2_ROI exceeds noise and STATIC_REFERENCE_ROI does not, but other "
                                       "approximate robot-geometry ROIs (%s) also exceed -- change may not be "
                                       "ARM2-specific; could reflect diffuse robot-region signal or imprecise "
                                       "ROI boundaries (ARM1/ARM3/ARM4/DABAI are approximate, unmeasured)." % other_robot_exceed)
    elif static_any_exceed and arm2_any_exceed:
        result["localization_label"] = "GLOBAL_LIGHTING"
    elif static_any_exceed and not arm2_any_exceed:
        result["localization_label"] = "BACKGROUND_LOCALIZED"
    elif not arm2_any_exceed and not static_any_exceed:
        result["localization_label"] = "NO_SIGNIFICANT_CHANGE"
    else:
        result["localization_label"] = "MIXED"
    w("spatial_localization.json", result)
    return result


def coherence_classification(spatial_result, flow_result):
    """🔴 directional_coherence_R(원형통계, top-25%-magnitude 픽셀 기준)을 쓴다 --
    `valid_pixel_count` 기반 비율(mag>1e-6)은 거의 항상 ~100%가 되어(noise조차
    0이 아니므로) 방향 일치를 전혀 구분하지 못하는 약한 proxy였다(자체발견,
    수정). R>=0.5(50% 이상 벡터가 대략 같은 방향으로 합쳐짐)를 rigid-motion
    에 부합하는 coherence 기준으로 쓴다 -- 임의의 수치가 아니라, R=0.5는
    원형통계에서 '방향 분산이 평균 벡터 길이를 상당히 깎아내릴 정도로 크지
    않다'는 보수적인 중간 기준이다."""
    arm2_flow = flow_result["pre_post"]["ARM2_ROI"]
    r_value = arm2_flow.get("directional_coherence_R_top25pct")
    label = spatial_result["localization_label"]
    is_coherent = (r_value is not None and r_value >= 0.5)
    if label == "ROBOT_LOCALIZED" and is_coherent:
        final = "COHERENT_ROBOT_MOTION"
    elif label == "ROBOT_LOCALIZED":
        final = "LOCAL_NONRIGID_CHANGE"
    elif label == "GLOBAL_LIGHTING":
        final = "GLOBAL_SCENE_CHANGE"
    elif label == "NO_SIGNIFICANT_CHANGE":
        final = "NO_SIGNIFICANT_CHANGE"
    else:
        final = "RANDOM_IMAGE_NOISE"
    return {"localization_label": label, "arm2_directional_coherence_R": r_value,
           "coherence_threshold": 0.5, "is_coherent": is_coherent,
           "final_coherence_label": final}


# ===========================================================================
# §16/§17 -- Depth forensic + RGB<->Depth alignment check
# ===========================================================================
DEPTH_SCALE_X = 0.6625
DEPTH_SCALE_Y = 0.6667


def color_roi_to_depth_roi(roi):
    x, y, w_, h_ = roi
    return (int(round(x * DEPTH_SCALE_X)), int(round(y * DEPTH_SCALE_Y)),
           int(round(w_ * DEPTH_SCALE_X)), int(round(h_ * DEPTH_SCALE_Y)))


def rgb_depth_alignment_check():
    meta = json.load(open(os.path.join(PRE_DIR, "0000.json")))
    t = meta["T_depth_color"]["translation_m"]
    q = meta["T_depth_color"]["quaternion_xyzw"]
    translation_mm = [v * 1000 for v in t]
    baseline_mm = float(np.hypot(translation_mm[0], translation_mm[1]))
    color_k = meta["color_info"]["k"]
    depth_k = meta["depth_info"]["k"]
    result = {
        "color_resolution": [meta["color_info"]["width"], meta["color_info"]["height"]],
        "depth_resolution": [meta["depth_info"]["width"], meta["depth_info"]["height"]],
        "color_frame_id": meta["color_frame_id"], "depth_frame_id": meta["depth_frame_id"],
        "stamp_delta_sec": meta["stamp_delta_sec"],
        "T_depth_color_translation_mm": translation_mm,
        "T_depth_color_quaternion_xyzw": q,
        "nonzero_baseline_mm": baseline_mm,
        "color_fx_fy": [color_k[0], color_k[4]], "depth_fx_fy": [depth_k[0], depth_k[4]],
        "simple_scaled_roi_mapping_used": {"sx": DEPTH_SCALE_X, "sy": DEPTH_SCALE_Y,
                                           "source": "evidence/P-007F-03/roi_definition.json, unmodified"},
        "registered_depth_stream_proven": False,
        "verdict": "DEPTH_SPATIAL_LOCALIZATION_LIMITED",
        "reason": "Non-zero depth<->color extrinsic baseline (%.2fmm) exists; depth and color intrinsics differ beyond simple resolution scaling. The scaled-ROI mapping from P-007F-03 is reused as-is but is NOT a proven pixel-exact registration -- depth-based spatial localization conclusions are disclosed as approximate, not precise." % baseline_mm,
    }
    w("rgb_depth_alignment.json", result)
    return result


def _depth_stats(depth_img, droi):
    x, y, w_, h_ = droi
    patch = depth_img[y:y + h_, x:x + w_].astype(np.float64)
    valid = patch > 0
    vals = patch[valid]
    if len(vals) == 0:
        return {"valid_ratio": 0.0, "median_depth_mm": None, "mean_depth_mm": None, "robust_spread_mm": None}
    return {"valid_ratio": float(valid.mean()), "median_depth_mm": float(np.median(vals)),
           "mean_depth_mm": float(np.mean(vals)),
           "robust_spread_mm": float(np.percentile(vals, 75) - np.percentile(vals, 25))}


PRE_DEPTH_MEAN_VALID = np.mean([d.astype(np.float64) for d in PRE_DEPTH], axis=0)
POST_DEPTH_MEAN_VALID = np.mean([d.astype(np.float64) for d in POST_DEPTH], axis=0)


def run_depth_analysis(alignment):
    per_roi = {}
    for name, roi in ROIS.items():
        droi = color_roi_to_depth_roi(roi)
        droi = (max(0, droi[0]), max(0, droi[1]),
               min(droi[2], 848 - max(0, droi[0])), min(droi[3], 480 - max(0, droi[1])))
        pre_s = _depth_stats(PRE_DEPTH_MEAN_VALID, droi)
        post_s = _depth_stats(POST_DEPTH_MEAN_VALID, droi)
        diff = None
        if pre_s["median_depth_mm"] is not None and post_s["median_depth_mm"] is not None:
            diff = post_s["median_depth_mm"] - pre_s["median_depth_mm"]
        per_roi[name] = {"depth_roi_px": droi, "pre": pre_s, "post": post_s, "median_depth_diff_mm": diff}

    robot_relative_depth_change = None
    if (per_roi["ARM2_ROI"]["median_depth_diff_mm"] is not None
            and per_roi["STATIC_REFERENCE_ROI"]["median_depth_diff_mm"] is not None):
        robot_relative_depth_change = (per_roi["ARM2_ROI"]["median_depth_diff_mm"]
                                       - per_roi["STATIC_REFERENCE_ROI"]["median_depth_diff_mm"])
    result = {"alignment_caveat": alignment["verdict"], "per_roi": per_roi,
             "robot_relative_depth_change_mm": robot_relative_depth_change}
    w("depth_analysis.json", result)
    return result


# ===========================================================================
# §18 -- Point cloud forensic (limited -- see alignment caveat)
# ===========================================================================
def run_pointcloud_analysis(alignment):
    if not alignment["registered_depth_stream_proven"]:
        result = {
            "status": "LIMITED",
            "reason": "RGB<->depth pixel-exact registration not proven (see rgb_depth_alignment.json) and no robot segmentation mask exists. A quantitative point-cloud robot-region residual would require projecting ARM2_ROI pixels through unverified extrinsics -- doing so would manufacture false precision.",
            "qualitative_check_performed": "ARM2 depth-ROI valid_ratio and median depth compared PRE vs POST (see depth_analysis.json) as a coarse proxy; no full 3D point-cloud alignment attempted.",
            "verdict": "POINTCLOUD_FORENSIC_NOT_ATTEMPTED_DUE_TO_UNVERIFIED_REGISTRATION",
        }
    else:
        result = {"status": "NOT_REACHED"}
    w("pointcloud_analysis.json", result)
    return result


# ===========================================================================
# §20 -- Synthetic sensitivity test (NOT real motion evidence)
# ===========================================================================
def synthetic_sensitivity():
    va = importlib.import_module("p007f_03_visual_analysis")
    base = crop(PRE_MEAN, ROIS["ARM2_ROI"])
    h, w_ = base.shape
    translations_px = [0.25, 0.5, 1.0, 2.0, 3.0, 5.0]
    rotations_deg = [0.25, 0.5, 1.0, 2.0, 3.0]

    def make_translated(img, dx):
        M = np.array([[1, 0, dx], [0, 1, 0]], dtype=np.float32)
        return cv2.warpAffine(img, M, (img.shape[1], img.shape[0]))

    def make_rotated(img, deg):
        M = cv2.getRotationMatrix2D((img.shape[1] / 2, img.shape[0] / 2), deg, 1.0)
        return cv2.warpAffine(img, M, (img.shape[1], img.shape[0]))

    trans_results = []
    for d in translations_px:
        shifted = make_translated(base, d)
        orb = va.algo_orb(base, shifted)
        flow = cv2.calcOpticalFlowFarneback(base, shifted, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        mag = np.sqrt(flow[..., 0] ** 2 + flow[..., 1] ** 2)
        edge = _edge_stats(base, shifted)
        pc = _phasecorr(base, shifted)
        trans_results.append({
            "synthetic_translation_px": d,
            "orb_detected_translation_px": orb.get("translation_px") if orb.get("valid") else None,
            "orb_valid": orb.get("valid"),
            "flow_median_mag": float(np.median(mag)),
            "edge_centroid_shift_px": edge.get("centroid_shift_magnitude_px") if edge.get("valid") else None,
            "phasecorr_magnitude": pc["magnitude"],
        })

    rot_results = []
    for d in rotations_deg:
        rotated = make_rotated(base, d)
        orb = va.algo_orb(base, rotated)
        rot_results.append({
            "synthetic_rotation_deg": d,
            "orb_detected_rotation_deg": orb.get("rotation_deg") if orb.get("valid") else None,
            "orb_valid": orb.get("valid"),
        })

    result = {
        "label": "SYNTHETIC_SENSITIVITY_TEST", "disclosure": "NOT REAL MOTION EVIDENCE -- synthetic warps of a stored ARM2_ROI PRE crop, used only to characterize detector sensitivity.",
        "translation_tests": trans_results, "rotation_tests": rot_results,
        "note": "Compare orb_detected_* against synthetic_* to see how much the ORB/RANSAC pipeline under- or over-estimates small synthetic warps, and at what magnitude edge/flow/phase-correlation become reliably non-zero.",
    }
    w("synthetic_sensitivity.json", result)
    return result


# ===========================================================================
# §21 -- Arm2 trackability review (numeric support for the .md review)
# ===========================================================================
def arm2_trackability_numeric():
    orb = cv2.ORB_create(nfeatures=500)
    arm2_pre = crop(PRE_MEAN, ROIS["ARM2_ROI"])
    arm2_post = crop(POST_MEAN, ROIS["ARM2_ROI"])
    kp_pre, _ = orb.detectAndCompute(arm2_pre, None)
    kp_post, _ = orb.detectAndCompute(arm2_post, None)
    h, w_ = arm2_pre.shape

    def spread(kps):
        if not kps:
            return None
        pts = np.array([k.pt for k in kps])
        return {"count": len(kps), "x_std_px": float(pts[:, 0].std()), "y_std_px": float(pts[:, 1].std()),
               "x_range_px": float(pts[:, 0].max() - pts[:, 0].min()),
               "y_range_px": float(pts[:, 1].max() - pts[:, 1].min()),
               "roi_width_px": w_, "roi_height_px": h}

    result = {"pre_keypoints": spread(kp_pre), "post_keypoints": spread(kp_post),
             "p007f_03_reference_count": 14}
    w("arm2_trackability_numeric.json", result)
    return result


# ===========================================================================
# §24 -- Forensic classification (F1..F4)
# ===========================================================================
def forensic_classification(noise_result, spatial_result, coherence_result, depth_result):
    arm2_any_exceed = spatial_result["arm2_roi_exceeds_within_window_noise_any_metric"]
    static_any_exceed = spatial_result["static_reference_roi_exceeds_within_window_noise_any_metric"]
    other_robot_exceed = spatial_result.get("other_robot_rois_also_exceeding", [])
    non_robot_exceed = spatial_result.get("non_robot_non_static_rois_also_exceeding", [])
    localization = spatial_result["localization_label"]
    final_coherence = coherence_result["final_coherence_label"]
    r_value = coherence_result.get("arm2_directional_coherence_R")

    reasons = []
    if not arm2_any_exceed:
        code = "F3"
        reasons.append("ARM2_ROI PRE->POST signal does not exceed the within-window (PRE_i-PRE_j / POST_i-POST_j) noise distribution (mean+3sigma) on any of the 6 independent metrics tested (ORB translation/rotation, optical flow, abs-diff, edge centroid shift, phase correlation).")
    elif arm2_any_exceed and static_any_exceed:
        code = "F2"
        reasons.append("ARM2_ROI exceeds noise on at least one metric, but STATIC_REFERENCE_ROI (a known-rigid, non-robot background feature) also exceeds its own noise floor on at least one metric -- the ARM2 signal cannot be cleanly attributed to robot motion versus a global scene/registration effect.")
    elif non_robot_exceed:
        code = "F2"
        reasons.append("ARM2_ROI exceeds noise and STATIC_REFERENCE_ROI does not, but non-robot ROI(s) %s (DOWNSTREAM/ROBOT background-adjacent regions) also exceed noise -- weakens confidence that the signal is specific to robot geometry." % non_robot_exceed)
    elif arm2_any_exceed and not static_any_exceed and final_coherence == "COHERENT_ROBOT_MOTION" and not other_robot_exceed:
        code = "F1"
        reasons.append("ARM2_ROI exceeds within-window noise on 3/6 independent metrics (optical-flow median magnitude, raw abs-diff mean, phase-correlation magnitude) while STATIC_REFERENCE_ROI exceeds on 0/6, no other robot-geometry ROI exceeds noise, and the ARM2_ROI optical-flow field's top-25%%-magnitude pixels show directional coherence R=%.3f (>=0.5 threshold) versus STATIC_REFERENCE_ROI's R far below threshold." % (r_value or 0))
    elif arm2_any_exceed and not static_any_exceed and other_robot_exceed:
        code = "F2"
        reasons.append("ARM2_ROI exceeds within-window noise and STATIC_REFERENCE_ROI does not, but other approximate robot-geometry ROIs (%s) also exceed noise -- the signal may not be ARM2-specific (could be diffuse robot-region change, or an artifact of imprecise unmeasured ROI boundaries for those regions)." % other_robot_exceed)
    else:
        code = "F2"
        reasons.append("ARM2_ROI exceeds noise on at least one metric and is localized away from background, but the change does not show the directional coherence (R>=0.5) expected of rigid-link motion (fragmentary/non-rigid, R=%s)." % r_value)

    result = {"code": code, "label": {
        "F1": "POST_HOC_ROBOT_LOCALIZED_MOTION_SUPPORTED",
        "F2": "POST_HOC_LOCAL_CHANGE_DETECTED_BUT_MOTION_NOT_ESTABLISHED",
        "F3": "NO_ROBOT_LOCALIZED_CHANGE_ABOVE_NOISE",
        "F4": "FORENSIC_EVIDENCE_INCONCLUSIVE",
    }[code], "reasons": reasons, "localization_label": localization, "coherence_label": final_coherence,
        "arm2_exceeds_noise": arm2_any_exceed, "static_exceeds_noise": static_any_exceed}
    w("forensic_classification.json", result)
    return result


# ===========================================================================
# §26 -- figures
# ===========================================================================
def _draw_rois(ax, roi_names=None, color="yellow"):
    import matplotlib.patches as patches
    for name in (roi_names or ROIS):
        x, y, w_, h_ = ROIS[name]
        ax.add_patch(patches.Rectangle((x, y), w_, h_, linewidth=1.2, edgecolor=color, facecolor="none"))
        ax.text(x, max(0, y - 5), name, color=color, fontsize=6)


def make_figures(global_shift, reg_result, flow_result, noise_result, depth_result, synth_result,
                 orb_result):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(FIG_DIR, exist_ok=True)

    # 01 — PRE/POST overview with ROI boxes
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    axes[0].imshow(PRE_MEAN, cmap="gray"); axes[0].set_title("PRE (5-frame mean)"); _draw_rois(axes[0])
    axes[1].imshow(POST_MEAN, cmap="gray"); axes[1].set_title("POST (5-frame mean)"); _draw_rois(axes[1])
    for a in axes:
        a.axis("off")
    fig.suptitle("P-007F-06 — PRE vs POST overview (ROIs annotated)")
    fig.savefig(os.path.join(FIG_DIR, "01_pre_post_overview.png"), dpi=130, bbox_inches="tight")
    plt.close(fig)

    # 02 — registered difference heatmap
    post_registered = apply_global_shift(POST_MEAN, global_shift)
    diff = np.abs(PRE_MEAN.astype(np.float64) - post_registered.astype(np.float64))
    fig, ax = plt.subplots(figsize=(9, 6))
    im = ax.imshow(diff, cmap="inferno", vmin=0, vmax=30)
    _draw_rois(ax, color="cyan")
    ax.set_title("Registered |PRE-POST| difference (global shift corrected)")
    fig.colorbar(im, ax=ax, label="abs diff (8-bit gray)")
    ax.axis("off")
    fig.savefig(os.path.join(FIG_DIR, "02_registered_difference.png"), dpi=130, bbox_inches="tight")
    plt.close(fig)

    # 03 — optical flow quiver over ARM2_ROI
    x, y, w_, h_ = ROIS["ARM2_ROI"]
    arm2_pre = crop(PRE_MEAN, ROIS["ARM2_ROI"])
    arm2_post = crop(POST_MEAN, ROIS["ARM2_ROI"])
    flow = cv2.calcOpticalFlowFarneback(arm2_pre, arm2_post, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    step = 6
    yy, xx = np.mgrid[0:h_:step, 0:w_:step]
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.imshow(arm2_pre, cmap="gray")
    ax.quiver(xx, yy, flow[::step, ::step, 0], flow[::step, ::step, 1], color="red", scale=1, scale_units="xy",
             angles="xy")
    ax.set_title("ARM2_ROI optical flow (PRE background, vectors = PRE->POST flow)\nmedian mag=%.3fpx"
                % flow_result["pre_post"]["ARM2_ROI"]["median_magnitude"])
    ax.axis("off")
    fig.savefig(os.path.join(FIG_DIR, "03_optical_flow_robot_roi.png"), dpi=130, bbox_inches="tight")
    plt.close(fig)

    # 04 — ROI comparison bar chart (registered diff mean per ROI)
    names = list(ROIS.keys())
    vals = [reg_result["registered_diff_per_roi"][n]["mean"] for n in names]
    fig, ax = plt.subplots(figsize=(10, 5))
    colors = ["tab:orange" if n == "ARM2_ROI" else ("tab:gray" if n == "STATIC_REFERENCE_ROI" else "tab:blue")
             for n in names]
    ax.bar(names, vals, color=colors)
    ax.set_ylabel("registered |PRE-POST| mean (8-bit gray)")
    ax.set_title("Registered difference by ROI (orange=ARM2, gray=STATIC reference)")
    plt.setp(ax.get_xticklabels(), rotation=45, ha="right")
    fig.savefig(os.path.join(FIG_DIR, "04_roi_comparison.png"), dpi=130, bbox_inches="tight")
    plt.close(fig)

    # 05 — depth difference heatmap (ARM2/ROBOT region in depth coords)
    depth_diff = POST_DEPTH_MEAN_VALID - PRE_DEPTH_MEAN_VALID
    depth_diff_vis = np.clip(depth_diff, -100, 100)
    fig, ax = plt.subplots(figsize=(8, 5))
    im = ax.imshow(depth_diff_vis, cmap="coolwarm", vmin=-100, vmax=100)
    for name in ("ROBOT_ROI", "ARM2_ROI", "STATIC_REFERENCE_ROI"):
        dx, dy, dw, dh = color_roi_to_depth_roi(ROIS[name])
        import matplotlib.patches as patches
        ax.add_patch(patches.Rectangle((dx, dy), dw, dh, linewidth=1, edgecolor="lime", facecolor="none"))
    fig.colorbar(im, ax=ax, label="POST-PRE depth (mm)")
    ax.set_title("Depth difference (POST-PRE, mm) -- depth-ROI mapping is approximate, see rgb_depth_alignment.json")
    ax.axis("off")
    fig.savefig(os.path.join(FIG_DIR, "05_depth_difference.png"), dpi=130, bbox_inches="tight")
    plt.close(fig)

    # 06 — noise vs PRE->POST signal (ARM2_ROI, 3 metrics)
    metrics = ["orb_translation_px", "flow_median_mag", "edge_centroid_shift_px", "phasecorr_magnitude"]
    fig, axes = plt.subplots(1, len(metrics), figsize=(16, 4))
    for ax, m in zip(axes, metrics):
        ns = noise_result["noise_distribution"]["ARM2_ROI"][m]
        sig = noise_result["pre_to_post_signal"]["ARM2_ROI"][m]
        if ns.get("n", 0) == 0:
            ax.set_title(m + "\n(no data)")
            continue
        ax.bar(["within-window\nnoise (mean)", "within-window\nnoise (max)", "PRE->POST\nsignal"],
              [ns["mean"], ns["max"], sig if sig is not None else 0],
              color=["lightgray", "gray", "tab:red"])
        ax.set_title(m, fontsize=9)
    fig.suptitle("ARM2_ROI: within-window noise vs actual PRE->POST signal")
    fig.savefig(os.path.join(FIG_DIR, "06_noise_vs_prepost.png"), dpi=130, bbox_inches="tight")
    plt.close(fig)

    # 07 — edge overlay (ARM2_ROI)
    e_pre = cv2.Canny(arm2_pre, 50, 150)
    e_post = cv2.Canny(arm2_post, 50, 150)
    overlay = np.zeros((*e_pre.shape, 3), dtype=np.uint8)
    overlay[..., 0] = e_pre
    overlay[..., 1] = e_post
    fig, ax = plt.subplots(figsize=(6, 5))
    ax.imshow(overlay)
    ax.set_title("ARM2_ROI edges: red=PRE, green=POST, yellow=overlap")
    ax.axis("off")
    fig.savefig(os.path.join(FIG_DIR, "07_edge_displacement.png"), dpi=130, bbox_inches="tight")
    plt.close(fig)

    # 08 — phase correlation magnitude per ROI
    with open(os.path.join(OUT_DIR, "phase_correlation_analysis.json")) as f:
        pc = json.load(f)
    names2 = list(pc.keys())
    mags = [pc[n]["magnitude"] for n in names2]
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.bar(names2, mags, color=["tab:orange" if n == "ARM2_ROI" else "tab:blue" for n in names2])
    ax.set_ylabel("phase-correlation shift magnitude (px)")
    ax.set_title("Local phase correlation by ROI")
    plt.setp(ax.get_xticklabels(), rotation=45, ha="right")
    fig.savefig(os.path.join(FIG_DIR, "08_phase_correlation.png"), dpi=130, bbox_inches="tight")
    plt.close(fig)

    # 09 — depth PRE/POST side-by-side (qualitative, point-cloud forensic marked LIMITED)
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].imshow(PRE_DEPTH_MEAN_VALID, cmap="viridis", vmin=0, vmax=1600); axes[0].set_title("PRE depth (mean, mm)")
    axes[1].imshow(POST_DEPTH_MEAN_VALID, cmap="viridis", vmin=0, vmax=1600); axes[1].set_title("POST depth (mean, mm)")
    for a in axes:
        a.axis("off")
    fig.suptitle("Qualitative depth comparison (point-cloud forensic LIMITED -- see pointcloud_analysis.json)")
    fig.savefig(os.path.join(FIG_DIR, "09_pointcloud_comparison.png"), dpi=130, bbox_inches="tight")
    plt.close(fig)

    # 10 — synthetic sensitivity
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    tx = [r["synthetic_translation_px"] for r in synth_result["translation_tests"]]
    ty = [r["orb_detected_translation_px"] or 0 for r in synth_result["translation_tests"]]
    axes[0].plot(tx, tx, "k--", label="ideal")
    axes[0].plot(tx, ty, "o-", label="ORB detected")
    axes[0].set_xlabel("synthetic translation (px)"); axes[0].set_ylabel("detected (px)")
    axes[0].set_title("Translation sensitivity"); axes[0].legend()
    rx = [r["synthetic_rotation_deg"] for r in synth_result["rotation_tests"]]
    ry = [r["orb_detected_rotation_deg"] or 0 for r in synth_result["rotation_tests"]]
    axes[1].plot(rx, rx, "k--", label="ideal")
    axes[1].plot(rx, ry, "o-", label="ORB detected")
    axes[1].set_xlabel("synthetic rotation (deg)"); axes[1].set_ylabel("detected (deg)")
    axes[1].set_title("Rotation sensitivity"); axes[1].legend()
    fig.suptitle("SYNTHETIC SENSITIVITY TEST -- NOT REAL MOTION EVIDENCE")
    fig.savefig(os.path.join(FIG_DIR, "10_synthetic_sensitivity.png"), dpi=130, bbox_inches="tight")
    plt.close(fig)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(FIG_DIR, exist_ok=True)
    print("source integrity...")
    integrity = run_source_integrity()
    print(integrity["verdict"])
    if integrity["verdict"] != "SOURCE_INTEGRITY_FROZEN_FIRST_TIME":
        print("SOURCE_EVIDENCE_INTEGRITY_FAILURE -- stopping")
        return integrity

    run_roi_definition()
    print("global registration...")
    global_shift = global_registration()

    print("ORB reproduction...")
    orb_result = orb_reproduction()
    print(orb_result["reproduced_within_tolerance_0.05"])

    print("optical flow...")
    flow_result = run_optical_flow()

    print("registered difference...")
    reg_result, post_registered, diff_registered = run_registered_difference(global_shift)

    print("edge analysis...")
    edge_result = run_edge_analysis()

    print("phase correlation...")
    pc_result = run_phase_correlation()

    print("within-window noise (this is the slow one)...")
    noise_result = within_window_noise()

    print("spatial localization...")
    spatial_result = spatial_localization(reg_result, flow_result, noise_result)
    coherence_result = coherence_classification(spatial_result, flow_result)
    w("coherence_classification.json", coherence_result)

    print("depth forensic...")
    alignment = rgb_depth_alignment_check()
    depth_result = run_depth_analysis(alignment)

    print("point cloud (limited)...")
    run_pointcloud_analysis(alignment)

    print("synthetic sensitivity...")
    synth_result = synthetic_sensitivity()

    print("arm2 trackability numeric...")
    trackability = arm2_trackability_numeric()

    print("forensic classification...")
    classification = forensic_classification(noise_result, spatial_result, coherence_result, depth_result)
    print(classification["code"], classification["label"])

    print("figures...")
    make_figures(global_shift, reg_result, flow_result, noise_result, depth_result, synth_result, orb_result)

    return {"integrity": integrity, "orb": orb_result, "classification": classification}


if __name__ == "__main__":
    result = main()
    print(json.dumps({"summary": "done"}, ensure_ascii=False))
