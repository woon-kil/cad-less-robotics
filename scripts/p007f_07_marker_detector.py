#!/usr/bin/env python3
"""P-007F-07 Phase A — §4/§12 ArUco marker detection pipeline. Read-only image
processing (cv2.aruco) -- 이 파일은 카메라를 열지 않는다(cv2.VideoCapture/
rclpy 전혀 import 안 함), 로봇/서보 API도 전혀 참조하지 않는다. 입력은 이미
메모리에 있는 numpy 프레임(실 캡처든 합성 테스트든)뿐이다.

DICT_4X4_50 을 기본 사전으로 쓴다 -- 4x4 비트는 marker 1개당 필요한 모듈 수가
적어(6x6/7x7 대비) 작은 pixel footprint(30~50px)에서도 상대적으로 안정적으로
디코딩된다(§4 "OpenCV 환경에서 가장 안정적으로 지원되는 방식"의 실무적 해석).
"""
import math

import cv2
import numpy as np

DEFAULT_DICT = cv2.aruco.DICT_4X4_50
PRIMARY_MARKER_ID = 0
SECONDARY_MARKER_ID = 1


def create_detector(dict_id=DEFAULT_DICT):
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    params = cv2.aruco.DetectorParameters()
    # 작은 footprint(30~50px)에서 subpixel corner refine이 검출 안정성에 중요
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    return cv2.aruco.ArucoDetector(dictionary, params)


def generate_marker_image(marker_id, size_px, dict_id=DEFAULT_DICT):
    """인쇄용 marker 비트맵 생성(실제 출력은 이 gate의 범위 밖 -- §27에서 사람이 수행)."""
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.generateImageMarker(dictionary, marker_id, size_px)


def _orientation_deg(corners):
    """corner[0]->corner[1] 변의 방향을 marker orientation 으로 쓴다(ArUco 표준
    corner ordering: 0=top-left,1=top-right,2=bottom-right,3=bottom-left)."""
    c0, c1 = corners[0], corners[1]
    return float(math.degrees(math.atan2(c1[1] - c0[1], c1[0] - c0[0])))


def detect_markers(frame_bgr_or_gray, detector=None, timestamp=None, frame_index=None):
    """단일 프레임에서 marker를 검출한다. 검출 실패해도 marker_detected=false 로
    기록만 하고 예외를 던지지 않는다(§12 — 실패 frame도 삭제하지 않는다)."""
    detector = detector or create_detector()
    if frame_bgr_or_gray.ndim == 3:
        gray = cv2.cvtColor(frame_bgr_or_gray, cv2.COLOR_BGR2GRAY)
    else:
        gray = frame_bgr_or_gray
    corners_list, ids, rejected = detector.detectMarkers(gray)

    results = []
    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            c = corners_list[i][0]  # shape (4,2)
            center = c.mean(axis=0)
            area = float(cv2.contourArea(c.astype(np.float32)))
            side_lengths = [float(np.linalg.norm(c[k] - c[(k + 1) % 4])) for k in range(4)]
            side_consistency = float(np.std(side_lengths) / (np.mean(side_lengths) + 1e-9))
            results.append({
                "frame_index": frame_index, "timestamp": timestamp,
                "marker_detected": True, "marker_id": int(marker_id),
                "corner_0": c[0].tolist(), "corner_1": c[1].tolist(),
                "corner_2": c[2].tolist(), "corner_3": c[3].tolist(),
                "center_x": float(center[0]), "center_y": float(center[1]),
                "orientation_deg": _orientation_deg(c),
                "marker_area_px": area,
                "quality": {"side_length_mean_px": float(np.mean(side_lengths)),
                           "side_length_consistency_cv": side_consistency},
            })
    if not results:
        return [{"frame_index": frame_index, "timestamp": timestamp, "marker_detected": False,
                 "marker_id": None, "num_rejected_candidates": 0 if rejected is None else len(rejected)}]
    return results


def detect_markers_by_id(frame_bgr_or_gray, expected_ids, detector=None, timestamp=None, frame_index=None):
    """expected_ids 각각에 대해 검출 결과를 반환한다(검출 안 되면 marker_detected=false)."""
    all_hits = detect_markers(frame_bgr_or_gray, detector, timestamp, frame_index)
    by_id = {h["marker_id"]: h for h in all_hits if h.get("marker_detected")}
    out = {}
    for mid in expected_ids:
        out[mid] = by_id.get(mid, {"frame_index": frame_index, "timestamp": timestamp,
                                   "marker_detected": False, "marker_id": mid})
    return out
