#!/usr/bin/env python3
"""P-007F-07 Phase A — §13/§14/§15 continuous marker trajectory extraction +
motion-type classification (M1..M6/MX). 순수 수치 계산 -- 카메라/로봇 API를
전혀 참조하지 않는다. 입력은 marker_detector.detect_markers() 결과의 시계열
(실측이든 §13 요구대로 Phase A에서는 synthetic이든)뿐이다.

🔴 §16 threshold는 이번 gate에서 실제 숫자로 고정하지 않는다 -- marker 부착
후(§27 static qualification) sham 분포로 정한다. 여기서는 "threshold formula"
와 "주어지면 어떻게 분류하는지"만 구현·검증한다(합성 테스트는 임의의 placeholder
threshold로 로직만 검증, 실제 사용 수치 아님).
"""
import math


def _translation_series(primary_series):
    """marker_detected=false 프레임은 None 으로 남긴다(삭제하지 않음, §12)."""
    out = []
    for rec in primary_series:
        if not rec.get("marker_detected"):
            out.append(None)
        else:
            out.append((rec["center_x"], rec["center_y"]))
    return out


def extract_trajectory(primary_series, background_series, baseline_frames=3):
    """§13/§14 -- primary marker 의 dx(t)/dy(t)/translation(t)/rotation(t) 를
    PRE baseline 대비, background(camera) motion을 뺀 값으로 계산한다.
    primary_series/background_series는 같은 길이, 같은 timestamp 순서의
    marker_detector 결과 리스트(또는 §14 STATIC_REFERENCE_ROI 추적 결과)."""
    n = len(primary_series)
    pts = _translation_series(primary_series)
    bg_pts = _translation_series(background_series)

    valid_baseline_pts = [p for p in pts[:baseline_frames] if p is not None]
    valid_baseline_bg = [p for p in bg_pts[:baseline_frames] if p is not None]
    if not valid_baseline_pts or not valid_baseline_bg:
        return {"valid": False, "reason": "NO_VALID_BASELINE_FRAMES"}
    base_x = sum(p[0] for p in valid_baseline_pts) / len(valid_baseline_pts)
    base_y = sum(p[1] for p in valid_baseline_pts) / len(valid_baseline_pts)
    bg_base_x = sum(p[0] for p in valid_baseline_bg) / len(valid_baseline_bg)
    bg_base_y = sum(p[1] for p in valid_baseline_bg) / len(valid_baseline_bg)

    dx, dy, translation, timestamps, valid_mask = [], [], [], [], []
    prev_translation = None
    velocity_proxy = []
    for i in range(n):
        ts = primary_series[i].get("timestamp")
        timestamps.append(ts)
        if pts[i] is None or bg_pts[i] is None:
            dx.append(None); dy.append(None); translation.append(None)
            valid_mask.append(False)
            velocity_proxy.append(None)
            prev_translation = None
            continue
        raw_dx = pts[i][0] - base_x
        raw_dy = pts[i][1] - base_y
        bg_dx = bg_pts[i][0] - bg_base_x
        bg_dy = bg_pts[i][1] - bg_base_y
        comp_dx = raw_dx - bg_dx
        comp_dy = raw_dy - bg_dy
        t = math.hypot(comp_dx, comp_dy)
        dx.append(comp_dx); dy.append(comp_dy); translation.append(t)
        valid_mask.append(True)
        if prev_translation is not None and timestamps[i] is not None and timestamps[i - 1] is not None:
            dt = timestamps[i] - timestamps[i - 1]
            velocity_proxy.append((t - prev_translation) / dt if dt else None)
        else:
            velocity_proxy.append(None)
        prev_translation = t

    return {"valid": True, "timestamps": timestamps, "dx": dx, "dy": dy,
           "translation": translation, "velocity_proxy": velocity_proxy,
           "baseline": {"primary_x": base_x, "primary_y": base_y,
                       "background_x": bg_base_x, "background_y": bg_base_y},
           "valid_mask": valid_mask,
           "valid_frame_ratio": sum(valid_mask) / n if n else 0.0}


def threshold_formula(sham_mean, sham_stdev, fixed_floor):
    """§16 -- T = max(fixed_floor, sham_mean + 5*sigma). 실제 sham_mean/stdev는
    marker static qualification(§27) 이후에만 채워진다."""
    return max(fixed_floor, sham_mean + 5 * sham_stdev)


def _count_up_crossings(above_bool_series):
    """below->above 전이 횟수(None/누락 프레임은 건너뛴다)."""
    crossings = 0
    prev = False
    for v in above_bool_series:
        if v is None:
            continue
        if v and not prev:
            crossings += 1
        prev = v
    return crossings


def _count_sign_reversals(dx, dy, t_translation):
    """🔴 magnitude(translation=hypot(dx,dy))만 보면 부호가 반전되는 진동
    (예: +3px -> -3px)에서 중간에 magnitude가 threshold 아래로 내려가지
    않을 수 있어(hypot는 항상 >=0) up-crossing 횟수만으로는 oscillation을
    놓친다(자체발견, Case C 합성 테스트 실패로 드러남). dx/dy 중 진폭이 더 큰
    축(dominant axis)의 **부호**가 threshold 이상 변위 구간에서 몇 번
    반전되는지 별도로 센다."""
    dx_amp = max((abs(v) for v in dx if v is not None), default=0.0)
    dy_amp = max((abs(v) for v in dy if v is not None), default=0.0)
    axis = dx if dx_amp >= dy_amp else dy
    reversals = 0
    prev_sign = 0
    for v in axis:
        if v is None or abs(v) < t_translation * 0.5:
            continue
        sign = 1 if v > 0 else -1
        if prev_sign != 0 and sign != prev_sign:
            reversals += 1
        prev_sign = sign
    return reversals


def classify_motion(trajectory, t_translation, secondary_trajectory=None,
                    end_window=3, hold_fraction=0.8, return_fraction=0.2,
                    background_instability=False, min_valid_frame_ratio=0.7):
    """§15 -- M1..M6/MX 분류. `trajectory`는 extract_trajectory() 반환값."""
    if not trajectory.get("valid"):
        return {"code": "MX", "label": "EXPERIMENT_INVALID", "reason": trajectory.get("reason")}
    if trajectory["valid_frame_ratio"] < min_valid_frame_ratio:
        return {"code": "MX", "label": "EXPERIMENT_INVALID",
               "reason": "INSUFFICIENT_VALID_FRAMES: %.2f < %.2f"
                        % (trajectory["valid_frame_ratio"], min_valid_frame_ratio)}
    if background_instability:
        return {"code": "MX", "label": "EXPERIMENT_INVALID", "reason": "BACKGROUND_REFERENCE_UNSTABLE"}

    translation = trajectory["translation"]
    above = [(t is not None and t > t_translation) for t in translation]
    any_above = any(v for v in above if v is not None) if any(above) else False
    any_above = any(above)

    secondary_above_any = False
    if secondary_trajectory is not None and secondary_trajectory.get("valid"):
        sec_above = [(t is not None and t > t_translation) for t in secondary_trajectory["translation"]]
        secondary_above_any = any(sec_above)

    if not any_above:
        if secondary_above_any:
            return {"code": "M5", "label": "DOWNSTREAM_LOCAL_MOTION_SUPPORTED",
                   "reason": "primary marker stays below threshold throughout; secondary marker exceeds threshold"}
        return {"code": "M1", "label": "NO_EXTERNAL_MOTION_DETECTED", "reason": "no frame exceeds threshold"}

    tail = above[-end_window:] if len(above) >= end_window else above
    tail_valid = [v for t, v in zip(translation[-end_window:], tail) if t is not None]
    end_above_fraction = (sum(tail_valid) / len(tail_valid)) if tail_valid else 0.0

    crossings = _count_up_crossings(above)
    sign_reversals = _count_sign_reversals(trajectory["dx"], trajectory["dy"], t_translation)
    # 🔴 oscillation_signal(방향 반전/복수 excursion)을 hold/transient 분기보다 먼저
    # 확인한다 -- end_above_fraction 만으로 분기하면(자체발견, Case C 최초 실패)
    # "중간 정도"로 끝나는 진동이 hold/transient 어느 쪽에도 깔끔히 안 들어가
    # 그냥 M6(unresolved)로 새 버린다. 방향 반전 자체가 hold/transient보다 더
    # 구체적이고 강한 신호이므로 우선한다.
    oscillation_signal = (crossings >= 2) or (sign_reversals >= 1)

    if oscillation_signal:
        return {"code": "M4", "label": "OSCILLATION_OR_VIBRATION_SUPPORTED",
               "end_above_fraction": end_above_fraction, "up_crossings": crossings,
               "sign_reversals": sign_reversals}
    if end_above_fraction >= hold_fraction:
        return {"code": "M2", "label": "RIGID_MOTION_AND_HOLD_SUPPORTED",
               "end_above_fraction": end_above_fraction, "up_crossings": crossings,
               "sign_reversals": sign_reversals}
    if end_above_fraction <= return_fraction:
        return {"code": "M3", "label": "TRANSIENT_MOTION_AND_RETURN_SUPPORTED",
               "end_above_fraction": end_above_fraction, "up_crossings": crossings,
               "sign_reversals": sign_reversals}
    return {"code": "M6", "label": "EXTERNAL_MOTION_DETECTED_BUT_TYPE_UNRESOLVED",
           "end_above_fraction": end_above_fraction, "up_crossings": crossings,
           "sign_reversals": sign_reversals}
