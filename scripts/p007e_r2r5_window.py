#!/usr/bin/env python3
"""P-007E-R2-R5 — PRE/POST visual window capture.

run_trial()(scripts/p007e_r2r2_c1r1_sham.py, CLOSED, 1바이트도 수정하지 않음)의
단일 window 루프(reference 또는 comparison 한 쪽)를 **그대로** 분리 호출 가능한
함수로 옮긴 것이다 — 알고리즘(estimator/mask/decode_gray/median)은 전혀
재구현하지 않고 그대로 주입받는다. run_trial() 자체를 재사용하지 않는 이유는
R3 preregistration(section_17_visual_changed_construction.why_not_reuse_run_trial_directly)
에 이미 기록되어 있다 — reference->(quiet+command+settle)->comparison 을 한 함수
안에서 끝까지 진행해 중간에 abort 결정점을 둘 수 없기 때문이다.

🔴 anchor 계약 — capture_window(..., anchor_gray=None) 을 PRE 호출에 쓰면
   첫 decoded frame 을 anchor 로 고정하고 그 **동일 객체**를 반환한다. POST
   호출은 그 반환값을 anchor_gray 로 다시 넘겨야 한다. 이 파일 자신은 어떤
   하드웨어도 import 하지 않는다 — reader/estimator/roi_mean_fn 은 duck-typed
   주입이다(run_trial() 과 동일 인터페이스: check_format_invariant/capture_one_frame).
"""
import os
import time


def capture_window(reader, estimator, roi_mean_fn, mask, M, run_root, window_name,
                   anchor_gray=None, frame_interval_s=0.25, sleep_fn=None):
    """반환: (window_dict, anchor_gray). window_dict["valid"] 는 run_trial() 과
    동일 기준(>=80% frame 이 estimator guard 통과 + format invariant 유지)이다."""
    import importlib
    meas = importlib.import_module("p007e_r2r2_measurement")
    sleep_fn = sleep_fn or time.sleep
    frame_dir = os.path.join(run_root, "frames", window_name) if run_root else None
    if frame_dir:
        os.makedirs(frame_dir, exist_ok=True)

    inv_before = reader.check_format_invariant()
    if not inv_before["match"]:
        return {"valid": False, "reason": "FORMAT_INVARIANT_VIOLATION_BEFORE",
                "format_invariant_before": inv_before, "median": None}, anchor_gray

    counters = {"frames_requested": 0, "frames_capture_or_decode_failed": 0,
               "frames_decoded": 0, "frames_guard_valid": 0, "frames_guard_rejected": 0,
               "frames_persisted": 0}
    rows = []
    first_frame_saved = False
    t_next = time.monotonic()
    for i in range(M):
        now = time.monotonic()
        if now < t_next:
            sleep_fn(t_next - now)
        t_next += frame_interval_s   # 🔴 run_trial() 과 동일한 0.25s 목표 페이싱

        counters["frames_requested"] += 1
        frame = reader.capture_one_frame()
        if not frame.ok:
            counters["frames_capture_or_decode_failed"] += 1
            rows.append({"sequence": frame.sequence, "t": frame.capture_timestamp,
                        "decoded": False, "valid": False, "roi_mean": None, "reason": frame.error})
            continue
        gray = meas.decode_gray(frame.encoded_bytes)
        if gray is None:
            counters["frames_capture_or_decode_failed"] += 1
            rows.append({"sequence": frame.sequence, "t": frame.capture_timestamp,
                        "decoded": False, "valid": False, "roi_mean": None, "reason": "DECODE_FAILED"})
            continue
        counters["frames_decoded"] += 1
        roi_mean = roi_mean_fn(gray, mask)
        if anchor_gray is None:
            anchor_gray = gray
            if frame_dir:
                open(os.path.join(frame_dir, "%s_first.jpg" % window_name), "wb").write(frame.encoded_bytes)
            counters["frames_persisted"] += 1
            first_frame_saved = True
        elif not first_frame_saved:
            if frame_dir:
                open(os.path.join(frame_dir, "%s_first.jpg" % window_name), "wb").write(frame.encoded_bytes)
            counters["frames_persisted"] += 1
            first_frame_saved = True
        est = estimator(anchor_gray, gray)
        if est["valid"]:
            counters["frames_guard_valid"] += 1
        else:
            counters["frames_guard_rejected"] += 1
        rows.append({"sequence": frame.sequence, "t": frame.capture_timestamp,
                    "decoded": True, "valid": est["valid"], "roi_mean": roi_mean,
                    "reason": est.get("reason"), "est": est})

    valid_rows = [r for r in rows if r["valid"]]
    median = None
    if len(valid_rows) >= int(0.8 * M):
        an = importlib.import_module("p007e_r2r2_analysis")
        median = {k: an.median([r["est"][k] for r in valid_rows])
                  for k in ("translation_x_px", "translation_y_px", "rotation_deg")}

    inv_after = reader.check_format_invariant()
    window = {"window_name": window_name, "n_frames": len(rows), "n_valid": len(valid_rows),
             "dropout_rate": 1.0 - len(valid_rows) / float(max(len(rows), 1)),
             "rows": rows, "median": median, "frame_accounting": counters,
             "format_invariant_before": inv_before, "format_invariant_after": inv_after,
             "valid": bool(median is not None and inv_after["match"])}
    return window, anchor_gray
