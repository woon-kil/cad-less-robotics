#!/usr/bin/env python3
"""P-007E-R2-R2 Stage C1-R1 — Analysis Layer (결정적 순수 함수).

🔴 trial 목록(JSON 구조)을 받아 noise metric·LOO/publication 임계·Q1~X 판정을 낸다.
   카메라·subprocess·시간을 전혀 모른다 — **입력은 전부 이미 메모리/디스크에 있는 데이터**다.
   사전등록(stage_c1r1_preregistration.json) 의 공식을 그대로 구현한다. 데이터를 보고
   바뀌는 부분이 없다 — 그래서 **데이터 없이도(합성 trial 로) 전 분기를 단위시험할 수 있다.**
"""
import math


def median(xs):
    xs = sorted(xs)
    n = len(xs)
    if n == 0:
        return float("nan")
    mid = n // 2
    return xs[mid] if n % 2 else (xs[mid - 1] + xs[mid]) / 2.0


def robust_sd(values):
    """σ_rob(x) = 1.4826 × median(|x − median(x)|)."""
    if not values:
        return 0.0
    m = median(values)
    return 1.4826 * median([abs(v - m) for v in values])


def sigma_eff(values, floor):
    return max(robust_sd(values), floor)


def spearman_abs_rho(xs, ys):
    """순수 python 순위상관(동순위는 평균순위로 처리). scipy 의존 없음."""
    n = len(xs)
    if n < 3:
        return None

    def ranks(vals):
        order = sorted(range(len(vals)), key=lambda i: vals[i])
        r = [0.0] * len(vals)
        i = 0
        while i < len(order):
            j = i
            while j + 1 < len(order) and vals[order[j + 1]] == vals[order[i]]:
                j += 1
            avg_rank = (i + j) / 2.0 + 1
            for k in range(i, j + 1):
                r[order[k]] = avg_rank
            i = j + 1
        return r

    rx, ry = ranks(xs), ranks(ys)
    mx, my = sum(rx) / n, sum(ry) / n
    cov = sum((a - mx) * (b - my) for a, b in zip(rx, ry))
    sx = math.sqrt(sum((a - mx) ** 2 for a in rx))
    sy = math.sqrt(sum((b - my) ** 2 for b in ry))
    if sx == 0 or sy == 0:
        return 0.0
    return abs(cov / (sx * sy))


def _valid_deltas(trials):
    v = [t for t in trials if t.get("verdict") == "VALID" and t.get("delta")]
    return v


def compute_thresholds(trials, K, floor_trans, floor_rot, exclude_index=None):
    """공식 그대로: thr_tx/ty = K·σ_eff(Δtx/ty), thr_norm = K·sqrt(σtx²+σty²), thr_rot = K·σ_eff(Δrot).
       exclude_index 가 주어지면 그 trial 의 delta 를 모집단에서 뺀다 (LOO)."""
    v = _valid_deltas(trials)
    if exclude_index is not None:
        v = [t for t in v if t["index"] != exclude_index]
    dtx = [t["delta"]["translation_x_px"] for t in v]
    dty = [t["delta"]["translation_y_px"] for t in v]
    drot = [t["delta"]["rotation_deg"] for t in v]
    s_tx = sigma_eff(dtx, floor_trans)
    s_ty = sigma_eff(dty, floor_trans)
    s_rot = sigma_eff(drot, floor_rot)
    return {"n_contributing_trials": len(v), "sigma_tx": s_tx, "sigma_ty": s_ty, "sigma_rot": s_rot,
            "thr_tx": K * s_tx, "thr_ty": K * s_ty,
            "thr_norm": K * math.hypot(s_tx, s_ty), "thr_rot": K * s_rot}


def compute_noise_metrics(trials, K, floor_trans, floor_rot):
    v = _valid_deltas(trials)
    dtx = [t["delta"]["translation_x_px"] for t in v]
    dty = [t["delta"]["translation_y_px"] for t in v]
    dnorm = [t["delta"]["norm"] for t in v]
    drot = [t["delta"]["rotation_deg"] for t in v]

    def stat(xs):
        return {"median": median(xs), "robust_sd": robust_sd(xs),
                "min": min(xs) if xs else None, "max": max(xs) if xs else None, "values": xs}

    dropout_rates = []
    invalid_count = 0
    inlier_counts, inlier_ratios, residuals = [], [], []
    for t in trials:
        if t.get("verdict") != "VALID":
            invalid_count += 1
        for w in t.get("windows", {}).values():
            dropout_rates.append(w.get("dropout_rate", 1.0))
            for row in w.get("rows", []):
                est = row.get("est")
                if row.get("valid") and est:
                    inlier_counts.append(est.get("inlier_count"))
                    inlier_ratios.append(est.get("inlier_ratio"))
                    residuals.append(est.get("residual_rms_inliers_px"))

    return {"delta_tx": stat(dtx), "delta_ty": stat(dty), "delta_norm": stat(dnorm), "delta_rot": stat(drot),
            "inlier_count_distribution": inlier_counts, "inlier_ratio_distribution": inlier_ratios,
            "residual_rms_distribution": residuals,
            "window_dropout_rates": dropout_rates, "window_dropout_rate_median": median(dropout_rates),
            "trial_invalid_count": invalid_count, "trial_invalid_rate": invalid_count / max(len(trials), 1)}


def compute_loo_false_positives(trials, K, floor_trans, floor_rot):
    v = _valid_deltas(trials)
    out = []
    for t in v:
        thr = compute_thresholds(trials, K, floor_trans, floor_rot, exclude_index=t["index"])
        d = t["delta"]
        detected = (d["norm"] > thr["thr_norm"]) or (abs(d["rotation_deg"]) > thr["thr_rot"])
        out.append({"index": t["index"], "loo_thr_norm": thr["thr_norm"], "loo_thr_rot": thr["thr_rot"],
                   "delta_norm": d["norm"], "delta_rot": d["rotation_deg"],
                   "detection": "DETECTED" if detected else "NOT_DETECTED",
                   "false_positive": detected})
    fp_count = sum(1 for r in out if r["false_positive"])
    return {"per_trial": out, "false_positive_count": fp_count,
            "false_positive_rate": fp_count / max(len(out), 1)}


def compute_q4_photometric_correlation(trials):
    """프레임 단위 pooled: (ROI 평균 밝기의 anchor 대비 변화량) vs (translation_norm_px)."""
    xs, ys = [], []
    for t in trials:
        for w in t.get("windows", {}).values():
            rows = w.get("rows", [])
            anchor_brightness = None
            for row in rows:
                if row.get("decoded") and row.get("roi_mean") is not None and anchor_brightness is None:
                    anchor_brightness = row["roi_mean"]
            for row in rows:
                est = row.get("est")
                if row.get("valid") and est and row.get("roi_mean") is not None and anchor_brightness is not None:
                    xs.append(row["roi_mean"] - anchor_brightness)
                    ys.append(est["translation_norm_px"])
    rho = spearman_abs_rho(xs, ys)
    return {"n_pairs": len(xs), "abs_spearman_rho": rho}


def classify_run(trials, K, floor_trans, floor_rot):
    """🔴 판정 우선순위 X > Q3 > Q4 > Q2 > Q1. 사전등록 공식 그대로."""
    n = len(trials)
    noise = compute_noise_metrics(trials, K, floor_trans, floor_rot)
    loo = compute_loo_false_positives(trials, K, floor_trans, floor_rot)
    q4 = compute_q4_photometric_correlation(trials)

    invalid_count = noise["trial_invalid_count"]
    window_dropout_20pct_trial_count = 0
    for t in trials:
        for w in t.get("windows", {}).values():
            if w.get("dropout_rate", 0) > 0.20:
                window_dropout_20pct_trial_count += 1
                break

    reasons = {
        "X": invalid_count >= 2 or n == 0,
        "Q3": noise["window_dropout_rate_median"] >= 0.10 or window_dropout_20pct_trial_count >= 2,
        "Q4": (q4["abs_spearman_rho"] is not None and q4["abs_spearman_rho"] >= 0.7),
        "Q2": loo["false_positive_count"] >= 2,
    }
    if reasons["X"]:
        verdict = "X — OBSERVATION INVALID"
    elif reasons["Q3"]:
        verdict = "Q3 — EXCESSIVE FEATURE DROPOUT"
    elif reasons["Q4"]:
        verdict = "Q4 — PHOTOMETRIC INSTABILITY DOMINANT"
    elif reasons["Q2"]:
        verdict = "Q2 — OBSERVATION CHANNEL NOT SENSITIVE/STABLE ENOUGH"
    elif invalid_count <= 1 and loo["false_positive_count"] <= 1:
        verdict = "Q1 — OBSERVATION CHANNEL QUALIFIED"
    else:
        verdict = "Q2 — OBSERVATION CHANNEL NOT SENSITIVE/STABLE ENOUGH"

    publication = compute_thresholds(trials, K, floor_trans, floor_rot)
    return {"verdict": verdict, "reasons": reasons, "noise_metrics": noise,
            "loo": loo, "q4_photometric": q4, "publication_thresholds": publication}
