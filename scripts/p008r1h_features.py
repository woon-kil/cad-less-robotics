"""P-008-R1H feature 변환과 선택 규칙.

점수는 여기 상수로만 등급을 매긴다. capture 파일을 읽지 않는다.
"""

import math

import numpy as np

BLOCK_HIGH = 0.90
BLOCK_MEDIUM = 0.75
TIE_ACCURACY = 0.02
MIN_VALID_FOLDS = 3
MIN_CONTACT_POINTS = 100
CONTACT_SHOE_HEIGHT = (0.005, 0.025)
CONTACT_SHOE_HORIZONTAL_MAX = 0.015
CONTACT_SUPPORT_HEIGHT_MAX = 0.005
CONTACT_SUPPORT_HORIZONTAL = (0.015, 0.035)

# IEC 61966-2-1 sRGB to CIE XYZ, then CIE L*a*b* with D65.
_SRGB_TO_XYZ = np.array(
    [
        [0.4124564, 0.3575761, 0.1804375],
        [0.2126729, 0.7151522, 0.0721750],
        [0.0193339, 0.1191920, 0.9503041],
    ],
    dtype=np.float64,
)
_D65 = np.array([0.95047, 1.0, 1.08883], dtype=np.float64)
_DELTA = 6.0 / 29.0

COLOR_SETS = ("RGB", "HSV", "Lab")
MULTIMODAL_SETS = ("Height", "HSV+Height", "Lab+Height", "RGB+Height")
SIMPLICITY = {
    "Height": 1,
    "RGB": 3,
    "HSV": 3,
    "Lab": 3,
    "HSV+Height": 4,
    "Lab+Height": 4,
    "RGB+Height": 4,
}
COLOR_TIE = ("Lab", "HSV", "RGB")
MULTI_TIE = ("Height", "Lab+Height", "HSV+Height", "RGB+Height")


def _finite_rgb(rgb):
    values = np.asarray(rgb, dtype=np.float64)
    if values.ndim == 1:
        values = values.reshape(1, 3)
    valid = np.isfinite(values).all(axis=1) & (values >= 0.0).all(axis=1) & (values <= 255.0).all(axis=1)
    return values, valid


def rgb_to_hsv(rgb):
    """H는 0..1, S·V는 0..1. 잘못된 RGB는 NaN."""
    values, valid = _finite_rgb(rgb)
    out = np.full((values.shape[0], 3), np.nan, dtype=np.float64)
    if not valid.any():
        return out
    channel = values[valid] / 255.0
    max_c = channel.max(axis=1)
    min_c = channel.min(axis=1)
    delta = max_c - min_c
    hue = np.zeros(channel.shape[0], dtype=np.float64)
    red = (max_c == channel[:, 0]) & (delta > 0)
    green = (max_c == channel[:, 1]) & (delta > 0)
    blue = (max_c == channel[:, 2]) & (delta > 0)
    hue[red] = np.mod((channel[red, 1] - channel[red, 2]) / delta[red], 6.0)
    hue[green] = (channel[green, 2] - channel[green, 0]) / delta[green] + 2.0
    hue[blue] = (channel[blue, 0] - channel[blue, 1]) / delta[blue] + 4.0
    hue = hue / 6.0
    saturation = np.where(max_c > 0.0, delta / np.maximum(max_c, 1e-12), 0.0)
    out[valid, 0] = hue
    out[valid, 1] = saturation
    out[valid, 2] = max_c
    return out


def rgb_to_lab(rgb):
    """sRGB 0..255 를 CIE L*a*b* (D65)로 보낸다. 잘못된 RGB는 NaN."""
    values, valid = _finite_rgb(rgb)
    out = np.full((values.shape[0], 3), np.nan, dtype=np.float64)
    if not valid.any():
        return out
    channel = values[valid] / 255.0
    linear = np.where(channel <= 0.04045, channel / 12.92, ((channel + 0.055) / 1.055) ** 2.4)
    xyz = linear @ _SRGB_TO_XYZ.T
    ratio = xyz / _D65
    f_ratio = np.where(ratio > _DELTA ** 3, np.cbrt(ratio), ratio / (3.0 * _DELTA ** 2) + 4.0 / 29.0)
    out[valid, 0] = 116.0 * f_ratio[:, 1] - 16.0
    out[valid, 1] = 500.0 * (f_ratio[:, 0] - f_ratio[:, 1])
    out[valid, 2] = 200.0 * (f_ratio[:, 1] - f_ratio[:, 2])
    return out


def plane_height(xyz, coefficients):
    """viewpoint로 향한 평면 ax+by+cz+d. 카메라 쪽이 양수다. NaN 점은 NaN."""
    points = np.asarray(xyz, dtype=np.float64)
    coeff = np.asarray(coefficients, dtype=np.float64)
    height = points @ coeff[:3] + coeff[3]
    invalid = ~np.isfinite(points).all(axis=1)
    height = height.astype(np.float64)
    height[invalid] = np.nan
    return height


def point_in_polygon(u, v, polygon):
    """홀수-짝수 규칙. polygon은 (u, v) 순서."""
    poly = np.asarray(polygon, dtype=np.float64)
    uu = np.asarray(u, dtype=np.float64)
    vv = np.asarray(v, dtype=np.float64)
    inside = np.zeros(uu.shape[0], dtype=bool)
    x0, y0 = poly[-1]
    for x1, y1 in poly:
        cond = ((y0 > vv) != (y1 > vv)) & (uu < (x1 - x0) * (vv - y0) / (y1 - y0 + 1e-15) + x0)
        inside ^= cond
        x0, y0 = x1, y1
    return inside


def histogram_js(left, right, edges):
    """log2 Jensen-Shannon. 범위는 0..1."""
    a = np.asarray(left, dtype=np.float64)
    b = np.asarray(right, dtype=np.float64)
    a = a[np.isfinite(a)]
    b = b[np.isfinite(b)]
    if a.size == 0 or b.size == 0:
        return None
    ha, _ = np.histogram(a, bins=edges)
    hb, _ = np.histogram(b, bins=edges)
    pa = ha.astype(np.float64) / ha.sum()
    pb = hb.astype(np.float64) / hb.sum()
    mean = 0.5 * (pa + pb)

    def kl(p):
        mask = p > 0
        return float(np.sum(p[mask] * np.log2(p[mask] / mean[mask])))

    return 0.5 * kl(pa) + 0.5 * kl(pb)


def bhattacharyya(left, right, edges):
    a = np.asarray(left, dtype=np.float64)
    b = np.asarray(right, dtype=np.float64)
    a = a[np.isfinite(a)]
    b = b[np.isfinite(b)]
    if a.size == 0 or b.size == 0:
        return None
    ha, _ = np.histogram(a, bins=edges)
    hb, _ = np.histogram(b, bins=edges)
    pa = ha.astype(np.float64) / ha.sum()
    pb = hb.astype(np.float64) / hb.sum()
    coeff = float(np.sum(np.sqrt(pa * pb)))
    return -math.log(max(coeff, 1e-12))


def distribution(values):
    data = np.asarray(values, dtype=np.float64)
    data = data[np.isfinite(data)]
    if data.size == 0:
        return {"count": 0}
    return {
        "count": int(data.size),
        "mean": float(data.mean()),
        "median": float(np.median(data)),
        "std": float(data.std(ddof=1)) if data.size > 1 else 0.0,
        "p05": float(np.percentile(data, 5)),
        "p25": float(np.percentile(data, 25)),
        "p75": float(np.percentile(data, 75)),
        "p95": float(np.percentile(data, 95)),
    }


def circular_mean_deg(hue_unit):
    data = np.asarray(hue_unit, dtype=np.float64)
    data = data[np.isfinite(data)]
    if data.size == 0:
        return None
    angle = data * 2.0 * math.pi
    mean = math.atan2(np.sin(angle).mean(), np.cos(angle).mean())
    if mean < 0.0:
        mean += 2.0 * math.pi
    return float(mean * 180.0 / math.pi)


def _sigmoid(logits):
    clipped = np.clip(logits, -30.0, 30.0)
    return 1.0 / (1.0 + np.exp(-clipped))


def fit_logistic(features, labels, l2=1.0, steps=30):
    """절편 있는 L2 logistic. C는 1로 고정하고 탐색하지 않는다."""
    x = np.asarray(features, dtype=np.float64)
    y = np.asarray(labels, dtype=np.float64)
    design = np.column_stack((np.ones(x.shape[0]), x))
    weights = np.zeros(design.shape[1], dtype=np.float64)
    penalty = np.ones(design.shape[1], dtype=np.float64)
    penalty[0] = 0.0
    for _ in range(steps):
        prob = _sigmoid(design @ weights)
        gradient = design.T @ (prob - y) + l2 * penalty * weights
        weight_diag = prob * (1.0 - prob)
        hessian = design.T @ (design * weight_diag[:, None]) + l2 * np.diag(penalty)
        try:
            step = np.linalg.solve(hessian, gradient)
        except np.linalg.LinAlgError:
            break
        weights -= step
        if np.linalg.norm(step) < 1e-6:
            break
    return weights


def predict_logistic(weights, features):
    design = np.column_stack((np.ones(len(features)), np.asarray(features, dtype=np.float64)))
    return _sigmoid(design @ weights) >= 0.5


def balanced_accuracy(predicted, labels):
    labels = np.asarray(labels, dtype=bool)
    predicted = np.asarray(predicted, dtype=bool)
    positive = labels.sum()
    negative = (~labels).sum()
    if positive == 0 or negative == 0:
        return None
    true_positive = np.logical_and(predicted, labels).sum() / positive
    true_negative = np.logical_and(~predicted, ~labels).sum() / negative
    return float(0.5 * (true_positive + true_negative))


def spatial_block_accuracy(features, labels, blocks):
    """사분면 block을 하나씩 빼고 평가한다. random point split이 아니다."""
    features = np.asarray(features, dtype=np.float64)
    labels = np.asarray(labels, dtype=np.int64)
    blocks = np.asarray(blocks, dtype=np.int64)
    finite = np.isfinite(features).all(axis=1)
    scores = []
    for block in range(4):
        test = finite & (blocks == block)
        train = finite & (blocks != block)
        if test.sum() < 10 or train.sum() < 20:
            continue
        if len(np.unique(labels[train])) < 2 or len(np.unique(labels[test])) < 2:
            continue
        mean = features[train].mean(axis=0)
        std = features[train].std(axis=0)
        std[std < 1e-8] = 1.0
        weights = fit_logistic((features[train] - mean) / std, labels[train])
        predicted = predict_logistic(weights, (features[test] - mean) / std)
        score = balanced_accuracy(predicted, labels[test])
        if score is not None:
            scores.append(score)
    if len(scores) < MIN_VALID_FOLDS:
        return {"folds": scores, "mean": None, "valid_folds": len(scores)}
    return {"folds": scores, "mean": float(np.mean(scores)), "valid_folds": len(scores)}


def grade(block_mean, valid_folds):
    if valid_folds < MIN_VALID_FOLDS or block_mean is None:
        return "LOW"
    if block_mean >= BLOCK_HIGH:
        return "HIGH"
    if block_mean >= BLOCK_MEDIUM:
        return "MEDIUM"
    return "LOW"


def _best(names, scores):
    ranked = sorted(names, key=lambda name: (-scores[name], SIMPLICITY[name], COLOR_TIE.index(name) if name in COLOR_TIE else MULTI_TIE.index(name)))
    best = ranked[0]
    tied = [name for name in ranked if scores[best] - scores[name] <= TIE_ACCURACY]
    order = [name for name in list(COLOR_TIE) + list(MULTI_TIE) if name in tied]
    if not order:
        return best
    return sorted(order, key=lambda name: (SIMPLICITY[name], order.index(name)))[0]


def select_method(global_grades, contact_grades, global_scores, contact_scores, contact_identifiable):
    """측정 전에 고정한 Case A/B/C. 등급 딕셔너리만 받는다."""
    if not contact_identifiable:
        return {
            "feature_separability": "NOT_ROBUSTLY_SEPARABLE",
            "selected_feature_family": "NONE",
            "selected_segmentation_strategy": "SEGMENTATION_STOP_RULE",
            "stop_rule_triggered": True,
            "reason": "contact_not_identifiable",
        }
    color_ok = [
        name
        for name in COLOR_SETS
        if global_grades[name] == "HIGH" and contact_grades[name] == "HIGH"
    ]
    if color_ok:
        family = _best(color_ok, contact_scores)
        return {
            "feature_separability": "COLOR_SEPARABLE",
            "selected_feature_family": family,
            "selected_segmentation_strategy": "COLOR_ASSISTED_ROI",
            "stop_rule_triggered": False,
            "reason": "color_high_on_interior_and_contact",
        }
    multi_ok = [
        name
        for name in MULTIMODAL_SETS
        if global_grades[name] == "HIGH" and contact_grades[name] == "HIGH"
    ]
    if multi_ok:
        family = _best(multi_ok, contact_scores)
        return {
            "feature_separability": "MULTIMODAL_SEPARABLE",
            "selected_feature_family": family,
            "selected_segmentation_strategy": "COLOR_GEOMETRY_ASSISTED_ROI",
            "stop_rule_triggered": False,
            "reason": "color_insufficient_combination_high",
        }
    return {
        "feature_separability": "NOT_ROBUSTLY_SEPARABLE",
        "selected_feature_family": "NONE",
        "selected_segmentation_strategy": "SEGMENTATION_STOP_RULE",
        "stop_rule_triggered": True,
        "reason": "no_set_high_on_interior_and_contact",
    }
