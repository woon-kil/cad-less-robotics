"""R1N 절단 단면 계산. 점군 파일은 읽지 않는다."""

import math

import numpy as np


def orient_axis(vector):
    """카메라 x가 양수가 되도록 축 방향을 고정한다."""
    direction = np.asarray(vector, dtype=np.float64)
    norm = float(np.linalg.norm(direction))
    if norm == 0.0 or not np.isfinite(norm):
        raise ValueError("축 길이가 0이다.")
    direction = direction / norm
    for component in direction:
        if component > 1e-12:
            return direction
        if component < -1e-12:
            return -direction
    return direction


def pca_in_plane(uv):
    """평면 좌표의 첫 주성분. 길이 2 이상이어야 한다."""
    points = np.asarray(uv, dtype=np.float64)
    if len(points) < 2:
        raise ValueError("축을 만들 점이 부족하다.")
    centered = points - points.mean(axis=0)
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    return vt[0]


def frame_coordinates(points, origin, longitudinal, lateral, normal, intercept):
    """절단 좌표 t, 가로 s, 지지 높이 h."""
    delta = np.asarray(points, dtype=np.float64) - np.asarray(origin, dtype=np.float64)
    along = delta @ np.asarray(longitudinal, dtype=np.float64)
    across = delta @ np.asarray(lateral, dtype=np.float64)
    height = np.asarray(points, dtype=np.float64) @ np.asarray(normal, dtype=np.float64) + float(intercept)
    return along, across, height


def slice_centers(start, end, spacing):
    """평가 구간 안의 절단 위치. 오름차순이다."""
    if spacing <= 0.0 or end <= start:
        return []
    count = int(math.floor((end - start) / spacing))
    return [start + (index + 0.5) * spacing for index in range(count) if start + (index + 0.5) * spacing <= end]


def local_slope_deg(position, height, index, neighborhood):
    """이웃 구간의 기울기 절댓값."""
    if neighborhood <= 0.0:
        raise ValueError("이웃 길이는 양수다.")
    lo = position[index] - neighborhood
    hi = position[index] + neighborhood
    chosen = (position >= lo) & (position <= hi)
    if int(np.count_nonzero(chosen)) < 3:
        return None
    ds = position[chosen] - position[index]
    dh = height[chosen] - height[index]
    denom = float(ds @ ds)
    if denom <= 1e-12:
        return None
    return float(np.degrees(np.arctan(abs(float(ds @ dh) / denom))))


def side_transition(position, height, neighborhood, steep_deg, shallow_deg):
    """바깥에서 안쪽으로 가며 가파른 뒤 처음으로 얕아지는 칸."""
    slopes = [local_slope_deg(position, height, index, neighborhood) for index in range(len(position))]
    center = 0.5 * (float(position[0]) + float(position[-1]))
    sides = {
        "positive": [index for index in range(len(position) - 1, -1, -1) if position[index] >= center],
        "negative": [index for index in range(len(position)) if position[index] <= center],
    }
    found = {}
    for name, indices in sides.items():
        steepest = None
        for index in indices:
            slope = slopes[index]
            if slope is None:
                continue
            if steepest is None:
                if slope >= steep_deg:
                    steepest = slope
                continue
            if slope >= steep_deg:
                steepest = max(steepest, slope)
                continue
            if slope <= shallow_deg:
                found[name] = {"index": index, "score_deg": float(steepest - slope)}
                break
    return found


def dominant_side(found, margin_deg):
    """한쪽만 있거나 점수 차가 충분하면 그 쪽을 고른다."""
    if not found:
        return None, "missing"
    if len(found) == 1:
        return next(iter(found)), "unique"
    names = list(found)
    scores = [found[name]["score_deg"] for name in names]
    winner = names[int(np.argmax(scores))]
    if max(scores) - min(scores) < margin_deg:
        return None, "ambiguous"
    return winner, "unique"


def max_consecutive(labels, target):
    best = 0
    run = 0
    for label in labels:
        if label == target:
            run += 1
            best = max(best, run)
        else:
            run = 0
    return best


def spacing_stats(points):
    """순서대로 놓은 점의 3D 간격."""
    values = np.asarray(points, dtype=np.float64)
    if len(values) < 2:
        return {"median_m": None, "p95_m": None, "max_m": None}
    gaps = np.linalg.norm(np.diff(values, axis=0), axis=1)
    return {
        "median_m": float(np.median(gaps)),
        "p95_m": float(np.percentile(gaps, 95)),
        "max_m": float(np.max(gaps)),
    }


def interval_coverage(centers, spacing, start, end):
    """특징 위치 주변 간격만큼의 합집합이 평가 구간에서 차지하는 비율."""
    span = float(end) - float(start)
    if span <= 0.0 or spacing <= 0.0 or len(centers) == 0:
        return 0.0
    half = spacing / 2.0
    segments = []
    for center in centers:
        lo = max(float(start), float(center) - half)
        hi = min(float(end), float(center) + half)
        if hi > lo:
            segments.append((lo, hi))
    if not segments:
        return 0.0
    segments.sort()
    total = 0.0
    current_lo, current_hi = segments[0]
    for lo, hi in segments[1:]:
        if lo <= current_hi:
            current_hi = max(current_hi, hi)
        else:
            total += current_hi - current_lo
            current_lo, current_hi = lo, hi
    total += current_hi - current_lo
    return float(total / span)


def jump_limit(spacing):
    return max(3.0 * float(spacing), 0.020)
