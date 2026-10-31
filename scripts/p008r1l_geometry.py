"""R1L 작업면 계산. 점군 파일은 읽지 않는다."""

import heapq
import math

import numpy as np


def boundary_clearance(uv, leaf):
    """평면 좌표에서 4-이웃 경계 셀 중심까지의 거리."""
    points = np.asarray(uv, dtype=np.float64)
    if leaf <= 0.0:
        raise ValueError("voxel 크기는 양수다.")
    if len(points) == 0:
        return np.zeros(0, dtype=np.float64)
    keys = np.floor(points / leaf).astype(np.int64)
    cells = {}
    for index, key in enumerate(map(tuple, keys)):
        cells.setdefault(key, []).append(index)
    boundary = []
    for key in cells:
        if any((key[0] + du, key[1] + dv) not in cells for du, dv in ((1, 0), (-1, 0), (0, 1), (0, -1))):
            boundary.append(key)
    centers = (np.asarray(boundary, dtype=np.float64) + 0.5) * leaf
    distances = np.empty(len(points), dtype=np.float64)
    for key, members in cells.items():
        center = (np.asarray(key, dtype=np.float64) + 0.5) * leaf
        dist = float(np.min(np.linalg.norm(centers - center, axis=1)))
        distances[members] = dist
    return distances


def usable_region_mask(valid_normal, normal_stable, density_ok, clearance_ok):
    """네 조건이 모두 참인 점만 작업면이다."""
    flags = [
        np.asarray(valid_normal, dtype=bool),
        np.asarray(normal_stable, dtype=bool),
        np.asarray(density_ok, dtype=bool),
        np.asarray(clearance_ok, dtype=bool),
    ]
    if len({len(flag) for flag in flags}) != 1:
        raise ValueError("판정 배열 길이가 다르다.")
    return flags[0] & flags[1] & flags[2] & flags[3]


def graph_extent(keys, leaf):
    """26-연결 최단 경로의 근사 지름. 끊긴 성분은 넘지 않는다."""
    cells = list(dict.fromkeys(tuple(key) for key in keys))
    if leaf <= 0.0:
        raise ValueError("voxel 크기는 양수다.")
    if len(cells) < 2:
        return 0.0, []
    present = set(cells)
    offsets = [
        (i, j, k)
        for i in (-1, 0, 1)
        for j in (-1, 0, 1)
        for k in (-1, 0, 1)
        if not (i == 0 and j == 0 and k == 0)
    ]
    weights = {step: leaf * math.sqrt(step[0] ** 2 + step[1] ** 2 + step[2] ** 2) for step in offsets}

    def farthest(start):
        dist = {start: 0.0}
        prev = {start: None}
        heap = [(0.0, start)]
        while heap:
            accrued, node = heapq.heappop(heap)
            if accrued != dist.get(node):
                continue
            for step, weight in weights.items():
                nxt = (node[0] + step[0], node[1] + step[1], node[2] + step[2])
                if nxt not in present:
                    continue
                updated = accrued + weight
                if updated < dist.get(nxt, math.inf):
                    dist[nxt] = updated
                    prev[nxt] = node
                    heapq.heappush(heap, (updated, nxt))
        end = max(dist, key=lambda key: (dist[key], key))
        return end, dist[end], prev

    first, _, _ = farthest(cells[0])
    second, length, prev = farthest(first)
    path = []
    node = second
    while node is not None:
        path.append(node)
        node = prev[node]
    path.reverse()
    return float(length), path


def normal_consistency(angles_deg, unstable_deg):
    """각도 표본의 중앙값, P95, 불안정 비율."""
    if unstable_deg < 0.0:
        raise ValueError("불안정 각은 0 이상이다.")
    angles = np.asarray(angles_deg, dtype=np.float64)
    angles = angles[np.isfinite(angles)]
    if len(angles) == 0:
        return {"count": 0, "median_deg": None, "p95_deg": None, "unstable_ratio": None}
    return {
        "count": int(len(angles)),
        "median_deg": float(np.median(angles)),
        "p95_deg": float(np.percentile(angles, 95)),
        "unstable_ratio": float(np.mean(angles > unstable_deg)),
    }
