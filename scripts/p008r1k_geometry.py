"""R1K 기하 계산. 점군 파일은 읽지 않는다."""

import math

import numpy as np


def signed_plane_height(points, normal, intercept):
    """h = n·p + d. n은 단위 벡터다. 카메라 쪽이 양수다."""
    values = np.asarray(points, dtype=np.float64)
    direction = np.asarray(normal, dtype=np.float64)
    norm = np.linalg.norm(direction)
    if norm == 0.0 or not np.isfinite(norm):
        raise ValueError("법선 길이가 0이다.")
    direction = direction / norm
    height = values @ direction + float(intercept)
    height = height.astype(np.float64)
    height[~np.isfinite(values).all(axis=1)] = np.nan
    return height


def orient_toward_camera(normal, point_on_plane):
    """평면 위의 점에서 카메라 원점으로 가는 방향과 n의 부호를 맞춘다."""
    direction = np.asarray(normal, dtype=np.float64)
    direction = direction / np.linalg.norm(direction)
    toward = -np.asarray(point_on_plane, dtype=np.float64)
    if direction @ toward < 0.0:
        direction = -direction
    return direction


def voxel_keys(points, leaf):
    if leaf <= 0.0:
        raise ValueError("voxel 크기는 양수다.")
    coords = np.floor(np.asarray(points, dtype=np.float64) / leaf).astype(np.int64)
    return [tuple(row) for row in coords]


def connected_components(keys):
    """26-연결. 입력 순서를 유지하는 결정적 BFS."""
    cells = list(dict.fromkeys(keys))
    index = {key: i for i, key in enumerate(cells)}
    seen = np.zeros(len(cells), dtype=bool)
    components = []
    offsets = [
        (i, j, k)
        for i in (-1, 0, 1)
        for j in (-1, 0, 1)
        for k in (-1, 0, 1)
        if not (i == 0 and j == 0 and k == 0)
    ]
    for start in range(len(cells)):
        if seen[start]:
            continue
        seen[start] = True
        queue = [start]
        members = []
        cursor = 0
        while cursor < len(queue):
            current = queue[cursor]
            cursor += 1
            members.append(cells[current])
            base = cells[current]
            for step in offsets:
                nxt = (base[0] + step[0], base[1] + step[1], base[2] + step[2])
                other = index.get(nxt)
                if other is not None and not seen[other]:
                    seen[other] = True
                    queue.append(other)
        components.append(members)
    return components


def polyline_distances(samples, curve):
    """각 샘플에서 닫힌 폴리라인까지의 최단 거리."""
    points = np.asarray(samples, dtype=np.float64)
    nodes = np.asarray(curve, dtype=np.float64)
    if len(nodes) < 2:
        raise ValueError("폴리라인 점이 부족하다.")
    closed = np.vstack((nodes, nodes[0]))
    best = np.full(len(points), np.inf)
    for start, end in zip(closed[:-1], closed[1:]):
        segment = end - start
        length2 = float(segment @ segment)
        if length2 == 0.0:
            dist = np.linalg.norm(points - start, axis=1)
        else:
            t = np.clip((points - start) @ segment / length2, 0.0, 1.0)
            closest = start + t[:, None] * segment
            dist = np.linalg.norm(points - closest, axis=1)
        best = np.minimum(best, dist)
    return best


def angle_deg(left, right):
    a = np.asarray(left, dtype=np.float64)
    b = np.asarray(right, dtype=np.float64)
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    cosine = float(np.clip(a @ b, -1.0, 1.0))
    return math.degrees(math.acos(cosine))
