"""P-009 표면 연관과 공정 프레임. 계약 숫자는 호출자가 넘긴다."""

import math

import numpy as np


FORBIDDEN_SPEC_KEYS = {
    "xyz",
    "final_xyz",
    "surface_anchor",
    "surface_anchors",
    "command_position",
    "command_positions",
    "normal",
    "normals",
    "tangent",
    "tangents",
    "quaternion",
    "quaternions",
    "joint",
    "joints",
    "trajectory",
    "point_index",
    "point_indices",
    "cad_pose",
    "traced_path",
}


def spec_forbidden(spec):
    """금지 입력이 있으면 그 키를 돌려준다."""
    found = []
    for key, value in spec.items():
        if key in FORBIDDEN_SPEC_KEYS and value not in (None, [], {}):
            found.append(key)
        if isinstance(value, dict):
            found.extend(spec_forbidden(value))
    return found


def orient_normal_toward_camera(normal, centroid):
    """법선이 표면에서 카메라(원점) 쪽을 보게 한다."""
    direction = np.asarray(normal, dtype=np.float64)
    direction = direction / np.linalg.norm(direction)
    if float(direction @ (-np.asarray(centroid, dtype=np.float64))) < 0.0:
        direction = -direction
    return direction


class NeighborIndex:
    """고정 잎 크기의 이웃 검색. 반경을 합격 기준으로 늘리지 않는다."""

    def __init__(self, points, leaf):
        self.points = np.asarray(points, dtype=np.float64)
        self.leaf = float(leaf)
        self.bins = {}
        if len(self.points) == 0:
            return
        keys = np.floor(self.points / self.leaf).astype(np.int64)
        for index, key in enumerate(map(tuple, keys)):
            self.bins.setdefault(key, []).append(index)

    def nearest(self, target, count, measure_radius):
        """측정 반경 안의 가까운 점. 반경 밖은 후보에 넣지 않는다."""
        point = np.asarray(target, dtype=np.float64)
        span = int(math.ceil(measure_radius / self.leaf))
        center = np.floor(point / self.leaf).astype(np.int64)
        chosen = []
        for ix in range(center[0] - span, center[0] + span + 1):
            for iy in range(center[1] - span, center[1] + span + 1):
                for iz in range(center[2] - span, center[2] + span + 1):
                    chosen.extend(self.bins.get((ix, iy, iz), ()))
        if not chosen:
            return np.zeros(0, dtype=np.int64), np.zeros(0, dtype=np.float64)
        ids = np.asarray(chosen, dtype=np.int64)
        distance = np.linalg.norm(self.points[ids] - point, axis=1)
        inside = distance <= measure_radius + 1e-12
        ids = ids[inside]
        distance = distance[inside]
        order = np.argsort(distance, kind="mergesort")
        order = order[:count]
        return ids[order], distance[order]


def associate_target(target, index, k, min_neighbors, max_kth, measure_radius):
    """kNN 국소 평면에 직교 투영한다. 지지가 부족하면 거절한다."""
    point = np.asarray(target, dtype=np.float64)
    ids, distance = index.nearest(point, k, measure_radius)
    count = int(len(ids))
    kth = float(distance[k - 1]) if count >= k else None
    record = {
        "neighbor_count": count,
        "kth_neighbor_distance_m": kth,
        "nearest_source_point_index": int(ids[0]) if count else None,
        "status": "REJECTED_UNSUPPORTED_GEOMETRY",
        "eigenvalues": None,
        "normal": None,
        "surface_anchor": None,
        "source_surface_distance_m": None,
    }
    if not np.isfinite(point).all() or count < min_neighbors or kth is None or kth > max_kth:
        return record
    neighbors = index.points[ids[:k]]
    centroid = neighbors.mean(axis=0)
    covariance = (neighbors - centroid).T @ (neighbors - centroid) / float(k)
    values, vectors = np.linalg.eigh(covariance)
    record["eigenvalues"] = values.tolist()
    if not np.isfinite(values).all() or float(values[1]) <= 1e-12:
        return record
    normal = orient_normal_toward_camera(vectors[:, 0], centroid)
    anchor = point - float((point - centroid) @ normal) * normal
    if not np.isfinite(anchor).all():
        return record
    record["normal"] = normal
    record["surface_anchor"] = anchor
    record["source_surface_distance_m"] = float(np.linalg.norm(neighbors - anchor, axis=1).min())
    record["status"] = "ACCEPTED"
    return record


def offset_target(anchor, y_axis, offset_m):
    """접선 가로 방향으로 요청점을 옮긴다. 표면 점은 다시 연관해야 한다."""
    return np.asarray(anchor, dtype=np.float64) + float(offset_m) * np.asarray(y_axis, dtype=np.float64)


def tangents_from_anchors(anchors, normals):
    """anchor 차분을 법선 평면에 투영한다."""
    points = np.asarray(anchors, dtype=np.float64)
    directions = np.zeros_like(points)
    count = len(points)
    for index in range(count):
        if count == 1:
            raw = np.zeros(3)
        elif index == 0:
            raw = points[1] - points[0]
        elif index == count - 1:
            raw = points[-1] - points[-2]
        else:
            raw = points[index + 1] - points[index - 1]
        normal = np.asarray(normals[index], dtype=np.float64)
        tangent = raw - float(raw @ normal) * normal
        norm = float(np.linalg.norm(tangent))
        directions[index] = tangent / norm if norm > 1e-12 else np.full(3, np.nan)
    return directions


def process_frame(tangent, normal):
    """오른손 공정 축. 퇴화면 None."""
    z_axis = -np.asarray(normal, dtype=np.float64)
    z_norm = float(np.linalg.norm(z_axis))
    if z_norm <= 1e-12:
        return None
    z_axis = z_axis / z_norm
    y_axis = np.cross(z_axis, np.asarray(tangent, dtype=np.float64))
    y_norm = float(np.linalg.norm(y_axis))
    if y_norm <= 1e-12:
        return None
    y_axis = y_axis / y_norm
    x_axis = np.cross(y_axis, z_axis)
    x_norm = float(np.linalg.norm(x_axis))
    if x_norm <= 1e-12:
        return None
    x_axis = x_axis / x_norm
    return np.column_stack((x_axis, y_axis, z_axis))


def quaternion_xyzw(rotation):
    """열벡터가 X, Y, Z인 회전의 quaternion [x, y, z, w]."""
    matrix = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    quat = np.array([x, y, z, w], dtype=np.float64)
    return quat / np.linalg.norm(quat)


def align_quaternion_sign(current, previous):
    """같은 회전의 부호만 맞춘다."""
    sign = np.asarray(current, dtype=np.float64)
    if previous is not None and float(sign @ previous) < 0.0:
        sign = -sign
    return sign


def command_from_anchor(anchor, normal, standoff_m):
    """양수 standoff는 법선 방향, 즉 카메라 쪽으로 옮긴다."""
    return np.asarray(anchor, dtype=np.float64) + float(standoff_m) * np.asarray(normal, dtype=np.float64)


def standoff_residuals(anchor, command, normal, standoff_m):
    """법선 성분 오차와 가로 잔차."""
    delta = np.asarray(command, dtype=np.float64) - np.asarray(anchor, dtype=np.float64)
    unit = np.asarray(normal, dtype=np.float64)
    unit = unit / np.linalg.norm(unit)
    along = float(delta @ unit)
    lateral = delta - along * unit
    return abs(along - float(standoff_m)), float(np.linalg.norm(lateral))


def consecutive_spacing(points):
    xyz = np.asarray(points, dtype=np.float64)
    if len(xyz) < 2:
        return np.zeros(0, dtype=np.float64)
    return np.linalg.norm(np.diff(xyz, axis=0), axis=1)


def spacing_within_tolerance(spacings, low, high):
    """마지막 구간을 뺀 내부 간격의 허용 비율."""
    values = np.asarray(spacings, dtype=np.float64)
    if len(values) < 2:
        return 0.0
    interior = values[:-1]
    return float(np.count_nonzero((interior >= low) & (interior <= high)) / len(interior))


def max_position_jump(spacings):
    values = np.asarray(spacings, dtype=np.float64)
    if len(values) == 0:
        return 0.0
    return float(values.max())


def geodesic_deg(q0, q1):
    dot = abs(float(np.asarray(q0) @ np.asarray(q1)))
    dot = min(1.0, max(-1.0, dot))
    return float(np.degrees(2.0 * math.acos(dot)))


def smoothing_within_limit(before, after, limit_m):
    shift = np.linalg.norm(np.asarray(after, dtype=np.float64) - np.asarray(before, dtype=np.float64), axis=1)
    return bool(len(shift) == 0 or float(shift.max()) <= limit_m), shift
