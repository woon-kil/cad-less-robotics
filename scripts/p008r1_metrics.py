#!/usr/bin/env python3
"""P-008-R1 경계 지표. 합성 도형 테스트와 capture_01 결과 검증에만 쓴다."""

import math

import numpy as np


def canonical_axis(vector):
    sign = 1.0
    for value in vector:
        if abs(value) > 1e-8:
            sign = 1.0 if value > 0.0 else -1.0
            break
    return vector * sign


def pca_frame(points):
    """동일 입력에서 축 부호가 뒤집히지 않는 workpiece frame."""
    xyz = np.asarray(points, dtype=np.float64)
    centroid = xyz.mean(axis=0)
    centered = xyz - centroid
    covariance = (centered.T @ centered) / float(xyz.shape[0])
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    order = np.argsort(eigenvalues)
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]
    axis0 = canonical_axis(eigenvectors[:, 2].copy())
    axis2 = eigenvectors[:, 0].copy()
    if float(np.dot(axis2, centroid)) > 0.0:
        axis2 = -axis2
    axis1 = np.cross(axis2, axis0)
    axes = np.column_stack((axis0, axis1, axis2))
    if np.linalg.det(axes) < 0.0:
        axis1 = -axis1
    return {
        "centroid": centroid,
        "eigenvalues": eigenvalues,
        "axis_0": axis0,
        "axis_1": axis1,
        "axis_2": axis2,
    }


def _orient(a, b, c):
    value = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    if abs(value) <= 1e-12:
        return 0
    return 1 if value > 0.0 else -1


def _proper_cross(a, b, c, d):
    o1 = _orient(a, b, c)
    o2 = _orient(a, b, d)
    o3 = _orient(c, d, a)
    o4 = _orient(c, d, b)
    return o1 * o2 < 0 and o3 * o4 < 0


def self_intersection_count(uv):
    polygon = np.asarray(uv, dtype=np.float64)
    count = polygon.shape[0]
    if count < 4:
        return 0
    hits = 0
    for i in range(count):
        a = polygon[i]
        b = polygon[(i + 1) % count]
        for j in range(i + 1, count):
            if abs(i - j) <= 1 or (i == 0 and j == count - 1):
                continue
            c = polygon[j]
            d = polygon[(j + 1) % count]
            if _proper_cross(a, b, c, d):
                hits += 1
    return hits


def signed_area(uv):
    polygon = np.asarray(uv, dtype=np.float64)
    nxt = np.roll(polygon, -1, axis=0)
    return float(0.5 * np.sum(polygon[:, 0] * nxt[:, 1] - nxt[:, 0] * polygon[:, 1]))


def spacing(points, closed=True):
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.shape[0] < 2:
        return np.zeros(0)
    if closed:
        return np.linalg.norm(np.roll(xyz, -1, axis=0) - xyz, axis=1)
    return np.linalg.norm(xyz[1:] - xyz[:-1], axis=1)


def support_distances(boundary, cloud):
    boundary = np.asarray(boundary, dtype=np.float64)
    cloud = np.asarray(cloud, dtype=np.float64)
    distances = np.empty(boundary.shape[0], dtype=np.float64)
    block = 256
    for start in range(0, boundary.shape[0], block):
        chunk = boundary[start:start + block]
        delta = chunk[:, None, :] - cloud[None, :, :]
        distances[start:start + chunk.shape[0]] = np.sqrt(np.min(np.sum(delta * delta, axis=2), axis=1))
    return distances


def distribution(values):
    data = np.asarray(values, dtype=np.float64)
    if data.size == 0:
        return {"mean": None, "median": None, "p95": None, "max": None}
    return {
        "mean": float(np.mean(data)),
        "median": float(np.median(data)),
        "p95": float(np.percentile(data, 95)),
        "max": float(np.max(data)),
    }


def duplicate_consecutive(uv, tol=1e-12):
    polygon = np.asarray(uv, dtype=np.float64)
    if polygon.shape[0] < 2:
        return 0
    nxt = np.roll(polygon, -1, axis=0)
    return int(np.count_nonzero(np.linalg.norm(nxt - polygon, axis=1) <= tol))


def finite_coordinates(points):
    return bool(np.isfinite(np.asarray(points, dtype=np.float64)).all())


def closed_gap(points):
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.shape[0] < 3:
        return math.inf
    return float(np.linalg.norm(xyz[0] - xyz[-1]))
