#!/usr/bin/env python3
"""workplane과 depth optical frame 사이의 rigid 식만 다룬다. scale은 없다."""

import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SENSOR_THRESHOLDS = ROOT / "evidence/P-006/camera-target/thresholds.yaml"
SENSOR_SHA256 = "303d1a4067b05d81cbbbb8ff24ae92ddbf34712b2c91ea0c470cb021a007b539"
ROI_X_M = (0.0, 0.400)
ROI_Y_M = (0.300, 0.800)
SQUARE_M = 0.0200

REG_IDS = ("REG_A", "REG_B", "REG_C")
VAL_REQUIRED = ("VAL_A", "VAL_B")
VAL_OPTIONAL = ("VAL_C",)
ALL_IDS = REG_IDS + VAL_REQUIRED + VAL_OPTIONAL


def sha256_file(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sensor_thresholds_unchanged():
    return sha256_file(SENSOR_THRESHOLDS) == SENSOR_SHA256


def se3(rotation, translation):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(rotation, dtype=np.float64)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return transform


def invert(transform):
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -(rotation.T @ translation)
    return inverse


def workplane_from_board(origin_m, yaw_deg):
    """보드 축을 작업면 축에 yaw만큼 돌린다. +Z는 작업면 위다. scale은 1이다."""
    yaw = math.radians(float(yaw_deg))
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotation = np.array([
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    return se3(rotation, origin_m)


def workplane_from_depth(workplane_board, depth_board):
    """p_workplane = T_workplane_board * inv(T_depth_board) * p_depth."""
    return workplane_board @ invert(depth_board)


def project_rotation(matrix):
    """SVD로 가장 가까운 회전을 구한다. det가 음수면 반사로 보고 뒤집지 않는다."""
    left, _, right = np.linalg.svd(np.asarray(matrix, dtype=np.float64))
    return left @ right


def mean_rotation(rotations):
    total = np.sum(np.asarray(rotations, dtype=np.float64), axis=0)
    return project_rotation(total)


def mean_translation(translations):
    return np.mean(np.asarray(translations, dtype=np.float64), axis=0)


def matrix_max_abs(matrix):
    return float(np.max(np.abs(matrix)))


def orthogonality_error(rotation):
    return matrix_max_abs(rotation.T @ rotation - np.eye(3))


def inverse_consistency_error(transform):
    forward = transform @ invert(transform) - np.eye(4)
    backward = invert(transform) @ transform - np.eye(4)
    return max(matrix_max_abs(forward), matrix_max_abs(backward))


def geodesic_deg(reference, other):
    relative = np.asarray(reference, dtype=np.float64).T @ np.asarray(other, dtype=np.float64)
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def directed_angle_deg(vector, reference):
    left = np.asarray(vector, dtype=np.float64)
    right = np.asarray(reference, dtype=np.float64)
    cosine = float(np.clip(np.dot(left, right) / (np.linalg.norm(left) * np.linalg.norm(right)), -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def finite_vector(values):
    array = np.asarray(values, dtype=np.float64).reshape(-1)
    return bool(array.size) and bool(np.all(np.isfinite(array)))


def triangle_area(points):
    first, second, third = [np.asarray(point, dtype=np.float64) for point in points]
    return abs(float((second[0] - first[0]) * (third[1] - first[1]) - (third[0] - first[0]) * (second[1] - first[1]))) / 2.0


def pairwise_min(points):
    best = None
    for index, left in enumerate(points):
        for right in points[index + 1:]:
            distance = float(np.linalg.norm(np.asarray(left) - np.asarray(right)))
            best = distance if best is None else min(best, distance)
    return best


def paper_corners_workplane(origin_xyz, yaw_deg, margins):
    """여백은 보드 축 기준이다. yaw가 0이 아니면 작업면으로 돌려서 ROI를 본다."""
    if any(margins.get(key) is None for key in ("minus_x", "plus_x", "minus_y", "plus_y")):
        return None
    local = np.array([
        [-float(margins["minus_x"]), -float(margins["minus_y"]), 0.0],
        [float(margins["plus_x"]), -float(margins["minus_y"]), 0.0],
        [float(margins["plus_x"]), float(margins["plus_y"]), 0.0],
        [-float(margins["minus_x"]), float(margins["plus_y"]), 0.0],
    ], dtype=np.float64)
    transform = workplane_from_board(origin_xyz, yaw_deg)
    homogeneous = np.concatenate([local, np.ones((4, 1))], axis=1)
    return (transform @ homogeneous.T).T[:, :3]


def paper_inside_roi(origin_xyz, yaw_deg, margins):
    corners = paper_corners_workplane(origin_xyz, yaw_deg, margins)
    if corners is None:
        return None
    return bool(
        np.min(corners[:, 0]) >= ROI_X_M[0]
        and np.max(corners[:, 0]) <= ROI_X_M[1]
        and np.min(corners[:, 1]) >= ROI_Y_M[0]
        and np.max(corners[:, 1]) <= ROI_Y_M[1]
    )


def number_or_none(value):
    if value is None or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def point_xyz(record):
    if not isinstance(record, dict):
        return None
    values = [number_or_none(record.get(key)) for key in ("x_m", "y_m", "z_m")]
    if any(value is None for value in values):
        return None
    return values


def anchor_problems(definition, rules):
    """측정되지 않은 좌표는 통과시키지 않는다."""
    problems = []
    margins = definition.get("paper_margin_from_origin_m") or {}
    if any(number_or_none(margins.get(key)) is None for key in ("minus_x", "plus_x", "minus_y", "plus_y")):
        problems.append("paper_margin_unmeasured")
    alignment = definition.get("alignment") or {}
    if alignment.get("operator_confirmed") is not True:
        problems.append("alignment_unconfirmed")
    if number_or_none(alignment.get("yaw_deg")) is None:
        problems.append("yaw_unmeasured")
    anchors = definition.get("anchors") or {}
    points = {}
    for name in REG_IDS + VAL_REQUIRED:
        point = point_xyz(anchors.get(name))
        if point is None:
            problems.append(f"{name}_unmeasured")
        else:
            points[name] = point
    optional = point_xyz(anchors.get("VAL_C"))
    if anchors.get("VAL_C") is not None and optional is None:
        problems.append("VAL_C_incomplete")
    if optional is not None:
        points["VAL_C"] = optional
    if problems:
        return problems
    reg = [points[name][:2] for name in REG_IDS]
    if pairwise_min(reg) < float(rules["reg_min_pairwise_distance_m"]):
        problems.append("reg_too_close")
    if triangle_area(reg) < float(rules["reg_min_triangle_area_m2"]):
        problems.append("reg_collinear")
    numeric_margins = {key: float(margins[key]) for key in ("minus_x", "plus_x", "minus_y", "plus_y")}
    yaw = float(alignment["yaw_deg"])
    for name, point in points.items():
        inside = paper_inside_roi(point, yaw, numeric_margins)
        if inside is not True:
            problems.append(f"{name}_paper_outside_roi")
    for val_name in VAL_REQUIRED + (("VAL_C",) if "VAL_C" in points else ()):
        for reg_name in REG_IDS:
            distance = float(np.linalg.norm(np.asarray(points[val_name][:2]) - np.asarray(points[reg_name][:2])))
            if distance < float(rules["val_min_distance_from_reg_m"]):
                problems.append(f"{val_name}_near_{reg_name}")
    return problems


def self_check():
    if not sensor_thresholds_unchanged():
        raise SystemExit("sensor threshold hash가 바뀌었다")
    yaw = workplane_from_board([0.10, 0.40, 0.0], 0.0)
    depth_board = se3(np.eye(3), [0.0, 0.0, 0.45])
    found = workplane_from_depth(yaw, depth_board)
    expected = yaw @ invert(depth_board)
    if matrix_max_abs(found - expected) > 1e-12:
        raise SystemExit("transform 합성이 어긋난다")
    if abs(float(np.linalg.det(found[:3, :3])) - 1.0) > 1e-12:
        raise SystemExit("det가 +1이 아니다")
    if inverse_consistency_error(found) > 1e-9:
        raise SystemExit("inverse consistency가 깨졌다")
    if orthogonality_error(found[:3, :3]) > 1e-12:
        raise SystemExit("orthogonality가 깨졌다")
    repeated = mean_rotation([found[:3, :3], found[:3, :3], found[:3, :3]])
    if matrix_max_abs(repeated - found[:3, :3]) > 1e-9:
        raise SystemExit("같은 회전의 평균이 달라졌다")
    mirrored = np.diag([1.0, 1.0, -1.0])
    if float(np.linalg.det(project_rotation(mirrored))) > 0.0:
        raise SystemExit("반사를 회전으로 고쳤다")
    collinear = triangle_area([[0.0, 0.0], [0.2, 0.0], [0.4, 0.0]])
    if collinear >= 0.004:
        raise SystemExit("일직선 면적 검사가 없다")
    spread = triangle_area([[0.05, 0.35], [0.25, 0.35], [0.15, 0.55]])
    if spread < 0.004:
        raise SystemExit("떨어진 삼각형을 거절했다")
    margins = {"minus_x": 0.02, "plus_x": 0.19, "minus_y": 0.03, "plus_y": 0.25}
    if paper_inside_roi((0.10, 0.40, 0.0), 0.0, margins) is not True:
        raise SystemExit("ROI 안 종이를 거절했다")
    if paper_inside_roi((0.10, 0.31, 0.0), 0.0, margins) is not False:
        raise SystemExit("ROI 밖 종이를 통과시켰다")
    if paper_inside_roi((0.10, 0.40, 0.0), 0.0, {"minus_x": None, "plus_x": 0.1, "minus_y": 0.1, "plus_y": 0.1}) is not None:
        raise SystemExit("없는 여백으로 포함 여부를 만들었다")
    origin = np.array([0.0, 0.0, 0.0])
    moved = (found @ np.array([0.0, 0.0, 0.45, 1.0]))[:3]
    if np.linalg.norm(moved - np.array([0.10, 0.40, 0.0])) > 1e-9:
        raise SystemExit("원점 변환이 어긋난다")
    if not finite_vector(moved) or not math.isfinite(float(origin[0])):
        raise SystemExit("유한성 검사가 없다")
    empty = {
        "paper_margin_from_origin_m": {"minus_x": None, "plus_x": None, "minus_y": None, "plus_y": None},
        "alignment": {"operator_confirmed": False, "yaw_deg": None},
        "anchors": {name: None for name in ALL_IDS},
    }
    rules = {"reg_min_pairwise_distance_m": 0.1, "reg_min_triangle_area_m2": 0.004, "val_min_distance_from_reg_m": 0.05}
    if "paper_margin_unmeasured" not in anchor_problems(empty, rules):
        raise SystemExit("빈 앵커를 측정값으로 봤다")
    print("SELF_CHECK_OK")


def check_anchors():
    definition = json.loads((ROOT / "evidence/P-006C-R2.5/anchor_definition.json").read_text())
    rules = json.loads((ROOT / "evidence/P-006C-R2.5/thresholds.json").read_text())
    freeze = json.loads((ROOT / "evidence/P-006C-R2.5/threshold_freeze.json").read_text())
    problems = anchor_problems(definition, rules["placement_preconditions"])
    if not freeze.get("commit"):
        problems.append("threshold_freeze_commit_missing")
    if problems or definition.get("coordinates_recorded") is not True:
        print("OPERATOR_ANCHOR_REQUIRED")
        for item in problems:
            print(item)
        return 2
    print("ANCHORS_READY")
    return 0


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--check-anchors":
        raise SystemExit(check_anchors())
    self_check()
