"""P-008B 측정 함수 synthetic 검사."""

import numpy as np

from p008b_metrics import (
    COLOR_MEDIAN_ABS_MAX,
    P95_MAX_M,
    POSITIVE_DEPTH_RATIO_MIN,
    color_correspondence,
    depth_cloud_metrics,
    extrinsic_rejection,
    quaternion_xyzw,
    unpack_rgb_little_endian,
)


def test_known_rotation_and_translation_are_accepted():
    angle = 0.04
    rotation = np.array(
        [[np.cos(angle), -np.sin(angle), 0.0], [np.sin(angle), np.cos(angle), 0.0], [0.0, 0.0, 1.0]]
    )
    assert extrinsic_rejection(rotation, np.array([0.024, 0.0, 0.0])) is None


def test_identity_fallback_is_rejected():
    assert extrinsic_rejection(np.eye(3), np.zeros(3)) == "identity_fallback"


def test_invalid_matrix_is_rejected():
    assert extrinsic_rejection(np.zeros((3, 3)), np.array([0.01, 0.0, 0.0])) == "invalid_matrix"


def test_quaternion_of_identity():
    quat = quaternion_xyzw(np.eye(3))
    assert abs(quat[3] - 1.0) < 1e-9


def test_depth_cloud_round_trip_passes_frozen_threshold():
    assert POSITIVE_DEPTH_RATIO_MIN == 0.99
    assert P95_MAX_M == 0.005
    depth = np.zeros((20, 30), dtype=np.uint16)
    depth[8, 12] = 1500
    fx = fy = 100.0
    cx, cy = 10.0, 6.0
    z = 1.5
    xyz = np.array([[(12 - cx) * z / fx, (8 - cy) * z / fy, z]])
    metrics = depth_cloud_metrics(xyz, depth, fx, fy, cx, cy)
    assert metrics["sample_count"] == 1
    assert metrics["positive_depth_ratio"] == 1.0
    assert metrics["p95_m"] < 1e-9
    assert metrics["passed"]


def test_zero_depth_pixel_fails_positive_ratio():
    depth = np.zeros((20, 30), dtype=np.uint16)
    xyz = np.array([[0.0, 0.0, 1.0]])
    metrics = depth_cloud_metrics(xyz, depth, 100.0, 100.0, 10.0, 6.0)
    assert metrics["positive_depth_ratio"] == 0.0
    assert not metrics["passed"]


def test_rgb_unpack_uses_ros_little_endian_offsets():
    point = np.zeros(16, dtype=np.uint8)
    point[12] = 7
    point[13] = 8
    point[14] = 9
    rgb = unpack_rgb_little_endian(point.tobytes(), 12, 16, 1)
    assert list(rgb[0]) == [9, 8, 7]


def test_color_match_passes_and_gross_mismatch_fails():
    assert COLOR_MEDIAN_ABS_MAX == 15.0
    image = np.zeros((20, 30, 3), dtype=np.uint8)
    image[8, 12] = (20, 30, 40)
    fx = fy = 100.0
    cx, cy = 10.0, 6.0
    z = 1.0
    xyz = np.array([[(12 - cx) * z / fx, (8 - cy) * z / fy, z]])
    good = color_correspondence(xyz, np.array([[20, 30, 40]], dtype=np.uint8), image, fx, fy, cx, cy)
    assert good["passed"]
    assert good["registration_accuracy"] == "NOT_CLAIMED"
    bad = color_correspondence(xyz, np.array([[200, 10, 10]], dtype=np.uint8), image, fx, fy, cx, cy)
    assert not bad["passed"]
