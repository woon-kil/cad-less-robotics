"""P-008-R1G correspondence 함수의 synthetic 검사."""

import numpy as np

from p008r1g_correspondence import (
    apply_rigid,
    assert_reference_capture,
    backproject_pinhole,
    depth_cloud_consistency,
    inside_image,
    project_pinhole,
)


FX = FY = 100.0
CX = 50.0
CY = 40.0


def test_known_intrinsics_round_trip():
    point = backproject_pinhole(np.array([60.0]), np.array([40.0]), np.array([2.0]), FX, FY, CX, CY)
    assert abs(point[0, 0] - 0.2) < 1e-12
    assert abs(point[0, 1]) < 1e-12
    u, v, valid = project_pinhole(point, FX, FY, CX, CY)
    assert valid[0]
    assert abs(u[0] - 60.0) < 1e-9
    assert abs(v[0] - 40.0) < 1e-9


def test_known_extrinsic_translation():
    point = np.array([[0.2, 0.0, 2.0]])
    moved = apply_rigid(point, np.eye(3), np.array([0.01, -0.02, 0.03]))
    assert np.allclose(moved, np.array([[0.21, -0.02, 2.03]]))


def test_identity_transform():
    point = np.array([[0.2, -0.1, 1.5]])
    assert np.allclose(apply_rigid(point, np.eye(3), np.zeros(3)), point)


def test_invalid_depth_is_not_projected():
    invalid = np.array([[0.1, 0.0, 0.0], [0.1, 0.0, -0.4], [np.nan, 0.0, 1.0]])
    _, _, valid = project_pinhole(invalid, FX, FY, CX, CY)
    assert not valid.any()


def test_image_bound_rejection():
    mask, _, _ = inside_image(np.array([-1.0, 0.0, 20.0]), np.array([0.0, -1.0, 5.0]), 20, 10)
    assert not mask.any()
    kept, _, _ = inside_image(np.array([0.0, 19.0]), np.array([0.0, 9.0]), 20, 10)
    assert kept.all()


def test_zero_depth_pixel_is_excluded_from_error():
    depth = np.zeros((10, 20), dtype=np.uint16)
    xyz = backproject_pinhole(np.array([5.0]), np.array([4.0]), np.array([1.0]), FX, FY, CX, CY)
    info = {
        "model": "plumb_bob",
        "distortion_all_zero": True,
        "r_is_identity": True,
        "fx": FX,
        "fy": FY,
        "cx": CX,
        "cy": CY,
    }
    metrics = depth_cloud_consistency(xyz, depth, info)
    assert metrics["valid_samples"] == 0
    assert metrics["in_bounds_count"] == 1


def test_matching_depth_has_near_zero_error():
    u = np.array([60.0])
    v = np.array([40.0])
    z_m = np.array([1.234])
    xyz = backproject_pinhole(u, v, z_m, FX, FY, CX, CY)
    depth = np.zeros((80, 100), dtype=np.uint16)
    depth[40, 60] = 1234
    info = {
        "model": "plumb_bob",
        "distortion_all_zero": True,
        "r_is_identity": True,
        "fx": FX,
        "fy": FY,
        "cx": CX,
        "cy": CY,
    }
    metrics = depth_cloud_consistency(xyz, depth, info)
    assert metrics["valid_samples"] == 1
    assert metrics["error_3d_max_m"] < 1e-9


def test_holdout_capture_is_rejected():
    try:
        assert_reference_capture("/tmp/capture_04/cloud.pcd")
    except SystemExit as exc:
        assert exc.code == 3
    else:
        raise AssertionError("holdout was accepted")
