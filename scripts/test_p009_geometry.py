"""P-009 계약 함수 테스트. 점군 파일은 읽지 않는다."""

import numpy as np

from p009_geometry import (
    NeighborIndex,
    align_quaternion_sign,
    associate_target,
    command_from_anchor,
    consecutive_spacing,
    geodesic_deg,
    max_position_jump,
    offset_target,
    orient_normal_toward_camera,
    process_frame,
    quaternion_xyzw,
    smoothing_within_limit,
    spacing_within_tolerance,
    spec_forbidden,
    standoff_residuals,
    tangents_from_anchors,
)


def grid(z=0.50):
    xs = np.linspace(-0.04, 0.04, 21)
    ys = np.linspace(-0.04, 0.04, 21)
    xx, yy = np.meshgrid(xs, ys)
    return np.column_stack((xx.ravel(), yy.ravel(), np.full(xx.size, z)))


def test_spec_rejects_forbidden_pose_input():
    clean = {"process_type": "SURFACE_STRIP", "sampling": {"spacing_m": 0.005}}
    assert spec_forbidden(clean) == []
    dirty = {"process_type": "SURFACE_STRIP", "final_xyz": [[0.0, 0.0, 0.0]]}
    assert "final_xyz" in spec_forbidden(dirty)


def test_knn_rejects_sparse_far_and_accepts_flat_plane():
    points = grid()
    index = NeighborIndex(points, 0.005)
    accepted = associate_target(np.array([0.0, 0.0, 0.51]), index, 20, 12, 0.015, 0.015)
    assert accepted["status"] == "ACCEPTED"
    assert accepted["neighbor_count"] >= 12
    assert accepted["kth_neighbor_distance_m"] <= 0.015
    assert abs(accepted["surface_anchor"][2] - 0.50) < 1e-6
    assert accepted["normal"][2] < 0.0
    sparse = associate_target(np.array([0.0, 0.0, 0.50]), NeighborIndex(points[:5], 0.005), 20, 12, 0.015, 0.015)
    assert sparse["status"] == "REJECTED_UNSUPPORTED_GEOMETRY"
    far = associate_target(np.array([0.20, 0.0, 0.50]), index, 20, 12, 0.015, 0.015)
    assert far["status"] == "REJECTED_UNSUPPORTED_GEOMETRY"
    assert far["kth_neighbor_distance_m"] is None or far["kth_neighbor_distance_m"] > 0.015 or far["neighbor_count"] < 20


def test_tilted_plane_projection_and_camera_normal():
    xs = np.linspace(-0.03, 0.03, 15)
    ys = np.linspace(-0.03, 0.03, 15)
    xx, yy = np.meshgrid(xs, ys)
    zz = 0.40 + 0.2 * xx
    points = np.column_stack((xx.ravel(), yy.ravel(), zz.ravel()))
    index = NeighborIndex(points, 0.005)
    target = np.array([0.0, 0.0, 0.41])
    found = associate_target(target, index, 20, 12, 0.015, 0.015)
    assert found["status"] == "ACCEPTED"
    assert abs(found["surface_anchor"][2] - 0.40) < 1e-3
    assert found["normal"] @ found["surface_anchor"] < 0.0
    assert orient_normal_toward_camera(np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 0.4]))[2] < 0.0


def test_hole_between_patches_is_rejected():
    left = grid()
    right = grid()
    left[:, 0] -= 0.06
    right[:, 0] += 0.06
    points = np.vstack((left, right))
    index = NeighborIndex(points, 0.005)
    hole = associate_target(np.array([0.0, 0.0, 0.50]), index, 20, 12, 0.015, 0.015)
    assert hole["status"] == "REJECTED_UNSUPPORTED_GEOMETRY"


def test_offset_must_be_reassociated():
    points = grid()
    index = NeighborIndex(points, 0.005)
    base = associate_target(np.array([0.0, 0.0, 0.50]), index, 20, 12, 0.015, 0.015)
    y_axis = np.array([0.0, 1.0, 0.0])
    requested = offset_target(base["surface_anchor"], y_axis, 0.01)
    requested[2] += 0.01
    again = associate_target(requested, index, 20, 12, 0.015, 0.015)
    assert again["status"] == "ACCEPTED"
    assert abs(again["surface_anchor"][2] - 0.50) < 1e-6
    assert abs(requested[2] - again["surface_anchor"][2]) > 1e-3
    missing = associate_target(offset_target(base["surface_anchor"], y_axis, 0.20), index, 20, 12, 0.015, 0.015)
    assert missing["status"] == "REJECTED_UNSUPPORTED_GEOMETRY"


def test_tangent_frame_quaternion_and_standoff():
    anchors = np.array([[0.0, 0.0, 0.5], [0.005, 0.0, 0.5], [0.010, 0.0, 0.5]])
    normals = np.tile(np.array([0.0, 0.0, -1.0]), (3, 1))
    tangent = tangents_from_anchors(anchors, normals)
    assert abs(tangent[1][0] - 1.0) < 1e-9
    assert abs(float(tangent[1] @ normals[1])) < 1e-9
    frame = process_frame(tangent[1], normals[1])
    assert abs(np.linalg.det(frame) - 1.0) < 1e-9
    assert np.allclose(frame.T @ frame, np.eye(3), atol=1e-9)
    quat = quaternion_xyzw(frame)
    assert abs(np.linalg.norm(quat) - 1.0) < 1e-9
    flipped = align_quaternion_sign(-quat, quat)
    assert np.allclose(flipped, quat)
    command = command_from_anchor(anchors[1], normals[1], 0.02)
    along, lateral = standoff_residuals(anchors[1], command, normals[1], 0.02)
    assert along < 1e-12 and lateral < 1e-12
    assert command[2] < anchors[1][2]


def test_spacing_gap_geodesic_and_smoothing_limit():
    points = np.array([[0.0, 0.0, 0.0], [0.005, 0.0, 0.0], [0.010, 0.0, 0.0], [0.022, 0.0, 0.0]])
    gaps = consecutive_spacing(points)
    assert spacing_within_tolerance(gaps, 0.004, 0.006) == 1.0
    assert max_position_jump(gaps) > 0.010
    assert geodesic_deg([0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]) < 1e-6
    ok, shift = smoothing_within_limit(points, points + np.array([0.001, 0.0, 0.0]), 0.003)
    assert ok and float(shift.max()) <= 0.003
    too_far, _shift = smoothing_within_limit(points, points + np.array([0.004, 0.0, 0.0]), 0.003)
    assert too_far is False
