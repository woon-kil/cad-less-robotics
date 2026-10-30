"""P-008-R3 frame 계산."""

import numpy as np

from p008r3_frames import (
    align_quaternion_signs,
    classify_normal_pair,
    orientation_step_deg,
    orient_normal_towards_viewpoint,
    rotation_to_quaternion,
    tangents,
    tool_frame,
)


def test_centered_tangent_follows_the_chord():
    points = np.array([[0.0, 0.0, 0.0], [0.002, 0.0, 0.0], [0.004, 0.0, 0.0]])
    direction = tangents(points)
    assert np.allclose(direction, [[1.0, 0.0, 0.0]] * 3)


def test_normal_turns_toward_the_camera():
    point = np.array([0.0, 0.0, 0.5])
    facing = orient_normal_towards_viewpoint(point, np.array([0.0, 0.0, 1.0]), np.zeros(3))
    assert facing[2] < 0.0
    already = orient_normal_towards_viewpoint(point, np.array([0.0, 0.0, -1.0]), np.zeros(3))
    assert np.allclose(already, [0.0, 0.0, -1.0])
    assert abs(np.linalg.norm(facing) - 1.0) < 1e-12


def test_negative_dot_with_camera_facing_normals_is_geometric():
    current = np.array([1.0, 0.0, -0.2])
    current = current / np.linalg.norm(current)
    nxt = np.array([-1.0, 0.0, -0.2])
    nxt = nxt / np.linalg.norm(nxt)
    point = np.array([0.0, 0.0, 0.5])
    kind = classify_normal_pair(
        current, current, nxt, nxt, point, point, 0.002, 0.02, True, True)
    assert kind == "GEOMETRIC_DISCONTINUITY"


def test_gram_schmidt_frame_is_right_handed():
    rotation = tool_frame(np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), 1e-6)
    assert rotation is not None
    assert np.allclose(rotation[:, 2], [0.0, 0.0, -1.0])
    assert abs(np.dot(rotation[:, 0], rotation[:, 2])) < 1e-12
    assert abs(np.linalg.det(rotation) - 1.0) < 1e-12
    gram = rotation.T @ rotation
    assert np.max(np.abs(gram - np.eye(3))) < 1e-12


def test_parallel_tangent_is_degenerate():
    assert tool_frame(np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, -1.0]), 1e-6) is None


def test_quaternion_round_trip_and_sign_alignment():
    rotation = tool_frame(np.array([1.0, 0.0, 0.2]), np.array([0.0, 0.0, 1.0]), 1e-6)
    quaternion = rotation_to_quaternion(rotation)
    assert abs(np.linalg.norm(quaternion) - 1.0) < 1e-12
    sequence = np.vstack((quaternion, -quaternion))
    aligned = align_quaternion_signs(sequence)
    assert float(np.dot(aligned[0], aligned[1])) > 0.0


def test_orientation_step_is_zero_for_the_same_pose():
    quaternion = np.array([0.0, 0.0, 0.0, 1.0])
    assert orientation_step_deg(quaternion, quaternion) == 0.0
    assert orientation_step_deg(quaternion, -quaternion) == 0.0


def test_position_array_is_not_rewritten_by_tangent():
    points = np.array([[0.0, 0.0, 0.0], [0.01, 0.0, 0.0], [0.02, 0.002, 0.0]])
    original = points.copy()
    tangents(points)
    assert np.allclose(points, original)
    assert float(np.max(np.linalg.norm(points - original, axis=1))) == 0.0
