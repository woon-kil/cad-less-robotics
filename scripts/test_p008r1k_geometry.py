"""R1K 기하 함수. 실측 점군은 쓰지 않는다."""

import numpy as np

from p008r1k_geometry import (
    angle_deg,
    connected_components,
    orient_toward_camera,
    polyline_distances,
    signed_plane_height,
    voxel_keys,
)


def test_signed_height_and_camera_sign():
    height = signed_plane_height([[0.0, 0.0, 0.5], [0.0, 0.0, 0.7]], [0.0, 0.0, -2.0], 0.6)
    assert abs(height[0] - 0.1) < 1e-9
    assert abs(height[1] + 0.1) < 1e-9
    normal = orient_toward_camera([0.0, 0.0, 1.0], [0.0, 0.0, 0.5])
    assert normal[2] < 0.0
    assert abs(np.linalg.norm(normal) - 1.0) < 1e-9


def test_connectivity_is_deterministic():
    keys = voxel_keys([[0.0, 0.0, 0.0], [0.005, 0.0, 0.0], [0.05, 0.0, 0.0]], 0.005)
    first = connected_components(keys)
    second = connected_components(keys)
    assert first == second
    assert len(first) == 2
    assert len(first[0]) == 2


def test_polyline_distance_on_segment():
    dist = polyline_distances([[0.5, 0.2], [2.0, 2.0]], [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    assert abs(dist[0] - 0.2) < 1e-9
    assert dist[1] > 1.0
    assert abs(angle_deg([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]) - 90.0) < 1e-6
