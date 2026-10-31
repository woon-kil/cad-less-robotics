"""R1N 기하 함수. 실측 점군은 쓰지 않는다."""

import numpy as np

from p008r1n_geometry import (
    dominant_side,
    frame_coordinates,
    interval_coverage,
    jump_limit,
    max_consecutive,
    orient_axis,
    pca_in_plane,
    side_transition,
    slice_centers,
    spacing_stats,
)


def test_axis_sign_and_pca():
    forward = orient_axis([2.0, -1.0, 0.0])
    backward = orient_axis([-2.0, 1.0, 0.0])
    assert forward[0] > 0.0
    assert np.allclose(forward, backward)
    line = np.column_stack((np.linspace(0.0, 1.0, 20), np.zeros(20)))
    axis = pca_in_plane(line)
    assert abs(abs(axis[0]) - 1.0) < 1e-6


def test_cutting_frame_and_slice_order():
    points = np.array([[0.10, 0.02, 0.30]])
    along, across, height = frame_coordinates(
        points, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], 0.0
    )
    assert abs(along[0] - 0.10) < 1e-9
    assert abs(across[0] - 0.02) < 1e-9
    assert abs(height[0] - 0.30) < 1e-9
    centers = slice_centers(0.0, 0.05, 0.01)
    assert centers == sorted(centers)
    assert len(centers) == 5


def test_transition_is_inward_of_the_steep_wall():
    position = np.arange(0.0, 0.041, 0.002)
    height = np.where(position >= 0.03, np.maximum(0.0, 0.03 - (position - 0.03) * 3.0), 0.03)
    found = side_transition(position, height, 0.008, 50.0, 25.0)
    side, label = dominant_side(found, 15.0)
    assert side == "positive" and label == "unique"
    assert position[found["positive"]["index"]] <= 0.032


def test_continuity_metrics_do_not_fill_gaps():
    assert max_consecutive(["unique", "missing", "missing", "missing", "unique"], "missing") == 3
    stats = spacing_stats([[0.0, 0.0, 0.0], [0.01, 0.0, 0.0], [0.04, 0.0, 0.0]])
    assert abs(stats["max_m"] - 0.03) < 1e-9
    assert abs(jump_limit(0.01) - 0.03) < 1e-9
    coverage = interval_coverage([0.02, 0.05], 0.01, 0.0, 0.10)
    assert abs(coverage - 0.2) < 1e-9
