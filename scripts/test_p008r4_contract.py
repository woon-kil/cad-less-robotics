"""P-008-R4 작업 구간."""

import numpy as np

from p008r4_contract import (
    apply_clearance,
    executable_window,
    first_hard_index,
    orientation_steps,
    tool_z,
    window_length,
)


def test_threshold_finds_only_steps_above_45():
    steps = np.array([0.0, 10.0, 45.0, 45.1, 20.0])
    assert first_hard_index(steps, 45.0) == 3
    assert first_hard_index(np.array([0.0, 44.9]), 45.0) is None


def test_window_stops_before_the_first_hard_step():
    steps = np.array([0.0, 10.0, 20.0, 90.0, 5.0])
    start, end, first = executable_window(steps, 45.0)
    assert (start, end, first) == (0, 2, 3)


def test_window_length_uses_arc_length():
    arc = np.array([0.0, 0.002, 0.010, 0.030])
    assert window_length(arc, 0, 2) == 0.010


def test_clearance_moves_opposite_tool_z():
    identity = np.array([[0.0, 0.0, 0.0, 1.0]])
    assert np.allclose(tool_z(identity), [[0.0, 0.0, 1.0]])
    points = np.array([[0.1, 0.2, 0.3]])
    task, offset = apply_clearance(points, identity, 0.020)
    assert np.allclose(task, [[0.1, 0.2, 0.28]])
    assert np.allclose(offset, [[0.0, 0.0, -0.020]])


def test_inputs_are_not_mutated():
    points = np.array([[0.0, 0.0, 0.5], [0.01, 0.0, 0.5]])
    quaternions = np.array([[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]])
    original_points = points.copy()
    original_quaternions = quaternions.copy()
    orientation_steps(quaternions)
    apply_clearance(points, quaternions, 0.020)
    assert np.array_equal(points, original_points)
    assert np.array_equal(quaternions, original_quaternions)
