"""P-008-R2 경로 계산."""

import numpy as np

from p008r2_path import (
    assign_bins,
    build_longitudinal_path,
    ordering_violations,
    orient_longitudinal,
    path_lengths,
    principal_axes,
    reject_jumps,
    representative_points,
    resample_path,
    nearest_distances,
)


def test_pca_orders_the_long_axis_first():
    line = np.stack([np.linspace(0, 1, 20), np.zeros(20), np.zeros(20)], axis=1)
    _, eigenvalues, axes = principal_axes(line)
    assert eigenvalues[0] > eigenvalues[1]
    assert abs(abs(axes[0, 0]) - 1.0) < 1e-6


def test_axis_points_toward_the_higher_end():
    points = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.2]], dtype=float)
    axis = orient_longitudinal(points, np.array([-1.0, 0.0, 0.0]), points.mean(0), np.array([0.0, 0.0, 1.0]), 0.0, 0.5)
    assert axis[0] > 0


def test_bins_and_representative_are_ordered():
    projection = np.array([0.0, 0.004, 0.012, 0.016])
    bins, _ = assign_bins(projection, 0.01)
    assert bins.tolist() == [0, 0, 1, 1]
    points = np.array([
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.02],
        [0.01, 0.0, 0.0],
        [0.01, 0.0, 0.04],
    ])
    height = points[:, 2]
    chosen, coordinates = representative_points(points, projection, height, 0.01, 0.75, 2)
    assert list(coordinates) == sorted(coordinates)
    assert chosen[0, 2] == 0.02
    assert chosen[1, 2] == 0.04


def test_jump_rejection_and_ordering():
    path = np.array([[0.0, 0.0, 0.0], [0.01, 0.0, 0.0], [0.08, 0.0, 0.0]])
    kept = reject_jumps(path, 0.03)
    assert len(kept) == 2
    assert ordering_violations([0.0, 0.1, 0.2]) == 0
    assert ordering_violations([0.0, 0.2, 0.1]) == 1


def test_arc_length_and_resample_spacing():
    path = np.array([[0.0, 0.0, 0.0], [0.004, 0.0, 0.0], [0.004, 0.004, 0.0]])
    segments, total = path_lengths(path)
    assert total == 0.008
    sampled = resample_path(path, 0.002)
    gaps, _ = path_lengths(sampled)
    assert abs(gaps.mean() - 0.002) < 1e-9
    assert sampled[0].tolist() == [0.0, 0.0, 0.0]
    assert sampled[-1].tolist() == [0.004, 0.004, 0.0]


def test_surface_distance_is_zero_on_the_cloud():
    surface = np.array([[0.0, 0.0, 0.0], [0.01, 0.0, 0.0]])
    distance, index = nearest_distances(surface, surface)
    assert distance.tolist() == [0.0, 0.0]
    assert index.tolist() == [0, 1]


def test_synthetic_longitudinal_path_stays_ordered():
    axis = np.linspace(-0.08, 0.08, 40)
    cloud = []
    for position in axis:
        for lateral in (-0.01, 0.0, 0.01):
            cloud.append([position, lateral, 0.02 + 0.05 * (position + 0.08)])
    points = np.asarray(cloud)
    result = build_longitudinal_path(
        points, np.array([0.0, 0.0, 1.0]), 0.0, {**{
            "bin_width_m": 0.008,
            "height_quantile": 0.75,
            "min_bin_points": 3,
            "end_fraction": 0.15,
            "jump_reject_m": 0.03,
            "smooth_passes": 1,
            "target_spacing_m": 0.002,
            "surface_reprojection": True,
        }})
    assert result["ordering_violations"] == 0
    assert len(result["final"]) > 10
    assert result["surface_distance"].max() == 0.0
