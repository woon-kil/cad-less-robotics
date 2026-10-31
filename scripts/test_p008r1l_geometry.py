"""R1L 작업면 함수. 실측 점군은 쓰지 않는다."""

import numpy as np

from p008r1l_geometry import (
    boundary_clearance,
    graph_extent,
    normal_consistency,
    usable_region_mask,
)


def test_boundary_clearance_on_grid():
    leaf = 0.01
    uv = [[i * leaf, j * leaf] for i in range(5) for j in range(5)]
    dist = boundary_clearance(uv, leaf)
    center = dist[2 * 5 + 2]
    assert abs(center - 0.02) < 1e-9
    assert boundary_clearance([[0.0, 0.0]], leaf)[0] == 0.0


def test_usable_region_requires_every_predicate():
    mask = usable_region_mask([True, True], [True, True], [True, False], [True, True])
    assert mask.tolist() == [True, False]
    assert usable_region_mask([True], [True], [True], [True]).tolist() == [True]


def test_extent_stays_inside_connected_voxels():
    leaf = 0.005
    line = [(i, 0, 0) for i in range(21)]
    length, path = graph_extent(line, leaf)
    assert abs(length - 0.1) < 1e-9
    assert {path[0], path[-1]} == {(0, 0, 0), (20, 0, 0)}
    gapped = [(i, 0, 0) for i in range(5)] + [(i, 0, 0) for i in range(7, 12)]
    gap_length, gap_path = graph_extent(gapped, leaf)
    assert gap_length < 0.03
    assert all(key[0] <= 4 for key in gap_path)


def test_normal_consistency_percentiles():
    stats = normal_consistency([0.0, 10.0, 20.0, 30.0], 25.0)
    assert stats["count"] == 4
    assert abs(stats["median_deg"] - 15.0) < 1e-9
    assert abs(stats["unstable_ratio"] - 0.25) < 1e-9
    assert stats["p95_deg"] > stats["median_deg"]
    empty = normal_consistency([np.nan], 25.0)
    assert empty["median_deg"] is None
