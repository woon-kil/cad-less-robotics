"""P-008-R1GEO 평면 거리와 군집 선택."""

import numpy as np
import pytest

from p008r1geo import finite_mask, normalize_plane, orient_toward_camera, select_cluster, signed_distance


PARAMS = {
    "select_min_points": 300,
    "select_min_median_height": 0.015,
    "select_min_extent": 0.06,
    "select_max_extent": 0.30,
    "max_object_height": 0.18,
}


def cluster(identifier, count, median, extent, centroid=(0.0, 0.0, 0.5), height_max=0.1):
    return {
        "id": identifier,
        "point_count": count,
        "height_median": median,
        "height_max": height_max,
        "extent": list(extent),
        "centroid": list(centroid),
    }


def test_signed_distance_and_normalization():
    normal, offset = normalize_plane([0.0, 0.0, 2.0], -2.0)
    assert normal == pytest.approx([0.0, 0.0, 1.0])
    assert offset == pytest.approx(-1.0)
    distance = signed_distance([[0.0, 0.0, 1.2]], normal, offset)
    assert distance == pytest.approx([0.2])


def test_orientation_flips_once_toward_camera():
    normal, offset, flipped = orient_toward_camera([0.0, 0.0, 1.0], -1.0, [0.0, 0.0, 1.0])
    assert flipped is True
    assert normal == pytest.approx([0.0, 0.0, -1.0])
    assert offset == pytest.approx(1.0)
    assert signed_distance([[0.0, 0.0, 0.0]], normal, offset) == pytest.approx([1.0])
    again, again_offset, flipped_again = orient_toward_camera(normal, offset, [0.0, 0.0, 1.0])
    assert flipped_again is False
    assert again == pytest.approx(normal)
    assert again_offset == pytest.approx(offset)


def test_invalid_points_are_excluded():
    mask = finite_mask([[0.0, 0.0, 1.0], [np.nan, 0.0, 1.0], [0.0, np.inf, 1.0]])
    assert mask.tolist() == [True, False, False]
    with pytest.raises(ValueError):
        normalize_plane([0.0, 0.0, 0.0], 1.0)


def test_selection_is_deterministic():
    lip = cluster(3, 800, 0.006, [0.20, 0.05, 0.04])
    small = cluster(1, 400, 0.04, [0.10, 0.08, 0.06], centroid=(-0.2, 0.0, 0.5))
    large = cluster(2, 900, 0.05, [0.18, 0.12, 0.10], centroid=(-0.1, 0.05, 0.55))
    chosen = select_cluster([small, lip, large], PARAMS)
    assert chosen["id"] == 2
    tie_a = cluster(5, 900, 0.05, [0.18, 0.12, 0.10], centroid=(0.2, 0.0, 0.5))
    tie_b = cluster(4, 900, 0.05, [0.18, 0.12, 0.10], centroid=(-0.2, 0.0, 0.5))
    assert select_cluster([tie_a, tie_b], PARAMS)["id"] == 4
    assert select_cluster([lip], PARAMS) is None
    huge = cluster(6, 5000, 0.05, [0.50, 0.40, 0.10])
    assert select_cluster([huge, small], PARAMS)["id"] == 1
