"""R1M 분류. 실측 점군은 쓰지 않는다."""

from p008r1l_geometry import normal_consistency
from p008r1m_geometry import complementary_observation, surface_class

RULES = {
    "min_side_points": 400,
    "min_side_voxels": 80,
    "min_valid_normal_ratio": 0.8,
    "max_median_neighbor_angle_deg": 25.0,
    "min_count_ratio_vs_a": 2.0,
    "tilt_delta_min_deg": 15.0,
    "tilt_min_elevated_points": 1000,
    "tilt_min_component_voxels": 150,
}


def test_surface_class_bounds():
    assert surface_class(34.9, 35.0, 50.0, 130.0) == "top"
    assert surface_class(35.0, 35.0, 50.0, 130.0) == "other"
    assert surface_class(50.0, 35.0, 50.0, 130.0) == "side"
    assert surface_class(130.0, 35.0, 50.0, 130.0) == "side"
    assert surface_class(130.1, 35.0, 50.0, 130.0) == "other"
    assert surface_class(float("nan"), 35.0, 50.0, 130.0) == "unknown"


def test_side_rule_needs_count_and_quality():
    weak = complementary_observation(250, 80, 0.9, 10.0, 1000, 150, 0.9, 10.0, 20.0, 100, 20.0, RULES)
    assert weak["complementary"] is False
    strong = complementary_observation(800, 80, 0.9, 10.0, 1000, 150, 0.9, 10.0, 20.0, 100, 20.0, RULES)
    assert strong["side_rule"] is True
    noisy = complementary_observation(800, 80, 0.9, 25.1, 1000, 150, 0.9, 10.0, 20.0, 100, 20.0, RULES)
    assert noisy["side_rule"] is False


def test_tilt_rule_needs_fifteen_degrees():
    near = complementary_observation(10, 1, 0.0, None, 1000, 150, 0.9, 10.0, 34.9, 10, 20.0, RULES)
    assert near["tilt_rule"] is False
    far = complementary_observation(10, 1, 0.0, None, 1000, 150, 0.9, 10.0, 35.0, 10, 20.0, RULES)
    assert far["tilt_rule"] is True
    assert far["complementary"] is True
    stats = normal_consistency([0.0, 10.0, 40.0], 25.0)
    assert stats["unstable_ratio"] == 1.0 / 3.0
