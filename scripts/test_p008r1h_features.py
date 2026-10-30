"""P-008-R1H 변환과 선택 규칙. 실측 점군은 쓰지 않는다."""

import math

import numpy as np

from p008r1h_features import (
    BLOCK_HIGH,
    BLOCK_MEDIUM,
    bhattacharyya,
    circular_mean_deg,
    grade,
    histogram_js,
    plane_height,
    point_in_polygon,
    rgb_to_hsv,
    rgb_to_lab,
    select_method,
)


def test_hsv_primary_colors_and_wrap():
    hsv = rgb_to_hsv([[255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 1, 0], [255, 0, 1]])
    assert abs(hsv[0, 0] - 0.0) < 1e-9
    assert abs(hsv[0, 1] - 1.0) < 1e-9 and abs(hsv[0, 2] - 1.0) < 1e-9
    assert abs(hsv[1, 0] - 1.0 / 3.0) < 1e-9
    assert abs(hsv[2, 0] - 2.0 / 3.0) < 1e-9
    near_zero = hsv[3, 0] * 2.0 * math.pi
    near_wrap = hsv[4, 0] * 2.0 * math.pi
    pair = np.array(
        [
            [math.sin(near_zero), math.cos(near_zero)],
            [math.sin(near_wrap), math.cos(near_wrap)],
        ]
    )
    assert np.linalg.norm(pair[0] - pair[1]) < 0.05
    assert circular_mean_deg([0.0, 0.99]) < 5.0 or circular_mean_deg([0.0, 0.99]) > 355.0


def test_lab_white_black_and_red():
    lab = rgb_to_lab([[255, 255, 255], [0, 0, 0], [255, 0, 0]])
    assert abs(lab[0, 0] - 100.0) < 0.05
    assert abs(lab[0, 1]) < 0.05 and abs(lab[0, 2]) < 0.05
    assert abs(lab[1, 0]) < 1e-6 and abs(lab[1, 1]) < 1e-6 and abs(lab[1, 2]) < 1e-6
    # Bruce Lindbloom sRGB D65 red, 소수 둘째 자리에서 비교한다.
    assert abs(lab[2, 0] - 53.24) < 0.05
    assert abs(lab[2, 1] - 80.09) < 0.05
    assert abs(lab[2, 2] - 67.20) < 0.05


def test_invalid_rgb_and_nan_height():
    assert np.isnan(rgb_to_hsv([[-1, 0, 0]])[0]).all()
    assert np.isnan(rgb_to_lab([[256, 0, 0]])[0]).all()
    height = plane_height([[0.0, 0.0, 0.8], [math.nan, 0.0, 1.0]], [0.0, 0.0, -1.0, 1.0])
    assert abs(height[0] - 0.2) < 1e-9
    assert math.isnan(height[1])


def test_polygon_and_identical_histograms():
    inside = point_in_polygon([1.5, 5.0], [1.5, 1.5], [(1, 1), (2, 1), (2, 2), (1, 2)])
    assert inside.tolist() == [True, False]
    edges = np.linspace(0.0, 1.0, 9)
    same = histogram_js([0.2, 0.4, 0.6], [0.2, 0.4, 0.6], edges)
    apart = histogram_js([0.1, 0.1], [0.9, 0.9], edges)
    assert same < 1e-9
    assert apart > 0.9
    assert bhattacharyya([0.2, 0.4], [0.2, 0.4], edges) < 1e-6


def test_selection_rules_are_fixed():
    high = {name: "LOW" for name in ("RGB", "HSV", "Lab", "Height", "HSV+Height", "Lab+Height", "RGB+Height")}
    scores = {name: 0.5 for name in high}
    contact_flag = True
    stopped = select_method(high, high, scores, scores, contact_flag)
    assert stopped["selected_segmentation_strategy"] == "SEGMENTATION_STOP_RULE"
    assert stopped["stop_rule_triggered"] is True
    unknown = select_method(high, high, scores, scores, False)
    assert unknown["reason"] == "contact_not_identifiable"

    color = dict(high)
    color["Lab"] = "HIGH"
    color["HSV"] = "HIGH"
    color_scores = dict(scores)
    color_scores["Lab"] = 0.93
    color_scores["HSV"] = 0.94
    chosen = select_method(color, color, color_scores, color_scores, True)
    assert chosen["feature_separability"] == "COLOR_SEPARABLE"
    assert chosen["selected_feature_family"] == "Lab"
    assert chosen["selected_segmentation_strategy"] == "COLOR_ASSISTED_ROI"

    only_height = dict(high)
    only_height["Height"] = "HIGH"
    only_height["Lab+Height"] = "HIGH"
    multi_scores = dict(scores)
    multi_scores["Height"] = 0.91
    multi_scores["Lab+Height"] = 0.96
    multi = select_method(only_height, only_height, multi_scores, multi_scores, True)
    assert multi["feature_separability"] == "MULTIMODAL_SEPARABLE"
    assert multi["selected_feature_family"] == "Lab+Height"
    assert grade(BLOCK_HIGH, 3) == "HIGH"
    assert grade(BLOCK_MEDIUM, 3) == "MEDIUM"
    assert grade(0.99, 2) == "LOW"
