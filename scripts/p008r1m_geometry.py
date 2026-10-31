"""R1M 뷰 비교. 점군 파일은 읽지 않는다."""

import math


def surface_class(angle_deg, top_max, side_min, side_max):
    """지지 법선과의 각으로 위를 보는 면과 옆면을 가른다."""
    if angle_deg is None or not math.isfinite(float(angle_deg)):
        return "unknown"
    angle = float(angle_deg)
    if angle < top_max:
        return "top"
    if side_min <= angle <= side_max:
        return "side"
    return "other"


def complementary_observation(
    side_points,
    side_voxels,
    side_valid_ratio,
    side_median_deg,
    elevated_points,
    elevated_voxels,
    elevated_valid_ratio,
    elevated_median_deg,
    mean_support_angle_deg,
    baseline_side_points,
    baseline_mean_support_angle_deg,
    rules,
):
    """옆면이 늘었거나 작업면 기울기가 달라지면 보완 관찰이다."""
    side_ratio = float(rules["min_count_ratio_vs_a"]) * max(int(baseline_side_points), 1)
    side_angles_ok = side_median_deg is not None and side_median_deg <= rules["max_median_neighbor_angle_deg"]
    side_rule = bool(
        side_points >= rules["min_side_points"]
        and side_voxels >= rules["min_side_voxels"]
        and side_valid_ratio >= rules["min_valid_normal_ratio"]
        and side_angles_ok
        and side_points >= side_ratio
    )
    tilt_angles_ok = (
        elevated_median_deg is not None
        and elevated_median_deg <= rules["max_median_neighbor_angle_deg"]
        and mean_support_angle_deg is not None
        and baseline_mean_support_angle_deg is not None
        and abs(mean_support_angle_deg - baseline_mean_support_angle_deg) >= rules["tilt_delta_min_deg"]
    )
    tilt_rule = bool(
        tilt_angles_ok
        and elevated_points >= rules["tilt_min_elevated_points"]
        and elevated_voxels >= rules["tilt_min_component_voxels"]
        and elevated_valid_ratio >= rules["min_valid_normal_ratio"]
    )
    return {"side_rule": side_rule, "tilt_rule": tilt_rule, "complementary": side_rule or tilt_rule}
