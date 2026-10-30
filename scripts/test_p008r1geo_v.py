"""P-008-R1GEO-V. hold-out은 같은 설정만 쓴다."""

import json
from pathlib import Path

from p008r1geo_v import HOLDOUT, build_metrics, gate_from_results, judge, load_frozen_parameters


def test_holdout_excludes_development_capture():
    assert "capture_01" not in HOLDOUT
    assert HOLDOUT == ("capture_02", "capture_03", "capture_04", "capture_05")
    try:
        build_metrics("capture_01", "abc", 1, {"identifiable": True}, [], None)
    except SystemExit:
        return
    raise AssertionError("capture_01이 집계에 들어갔다.")


def test_same_configuration_and_no_override_in_source():
    source = Path("scripts/p008r1geo_v.py").read_text()
    assert "if capture_0" not in source
    frozen = load_frozen_parameters()
    again = load_frozen_parameters()
    assert again == frozen
    assert frozen["ransac_distance"] == 0.004
    assert frozen["voxel_leaf"] == 0.002
    assert frozen["cluster_tolerance"] == 0.01


def test_selection_and_serialization_are_deterministic():
    from p008r1geo import select_cluster

    params = load_frozen_parameters()
    clusters = [
        {
            "id": 1,
            "point_count": 400,
            "height_median": 0.04,
            "height_max": 0.08,
            "height_min": 0.01,
            "height_p95": 0.07,
            "extent": [0.12, 0.08, 0.06],
            "centroid": [-0.1, 0.0, 0.5],
            "aabb_min": [-0.2, -0.1, 0.4],
            "aabb_max": [-0.08, 0.08, 0.6],
        },
        {
            "id": 0,
            "point_count": 900,
            "height_median": 0.05,
            "height_max": 0.1,
            "height_min": 0.02,
            "height_p95": 0.09,
            "extent": [0.18, 0.12, 0.1],
            "centroid": [-0.12, 0.05, 0.55],
            "aabb_min": [-0.2, 0.0, 0.5],
            "aabb_max": [-0.02, 0.12, 0.6],
        },
    ]
    first = select_cluster(clusters, params)
    second = select_cluster(list(reversed(clusters)), params)
    assert first["id"] == second["id"] == 0
    metrics = build_metrics("capture_02", "abc", 10, {"identifiable": True, "coefficients": [0, 0, 1, -1], "inlier_count": 3, "inlier_ratio": 0.5, "residual_median": 0.001, "residual_p95": 0.002}, clusters, first)
    encoded = json.dumps(metrics)
    assert json.loads(encoded)["selected_cluster"] == 0
    assert json.loads(encoded)["capture"] == "capture_02"


def test_visual_judge_does_not_change_thresholds():
    metrics = {"support_plane_identifiable": True, "shoe_candidate_generated": True}
    assert judge(metrics, True, False)[0] == "PASS"
    assert judge(metrics, False, False)[1] == "SHOE_SURFACE_LOSS"
    assert judge(metrics, True, True)[1] == "BACKGROUND_CONTAMINATION"
    assert judge({"support_plane_identifiable": False, "shoe_candidate_generated": False}, True, False)[1] == "SUPPORT_PLANE_FAILURE"
    assert gate_from_results(["PASS", "PASS", "PASS", "PASS"])[0] == "PASS_HOLDOUT_VALIDATION"
    assert gate_from_results(["PASS", "PASS", "PASS", "FAIL"])[2] is True
    assert gate_from_results(["FAIL", "FAIL", "PASS", "PASS"])[0] == "FAIL_HOLDOUT_GENERALIZATION"
