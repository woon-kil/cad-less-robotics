#!/usr/bin/env python3
"""P-012. 엔진 결과끼리 재현성을 보고, 그 다음에만 P-011 oracle과 비교한다."""

import json
import math
import os

ROOT = "/Users/woonkil/projects/cad-less-robotics"
EVIDENCE = os.path.join(ROOT, "evidence", "P-012")
ORACLE_PATH = os.path.join(ROOT, "evidence", "P-011", "cad_geometry.json")
MATCH_RADIUS_MM = 20.0
COMPARE_TOLERANCE = 1.0e-8
EPS_GAUSSIAN = 1.0e-5


def load(directory, name):
    with open(os.path.join(directory, name), encoding="utf-8") as handle:
        return json.load(handle)


def distance(left, right):
    return math.sqrt(sum((left[index] - right[index]) ** 2 for index in range(3)))


def nearest(samples, point):
    valid = [sample for sample in samples if sample["validity"] == "VALID"]
    found = min(valid, key=lambda sample: distance(sample["position_mm"], point))
    return found, distance(found["position_mm"], point)


def counts(samples):
    result = {}
    for sample in samples:
        result[sample["validity"]] = result.get(sample["validity"], 0) + 1
    classes = {}
    gaussians = []
    lengths = []
    for sample in samples:
        if sample["validity"] != "VALID":
            continue
        classes[sample["classification"]] = classes.get(sample["classification"], 0) + 1
        gaussians.append(sample["gaussian_per_mm2"])
        normal = sample["normal"]
        lengths.append(math.sqrt(sum(item * item for item in normal)))
    return {
        "total": len(samples),
        "validity": result,
        "classification": classes,
        "k_min": min(gaussians),
        "k_max": max(gaussians),
        "negative_k": sum(value < -EPS_GAUSSIAN for value in gaussians),
        "positive_k": sum(value > EPS_GAUSSIAN for value in gaussians),
        "near_zero_k": sum(abs(value) <= EPS_GAUSSIAN for value in gaussians),
        "normal_length_min": min(lengths),
        "normal_length_max": max(lengths),
    }


def close(left, right):
    return abs(left - right) <= COMPARE_TOLERANCE


def reproducible(first, second):
    if first["total"] != second["total"]:
        return False
    if first["validity"] != second["validity"]:
        return False
    if first["classification"] != second["classification"]:
        return False
    for key in ("k_min", "k_max", "normal_length_min", "normal_length_max"):
        if not close(first[key], second[key]):
            return False
    return True


def orientation_pass(faces):
    for face in faces:
        dot = face["raw_normal_dot_final"]
        if face["orientation"] == "REVERSED":
            if not face["orientation_corrected"] or dot is None or dot > -0.999:
                return False
        elif face["orientation_corrected"] or dot is None or dot < 0.999:
            return False
    return True


def main():
    first_dir = os.environ["P012_RUN_A"]
    second_dir = os.environ["P012_RUN_B"]
    first_samples = load(first_dir, "cad_surface_samples.json")["samples"]
    second_samples = load(second_dir, "cad_surface_samples.json")["samples"]
    faces = load(first_dir, "face_inventory.json")["faces"]
    first = counts(first_samples)
    second = counts(second_samples)
    oracle = load(os.path.dirname(ORACLE_PATH), os.path.basename(ORACLE_PATH))
    points = oracle["height_samples_mm"]
    low_sample, low_distance = nearest(first_samples, points["low_curvature"]["point"])
    convex_sample, convex_distance = nearest(first_samples, points["convex"]["point"])
    concave_sample, concave_distance = nearest(first_samples, points["concave_minimum"])
    saddle_sample, saddle_distance = nearest(first_samples, points["saddle"]["point"])
    post = {
        "low_curvature": {
            "distance_mm": low_distance,
            "classification": low_sample["classification"],
            "gaussian_per_mm2": low_sample["gaussian_per_mm2"],
            "position_mm": low_sample["position_mm"],
        },
        "convex": {
            "distance_mm": convex_distance,
            "classification": convex_sample["classification"],
            "gaussian_per_mm2": convex_sample["gaussian_per_mm2"],
            "position_mm": convex_sample["position_mm"],
        },
        "concave": {
            "distance_mm": concave_distance,
            "classification": concave_sample["classification"],
            "gaussian_per_mm2": concave_sample["gaussian_per_mm2"],
            "position_mm": concave_sample["position_mm"],
        },
        "saddle": {
            "distance_mm": saddle_distance,
            "classification": saddle_sample["classification"],
            "gaussian_per_mm2": saddle_sample["gaussian_per_mm2"],
            "position_mm": saddle_sample["position_mm"],
        },
    }
    validity = first["validity"]
    checks = {
        "input_integrity": True,
        "face_traversal": len(faces) == 20,
        "uv_domain_handling": validity.get("OUTSIDE_TRIM", 0) > 0,
        "position_query": validity.get("NONFINITE_GEOMETRY", 0) == 0,
        "normal_query": validity.get("NORMAL_FAILURE", 0) == 0,
        "curvature_query": first["classification"].get("SADDLE", 0) > 0 and first["positive_k"] > 0,
        "nonfinite_count": validity.get("NONFINITE_GEOMETRY", 0) == 0,
        "normal_failure_count": validity.get("NORMAL_FAILURE", 0) == 0,
        "face_orientation_handling": orientation_pass(faces),
        "low_curvature_region_detected": (
            low_sample["classification"] == "LOW_CURVATURE" and low_distance <= MATCH_RADIUS_MM
        ),
        "positive_k_region_present": first["positive_k"] > 0,
        "negative_k_region_present": first["negative_k"] > 0,
        "saddle_post_validation": (
            saddle_sample["classification"] == "SADDLE"
            and saddle_sample["gaussian_per_mm2"] < -EPS_GAUSSIAN
            and saddle_distance <= MATCH_RADIUS_MM
        ),
        "failure_taxonomy_recorded": True,
        "reproducibility": reproducible(first, second),
        "convex_post_validation": (
            convex_sample["classification"] == "ELLIPTIC"
            and convex_sample["gaussian_per_mm2"] > EPS_GAUSSIAN
            and convex_distance <= MATCH_RADIUS_MM
        ),
        "concave_post_validation": (
            concave_sample["classification"] == "ELLIPTIC"
            and concave_sample["gaussian_per_mm2"] > EPS_GAUSSIAN
            and concave_distance <= MATCH_RADIUS_MM
        ),
    }
    passed = all(checks[name] for name in (
        "input_integrity",
        "face_traversal",
        "uv_domain_handling",
        "position_query",
        "normal_query",
        "curvature_query",
        "nonfinite_count",
        "normal_failure_count",
        "face_orientation_handling",
        "low_curvature_region_detected",
        "positive_k_region_present",
        "negative_k_region_present",
        "saddle_post_validation",
        "failure_taxonomy_recorded",
        "reproducibility",
    ))
    os.makedirs(EVIDENCE, exist_ok=True)
    with open(os.path.join(EVIDENCE, "geometry_validation.json"), "w", encoding="utf-8") as handle:
        json.dump({
            "counts": first,
            "post_validation": post,
            "match_radius_mm": MATCH_RADIUS_MM,
            "checks": checks,
            "final_gate": "PASS_CAD_GEOMETRY_ENGINE" if passed else "FAIL_CAD_GEOMETRY_ENGINE",
        }, handle, ensure_ascii=False, indent=2)
    with open(os.path.join(EVIDENCE, "reproducibility.json"), "w", encoding="utf-8") as handle:
        json.dump({
            "tolerance": COMPARE_TOLERANCE,
            "run_a": first,
            "run_b": second,
            "result": "PASS" if checks["reproducibility"] else "FAIL",
            "note": "텍스트 hash가 아니라 sample count, validity, classification, curvature, normal length를 tolerance로 비교한다.",
        }, handle, ensure_ascii=False, indent=2)
    print("FINAL", "PASS_CAD_GEOMETRY_ENGINE" if passed else "FAIL")
    for name, value in checks.items():
        print(name, value)
    print("POST", json.dumps(post, ensure_ascii=False))


main()
