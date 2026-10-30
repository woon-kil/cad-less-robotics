#!/usr/bin/env python3
"""P-008-R1 지표 단위 테스트. 합성 도형만 사용한다."""

import unittest

import numpy as np

from p008r1_metrics import (
    distribution,
    duplicate_consecutive,
    finite_coordinates,
    pca_frame,
    self_intersection_count,
    signed_area,
    spacing,
    support_distances,
)


class PcaTests(unittest.TestCase):
    def test_axis_sign_is_stable(self):
        rng = np.random.default_rng(7)
        base = rng.normal(size=(80, 3))
        base[:, 0] *= 4.0
        shifted = base + np.array([0.4, -0.2, 0.9])
        first = pca_frame(shifted)
        second = pca_frame(shifted)
        for key in ("axis_0", "axis_1", "axis_2"):
            self.assertTrue(np.allclose(first[key], second[key]))
        flipped = pca_frame(shifted[::-1])
        for key in ("axis_0", "axis_1", "axis_2"):
            self.assertTrue(np.allclose(first[key], flipped[key], atol=1e-8))
        self.assertGreater(first["axis_0"][0], 0.0)
        self.assertLess(float(np.dot(first["axis_2"], first["centroid"])), 0.0)
        axes = np.column_stack((first["axis_0"], first["axis_1"], first["axis_2"]))
        self.assertGreater(np.linalg.det(axes), 0.0)


class ContourTests(unittest.TestCase):
    def test_square_is_closed_without_crossing(self):
        square = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
        self.assertEqual(self_intersection_count(square), 0)
        self.assertEqual(duplicate_consecutive(square), 0)
        self.assertAlmostEqual(signed_area(square), 1.0)
        self.assertTrue(finite_coordinates(square))

    def test_bowtie_is_detected(self):
        bowtie = np.array([[0.0, 0.0], [1.0, 1.0], [0.0, 1.0], [1.0, 0.0]])
        self.assertGreater(self_intersection_count(bowtie), 0)


class SupportTests(unittest.TestCase):
    def test_known_nearest_distance(self):
        cloud = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        boundary = np.array([[0.0, 0.0, 0.0], [0.3, 0.0, 0.0]])
        distances = support_distances(boundary, cloud)
        self.assertAlmostEqual(distances[0], 0.0)
        self.assertAlmostEqual(distances[1], 0.3)


class SpacingTests(unittest.TestCase):
    def test_known_gap(self):
        line = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.2, 0.0, 0.0], [0.5, 0.0, 0.0]])
        gaps = spacing(line, closed=False)
        stats = distribution(gaps)
        self.assertAlmostEqual(stats["max"], 0.3)
        self.assertAlmostEqual(stats["p95"], float(np.percentile(gaps, 95)))
        self.assertEqual(int(np.argmax(gaps)), 2)


if __name__ == "__main__":
    unittest.main()
