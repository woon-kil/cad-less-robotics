#!/usr/bin/env python3
"""P-012. 샘플, 법선, Gaussian curvature, 분류 그림을 만든다."""

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = Path("/Users/woonkil/projects/cad-less-robotics")
OUT = ROOT / "evidence/P-012"
MESH = json.loads((OUT / "mesh.json").read_text(encoding="utf-8"))
SAMPLES = json.loads((OUT / "cad_surface_samples.json").read_text(encoding="utf-8"))["samples"]
VALID = [sample for sample in SAMPLES if sample["validity"] == "VALID"]
CLASS_COLORS = {
    "LOW_CURVATURE": "#7f8c6a",
    "ELLIPTIC": "#c0392b",
    "SADDLE": "#1a5276",
    "PARABOLIC_OR_TRANSITION": "#d4a017",
}


def style(axis):
    axis.set_xlabel("X mm")
    axis.set_ylabel("Y mm")
    axis.set_zlabel("Z mm")
    axis.set_xlim(-10, 200)
    axis.set_ylim(-10, 120)
    axis.set_zlim(0, 70)
    axis.set_box_aspect((210, 130, 70))
    axis.view_init(elev=24, azim=-62)


def add_mesh(axis):
    triangles = []
    for face in MESH["faces"]:
        triangles.extend(face["triangles"])
    axis.add_collection3d(Poly3DCollection(triangles, facecolor="#d9d3c7", edgecolor="none", alpha=0.35))


def save(figure, name):
    figure.tight_layout()
    figure.savefig(OUT / name, dpi=140)
    plt.close(figure)


def samples_figure():
    figure = plt.figure(figsize=(9, 6))
    axis = figure.add_subplot(111, projection="3d")
    add_mesh(axis)
    xs = [sample["position_mm"][0] for sample in VALID]
    ys = [sample["position_mm"][1] for sample in VALID]
    zs = [sample["position_mm"][2] for sample in VALID]
    axis.scatter(xs, ys, zs, s=4, c="#1f4e79", depthshade=False)
    style(axis)
    save(figure, "cad_surface_samples.png")


def normals_figure():
    figure = plt.figure(figsize=(9, 6))
    axis = figure.add_subplot(111, projection="3d")
    add_mesh(axis)
    for sample in VALID:
        normal = sample["normal"]
        if normal[2] < 0.5:
            continue
        point = sample["position_mm"]
        axis.quiver(
            point[0], point[1], point[2],
            normal[0] * 12, normal[1] * 12, normal[2] * 12,
            color="#111111", arrow_length_ratio=0.25, linewidth=0.6,
        )
    style(axis)
    save(figure, "cad_normals.png")


def gaussian_figure():
    figure = plt.figure(figsize=(9, 6))
    axis = figure.add_subplot(111, projection="3d")
    add_mesh(axis)
    xs = [sample["position_mm"][0] for sample in VALID]
    ys = [sample["position_mm"][1] for sample in VALID]
    zs = [sample["position_mm"][2] for sample in VALID]
    values = [sample["gaussian_per_mm2"] for sample in VALID]
    limit = max(abs(min(values)), abs(max(values)))
    plot = axis.scatter(xs, ys, zs, s=8, c=values, cmap="coolwarm", vmin=-limit, vmax=limit, depthshade=False)
    figure.colorbar(plot, ax=axis, shrink=0.6, label="K (1/mm²)")
    style(axis)
    save(figure, "cad_gaussian_curvature.png")


def classification_figure():
    figure = plt.figure(figsize=(9, 6))
    axis = figure.add_subplot(111, projection="3d")
    add_mesh(axis)
    for name, color in CLASS_COLORS.items():
        chosen = [sample for sample in VALID if sample["classification"] == name]
        if not chosen:
            continue
        axis.scatter(
            [sample["position_mm"][0] for sample in chosen],
            [sample["position_mm"][1] for sample in chosen],
            [sample["position_mm"][2] for sample in chosen],
            s=8, c=color, depthshade=False, label=name,
        )
    axis.legend(loc="upper left", fontsize=8)
    style(axis)
    save(figure, "cad_region_classification.png")


def main():
    samples_figure()
    normals_figure()
    gaussian_figure()
    classification_figure()


main()
