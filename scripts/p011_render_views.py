#!/usr/bin/env python3
"""P-011. qualification 메시로 coupon 그림 세 장을 만든다."""

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = Path("/Users/woonkil/projects/cad-less-robotics")
OUT = ROOT / "evidence/P-011"
MESH = json.loads((OUT / "mesh_faces.json").read_text(encoding="utf-8"))
TOPOLOGY = json.loads((OUT / "cad_topology.json").read_text(encoding="utf-8"))
COLORS = {
    "low_curvature": "#7f8c6a",
    "convex": "#c0392b",
    "saddle": "#1a7a72",
    "concave": "#d4a017",
    "side": "#d9d3c7",
    "base": "#b0b0b0",
    "landmark": "#1f4e79",
}


def color_of(face):
    if face["region"] in COLORS:
        return COLORS[face["region"]]
    return COLORS[face["role"]]


def add_axes(axis):
    axis.quiver(0, 0, 0, 30, 0, 0, color="#c0392b", arrow_length_ratio=0.12)
    axis.quiver(0, 0, 0, 0, 30, 0, color="#1e8449", arrow_length_ratio=0.12)
    axis.quiver(0, 0, 0, 0, 0, 30, color="#1a5276", arrow_length_ratio=0.12)
    axis.text(32, 0, 0, "X")
    axis.text(0, 34, 0, "Y")
    axis.text(0, 0, 32, "Z")


def style(axis):
    axis.set_xlabel("X mm")
    axis.set_ylabel("Y mm")
    axis.set_zlabel("Z mm")
    axis.set_xlim(-10, 200)
    axis.set_ylim(-10, 120)
    axis.set_zlim(0, 70)
    axis.set_box_aspect((210, 130, 70))


def draw(path, label_faces, draw_normals, caption):
    figure = plt.figure(figsize=(9, 6))
    axis = figure.add_subplot(111, projection="3d")
    for face in MESH["faces"]:
        axis.add_collection3d(Poly3DCollection(
            face["triangles"],
            facecolor=color_of(face),
            edgecolor="none",
            alpha=0.95,
        ))
    if label_faces:
        for item in TOPOLOGY["faces"]:
            if not item["region"]:
                continue
            x, _y, _z = item["center_mm"]
            axis.text(x, 112, 48, item["region"], fontsize=8)
    if draw_normals:
        for probe in MESH["probes"]:
            point = probe["point"]
            normal = probe["normal"]
            length = (normal[0] ** 2 + normal[1] ** 2 + normal[2] ** 2) ** 0.5
            scale = 18.0 / length
            axis.quiver(
                point[0], point[1], point[2],
                normal[0] * scale, normal[1] * scale, normal[2] * scale,
                color="#111111", arrow_length_ratio=0.2,
            )
    add_axes(axis)
    style(axis)
    axis.view_init(elev=24, azim=-62)
    if caption:
        figure.text(0.02, 0.96, caption, fontsize=9)
    figure.tight_layout()
    figure.savefig(path, dpi=140)
    plt.close(figure)


def main():
    draw(OUT / "coupon_overview.png", False, False, "180 x 100 x 41.3 mm    landmark: +X, y=84, z=5–15")
    draw(OUT / "coupon_faces.png", True, False, "")
    draw(OUT / "coupon_normals_probe.png", False, True, "")


main()
