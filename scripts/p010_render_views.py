#!/usr/bin/env python3
"""P-010. 검증 메시와 법선 표본으로 CAD 그림 세 장을 만든다."""

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = Path("/Users/woonkil/projects/cad-less-robotics")
OUT = ROOT / "evidence/P-010"
MESH = json.loads((OUT / "mesh_faces.json").read_text(encoding="utf-8"))
VALIDATION = json.loads((OUT / "cad_validation.json").read_text(encoding="utf-8"))
GEOMETRY = json.loads((OUT / "cad_geometry.json").read_text(encoding="utf-8"))
COLORS = ("#d4a017", "#1a7a72", "#c0392b")


def triangles():
    grouped = []
    for face in MESH:
        points = face["points"]
        faces = []
        for facet in face["facets"]:
            faces.append([points[index] for index in facet])
        grouped.append(faces)
    return grouped


def add_box(axis, box):
    xmin, ymin, zmin = box["min"]
    xmax, ymax, zmax = box["max"]
    corners = [
        (xmin, ymin, zmin), (xmax, ymin, zmin), (xmax, ymax, zmin), (xmin, ymax, zmin),
        (xmin, ymin, zmax), (xmax, ymin, zmax), (xmax, ymax, zmax), (xmin, ymax, zmax),
    ]
    edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
    for start, end in edges:
        axis.plot(
            [corners[start][0], corners[end][0]],
            [corners[start][1], corners[end][1]],
            [corners[start][2], corners[end][2]],
            color="#333333", linewidth=0.6,
        )


def add_axes(axis, origin, length):
    axis.quiver(origin[0], origin[1], origin[2], length, 0, 0, color="#c0392b", arrow_length_ratio=0.08)
    axis.quiver(origin[0], origin[1], origin[2], 0, length, 0, color="#1e8449", arrow_length_ratio=0.08)
    axis.quiver(origin[0], origin[1], origin[2], 0, 0, length, color="#1a5276", arrow_length_ratio=0.08)
    axis.text(origin[0] + length, origin[1], origin[2], "X")
    axis.text(origin[0], origin[1] + length, origin[2], "Y")
    axis.text(origin[0], origin[1], origin[2] + length, "Z")


def style(axis):
    axis.set_xlabel("X mm")
    axis.set_ylabel("Y mm")
    axis.set_zlabel("Z mm")
    axis.set_box_aspect((300, 100, 120))


def main():
    grouped = triangles()
    box = GEOMETRY["bounding_box_mm"]
    origin = box["min"]
    figure = plt.figure(figsize=(8, 6))
    axis = figure.add_subplot(111, projection="3d")
    for index, faces in enumerate(grouped):
        axis.add_collection3d(Poly3DCollection(faces, facecolor=COLORS[index], edgecolor="none", alpha=0.9))
    add_box(axis, box)
    add_axes(axis, origin, 40)
    style(axis)
    axis.view_init(elev=18, azim=-58)
    figure.tight_layout()
    figure.savefig(OUT / "cad_overview.png", dpi=140)
    plt.close(figure)

    figure = plt.figure(figsize=(8, 6))
    axis = figure.add_subplot(111, projection="3d")
    for index, faces in enumerate(grouped):
        axis.add_collection3d(Poly3DCollection(
            faces,
            facecolor=COLORS[index],
            edgecolor="#222222",
            linewidths=0.15,
            alpha=0.22 if index == 0 else 1.0,
        ))
        centroid = [
            sum(point[axis_index] for triangle in faces for point in triangle) / (3 * len(faces))
            for axis_index in range(3)
        ]
        axis.text(centroid[0], centroid[1], centroid[2] + 8, f"F{index}", color="black")
    style(axis)
    axis.view_init(elev=18, azim=-58)
    figure.tight_layout()
    figure.savefig(OUT / "cad_faces.png", dpi=140)
    plt.close(figure)

    figure = plt.figure(figsize=(8, 6))
    axis = figure.add_subplot(111, projection="3d")
    for index, faces in enumerate(grouped):
        axis.add_collection3d(Poly3DCollection(faces, facecolor=COLORS[index], edgecolor="none", alpha=0.35))
    for face in VALIDATION["runs"][0]["faces"]:
        for sample in face["samples"]:
            if abs(sample["u"] - face["samples"][4]["u"]) > 1e-9:
                continue
            if abs(sample["v"] - face["samples"][4]["v"]) > 1e-9:
                continue
            point = sample["point_mm"]
            normal = sample["normal"]
            axis.quiver(
                point[0], point[1], point[2],
                normal[0] * 25, normal[1] * 25, normal[2] * 25,
                color="#1a1a1a", arrow_length_ratio=0.2,
            )
    style(axis)
    axis.view_init(elev=18, azim=-58)
    figure.tight_layout()
    figure.savefig(OUT / "cad_normals.png", dpi=140)
    plt.close(figure)


if __name__ == "__main__":
    main()
