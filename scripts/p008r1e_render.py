# P-008-R1E 그림. capture_01 depth와 prism 산출만 사용한다.

import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


def load_cloud(path):
    raw = Path(path).read_bytes()
    count = int(np.frombuffer(raw, dtype="<u4", count=1)[0])
    return np.frombuffer(raw, dtype="<f4", count=count * 3, offset=4).reshape(count, 3)


def load_clusters(path):
    raw = Path(path).read_bytes()
    count = int(np.frombuffer(raw, dtype="<u4", count=1)[0])
    record = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("id", "<i4")])
    data = np.frombuffer(raw, dtype=record, count=count, offset=4)
    xyz = np.column_stack([data["x"], data["y"], data["z"]]).astype(np.float64)
    return xyz, data["id"].astype(np.int32)


def main():
    out_dir = Path(sys.argv[1])
    depth_path = Path(sys.argv[2])
    fx = fy = 410.4136657714844
    cx = 423.66876220703125
    cy = 240.9937744140625
    depth = np.array(Image.open(depth_path))
    view = depth.astype(np.float32)
    low, high = np.percentile(view[view > 0], [2, 98])
    image = np.clip((view - low) / (high - low), 0, 1)
    image[view == 0] = 0
    base = (np.dstack([image, image, image]) * 255).astype(np.uint8)

    def project(points):
        xyz = np.asarray(points, dtype=np.float64)
        return np.rint(fx * xyz[:, 0] / xyz[:, 2] + cx).astype(int), np.rint(
            fy * xyz[:, 1] / xyz[:, 2] + cy
        ).astype(int)

    def paint(canvas, points, color, step):
        pixels = canvas.load()
        xs, ys = project(points)
        for x_coord, y_coord in zip(xs[::step], ys[::step]):
            if 0 <= x_coord < 848 and 0 <= y_coord < 480:
                pixels[int(x_coord), int(y_coord)] = color

    def save(name, draw_calls):
        canvas = Image.fromarray(base.copy(), "RGB")
        for points, color, step in draw_calls:
            if len(points) == 0:
                continue
            paint(canvas, points, color, step)
        draw = ImageDraw.Draw(canvas)
        draw.text((8, 460), name, fill=(255, 255, 255))
        canvas.save(out_dir / "figures" / f"{name}.png")

    inliers = load_cloud(out_dir / "stage_inliers.bin")
    hull = load_cloud(out_dir / "stage_hull.bin")
    selected = load_cloud(out_dir / "stage_prism_selected.bin")
    rejected = load_cloud(out_dir / "stage_prism_rejected.bin")
    disk = load_cloud(out_dir / "stage_shoe_disk.bin")
    shoe = load_cloud(out_dir / "stage_shoe_cluster.bin")
    clusters, labels = load_clusters(out_dir / "stage_clusters.bin")
    figure_dir = out_dir / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    save("01_plane_hull", [(inliers, (170, 170, 170), 2), (hull, (255, 220, 0), 1)])
    save("02_old_halfspace_failure", [(disk, (255, 40, 40), 1)])
    save("03_prism_selected_points", [(selected, (40, 210, 80), 3), (disk, (255, 220, 0), 1)])
    save("04_prism_rejected_points", [(rejected, (220, 60, 60), 5)])
    save(
        "05_clusters_after_prism",
        [
            (clusters[labels == 0], (255, 140, 0), 3),
            (clusters[labels == 1], (255, 255, 255), 2),
            (clusters[labels == 7], (0, 220, 255), 1),
            (disk, (255, 220, 0), 2),
        ],
    )
    save("06_shoe_cluster", [(shoe, (80, 160, 255), 2), (disk, (255, 220, 0), 1)])


if __name__ == "__main__":
    main()
