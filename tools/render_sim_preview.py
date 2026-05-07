#!/usr/bin/env python3
"""Render a lightweight RViz-style preview of cones, robot, and planned path."""

import math
from pathlib import Path
from typing import List, Sequence, Set, Tuple

from PIL import Image, ImageDraw, ImageFont


Point2 = Tuple[float, float]
LabeledPoint = Tuple[float, float, str]
Triangle = Tuple[int, int, int]
Edge = Tuple[int, int]


ROOT = Path(__file__).resolve().parent.parent
OUTPUT = ROOT / "artifacts" / "sim_cones_robot_path.png"


def distance(point: Point2) -> float:
    return math.hypot(point[0], point[1])


def rolling_average(points: Sequence[Point2], window: int) -> List[Point2]:
    if window <= 1 or len(points) <= 1:
        return list(points)

    half = window // 2
    smoothed = []
    for idx in range(len(points)):
        span = points[max(0, idx - half) : min(len(points), idx + half + 1)]
        smoothed.append(
            (
                sum(point[0] for point in span) / len(span),
                sum(point[1] for point in span) / len(span),
            )
        )
    return smoothed


def cross(a: Point2, b: Point2, c: Point2) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def circumcircle_contains(a: Point2, b: Point2, c: Point2, p: Point2) -> bool:
    ax = a[0] - p[0]
    ay = a[1] - p[1]
    bx = b[0] - p[0]
    by = b[1] - p[1]
    cx = c[0] - p[0]
    cy = c[1] - p[1]
    det = (
        (ax * ax + ay * ay) * (bx * cy - by * cx)
        - (bx * bx + by * by) * (ax * cy - ay * cx)
        + (cx * cx + cy * cy) * (ax * by - ay * bx)
    )
    return det > 1e-9 if cross(a, b, c) > 0.0 else det < -1e-9


def delaunay(points: Sequence[LabeledPoint]) -> Set[Triangle]:
    triangles = set()
    xy = [(point[0], point[1]) for point in points]
    for i in range(len(points) - 2):
        for j in range(i + 1, len(points) - 1):
            for k in range(j + 1, len(points)):
                a, b, c = xy[i], xy[j], xy[k]
                if abs(cross(a, b, c)) < 1e-9:
                    continue
                if all(
                    m in (i, j, k) or not circumcircle_contains(a, b, c, xy[m])
                    for m in range(len(points))
                ):
                    triangles.add((i, j, k))
    return triangles


def delaunay_edges(triangles: Sequence[Triangle]) -> Set[Edge]:
    edges = set()
    for i, j, k in triangles:
        edges.add(tuple(sorted((i, j))))
        edges.add(tuple(sorted((j, k))))
        edges.add(tuple(sorted((i, k))))
    return edges


def midpoints_from_delaunay(points: Sequence[LabeledPoint]) -> List[Point2]:
    mids = []
    for i, j in sorted(delaunay_edges(delaunay(points))):
        a = points[i]
        b = points[j]
        if a[2] == b[2]:
            continue
        if math.hypot(a[0] - b[0], a[1] - b[1]) > 6.0:
            continue
        midpoint = ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5)
        if midpoint[0] >= -0.2:
            mids.append(midpoint)

    mids.sort(key=lambda point: (point[0], distance(point), abs(point[1])))
    deduped = []
    for point in mids:
        if not deduped or math.hypot(point[0] - deduped[-1][0], point[1] - deduped[-1][1]) > 0.2:
            deduped.append(point)
    return deduped


def catmull_rom(points: Sequence[Point2], samples_per_segment: int = 6) -> List[Point2]:
    if len(points) < 3:
        return list(points)

    padded = [points[0], *points, points[-1]]
    smoothed = [points[0]]
    for idx in range(1, len(padded) - 2):
        p0, p1, p2, p3 = padded[idx - 1], padded[idx], padded[idx + 1], padded[idx + 2]
        for sample_idx in range(1, samples_per_segment + 1):
            t = sample_idx / samples_per_segment
            t2 = t * t
            t3 = t2 * t
            x = 0.5 * (
                2.0 * p1[0]
                + (-p0[0] + p2[0]) * t
                + (2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]) * t2
                + (-p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]) * t3
            )
            y = 0.5 * (
                2.0 * p1[1]
                + (-p0[1] + p2[1]) * t
                + (2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]) * t2
                + (-p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]) * t3
            )
            smoothed.append((x, y))
    return smoothed


def build_scene():
    left = [
        (1.2, 1.05),
        (2.5, 1.32),
        (3.8, 1.48),
        (5.2, 1.56),
        (6.6, 1.42),
        (8.0, 1.18),
        (9.3, 0.82),
        (10.5, 0.42),
    ]
    right = [
        (1.0, -1.05),
        (2.3, -1.18),
        (3.7, -1.22),
        (5.1, -1.14),
        (6.5, -0.92),
        (7.9, -0.58),
        (9.1, -0.16),
        (10.3, 0.22),
    ]
    labeled = [(*point, "left") for point in left] + [(*point, "right") for point in right]
    midpoints = [(0.0, 0.0)] + rolling_average(midpoints_from_delaunay(labeled), 5)
    path = catmull_rom(midpoints, 8)
    return left, right, labeled, delaunay_edges(delaunay(labeled)), path


def render():
    width, height = 1500, 900
    margin = 95
    x_min, x_max = -1.0, 11.5
    y_min, y_max = -3.0, 3.0

    def world(point: Point2) -> Tuple[int, int]:
        x, y = point
        px = margin + (x - x_min) / (x_max - x_min) * (width - 2 * margin)
        py = height - margin - (y - y_min) / (y_max - y_min) * (height - 2 * margin)
        return int(px), int(py)

    image = Image.new("RGB", (width, height), (28, 32, 35))
    draw = ImageDraw.Draw(image)
    left, right, labeled, edges, path = build_scene()

    grid_color = (63, 70, 74)
    axis_color = (103, 114, 120)
    for x in range(0, 12):
        draw.line([world((x, y_min)), world((x, y_max))], fill=grid_color, width=1)
    for y_index in range(-3, 4):
        draw.line([world((x_min, y_index)), world((x_max, y_index))], fill=grid_color, width=1)
    draw.line([world((0, y_min)), world((0, y_max))], fill=axis_color, width=2)
    draw.line([world((x_min, 0)), world((x_max, 0))], fill=axis_color, width=2)

    # Delaunay triangulation overlay.
    for i, j in edges:
        a = labeled[i]
        b = labeled[j]
        color = (88, 112, 125) if a[2] == b[2] else (118, 150, 158)
        draw.line([world((a[0], a[1])), world((b[0], b[1]))], fill=color, width=1)

    # Smoothed path line.
    path_pixels = [world(point) for point in path]
    if len(path_pixels) > 1:
        draw.line(path_pixels, fill=(40, 235, 105), width=8, joint="curve")
        draw.line(path_pixels, fill=(180, 255, 195), width=3, joint="curve")

    def cone(point: Point2, fill: Tuple[int, int, int], outline: Tuple[int, int, int]):
        cx, cy = world(point)
        size = 17
        triangle = [(cx, cy - size), (cx - size, cy + size), (cx + size, cy + size)]
        draw.polygon(triangle, fill=fill, outline=outline)
        draw.line([(cx - size, cy + size), (cx + size, cy + size)], fill=outline, width=2)

    for point in left:
        cone(point, (245, 217, 55), (255, 246, 150))
    for point in right:
        cone(point, (48, 118, 245), (160, 198, 255))

    # Robot footprint at base_link.
    robot_center = world((0.0, 0.0))
    rx, ry = robot_center
    body = [(rx - 28, ry - 18), (rx + 36, ry - 18), (rx + 52, ry), (rx + 36, ry + 18), (rx - 28, ry + 18)]
    draw.polygon(body, fill=(225, 230, 233), outline=(255, 255, 255))
    draw.line([robot_center, world((0.75, 0.0))], fill=(255, 90, 80), width=5)
    draw.ellipse((rx - 5, ry - 5, rx + 5, ry + 5), fill=(255, 90, 80))

    font = ImageFont.load_default()
    draw.text((40, 30), "Simulated cone-navigation preview", fill=(235, 240, 242), font=font)
    draw.text((40, 54), "Blue/yellow cones, robot base_link, Delaunay edges, cubic-spline path", fill=(180, 190, 195), font=font)
    draw.text((40, height - 55), "+X forward   +Y left   fixed frame: base_link", fill=(180, 190, 195), font=font)

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    image.save(OUTPUT)
    print(OUTPUT)


if __name__ == "__main__":
    render()
