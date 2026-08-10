#!/usr/bin/env python3
"""Render a hardware/software integration graphic for the cone-navigation car."""

from pathlib import Path
from typing import Iterable, Tuple

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parent.parent
OUTPUT = ROOT / "artifacts" / "hardware_software_integration.png"

Color = Tuple[int, int, int]


def load_font(name: str, size: int):
    candidates = [
        f"/System/Library/Fonts/Supplemental/{name}.ttf",
        f"/System/Library/Fonts/{name}.ttf",
    ]
    for path in candidates:
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            pass
    return ImageFont.load_default()


FONT_TITLE = load_font("Arial Bold", 44)
FONT_SECTION = load_font("Arial Bold", 27)
FONT_BOX = load_font("Arial Bold", 26)
FONT_BODY = load_font("Arial", 18)
FONT_SMALL = load_font("Arial", 15)


def hex_color(value: str) -> Color:
    value = value.lstrip("#")
    return tuple(int(value[idx : idx + 2], 16) for idx in (0, 2, 4))


COLORS = {
    "bg": hex_color("#f6f8fb"),
    "text": hex_color("#111827"),
    "muted": hex_color("#4b5563"),
    "line": hex_color("#344054"),
    "hw": hex_color("#dbeafe"),
    "hw_edge": hex_color("#2563eb"),
    "sw": hex_color("#dcfce7"),
    "sw_edge": hex_color("#16a34a"),
    "slam": hex_color("#ede9fe"),
    "slam_edge": hex_color("#7c3aed"),
    "plan": hex_color("#fef3c7"),
    "plan_edge": hex_color("#d97706"),
    "act": hex_color("#fee2e2"),
    "act_edge": hex_color("#dc2626"),
    "debug": hex_color("#e5e7eb"),
    "debug_edge": hex_color("#6b7280"),
    "strip": hex_color("#111827"),
    "strip_arrow": hex_color("#9ca3af"),
}


def draw_box(
    draw: ImageDraw.ImageDraw,
    xywh: Tuple[int, int, int, int],
    title: str,
    lines: Iterable[str],
    fill: Color,
    edge: Color,
) -> None:
    x, y, w, h = xywh
    draw.rounded_rectangle((x, y, x + w, y + h), radius=8, fill=fill, outline=edge, width=3)
    draw.text((x + 18, y + 16), title, font=FONT_BOX, fill=COLORS["text"])
    line_y = y + 58
    for line in lines:
        draw.text((x + 20, line_y), line, font=FONT_BODY, fill=COLORS["muted"])
        line_y += 26


def draw_arrow(
    draw: ImageDraw.ImageDraw,
    start: Tuple[int, int],
    end: Tuple[int, int],
    color: Color = COLORS["line"],
    width: int = 4,
) -> None:
    x1, y1 = start
    x2, y2 = end
    draw.line((x1, y1, x2, y2), fill=color, width=width)

    import math

    angle = math.atan2(y2 - y1, x2 - x1)
    size = 14
    p1 = (x2 - size * math.cos(angle - 0.45), y2 - size * math.sin(angle - 0.45))
    p2 = (x2 - size * math.cos(angle + 0.45), y2 - size * math.sin(angle + 0.45))
    draw.polygon((end, p1, p2), fill=color)


def draw_poly_arrow(
    draw: ImageDraw.ImageDraw,
    points: Iterable[Tuple[int, int]],
    color: Color = COLORS["line"],
    width: int = 4,
) -> None:
    points = list(points)
    for start, end in zip(points, points[1:-1]):
        draw.line((*start, *end), fill=color, width=width)
    draw_arrow(draw, points[-2], points[-1], color, width)


def render() -> Path:
    width, height = 1900, 1100
    image = Image.new("RGB", (width, height), COLORS["bg"])
    draw = ImageDraw.Draw(image)

    draw.text(
        (70, 45),
        "Autonomous Cone Navigation: Hardware and Software Integration",
        font=FONT_TITLE,
        fill=COLORS["text"],
    )
    draw.text(
        (73, 102),
        "Stereo vision, depth localization, Delaunay path planning, cubic spline smoothing, and pure pursuit control",
        font=FONT_BODY,
        fill=COLORS["muted"],
    )

    sections = [
        ("Hardware", 160, COLORS["hw_edge"]),
        ("Perception and Localization", 600, COLORS["sw_edge"]),
        ("Planning and Control", 1090, COLORS["plan_edge"]),
        ("Actuation / Debug", 1535, COLORS["act_edge"]),
    ]
    for label, x, color in sections:
        draw.text((x, 168), label, font=FONT_SECTION, fill=color)

    boxes = {
        "camera": (80, 230, 340, 150),
        "imu": (80, 430, 340, 130),
        "jetson": (80, 620, 340, 130),
        "detector": (540, 225, 390, 135),
        "localizer": (540, 410, 390, 145),
        "slam": (540, 640, 390, 145),
        "planner": (1040, 245, 390, 160),
        "pursuit": (1040, 525, 390, 150),
        "vesc": (1510, 250, 300, 135),
        "rviz": (1510, 535, 300, 145),
    }

    draw_box(draw, boxes["camera"], "ZED 2i Stereo Camera", ["left/right RGB images", "registered depth map", "camera intrinsics"], COLORS["hw"], COLORS["hw_edge"])
    draw_box(draw, boxes["imu"], "ZED IMU", ["angular velocity", "linear acceleration", "visual-inertial input"], COLORS["hw"], COLORS["hw_edge"])
    draw_box(draw, boxes["jetson"], "Jetson Orin Nano", ["ROS 2 Humble runtime", "CUDA + TensorRT", "onboard compute"], COLORS["hw"], COLORS["hw_edge"])

    draw_box(draw, boxes["detector"], "cone_detector_node", ["TensorRT YOLO inference", "/cone_detections", "blue / yellow / orange classes"], COLORS["sw"], COLORS["sw_edge"])
    draw_box(draw, boxes["localizer"], "cone_localizer_node", ["bbox center + ZED depth", "camera frame -> base_link", "/cones/left and /cones/right"], COLORS["sw"], COLORS["sw_edge"])
    draw_box(draw, boxes["slam"], "ORB-SLAM3 Sidecar", ["stereo-inertial odometry", "stereo images + IMU", "evaluation / future map fusion"], COLORS["slam"], COLORS["slam_edge"])

    draw_box(draw, boxes["planner"], "path_planner_node.py", ["Delaunay triangulation", "centerline midpoint extraction", "cubic spline smoothing", "/path and /path/markers"], COLORS["plan"], COLORS["plan_edge"])
    draw_box(draw, boxes["pursuit"], "pure_pursuit_node", ["adaptive lookahead", "steering clamp", "speed reduction in turns", "/drive"], COLORS["plan"], COLORS["plan_edge"])

    draw_box(draw, boxes["vesc"], "VESC / Vehicle", ["Ackermann steering", "throttle command", "safety stop on stale path"], COLORS["act"], COLORS["act_edge"])
    draw_box(draw, boxes["rviz"], "RViz / Debug Views", ["camera image", "3D cone markers", "green path line", "TF frames"], COLORS["debug"], COLORS["debug_edge"])

    # Dataflow arrows. Keep labels in a small legend instead of inline, avoiding clutter.
    draw_arrow(draw, (420, 280), (540, 280))
    draw_arrow(draw, (420, 315), (540, 475))
    draw_arrow(draw, (420, 490), (540, 710))
    draw_arrow(draw, (420, 685), (540, 305))

    draw_arrow(draw, (930, 292), (1040, 320))
    draw_arrow(draw, (930, 482), (1040, 330))
    draw_arrow(draw, (930, 712), (1040, 595), COLORS["slam_edge"])
    draw_arrow(draw, (1235, 405), (1235, 525), COLORS["plan_edge"])

    draw_poly_arrow(draw, [(1430, 600), (1470, 600), (1470, 318), (1510, 318)])
    draw_poly_arrow(draw, [(1430, 346), (1480, 346), (1480, 585), (1510, 585)])
    draw_poly_arrow(draw, [(930, 505), (970, 505), (970, 705), (1490, 705), (1490, 630), (1510, 630)], COLORS["debug_edge"], 3)

    legend_x, legend_y = 1028, 710
    draw.rounded_rectangle((legend_x, legend_y, legend_x + 390, legend_y + 115), radius=8, fill="#ffffff", outline="#cbd5e1", width=2)
    draw.text((legend_x + 18, legend_y + 15), "Key topic handoffs", font=FONT_BODY, fill=COLORS["text"])
    for idx, label in enumerate(["/cone_detections", "/cones/left + /cones/right", "/path + /path/markers", "/drive"]):
        draw.text((legend_x + 18, legend_y + 43 + idx * 18), label, font=FONT_SMALL, fill=COLORS["muted"])

    strip = (80, 875, 1820, 955)
    draw.rounded_rectangle(strip, radius=8, fill=COLORS["strip"])
    pipeline = [
        ("Image", 145),
        ("Detection2DArray", 365),
        ("PoseArray", 680),
        ("Path", 940),
        ("AckermannDriveStamped", 1150),
        ("Vehicle Motion", 1555),
    ]
    for idx, (label, x) in enumerate(pipeline):
        draw.text((x, 902), label, font=FONT_BODY, fill=(255, 255, 255))
        if idx < len(pipeline) - 1:
            label_right = draw.textbbox((x, 902), label, font=FONT_BODY)[2]
            draw_arrow(draw, (label_right + 34, 915), (pipeline[idx + 1][1] - 28, 915), COLORS["strip_arrow"], 3)

    draw.text(
        (82, 990),
        "Current planner is reactive in base_link. ORB-SLAM3 is wired for stereo-inertial odometry evaluation and future map-based fusion.",
        font=FONT_SMALL,
        fill=COLORS["muted"],
    )

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    image.save(OUTPUT)
    return OUTPUT


if __name__ == "__main__":
    print(render())
