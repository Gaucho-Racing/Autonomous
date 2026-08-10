#!/usr/bin/env python3
"""Render a simplified exploded hardware/software integration view."""

from pathlib import Path
from typing import Tuple

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parent.parent
OUTPUT = ROOT / "artifacts" / "car_exploded_integration.png"

Color = Tuple[int, int, int]


def load_font(name: str, size: int):
    for base in ("/System/Library/Fonts/Supplemental", "/System/Library/Fonts"):
        try:
            return ImageFont.truetype(f"{base}/{name}.ttf", size)
        except OSError:
            pass
    return ImageFont.load_default()


FONT_TITLE = load_font("Arial Bold", 48)
FONT_LABEL = load_font("Arial Bold", 22)
FONT_SMALL = load_font("Arial", 17)


def hex_color(value: str) -> Color:
    value = value.lstrip("#")
    return tuple(int(value[i : i + 2], 16) for i in (0, 2, 4))


BG = hex_color("#f7fafc")
INK = hex_color("#111827")
MUTED = hex_color("#64748b")
LINE = hex_color("#334155")
BLUE = hex_color("#2563eb")
GREEN = hex_color("#16a34a")
YELLOW = hex_color("#facc15")
ORANGE = hex_color("#f97316")
RED = hex_color("#dc2626")
PURPLE = hex_color("#7c3aed")
DARK = hex_color("#1f2937")
BODY = hex_color("#e5e7eb")
GLASS = hex_color("#bfdbfe")
TIRE = hex_color("#111827")
CHIP = hex_color("#dcfce7")
CHIP_EDGE = hex_color("#15803d")


def round_box(draw: ImageDraw.ImageDraw, xy, fill, edge, radius=12, width=3):
    draw.rounded_rectangle(xy, radius=radius, fill=fill, outline=edge, width=width)


def label(draw: ImageDraw.ImageDraw, center, text, fill=INK):
    bbox = draw.textbbox((0, 0), text, font=FONT_LABEL)
    draw.text((center[0] - (bbox[2] - bbox[0]) / 2, center[1]), text, font=FONT_LABEL, fill=fill)


def small(draw: ImageDraw.ImageDraw, center, text, fill=MUTED):
    bbox = draw.textbbox((0, 0), text, font=FONT_SMALL)
    draw.text((center[0] - (bbox[2] - bbox[0]) / 2, center[1]), text, font=FONT_SMALL, fill=fill)


def arrow(draw: ImageDraw.ImageDraw, start, end, color=LINE, width=4):
    import math

    draw.line((*start, *end), fill=color, width=width)
    angle = math.atan2(end[1] - start[1], end[0] - start[0])
    size = 15
    p1 = (end[0] - size * math.cos(angle - 0.45), end[1] - size * math.sin(angle - 0.45))
    p2 = (end[0] - size * math.cos(angle + 0.45), end[1] - size * math.sin(angle + 0.45))
    draw.polygon((end, p1, p2), fill=color)


def draw_camera(draw, x, y):
    round_box(draw, (x, y, x + 170, y + 90), hex_color("#dbeafe"), BLUE)
    draw.rectangle((x + 20, y + 34, x + 120, y + 56), fill=DARK)
    draw.ellipse((x + 28, y + 24, x + 72, y + 68), fill=INK, outline=GLASS, width=5)
    draw.ellipse((x + 78, y + 24, x + 122, y + 68), fill=INK, outline=GLASS, width=5)
    draw.rectangle((x + 126, y + 40, x + 150, y + 50), fill=BLUE)


def draw_imu(draw, x, y):
    round_box(draw, (x, y, x + 125, y + 95), hex_color("#eef2ff"), PURPLE)
    draw.arc((x + 23, y + 22, x + 82, y + 78), 25, 310, fill=PURPLE, width=5)
    arrow(draw, (x + 78, y + 42), (x + 98, y + 28), PURPLE, 4)
    draw.line((x + 34, y + 62, x + 68, y + 32), fill=PURPLE, width=4)
    draw.line((x + 68, y + 32, x + 97, y + 58), fill=PURPLE, width=4)


def draw_jetson(draw, x, y):
    round_box(draw, (x, y, x + 190, y + 120), hex_color("#dcfce7"), GREEN)
    draw.rectangle((x + 26, y + 27, x + 164, y + 92), fill=hex_color("#bbf7d0"), outline=GREEN, width=3)
    for px in range(x + 38, x + 154, 24):
        draw.rectangle((px, y + 15, px + 9, y + 27), fill=GREEN)
        draw.rectangle((px, y + 92, px + 9, y + 104), fill=GREEN)
    draw.ellipse((x + 62, y + 42, x + 126, y + 88), outline=GREEN, width=5)
    draw.ellipse((x + 80, y + 51, x + 108, y + 79), fill=GREEN)


def draw_vesc(draw, x, y):
    round_box(draw, (x, y, x + 165, y + 100), hex_color("#fee2e2"), RED)
    draw.rectangle((x + 25, y + 26, x + 140, y + 74), fill=hex_color("#fecaca"), outline=RED, width=3)
    for px in (x + 42, x + 72, x + 102):
        draw.line((px, y + 30, px, y + 70), fill=RED, width=4)
    draw.line((x + 140, y + 50, x + 158, y + 50), fill=RED, width=5)


def draw_cone(draw, x, y, color):
    draw.polygon(((x, y), (x - 28, y + 76), (x + 28, y + 76)), fill=color, outline=INK)
    draw.rectangle((x - 37, y + 76, x + 37, y + 91), fill=DARK)
    draw.line((x - 15, y + 42, x + 15, y + 42), fill=hex_color("#ffffff"), width=6)


def draw_car(draw):
    # Wheels first, then chassis.
    for x in (665, 965):
        for y in (420, 640):
            round_box(draw, (x, y, x + 86, y + 155), TIRE, TIRE, radius=22)
            draw.rectangle((x + 34, y + 12, x + 52, y + 143), fill=hex_color("#374151"))

    draw.rounded_rectangle((700, 330, 1020, 765), radius=78, fill=BODY, outline=INK, width=5)
    draw.rounded_rectangle((760, 390, 960, 535), radius=28, fill=GLASS, outline=BLUE, width=4)
    draw.rounded_rectangle((760, 565, 960, 700), radius=30, fill=hex_color("#d1d5db"), outline=hex_color("#9ca3af"), width=4)
    draw.rectangle((835, 300, 885, 350), fill=DARK)
    draw.rectangle((815, 285, 905, 315), fill=BLUE)
    draw.polygon(((860, 260), (817, 285), (903, 285)), fill=GLASS, outline=BLUE)
    draw.line((860, 350, 860, 750), fill=hex_color("#9ca3af"), width=3)
    label(draw, (860, 780), "Autonomous Car")


def draw_software_stack(draw):
    x, y = 1190, 315
    round_box(draw, (x, y, x + 290, y + 355), hex_color("#fffbeb"), ORANGE)
    label(draw, (x + 145, y + 20), "ROS 2 Stack", ORANGE)

    chips = [
        ("Detect", BLUE),
        ("Depth", GREEN),
        ("Plan", ORANGE),
        ("Control", RED),
    ]
    for i, (name, edge) in enumerate(chips):
        cy = y + 75 + i * 62
        round_box(draw, (x + 55, cy, x + 235, cy + 42), CHIP, edge, radius=8, width=3)
        label(draw, (x + 145, cy + 6), name, edge)

    draw.rounded_rectangle((x + 55, y + 310, x + 235, y + 335), radius=8, fill=hex_color("#ede9fe"), outline=PURPLE, width=2)
    small(draw, (x + 145, y + 313), "SLAM + IMU", PURPLE)


def render() -> Path:
    width, height = 1700, 1050
    image = Image.new("RGB", (width, height), BG)
    draw = ImageDraw.Draw(image)

    draw.text((70, 52), "Hardware + Software Integration", font=FONT_TITLE, fill=INK)
    draw.text((73, 112), "Exploded view of the autonomous cone-navigation car", font=FONT_SMALL, fill=MUTED)

    draw_car(draw)

    # Hardware objects around the car.
    draw_camera(draw, 190, 255)
    label(draw, (275, 355), "ZED 2i", BLUE)

    draw_imu(draw, 250, 515)
    label(draw, (312, 615), "IMU", PURPLE)

    draw_jetson(draw, 205, 745)
    label(draw, (300, 870), "Jetson", GREEN)

    draw_vesc(draw, 1320, 735)
    label(draw, (1403, 842), "VESC", RED)

    draw_software_stack(draw)

    # Visual perception scene: cones and path.
    draw_cone(draw, 570, 195, BLUE)
    draw_cone(draw, 650, 155, YELLOW)
    draw_cone(draw, 760, 135, BLUE)
    draw_cone(draw, 870, 160, YELLOW)
    draw_cone(draw, 970, 200, BLUE)
    draw.line((610, 284, 710, 255, 820, 250, 920, 282), fill=GREEN, width=7)

    # Simple exploded connectors.
    arrow(draw, (360, 300), (815, 300), BLUE, 4)
    arrow(draw, (375, 545), (700, 485), PURPLE, 4)
    arrow(draw, (395, 790), (730, 645), GREEN, 4)
    arrow(draw, (1015, 560), (1190, 485), ORANGE, 4)
    arrow(draw, (1340, 670), (1375, 735), RED, 4)
    arrow(draw, (1480, 500), (1515, 390), GREEN, 4)

    # Output/motion hint.
    draw.rounded_rectangle((1490, 235, 1635, 385), radius=18, fill=hex_color("#e0f2fe"), outline=BLUE, width=3)
    draw.arc((1525, 280, 1600, 355), 210, 30, fill=BLUE, width=8)
    arrow(draw, (1565, 313), (1620, 313), BLUE, 5)
    small(draw, (1562, 395), "motion", BLUE)

    # Minimal legend strip.
    draw.rounded_rectangle((190, 940, 1510, 990), radius=12, fill=INK)
    legend = [("Camera", BLUE), ("Depth", GREEN), ("SLAM/IMU", PURPLE), ("Planner", ORANGE), ("Drive", RED)]
    x = 290
    for name, color in legend:
        draw.ellipse((x, 958, x + 16, 974), fill=color)
        draw.text((x + 28, 953), name, font=FONT_SMALL, fill=hex_color("#ffffff"))
        x += 235

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    image.save(OUTPUT)
    return OUTPUT


if __name__ == "__main__":
    print(render())
