import os
import json
import shutil
from pathlib import Path
from PIL import Image
from tqdm import tqdm
import random

# Paths
FSOCO_DIR = Path("/Users/adi/Downloads/fsoco_bounding_boxes_train")
OUTPUT_DIR = Path("/Users/adi/Desktop/PycharmProjects/Autonomous/data/yolo_format")

# Class mapping (FSOCO -> YOLO index)
CLASS_MAP = {
    "blue_cone": 0,
    "yellow_cone": 1,
    "orange_cone": 2,
    "large_orange_cone": 3,
    "unknown_cone": 4,
    # Handle variations in naming
    "blue": 0,
    "yellow": 1,
    "orange": 2,
    "big_orange": 3,
    "orange_big": 3,
}

# Train/val split ratio
TRAIN_RATIO = 0.85


def supervisely_to_yolo(annotation, img_width, img_height):
    """Convert Supervisely annotation to YOLO format."""
    yolo_labels = []

    objects = annotation.get("objects", [])
    for obj in objects:
        class_title = obj.get("classTitle", "").lower().replace(" ", "_")

        # Map to class index
        class_idx = None
        for key, idx in CLASS_MAP.items():
            if key in class_title:
                class_idx = idx
                break

        if class_idx is None:
            continue

        # Get bounding box
        points = obj.get("points", {})
        exterior = points.get("exterior", [])

        if len(exterior) >= 2:
            x1, y1 = exterior[0]
            x2, y2 = exterior[1]

            # Convert to YOLO format (normalized center x, center y, width, height)
            x_center = ((x1 + x2) / 2) / img_width
            y_center = ((y1 + y2) / 2) / img_height
            width = abs(x2 - x1) / img_width
            height = abs(y2 - y1) / img_height

            # Clamp values
            x_center = max(0, min(1, x_center))
            y_center = max(0, min(1, y_center))
            width = max(0, min(1, width))
            height = max(0, min(1, height))

            if width > 0 and height > 0:
                yolo_labels.append(f"{class_idx} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}")

    return yolo_labels


def convert_dataset():
    """Convert entire FSOCO dataset to YOLO format."""

    # Create output directories
    for split in ["train", "val"]:
        (OUTPUT_DIR / split / "images").mkdir(parents=True, exist_ok=True)
        (OUTPUT_DIR / split / "labels").mkdir(parents=True, exist_ok=True)

    # Find all image/annotation pairs
    all_samples = []

    # FSOCO structure: team_folders containing img/ and ann/ subfolders
    for team_dir in FSOCO_DIR.iterdir():
        if not team_dir.is_dir():
            continue

        img_dir = team_dir / "img"
        ann_dir = team_dir / "ann"

        if not img_dir.exists() or not ann_dir.exists():
            # Try alternate structure
            img_dir = team_dir
            ann_dir = team_dir

        if not img_dir.exists():
            continue

        # Find images
        for img_path in img_dir.glob("*.[jJpP][pPnN][gG]*"):
            # Find corresponding annotation
            ann_name = img_path.stem + ".json"
            ann_path = ann_dir / ann_name

            if not ann_path.exists():
                # Try with image extension
                ann_path = ann_dir / (img_path.name + ".json")

            if ann_path.exists():
                all_samples.append((img_path, ann_path))

    print(f"Found {len(all_samples)} image-annotation pairs")

    if len(all_samples) == 0:
        print("\nNo samples found! Check the FSOCO directory structure.")
        print(f"Expected location: {FSOCO_DIR}")
        print("\nThe structure should be:")
        print("  fsoco/")
        print("    team1/")
        print("      img/")
        print("      ann/")
        print("    team2/")
        print("      ...")
        return

    # Shuffle and split
    random.seed(42)
    random.shuffle(all_samples)

    split_idx = int(len(all_samples) * TRAIN_RATIO)
    train_samples = all_samples[:split_idx]
    val_samples = all_samples[split_idx:]

    print(f"Train: {len(train_samples)}, Val: {len(val_samples)}")

    # Process samples
    stats = {"train": 0, "val": 0, "cones": 0}

    for split, samples in [("train", train_samples), ("val", val_samples)]:
        print(f"\nProcessing {split} set...")

        for img_path, ann_path in tqdm(samples):
            try:
                # Load image to get dimensions
                with Image.open(img_path) as img:
                    img_width, img_height = img.size

                # Load annotation
                with open(ann_path, 'r') as f:
                    annotation = json.load(f)

                # Convert to YOLO format
                yolo_labels = supervisely_to_yolo(annotation, img_width, img_height)

                if len(yolo_labels) == 0:
                    continue

                # Generate unique filename
                out_name = f"{img_path.parent.parent.name}_{img_path.stem}"

                # Copy image
                out_img_path = OUTPUT_DIR / split / "images" / f"{out_name}{img_path.suffix}"
                shutil.copy(img_path, out_img_path)

                # Write labels
                out_label_path = OUTPUT_DIR / split / "labels" / f"{out_name}.txt"
                with open(out_label_path, 'w') as f:
                    f.write("\n".join(yolo_labels))

                stats[split] += 1
                stats["cones"] += len(yolo_labels)

            except Exception as e:
                print(f"\nError processing {img_path}: {e}")
                continue

    print(f"\n{'='*60}")
    print("Conversion complete!")
    print(f"Train images: {stats['train']}")
    print(f"Val images: {stats['val']}")
    print(f"Total cones: {stats['cones']}")
    print(f"Output: {OUTPUT_DIR}")
    print(f"{'='*60}")


if __name__ == "__main__":
    convert_dataset()