import json
import random
import shutil
from pathlib import Path

from PIL import Image
from tqdm import tqdm


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
DEFAULT_FSOCO_DIR = Path.home() / "Downloads" / "fsoco_bounding_boxes_train"
FSOCO_DIR = Path(
    __import__("os").environ.get("FSOCO_DIR", str(DEFAULT_FSOCO_DIR))
).expanduser().resolve()
OUTPUT_DIR = REPO_ROOT / "data" / "yolo_format"
DATA_CONFIG_PATH = REPO_ROOT / "fsoco.yaml"

# Collapse everything to the 3 runtime classes expected by cone_nav.
CLASS_MAP = {
    "blue_cone": 0,
    "blue": 0,
    "yellow_cone": 1,
    "yellow": 1,
    "orange_cone": 2,
    "large_orange_cone": 2,
    "big_orange": 2,
    "orange_big": 2,
    "orange": 2,
}

CLASS_NAMES = {
    0: "blue_cone",
    1: "yellow_cone",
    2: "orange_cone",
}

TRAIN_RATIO = 0.85


def supervisely_to_yolo(annotation, img_width, img_height):
    """Convert one Supervisely annotation into YOLO labels."""
    yolo_labels = []

    for obj in annotation.get("objects", []):
        class_title = obj.get("classTitle", "").lower().replace(" ", "_")

        class_idx = None
        for key, idx in CLASS_MAP.items():
            if key in class_title:
                class_idx = idx
                break

        if class_idx is None:
            continue

        points = obj.get("points", {})
        exterior = points.get("exterior", [])
        if len(exterior) < 2:
          continue

        x1, y1 = exterior[0]
        x2, y2 = exterior[1]

        x_center = ((x1 + x2) / 2.0) / img_width
        y_center = ((y1 + y2) / 2.0) / img_height
        width = abs(x2 - x1) / img_width
        height = abs(y2 - y1) / img_height

        x_center = max(0.0, min(1.0, x_center))
        y_center = max(0.0, min(1.0, y_center))
        width = max(0.0, min(1.0, width))
        height = max(0.0, min(1.0, height))

        if width > 0.0 and height > 0.0:
            yolo_labels.append(
                f"{class_idx} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}"
            )

    return yolo_labels


def write_data_config():
    """Write fsoco.yaml in the repo root."""
    lines = [
        f"path: {OUTPUT_DIR}",
        "train: train/images",
        "val: val/images",
        "",
        "names:",
    ]
    for class_idx, class_name in CLASS_NAMES.items():
        lines.append(f"  {class_idx}: {class_name}")

    DATA_CONFIG_PATH.write_text("\n".join(lines) + "\n", encoding="ascii")
    print(f"Wrote dataset config: {DATA_CONFIG_PATH}")


def find_samples():
    """Discover image/annotation pairs in the FSOCO directory."""
    all_samples = []

    for team_dir in FSOCO_DIR.iterdir():
        if not team_dir.is_dir():
            continue

        img_dir = team_dir / "img"
        ann_dir = team_dir / "ann"

        if not img_dir.exists() or not ann_dir.exists():
            img_dir = team_dir
            ann_dir = team_dir

        if not img_dir.exists():
            continue

        for img_path in img_dir.glob("*.[jJpP][pPnN][gG]*"):
            ann_path = ann_dir / f"{img_path.stem}.json"
            if not ann_path.exists():
                ann_path = ann_dir / f"{img_path.name}.json"

            if ann_path.exists():
                all_samples.append((img_path, ann_path))

    return all_samples


def reset_output_dirs():
    """Remove stale converted data and recreate the YOLO folder layout."""
    if OUTPUT_DIR.exists():
        shutil.rmtree(OUTPUT_DIR)

    for split in ("train", "val"):
        (OUTPUT_DIR / split / "images").mkdir(parents=True, exist_ok=True)
        (OUTPUT_DIR / split / "labels").mkdir(parents=True, exist_ok=True)


def convert_dataset():
    """Convert FSOCO into YOLO train/val folders inside this repo."""
    print("=" * 60)
    print("FSOCO -> YOLO Dataset Conversion")
    print("=" * 60)
    print(f"Source dataset: {FSOCO_DIR}")
    print(f"Output dataset: {OUTPUT_DIR}")

    if not FSOCO_DIR.exists():
        print("\nDataset directory not found.")
        print(f"Expected location: {FSOCO_DIR}")
        print("Set FSOCO_DIR to override the source path.")
        return

    reset_output_dirs()
    all_samples = find_samples()

    print(f"Found {len(all_samples)} image-annotation pairs")
    if not all_samples:
        print("\nNo samples found. Expected a structure like:")
        print("  fsoco/")
        print("    team1/")
        print("      img/")
        print("      ann/")
        return

    random.seed(42)
    random.shuffle(all_samples)

    split_idx = int(len(all_samples) * TRAIN_RATIO)
    train_samples = all_samples[:split_idx]
    val_samples = all_samples[split_idx:]

    print(f"Train: {len(train_samples)}, Val: {len(val_samples)}")

    stats = {"train": 0, "val": 0, "cones": 0}
    for split, samples in (("train", train_samples), ("val", val_samples)):
        print(f"\nProcessing {split} set...")

        for img_path, ann_path in tqdm(samples):
            try:
                with Image.open(img_path) as img:
                    img_width, img_height = img.size

                with ann_path.open("r", encoding="utf-8") as handle:
                    annotation = json.load(handle)

                yolo_labels = supervisely_to_yolo(annotation, img_width, img_height)
                if not yolo_labels:
                    continue

                out_name = f"{img_path.parent.parent.name}_{img_path.stem}"
                out_img_path = OUTPUT_DIR / split / "images" / f"{out_name}{img_path.suffix.lower()}"
                out_label_path = OUTPUT_DIR / split / "labels" / f"{out_name}.txt"

                shutil.copy(img_path, out_img_path)
                out_label_path.write_text("\n".join(yolo_labels), encoding="ascii")

                stats[split] += 1
                stats["cones"] += len(yolo_labels)
            except Exception as exc:
                print(f"\nError processing {img_path}: {exc}")

    write_data_config()

    print(f"\n{'=' * 60}")
    print("Conversion complete")
    print(f"Train images: {stats['train']}")
    print(f"Val images: {stats['val']}")
    print(f"Total cones: {stats['cones']}")
    print(f"Output dataset: {OUTPUT_DIR}")
    print(f"Dataset config: {DATA_CONFIG_PATH}")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    convert_dataset()
