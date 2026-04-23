"""
Train YOLO on the FSOCO cone dataset using repo-local paths.
"""

from pathlib import Path

import torch
from ultralytics import YOLO


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
DATA_CONFIG = REPO_ROOT / "fsoco.yaml"
DATA_DIR = REPO_ROOT / "data" / "yolo_format"
OUTPUT_DIR = REPO_ROOT / "models"
RUN_NAME = "fsoco_yolo26n"

CONFIG = {
    "model": "yolo26n.pt",
    "epochs": 100,
    "imgsz": 640,
    "batch": 8,
    "patience": 20,
    "device": "mps",
    "workers": 0,
    "project": str(OUTPUT_DIR),
    "name": RUN_NAME,
    "exist_ok": True,
    "pretrained": True,
    "optimizer": "AdamW",
    "lr0": 0.001,
    "lrf": 0.01,
    "momentum": 0.937,
    "weight_decay": 0.0005,
    "warmup_epochs": 3,
    "box": 7.5,
    "cls": 0.5,
    "dfl": 1.5,
    "augment": True,
    "hsv_h": 0.015,
    "hsv_s": 0.7,
    "hsv_v": 0.4,
    "degrees": 10,
    "translate": 0.1,
    "scale": 0.5,
    "flipud": 0.0,
    "fliplr": 0.5,
    "mosaic": 1.0,
    "mixup": 0.1,
    "amp": False,
    "deterministic": False,
}


def check_device():
    if torch.backends.mps.is_available():
        print("MPS available")
        return "mps"
    if torch.cuda.is_available():
        print("CUDA available")
        return "cuda:0"
    print("GPU not available, using CPU")
    return "cpu"


def existing_checkpoint():
    weights_dir = OUTPUT_DIR / RUN_NAME / "weights"
    last_ckpt = weights_dir / "last.pt"
    best_ckpt = weights_dir / "best.pt"
    if last_ckpt.exists():
        return last_ckpt
    if best_ckpt.exists():
        return best_ckpt
    return None


def train():
    print("=" * 60)
    print("FSOCO Cone Detection Training")
    print("=" * 60)
    print(f"Repo root: {REPO_ROOT}")
    print(f"Dataset config: {DATA_CONFIG}")
    print(f"Dataset root: {DATA_DIR}")
    print(f"Training output: {OUTPUT_DIR / RUN_NAME}")

    CONFIG["device"] = check_device()

    if not DATA_CONFIG.exists():
        print("\nDataset config not found.")
        print("Run ConvertToYOLO.py first to generate fsoco.yaml and data/yolo_format.")
        return

    train_dir = DATA_DIR / "train" / "images"
    val_dir = DATA_DIR / "val" / "images"
    if not train_dir.exists() or not any(train_dir.iterdir()):
        print(f"\nTraining data not found in: {train_dir}")
        print("Run ConvertToYOLO.py first.")
        return
    if not val_dir.exists() or not any(val_dir.iterdir()):
        print(f"\nValidation data not found in: {val_dir}")
        print("Run ConvertToYOLO.py first.")
        return

    checkpoint = existing_checkpoint()
    model_path = str(checkpoint) if checkpoint else CONFIG["model"]

    print(f"Loading model: {model_path}")
    model = YOLO(model_path)

    print("\nStarting training...")
    print(f"Epochs: {CONFIG['epochs']}")
    print(f"Image size: {CONFIG['imgsz']}")
    print(f"Batch size: {CONFIG['batch']}")
    print(f"Device: {CONFIG['device']}")
    print("-" * 60)

    model.train(
        data=str(DATA_CONFIG),
        epochs=CONFIG["epochs"],
        imgsz=CONFIG["imgsz"],
        batch=CONFIG["batch"],
        patience=CONFIG["patience"],
        device=CONFIG["device"],
        workers=CONFIG["workers"],
        project=CONFIG["project"],
        name=CONFIG["name"],
        exist_ok=CONFIG["exist_ok"],
        pretrained=CONFIG["pretrained"],
        optimizer=CONFIG["optimizer"],
        lr0=CONFIG["lr0"],
        lrf=CONFIG["lrf"],
        momentum=CONFIG["momentum"],
        weight_decay=CONFIG["weight_decay"],
        warmup_epochs=CONFIG["warmup_epochs"],
        box=CONFIG["box"],
        cls=CONFIG["cls"],
        dfl=CONFIG["dfl"],
        hsv_h=CONFIG["hsv_h"],
        hsv_s=CONFIG["hsv_s"],
        hsv_v=CONFIG["hsv_v"],
        degrees=CONFIG["degrees"],
        translate=CONFIG["translate"],
        scale=CONFIG["scale"],
        flipud=CONFIG["flipud"],
        fliplr=CONFIG["fliplr"],
        mosaic=CONFIG["mosaic"],
        mixup=CONFIG["mixup"],
        amp=CONFIG["amp"],
        deterministic=CONFIG["deterministic"],
    )

    best_model = OUTPUT_DIR / RUN_NAME / "weights" / "best.pt"
    print("\n" + "=" * 60)
    print("Training complete")
    print("=" * 60)
    print(f"Best model: {best_model}")

    if best_model.exists():
        print("\nExporting ONNX next to the trained weights...")
        model_best = YOLO(str(best_model))
        model_best.export(format="onnx", imgsz=CONFIG["imgsz"], simplify=True)
        print(f"ONNX export expected at: {best_model.with_suffix('.onnx')}")

    print("\nNext steps:")
    print("1. Validate with: yolo detect val model=models/fsoco_yolo26n/weights/best.pt data=fsoco.yaml")
    print("2. Export TensorRT on the Jetson from best.pt")
    print("3. Place the engine at cone_nav/models/cone_yolo.engine")


if __name__ == "__main__":
    train()
