"""
Train YOLOv8 on FSOCO dataset for cone detection.
"""

import os
from pathlib import Path
from ultralytics import YOLO
import torch

# Paths
PROJECT_DIR = Path("/Users/adi/Desktop/PycharmProjects/Autonomous")
DATA_CONFIG = PROJECT_DIR / "fsoco.yaml"
OUTPUT_DIR = PROJECT_DIR / "models"

# Training configuration
CONFIG = {
    "model": "yolov8n.pt", # nano model - fast, good for edge deployment
    "epochs": 100,
    "imgsz": 640,
    "batch": 8, # Adjust
    "patience": 20, # Early stopping
    "device": "mps",
    "workers": 0,
    "project": str(OUTPUT_DIR.resolve()),
    "name": "fsoco_yolov8n",
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


def check_mps():
    if torch.backends.mps.is_available():
        print("MPS available")
        return "mps"
    else:
        print("MPS not available, using CPU")
        return "cpu"


def train():
    """Run training."""
    print("=" * 60)
    print("FSOCO Cone Detection Training")
    print("=" * 60)

    # Check device
    device = check_mps()
    CONFIG["device"] = device

    # Check if dataset exists
    if not DATA_CONFIG.exists():
        print(f"\nDataset config not found: {DATA_CONFIG}")
        print("Run convert_to_yolo.py first")
        return

    # Check if data exists
    data_dir = PROJECT_DIR / "data" / "yolo_format"
    train_dir = data_dir / "train" / "images"
    if not train_dir.exists() or len(list(train_dir.glob("*"))) == 0:
        print(f"\n✗ Training data not found in: {train_dir}")
        print("Run convert_to_yolo.py first")
        return

    print(f"\nDataset config: {DATA_CONFIG}")
    print(f"Training data: {train_dir}")

    # Load model
    last_ckpt = PROJECT_DIR / "models" / "fsoco_yolov8n" / "weights" / "last.pt"
    model_path = str(last_ckpt) if last_ckpt.exists() else CONFIG["model"]

    print(f"Loading model: {model_path}")
    model = YOLO(model_path)

    # Start training
    print("\nStarting training...")
    print(f"Epochs: {CONFIG['epochs']}")
    print(f"Image size: {CONFIG['imgsz']}")
    print(f"Batch size: {CONFIG['batch']}")
    print(f"Device: {CONFIG['device']}")
    print("-" * 60)

    results = model.train(
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

    # Print results
    print("\n" + "=" * 60)
    print("Training Complete!")
    print("=" * 60)

    best_model = OUTPUT_DIR / CONFIG["name"] / "weights" / "best.pt"
    print(f"\nBest model saved to: {best_model}")

    # Export to ONNX for Jetson deployment
    print("\nExporting to ONNX format")
    model_best = YOLO(str(best_model))
    model_best.export(format="onnx", imgsz=CONFIG["imgsz"], simplify=True)

    onnx_path = best_model.with_suffix(".onnx")
    print(f"ONNX model saved to: {onnx_path}")

    print("\n" + "=" * 60)
    print("Next steps:")
    print("1. Copy best.pt and best.onnx to your Jetson")
    print("2. On Jetson, convert ONNX to TensorRT for faster inference")
    print("=" * 60)


if __name__ == "__main__":
    train()