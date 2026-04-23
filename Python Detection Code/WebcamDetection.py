"""
Run live cone detection from a webcam using the trained YOLO model in this repo.
"""

import argparse
from pathlib import Path

import cv2
from ultralytics import YOLO


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent

DEFAULT_MODEL_CANDIDATES = [
    REPO_ROOT / "models" / "fsoco_yolo26n" / "weights" / "best.pt",
    REPO_ROOT / "models" / "fsoco_yolov8n" / "weights" / "best.pt",
    REPO_ROOT / "models" / "fsoco_yolov8n" / "weights" / "last.pt",
]

CLASS_COLORS = {
    0: (255, 80, 40),   # blue cone
    1: (0, 220, 255),   # yellow cone
    2: (0, 140, 255),   # orange cone
}


def resolve_model_path(cli_model_path):
    if cli_model_path:
        model_path = Path(cli_model_path).expanduser().resolve()
        if not model_path.exists():
            raise FileNotFoundError(f"Model not found: {model_path}")
        return model_path

    for candidate in DEFAULT_MODEL_CANDIDATES:
        if candidate.exists():
            return candidate

    searched = "\n".join(str(path) for path in DEFAULT_MODEL_CANDIDATES)
    raise FileNotFoundError(
        "No trained model was found. Looked for:\n"
        f"{searched}\n"
        "Pass --model /absolute/path/to/best.pt to override."
    )


def draw_detections(frame, result, conf_threshold):
    boxes = result.boxes
    if boxes is None:
        return frame

    for box in boxes:
        confidence = float(box.conf[0])
        if confidence < conf_threshold:
            continue

        class_id = int(box.cls[0])
        label = result.names.get(class_id, str(class_id))
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        color = CLASS_COLORS.get(class_id, (0, 255, 0))

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        caption = f"{label} {confidence:.2f}"
        cv2.putText(
            frame,
            caption,
            (x1, max(24, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
            cv2.LINE_AA,
        )

    return frame


def main():
    parser = argparse.ArgumentParser(description="Live webcam cone detection")
    parser.add_argument("--camera", type=int, default=0, help="Webcam index, default 0")
    parser.add_argument("--model", type=str, default="", help="Override model path")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold")
    parser.add_argument("--imgsz", type=int, default=640, help="Inference image size")
    args = parser.parse_args()

    model_path = resolve_model_path(args.model)
    print(f"Using model: {model_path}")

    model = YOLO(str(model_path))
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open webcam index {args.camera}")

    print("Press q to quit.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame from webcam.")
                break

            results = model.predict(frame, conf=args.conf, imgsz=args.imgsz, verbose=False)
            annotated = draw_detections(frame.copy(), results[0], args.conf)

            cv2.imshow("Cone Detection", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
