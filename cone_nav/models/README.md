# TensorRT Engine

Place the YOLO-format TensorRT engine at:

```text
models/cone_yolo.engine
```

For the included YOLO26 checkpoint, export on the target NVIDIA GPU with
Ultralytics `nms=False`. This produces FP32 rows in the supported `xyxy`
layout and is selected by `config/isaac_params.yaml`.

```bash
yolo export \
  model=models/fsoco_yolo26n/weights/best.pt \
  format=engine imgsz=640 batch=1 dynamic=False \
  quantize=32 nms=False device=0 workspace=4

cp models/fsoco_yolo26n/weights/best.engine \
  cone_nav/models/cone_yolo.engine
```

The detector parses `[batch, num_detections, 6]` or `[num_detections, 6]`.
Set `trt_bbox_format` to match each row:

```text
xyxy:   x1, y1, x2, y2, confidence, class
cxcywh: center_x, center_y, width, height, confidence, class
```

Class IDs are:

```text
0 = blue cone
1 = yellow cone
2 = orange / big cone
```
