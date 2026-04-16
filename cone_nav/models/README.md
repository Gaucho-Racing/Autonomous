# TensorRT Engine

Place the YOLO-format TensorRT engine at:

```text
models/cone_yolo.engine
```

The detector parses the simple output layout `[batch, num_detections, 6]` or
`[num_detections, 6]`, where each row is:

```text
x, y, w, h, confidence, class
```

Class IDs are:

```text
0 = blue cone
1 = yellow cone
2 = orange / big cone
```
