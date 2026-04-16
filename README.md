# AutonomousGR

AutonomousGR is a ROS 2 Humble workspace for reactive cone navigation on an F1TENTH-style vehicle. The active runtime package is `cone_nav`: it takes ZED 2i stereo camera input, runs cone detection with a TensorRT YOLO engine, projects detections into 3D using depth, plans a centerline path, and publishes pure pursuit drive commands.

The system is designed for a Jetson Orin Nano and can also run in simulation by swapping the camera/depth and drive topics through launch parameters.

## Repository Layout

```text
.
+-- cone_nav/                 # Main ROS 2 package
|   +-- cone_nav/             # Python node package
|   +-- src/                  # C++ ROS 2 nodes
|   +-- config/               # Shared params and RViz config
|   +-- launch/               # Real and sim launch files
|   +-- models/               # TensorRT engine location
|   +-- urdf/                 # Lightweight sim robot description
+-- Python Detection Code/    # Legacy/training utilities for cone detection
+-- URCA Documents/           # Project documents
+-- README.md
+-- License.md
```

## Runtime Architecture

The ROS graph is intentionally flat:

```text
Image -> Detection2DArray -> PoseArray + PoseArray -> Path -> AckermannDriveStamped
```

Nodes:

- `cone_detector_node`  
  C++ TensorRT/CUDA detector. Subscribes to the ZED left image and publishes `/cone_detections` as `vision_msgs/Detection2DArray`.

- `cone_localizer_node`  
  C++ depth localizer. Synchronizes detections with the ZED registered depth image, projects cone centers into 3D, transforms them to `base_link`, deduplicates nearby detections, and publishes `/cones/left`, `/cones/right`, and `/cones/all`.

- `path_planner_node.py`  
  Python centerline planner. Pairs left/right cones, computes midpoints, smooths the path, and publishes `/path` plus `/path/markers`.

- `pure_pursuit_node`  
  C++ controller. Selects a lookahead point on `/path`, computes steering and speed, and publishes `/drive` as `ackermann_msgs/AckermannDriveStamped`.

## Topics

Main inputs:

```text
/zed2i/zed_node/left/image_rect_color
/zed2i/zed_node/depth/depth_registered
/zed2i/zed_node/left/camera_info
```

Main outputs:

```text
/cone_detections
/cones/left
/cones/right
/cones/all
/path
/path/markers
/drive
```

## Dependencies

Target platform:

- Ubuntu 22.04
- ROS 2 Humble
- Jetson Orin Nano
- CUDA
- TensorRT
- ZED 2i and ZED ROS 2 wrapper

ROS package dependencies include:

```text
rclcpp
rclpy
sensor_msgs
vision_msgs
geometry_msgs
nav_msgs
ackermann_msgs
visualization_msgs
cv_bridge
image_transport
tf2
tf2_ros
tf2_geometry_msgs
message_filters
zed_wrapper
rviz2
```

Install the ROS dependencies with your normal ROS 2 dependency workflow, for example:

```bash
rosdep install --from-paths . --ignore-src -r -y
```

TensorRT and CUDA are expected to come from the Jetson/JetPack installation.

## TensorRT Model

The TensorRT engine is not committed to the repository. Put it here:

```text
cone_nav/models/cone_yolo.engine
```

The detector currently expects a YOLO-style TensorRT output shaped like:

```text
[num_detections, 6]
```

or:

```text
[1, num_detections, 6]
```

Each detection row must be:

```text
x, y, w, h, confidence, class
```

Class IDs:

```text
0 = blue cone
1 = yellow cone
2 = orange / big cone
```

You can also pass an engine path at launch:

```bash
ros2 launch cone_nav real.launch.py engine_path:=/absolute/path/to/cone_yolo.engine
```

## Build

From the repository root:

```bash
colcon build --packages-select cone_nav --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

If TensorRT is not found, confirm that `NvInfer.h`, `libnvinfer`, `libnvinfer_plugin`, and CUDA are installed and visible to CMake.

## Run On Vehicle

Use the real launch file:

```bash
source install/setup.bash
ros2 launch cone_nav real.launch.py
```

This launches:

- ZED ROS 2 wrapper for `zed2i`
- Static `base_link -> zed2i_left_camera_frame` transform
- All four `cone_nav` nodes
- RViz with cone/path visualization

The shared parameter file is:

```text
cone_nav/config/params.yaml
```

Important parameters include:

```yaml
use_sim: false
camera_frame: "zed2i_left_camera_frame"
base_frame: "base_link"
detection_confidence_threshold: 0.5
nms_iou_threshold: 0.45
max_detection_depth: 15.0
min_detection_depth: 0.3
cone_cluster_radius: 0.4
lookahead_distance: 1.5
speed_target: 1.5
speed_max: 3.0
wheelbase: 0.32
publish_rate_hz: 30.0
```

## Run In Simulation

Use:

```bash
source install/setup.bash
ros2 launch cone_nav sim.launch.py sim_type:=f1tenth
```

or:

```bash
ros2 launch cone_nav sim.launch.py sim_type:=fsae
```

Simulation topics are configured in `cone_nav/config/params.yaml`:

```yaml
sim_image_topic: "/sim/camera/image_raw"
sim_depth_topic: "/sim/camera/depth"
sim_camera_info_topic: "/sim/camera/camera_info"
```

Adjust those names to match the simulator bridge you are using.

## RViz

RViz config:

```text
cone_nav/config/viz.rviz
```

Fixed frame:

```text
base_link
```

Displays include:

- Left camera image
- TF tree
- Cone markers
- Centerline path
- Path markers

## Safety Behavior

The runtime code handles the main failure cases:

- Invalid depth values are skipped.
- Detections outside the configured depth range are ignored.
- Duplicate cone detections are merged.
- If the TensorRT engine fails to load, the detector logs a fatal error and exits.
- If the depth topic stops publishing, the localizer warns every 5 seconds.
- If a path has fewer than two waypoints, pure pursuit publishes zero speed.
- If no new path arrives for 0.5 seconds, pure pursuit publishes zero speed.
- Steering is clamped to the configured physical limit.

## Legacy Detection Utilities

`Python Detection Code/` contains earlier Python scripts for model training/conversion work:

```text
Python Detection Code/ConeDetection.py
Python Detection Code/ConvertToYOLO.py
```

These are not part of the ROS runtime path. Use them as reference or training utilities only.

## Notes

- The runtime package uses only standard ROS 2 message types.
- No custom messages are defined.
- The planner is reactive and frame-local: paths are published in `base_link`, with no map, odometry, or SLAM dependency.
- TensorRT engines should generally be built on the target Jetson because engines are tied to GPU architecture, TensorRT version, CUDA version, and JetPack version.
