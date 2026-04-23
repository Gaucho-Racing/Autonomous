# AutonomousGR

AutonomousGR is a ROS 2 Humble workspace for cone-based autonomous navigation on an F1TENTH-style vehicle. The active runtime package is `cone_nav`: it takes ZED 2i stereo input, runs cone detection with a TensorRT YOLO engine, projects detections into 3D with depth, builds a centerline using Delaunay triangulation, smooths it with a cubic spline, and follows it with pure pursuit.

The repo also includes Python utilities for dataset conversion, model training, and live webcam testing.

## Repository Layout

```text
.
+-- cone_nav/                 # Main ROS 2 package
|   +-- cone_nav/             # Python ROS node package
|   +-- src/                  # C++ ROS 2 nodes
|   +-- config/               # Params, RViz, ORB-SLAM config
|   +-- launch/               # Real and sim launch files
|   +-- models/               # TensorRT engine location
|   +-- urdf/                 # Lightweight sim robot description
+-- models/                   # Local training outputs (.pt, .onnx), ignored by git
+-- data/                     # Local YOLO-format dataset, ignored by git
+-- Python Detection Code/    # Training, conversion, webcam tools
+-- URCA Documents/           # Project documents
+-- fsoco.yaml                # YOLO dataset config
+-- README.md
+-- License.md
```

## Runtime Architecture

The ROS runtime graph is flat and pragmatic:

```text
Image -> Detection2DArray -> PoseArray + PoseArray -> Path -> AckermannDriveStamped
```

Main nodes:

- `cone_detector_node`  
  C++ TensorRT/CUDA detector. Subscribes to the left image and publishes `/cone_detections` as `vision_msgs/Detection2DArray`.

- `cone_localizer_node`  
  C++ depth localizer. Synchronizes detections with registered depth, back-projects cone centers into 3D, transforms them to `base_link`, deduplicates nearby detections, and publishes `/cones/left`, `/cones/right`, and `/cones/all`.

- `path_planner_node.py`  
  Python planner. Builds a centerline from left/right cones using Delaunay triangulation, falls back to greedy pairing if needed, then applies cubic spline smoothing and publishes `/path` plus `/path/markers`.

- `pure_pursuit_node`  
  C++ controller. Selects a lookahead point on `/path`, computes steering and speed, and publishes `/drive` as `ackermann_msgs/AckermannDriveStamped`.

Optional localization / visual-inertial odometry:

- `orbslam3 stereo-inertial`  
  Launched from `real.launch.py` by default. Uses stereo images plus IMU data to provide visual-inertial SLAM / odometry. The launch wiring is present in this repo; production accuracy depends on a correct ORB-SLAM3 vocabulary and a real ZED 2i stereo-inertial calibration file.

## Topics

Primary runtime inputs:

```text
/zed2i/zed_node/left/image_rect_color
/zed2i/zed_node/right/image_rect_color
/zed2i/zed_node/depth/depth_registered
/zed2i/zed_node/left/camera_info
/zed2i/zed_node/imu/data
```

Primary runtime outputs:

```text
/cone_detections
/cones/left
/cones/right
/cones/all
/path
/path/markers
/drive
```

## Planning and Control

The planner is now built around:

- left/right cone extraction in `base_link`
- Delaunay triangulation over visible cones
- cross-color edge midpoint extraction for centerline candidates
- fallback greedy pairing when triangulation is sparse
- cubic Catmull-Rom spline smoothing
- pure pursuit on the smoothed path

Relevant planner parameters in [params.yaml](/Users/adi/Desktop/PycharmProjects/AutonomousGR/cone_nav/config/params.yaml):

```yaml
path_smoothing_window: 5
track_half_width: 0.75
spline_samples_per_segment: 6
delaunay_max_edge_length: 6.0
path_min_forward_x: -0.2
lookahead_distance: 1.5
lookahead_min: 0.8
lookahead_max: 3.0
speed_target: 1.5
speed_max: 3.0
wheelbase: 0.32
```

## Visual-Inertial Odometry / SLAM

The real launch file supports stereo-inertial ORB-SLAM3 integration. It is enabled by default in:

- [real.launch.py](/Users/adi/Desktop/PycharmProjects/AutonomousGR/cone_nav/launch/real.launch.py)

It remaps ORB-SLAM3 inputs to:

```text
/zed2i/zed_node/left/image_rect_color
/zed2i/zed_node/right/image_rect_color
/zed2i/zed_node/imu/data
```

The starter ORB-SLAM3 settings file is:

- [orbslam3_zed2i_stereo_inertial.yaml](/Users/adi/Desktop/PycharmProjects/AutonomousGR/cone_nav/config/orbslam3_zed2i_stereo_inertial.yaml)

Important caveat:

- The launch and config plumbing are in place.
- The provided ORB-SLAM3 YAML is only a starter file.
- Replace its camera intrinsics, stereo baseline, IMU noise terms, and body-camera transform with your actual ZED 2i calibration before trusting the output.

Vocabulary file is passed at launch and defaults to:

```text
/opt/orbslam3/Vocabulary/ORBvoc.txt
```

## Dependencies

Target platform:

- Ubuntu 22.04
- ROS 2 Humble
- Jetson Orin Nano
- CUDA
- TensorRT
- ZED 2i with ZED ROS 2 wrapper
- ORB-SLAM3 ROS 2 wrapper for stereo-inertial mode

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
orbslam3
```

Install normal ROS dependencies with your usual workflow, for example:

```bash
rosdep install --from-paths . --ignore-src -r -y
```

TensorRT, CUDA, and the ORB-SLAM3 installation are system-level dependencies and are not managed by this repo.

## TensorRT Model

The TensorRT engine is not committed to the repository. Put it here:

```text
cone_nav/models/cone_yolo.engine
```

The detector currently expects a YOLO-style output shaped like:

```text
[num_detections, 6]
```

or:

```text
[1, num_detections, 6]
```

Each row is:

```text
x, y, w, h, confidence, class
```

Class IDs:

```text
0 = blue cone
1 = yellow cone
2 = orange / big cone
```

You can override the engine path at launch:

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
- static `base_link -> zed2i_left_camera_frame` transform
- ORB-SLAM3 stereo-inertial node by default
- all four `cone_nav` nodes
- RViz with cone/path visualization

If needed, disable ORB-SLAM3:

```bash
ros2 launch cone_nav real.launch.py enable_orbslam:=false
```

If your ORB vocabulary file lives elsewhere:

```bash
ros2 launch cone_nav real.launch.py \
  orbslam_vocabulary_file:=/absolute/path/to/ORBvoc.txt
```

If you have a tuned ORB-SLAM3 camera/IMU settings file:

```bash
ros2 launch cone_nav real.launch.py \
  orbslam_settings_file:=/absolute/path/to/your_zed2i_stereo_inertial.yaml
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

Simulation defaults:

- ORB-SLAM3 is disabled by default in sim.
- The planner/controller/detector pipeline still runs.

To enable ORB-SLAM3 in sim:

```bash
ros2 launch cone_nav sim.launch.py sim_type:=f1tenth enable_orbslam:=true
```

Simulation topics configured in [params.yaml](/Users/adi/Desktop/PycharmProjects/AutonomousGR/cone_nav/config/params.yaml):

```yaml
sim_image_topic: "/sim/camera/image_raw"
sim_right_image_topic: "/sim/camera/right/image_raw"
sim_depth_topic: "/sim/camera/depth"
sim_camera_info_topic: "/sim/camera/camera_info"
sim_imu_topic: "/sim/imu"
```

Adjust those names to match the simulator bridge you are using.

## RViz

RViz config:

- [viz.rviz](/Users/adi/Desktop/PycharmProjects/AutonomousGR/cone_nav/config/viz.rviz)

Fixed frame:

```text
base_link
```

Displays include:

- left camera image
- TF tree
- cone markers
- path line
- path markers

## Training and Dataset Utilities

The Python tools in [Python Detection Code](/Users/adi/Desktop/PycharmProjects/AutonomousGR/Python%20Detection%20Code) are now repo-local rather than pointing at the old `Autonomous` workspace.

Files:

```text
Python Detection Code/ConvertToYOLO.py
Python Detection Code/ConeDetection.py
Python Detection Code/WebcamDetection.py
fsoco.yaml
```

### Dataset Conversion

Convert raw FSOCO annotations into YOLO format:

```bash
python3 "Python Detection Code/ConvertToYOLO.py"
```

Default raw dataset location:

```text
~/Downloads/fsoco_bounding_boxes_train
```

Override the raw dataset path for one run:

```bash
FSOCO_DIR=/absolute/path/to/fsoco python3 "Python Detection Code/ConvertToYOLO.py"
```

This writes:

```text
data/yolo_format/train/images
data/yolo_format/train/labels
data/yolo_format/val/images
data/yolo_format/val/labels
fsoco.yaml
```

The dataset is collapsed to the same 3 classes used by the ROS runtime:

```text
0 = blue_cone
1 = yellow_cone
2 = orange_cone
```

### Training

Train with YOLO26:

```bash
python3 "Python Detection Code/ConeDetection.py"
```

Current training defaults:

- base checkpoint: `yolo26n.pt`
- output run name: `fsoco_yolo26n`
- output directory: `models/fsoco_yolo26n`

Expected trained weights:

```text
models/fsoco_yolo26n/weights/best.pt
models/fsoco_yolo26n/weights/last.pt
```

If an existing checkpoint is present in that run directory, the training script resumes from it.

### Webcam Test

Quick live sanity check with a webcam:

```bash
python3 "Python Detection Code/WebcamDetection.py"
```

You can override the camera index or model path:

```bash
python3 "Python Detection Code/WebcamDetection.py" --camera 1
python3 "Python Detection Code/WebcamDetection.py" --model "/absolute/path/to/best.pt"
```

Press `q` to quit.

## Safety Behavior

The runtime code handles the main failure cases:

- invalid depth values are skipped
- detections outside the configured depth range are ignored
- duplicate cone detections are merged
- if the TensorRT engine fails to load, the detector logs a fatal error and exits
- if the depth topic stops publishing, the localizer warns every 5 seconds
- if a path has fewer than two waypoints, pure pursuit publishes zero speed
- if no new path arrives for 0.5 seconds, pure pursuit publishes zero speed
- steering is clamped to the configured physical limit

## Notes

- The runtime package uses only standard ROS 2 message types.
- No custom messages are defined.
- TensorRT engines should generally be built on the target Jetson because engines are tied to GPU architecture, TensorRT version, CUDA version, and JetPack version.
- The planning path is still reactive and camera-driven; ORB-SLAM3 is integrated as a visual-inertial odometry / SLAM source, but the current planner itself is not yet consuming a global map or odom frame for long-horizon planning.
