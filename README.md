# AutonomousGR

AutonomousGR is a ROS 2 Humble workspace for cone-based autonomous navigation on an F1TENTH-style vehicle. The active runtime package is `cone_nav`: it takes ZED 2i stereo input, runs cone detection with a TensorRT YOLO engine, projects detections into 3D with depth, builds a centerline, filters that route through a depth-based local obstacle grid, and follows the safe path with pure pursuit and independent command supervision.

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
+-- isaac_sim/                # Isaac Sim 6.0.1 scene and Brev launcher
+-- models/                   # Local training outputs (.pt, .onnx), ignored by git
+-- data/                     # Local YOLO-format dataset, ignored by git
+-- Python Detection Code/    # Training, conversion, webcam tools
+-- URCA Documents/           # Project documents
+-- fsoco.yaml                # YOLO dataset config
+-- README.md
+-- License.md
```

## Runtime Architecture

The ROS runtime graph keeps the public `/path` and Ackermann `/drive` contracts stable while separating nominal planning from obstacle avoidance and command safety:

```text
Image -> cone detections -> /path/nominal -> obstacle avoidance -> /path
Registered depth -> local obstacle grid -----------^          |
Registered depth -> drive safety <---- /drive_candidate <---- pure pursuit
                                      drive safety -> /drive
```

Main nodes:

- `cone_detector_node`  
  C++ TensorRT/CUDA detector. Subscribes to the left image and publishes `/cone_detections` as `vision_msgs/Detection2DArray`.

- `cone_localizer_node`  
  C++ depth localizer. Synchronizes detections with registered depth, back-projects cone centers into 3D, transforms them to `base_link`, deduplicates nearby detections, and publishes `/cones/left`, `/cones/right`, and `/cones/all`.

- `path_planner_node.py`  
  Python planner. Builds and smooths the cone centerline. Launch files remap its output to `/path/nominal` and `/path/nominal_markers`.

- `depth_obstacle_node`
  C++ registered-depth processor. Back-projects valid pixels into `base_link`, filters the ground/hood/range, inflates vehicle clearance, and publishes `/obstacles/local_grid` plus `/obstacles/points`.

- `obstacle_avoidance_node.py`
  Samples laterally shifted, curvature-continuous local paths, rejects grid collisions and cone-boundary crossings, and publishes the selected safe route on `/path`. No feasible route produces an empty path.

- `pure_pursuit_node`  
  C++ controller. Selects a lookahead point on `/path` and publishes `/drive_candidate` through a launch remap.

- `drive_safety_node`
  The sole `/drive` publisher. It checks the commanded Ackermann arc against the current obstacle grid, applies the configured braking envelope, and stops on stale depth or commands.

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
/obstacles/local_grid
/obstacles/points
/path/nominal
/path/nominal_markers
/path
/drive_candidate
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
- local lateral candidate generation inside the cone corridor
- inflated-depth collision checking
- independent stopping-distance enforcement before `/drive`

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
obstacle_inflation_radius: 0.22
avoidance_max_lateral_offset: 0.55
safety_grid_timeout_sec: 0.20
safety_braking_deceleration: 2.0
safety_stop_margin: 0.30
```

Obstacle avoidance is enabled by default for `real.launch.py` and
`isaac.launch.py`. Use `avoidance_shadow_mode:=true` to compute safety state
while continuing to publish the nominal path, or `avoidance_enabled:=false` to
bypass avoidance and grid command checks during a controlled regression test.

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

Isaac Sim target:

- NVIDIA Isaac Sim 6.0.1
- Ubuntu 22.04 with ROS 2 Humble
- NVIDIA RTX GPU; the supplied Brev launcher targets an L40S-class instance
- Fast DDS with the same `ROS_DOMAIN_ID` in Isaac Sim and the ROS workspace
- TensorRT 8 or 10; the detector contains compatibility paths for both APIs

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

ZED wrapper topic names vary across releases. The repository defaults preserve
the original `image_rect_color` contract, but all real sensor topics are launch
arguments. For a wrapper using the newer rectified color naming, launch with:

```bash
ros2 launch cone_nav real.launch.py \
  left_image_topic:=/zed2i/zed_node/left/color/rect/image \
  right_image_topic:=/zed2i/zed_node/right/color/rect/image \
  camera_info_topic:=/zed2i/zed_node/left/color/rect/camera_info
```

Confirm the exact names with `ros2 topic list` for the installed wrapper. The
depth argument continues to default to
`/zed2i/zed_node/depth/depth_registered`.

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

## Run In NVIDIA Isaac Sim

The repository contains a dedicated Isaac Sim 6.0.1 integration. It is separate
from the legacy `f1tenth` and `fsae` launch paths so `/drive` remains an
`ackermann_msgs/AckermannDriveStamped` topic end to end.

Included files:

- `isaac_sim/autonomousgr_scene.py`: creates a Leatherback Ackermann vehicle,
  starter cone corridor, selectable collision-obstacle scenarios, a ZED 2i-like
  0.12 m stereo pair, registered-left depth, `/clock`, camera-info publishers,
  and a `/drive` subscriber/controller graph.
- `isaac_sim/run_brev.sh`: launches the pinned Isaac Sim 6.0.1 container on a
  Linux NVIDIA GPU host and optionally enables WebRTC.
- `isaac_sim/check_topics.sh`: checks the required ROS topic and TF contract.
- `cone_nav/launch/isaac.launch.py`: launches only the autonomy stack, TF, and
  optional RViz/ORB-SLAM; it does not start a conflicting simulator bridge.
- `cone_nav/config/isaac_params.yaml`: low-speed, simulation-time configuration.
- `cone_nav/config/viz_isaac.rviz`: RViz view using the simulated camera topics.

### Brev / cloud setup

Create a Brev VM-mode instance with one L40S GPU. Expose WebRTC ports `49100`
and `47998` only to your current public IP. Clone this repository on the VM.

Start Isaac Sim from the repository root:

```bash
export PUBLIC_IP=<brev-instance-public-ip>
export ROS_DOMAIN_ID=0
ENABLE_LIVESTREAM=1 ./isaac_sim/run_brev.sh
```

For a non-interactive/headless validation run:

```bash
export ROS_DOMAIN_ID=0
ENABLE_LIVESTREAM=0 ./isaac_sim/run_brev.sh
```

Choose a collision scenario with `--obstacle-scenario`. Available values are
`clear`, `center`, `right`, `narrow`, and `blocked`:

```bash
ENABLE_LIVESTREAM=0 ./isaac_sim/run_brev.sh --obstacle-scenario blocked
```

The launcher uses `nvcr.io/nvidia/isaac-sim:6.0.1`, host networking, Isaac
Sim's internal ROS 2 Humble libraries, and the host `ROS_DOMAIN_ID`. Override the
image with `ISAAC_IMAGE` only when deliberately testing another release.

### Build and start the ROS stack

In another terminal on the same GPU VM, install/source ROS 2 Humble and build:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --packages-select cone_nav --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

Place a TensorRT engine built for the VM's GPU and TensorRT version at
`cone_nav/models/cone_yolo.engine`, or pass an absolute path. TensorRT engines
are not portable between arbitrary GPU/TensorRT combinations.

Start the autonomy stack **disarmed**:

```bash
ros2 launch cone_nav isaac.launch.py \
  enable_rviz:=false \
  engine_path:=/absolute/path/to/cone_yolo.engine
```

Use `enable_rviz:=true` only in a terminal with a working display. Isaac Sim and
all navigation nodes use `/clock` and `use_sim_time:=true`.

Verify the bridge before allowing motion:

```bash
./isaac_sim/check_topics.sh
ros2 topic hz /sim/camera/image_raw
ros2 topic hz /sim/camera/depth
ros2 topic hz /obstacles/local_grid
ros2 topic info -v /drive_candidate
ros2 topic info -v /drive
```

Required navigation inputs are:

```text
/sim/camera/image_raw             sensor_msgs/Image (RGB)
/sim/camera/right/image_raw       sensor_msgs/Image (RGB)
/sim/camera/depth                 sensor_msgs/Image (32FC1 metres)
/sim/camera/camera_info           sensor_msgs/CameraInfo
/sim/camera/right/camera_info     sensor_msgs/CameraInfo
/clock                            rosgraph_msgs/Clock
```

Only after image, depth, intrinsics, TF, detections, and stop behavior have been
checked should drive output be armed:

```bash
ros2 launch cone_nav isaac.launch.py \
  enable_rviz:=false \
  drive_enabled:=true \
  engine_path:=/absolute/path/to/cone_yolo.engine
```

The Isaac defaults limit target speed to `0.5 m/s` and maximum speed to
`1.0 m/s`. The launch publishes the correct ROS optical transform from
`base_link` to `zed2i_left_camera_optical_frame`.

### Stereo and ORB-SLAM scope

Isaac supports two selectable depth modes. The default renderer depth is a
deterministic oracle for functional tests:

```bash
ros2 launch cone_nav isaac.launch.py depth_mode:=ground_truth
```

The stereo qualification mode runs OpenCV SGBM over the synchronized rectified
left/right images, derives the baseline from the right `CameraInfo.P` matrix
(falling back to `0.12 m`), and publishes registered `32FC1` depth:

```bash
ros2 launch cone_nav isaac.launch.py depth_mode:=stereo
```

Use ground-truth mode for repeatable collision tests and stereo mode to expose
calibration, texture, correspondence, and dropout problems. On the vehicle,
the ZED SDK's `/depth/depth_registered` output is already stereo-derived and is
the production source.

ORB-SLAM3 remains disabled by default. The starter scene does not publish a
calibrated `/sim/imu`; do not set `enable_orbslam:=true` until an Isaac IMU
publisher and a matching stereo-inertial calibration are added and validated.
This limitation does not affect the detector, depth localizer, centerline
planner, or Ackermann controller.

### Isaac contract tests

Run source-level tests on any development machine:

```bash
python3 -m unittest cone_nav/test/test_isaac_contract.py \
  cone_nav/test/test_avoidance_algorithms.py
bash -n isaac_sim/run_brev.sh isaac_sim/check_topics.sh
```

On the RTX host, also run a bounded Isaac smoke test:

```bash
ENABLE_LIVESTREAM=0 ./isaac_sim/run_brev.sh \
  --test-steps 120 --obstacle-scenario center
```

Full compatibility is established only after this runtime smoke test and the
ROS topic checks pass on the target RTX host; macOS cannot execute that portion.

## Legacy Simulation Bridges

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
- Obstacle avoidance is bypassed because the legacy bridges do not guarantee
  registered stereo depth. Enable it only after the `/sim/camera/*` contract is
  verified.
- The FSAE path uses `ackermann_to_twist_node` to convert the final safe
  Ackermann command to a correctly typed `/cmd_vel`; it is no longer a topic-only
  remap between incompatible message types.

To enable ORB-SLAM3 in sim:

```bash
ros2 launch cone_nav sim.launch.py sim_type:=f1tenth enable_orbslam:=true
```

To enable depth avoidance on a compatible legacy bridge:

```bash
ros2 launch cone_nav sim.launch.py sim_type:=f1tenth avoidance_enabled:=true
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
- nominal and safe path lines
- nominal path markers
- inflated obstacle occupancy grid
- optional filtered obstacle point cloud

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
- a last valid path is held for at most 0.25 seconds; stale perception then
  publishes an empty path instead of refreshing old commands indefinitely
- depth obstacles are inflated by the vehicle clearance radius before planning
- a stale obstacle grid or stale `/drive_candidate` produces an immediate stop
- a blocked cone corridor publishes an empty `/path`
- `drive_safety_node` is the only final `/drive` publisher and limits speed with
  `v * latency + v^2 / (2 * deceleration) + margin`
- Isaac drive output is disarmed by default and must be explicitly enabled
- steering is clamped to the configured physical limit

## Isaac Sim Integration Changes

The Isaac integration introduced the following repository changes:

- Added a version-pinned Isaac Sim/Brev launch workflow and generated starter scene.
- Added native ROS 2 camera, stereo camera-info, clock, and Ackermann OmniGraphs.
- Added an Isaac-only ROS launch so `/drive` is never remapped to `/cmd_vel`.
- Corrected localization to use a ROS optical camera frame and added the matching TF.
- Enabled simulation time consistently across navigation, TF, ORB-SLAM, and RViz.
- Added a default-disarmed controller and configurable command timeout.
- Fixed stale-path behavior so losing cones results in a stop command.
- Added x86_64 TensorRT discovery and TensorRT 8/10 detector API compatibility.
- Added Isaac-specific RViz configuration and source-level contract tests.
- Added registered-depth obstacle extraction, local path avoidance, and a
  fail-closed Ackermann command supervisor.
- Added selectable Isaac ground-truth and stereo-SGBM depth modes.
- Added collision-enabled `clear`, `center`, `right`, `narrow`, and `blocked`
  Isaac scenarios and a right optical-frame baseline contract.

## Notes

- The runtime package uses only standard ROS 2 message types.
- No custom messages are defined.
- TensorRT engines should generally be built on the target Jetson because engines are tied to GPU architecture, TensorRT version, CUDA version, and JetPack version.
- The planning path is still reactive and camera-driven; ORB-SLAM3 is integrated as a visual-inertial odometry / SLAM source, but the current planner itself is not yet consuming a global map or odom frame for long-horizon planning.
