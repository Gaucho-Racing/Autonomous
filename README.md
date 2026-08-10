# AutonomousGR

AutonomousGR is a ROS 2 Humble autonomy stack for an F1TENTH-style vehicle using a ZED 2i stereo camera. It detects track cones, estimates their 3D positions, builds a centerline, avoids depth obstacles, follows the safe path with pure pursuit, and supervises the final Ackermann command.

The stack targets:

- Jetson Orin Nano, CUDA, TensorRT, and ZED 2i on the vehicle
- NVIDIA Isaac Sim 6.0.1 on Ubuntu 22.04 with an RTX GPU
- ROS 2 Humble and `ackermann_msgs/AckermannDriveStamped`

## System overview

```text
Left image -> TensorRT cone detector -> depth localization -> left/right cones
                                                            |
                                                            v
                                                centerline /path/nominal
                                                            |
Registered depth -> inflated local obstacle grid -----------+
                                                            v
                                                  obstacle avoidance -> /path
                                                                        |
                                                                        v
                                                      pure pursuit -> /drive_candidate
                                                                        |
Obstacle grid + freshness + braking envelope ---------------------------+
                                                                        v
                                                         drive safety -> /drive
```

Core nodes:

- `cone_detector_node`: TensorRT/CUDA cone detection.
- `cone_localizer_node`: registered-depth projection into `base_link`.
- `path_planner_node.py`: Delaunay/greedy cone pairing and spline-smoothed nominal centerline.
- `depth_obstacle_node`: ground/hood filtering, point projection, grid inflation, and `/obstacles/local_grid`.
- `obstacle_avoidance_node.py`: laterally shifted paths constrained by obstacles and cone boundaries.
- `pure_pursuit_node`: steering and speed generation on `/drive_candidate`.
- `drive_safety_node`: sole `/drive` publisher; stops or limits speed using obstacle clearance and data freshness.
- `stereo_depth_node`: optional OpenCV SGBM depth generation for Isaac stereo qualification.

The externally important contracts remain:

```text
/path   nav_msgs/Path in base_link
/drive  ackermann_msgs/AckermannDriveStamped
```

## Repository layout

```text
cone_nav/                  ROS 2 package, nodes, launch files, config, RViz, URDF
isaac_sim/                 Isaac scene, container launcher, and topic checks
Python Detection Code/     dataset conversion, training, and webcam utilities
fsoco.yaml                 YOLO dataset configuration
URCA Documents/            project documents
```

## Requirements and model

Install normal ROS dependencies, then provide system-level CUDA, TensorRT, the ZED ROS 2 wrapper, and optional ORB-SLAM3:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
```

Place the TensorRT engine at:

```text
cone_nav/models/cone_yolo.engine
```

It must expose FP32 input/output and return `[N,6]` or `[1,N,6]` rows containing `x, y, width, height, confidence, class`. Classes are `0=blue`, `1=yellow`, and `2=orange/big cone`. TensorRT engines are GPU- and TensorRT-version-specific; build the production engine on the target platform.

## Build

```bash
colcon build --packages-select cone_nav --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

## Run on the vehicle

```bash
ros2 launch cone_nav real.launch.py
```

This starts the ZED wrapper, cone pipeline, obstacle avoidance, safety supervisor, RViz, and ORB-SLAM3. Disable ORB-SLAM when it is not installed or calibrated:

```bash
ros2 launch cone_nav real.launch.py enable_orbslam:=false
```

The supplied ORB-SLAM YAML is only a starting point. Replace its intrinsics, stereo baseline, IMU noise values, and body-camera transform before trusting its output. The reactive planner itself does not require or consume ORB-SLAM odometry.

ZED wrapper topic names vary by release. Defaults use the repository's original names, while launch arguments support newer naming:

```bash
ros2 launch cone_nav real.launch.py \
  left_image_topic:=/zed2i/zed_node/left/color/rect/image \
  right_image_topic:=/zed2i/zed_node/right/color/rect/image \
  camera_info_topic:=/zed2i/zed_node/left/color/rect/camera_info
```

Confirm the installed wrapper's topics with `ros2 topic list`. Registered depth defaults to `/zed2i/zed_node/depth/depth_registered`.

Useful modes:

```bash
# Compute avoidance but publish the nominal path; drive safety remains active.
ros2 launch cone_nav real.launch.py avoidance_shadow_mode:=true

# Controlled regression bypass of avoidance and grid command checks.
ros2 launch cone_nav real.launch.py avoidance_enabled:=false
```

## Run in Isaac Sim

The Isaac integration provides a Leatherback Ackermann vehicle, ZED 2i-like 0.12 m stereo pair, RGB, camera info, registered depth, `/clock`, cone corridor, collision obstacles, and native `/drive` control.

### 1. Start Isaac Sim

On an Ubuntu RTX host or Brev VM:

```bash
export ROS_DOMAIN_ID=0
export PUBLIC_IP=<host-public-ip>  # Only required for livestreaming.
ENABLE_LIVESTREAM=1 ./isaac_sim/run_brev.sh
```

For headless operation:

```bash
ENABLE_LIVESTREAM=0 ./isaac_sim/run_brev.sh --obstacle-scenario center
```

Available scenarios are `clear`, `center`, `right`, `narrow`, and `blocked`.

### 2. Start the autonomy stack disarmed

In another terminal on the same host:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cone_nav isaac.launch.py \
  enable_rviz:=false \
  engine_path:=/absolute/path/to/cone_yolo.engine
```

Isaac and all autonomy nodes must share `ROS_DOMAIN_ID`, Fast DDS, and simulation time.

### 3. Select the depth source

Ground-truth renderer depth is the deterministic default for functional and collision tests:

```bash
ros2 launch cone_nav isaac.launch.py depth_mode:=ground_truth
```

Stereo mode runs SGBM on the synchronized left/right images and publishes registered `32FC1` depth:

```bash
ros2 launch cone_nav isaac.launch.py depth_mode:=stereo
```

Use ground truth to validate navigation and stereo mode to expose calibration, texture, correspondence, and dropout problems. The vehicle uses the ZED SDK's stereo-derived registered depth.

### 4. Verify, then arm

```bash
./isaac_sim/check_topics.sh
ros2 topic hz /sim/camera/image_raw
ros2 topic hz /sim/camera/depth
ros2 topic hz /obstacles/local_grid
ros2 topic info -v /drive_candidate
ros2 topic info -v /drive
```

Only after RGB, depth, camera intrinsics, TF, detections, obstacle grid, empty-path stopping, and single `/drive` ownership are confirmed:

```bash
ros2 launch cone_nav isaac.launch.py \
  drive_enabled:=true \
  enable_rviz:=false \
  engine_path:=/absolute/path/to/cone_yolo.engine
```

Isaac defaults to a `0.5 m/s` target and `1.0 m/s` maximum. ORB-SLAM3 is disabled because the starter scene does not publish a calibrated `/sim/imu`.

## Safety and tuning

The safety path is intentionally separate from planning:

- Invalid or out-of-range depth is discarded; ground and the vehicle hood are filtered.
- Obstacles are inflated before candidate-path collision checks.
- Candidate paths must remain inside the cone corridor.
- No feasible path produces an empty `/path`, which stops pure pursuit.
- Stale paths, grids, depth, or `/drive_candidate` messages produce zero speed.
- `drive_safety_node` is the only final `/drive` publisher.
- Speed is limited using `v * latency + v^2 / (2 * deceleration) + margin`.
- Isaac starts disarmed.

Primary tuning lives in [`params.yaml`](cone_nav/config/params.yaml) and [`isaac_params.yaml`](cone_nav/config/isaac_params.yaml):

```yaml
track_half_width: 0.75
lookahead_distance: 1.5
speed_target: 1.5
wheelbase: 0.32
obstacle_inflation_radius: 0.22
avoidance_max_lateral_offset: 0.55
safety_grid_timeout_sec: 0.20
safety_braking_deceleration: 2.0
safety_stop_margin: 0.30
```

Measure real braking performance before increasing vehicle speed. The defaults are starting values, not a validated physical safety case.

## Legacy simulators

```bash
ros2 launch cone_nav sim.launch.py sim_type:=f1tenth
ros2 launch cone_nav sim.launch.py sim_type:=fsae
```

Legacy simulation bypasses obstacle avoidance by default because these bridges do not guarantee registered stereo depth. Enable it only after verifying `/sim/camera/image_raw`, right image, depth, camera info, timestamps, frames, and TF:

```bash
ros2 launch cone_nav sim.launch.py sim_type:=f1tenth avoidance_enabled:=true
```

The FSAE path uses `ackermann_to_twist_node` for a typed `/drive` to `/cmd_vel` conversion.

## Validation

Source-level tests available on any machine:

```bash
python3 -m unittest discover -s cone_nav/test -p 'test_*.py' -v
bash -n isaac_sim/run_brev.sh isaac_sim/check_topics.sh
```

Bounded Isaac smoke test on the RTX host:

```bash
ENABLE_LIVESTREAM=0 ./isaac_sim/run_brev.sh \
  --test-steps 120 --obstacle-scenario center
```

Full compatibility requires a successful target `colcon` build, Isaac runtime test, ROS topic/type/rate checks, and disarmed stop-behavior validation. macOS cannot perform the ROS/Isaac runtime portion.

## Training utilities

The repository retains lightweight dataset and model tools:

```bash
python3 "Python Detection Code/ConvertToYOLO.py"   # FSOCO -> three-class YOLO data
python3 "Python Detection Code/ConeDetection.py"  # train YOLO26
python3 "Python Detection Code/WebcamDetection.py" # quick camera/model check
```

Training outputs and datasets are intentionally ignored by Git. Only the TensorRT runtime contract and class mapping are required by `cone_nav`.

## Known limitations

- The planner is reactive and camera-relative; it does not perform global-map planning.
- ORB-SLAM3 is optional and requires real stereo/IMU calibration.
- Isaac ground-truth depth validates navigation but not stereo correspondence quality; use `depth_mode:=stereo` for that.
- A TensorRT engine must match the target GPU, CUDA, and TensorRT versions.
- The stack uses standard ROS 2 messages and defines no custom interfaces.
