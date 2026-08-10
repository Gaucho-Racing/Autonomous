#!/usr/bin/env bash
set -euo pipefail

REQUIRED_TOPICS=(
  /clock
  /sim/camera/image_raw
  /sim/camera/right/image_raw
  /sim/camera/depth
  /sim/camera/camera_info
)

missing=0
topic_list="$(ros2 topic list)"
for topic in "${REQUIRED_TOPICS[@]}"; do
  if ! grep -Fxq "${topic}" <<<"${topic_list}"; then
    echo "MISSING ${topic}" >&2
    missing=1
  else
    echo "OK ${topic}"
  fi
done

if [[ "${missing}" -ne 0 ]]; then
  exit 1
fi

timeout 10 ros2 topic echo --once /sim/camera/camera_info
test "$(ros2 topic type /sim/camera/depth)" = "sensor_msgs/msg/Image"
test "$(ros2 topic type /sim/camera/camera_info)" = "sensor_msgs/msg/CameraInfo"
timeout 5 ros2 run tf2_ros tf2_echo base_link zed2i_left_camera_optical_frame ||
  [[ "$?" -eq 124 ]]
timeout 5 ros2 run tf2_ros tf2_echo zed2i_left_camera_optical_frame zed2i_right_camera_optical_frame ||
  [[ "$?" -eq 124 ]]
