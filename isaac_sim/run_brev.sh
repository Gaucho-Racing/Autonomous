#!/usr/bin/env bash
set -euo pipefail

ISAAC_IMAGE="${ISAAC_IMAGE:-nvcr.io/nvidia/isaac-sim:6.0.1}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
PRIVACY_CONSENT="${PRIVACY_CONSENT:-N}"
ENABLE_LIVESTREAM="${ENABLE_LIVESTREAM:-1}"
PUBLIC_IP="${PUBLIC_IP:-}"
EXTRA_SCENE_ARGS=("$@")

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "Isaac Sim containers require a Linux host with an NVIDIA GPU." >&2
  exit 1
fi

command -v docker >/dev/null || { echo "docker is required" >&2; exit 1; }
command -v nvidia-smi >/dev/null || { echo "nvidia-smi is required" >&2; exit 1; }
nvidia-smi >/dev/null

SCENE_ARGS=(--headless --ros-domain-id "${ROS_DOMAIN_ID}")
KIT_ARGS=()
if [[ "${ENABLE_LIVESTREAM}" == "1" ]]; then
  if [[ -z "${PUBLIC_IP}" ]]; then
    echo "Set PUBLIC_IP to the Brev instance public IP when ENABLE_LIVESTREAM=1." >&2
    exit 1
  fi
  SCENE_ARGS+=(--livestream)
  KIT_ARGS+=(
    "--/app/livestream/publicEndpointAddress=${PUBLIC_IP}"
    "--/app/livestream/port=49100"
  )
fi

docker pull "${ISAAC_IMAGE}"
docker run --rm --name autonomousgr-isaac-sim \
  --gpus all \
  --network=host \
  --ipc=host \
  --entrypoint bash \
  -e ACCEPT_EULA=Y \
  -e "PRIVACY_CONSENT=${PRIVACY_CONSENT}" \
  -e ROS_DISTRO=humble \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  -v "${REPOSITORY_ROOT}:/workspace/AutonomousGR:ro" \
  "${ISAAC_IMAGE}" -lc '
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:/isaac-sim/exts/isaacsim.ros2.core/humble/lib"
    exec /isaac-sim/python.sh /workspace/AutonomousGR/isaac_sim/autonomousgr_scene.py "$@"
  ' bash "${SCENE_ARGS[@]}" "${EXTRA_SCENE_ARGS[@]}" "${KIT_ARGS[@]}"
