#!/usr/bin/env bash
set -eo pipefail

BUNDLE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${ROS_DISTRO:?请先 source /opt/ros/humble/setup.bash}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${INSTALL_SETUP:-${BUNDLE_ROOT}/ros2_ws/install/setup.bash}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-96}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11396}"
REPORT_DIR="${BUNDLE_ROOT}/reports/multi_robot"
mkdir -p "${REPORT_DIR}"

setsid ros2 launch omnifleet_multi_robot_sim multi_robot_classic.launch.py \
  gui:=false rviz:=false foxglove:=false auto_demo:=true \
  >"${REPORT_DIR}/launch.log" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true
    for _ in $(seq 1 40); do
      kill -0 "${LAUNCH_PID}" 2>/dev/null || break
      sleep 0.25
    done
    kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
  fi
  wait "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 run omnifleet_multi_robot_sim runtime_verifier \
  --timeout "${VERIFY_TIMEOUT:-120}" \
  --output "${REPORT_DIR}/runtime_verification.json"
