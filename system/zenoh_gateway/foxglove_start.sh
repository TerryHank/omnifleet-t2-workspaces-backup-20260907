#!/usr/bin/env bash
set -eo pipefail

source /home/iecme/omnifleet_fleet/env.bash

# Foxglove uses the public robot-scoped navigation action directly.
# Keep the fleet-wide backend/cmd_vel remaps for other processes.
export OMNIFLEET_FOXGLOVE_DIRECT_ACTIONS=1

runtime_dir="/run/user/$(id -u)/omnifleet_t2"
rendered_config="${runtime_dir}/foxglove-${OMNIFLEET_ROBOT_ID}.yaml"
mkdir -p "${runtime_dir}"

python3 /home/iecme/omnifleet_fleet/render_foxglove_config.py \
  --source /etc/omnifleet_t2/foxglove.yaml \
  --output "${rendered_config}"

exec /bin/bash /home/iecme/omnifleet_fleet/start.sh run foxglove_bridge foxglove_bridge \
  --ros-args \
  -r __node:=omnifleet_t2_foxglove_bridge \
  --params-file "${rendered_config}" \
  -p address:="${OMNIFLEET_T2_FOXGLOVE_ADDRESS}" \
  -p port:="${OMNIFLEET_T2_FOXGLOVE_PORT}"
