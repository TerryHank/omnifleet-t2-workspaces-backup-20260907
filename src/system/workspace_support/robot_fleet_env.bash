#!/usr/bin/env bash
set -e

source /home/iecme/omnifleet_ws/src/system/workspace_support/mola_latest_20260917_ws/latest_env.bash
source /home/iecme/omnifleet_ws/install/local_setup.bash

set -a
source /etc/omnifleet_t2/robot.env
source /etc/omnifleet_t2/architecture.env
set +a

export PYTHONPATH="/home/iecme/omnifleet_ws/src/system/omnifleet_fleet:${PYTHONPATH:-}"
unset ZENOH_CONFIG_OVERRIDE
export LD_LIBRARY_PATH="/home/iecme/omnifleet_ws/src/system/omnifleet_fleet/zenoh-patched-v2/lib:${LD_LIBRARY_PATH:-}"
export LD_PRELOAD=/home/iecme/omnifleet_ws/src/system/omnifleet_fleet/zenoh-patched-v2/lib/libzenohc.so
