#!/usr/bin/env bash
set -e
source /home/iecme/omnifleet_fleet/env.bash
exec python3 /home/iecme/omnifleet_fleet/run.py "$@"
