#!/bin/bash
set -e
exec /bin/bash /home/iecme/omnifleet_fleet/start.sh run omnifleet_msc "$1" --config "$2"
