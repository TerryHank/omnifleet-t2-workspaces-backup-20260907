#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /home/iecme/hardware_drivers_ws/install/setup.bash
source /home/iecme/mola_3_2_ws/install/setup.bash
source /home/iecme/omnifleet_t2_ws/install/setup.bash
source /home/iecme/msc_v1_ws/install/setup.bash
set -a
source /etc/omnifleet_t2/robot.env
set +a
exec "$@"