#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /home/iecme/hardware_drivers_ws/install/setup.bash
source /home/iecme/mola_3_2_ws/install/setup.bash
source /home/iecme/omnifleet_t2_ws/install/setup.bash
source /home/iecme/omnifleet_t2_mola_experiments_ws/install/setup.bash
source /home/iecme/nav2_height_ws/install/setup.bash
if [ -f /home/iecme/mrpt_2_15_21_ws/install/ros2bridge/setup.bash ]; then
  source /home/iecme/mrpt_2_15_21_ws/install/ros2bridge/setup.bash
fi
export LD_LIBRARY_PATH=/home/iecme/mrpt_2_15_21_ws/install/ros2bridge/mrpt_libros_bridge/lib:/home/iecme/mrpt_2_15_21_ws/install/mrpt/lib:${LD_LIBRARY_PATH:-}
set -a
source /etc/omnifleet_t2/robot.env
set +a
exec ros2 launch omnifleet_t2_mola_experiments mola_nav2_unified.launch.py "$@"
