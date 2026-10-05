source /home/iecme/workspace/mola_latest_20260917_ws/latest_env.bash
source /home/iecme/workspace/hardware_drivers_ws/install/local_setup.bash
source /home/iecme/workspace/mola_latest_20260917_ws/install/local_setup.bash
source /home/iecme/workspace/omnifleet_t2_ws/install/local_setup.bash
source /home/iecme/workspace/omnifleet_t2_mola_experiments_ws/install/local_setup.bash
source /home/iecme/msc_v1_ws/install/local_setup.bash
source /home/iecme/nav2_height_ws/install/local_setup.bash
export LD_LIBRARY_PATH="/home/iecme/mrpt_2_15_21_ws/install/ros2bridge/mrpt_libros_bridge/lib:/home/iecme/mrpt_2_15_21_ws/install/mrpt/lib:${LD_LIBRARY_PATH:-}"
set -a
source /etc/omnifleet_t2/robot.env
source /etc/omnifleet_t2/architecture.env
set +a
export PYTHONPATH="/home/iecme/omnifleet_fleet:${PYTHONPATH:-}"
unset ZENOH_CONFIG_OVERRIDE

# Matched RMW/C headers and ROS-pinned Zenoh 1.8 with PR 2709 backport.
source /home/iecme/fleet_rmw_ws/install/local_setup.bash
export LD_LIBRARY_PATH="/home/iecme/omnifleet_fleet/zenoh-patched-v2/lib:${LD_LIBRARY_PATH:-}"
