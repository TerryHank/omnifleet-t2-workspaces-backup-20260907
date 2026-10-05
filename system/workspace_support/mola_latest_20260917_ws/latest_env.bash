#!/usr/bin/env bash

# Isolated build/runtime environment for the latest MOLA workspace.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
unset MOLA_DIR mp2p_icp_DIR mola_DIR
# Rebuild the loader search path instead of inheriting an old MOLA overlay.
unset LD_LIBRARY_PATH PKG_CONFIG_PATH

source /opt/ros/humble/setup.bash

export CMAKE_PREFIX_PATH="/home/iecme/mrpt_2_15_21_ws/install/mrpt:/home/iecme/mrpt_2_15_21_ws/install/mrpt/share/mrpt:/home/iecme/mrpt_2_15_21_ws/install/ros2bridge/mrpt_libros_bridge:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="/home/iecme/mrpt_2_15_21_ws/install/mrpt/lib:/home/iecme/mrpt_2_15_21_ws/install/ros2bridge/mrpt_libros_bridge/lib:${LD_LIBRARY_PATH:-}"
export PKG_CONFIG_PATH="/home/iecme/mrpt_2_15_21_ws/install/mrpt/lib/pkgconfig:${PKG_CONFIG_PATH:-}"

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

