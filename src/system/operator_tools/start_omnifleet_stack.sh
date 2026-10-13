#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /home/iecme/omnifleet_ws/hardware_drivers_ws/install/setup.bash
source /home/iecme/omnifleet_ws/mola_3_2_ws/install/setup.bash
source /home/iecme/omnifleet_ws/omnifleet_t2_ws/install/setup.bash
source /home/iecme/omnifleet_ws/omnifleet_t2_mola_experiments_ws/install/setup.bash
source /home/iecme/mrpt_2_15_21_ws/install/ros2bridge/setup.bash
set -u
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=DEFAULT
sudo systemctl start omnifleet-t2-airy.service omnifleet-t2-description.service omnifleet-t2-chassis.service omnifleet-t2-foxglove.service omnifleet-t2-waypoints.service omnifleet-t2-nav2-parameters.service
sleep 8
if pgrep -f 'mola_nav2_unified.launch.py' >/dev/null; then
  echo '统一 launch 已在运行。'
  sudo systemctl restart omnifleet-t2-foxglove.service
else
  nohup ros2 launch omnifleet_t2_mola_experiments mola_nav2_unified.launch.py >/tmp/mola_nav2_desktop_start.log 2>&1 </dev/null &
  echo '已启动统一 launch，日志：/tmp/mola_nav2_desktop_start.log'
  for i in $(seq 1 90); do
    if grep -q 'Managed nodes are active' /tmp/mola_nav2_desktop_start.log 2>/dev/null; then
      sudo systemctl restart omnifleet-t2-foxglove.service
      echo 'Nav2 已 active，Foxglove bridge 已重新扫描话题。'
      break
    fi
    sleep 1
  done
fi
if ! pgrep -f '^/opt/Foxglove-Studio-CN/foxglove-studio-cn ' >/dev/null; then
  DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority XDG_RUNTIME_DIR=/run/user/1000 DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus nohup /opt/Foxglove-Studio-CN/foxglove-studio-cn --force-device-scale-factor=0.75 >/tmp/foxglove-desktop-start.log 2>&1 </dev/null &
fi
echo '启动完成。'
