# OmniFleet T2 MOLA isolated experiments

This overlay does not modify the production workspace. The chassis service is
kept running for `/cmd_vel` and raw `/odom`, with its default `odom ->
base_link` TF disabled for the pure-LIO deployment. MOLA owns the direct
`map -> base_link` localization TF. Stop competing MOLA/Nav2 launches before
starting an experiment.

- `simple_direct`: official MOLA Simple LIO direct `map -> base_link` mode;
  Nav2 uses `/lidar_odometry/pose` and local/global costmaps use `map`.
- `simple_rep105`: rollback profile using chassis `odom -> base_link` and MOLA
  `map -> odom`.
- `smoother_direct`: compatibility profile; not the production default.

The experiment map is saved below
`/var/lib/omnifleet_t2/maps/experiments/<profile>/`.

## Cold-switch preflight

Never run two MOLA or two Nav2 launches together:

```bash
ps -eo pid,args | grep -E 'mola_no_wheel.launch.py|nav2_direct.launch.py' | grep -v grep
```

Source the experiment overlay after the production and official MOLA workspaces:

```bash
source /opt/ros/humble/setup.bash
source /home/iecme/workspace/hardware_drivers_ws/install/setup.bash
source /home/iecme/workspace/mola_3_2_ws/install/setup.bash
source /home/iecme/workspace/omnifleet_t2_ws/install/setup.bash
source /home/iecme/workspace/omnifleet_t2_mola_experiments_ws/install/setup.bash
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## Simple direct

Terminal 1:

```bash
ros2 launch omnifleet_t2_mola_experiments mola_no_wheel.launch.py \
  profile:=simple_direct start_chassis:=false \
  map_path:=/var/lib/omnifleet_t2/maps/saved/foxglove_map.mm
```

Terminal 2:

```bash
ros2 launch omnifleet_t2_mola_experiments nav2_direct.launch.py \
  transform_tolerance:=0.8 odom_topic:=/lidar_odometry/pose
```

## Decoupled Smoother direct

Terminal 1:

```bash
ros2 launch omnifleet_t2_mola_experiments mola_no_wheel.launch.py \
  profile:=smoother_direct
```

Terminal 2:

```bash
ros2 launch omnifleet_t2_mola_experiments nav2_direct.launch.py \
  transform_tolerance:=0.8 \
  odom_topic:=/mola_smoother/state_estimation/pose
```

This overlay never launches a chassis node. The production chassis service is
started automatically by systemd; use `sudo systemctl start|stop
omnifleet-t2-chassis.service` only for maintenance.

For a strict no-TF-filter static-map check, append:

```bash
obstacle_topic:=/lidar_odometry/nav_voxelmap_points obstacle_clearing:=false
```

That input is already expressed in `map`, but updates more slowly and does not
ray-clear dynamic obstacles. Keep the default raw Airy input for real dynamic
obstacle navigation.
