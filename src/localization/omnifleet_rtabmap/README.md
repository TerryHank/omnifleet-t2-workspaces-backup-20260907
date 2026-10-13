# Native RTAB-Map trial

The upstream core and Humble wrapper sources are pinned at version 0.23.7 in `localization/rtabmap/`. Matching native binaries are already installed. Source snapshots have COLCON_IGNORE to keep this workspace build from silently replacing the binary underlay.

```bash
source /home/iecme/omnifleet_ws/env.bash
ros2 launch omnifleet_rtabmap lidar_grid.launch.py --show-args
# Launch only for a separately scheduled sensor trial:
ros2 launch omnifleet_rtabmap lidar_grid.launch.py
```

The default pipeline uses native `rtabmap_odom/icp_odometry` and `rtabmap_slam/rtabmap`, with native scan occupancy grids. It has no dependency on other localization engines or their projection tools. `/rtabmap_trial/map` is an experimental grid; TF publication is disabled and the active navigation backend is not switched. Its database is preserved across runs, not deleted on launch.

The current USB RGB camera has no verified depth/calibration/synchronization contract for a visual SLAM profile. The lidar profile is provided first. Motion deskew, sensor extrinsics, map alignment, native odometry quality and live grid/loop closure need sensor and replay validation before navigation use. The external-odometry argument exists for a deterministic isolated occupancy-grid fixture, not as a production dependency.
