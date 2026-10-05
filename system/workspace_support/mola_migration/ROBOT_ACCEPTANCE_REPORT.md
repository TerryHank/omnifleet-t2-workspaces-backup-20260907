# Robot MOLA migration acceptance

Date: 2026-10-06
Host: `iecme@10.0.0.198`
Status: **PASS**

## Build and regression

- Built all 37 canonical MOLA packages in the final workspace.
- Passed all 20 package test gates.
- Passed bridge odometry, smoother direct-TF, static GNSS integration, and moving
  georeferencing integration regressions.
- Replayed the same recorded LiDAR fragment with the baseline and candidate. Both
  produced 23 finite poses and byte-identical trajectories; the saved native map
  reloaded successfully.
- The live 500x500, 0.1 m occupancy map exported to PGM and YAML through
  `nav2_map_server/map_saver_cli`.

## Runtime and parameter checks

- Preserved all 85 captured MOLA, IMU, LiDAR, ROS-domain, and robot-ID runtime
  configuration values. The only new runtime variable is the explicit
  `MOLA_MODULES_LIB_PATH` selecting the unified install.
- Kept the `controller_server` and `planner_server` parameter files byte-identical
  to their pre-migration snapshots.
- `ros2 pkg prefix mola_lidar_odometry` resolves to
  `/home/iecme/workspace/mola_latest_20260917_ws/install/mola_lidar_odometry`.
- The active `mola-cli` process mapped 37 MOLA/MP2P libraries, all from
  `mola_latest_20260917_ws/install_unified`; none came from the old workspace or
  a backup build tree.
- After refreshing ROS services through the updated fleet environment, no live
  process environment or memory map contained the old workspace path.
- The MOLA systemd service and all nine ROS services that had inherited the old
  environment restarted successfully through their existing launch entries.
- A 30-second sensor sample after old-workspace deletion received 301 point clouds
  and 301 poses. Every pose was finite, the maximum age was 126.7 ms, and no pose
  exceeded the existing 500 ms freshness gate. The map publisher count was one;
  the map-to-base transform had one MOLA owner.
- No chassis motion command was issued.

## Retired workspace recovery

The complete `/home/iecme/workspace/mola_3_2_ws` tree was archived before removal.
The archive was checked against a manifest covering 6,295 entries: 4,935 regular
files were SHA-256 verified, 139 symlink targets matched, and 1,221 directories
were present. The compressed archive is 103,250,495 bytes; its SHA-256 is
`cd1777c5d27e535b81b4ceef03a0e4ba567aad9190f729f76be2d1c53c64b65f`.

Archive: `/home/iecme/robot_backups/mola_3_2_ws_pre_unified_20261006.tar.gz`
Manifest:
`/home/iecme/robot_backups/mola_unified_f3e89cf/mola_3_2_ws_pre_unified_20261006.manifest.json`

The live rollback script now restores this archive if the old workspace directory
is absent. Its pre-edit copy and the old MOLA environment script are also saved
under `/home/iecme/robot_backups/mola_unified_f3e89cf/`.

## Limits

The native map saved by the GICP pipeline uses a `KeyframePointCloudMap` layer;
`mm2txt` does not convert that layer. Native save/reload passed, and the occupancy
map export path passed through Nav2's map saver. No claim is made that `mm2txt`
exports this native layer.
