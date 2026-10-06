# OmniFleet robot 113 workspace source backup

Source: `/home/iecme/workspace` on `iecme@192.168.3.113`  
Snapshot date: 2026-09-24  
Original tar SHA-256: `05a00928acd12cdf1eb63e52f55b84f06f44f9e487f1b84a07ac77d112354c2e`

The repository root follows the subsystem grouping used by [Autoware Universe](https://github.com/autowarefoundation/autoware_universe): `common`, `control`, `localization`, `perception`, `planning`, `sensing`, `simulator`, `system`, `vehicle`, and `visualization`. These categories organize the existing robot code; they do not make it an Autoware project. Active MOLA package directories are canonical under `localization/omnifleet_mola/<package>`, based on the latest source snapshot. Eleven previously identical pairs remain shared; the 24 legacy differing packages have been retired from the local and Git source trees after explicit user approval and verified external archival. Upstream metadata is under localization/omnifleet_mola/mola_source_metadata. See system/workspace_support/mola_migration/FEATURE_PARAMETER_MATRIX.md and its acceptance status. `SOURCE_PATHS.tsv` and the dated manifests preserve source-path provenance. Non-`src` configuration and scripts are under `system/workspace_support`, except Foxglove files under `visualization`.

`SOURCE_PATHS.tsv` maps the original source-snapshot entries back to their source paths. `PATH_RELOCATION.tsv` records each path moved in this layout update. All six workspaces from the source root are represented. Restore on Linux by copying each mapped path to the original relative path beneath a new workspace directory. Git stores the original relative symlink targets.

Excluded from this Git source backup: ROS/colcon `build`, `install`, and `log` outputs; nested `.git` histories; Python caches; Foxglove `node_modules`; MOLA generated reports and dependency mirrors; historical backups; crash dumps. Rebuild dependencies and generated files when restoring. The original compressed source snapshot remains on robot 113 at `/home/iecme/robot_backups/workspace_autoware_root_20260924/workspace-source.tar.gz` and has the SHA-256 above.

This repository is public. The owner explicitly authorized this source refresh on 2026-10-07. The filtered snapshot contains no credential files, private keys, or high-confidence token values; runtime credentials are loaded from outside the workspace.

## Foxglove update (2026-09-26)

The current Foxglove source and optimized artifacts from robot 113 are under `visualization/omnifleet_t2_ws/foxglove`. Layout/style snapshots are under `visualization/foxglove_layouts`; WebSocket/Zenoh and extension optimization evidence is under `visualization/foxglove_optimized`. Their individual README files describe the source paths and exclusions. The payload SHA-256 is recorded in `visualization/foxglove_snapshot.sha256`.

## Multi-robot shared-map and Zenoh update (2026-09-26)

The repository now includes the related 113 materials under `system/README_MULTI_ROBOT_ZENOH.md`, with a path manifest at `system/MULTI_ROBOT_ZENOH_PATHS.tsv` and source snapshot hash at `system/MULTI_ROBOT_ZENOH_SNAPSHOT.sha256`.

## Source refresh (2026-09-29)

Source: `iecme@10.0.0.198:/home/iecme/workspace` (`iecme-tank`). The six workspaces were inventoried before backup. This Git refresh maps 4,920 regular files and 10 symlinks into the existing subsystem layout. `SOURCE_PATHS_20260929.tsv` is the exact selected-file list, while `SOURCE_PATHS.tsv` keeps cumulative source-path provenance. The previous repository materials and historical snapshot entries remain available.

The source archive is stored on the robot at `/home/iecme/robot_backups/workspace_source_20260929/workspace-source-20260929.tar.gz`; its SHA-256 is in `SOURCE_ARCHIVE_20260929.sha256`. The Git backup excludes generated build/install/log/report trees, nested Git histories, caches, node_modules, old backup directories, and ROS bag files. Package source, configuration, vendored runtime files, source archives, and dependency manifests are included. The raw source archive also retains eight Git submodule pointer files that GitHub rejects as path components; they are excluded from the Git tree. A source backup does not establish build or live robot acceptance.

## Unified MOLA migration and remote workspace normalization (2026-10-06)

The active robot workspace is `/home/iecme/workspace`. Its source packages are directly under the ten subsystem roots used by the local tree and the Autoware Universe layout: `common`, `control`, `localization`, `perception`, `planning`, `sensing`, `simulator`, `system`, `vehicle`, and `visualization`. This is a directory-organization reference only; this project is not an Autoware distribution. The 37 MOLA/MP2P source packages are retained once each under `localization/omnifleet_mola/<package>`. The complete active workspace lists 68 packages with 68 unique names.

The canonical robot install path is `/home/iecme/workspace/.runtime/install`; build, install, and log outputs stay under `.runtime`. When running colcon manually, pass `--log-base .runtime/log` (or `--log-base /dev/null` for read-only inventory commands) so colcon does not create a top-level `log` directory. MOLA-related ROS packages are grouped under `localization/omnifleet_mola`; the group remains a container, and its 39 ROS packages are still discovered independently.

After the grouping, the complete 68-package build passed. The full colcon result is 676 tests, 0 errors, 20 failures, and 0 skipped; all failures are lint/copyright checks in `ros_robot_controller`, `ros_robot_controller_msgs`, and `rslidar_msg`. The `omnifleet_localization` tests pass with paths adjusted for the added group level. All 16 services are active. An 8-second read-only probe observed 81 finite odometry samples, zero displacement, no `/robot_113/msc/nav_cmd_vel` samples, and a live 500×500 map. No chassis motion command was sent. Earlier replay, map reload/export, and relocalizer smoke evidence is recorded in the acceptance report; those checks were not repeated after this path-only grouping.

The pre-group build/install and intermediate zero-byte build artifacts are preserved under `/home/iecme/robot_backups/workspace_layout_reorg_20261006/mola-grouping-rollback-20261006/`. See `system/workspace_support/workspace_reorganization/REMOTE_LAYOUT_ACCEPTANCE_20261006.md` and `system/workspace_support/mola_migration/ROBOT_ACCEPTANCE_REPORT.md` for detailed evidence.

After the source move and service refresh, 16 Omnifleet services were active and no service/process referenced the old workspace wrappers. A separate sibling simulation tree with three duplicate package names was moved intact to the robot backup's `retired` directory and marked `COLCON_IGNORE`. No chassis motion command was issued. The pre-change workspace/configuration archives and hashes are recorded in the robot acceptance report; the full tree audit is in `system/workspace_support/workspace_reorganization/REMOTE_LAYOUT_ACCEPTANCE_20261006.md`. The dated source snapshot manifests above remain historical provenance records and were not rewritten.

The GitHub backup page was checked on 2026-10-06 and showed public visibility. This workspace operation does not change GitHub visibility or push runtime acceptance records.

## Source refresh (2026-10-07)

Source: iecme@10.0.0.198:/home/iecme/workspace (iecme-tank). This refresh records the workspace after MOLA packages were grouped under localization/omnifleet_mola. The active workspace colcon list contained 68 packages with 68 unique package names.

The filtered snapshot contains 3,427 regular files and 9 symbolic links (412,252,403 regular-file bytes). SOURCE_PATHS_20261007.tsv lists each archived path and its SHA-256 or symlink target. SNAPSHOT_20261007.json records the source, exclusions, counts, and archive SHA-256 c2b3328239967c4eebaf8389364a3a9c448afabdbb21b9f768afda9921835a7d; SOURCE_ARCHIVE_20261007.sha256 stores the same archive hash. The compressed transfer archive is not committed and is removed after remote verification.

Excluded generated content: .runtime, all build, install, and log directories, nested .git histories, node_modules, caches, reports, backups, Python bytecode, logs, Foxglove dist bundles, and .foxe extension packages. Source, configuration, test fixtures, model weights, and vendor runtime libraries remain in the source snapshot.
