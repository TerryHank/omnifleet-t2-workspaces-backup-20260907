# OmniFleet robot 113 workspace source backup

Source: `/home/iecme/workspace` on `iecme@192.168.3.113`  
Snapshot date: 2026-09-24  
Original tar SHA-256: `05a00928acd12cdf1eb63e52f55b84f06f44f9e487f1b84a07ac77d112354c2e`

The repository root follows the subsystem grouping used by [Autoware Universe](https://github.com/autowarefoundation/autoware_universe): `common`, `control`, `localization`, `perception`, `planning`, `sensing`, `simulator`, `system`, `vehicle`, and `visualization`. These categories organize the existing robot code; they do not make it an Autoware project. Active MOLA package directories are canonical under localization/<package>, based on the latest source snapshot. Eleven previously identical pairs remain shared; the 24 legacy differing packages have been retired from the local and Git source trees after explicit user approval and verified external archival. Upstream metadata is under localization/mola_source_metadata. See system/workspace_support/mola_migration/FEATURE_PARAMETER_MATRIX.md and its acceptance status. SOURCE_PATHS.tsv records the current retained source paths. Non-`src` configuration and scripts are under `system/workspace_support`, except Foxglove files under `visualization`.

`SOURCE_PATHS.tsv` maps the original source-snapshot entries back to their source paths. `PATH_RELOCATION.tsv` records each path moved in this layout update. All six workspaces from the source root are represented. Restore on Linux by copying each mapped path to the original relative path beneath a new workspace directory. Git stores the original relative symlink targets.

Excluded from this Git source backup: ROS/colcon `build`, `install`, and `log` outputs; nested `.git` histories; Python caches; Foxglove `node_modules`; MOLA generated reports and dependency mirrors; historical backups; crash dumps. Rebuild dependencies and generated files when restoring. The original compressed source snapshot remains on robot 113 at `/home/iecme/robot_backups/workspace_autoware_root_20260924/workspace-source.tar.gz` and has the SHA-256 above.

This repository is private because robot, device and network configuration are included. The snapshot was scanned for high-confidence API tokens and private key markers before import.

## Foxglove update (2026-09-26)

The current Foxglove source and optimized artifacts from robot 113 are under `visualization/omnifleet_t2_ws/foxglove`. Layout/style snapshots are under `visualization/foxglove_layouts`; WebSocket/Zenoh and extension optimization evidence is under `visualization/foxglove_optimized`. Their individual README files describe the source paths and exclusions. The payload SHA-256 is recorded in `visualization/foxglove_snapshot.sha256`.

## Multi-robot shared-map and Zenoh update (2026-09-26)

The repository now includes the related 113 materials under `system/README_MULTI_ROBOT_ZENOH.md`, with a path manifest at `system/MULTI_ROBOT_ZENOH_PATHS.tsv` and source snapshot hash at `system/MULTI_ROBOT_ZENOH_SNAPSHOT.sha256`.

## Source refresh (2026-09-29)

Source: `iecme@10.0.0.198:/home/iecme/workspace` (`iecme-tank`). The six workspaces were inventoried before backup. This Git refresh maps 4,920 regular files and 10 symlinks into the existing subsystem layout. `SOURCE_PATHS_20260929.tsv` is the exact selected-file list, while `SOURCE_PATHS.tsv` keeps cumulative source-path provenance. The previous repository materials and historical snapshot entries remain available.

The source archive is stored on the robot at `/home/iecme/robot_backups/workspace_source_20260929/workspace-source-20260929.tar.gz`; its SHA-256 is in `SOURCE_ARCHIVE_20260929.sha256`. The Git backup excludes generated build/install/log/report trees, nested Git histories, caches, node_modules, old backup directories, and ROS bag files. Package source, configuration, vendored runtime files, source archives, and dependency manifests are included. The raw source archive also retains eight Git submodule pointer files that GitHub rejects as path components; they are excluded from the Git tree. A source backup does not establish build or live robot acceptance.

## MOLA migration candidate (2026-10-05)

Only local code and GitHub candidate changes are made. Local/Git legacy cleanup was explicitly approved on 2026-10-05. Robot retirement remains gated
on Linux build, replay, stationary robot and final-prefix validation. Dated snapshot
manifests remain immutable and resolve against backup-20260929-source-198.

The local project and current Git tree contain one copy of each of the 37 MOLA
packages. Recovery is recorded in system/workspace_support/mola_migration/RETIREMENT_RECORD.json.
C++ build and ROS runtime validation are still pending; the robot was not changed.
