# MOLA consolidation candidate
Status: PENDING_LINUX_VALIDATION. No robot connection or deployment is performed in this stage.

The active tree has 37 canonical MOLA package roots under localization/<package>.
The 24 legacy packages, four metadata groups and old source archives were retired
from the local project and Git branch by explicit user approval on 2026-10-05.
They were archived outside the project and verified before removal; see
RETIREMENT_RECORD.json. Git history and the rollback tag retain recovery.
Every active MOLA package now has exactly one source directory.
Linux compilation and runtime acceptance remain pending; the robot is untouched.

Base: 8c3fae37989f7488503f35ae6ba0c0b093bb7b03.
Rollback: pre-mola-latest-consolidation-20261005; raw archive and existing backup tags remain.
The dated SOURCE_PATHS_20260929.tsv and SNAPSHOT_20260929.json describe the immutable
backup-20260929-source-198 tree, not this migration candidate. Current SOURCE_PATHS.tsv
and PATH_RELOCATION.tsv are remapped to the active and ignored trees.

## Validation
Run validate_candidate.py with Python, PyYAML installed. Run the existing MOLA
backend and experiment pytest contracts. These validate syntax/layout/configuration,
not C++ compilation or ROS behavior.

For Linux, materialize the 37 roots from ACTIVE_MOLA_PACKAGES.tsv as src/<package>
in an isolated candidate workspace, preserving executable modes and symlinks.
Use the existing latest_env.bash dependency environment, never an old install overlay:
  colcon build --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
  colcon test --event-handlers console_direct+
  colcon test-result --verbose
Rebuild all MOLA libraries and dependent OmniFleet overlays: LocalizationUpdate
changed C++ ABI. Do not mix previously built libraries.

Replay recorded data before stationary sensor testing. Verify all profiles, native
map save/load/export, source timestamps, velocity and covariance selection, unique
TF publishers and the existing 0.5-second freshness gate with timely input.
Check the effective latest ROS_ARGS and launch remapping order, plus fleet_scope
and external MRPT/GTSAM/Zenoh dependencies. No chassis motion commands are authorized.

After isolated validation, deploy source to the final latest workspace prefix and
rebuild there; install scripts can contain absolute build-prefix paths. Repeat checks
at that final location. Robot legacy retirement remains gated on the recorded runtime
checks and a verified recovery archive outside /home/iecme/workspace.
Local/Git cleanup was separately approved and does not imply robot acceptance.
This stage contains no robot deletion command.

## Changes to public interfaces
LocalizationUpdate adds optional twist and twist_cov; absence preserves zero output.
The smoother's publication options and bridge wheel-source options are opt in.
FEATURE_PARAMETER_MATRIX.md documents package, function and parameter coverage.

The runtime fleet environment must be updated from system/zenoh_gateway/env.bash
together with latest_env.bash before replay. The fleet environment performs the
single loader-path reset and then restores sensor, project and patched Zenoh paths;
the experiment subprocess must not reset those paths a second time.
