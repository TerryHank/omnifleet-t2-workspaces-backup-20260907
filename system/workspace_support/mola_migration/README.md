# MOLA consolidation candidate
Status: PENDING_LINUX_VALIDATION. No robot connection or deployment is performed in this stage.

The active tree has 37 canonical MOLA package roots under localization/<package>.
The 24 differing legacy packages, four metadata groups and old workspace source
archives remain under localization/_legacy_mola_3_2_pending_validation with COLCON_IGNORE.
They are unchanged and must stay until every robot acceptance gate passes.

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
at that final location. Only then archive the old workspace outside /home/iecme/workspace,
verify archive hashes and loaded library/underlay paths, and remove the legacy tree
in Linux/local/Git together. This stage contains no deletion command.

## Changes to public interfaces
LocalizationUpdate adds optional twist and twist_cov; absence preserves zero output.
The smoother's publication options and bridge wheel-source options are opt in.
FEATURE_PARAMETER_MATRIX.md documents package, function and parameter coverage.
