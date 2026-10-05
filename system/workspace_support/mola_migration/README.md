# MOLA consolidation and robot acceptance

Status: **PASS**. Local and GitHub source trees keep one canonical source for each
of the 37 MOLA packages under `localization/<package>`. The robot now uses
`/home/iecme/workspace/mola_latest_20260917_ws` as its only MOLA workspace.

The old robot workspace was archived outside `/home/iecme/workspace`, verified
against a file-by-file manifest, and removed after build, replay, and stationary
sensor acceptance. See [ROBOT_ACCEPTANCE_REPORT.md](ROBOT_ACCEPTANCE_REPORT.md)
and [ROBOT_RETIREMENT_RECORD.json](ROBOT_RETIREMENT_RECORD.json) for evidence and
the recovery archive checksums.

## Source and runtime behavior

The latest implementation and parameters are the baseline. The migration retains
the old-only optional odometry twist/covariance output, smoother direct
`map -> odom` publication, and bridge wheel-source selection. These features
remain opt-in. Existing production parameter values were compared against the
robot snapshot and preserved. See [FEATURE_PARAMETER_MATRIX.md](FEATURE_PARAMETER_MATRIX.md).

`latest_env.bash` clears inherited MOLA module paths before setting the canonical
installed package paths. The launcher gives `MOLA_MODULES_LIB_PATH` precedence
over its build-time module path so a running deployment cannot load duplicate
modules from an archived build tree. The production service environment and its
systemd loader-path drop-in are versioned here as
`robot_mola_env.bash` and `24-unified-mola-module-path.conf`.

## Validation

- Static layout: 67 active ROS packages, 37 MOLA packages, no duplicate package roots.
- Local Python contracts: 12 tests passed; launch Python and YAML syntax checks passed.
- Robot build: all 37 packages built; all 20 package test gates passed.
- Recorded-data replay: baseline and candidate trajectories were byte-identical;
  the saved native map reloaded successfully.
- Stationary robot: post-deletion 30-second sample had 301 point clouds and 301
  finite poses; maximum pose age was 126.7 ms, with no samples over 500 ms.
- Map export: the live 500x500 occupancy map exported to PGM and YAML.
- Runtime: all 37 mapped MOLA/MP2P libraries came from the unified install;
  no live process environment or memory map retained the old workspace path.
- Nav2 `controller_server` and `planner_server` parameter files matched their
  pre-migration snapshots.

No chassis motion command was sent. The archive is the recovery source for the
removed robot workspace; the retained rollback script restores it before running
the prior production restart procedure.
