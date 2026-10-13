# Unified MOLA + Nav2 REP-105 experiment

This staged launch starts MOLA and gates Nav2 on `/lidar_odometry/pose`, `/map`,
and `odom -> base_link`. It does not start the chassis or send goals. The
experimental YAML changes only local costmap `global_frame` to `odom`; the
production YAML remains untouched.
