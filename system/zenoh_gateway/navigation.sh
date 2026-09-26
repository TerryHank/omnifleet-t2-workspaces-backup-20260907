#!/usr/bin/env bash
set -e
mode="${1:-pure_mola}"
if [ "$#" -gt 0 ]; then shift; fi
case "$mode" in
  pure_mola) profile=simple_direct_observation ;;
  rep105) profile=simple_rep105 ;;
  *) echo "Usage: bash $0 pure_mola|rep105 [map_path:=... initial_pose:=...]" >&2; exit 2 ;;
esac
source /home/iecme/omnifleet_fleet/env.bash
if [ "$OMNIFLEET_ARCHITECTURE" != "$mode" ]; then
  sudo -n bash /home/iecme/msc_v1_ws/deployment/architecture/switch_architecture.sh "$mode"
  source /home/iecme/omnifleet_fleet/env.bash
fi
cd /home/iecme/workspace/omnifleet_t2_mola_experiments_ws
exec ros2 launch omnifleet_t2_mola_experiments mola_nav2_unified.launch.py "profile:=$profile" "$@"
