import json,shutil,yaml
from pathlib import Path
backup=Path('/home/iecme/robot_backups/msc_shared_map_dsh_20260909');backup.mkdir(parents=True,exist_ok=True)
config=Path('/home/iecme/workspace/src/omnifleet_planner/config/nav2_t2.yaml')
target=backup/'nav2-before-shared-map.yaml'
if not target.exists():shutil.copy2(config,target)
data=yaml.safe_load(config.read_text());global_map=data['global_costmap']['global_costmap']['ros__parameters']
global_map['global_frame']='fleet_map';global_map['static_layer']['map_topic']='/fleet/map'
global_map['static_layer']['map_subscribe_transient_local']=True
if 'peer_layer' not in global_map['plugins']:global_map['plugins'].insert(1,'peer_layer')
global_map['peer_layer']={'plugin':'nav2_costmap_2d::StaticLayer','map_topic':'/msc/peer_map',
    'map_subscribe_transient_local':True,'subscribe_to_updates':False,'use_maximum':True}
config.write_text(yaml.safe_dump(data,sort_keys=False,allow_unicode=True))
print('global_costmap: frame=fleet_map, static source=/fleet/map; local costmap unchanged')
