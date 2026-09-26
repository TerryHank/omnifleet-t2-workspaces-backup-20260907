import re
from pathlib import Path
ROOT=Path(__file__).parents[1]
ARCH=ROOT/'deployment'/'architecture'
def test_unified_defaults_to_pure_mola_common_map():
 text=(ARCH/'mola_nav2_unified.launch.py').read_text()
 assert "DeclareLaunchArgument('profile',default_value='simple_direct'" in text
 assert "DEFAULT_MAP='/home/iecme/maps/foxglove_map.mm'" in text
 assert "'map_path:='+map_path" in text
 assert "if initial_pose:command.append('initial_pose:='+initial_pose)" in text
def test_rep105_alias_uses_same_unified_path():
 text=(ARCH/'mola_nav2_rep105.launch.py').read_text()
 assert "'profile':'simple_rep105'" in text and 'foxglove_map.mm' in text
def test_profile_and_tf_ownership_are_checked():
 text=(ARCH/'mola_no_wheel.launch.py').read_text()
 assert "required_mode='rep105'" in text
 assert 'architecture mismatch' in text
 script=(ARCH/'switch_architecture.sh').read_text()
 assert 'OMNIFLEET_PUBLISH_ODOM_TF' in script and "[m]ola-cli" in script
def test_map_file_and_env_defaults_are_one_contract():
 env=(ARCH/'architecture.env').read_text()
 assert 'OMNIFLEET_ARCHITECTURE=pure_mola' in env and 'OMNIFLEET_PUBLISH_ODOM_TF=false' in env
 path=Path('/home/iecme/maps/foxglove_map.mm')
 if not path.exists():path=ROOT/'evidence'/'shared_map_architecture'/'foxglove_map.mm'
 assert path.stat().st_size>100000
