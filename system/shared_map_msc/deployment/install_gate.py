"""Scoped source edit. Run only after idle checks and backups on the target robot."""
from pathlib import Path

root=Path('/home/iecme/workspace/src')
driver=root/'omnifleet_bringup/omnifleet_bringup/omnifleet_t2_driver.py'
text=driver.read_text()
old='self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)'
new='self.create_subscription(Twist, os.environ.get("OMNIFLEET_CMD_VEL_INPUT", "/cmd_vel"), self.cmd_vel_callback, 1)'
if new not in text:
    assert text.count(old)==1,'driver source differs'
    text=text.replace(old,new)
    if '\nimport os\n' not in text:text=text.replace('\nimport math\n','\nimport math\nimport os\n')
    driver.write_text(text)
navigation=root/'omnifleet_planner/launch/navigation.launch.py';text=navigation.read_text()
text=text.replace('("cmd_vel", "/cmd_vel")','("cmd_vel", "/msc/nav_cmd_vel")')
needle='executable="bt_navigator",'
if 'executable="bt_navigator",\n            remappings=' not in text:
    assert needle in text
    text=text.replace(needle,needle+'\n            remappings=[("/cmd_vel", "/msc/nav_cmd_vel")],')
navigation.write_text(text)
chassis=root/'omnifleet_bringup/launch/chassis_core.launch.py';text=chassis.read_text()
if 'OMNIFLEET_PUBLISH_ODOM_TF' not in text:
    text=text.replace('from launch import','import os\n\nfrom launch import',1)
    text=text.replace('DeclareLaunchArgument("publish_odom_tf", default_value="true")',
        'DeclareLaunchArgument("publish_odom_tf", default_value=os.environ.get("OMNIFLEET_PUBLISH_ODOM_TF", "true"))')
chassis.write_text(text)
print('Driver input and Nav2 output hooks installed; services must be updated/restarted separately.')
