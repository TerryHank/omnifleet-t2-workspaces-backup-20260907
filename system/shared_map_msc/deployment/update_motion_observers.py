"""Keep existing diagnostics and parameter safety checks aware of the new velocity route."""
import ast,shutil
from pathlib import Path
root=Path('/home/iecme/workspace/foxglove')
backup=Path('/home/iecme/robot_backups/msc_v1_20260909/motion_observers');backup.mkdir(parents=True,exist_ok=True)
patches={
 'nav2_permanent_panel/nav2_parameter_store.py':[
 ("            self.create_subscription(Twist, '/cmd_vel', self.observe_motion, 1, callback_group=self.clients_group)",
  "            for motion_topic in ('/cmd_vel', '/msc/nav_cmd_vel', '/motor_command_sent'):\n                self.create_subscription(Twist, motion_topic, self.observe_motion, 1, callback_group=self.clients_group)")],
 'dsh_diagnostic_panel/diagnostic_node.py':[
 ("for t in ('/odom','/lidar_odometry/pose','/cmd_vel')", "for t in ('/odom','/lidar_odometry/pose','/cmd_vel','/msc/nav_cmd_vel','/msc/cmd_vel_safe','/motor_command_sent')"),
 ("('/cmd_vel',Twist,sensor),('/plan',Path,sensor)", "('/cmd_vel',Twist,sensor),('/msc/nav_cmd_vel',Twist,sensor),('/msc/cmd_vel_safe',Twist,sensor),('/motor_command_sent',Twist,sensor),('/plan',Path,sensor)"),
 ("            if topic!='/cmd_vel':", "            if topic in ('/odom','/lidar_odometry/pose'):"),
 ("        data['motion_window']={}",
  "        if any(t=='/msc/cmd_vel_safe' for t,_ in self.get_topic_names_and_types()):\n            data['velocity_topic_roles']={'/cmd_vel':'manual input', '/msc/nav_cmd_vel':'Nav2 requested velocity before safety gate', '/msc/cmd_vel_safe':'command after safety gate', '/motor_command_sent':'driver transmitted command, not measured wheel velocity', '/odom':'measured chassis velocity'}\n        data['motion_window']={}")]
}
for relative,replacements in patches.items():
 path=root/relative
 if not path.exists():print('not installed:',relative);continue
 original=path.read_text();updated=original
 for old,new in replacements:
  if new in updated:continue
  assert updated.count(old)==1,(relative,old)
  updated=updated.replace(old,new,1)
 ast.parse(updated)
 if updated!=original:
  target=backup/path.name
  if not target.exists():shutil.copy2(path,target)
  path.write_text(updated)
 print('verified:',relative)
