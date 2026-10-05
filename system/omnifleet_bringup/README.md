# omnifleet_bringup（OmniFleet T2）

本包是 113 履带车的生产启动入口。STM32 底盘只通过
`/dev/omnifleet_t2_stm32` 打开；左右履带差速、电机方向、PWM 补偿和编码器
PID 由履带固件负责，上位机发布标准 `/cmd_vel`。

## 常驻服务

```bash
systemctl status omnifleet-t2-chassis.service
systemctl status omnifleet-t2-airy.service
journalctl -u omnifleet-t2-chassis.service -f
```

底盘发布 `/odom`、`odom -> base_link`、`/voltage`、`/voltage/display`、
`/firmware/version`、`/vel_raw`、`/wheel/odometry`、`/imu/data_raw`、
`/imu/mag` 和 `/motor_command_sent`。Airy 服务发布 `/rslidar_points` 与
`/rslidar_imu_data`。

`omnifleet-t2-navigation.service` 固定保持 disabled，导航由操作员手动启动。

## 手动启动 MOLA + Nav2

```bash
source /opt/ros/humble/setup.bash
source /home/iecme/workspace/hardware_drivers_ws/install/setup.bash
source /home/iecme/workspace/mola_latest_20260917_ws/install/setup.bash
cd /home/iecme/workspace/omnifleet_t2_ws
source install/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch omnifleet_bringup production.launch.py \
  backend:=mola \
  map_id:=active \
  map_dir:=/var/lib/omnifleet_t2/maps \
  start_description:=false
```

生产默认使用 MOLA、SmacPlanner2D 和 DWB。底盘驱动永远由
`omnifleet-t2-chassis.service` 通过 `systemctl` 管理；任何 MOLA、Nav2 或
上层 `robot.launch.py` 入口都不再接受或启动底盘节点。Airy 和车体描述同样
由各自常驻服务管理，生产导航入口只连接它们。

## 只读健康检查

```bash
ros2 run omnifleet_bringup robot_doctor --scope base
ros2 run omnifleet_bringup robot_doctor --scope nav
ros2 run omnifleet_bringup vehicle_status
```
