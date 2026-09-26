# 接口合同

| 接口 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 输入 | `linear.x` 车体速度，`angular.z` 偏航角速度 |
| `/rrc_safety/zero_lock` | `std_msgs/msg/Bool` | 输入 | true 时清除旧命令并强制停车 |
| `/odom` | `nav_msgs/msg/Odometry` | 输出 | `odom→base_link` 位姿与速度 |
| `/wheel/odometry` | `nav_msgs/msg/Odometry` | 输出 | 与实车反馈 adapter 对齐的轮式里程计接口 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 输出 | 双前轮舵角与四轮转角 |
| `/vel_raw` | `geometry_msgs/msg/Twist` | 输出 | `linear.y` 保留实车合同：中心舵角，单位度 |
| `/motor_command_sent` | `geometry_msgs/msg/Twist` | 输出 | 最终仿真执行命令 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 输出 | 唯一 `odom→base_link` 动态 TF |
| `/driver/mode` | `std_msgs/msg/String` | 输出 | 仿真驱动发布 `simulation` |

上表的扩展反馈话题由纯 ROS 运动学后端完整提供。Gazebo Classic 官方插件直接提供
`/cmd_vel`、`/odom`、TF、`/joint_states`、`/distance` 和 `/steerangle`；详见
`BACKEND_CLASSIFICATION.md`。

## 几何与运动学

- `base_link`：后轴中点、轮心高度，X 前、Y 左、Z 上。
- 轴距：`0.362295943 m`
- 前轮距：`0.264956799 m`
- 后轮距：`0.245400019 m`
- 轮半径：`0.058528263 m`
- 最大中心舵角：`0.523599 rad`
- 默认最大车速：`±0.40 m/s`
- 默认最大偏航角速度：`±0.30 rad/s`

仿真驱动使用自行车模型 `ω = v·tan(δ)/L`，再根据前轮距求左右 Ackermann 舵角。
当 `v=0` 时 `ω` 强制为零；可通过实车兼容合同 `cmd_vel.linear.y`（单位度）测试静止舵机。
