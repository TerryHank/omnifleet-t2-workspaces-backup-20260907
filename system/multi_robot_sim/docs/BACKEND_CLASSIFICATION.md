# 后端分类

| 分类 | 启动入口 | 是否访问串口 | 主要输出 | 用途 |
|---|---|---:|---|---|
| Gazebo Classic 11（默认） | `gazebo_classic.launch.py` | 否 | `/odom`、TF、`/joint_states` | ROS 2 Humble 物理仿真 |
| 纯 ROS 运动学 | `standalone_sim.launch.py` | 否 | `/odom`、TF、`/joint_states`、`/vel_raw`、`/motor_command_sent` | 无 Gazebo、CI、Nav2 快速联调 |
| Gazebo Harmonic | `agent_sim.launch.py` | 否 | `/odom`、TF、`/joint_states`、传感器桥 | Jazzy/Harmonic 兼容 |
| STM32 实车驱动 | `omnifleet_bringup`（默认忽略） | **是** | 实车轮速、IMU、电压、固件、关节 | 只用于真实 R2 底盘 |

不要同时启动任意两个仿真后端；否则 `/odom`、`/tf` 和 `/joint_states` 会出现重复发布者。

Gazebo Classic 插件没有本包运动学节点的 0.35 秒输入看门狗，停止 Classic 模型时必须发送
显式零 `Twist`。`scripts/verify_gazebo_classic_runtime.py` 和自动 smoke test 已这样处理。
