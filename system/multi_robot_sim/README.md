# OmniFleet 阿克曼车模与底盘驱动 Agent 交付包

本包来自远端生产工作空间 `/home/iecme/ackermann_mid360_glim_ws`，目标是：

1. 保留当前实车正在使用的 URDF、完整描述资源和 STM32 底盘驱动源码；
2. 解压后默认使用**不访问串口**的仿真驱动，能够直接接收 `/cmd_vel`；
3. 以 Gazebo Classic 11 为主入口，同时提供纯 ROS 运动学和 Gazebo Harmonic 兼容入口。

当前交付版本为 **1.1.1**；版本修复、布局加载和验证摘要见
[DELIVERY_V1.1.1](docs/DELIVERY_V1.1.1.md)。

## 交付内容

```text
ros2_ws/src/
├── omnifleet_description/       # 远端完整模型包，含生产 lite URDF 与全部 CAD STL
├── omnifleet_kinematic_sim/     # 新增：无 Gazebo、无串口的 Ackermann 仿真驱动
├── omnifleet_simulator/         # Gazebo Classic 主入口及 Harmonic 兼容入口
└── omnifleet_bringup/           # 远端实车驱动快照，默认 COLCON_IGNORE
hardware_samples/                # 远端 systemd / udev / 环境文件只读样例
scripts/                         # 安装、构建、运行和自动验收脚本
docs/                            # 接口、来源和边界说明
```

生产模型真值为：

```text
omnifleet_description/urdf/lunshi_ackermann_lite.urdf
```

其 `base_link` 位于后轴中心，关键尺寸为：轴距 `0.362295943 m`、前轮距
`0.264956799 m`、后轮距 `0.245400019 m`、轮半径 `0.058528263 m`、最大前轮
转角 `±0.523599 rad`。

## 方案 A：Gazebo Classic 11 物理仿真（默认）

目标环境：Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11。

```bash
cd omnifleet_ackermann_agent_delivery_20260823_162133
source /opt/ros/humble/setup.bash
bash scripts/install_dependencies.sh
bash scripts/build.sh
source ros2_ws/install/setup.bash
ros2 launch omnifleet_simulator gazebo_classic.launch.py
```

该命令默认打开 Gazebo GUI、关闭 RViz，并使用简易模型：

```text
omnifleet_description/urdf/lunshi_ackermann_lite_gazebo_classic.urdf.xacro
```

它加载官方 `libgazebo_ros_ackermann_drive.so`，直接提供 `/cmd_vel`、`/odom`、
`odom→base_link` TF 和 `/joint_states`。

另开终端测试运动：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.10}}" -r 10
```

自动验收：

```bash
bash scripts/smoke_test_gazebo_classic.sh
```

## 多机协同

三车 Gazebo Classic 演示固定使用 `robot1`、`robot2`、`robot3`，每车具有独立的
`cmd_vel`、`odom`、`joint_states`、`robot_description` 和唯一 TF frame。默认同时打开
Gazebo GUI、RViz、Foxglove（端口 `8796`），并在三车里程计就绪后沿三条开放通道执行
一次自动演示：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
bash scripts/run_multi_robot.sh
```

可通过 RViz 的 2D Goal Pose 向 `/fleet/goal` 发布目标；控制器会把目标分配给最近的在线
空闲车辆。无界面自动验收使用隔离的 ROS domain 与 Gazebo master，并把证据写入
`reports/multi_robot/`：

```bash
bash scripts/smoke_test_multi_robot.sh
```

这是原生轻量点目标控制，不是 Nav2。UWB/RTK 未参与本演示；当前安全能力仅来自车间距
停车/降速和预设开放通道。场景中加入任意障碍后，需要后续接入 Nav2 的定位、规划和避障
能力，不能把本演示控制器作为通用障碍环境导航器。

同一车辆连续多航点使用 `nav_msgs/msg/Path`。例如把三点路线直接派给 `robot1`：

```bash
ros2 topic pub --once /robot1/waypoints nav_msgs/msg/Path \
  "{header: {frame_id: world}, poses: [{pose: {position: {x: -0.5}, orientation: {w: 1.0}}}, {pose: {position: {x: 0.5}, orientation: {w: 1.0}}}, {pose: {position: {x: 1.5}, orientation: {w: 1.0}}}]}"
```

`/fleet/waypoints` 会按第一点选择最近在线空闲车，整条路线固定由该车顺序执行。这仍是
开放场景的轻量连续点控制，不提供 Nav2 障碍规划。

完整的 topic、服务、RViz、Foxglove 和隔离运行说明见
[多机协同操作指南](docs/MULTI_ROBOT.md)。可直接加载根目录中的
[RViz 布局](layouts/omnifleet_multi_robot.rviz) 与
[Foxglove 布局](layouts/omnifleet_multi_robot_foxglove.json)；加载方法见
[布局说明](layouts/README.md)。

没有安装 Gazebo 的 Ubuntu 22.04 远端可使用新增的三车运动学入口；它不访问串口，
并可在独立 ROS 域通过 Foxglove 执行多点路线：

```bash
export ROS_DOMAIN_ID=97
bash scripts/run_multi_robot_kinematic.sh \
  rviz:=false foxglove:=true foxglove_port:=8796 auto_demo:=false
```

远端启动、Foxglove 连接与多点消息示例见
[远端运动学多车 Foxglove 指南](docs/REMOTE_KINEMATIC_FOXGLOVE.md)。

## 方案 B：纯 ROS 运动学仿真（无 Gazebo 回退）

适用于 ROS 2 Humble / Jazzy，不需要 Gazebo，也不会访问 `/dev/myserial`。

```bash
cd omnifleet_ackermann_agent_delivery_20260823_162133
source /opt/ros/$ROS_DISTRO/setup.bash
bash scripts/install_dependencies.sh
bash scripts/build.sh
bash scripts/run_kinematic_sim.sh use_rviz:=true
```

另开终端测试运动：

```bash
cd omnifleet_ackermann_agent_delivery_20260823_162133
source /opt/ros/$ROS_DISTRO/setup.bash
source ros2_ws/install/setup.bash

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.10}}" -r 10
```

自动验收：

```bash
bash scripts/smoke_test_kinematic.sh
```

## 方案 C：Gazebo Harmonic 兼容入口

推荐环境为 Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic。安装 `ros_gz` 后：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
bash scripts/build.sh
bash scripts/run_gazebo_harmonic.sh headless:=false use_rviz:=false
```

默认生成与实车 frame/尺寸一致的 primitive 模型：

```text
omnifleet_description/urdf/lunshi_ackermann_lite_gazebo.urdf.xacro
```

Harmonic 端接入 `gz::sim::systems::AckermannSteering`，并桥接：

- ROS→Gazebo：`/cmd_vel`
- Gazebo→ROS：`/odom`、`/tf`、`/joint_states`、`/clock`

## WSL2 NVIDIA GPU 传感器仿真（Humble + Fortress）

Ubuntu 22.04 / ROS 2 Humble 自带的 Gazebo Fortress 使用专用 Ogre 1 入口，避开
WSLg D3D12 上 Ogre 2 的纹理复制限制：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch omnifleet_simulator agent_gpu_sim.launch.py
```

验证 GPU renderer 和传感器数据：

```bash
glxinfo -B | grep -E "direct rendering|Accelerated|OpenGL renderer"
ros2 topic echo /scan --once
ros2 topic hz /camera/color/image_raw
nvidia-smi
```

Gazebo 物理求解仍使用 CPU；RPLIDAR S3 `gpu_lidar`、RGB/深度相机和场景渲染使用 GPU。
该入口默认打开 Gazebo GUI，同时保持 `use_rviz:=false`。无图形界面运行时使用：

```bash
ros2 launch omnifleet_simulator agent_gpu_sim.launch.py headless:=true
```

仍可在命令行覆盖 `model`、`world`、`headless`、`render_engine` 和 `use_rviz`。

## 实车驱动边界

实车文件位于 `ros2_ws/src/omnifleet_bringup`，其中核心是：

```text
omnifleet_bringup/Ackman_driver_R2.py
omnifleet_bringup/Rosmaster_Lib.py
scripts/ackermann_driver
launch/chassis_core.launch.py
```

该包含有 `COLCON_IGNORE`，默认不会参与仿真构建。请勿在仿真机上删除该文件并启动
`chassis_core.launch.py`，否则驱动会尝试打开 `/dev/myserial`。

## 重要说明

- 仿真中 `angular.z` 使用标准 ROS 偏航角速度符号，不加入任何实车硬件专用反号。
- 车速为零时，只允许舵轮转向，不模拟阿克曼车原地旋转。
- 纯 ROS 运动学驱动保留 `0.35 s` 命令看门狗和 `/rrc_safety/zero_lock`；Gazebo Classic 停车需发送显式零命令。
- 完整 CAD Xacro 保留用于外观/旧 S3+HIK 传感器展示；生产 frame 真值仍以 lite URDF 为准。
- 不包含远端 `build/`、`install/`、`log/`、缓存或设备文件。

更多信息见 [AGENT_HANDOFF.md](AGENT_HANDOFF.md) 和 `docs/`。
