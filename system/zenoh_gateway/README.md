# 113 / 104 Zenoh 与双模式导航部署

目标主机：`iecme@192.168.3.113`、`iecme@192.168.3.104`。用户明确要求本轮仅进行不运动验收。

本轮仅验收软件部署、真实传感器输入、静态规划和通信故障恢复，不执行实车运动。

## 已部署结构

- 两车 ROS 2 Humble 节点统一使用 `rmw_zenoh_cpp`，`ROS_DOMAIN_ID=0`。
- `/robot_113/...`、`/robot_104/...` 使用相同相对接口，传感器、控制、action、服务互不串车。
- TF frame 也有车辆前缀；本地 TF 话题分别为 `/robot_113/tf`、`/robot_104/tf`。
- 104 的模型根节点已与 113 对齐为 base_link；移除重复的裸名雷达静态变换。网关合并同一 TF 话题的多发布者变换，保留每条边原始时间戳，避免只转发最后一条而丢失 TF 链。
- 每台车具有自己的雷达、定位、地图、Nav2、速度安全代理和仅监听回环地址的本地 Zenoh 路由。跨车连接不是本地导航启动条件。
- 跨机网关使用独立进程中的原生 `rmw_zenoh_cpp` / Domain 0，通过无线地址上的 TCP 7600 互通。网关同步对端业务话题并代理服务与 Action；底盘安全输出和 Nav2 bond 不通过跨机链路控制。本地与网络进程仅通过回环 HTTP 7601/7602 交换 CDR 数据，网络故障不会阻塞本车 ROS 会话。
- 跨机点云、图像最多同步 2 Hz，其他遥测最多 10 Hz；本车原始话题频率不变。消息保留原始时间戳和车辆 frame。网络状态不能用过期地图或最后一条遥测冒充在线。
- 协同状态、任务和共享地图接口继续使用 `/fleet/...`。现有协调器保留在 113；104 的本地导航不再依赖协调器或 `/fleet/map`。
- `pure_mola`：MOLA 只订阅激光数据，不接入 IMU、轮式里程计；`map → base_link`。Nav2 使用由激光位姿差分得到的 `lidar_odometry/nav_odom`。
- `rep105`：`map → odom → base_link`；MOLA 使用激光、校正后的雷达 IMU 和底盘里程计；Nav2 局部坐标系使用各车 `odom`。
- 104 的单点导航 launch、规划器配置和参数面板以 113 为基线。104 的 MOLA 内核、桥接、状态估计和激光定位模块已按相同头文件重新编译，修复定位质量值误读为速度的 ABI 不一致。
- 两车显示布局、语义点位面板、Nav2 参数服务均按本车前缀适配。104 不支持的 STM32 PWM 参数独立禁用，不阻塞导航参数读取；没有烧录固件。
- 就绪检查进程与 Nav2 同生命周期运行，避免探针关闭会话失败阻断 Nav2 启动。网关注册串行化，按 Goal UUID 去重，防止重复转发同一 Action 目标。

## 已取得的最终证据

| 项目 | 实测证据 |
|---|---|
| 断网期间本车继续运行 | `gateway-final-continuity.json`：两车各 750 条样本，稳定阶段 735/735 全部 Nav2 就绪；最大间隔 113 为 0.2145 s、104 为 0.2235 s |
| 故障覆盖 | 上述 150 s 观测包含 90 s 双向网络隔离；104 协调器断联年龄达到 91.15 s，本地导航仍就绪 |
| 控制与反馈 | 观测期间两车实际里程计速度为零；规划测试采集的安全控制输出也全部为零 |
| 重连后双向规划 | 两车的 `final-gateway-reconnected.json` 均对 113 和 104 完成真实 `ComputePathToPose`，状态 4、路径 7 个点 |
| REP105 双向规划 | 两车 `final-rep105-cross.json` 均通过；全局坐标系为本车 map，局部坐标系为本车 odom |
| REP105 离线冷启动 | 两车 `final-offline-rep105-start.json`：网络隔离期间分别重新启动 Nav2，四个管理节点均 active，规划状态 4 |
| 参数服务 | 113 读取 89 项；104 读取 87 项并明确 `stm32_pwm=false` |

最终 RMW 为源码构建的 0.1.9，Zenoh 1.8 与配套头文件、C++ 绑定一致。运行库包含[上游候选修复 PR 2709](https://github.com/eclipse-zenoh/zenoh/pull/2709)的回移，以及取得队列锁后的关闭状态复检。该 PR 不是已合并的官方发行修复；本项目保存源码差异、哈希和回退文件。

最终使用 `/home/iecme/omnifleet_fleet/zenoh-patched-v2/lib/libzenohc.so`，SHA-256：`cea828745de6bc41c51563e221276fad3d3251bca78bae3dbb78d52382f49952`。ROS RMW overlay 为 `/home/iecme/fleet_rmw_ws/install`。

## 手动启动

在需要操作的车上执行以下命令。切换模式前先用 Ctrl+C 停止之前手动启动的导航。

```bash
source /home/iecme/omnifleet_fleet/env.bash
cd /home/iecme/workspace/omnifleet_t2_mola_experiments_ws

# 纯激光模式，使用当前环境创建本地地图；引号中的空格不可省略。
bash /home/iecme/omnifleet_fleet/navigation.sh pure_mola "map_path:= " map_dir:=/home/iecme/maps
```

切换到 REP105：

```bash
source /home/iecme/omnifleet_fleet/env.bash
cd /home/iecme/workspace/omnifleet_t2_mola_experiments_ws
bash /home/iecme/omnifleet_fleet/navigation.sh rep105 "map_path:= " map_dir:=/home/iecme/maps
```

环境脚本按 Humble → hardware_drivers_ws → mola_3_2_ws → omnifleet_t2_ws 加载，再加载本项目的实验、协同与 Nav2 高度修复 overlay。

不带空白 `map_path` 时，统一入口默认加载 `/home/iecme/maps/foxglove_map.mm`。已有地图定位需要正确的初始位姿；`initial_pose` 参数格式为 `[x,y,z,yaw_deg,pitch_deg,roll_deg]`。不要把两车都随意设为同一个零位姿。共享地图文件相同不等于两车的现场初始定位已验证。

只读检查：

```bash
source /home/iecme/omnifleet_fleet/env.bash
printf 'robot=%s domain=%s rmw=%s\n' "$OMNIFLEET_ROBOT_ID" "$ROS_DOMAIN_ID" "$RMW_IMPLEMENTATION"
ros2 topic list
ros2 topic echo "/$OMNIFLEET_ROBOT_ID/msc/local_status" std_msgs/msg/String --once
ros2 action list
ros2 run tf2_ros tf2_echo "$OMNIFLEET_ROBOT_ID/map" "$OMNIFLEET_ROBOT_ID/base_link" --ros-args -r /tf:="/$OMNIFLEET_ROBOT_ID/tf" -r /tf_static:="/$OMNIFLEET_ROBOT_ID/tf_static"

# 113 上查看现有协同任务和成员状态：
ros2 run omnifleet_msc msc status
```

## 关键接口

| 功能 | 相对话题或 action |
|---|---|
| 原始激光 | `rslidar_points` |
| 底盘里程计 | `odom` |
| MOLA 位姿 | `lidar_odometry/pose` |
| 纯激光速度反馈 | `lidar_odometry/nav_odom` |
| 本地地图 | `map` |
| 代价地图 | `global_costmap/costmap`、`local_costmap/costmap` |
| 单点导航 | `navigate_to_pose` |
| 只规划不运动 | `compute_path_to_pose` |
| 本地人工控制输入 | `cmd_vel` |
| Nav2 控制输入 | `msc/nav_cmd_vel` |
| 唯一底盘输出 | `msc/cmd_vel_safe` |
| 本车状态 | `msc/local_status` |

上述路径必须加 `/robot_113/` 或 `/robot_104/` 前缀。113 的诊断助手等附加话题不要求在 104 上复制。

## 恢复点与验证边界

两车修改前恢复点均在 `/home/iecme/robot_backups/fleet_zenoh_20260910/`。`before.tgz` 保存原配置与相关源码；`frame-before/`、`ui-files-before/` 保存后续专门修改的前态。104 还有 `mola-abi-before.tgz`。

本轮测试只调用只读服务和 `ComputePathToPose`，不发送 `NavigateToPose` 运动目标；控制输出采样必须保持零。控制决策单元测试使用模拟发布器，没有初始化 ROS 通信。

现场实际到点、交叉避让、编队运动、共享地图初始位姿精度不属于用户授权的本轮不运动验收，不能由静态规划成功代替。

中间失败记录保留用于解释修复过程；不能把早期通过结果当成后续配置的最终通过证据。
