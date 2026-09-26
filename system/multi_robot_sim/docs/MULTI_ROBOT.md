# 多机协同操作指南

`omnifleet_multi_robot_sim` 提供三车 Gazebo Classic 场景、轻量车队控制器、RViz
配置、Foxglove bridge 和运行验证器。它固定管理 `robot1`、`robot2`、`robot3`。

## 启动

默认启动会打开 Gazebo GUI、RViz、Foxglove bridge（`ws://localhost:8796`），并在
三车里程计就绪后执行一次三通道自动演示：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
bash scripts/run_multi_robot.sh
```

操作员手动派单时关闭自动演示：

```bash
bash scripts/run_multi_robot.sh auto_demo:=false
```

操作界面可加载根目录的
[RViz 布局](../layouts/omnifleet_multi_robot.rviz) 或
[Foxglove 布局](../layouts/omnifleet_multi_robot_foxglove.json)，具体导入步骤和接口表见
[布局说明](../layouts/README.md)。

每车提供独立的 `/robotN/cmd_vel`、`/robotN/odom`、`/robotN/joint_states`、
`/robotN/robot_description` 和 `robotN/odom`、`robotN/base_link` TF。车队接口为：

- `/fleet/goal`、`/goal_pose`：`geometry_msgs/msg/PoseStamped`，分配给最近的在线空闲车；
- `/robotN/goal_pose`：直接派给指定车辆；
- `/fleet/waypoints`：`nav_msgs/msg/Path`，按第一点选最近空闲车，整条路线固定该车；
- `/robotN/waypoints`：把整条 `Path` 固定派给指定车辆；
- `/fleet/status`：transient-local `std_msgs/msg/String` JSON 状态；
- `/fleet/events`：派单、完成、上下线等 JSON 事件；
- `/fleet/cancel_all`：`std_srvs/srv/Trigger`，取消活动任务和排队任务；
- `/fleet/clear_queue`：`std_srvs/srv/Trigger`，只清空等待队列。

状态 JSON 包含每车的 `online`、`state`、`pose`、`task`、`goal`、`route_id`、
`waypoint_index`、`waypoint_count`，以及车队级
`queue`、`completed_count`、`min_pair_distance`。

## 发送目标

RViz 的 **2D Goal Pose** 工具已经配置为向 `/fleet/goal` 发布目标。

Foxglove Studio 中连接 `ws://localhost:8796`，添加 **Publish** 面板，选择 topic
`/fleet/goal` 和 schema `geometry_msgs/msg/PoseStamped`，发布例如：

```json
{
  "header": {"frame_id": "world"},
  "pose": {
    "position": {"x": 0.5, "y": 0.0, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  }
}
```

命令行车队派单：

```bash
ros2 topic pub --once /fleet/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.5, y: 0.0}, orientation: {w: 1.0}}}"
```

直接派给 `robot2`：

```bash
ros2 topic pub --once /robot2/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.5, y: 1.2}, orientation: {w: 1.0}}}"
```

同一车辆连续执行任意点数路线（下面是三点格式示例）：

```bash
ros2 topic pub --once /robot1/waypoints nav_msgs/msg/Path \
  "{header: {frame_id: world}, poses: [{pose: {position: {x: -0.5}, orientation: {w: 1.0}}}, {pose: {position: {x: 0.5}, orientation: {w: 1.0}}}, {pose: {position: {x: 1.5}, orientation: {w: 1.0}}}]}"
```

使用 `/fleet/waypoints` 时，控制器只用第一点选择最近的在线空闲车，后续点不会被其他
车辆抢走。`poses` 数组有 N 个元素就会执行 N 个点，没有三点上限；空 `Path` 会被拒绝
并发布 `/fleet/events` 事件，单点 `Path` 合法。

Foxglove 的 **Publish** 面板可选择 `/robot1/waypoints` 或 `/fleet/waypoints`，schema
选择 `nav_msgs/msg/Path`，发布：

```json
{
  "header": {"frame_id": "world"},
  "poses": [
    {"header": {"frame_id": "world"}, "pose": {"position": {"x": -0.5, "y": 0.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}},
    {"header": {"frame_id": "world"}, "pose": {"position": {"x": 0.5, "y": 0.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}},
    {"header": {"frame_id": "world"}, "pose": {"position": {"x": 1.5, "y": 0.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}
  ]
}
```

查看状态与调用服务：

```bash
ros2 topic echo /fleet/status --once
ros2 service call /fleet/clear_queue std_srvs/srv/Trigger '{}'
ros2 service call /fleet/cancel_all std_srvs/srv/Trigger '{}'
```

## 隔离运行

与其他 ROS/Gazebo 实例并行时必须使用独立 domain 和 master，例如：

```bash
export ROS_DOMAIN_ID=97
export GAZEBO_MASTER_URI=http://127.0.0.1:11397
bash scripts/run_multi_robot.sh auto_demo:=false
```

不要把不同 `ROS_DOMAIN_ID` 或 `GAZEBO_MASTER_URI` 的进程混用，也不要用
`killall`/宽泛 `pkill` 清理其他实例。

## 能力边界

本控制器是开放场景中的原生轻量点目标/连续航点控制，不是 Nav2，也不做障碍规划。
UWB/RTK 未参与控制；当前安全能力来自
车间距停车/降速和预设开放通道。任意障碍环境需要后续接入定位、规划和避障，不能直接
沿用本演示作为通用导航方案。
