# 多机操作布局

## RViz

启动 RViz 后选择 **File > Open Config**，加载
`layouts/omnifleet_multi_robot.rviz`。布局以 `world` 为 Fixed Frame，显示三车模型、TF、
里程计轨迹，并将 **2D Goal Pose** 发布到 `/fleet/goal`。

## Foxglove

先运行多机仿真并在 Foxglove 中连接 `ws://localhost:8796`，然后导入
`layouts/omnifleet_multi_robot_foxglove.json`。布局提供三车 3D/里程计视图、状态与事件、
单目标、任意点数路线以及两个车队服务面板。

| 面板 | 当前接口 | 类型 |
| --- | --- | --- |
| 车队状态 | `/fleet/status` | `std_msgs/msg/String` |
| 车队事件 | `/fleet/events` | `std_msgs/msg/String` |
| 最近空闲车目标 | `/fleet/goal` | `geometry_msgs/msg/PoseStamped`，`frame_id=world` |
| robot1 任意点数路线 | `/robot1/waypoints` | `nav_msgs/msg/Path`，`poses` 数组有几个就执行几个 |
| 取消全部任务 | `/fleet/cancel_all` | `std_srvs/srv/Trigger`，请求 `{}` |
| 清空等待队列 | `/fleet/clear_queue` | `std_srvs/srv/Trigger`，请求 `{}` |

该布局对应开放场景轻量控制器，不包含 Nav2 障碍规划。

路线面板中的预置项只是可编辑示例，不是上限。复制或删除完整的 `poses` 数组元素即可
设置任意正整数个航点；空数组会被控制器拒绝。
