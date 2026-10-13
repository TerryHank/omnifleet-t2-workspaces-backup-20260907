# Nav2 两车手动演示

主机：iecme@10.0.0.198。robot_113 为领车，robot_104 为跟车。

- 同伴障碍图发布前，先转换到本车 map，并使用本车全局代价地图的尺寸、分辨率、原点。该修复已应用在两车 MSC agent。
- 动态跟随：Nav2 Humble GoalUpdater → ComputePathToPose → FollowPath；发布领车后方的虚拟目标，避免将领车车体作为规划终点。follow_point 原模板的 TruncatePath 不再需要。
- 固定航点：手动创建 Nav2 Waypoint Follower，并通过 FollowWaypoints 依次调用本车 navigate_to_pose。两车都经过现有 local_navigation 和 sole chassis output。
- 本入口属于手动测试工具，不添加 systemd 服务，不修改 Foxglove/MSC 原任务后端；编队运动锁保持关闭。测试使用 LOCAL 导航任务，检查两车定位、间距和实际速度；它不验证 MSC 租约、任务分配或道路预约。
- 测试期间临时使用 MPPI vx_max=0.025、vx_min=-0.025、wz_max=0.08，xy_goal_tolerance=0.08。每次运行先保存原值，退出后恢复。不能仅依靠 SpeedLimit，因为新控制任务初始化期间出现过瞬时速度超过预期。
- 启动等待定位稳定，实际行驶时定位年龄必须小于 0.8 秒，两车中心间距必须至少 1.1 米；不满足则取消两车任务。动态行为树还设置 90 秒上限。测试现场需继续具备两车急停条件。

## 固定航点预检（不发车）

```bash
source ~/omnifleet_ws/env.bash
python3 ~/omnifleet_ws/src/system/shared_map_msc/tools/nav2_modes_demo.py \
  --mode fixed \
  --routes ~/omnifleet_data/recordings/bags/nav2_modes_20261009_1720/fixed-routes.json \
  --output "$HOME/omnifleet_data/recordings/bags/fixed_check_$(date +%Y%m%d_%H%M%S)"
```

现场条件确认后，在该命令中加入 `--execute` 才会发车并自动录包。固定路线 JSON 使用 fleet_map 坐标；这份文件对应本次验收的位置和配准，移动车辆或重载地图后需要重新检查/修改路线。

## 动态跟随预检

同样使用脚本，改为 `--mode leader --routes <领车路线.json>`，先不传 `--execute`。路线 JSON 格式如下；坐标必须根据当前共享地图填写：

```json
{"leader":"robot_113","follower":"robot_104","spacing":1.5,"routes":{"robot_113":[[0.0,0.0,1.57]],"robot_104":[]}}
```

原验收 leader-routes3.json 的终点已经到达，不应直接用于下一次运动演示。必须设置新的、经过预检的短路线。

## 2026-10-09 验收

- leader-test8：领车约移动 0.49 m，跟车约移动 0.68 m。定位估计的中心间距 1.59～1.89 m，最后约 1.59 m，设定为 1.5 m。
- fixed-test2：两车各两个航点，两个 FollowWaypoints 结果均为 SUCCEEDED，missed_waypoints 为空。最小中心间距约 1.51 m。
- 两车结束时速度为零、无活动/待接受导航目标；控制器原值 vx_max=0.4、vx_min=-0.35、wz_max=1.2、xy_goal_tolerance=0.25 已通过参数服务复核。
- 协同状态定位年龄最高约 682 ms，仍需排查延迟；未验收长距离、急转弯或长时间连续负载。
- 总报告：~/omnifleet_data/recordings/bags/nav2_modes_20261009_1720/acceptance-report.json
- 成功录包：上述目录下 leader-test8/bag、fixed-test2/bag。
- 源码备份：两车 /var/tmp/omnifleet_nav2_modes_fix_20261009_1720/agent.before.py。
