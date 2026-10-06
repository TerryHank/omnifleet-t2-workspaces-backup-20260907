# 104 / 113 两车协同 v0.2.0

当前版本是 **MSC-V1 两车静态部署版**。公共地图已在两车全局规划中静态验证，编队运动在协调器和两端代理中均锁定。本版不能称为“多机协同实车闭环完成”。当前连接、部署版本和剩余动作以最新验收记录为准，不以历史静态结果代替当前在线状态。

- [最新部署结果与验收证据](docs/ACCEPTANCE_V02.md)
- [MSC-01～74 逐项状态](docs/TRACEABILITY_V02.md)
- [v0.1 历史记录](docs/ACCEPTANCE_20260909.md)
- [用户提供的原规范](docs/MSC-V1.md)
- [最近一次确认的叠加图](evidence/after_reference_restart/registration-overlay.png)

## 部署结构

| 主机 | 本地 ROS | 协同组件 |
|---|---|---|
| 192.168.3.104 | Humble / Domain 0 / rmw_zenoh_cpp | robot_104 代理、公共地图接收器、唯一底盘速度入口 |
| 192.168.3.113 | Humble / Domain 73 / rmw_fastrtps_cpp | robot_113 代理、协调器与公共地图发布、唯一底盘速度入口 |

两端本地 ROS 图隔离，代理通过带身份校验的 HTTP 接口交换状态。没有桥接裸 `/cmd_vel`、`/map` 或 Nav2 action，避免同名目标串车。

速度链路：Nav2 → `/msc/nav_cmd_vel` → 代理 → `/msc/cmd_vel_safe` → 底盘。原 `/cmd_vel` 作为人工输入；代理启动时取消旧任务并等待停车确认。协同代理不是纯监视器，底盘现在依赖它提供速度输入。

共享坐标为 `fleet_map`，以 113 的 `map` 为参考。113 将权威底图发布为 `/fleet/map`，104 接收相同栅格、分辨率和原点；两车全局代价地图均使用 `global_frame=fleet_map`、`static_layer.map_topic=/fleet/map`。104 原 MOLA `/map` 留作定位/排查，已不作为全局规划底图；局部地图继续接收本车雷达。两车局部/全局实时障碍代价并不要求逐格相同。

另有 `/msc/peer_map` 标记同伴占用，未标记部分保持未知，避免覆盖原地图未知区。该图是代价图层输入，不是第二个公共底图。重启任一 MOLA 或检测到定位跳变时，旧配准作废，不能直接沿用。没有地图融合或自动重定位保证。

## 手动启动与查看

分别 SSH 到两车：`ssh iecme@192.168.3.104` 或 `ssh iecme@192.168.3.113`。

两车的协同代理已启用开机启动：

```bash
sudo systemctl start omnifleet-msc-agent.service
sudo systemctl status omnifleet-msc-agent.service --no-pager
```

104 还需要地图接收器（已启用开机启动）：

```bash
sudo systemctl start omnifleet-msc-map.service
```

113 协调器：

```bash
sudo systemctl start omnifleet-msc-coordinator.service
cd /home/iecme/msc_v1_ws
source /opt/ros/humble/setup.bash
source /home/iecme/hardware_drivers_ws/install/setup.bash
source /home/iecme/mola_3_2_ws/install/setup.bash
source /home/iecme/omnifleet_t2_ws/install/setup.bash
source /home/iecme/msc_v1_ws/install/setup.bash
set -a
source /etc/omnifleet_t2/robot.env
set +a
ros2 run omnifleet_msc msc status
ros2 run omnifleet_msc msc report
ros2 run omnifleet_msc msc tasks
ros2 topic echo /fleet/status --once
```

两车的本地 MOLA + Nav2 启动入口（需先结束当前同一栈实例，避免重复运行）：

```bash
cd /home/iecme/msc_v1_ws
bash deployment/start_local_stack.sh
```

该脚本按顺序 source Humble、hardware_drivers_ws、mola_3_2_ws、omnifleet_t2_ws、omnifleet_t2_mola_experiments_ws、nav2_height_ws 和可用的 MRPT overlay，最后读取各自 `/etc/omnifleet_t2/robot.env` 并运行：

```bash
ros2 launch omnifleet_t2_mola_experiments mola_nav2_unified.launch.py
```

不要把 113 的 Domain 73 或 Fast DDS 设置抄到 104。104 的 Zenoh Router 基础服务保持运行。重启地图后必须重新采集、配准并核对现场，不要重放历史 `alignment` 命令。

MOLA 已在运行、只需重启 Nav2 时，在对应主机使用：

```bash
# 按主机选择服务名：104 使用 msc-shared-nav-104，113 使用 msc-shared-nav-113
systemctl --user stop msc-shared-nav-104.service
cd /home/iecme/msc_v1_ws
bash deployment/start_navigation.sh
```

`start_navigation.sh` 包含必要 source，最后运行 `ros2 launch omnifleet_t2_mola_experiments nav2_direct.launch.py odom_topic:=/odom obstacle_topic:=/rslidar_points obstacle_clearing:=true`。首次启动共同坐标尚未确认时，全局规划等待 TF 属于预期限制。

## 停止

在 113 已 source 的终端中：

```bash
ros2 run omnifleet_msc msc stop
ros2 run omnifleet_msc msc status
```

`stop_confirmed=true` 才代表已获得新鲜速度、静止位姿、任务取消和命令确认。`stop` 是软件停车，不能替代硬件急停。停车锁会保存在代理上；检查后可用 `ros2 run omnifleet_msc msc release_stop` 解除，它不会解除编队运动锁。

不再保留本次部署服务时，在确认停车后：

```bash
# 113：停止协调器
sudo systemctl stop omnifleet-msc-coordinator.service
# 两车：停止代理；底盘会失去速度源，不能继续正常手动驾驶
sudo systemctl stop omnifleet-msc-agent.service
# 104：停止本次保留的本地应用栈，基础服务继续运行
sudo systemctl stop omnifleet-msc-map.service
systemctl --user stop msc-shared-nav-104.service
systemctl --user stop msc-local-stack-104.service
# 113：只停止本次重新启动的 Nav2
systemctl --user stop msc-shared-nav-113.service
# 113 中途重启后恢复的是完整应用栈，当前使用这个服务名
systemctl --user stop msc-shared-stack-113.service
```

若需恢复旧速度链路，应成套还原驱动订阅、Nav2 remap 与 chassis 的 `msc.conf`，不能只停止代理或只把驱动改回 `/cmd_vel`。备份位于两车 `/home/iecme/robot_backups/msc_v1_20260909/`。先恢复文件、运行 `sudo systemctl daemon-reload`，再重启底盘及应用栈并核对唯一速度源。不要还原整个 `/etc`。

## 配置与任务接口

- `/etc/omnifleet_msc/agent.json`：`enable_control=true`，`allow_fleet_motion=false`。
- 113 `/etc/omnifleet_msc/coordinator.json`：`allow_motion=false`。
- 身份密钥只在权限为 600 的配置中；不应上传 Git 或放入普通报告。
- 跟车间隔默认 1.4 m；最小车间安全包络按两车各 0.39 m 半径加 0.20 m 余量计算。这是软件约束，尚无动态制动距离验收。
- `require_shared_map=true`、`require_obstacles=true`：共同地图和实时障碍信息必须有效。
- 独立任务接口 `POST /v1/tasks` 支持 submit/list/cancel/confirm_stopped，按能力、载重、优先级和路径成本排他认领；当前运动锁只允许排队和取消，不会发车。无额定载重资料时按 0 kg，不能虚构承载能力。
- 为避免抢行，独立任务按优先级依次运动；后续任务可以排队，但不强行抢占正在运动的任务。人工确认失联车辆停止前，不重新分配它的任务。
- `msc tasks --file request.json` 读取 JSON 请求；所有目标必须使用 `frame_id: fleet_map`。`msc pause/cancel/stop` 同样管理独立任务；暂停后的任务用 `msc resume` 显式恢复。
- 纵队为 `column`，横队为 `row`，`line` 当前是纵队别名。三角形未实现，本期排除。

`/fleet/report` 和 `msc report` 提供任务完成率、每车航点完成率、队形误差、最小距离、掉队/失联/恢复时间及最终速度/残留任务。未安装接触检测时碰撞次数为 null，不代表没有碰撞。

## DSH 诊断

113 的 DSH 现在读取公共地图和两车运行状态/参数，并区分面板归属车辆。修复将推理输出误报成答案的计时方式；Qwen3.8 诊断请求使用 `thinking_budget=2048`、`max_tokens=6144`，不修改正常 DSH 会话的默认模型。无完整分组结论的回答不再标记完成。

标准话题仍为 `/omnifleet_t2/diagnostics/question` 和 `/omnifleet_t2/diagnostics/answer`。实际 Foxglove 3.5.0 桥接使用 `foxglove.sdk.v1` 子协议；最新短诊断端到端 WebSocket 回归约 38.7 秒成功，但不能承诺模型服务永不超时。

## 本地回归

在任一已安装工作空间的 ROS 主机上执行以下两个纯软件测试：

```bash
source /opt/ros/humble/setup.bash
source /home/iecme/msc_v1_ws/install/setup.bash
python3 -m unittest discover -s /home/iecme/msc_v1_ws/tests -p test_core.py -v
python3 -m unittest discover -s /home/iecme/msc_v1_ws/tests -p test_traffic.py -v
```

控制权测试必须在隔离域运行；其中的模拟速度仅发送到假底盘，不访问串口：

```bash
env ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  python3 /home/iecme/msc_v1_ws/tests/test_guard_ros.py
```

`final_static_probe.py` 是只读采样；`static_acceptance.py` 会发送软件停车并取消目标，不能在用户正常导航时随意执行。
