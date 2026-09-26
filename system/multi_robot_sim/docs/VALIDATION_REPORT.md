# 验证报告

日期：2026-08-23

## ROS 2 Humble + Gazebo Classic 11

环境：WSL Ubuntu 22.04、ROS 2 Humble、gazebo11、gazebo_ros_pkgs 3.9.0。

- `colcon build`：`omnifleet_description`、`omnifleet_kinematic_sim`、`omnifleet_simulator` 三包通过。
- Gazebo Classic headless 真实启动通过。
- `lunshi_ackermann_lite_gazebo_classic.urdf.xacro` 成功生成并加载。
- `/cmd_vel = (0.15 m/s, 0.10 rad)` 连续两轮测试：模型位移分别为 `0.907 m`、`0.929 m`。
- `/joint_states`：6 个可动关节。
- `/odom` 和 `odom→base_link`：持续发布。
- 显式零命令后：停车通过。

自动命令：

```bash
bash scripts/smoke_test_gazebo_classic.sh
```

## 纯 ROS 运动学后端

- 单元测试：`6 passed`。
- 运行时位移：`0.300 m`。
- 关节状态：6 个。
- 停止发布命令后：0.35 秒看门狗使 `/vel_raw` 与 `/motor_command_sent` 归零。

自动命令：

```bash
bash scripts/smoke_test_kinematic.sh
```

## ROS 2 Jazzy 构建

环境：WSL Ubuntu 24.04、ROS 2 Jazzy、Gazebo Sim 8 / Harmonic、ros_gz 1.0.22。

- 三包 `colcon build --symlink-install` 通过。
- Harmonic 文件与桥接配置完成语法及构建验证。

Gazebo Classic 是本次按用户要求的主验收项。
