# MOLA 功能与参数迁移对照

状态：本地源码迁移候选；Linux 构建、回放和机器人验收尚未执行。旧源码保留在 COLCON_IGNORE 隔离目录，禁止据此删除。

## 包覆盖

| 包 | 旧版 | 新版基准 | 处理 | 旧独有剩余文件 |
|---|---|---|---|---|
| kitti_metrics_eval | 3.0.0 | 3.0.0 | 之前字节相同，继续共享 | 0 |
| mola | 3.2.0 | 3.2.1 | 新版实现优先 | 0 |
| mola_academic_datasets | 3.0.0 | 3.0.0 | 之前字节相同，继续共享 | 0 |
| mola_bridge_ros2 | 3.2.0 | 3.2.1 | 新版实现，补齐旧接口 | 0 |
| mola_common | 0.6.1 | 0.6.1 | 之前字节相同，继续共享 | 0 |
| mola_demos | 3.2.0 | 3.2.1 | 新版实现优先 | 0 |
| mola_georeferencing | 2.4.2 | 2.4.2 | 之前字节相同，继续共享 | 0 |
| mola_gtsam_factors | 2.4.2 | 2.4.2 | 新版实现优先 | 0 |
| mola_imu_preintegration | 1.17.1 | 2.0.0 | 新版实现优先 | 0 |
| mola_input_euroc_dataset | 3.0.0 | 3.0.0 | 之前字节相同，继续共享 | 0 |
| mola_input_kitti360_dataset | 3.0.0 | 3.0.0 | 之前字节相同，继续共享 | 0 |
| mola_input_kitti_dataset | 3.0.0 | 3.0.0 | 之前字节相同，继续共享 | 0 |
| mola_input_lidar_bin_dataset | 3.2.0 | 3.2.1 | 新版实现优先 | 0 |
| mola_input_mulran_dataset | 3.0.0 | 3.0.0 | 之前字节相同，继续共享 | 0 |
| mola_input_paris_luco_dataset | 3.0.0 | 3.0.0 | 之前字节相同，继续共享 | 0 |
| mola_input_rawlog | 3.2.0 | 3.2.1 | 新版实现优先 | 0 |
| mola_input_rosbag2 | 3.2.0 | 3.2.1 | 新版实现优先 | 0 |
| mola_input_video | 3.2.0 | 3.2.1 | 新版实现优先 | 0 |
| mola_kernel | 3.2.0 | 3.2.1 | 新版实现，补齐旧接口 | 0 |
| mola_launcher | 3.2.0 | 3.2.1 | 新版实现优先 | 0 |
| mola_lidar_odometry | 3.2.0 | 3.2.0 | 新版实现优先 | 0 |
| mola_metric_maps | 3.2.0 | 3.2.1 | 新版实现优先 | 0 |
| mola_msgs | 3.2.0 | 3.2.1 | 新版实现优先 | 0 |
| mola_pose_list | 3.2.0 | 3.2.1 | 新版实现优先 | 0 |
| mola_relocalization | 3.2.0 | 3.2.1 | 新版实现优先 | 0 |
| mola_sm_loop_closure | 无 | 1.2.2 | 新版新增包保留 | 0 |
| mola_state_estimation | 2.4.2 | 2.4.2 | 之前字节相同，继续共享 | 0 |
| mola_state_estimation_simple | 2.4.2 | 2.4.2 | 之前字节相同，继续共享 | 0 |
| mola_state_estimation_smoother | 2.4.2 | 2.4.2 | 新版实现，补齐旧接口 | 10 |
| mola_test_datasets | 无 | 0.5.0 | 新版新增包保留 | 0 |
| mola_traj_tools | 3.2.0 | 3.2.1 | 新版实现优先 | 0 |
| mola_viz | 3.2.0 | 3.2.1 | 新版实现优先 | 0 |
| mola_viz_imgui | 3.2.0 | 3.2.1 | 新版实现优先 | 0 |
| mola_yaml | 3.2.0 | 3.2.1 | 新版实现优先 | 0 |
| mp2p_icp | 2.13.1 | 2.14.1 | 新版实现优先 | 0 |
| mp2p_icp_core | 2.13.1 | 2.14.1 | 新版实现优先 | 0 |
| mp2p_icp_viz | 2.13.1 | 2.14.1 | 新版实现优先 | 0 |

## 功能验收矩阵

| 功能 | 候选处理 | 验收证据或门槛 |
|---|---|---|
| 估计速度及 6×6 协方差 | LocalizationUpdate 可选字段；smoother publish_twist 默认 false，实验 smoother_direct 显式开启 | test-publish-map-to-odom + test-odometry-twist；C++ 待构建 |
| 直接 map→odom | 从估计器图变量发布；源帧与输出 child 分离；默认关闭 | 禁用/启用/child 覆盖/不存在源帧测试；回放确认唯一 TF owner |
| 新鲜轮速优先 | 时间有效时速度和协方差整体采用轮速；过期/提前超过 0.1s 回退估计；拒绝缓存时间倒退 | 新 C++ 测试覆盖零值、估计、轮速、0.5s/-0.1s 边界和过期回退 |
| IMU 采集时间 | 明确使用输入 header.stamp，不依赖 MRPT 转换版本 | Linux ROS 消息回归待执行 |
| 新版地图输出 | 保留独立 map executor、流式点云 SensorDataQoS 和导航全程 deskew | 回放/静止传感器检查地图、点云及旧地图 mm 读取 |
| 生产及四个实验 profile | simple_direct / simple_direct_observation / simple_rep105 / smoother_direct 保留；规范加载 latest 环境 | Python 语法/配置合同检查；Linux 启动和图验收待执行 |
| ROS namespace/TF | 保留新版 robot_113 输出重映射与 fleet_scope；旧未命名空间的测试预期已更新 | ROS_ARGS 与 launch SetRemap 的最终优先级、订阅连接和 TF owner 必须在回放验证 |
| 数据集/地图导入导出/GNSS/GUI工具 | 35 个旧包全部有新版对应，保留新版新包；脚本和服务采用新版 | Linux upstream tests、CLI help、地图保存/重载和导出待执行；未证明 GUI/全部数据集运行 |
| 诊断及线程处理 | 新版 CallbackTrace、地图线程和安全退出优先；不恢复旧绝对路径 causal_trace 头文件或默认 crash 前缀 | 退出/线程回归待执行 |
| 旧测试 | gtsam 旧 IMU helper 和有效 map→odom 测试已注册；其余 10 个未注册历史测试留在隔离区 | 不以测试文件存在推断 async_backend 等未实现 API；避免人为新增历史未实现功能 |

## 参数链

| 参数 | 环境变量 | 默认 | 消费端 |
|---|---|---|---|
| publish_twist | MOLA_STATE_ESTIMATOR_PUBLISH_TWIST | false | smoother Parameters → spinOnce → LocalizationUpdate → bridge |
| publish_map_to_odom_tf | MOLA_PUBLISH_MAP_TO_ODOM_TF | false | smoother Parameters → publishMapToOdom → bridge source routing |
| map_to_odom_frame_name | MOLA_MAP_TO_ODOM_FRAME | 空 | smoother 源 odometry frame |
| map_to_odom_child_frame | MOLA_MAP_TO_ODOM_CHILD_FRAME | 空，回落到源帧 | smoother 输出 child frame |
| publish_twist_from_latest_odometry | MOLA_PUBLISH_TWIST_FROM_LATEST_ODOM | false | bridge callback cache → OdometryTwist |
| latest_odom_twist_source | MOLA_LATEST_ODOM_TWIST_SOURCE | wheel_odom | bridge 按 sensor label 选择源 |
| latest_odom_twist_max_age | MOLA_LATEST_ODOM_TWIST_MAX_AGE | 0.5 秒 | bridge 按 ROS clock 判定年龄 |

冲突参数和既有算法采用新版。参数差异完整清单见 PARAMETER_DIFF.tsv；文件差异见 SOURCE_DIFF_INVENTORY.tsv。
物理标定、IMU 量纲及当前新版性能调节参数保持现有值，不把旧默认值重新覆盖到新版。
