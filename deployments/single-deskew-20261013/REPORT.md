# 单次去畸变修改结果

日期：2026-10-13。主机：`iecme@192.168.3.113`（`iecme-tank`）。

已完成当前生产 GICP 链的源码修改、独立构建、回归验证及安装。运行中的 MOLA 服务保留原进程和地图会话；本次没有重启服务，新版本在下次进程加载时生效。

## 修改后的数据流

```text
驱动输出 → raw → 一次 FilterDeskew → deskewed
                                  ├→ 导航输出 worker
                                  └→ 建图降采样 → ICP降采样 → GICP
```

- `pipelines/lidar3d-gicp.yaml`：从共享去畸变流水线移除 `FilterByRange`、`raw_range_filtered` 中间层及清理；单一 `FilterDeskew` 从 `raw` 生成 `deskewed`；删除独立 `observations_navigation_deskew_pass`。
- `module/src/LidarOdometry_ProcessScan.cpp`：导航和定位共用一次 `pc_deskew` 调用；导航任务持有共享结果的指针，定位降采样生成独立输出层；不为仅导航输出额外复制整帧点云到 map 坐标系，原异步导航 worker 继续负责输出处理。
- `module/src/LidarOdometry_Initialize.cpp`、`module/include/mola_lidar_odometry/LidarOdometry.h`：移除独立导航去畸变配置加载和成员。
- `module/include/mola_lidar_odometry/WorkerPipelineStats.h`：诊断阶段统一为 `deskew`，相应输出字段为 `last_deskew_ms`、`p95_deskew_ms`。
- `test/test_single_deskew.cpp`、`test/CMakeLists.txt`：增加单一配置、导航输出等价、字段保持及降采样输入所有权回归。

范围为当前 `lidar3d-gicp.yaml` 的生产处理链。其他 pipeline 的原范围过滤与非 IMU/optimize_twist 的可视化回退配置保留。导航输出 worker 中已有的车体轮廓处理保留。驱动当前查到的距离阈值为 0.05–200 m；本次没有改变驱动配置，取消定位侧范围过滤意味着定位输入范围扩大。

## 验证

- 独立 `colcon build` 成功，构建耗时约 4 分 33 秒。
- 6 组单元测试通过：`test_single_deskew`、`test_worker_optimizations`、`test_imu_scan_sync`、`test_gravity_map_aligner`、`test_keyframe_decider`、`test_map_frame_relevel`。
- `test_single_deskew` 的两项用例分别检查生产流水线只包含一项去畸变，以及运动点云的坐标、时间、强度、ring 与原完整导航分支一致；检查近点/远点保留，以及定位降采样不会修改导航共享点云或原始输入。该几何对照用例使用恒定线速度和角速度，不代表真实 IMU 实车精度验收。
- RSLIDAR 官方片段录包回归 `test_lidar_odometry_rslidar` 通过。它是已有数据回归，不作为当前生产 IMU 模式的实车验收。
- 已安装 CLI 的 `--help` smoke 通过；`ldd` 确认它解析到 `/home/iecme/omnifleet_ws/install/mola_lidar_odometry/lib`。
- 针对已安装的新库再次执行两项单次去畸变测试，通过。
- 安装前以 SHA-256 核对待改源文件，拒绝覆盖并发修改；源码与安装后的 GICP YAML 内容一致。
- 安装采用原子替换 ELF 文件，旧进程继续持有原映射，避免原地截断已加载库。
- MOLA 服务前后均为 active，MainPID 均为 `2524`，启动时刻不变。

## 安装与回滚证据

源码根目录：

`/home/iecme/omnifleet_ws/src/localization/omnifleet_mola/mola_lidar_odometry`

安装目录：

`/home/iecme/omnifleet_ws/install/mola_lidar_odometry`

备份目录：

`/home/iecme/robot_backups/deskew_unification_20261013`

- `source-before.tar.gz`：原包源码。
- `install-before/`：被替换的原库、CLI、两个头文件及 GICP 配置。
- `manifest.json`：7 个源文件修改及前后哈希。
- `deployment.json`：5 个安装文件及安装后哈希、原安装链接、服务保持记录。
- `single-deskew.patch`：完整源文件差异。
- `build.log`、`unit-tests.log`、`rslidar-test.log`、`installed-library-test.log`：构建与测试证据。
- `runtime-before.json`、`runtime-before-deploy.json`：只读运行快照。

回滚前应核对这些文件之后是否被主线程继续修改，按 manifest 恢复原源码和 `install-before` 中对应产物；不要覆盖后续独立改动。

## 尚未验证

状态：**PASS_BUILD / PASS_UNIT / PASS_ROSBAG_REGRESSION / PENDING_RUNTIME_ACTIVATION**。

主线程同时在操作机器人工作区，因此本次保留长驻定位进程和当前地图会话。当前运行仍使用此前已加载的双分支版本；新的 `deskew_ms` 诊断和处理链需要重新加载进程后才能观察。

尚未进行重启后的真实 IMU/点云验收、运动精度验证或同条件性能对照，不能声称 `worker busy` 已消除。只读快照显示 CPU 调度等待随其他编译负载明显变化，后续应在可比负载下测量总耗时、P95/P99、排队和丢帧。
