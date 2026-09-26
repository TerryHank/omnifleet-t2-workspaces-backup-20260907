# 实车驱动边界

`omnifleet_bringup` 是远端实车源码快照，不是 Gazebo 插件。

它启动时会立即：

1. 通过 pyserial 打开 `/dev/myserial`（115200）；
2. 设置 STM32 R2 车型；
3. 订阅 `/cmd_vel` 并发送 `FUNC_MOTION=0x12`；
4. 由 STM32 固件独占计算 Ackermann 舵角与轮速；
5. 接收速度、IMU、电压和固件反馈。

因此它不能直接驱动 Gazebo。交付包中的仿真入口均不会 import 或启动它。

如果未来确实部署回实车：

- 删除 `omnifleet_bringup/COLCON_IGNORE`；
- 补齐完整 OmniFleet 依赖；
- 按 `hardware_samples/` 重新核对 udev、systemd 和环境路径；
- 先确认 `/dev/myserial` 对应 CH341 `1a86:7523`；
- 不要把样例文件盲目复制到不同用户名/工作空间路径。
