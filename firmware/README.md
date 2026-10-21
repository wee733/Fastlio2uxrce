# MicoAir H743 V2 固件补丁

仅适用 `micoair_h743-v2_default`，基线 PX4 v1.17.0 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。Agent 2.4.3、PX4 消息 release/1.17 的 SHA 均固定在清单中。

`patches/px4-v1.17-micoair-imu-sync-dshot.patch` 包含：

1. `sensor_combined` DDS `rate_limit: 200`（上限，不保证实收 200 Hz）。
2. uXRCE 运行时同步请求周期从 1 秒改为 10 ms，匹配本机飞控时钟漂移与原样本驱动同步滤波器。
3. 板级复位 GPIO 主动拉低，修正双向 DShot 电调启动顺序相关问题。软件重启验证不代替整机冷启动验证。

如需重新构建：按 `optional.repos` 获取固定 PX4 提交，初始化其子模块，安装该版本 PX4 官方构建依赖，使用 `git apply --check` 检查补丁后应用，再执行 `make micoair_h743-v2_default`。本仓库不提供自动刷写或自动参数覆盖脚本。

标定工具同样在 `optional.repos`：应用 `patches/li-init-jazzy.patch`，复制 `calibration/mid360_fc_imu.yaml` 到其 `config/`，创建上游要求的 `Log/` 和 `result/`，安装 `libceres-dev` 后按 Jazzy colcon 构建。该补丁解决构建和实现缺陷，不保证所有时间/外参估计都可直接用于飞行。
