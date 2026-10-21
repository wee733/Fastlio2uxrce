# 来源与许可

本仓库不是从零编写的独立算法实现，不统一覆盖上游许可证。

- FAST-LIO 和桥接快照来自 [W3378516198/ros2_px4_fastlio_ego](https://github.com/W3378516198/ros2_px4_fastlio_ego)，基线 `059dc6ae5aecb4eabf0e284b1b7b6374c1703ac8` 的 `px4_real/src`，加上本项目修正。FAST-LIO 保留原 `LICENSE`、源文件版权和上游 README。为减小仓库，未复制论文、演示图片和生成数据；上游 README 中这些资源请到原仓库查看。
- FAST-LIO 的 IKFoM、ikd-Tree 等代码保持各文件版权和许可证；不要抹除原作者信息。
- 原桥接 package.xml 的许可证为 TODO，原作者信息保留；本项目不擅自替原作者选择新许可证。私有保存或公开可见均不等于获得额外授权，分发/商业使用前需核清对应许可。
- PX4 固件补丁基于 PX4/PX4-Autopilot v1.17.0（BSD-3-Clause）；GPIO 复位改动对应上游已合并的 [PR #28746](https://github.com/PX4/PX4-Autopilot/pull/28746)。
- LI-Init 补丁基于 [morte2025/LiDAR_IMU_Init_ROS2](https://github.com/morte2025/LiDAR_IMU_Init_ROS2) 固定提交，原算法来自 [hku-mars/LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init)。保留其各自上游许可；本仓库未重新授权算法。
- Livox SDK/驱动、PX4 消息、Micro XRCE-DDS Agent、spdlog、linuxptp 使用 `dependencies.repos` 固定提交，保留各自许可，按脚本下载，不复制其完整源码。
