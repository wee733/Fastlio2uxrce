# MID360S + PX4 uXRCE-DDS + FAST-LIO2（ROS 2 Jazzy）

使用 **MicoAir H743 V2 的 IMU**，而非雷达内置 IMU。保存已在本机运行的桥接、标定配置、PX4 修正补丁与固定依赖版本。

2026-10-21/22 已完成：Jazzy/ARM64 编译、节点行为测试、标定、录包回放及实机静态 EKF 融合。**未完成解锁和飞行验证**：最后一次 Position 模式解锁检查受遥控输入缺失阻塞，未发送解锁命令。参见 [验证记录](docs/validation.md)。

## 数据链路

```text
PX4 sensor_combined (FRD, uXRCE timestamp conversion)
  → px4_imu_to_ros → /lio/imu (FLU, SI units)
MID360S + PTP → /livox/lidar (CustomMsg)
  → FAST-LIO2 → /Odometry (gravity-aligned camera_init / FC IMU body)
  → fastlio_to_px4_visual_odometry → /fmu/in/vehicle_visual_odometry
  → PX4 EKF2
```

点云和里程计约 10 Hz；200 Hz DDS 上限下 FC IMU 实收约 173 Hz。
桥接将世界与机体 FLU 同时转换为 FRD，保留测量时间，不重复加时钟偏移。速度字段默认 NaN，不发送伪造零速度。地图话题 `/Laser_map` 的定时器为 1 Hz，仅用于累计可视化，不代表 LIO 更新率。

## 目录

- `ros_ws/src/fast_lio`：FAST-LIO2 及重力对齐、外参检查、协方差发布修正。
- `ros_ws/src/fastlio_px4_bridge`：两个 C++ 桥接节点、启动文件与行为测试。
- `dependencies.repos`：Livox 驱动/SDK、PX4 消息、Agent、spdlog、linuxptp 的固定提交。
- `patches/`：PX4 固件、Livox Jazzy 退出处理、LI-Init Jazzy 适配补丁。
- `calibration/`：这台飞机的标定结果与 LI-Init 配置。
- `validation/`：只读频率、时间戳和融合状态探针。
- [来源与许可](THIRD_PARTY.md)、[固件说明](firmware/README.md)。

## 构建

Ubuntu 24.04 + ROS 2 Jazzy，建议至少预留几 GB 空间；编译默认最多 2 个任务。

```bash
sudo apt install git cmake build-essential python3-vcstool python3-colcon-common-extensions \
  python3-rosdep python3-numpy python3-dev libeigen3-dev libpcl-dev \
  ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-ament-cmake-gtest \
  ros-jazzy-ament-cmake-pytest ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common
./scripts/fetch_dependencies.sh
source /opt/ros/jazzy/setup.bash
# 若尚未初始化 rosdep，先按 ROS 安装说明初始化。
rosdep install --from-paths ros_ws/src --ignore-src -r -y --rosdistro jazzy
./scripts/build_dependencies.sh
./scripts/build_ros.sh
./scripts/test_ros.sh
```

依赖构建复用了本机已验证的构建选项；新目录中的两 ROS 包另行编译验证。完整联网下载、系统依赖安装与全部第三方重建尚未在全新系统上重演。

## 启动（每项单独终端）

先配置雷达网口：本机 `enP8p1s0=192.168.1.5`，雷达 `192.168.1.150`。其他机器须修改 `ros_ws/src/livox_ros_driver2/config/MID360s_config.json` 并重新安装配置。每个服务只能有一个实例，不能与已经运行的旧工作空间重复启动。

1. PTP：本雷达实测使用 linuxptp 3.1.1 的 PTPv2.0，而非本机 linuxptp 4.0 默认配置。

```bash
sudo ./deps/linuxptp/ptp4l -i enP8p1s0 \
  -f ros_ws/src/fastlio_px4_bridge/config/ptp4l-livox-v3.conf -m
```

2. Agent：串口实际为 THS3，稳定别名 `/dev/damiao-uart0`，921600 baud。

```bash
source /opt/ros/jazzy/setup.bash
./deps/install/bin/MicroXRCEAgent serial --dev /dev/damiao-uart0 -b 921600 -v 4
```

3. 雷达驱动：

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/local_setup.bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

4. LIO + 飞控 IMU + 预览桥接；初始化时保持静止：

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/local_setup.bash
ros2 launch fastlio_px4_bridge fc_imu_lio.launch.py
```

默认仅向 `/lio/vehicle_visual_odometry` 输出。确认当前机体、外参、重力对齐与时间同步后，**停止预览整套 launch**，再运行：

```bash
ros2 launch fastlio_px4_bridge fc_imu_lio.launch.py \
  output_topic:=/fmu/in/vehicle_visual_odometry reset_counter:=1
```

LIO 重启会改变原点；每次重启需递增 `reset_counter`（uint8），不能一直使用 1。没有自动重置检测，也没有自动解锁、飞行控制或开机自启。

## 外参和 PX4 参数

`ros_ws/src/fast_lio/config/mid360s_fc_imu.yaml` **属于这台飞机**。定义 `p_IMU_FLU = R_IL * p_LIDAR + t_IL`，矩阵按行、平移单位米。复算平移约 `[0.043247, -0.013759, 0.095454]`，主要倾角约 15°。换装或其他飞机应先将 `mapping.extrinsic_calibrated=false` 并重新标定。

FAST-LIO：`gravity_align=true`、`time_sync_en=false`、`time_offset_lidar_to_imu=0`。LI-Init 报告的 −30 ms 不能直接作为真实时钟偏差，详见验证记录。

实际台架参数快照（不提供自动批量写入脚本）：

| 参数 | 台架取值 |
|---|---:|
| UXRCE_DDS_CFG | 102（本板 TEL2 / ttyS3） |
| SER_TEL2_BAUD | 921600 |
| UXRCE_DDS_SYNCT / SYNCC | 1 / 0 |
| IMU_INTEG_RATE | 200 |
| EKF2_EV_CTRL | 11：水平位置、高度、航向；不融合外部速度 |
| EKF2_HGT_REF | 3：Vision |
| EKF2_EV_DELAY | 0 |
| EKF2_EV_QMIN | 0：桥接质量未知 |
| EKF2_EV_POS_X/Y/Z | 0 / 0 / 0：本次输出原点为 FC IMU |
| DSHOT_BIDIR_EN | 1 |

任意航向 FRD 不是北东地 NED；不要将 pose_frame 改成 NED 来伪造北向对齐。同一个 FC IMU 进入 LIO 和 EKF，误差有相关性；固定协方差不是实测定位精度。噪声、失效与动态行为还需验证。

## 只读检查

```bash
python3 validation/imu_link_probe.py --seconds 60 --output /tmp/imu.json
python3 validation/probe_live_lio.py --seconds 60 --output /tmp/lio.json
ros2 topic hz /Odometry
```

`lidar_clock_probe.py` 是原始网络抓包工具，需相应权限；使用前查看 `--help`。不要同时反序列化大量点云来推断 IMU 的真实发布频率。
