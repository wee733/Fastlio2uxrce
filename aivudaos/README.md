# FAST-LIO for AivudaOS

本应用使用 Jetson 上的 ROS 2 Jazzy、MID-360S 和飞控 IMU。App 包含 aarch64 版 FAST-LIO、PX4 桥接及对应源码。预先运行雷达驱动和 uXRCE Agent，并确认飞控消息定义与当前固件匹配。

## 配置和启动

在 AivudaOS 中设置 ROS 环境和工作空间。此机的 PX4、Livox 消息由 `/home/nvidia/ros2_ws/install/local_setup.bash` 提供。`workspaces.fastlio_setup` 留空时使用包内工作空间，也可指定兼容版本的绝对路径。`fastlio.ros_domain_id = -1` 继承系统域。配置修改在重启应用后生效。

机载传感器位于同一 Jetson 时，App 默认使用 `fastlio.transport_mode: localhost_udp`。飞控 IMU 转换、FAST-LIO、里程计桥接和发布者预检都通过 Fast DDS 的 IPv4 环回单播发现和通信，保留配置的 ROS 域，关闭共享内存传输及读写端的数据共享。每个 UDP 接收缓冲区申请 4 MiB，可发现默认起始的 48 个本机参与者索引。主机的 `net.core.rmem_max` 至少需要 4194304 字节，App 启动时检查，系统管理员负责配置。若雷达或飞控 DDS 发布者位于另一台机器、使用其他 RMW 或自定义发现方式，将 `fastlio.transport_mode` 设为 `inherit` 以完整继承运行环境。

直接运行 ROS 工作空间的 `ros2 launch fastlio_px4_bridge fc_imu_lio.launch.py` 时，默认沿用调用方环境。添加 `transport_mode:=localhost_udp` 可为本次 launch 的三个节点启用同一份 `fastlio_px4_bridge/config/localhost_udp.xml`。App 的启动脚本会先为整个进程树应用该配置，因此发布者冲突检查和三个子节点使用一致的传输方式。

每次启动根据保存的配置在 AivudaOS 指定的运行数据目录写入 `fastlio_params.yaml` 和 `bridge_params.yaml`，并将这两个文件分别交给定位和桥接节点。使用 `common` 设置雷达、转换后的飞控 IMU 话题和时间偏移，使用 `preprocess`、`mapping`、`publish` 设置滤波与输出。默认近场盲区为 0.5 m，每 2 点取 1 点，迭代上限为 4，表面和地图体素尺寸分别为 0.15 m 和 0.2 m。`/cloud_registered` 默认发布去畸变的稠密扫描点。单帧点数和分布仍依赖实际视野与有效回波。

`mapping.extrinsic_T` 是以米为单位的雷达到飞控 IMU 平移，`mapping.extrinsic_R` 是按行排列的旋转矩阵。默认值来自本机标定。默认开启重力对齐、固定外参。关闭重力对齐后可查看 FAST-LIO 的原始输出，桥接器会停止向 PX4 提交未经水平对齐的里程计。

`px4_bridge` 配置输出和诊断话题、方差及允许的最大采样延迟。健康门控要求连续新鲜的 IMU 和里程计。任一组件退出会结束整套 launch。启动前检查配置的里程计输出、`/Odometry` 和 IMU 话题是否已有发布者，并获取 `/tmp/fastlio-px4-<uid>` 的进程锁。切换外部定位源前应先停止原定位源。

## 话题和状态

- 默认输入为 `/livox/lidar` 和 `/fmu/out/sensor_combined`。转换器输出与 FAST-LIO 和桥接器使用同一个 `common.imu_topic`，默认为 `/lio/imu`。
- `/Odometry` 为 FAST-LIO 原始里程计。`/fmu/in/vehicle_visual_odometry` 为默认的 PX4 输出。
- `/fast_lio/bridge_status` 使用 `diagnostic_msgs/msg/DiagnosticArray`，提供 IMU、里程计频率、采样年龄及健康门控状态。在地面站使用相应 UAV 命名空间。

进入 Ready 后，连续 100 ms 无新鲜 IMU，或里程计超过 `px4_bridge.max_sample_age`，守卫会停止输出并退出。退出日志保留首次失效的输入和当时距上个有效 IMU、里程计的毫秒数。若原始飞控 IMU 随后恢复，转换器的 `SensorCombined callback gap` 日志会同时报告回调间隔、相邻消息的源时间戳差和当前样本年龄，可用来区分投递积压与源时间戳本身的跳变。

`state.counter_directory` 留空时使用 `$AIVUDA_APP_RUNTIME_DATA_PATH/reset_counter/`。每次通过预检后递增目录内的 `reset_counter` 文件，255 之后回到 0。切换旧版本或共享定位源时，将原有计数值迁入新目录，或者给所有配合使用的应用配置同一个可写的绝对目录。

## 重新打包

`ros_ws/src` 包含与包内二进制对应的源码，FAST-LIO 许可证位于 `ros_ws/src/fast_lio/LICENSE`。在目标架构上加载 ROS、PX4 和 Livox 工作空间后，可在 `ros_ws` 中通过 `colcon build --packages-select fast_lio fastlio_px4_bridge --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release` 重新构建。运行 `python3 -m unittest discover -s tests` 测试启动配置和互斥，再执行 `./pack.sh /path/to/output` 生成与 manifest 应用标识和版本匹配的 tar.gz。
