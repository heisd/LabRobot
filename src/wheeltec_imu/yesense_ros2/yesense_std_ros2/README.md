# yesense_std_ros2

元生(YeSense)IMU 系列的 ROS2 驱动节点,通过串口读取 YeSense 协议数据并发布为标准 `sensor_msgs/Imu` 与厂商自定义消息。

## 概述

该包基于 `serial`(ros_serial)库读取 YeSense IMU 发送的二进制协议帧,解析后分发为:
- 标准 `sensor_msgs/Imu`(供 EKF、Cartographer 等使用)
- `yesense_interface` 包定义的细粒度消息(姿态、四元数、GNSS、温度等)

## 目录结构

```
yesense_std_ros2/
├── CMakeLists.txt
├── package.xml
├── include/
├── src/
│   ├── yesense_node.cpp                       # 主发布节点
│   ├── yesense_decoder.cpp                    # 通用协议解析器
│   ├── yesense_std_out_decoder.cpp            # 标准输出解析器
│   └── yesense_node_subscriber_example.cpp    # 订阅示例
├── config/
│   └── yesense_config.yaml                    # 串口/话题/坐标系参数
└── launch/
    └── yesense_node.launch.py
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`sensor_msgs`、`std_msgs`、`geometry_msgs`、`serial`、`yesense_interface`

## 节点说明

### `yesense_node_publisher`

发布:
- `<imu_topic_ros>` (sensor_msgs/Imu) — 默认 `imu/data_raw`,可直接喂给 imu_filter_madgwick 或 EKF
- `<imu_topic>` (yesense_interface/ImuData) — 默认 `imu_data`,完整厂商协议数据
- 其他 yesense_interface 消息(欧拉角、四元数、GNSS 等)按驱动版本发布

参数(`config/yesense_config.yaml`):
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `serial_port` | `/dev/wheeltec_IMU` | IMU 串口设备 |
| `baud_rate` | `460800` | 波特率 |
| `frame_id` | `gyro_link` | 输出 frame |
| `driver_type` | `ros_serial` | 驱动后端类型 |
| `imu_topic_ros` | `imu/data_raw` | 标准 IMU 输出话题 |
| `imu_topic` | `imu_data` | 自定义 ImuData 输出话题 |

### `yesense_node_subscriber_example`

示例订阅节点,演示如何接收 `imu_data`,可作为二次开发模板。

## 启动文件

### `yesense_node.launch.py`

启动 `yesense_node_publisher`,加载 `config/yesense_config.yaml`。

## 编译与运行

```bash
colcon build --packages-up-to yesense_std_ros2
source install/setup.bash
ros2 launch yesense_std_ros2 yesense_node.launch.py
```

## 使用示例

```bash
# 1. 确认 udev 已经建立 /dev/wheeltec_IMU 软链接
ls -l /dev/wheeltec_IMU

# 2. 启动驱动
ros2 launch yesense_std_ros2 yesense_node.launch.py

# 3. 查看标准 IMU 话题
ros2 topic echo imu/data_raw

# 4. 启动 imu_filter_madgwick 过滤(turn_on_wheeltec_robot 中已包含)
```

## 注意事项

1. 默认 `serial_port` 为 `/dev/wheeltec_IMU`,需要先安装 `wheeltec_udev.sh` 中的 udev 规则。
2. 波特率必须与 IMU 实际设置一致,否则解析失败。
3. `frame_id` 必须与 URDF 中 IMU 的 link 一致(本系统为 `gyro_link`)。
4. 标准 IMU 话题已遵循 ROS 右手系,与 imu_filter_madgwick / EKF 兼容。
5. 该驱动与 `turn_on_wheeltec_robot` 内置的 IMU 数据来源不同,二者不要同时发布 `imu/data_raw`。
