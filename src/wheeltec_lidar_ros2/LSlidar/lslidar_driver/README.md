# lslidar_driver

镭神 LSM10 / LSN10 系列 2D 单线激光雷达的 ROS2 驱动包,支持 UART 与网口两种连接方式,提供单雷达、双雷达多种启动示例。

## 概述

驱动以 ROS2 生命周期节点(LifecycleNode)实现,启动后即开始向 `/scan` 发布 `sensor_msgs/LaserScan`。包内同时附带 RViz 可视化预设。

## 目录结构

```
lslidar_driver/
├── CMakeLists.txt
├── package.xml
├── include/
├── src/                                  # 驱动源码(C++)
├── rviz/                                 # RViz 预设
├── params/
│   ├── lidar_uart_ros2/                  # 串口型号参数
│   │   ├── lsm10.yaml
│   │   ├── lsm10_p.yaml
│   │   ├── lsn10.yaml
│   │   └── lsn10p.yaml
│   ├── lidar_net_ros2/                   # 网口型号参数
│   │   ├── lsm10_net.yaml
│   │   └── lsm10p_net.yaml
│   ├── lsx10.yaml / lsx10_1.yaml / lsx10_2.yaml
└── launch/
    ├── lsn10_launch.py / lsn10p_launch.py
    ├── lsm10_uart_launch.py / lsm10_net_launch.py
    ├── lsm10p_uart_launch.py / lsm10p_net_launch.py
    ├── lslidar_double_launch.py          # 双雷达启动
    └── viewer_scan_launch.py             # RViz 查看
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`rclcpp_lifecycle`、`sensor_msgs`、`std_msgs`、`lslidar_msgs`、`pcl_conversions`、`serial`、`tf2`、`tf2_ros`、`tf2_geometry_msgs`、`PCL`

## 节点说明

### `lslidar_driver_node`(LifecycleNode)

发布:
- `/scan` (sensor_msgs/LaserScan) — 标准 2D 扫描
- 可选 `pointcloud` (sensor_msgs/PointCloud2)
- 自定义 `lslidar_msgs/*` 调试话题

参数(根据 yaml 不同):
| 参数 | 含义 |
| --- | --- |
| `device_ip` | 网口型号:雷达 IP |
| `msop_port` / `difop_port` | 网口型号:数据/控制端口 |
| `serial_port` | 串口型号:`/dev/ttyUSB0` 等 |
| `baud_rate` | 串口波特率 |
| `frame_id` | 默认 `laser` |
| `scan_topic` | 输出话题 |
| `min_range` / `max_range` | 距离过滤范围 |
| `angle_disable_min` / `angle_disable_max` | 屏蔽角度区间 |
| `use_gps_ts` | 是否使用 GPS 时间戳 |

具体字段以对应 yaml 文件为准,例如 `params/lidar_uart_ros2/lsn10.yaml`。

## 启动文件

| 文件 | 用途 |
| --- | --- |
| `lsn10_launch.py` | 启动 LSN10 串口雷达 |
| `lsn10p_launch.py` | 启动 LSN10P 串口雷达 |
| `lsm10_uart_launch.py` | 启动 LSM10 串口 |
| `lsm10p_uart_launch.py` | 启动 LSM10P 串口 |
| `lsm10_net_launch.py` | 启动 LSM10 网口 |
| `lsm10p_net_launch.py` | 启动 LSM10P 网口 |
| `lslidar_double_launch.py` | 同时启动两台雷达,发布到 `/scan1`、`/scan2`,配合 `double_lidar_fusion` |
| `viewer_scan_launch.py` | 启动雷达 + RViz 可视化 |

## 编译与运行

```bash
colcon build --packages-up-to lslidar_driver
source install/setup.bash

# LSN10 串口
ros2 launch lslidar_driver lsn10_launch.py

# LSM10 网口
ros2 launch lslidar_driver lsm10_net_launch.py
```

## 使用示例

```bash
# 单雷达
ros2 launch lslidar_driver lsn10_launch.py
ros2 topic echo /scan

# 双雷达 + 融合
ros2 launch lslidar_driver lslidar_double_launch.py
ros2 launch double_lidar_fusion double_lidar_fusion.launch.py
```

## 注意事项

1. 串口型号必须在 `udev` 中绑定固定设备名(如 `/dev/wheeltec_lidar`),否则参数中的 `serial_port` 可能找不到设备。
2. 网口型号的雷达 IP 与本机网卡必须在同一子网,且 MSOP/DIFOP 端口未被占用。
3. 与 `wheeltec_robot` 串口冲突时优先检查 udev 规则。
4. 默认 `frame_id=laser`,务必有对应的 TF。
