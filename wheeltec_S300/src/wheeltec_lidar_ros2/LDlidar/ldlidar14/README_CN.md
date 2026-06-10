# ldlidar_stl_ros2 (LD14)

LDRobot LDLiDAR LD14 系列激光雷达的 ROS2 驱动包。

## 概述

LD14 是 LDRobot 推出的低成本 360° 2D 单线激光雷达,本包提供 ROS2 节点,将原始扫描数据转换为标准 `sensor_msgs/LaserScan` 输出。

## 目录结构

```
ldlidar14/
├── CMakeLists.txt
├── package.xml
├── README.md / LICENSE
├── include/
├── src/                       # 驱动源码
├── scripts/
├── rviz2/                     # RViz 预设
└── launch/
    ├── ld14.launch.py         # 启动雷达
    └── viewer_ld14.launch.py  # 启动雷达 + RViz
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`sensor_msgs`

## 节点说明

### `ldlidar_stl_ros2_node`

发布:
- `<topic_name>` (sensor_msgs/LaserScan) — 标准扫描

参数:
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `product_name` | `LDLiDAR_LD14` | 产品型号 |
| `topic_name` | `scan` | 发布话题 |
| `port_name` | `/dev/ttyUSB0` | 串口设备 |
| `frame_id` | `base_laser` | 输出 frame |
| `laser_scan_dir` | `True` | 扫描方向 |
| `enable_angle_crop_func` | `False` | 是否屏蔽角度 |
| `angle_crop_min` / `angle_crop_max` | `135.0` / `225.0` | 屏蔽角度区间 |

## 启动文件

- **`ld14.launch.py`** — 启动 LD14。
- **`viewer_ld14.launch.py`** — 启动 LD14 + RViz。

## 编译与运行

```bash
colcon build --packages-select ldlidar_stl_ros2
source install/setup.bash
ros2 launch ldlidar_stl_ros2 ld14.launch.py
```

## 注意事项

1. 与同源的 LD06/LD19 包(`ldlidar06` 目录)注意区分,不同型号扫描频率与点数不同。
2. udev 绑定固定串口名后,需要在 `port_name` 中相应修改。
3. LD14 默认顺时针,如发现地图镜像请切换 `laser_scan_dir`。
