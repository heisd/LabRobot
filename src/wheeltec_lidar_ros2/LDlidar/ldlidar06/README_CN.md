# ldlidar_stl_ros2(LD06/LD19)

LDRobot 公司 LDLiDAR LD06 / LD19 2D 单线激光雷达的 ROS2 驱动包。

## 概述

LD06 / LD19 是常见的 USB 串口激光雷达,广泛用于扫地机、低成本移动机器人。本包提供 ROS2 节点,发布标准 `sensor_msgs/LaserScan`。

## 目录结构

```
ldlidar06/
├── CMakeLists.txt
├── package.xml
├── README.md / LICENSE          # 原始英文
├── include/                     # 头文件
├── src/
│   └── demo.cpp                 # 主驱动节点
├── scripts/
├── rviz2/                       # RViz 预设
├── ws_deploy.sh                 # 工作空间部署脚本
└── launch/
    ├── ld06.launch.py
    ├── ld19.launch.py
    ├── viewer_ld06.launch.py
    └── viewer_ld19.launch.py
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`sensor_msgs`

## 节点说明

### `ldlidar_stl_ros2_node`(对应 demo.cpp)

发布:
- `<topic_name>` (sensor_msgs/LaserScan) — 标准扫描数据,话题名通过参数 `topic_name` 设置

参数:
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `product_name` | `LDLiDAR_LD06` 或 `LDLiDAR_LD19` | 产品型号 |
| `topic_name` | `scan` | 发布话题 |
| `port_name` | `/dev/ttyUSB0` | 串口设备 |
| `frame_id` | `base_laser` | 输出 frame |
| `laser_scan_dir` | `True` | 是否逆时针 |
| `enable_angle_crop_func` | `False` | 是否启用角度遮挡 |
| `angle_crop_min` / `angle_crop_max` | `135.0` / `225.0` | 屏蔽角度区间(度) |

## 启动文件

- **`ld06.launch.py`** — 启动 LD06,默认参数。
- **`ld19.launch.py`** — 启动 LD19。
- **`viewer_ld06.launch.py`** / **`viewer_ld19.launch.py`** — 启动雷达 + RViz。

## 编译与运行

```bash
colcon build --packages-select ldlidar_stl_ros2
source install/setup.bash
ros2 launch ldlidar_stl_ros2 ld06.launch.py
```

## 使用示例

```bash
# 启动雷达并查看
ros2 launch ldlidar_stl_ros2 viewer_ld06.launch.py
ros2 topic echo /scan
```

## 注意事项

1. 串口设备名通常会变动,建议通过 `udev` 绑定固定名称(如 `/dev/wheeltec_lidar`)。
2. LD06 / LD19 转速不同,扫描频率会影响 SLAM 性能。
3. `laser_scan_dir` 设置错误会导致地图镜像,可通过对比 RViz 看是否正常。
4. `angle_crop` 用于屏蔽自身机器人结构的反射点。
