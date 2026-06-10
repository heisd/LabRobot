# double_lidar_fusion

WheelTec 机器人双激光雷达点云融合节点,将两个雷达的 `LaserScan` 数据按各自的安装位姿合并为一帧统一 360° 扫描,扩大 FOV 并补全死角。

## 概述

S300 等机型常在车体两侧分别安装一颗雷达。本节点订阅两路 `LaserScan`,对每个点做旋转 / 平移 / 角度合成,生成一帧融合扫描发布出去,供后续 SLAM、避障使用。

## 目录结构

```
double_lidar_fusion/
├── CMakeLists.txt
├── package.xml
├── include/
├── src/
│   ├── lidar_fusion.cpp     # 主节点(实际编译为可执行文件)
│   └── lidar_fun_back.cpp   # 备份/历史实现
└── launch/
    └── double_lidar_fusion.launch.py
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`sensor_msgs`、`message_filters`、`tf2`、`tf2_ros`

## 节点说明

### `lidar_fusion`(可执行文件:`lidar_fusion`)

订阅(两路**各自独立订阅**并缓存最新帧,QoS 为 `SensorDataQoS`/best-effort,
兼容 reliable 与 best-effort 的雷达驱动):
- `scan1_topic`(默认 `/scan1`)— 雷达 1 `sensor_msgs/LaserScan`
- `scan2_topic`(默认 `/scan2`)— 雷达 2 `sensor_msgs/LaserScan`

发布:
- `fused_topic`(默认 `scan`,launch 中重命名为 `scan`)— 融合后的 `sensor_msgs/LaserScan`

**故障降级(单雷达容错)**:由定时器按 `publish_rate_hz` 输出,根据各路最近一帧
是否在 `scan_timeout_sec` 内判断"存活":
- 两台都在线 → 融合两台(原有行为);
- 只有一台在线 → **仅输出存活的那一台**(仍按其外参投影到基坐标系),不再整路停掉;
- 两台都掉线 → 暂停发布,不发空帧。
状态切换时会打一条日志(`两台均在线 / 降级为只输出雷达X / 两台均无数据`)。

参数:
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `scan1_topic` | `/scan1` | 雷达 1 话题 |
| `scan2_topic` | `/scan2` | 雷达 2 话题 |
| `fused_topic` | `scan` | 融合输出话题 |
| `frame_id` | `laser` | 融合扫描所属坐标系 |
| `lidar1_angle_deg` | `45.0` | 雷达 1 安装角度(度) |
| `lidar1_x_offset_m` | `0.3` | 雷达 1 在车体的 X 偏移(m) |
| `lidar1_y_offset_m` | `0.235` | 雷达 1 在车体的 Y 偏移(m) |
| `lidar2_angle_deg` | `-135.0` | 雷达 2 安装角度(度) |
| `lidar2_x_offset_m` | `-0.3` | 雷达 2 在车体的 X 偏移(m) |
| `lidar2_y_offset_m` | `-0.235` | 雷达 2 在车体的 Y 偏移(m) |
| `scan_timeout_sec` | `0.5` | 超过此时长没收到帧即判定该雷达掉线(秒) |
| `publish_rate_hz` | `12.0` | 融合结果发布频率(Hz),建议设为雷达扫描频率 |

## 启动文件

### `double_lidar_fusion.launch.py`

直接以默认参数(对应 S300 Pro 两个雷达的安装位姿)启动 `lidar_fusion` 节点。

## 编译与运行

```bash
colcon build --packages-select double_lidar_fusion
source install/setup.bash
ros2 launch double_lidar_fusion double_lidar_fusion.launch.py
```

## 使用示例

```bash
# 1. 启动两台雷达,分别发布到 /scan1、/scan2
ros2 launch lslidar_driver lslidar_double_launch.py

# 2. 启动融合
ros2 launch double_lidar_fusion double_lidar_fusion.launch.py

# 3. SLAM/Nav2 订阅 /scan 即可
```

## 注意事项

1. 两个雷达需要在硬件上做触发同步或保证相近周期,否则融合后会出现伪影。
2. 安装角度/偏移参数必须按实际车体测量,否则两路雷达的同一物体会出现重影。
3. 默认输出 `frame_id = laser`,务必有 `base_link -> laser` 的静态 TF。
4. 与单雷达驱动同时启动同一台机器时,注意不要冲突占用串口/网口。
