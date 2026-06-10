# turn_on_wheeltec_robot

WheelTec S300 系列机器人底盘的 ROS2 启动与驱动包,负责与下位机串口通讯、发布里程计/IMU/电源等话题,并完成静态坐标变换、EKF 融合定位等核心功能。

## 概述

该包是整个机器人系统的"启动入口",一次启动便会把串口驱动、坐标变换、IMU 滤波、EKF 融合等基础模块全部拉起来,后续的 SLAM、导航、视觉等功能模块都依赖它发布的话题。

## 目录结构

```
turn_on_wheeltec_robot/
├── CMakeLists.txt
├── package.xml
├── wheeltec_udev.sh              # 创建 /dev/wheeltec_controller、雷达、IMU 等 udev 软链接
├── include/                      # 头文件 (turn_on_wheeltec_robot.h 等)
├── src/
│   ├── wheeltec_robot.cpp        # 串口通讯与里程计/IMU 解析主节点
│   └── Quaternion_Solution.cpp   # 姿态四元数解算
├── msg/Position.msg              # 自定义位置消息
├── config/
│   ├── ekf.yaml                  # robot_localization ekf_node 配置
│   ├── ekf_carto.yaml            # 使用 cartographer 时的 EKF 配置
│   ├── imu.yaml                  # imu_filter_madgwick 滤波参数
│   └── camera_info.yaml          # 相机标定信息
└── launch/
    ├── turn_on_wheeltec_robot.launch.py   # 顶层启动文件
    ├── base_serial.launch.py              # 拉起底层串口节点
    ├── robot_mode_description.launch.py   # 加载 URDF/joint_state_publisher
    ├── wheeltec_ekf.launch.py             # robot_localization EKF 节点
    ├── wheeltec_lidar.launch.py           # 雷达驱动
    ├── wheeltec_camera.launch.py          # 相机驱动
    └── wheeltec_sensors.launch.py         # 所有传感器一起启动
```

## 依赖项

- buildtool: `ament_cmake`、`rosidl_default_generators`
- 主要 depend: `rclcpp`、`rclpy`、`rclcpp_action`、`tf2`、`tf2_ros`、`tf2_geometry_msgs`、`geometry_msgs`、`sensor_msgs`、`nav_msgs`、`nav2_msgs`、`std_msgs`、`std_srvs`、`ackermann_msgs`、`serial`、`ament_index_cpp`、`turtlesim`
- 运行依赖: `rosidl_default_runtime`

## 节点说明

### `wheeltec_robot`(可执行文件:`wheeltec_robot_node`)

负责通过串口与底盘 MCU 通讯,并把电机/IMU/电池等数据转换为标准 ROS2 话题。

订阅话题:
- `cmd_vel` (geometry_msgs/Twist) — 速度指令,话题名通过参数 `cmd_vel` 重映射
- `red_vel` (geometry_msgs/Twist) — 红外避障速度指令
- `robot_recharge_flag` (std_msgs/Int8) — 自动回充触发标志
- `chassis_security` (std_msgs/Int8) — 安全停车标志

发布话题:
- `odom` (nav_msgs/Odometry) — 里程计
- `imu/data_raw` (sensor_msgs/Imu) — 原始 IMU 数据
- `PowerVoltage` (std_msgs/Float32) — 电池电压
- `Distance` (robot_interfaces/Supersonic) — 超声波距离
- `robot_charging_flag` (std_msgs/Bool) — 是否在充电
- `robot_charging_current` (std_msgs/Float32) — 充电电流
- `robot_red_flag` (std_msgs/Bool) — 红外避障状态
- `/self_check_data` (std_msgs/UInt32) — 自检数据

主要参数:
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `usart_port_name` | `/dev/wheeltec_controller` | 下位机串口设备 |
| `serial_baud_rate` | `115200` | 串口波特率 |
| `cmd_vel` | `cmd_vel` | 速度话题名 |
| `odom_frame_id` | `odom_combined` | 里程计父坐标系 |
| `robot_frame_id` | `base_footprint` | 里程计子坐标系 |
| `gyro_frame_id` | `gyro_link` | IMU 坐标系 |
| `car_mode` | `S300` | 车型 |
| `robot_type` | `Plus` | 机器人型号 |
| `odom_x_scale`、`odom_y_scale`、`odom_z_scale_positive`、`odom_z_scale_negative` | `1.0` | 里程计标定系数 |

## 启动文件

- **`turn_on_wheeltec_robot.launch.py`**(顶层) — 依次包含 `base_serial.launch.py`、`wheeltec_ekf.launch.py`、`robot_mode_description.launch.py`,并启动 `tf2_ros/static_transform_publisher`(`base_footprint -> base_link`)和 `imu_filter_madgwick_node`。根据环境变量 `ROBOT_TYPE`(`s300_pro` / `s300_mini` / 其他)选择对应的静态 TF 平移参数。支持 `carto_slam`(默认 `false`)参数,启用时切换到 cartographer 友好的 EKF 配置。
- **`base_serial.launch.py`** — 启动 `wheeltec_robot_node`,加载串口、超声波避障等参数。
- **`wheeltec_ekf.launch.py`** — 启动 `robot_localization/ekf_node`,使用 `config/ekf.yaml` 或 `config/ekf_carto.yaml`。
- **`robot_mode_description.launch.py`** — 加载机器人 URDF 模型并发布 joint_states/TF。
- **`wheeltec_lidar.launch.py`** / **`wheeltec_camera.launch.py`** / **`wheeltec_sensors.launch.py`** — 分别启动雷达、相机、全部传感器。

## 消息定义

`msg/Position.msg`:
```
float32 angle_x   # x 方向角度
float32 angle_y   # y 方向角度
float32 distance  # 距离
```

## 编译与运行

```bash
cd ~/wheeltec_S300
colcon build --packages-select turn_on_wheeltec_robot robot_interfaces
source install/setup.bash

# 第一次运行先安装 udev 规则,确保串口/雷达设备名固定
sudo cp src/turn_on_wheeltec_robot/wheeltec_udev.sh /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger

# 启动底盘
export ROBOT_TYPE=s300_pro
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

## 使用示例

```bash
# 一键启动底盘 + EKF + URDF + IMU 滤波
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 与 cartographer 联用
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py carto_slam:=true

# 启动全部传感器
ros2 launch turn_on_wheeltec_robot wheeltec_sensors.launch.py
```

## 注意事项

1. 需要先执行 `wheeltec_udev.sh` 创建串口软链接,否则会找不到 `/dev/wheeltec_controller`。
2. 部署到 S300_Pro / S300_Mini 时,必须设置环境变量 `ROBOT_TYPE`,否则使用默认 TF 偏移可能导致定位/导航不准。
3. 该包依赖 `robot_interfaces`,务必同时编译。
4. 该节点对串口有独占访问,必须停止后才能再启动其他用到串口的程序。
