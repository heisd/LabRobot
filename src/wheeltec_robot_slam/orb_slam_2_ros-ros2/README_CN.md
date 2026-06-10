# orb_slam2_ros (WheelTec ROS2 适配)

ORB-SLAM2 的 ROS2 节点封装,基于稀疏特征点的视觉 SLAM。

> 重要说明:本仓库的 `CMakeLists.txt` 中只构建并安装了 **RGB-D**
> 可执行 `orb_slam2_ros_rgbd`,`*_mono` 和 `*_stereo` 两个 target 已被
> 注释掉。因此在不修改 CMake 的前提下,只能使用 RGB-D 模式;包内
> 提供的 mono / stereo launch 与 yaml 仅为模板,直接运行会因找不到
> 可执行而失败。

## 概述

ORB-SLAM2 是稀疏特征点视觉 SLAM 的代表性算法。本包将其封装为 ROS2 节点,直接订阅图像与相机内参,发布相机位姿、地图点、稀疏点云等话题,适合用于 WheelTec 机器人配合 RGB-D 相机的视觉建图与定位。

## 目录结构

```
orb_slam_2_ros-ros2/
├── CMakeLists.txt                 # 仅构建 orb_slam2_ros_rgbd
├── package.xml                    # <name>orb_slam2_ros</name>
├── README.md / LICENSE.txt / License-gpl.txt
├── Dependencies.md
├── orb_slam2/                     # ORB-SLAM2 核心库源码与第三方依赖(g2o、DBoW2、Vocabulary)
├── ros/
│   ├── launch/                    # ROS2 launch
│   │   ├── orb_slam2_Astra_rgbd_launch.py
│   │   ├── orb_slam2_Gemini_rgb_launch.py
│   │   ├── orb_slam2_d435_rgbd_launch.py
│   │   ├── orb_slam2_d435_mono_launch.py     # 需先开启 mono 构建
│   │   ├── orb_slam2_t265_mono_launch.py     # 需先开启 mono 构建
│   │   └── orb_slam2_t265_stereo_launch.py   # 需先开启 stereo 构建
│   └── config/
│       ├── params_Astra_rgbd.yaml
│       ├── params_Gemini_rgbd.yaml
│       ├── params_d435_rgbd.yaml
│       ├── params_d435_mono.yaml
│       ├── params_t265_mono.yaml
│       ├── params_t265_stereo.yaml
│       ├── octomap_server.yaml
│       └── rviz_config.rviz
├── docker/                        # Docker 部署示例
├── srv/
│   ├── SaveMap.srv
│   └── SaveCloud.srv
└── rviz_config.rviz
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`sensor_msgs`、`geometry_msgs`、`nav_msgs`、`tf2`、`tf2_ros`、`tf2_geometry_msgs`、`cv_bridge`、`image_transport`、`message_filters`、`OpenCV`、`Pangolin`、`Eigen3`、`PCL`、`octomap_server`

## 实际构建的可执行

`CMakeLists.txt` 当前状态:
```cmake
#add_executable(${PROJECT_NAME}_mono   …)   # 已注释
#add_executable(${PROJECT_NAME}_stereo …)   # 已注释
add_executable(${PROJECT_NAME}_rgbd    …)   # 唯一被构建

install(TARGETS ${PROJECT_NAME}_rgbd …)     # 仅安装 rgbd
```

因此 `colcon build` 后只会得到一个可执行:

- `orb_slam2_ros_rgbd` — RGB-D 模式入口

`orb_slam2_ros_mono`、`orb_slam2_ros_stereo` 默认不会生成。若需要它们,请先在 `CMakeLists.txt` 中取消 mono/stereo 相关 `add_executable` 与 `install(TARGETS …)` 的注释,然后重新编译。

## 节点说明

### `orb_slam2_ros_rgbd`

订阅:
- `/camera/rgb/image_raw` (sensor_msgs/Image)
- `/camera/depth/image_raw` (sensor_msgs/Image)
- `/camera/rgb/camera_info` (sensor_msgs/CameraInfo)

(具体话题名以各 launch 中 remap 为准,例如 Astra 使用 `/camera/rgb/image_raw` + `/camera/depth/image_raw`。)

发布:
- `~/map_points` (sensor_msgs/PointCloud2) — 稀疏地图点(`publish_pointcloud=true` 时)
- `~/pose` (geometry_msgs/PoseStamped) — 相机位姿(`publish_pose=true` 时)
- TF: `map -> camera_link`

主要参数(摘自 `params_Astra_rgbd.yaml`):
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `publish_pointcloud` | `true` | 是否发布稀疏地图点 |
| `publish_pose` | `true` | 是否发布相机位姿 |
| `localize_only` | `false` | 纯定位模式 |
| `reset_map` | `false` | 启动时清空地图 |
| `load_map` | `false` | 启动时加载序列化地图 |
| `map_file` | `map.bin` | 地图文件名 |
| `savePCDDirectory` | `/wheeltec_ros2/src/data/` | 点云保存路径 |
| `pointcloud_frame_id` | `map` | 点云所在 frame |
| `camera_frame_id` | `camera_link` | 相机 frame |
| `min_num_kf_in_map` | `5` | 地图最少关键帧数 |
| `ORBextractor/nFeatures` | `1000` | ORB 特征点数 |
| `ORBextractor/scaleFactor` | `1.2` | 尺度因子 |
| `ORBextractor/nLevels` | `8` | 金字塔层数 |
| `ORBextractor/iniThFAST` / `minThFAST` | `20` / `7` | FAST 阈值 |
| `camera_fps` | `30` | 帧率 |
| `camera_rgb_encoding` | `true` | RGB 编码顺序 |
| `ThDepth` | `40.0` | 近/远阈值 |
| `depth_map_factor` | `250.0` | 深度缩放因子 |
| `camera_baseline` | `75.0` | 基线(mm) |

## 服务定义

### `srv/SaveMap.srv`

```
string name
---
bool success
```

### `srv/SaveCloud.srv`

```
string name
---
bool success
```

均通过 `ros2 service call /<node>/save_map "{name: 'my_map.bin'}"` 触发。

## 启动文件

- **`orb_slam2_Astra_rgbd_launch.py`** — 奥比中光 Astra RGB-D(可直接运行)
- **`orb_slam2_Gemini_rgb_launch.py`** — 奥比中光 Gemini RGB-D(可直接运行)
- **`orb_slam2_d435_rgbd_launch.py`** — Intel RealSense D435 RGB-D(可直接运行)
- **`orb_slam2_d435_mono_launch.py`** — D435 单目(需先开启 mono 构建)
- **`orb_slam2_t265_mono_launch.py`** — T265 单目(需先开启 mono 构建)
- **`orb_slam2_t265_stereo_launch.py`** — T265 双目(需先开启 stereo 构建)

每个 launch 内部都使用:
```python
Node(package='orb_slam2_ros', executable='orb_slam2_ros_rgbd', …)
```
并通过 remap 将相机话题接入,加载对应的 `params_*.yaml`。

## 编译与运行

```bash
# 第一次编译需先准备 ORBvoc.txt 词袋(包内 orb_slam2/Vocabulary/ 已附带)
colcon build --packages-up-to orb_slam2_ros
source install/setup.bash

# 直接运行 RGB-D 可执行(需自己传入参数 yaml,通常通过 launch 启动)
ros2 run orb_slam2_ros orb_slam2_ros_rgbd \
    --ros-args --params-file $(ros2 pkg prefix orb_slam2_ros)/share/orb_slam2_ros/config/params_Astra_rgbd.yaml
```

## 使用示例

```bash
# 1. 启动 Astra RGB-D 相机
ros2 launch astra_camera astra.launch.xml

# 2. 启动 ORB-SLAM2 RGB-D
ros2 launch orb_slam2_ros orb_slam2_Astra_rgbd_launch.py

# 3. RViz 加载 rviz_config.rviz 查看地图与轨迹

# 4. 保存地图
ros2 service call /orb_slam2_rgbd/save_map orb_slam2_ros/srv/SaveMap "{name: 'my_map.bin'}"
```

## 注意事项

1. **mono / stereo 默认不构建**。如果运行 `ros2 launch orb_slam2_ros orb_slam2_d435_mono_launch.py` 报 `executable 'orb_slam2_ros_mono' not found`,属于预期行为。要使用单目或双目,请取消 `CMakeLists.txt` 中相关 `add_executable` 与 `install(TARGETS …)` 的注释,再 `colcon build` 重新编译。
2. 包名是 `orb_slam2_ros`(无 `2_`),不是目录名 `orb_slam_2_ros-ros2`。
3. `settings_file` / 内参需按每个相机型号的 `params_*.yaml` 配置,WheelTec 已为常见相机预设。
4. `savePCDDirectory` 默认 `/wheeltec_ros2/src/data/`,部署到其他路径时需修改 yaml。
5. 大场景或快速运动可能造成跟踪丢失,需要做闭环或重定位。
6. Pangolin 可视化默认开启,部署到无显示器环境需关闭。
