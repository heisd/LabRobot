# astra_camera

奥比中光(Orbbec)Astra / Dabai / Gemini 系列 RGB-D 相机的 ROS2 驱动包。

## 概述

基于 Orbbec OpenNI2 / OB SDK 实现的 ROS2 多设备深度相机驱动,支持彩色、深度、IR、点云、IMU 等数据流,可同时驱动多台同型号或不同型号相机。

## 目录结构

```
astra_camera/
├── CMakeLists.txt
├── package.xml
├── include/                       # 头文件
├── src/
│   ├── main.cpp                   # 主入口
│   ├── ob_camera_node.cpp         # 单相机节点
│   ├── ob_camera_node_factory.cpp # 多相机工厂
│   ├── ob_camera_info.cpp         # 内/外参处理
│   ├── ob_context.cpp             # SDK 上下文
│   ├── ob_timer_filter.cpp        # 时间滤波
│   ├── list_devices_node.cpp      # 列出当前所有连接设备
│   ├── clean_up_shm_node.cpp      # 清理共享内存
│   ├── dynamic_params.cpp         # 动态参数
│   ├── ros_param_backend.cpp      # ROS 参数后端
│   ├── ros_service.cpp            # service 实现
│   ├── uvc_camera_driver.cpp      # UVC 相机(彩色)驱动
│   ├── utils.cpp
│   └── point_cloud_proc/          # 点云后处理
├── config/                        # 标定/参数文件目录(空,运行时填充)
├── rviz/                          # RViz 预设
├── scripts/                       # 辅助脚本
└── launch/                        # 每种型号一个 launch
    ├── astra.launch.xml
    ├── astra_pro.launch.xml / astro_pro_plus.launch.xml
    ├── dabai_dc1.launch.xml / dabai_dcw.launch.xml / dabai_dw.launch.xml
    ├── dabai_pro.launch.xml / dabai_u3.launch.xml
    ├── deeyea.launch.xml
    ├── embedded_s.launch.xml / embedded_u3.launch.xml
    ├── gemini.launch.xml / gemini_e.launch.xml / gemini_e_lite.launch.xml
    ├── gemini_arm.launch.{xml,py} / gemini_car.launch.xml
    ├── stereo_s.launch.xml / stereo_s_u3.launch.xml
    ├── multi_*.launch.xml         # 多设备
    └── list_devices.launch.xml    # 列设备
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`sensor_msgs`、`std_msgs`、`std_srvs`、`tf2`、`tf2_ros`、`image_transport`、`camera_info_manager`、`astra_camera_msgs`、`OpenNI2`、`libuvc`、`Eigen3`、`OpenCV`、`PCL`、`pcl_conversions`

## 节点说明

### `astra_camera_node`(对应 `ob_camera_node_factory.cpp` 入口)

发布(常见话题,以 `gemini.launch.xml` 默认 namespace=`camera` 为例):
- `/camera/color/image_raw` (sensor_msgs/Image)
- `/camera/color/camera_info` (sensor_msgs/CameraInfo)
- `/camera/depth/image_raw` (sensor_msgs/Image)
- `/camera/depth/camera_info` (sensor_msgs/CameraInfo)
- `/camera/depth/points` (sensor_msgs/PointCloud2)
- `/camera/depth/color/points` (sensor_msgs/PointCloud2) — 彩色点云
- `/camera/ir/image_raw` (sensor_msgs/Image) — 红外
- 设备信息:`/camera/device_info`、`/camera/extrinsics/*`、`/camera/depth/metadata` 等

服务(对应 `astra_camera_msgs/srv`):
- `~/get_camera_info`、`~/get_camera_params`、`~/get_device_info`
- `~/get_int32` / `~/set_int32` / `~/get_string` — 设置/读取 SDK 属性
- `~/reset_camera`、`~/toggle_*` — 流控制

主要参数(launch xml 中可见):
| 参数 | 含义 |
| --- | --- |
| `serial_number` | 多设备时指定 |
| `color_width/height/fps` | 彩色分辨率 |
| `depth_width/height/fps` | 深度分辨率 |
| `ir_width/height/fps` | IR 分辨率 |
| `enable_color/enable_depth/enable_ir/enable_point_cloud/enable_colored_point_cloud` | 流开关 |
| `depth_align` | 深度对齐到彩色 |
| `publish_tf` | 是否发布 TF |
| `tf_publish_rate` | TF 发布频率 |

### `list_devices_node`

列出当前 USB 上识别到的相机及序列号,用于多设备配置。

### `clean_up_shm_node`

清理 SDK 残留的 `/dev/shm` 共享内存。

## 启动文件

每个 launch 对应一种型号(astra、astra_pro、astro_pro_plus、dabai_dc1/dcw/dw/pro/u3、deeyea、embedded_s/u3、gemini/gemini_e/_lite/_arm/_car、stereo_s/_u3)。`multi_*.launch.xml` 系列用于同时启动多台同型号相机。

## 编译与运行

```bash
# 安装依赖
sudo -E apt-get install libuvc-dev
colcon build --packages-up-to astra_camera
source install/setup.bash

# 启动单台 Gemini
ros2 launch astra_camera gemini.launch.xml

# 列设备
ros2 launch astra_camera list_devices.launch.xml
```

## 使用示例

```bash
# 1. 列出设备,确认序列号
ros2 launch astra_camera list_devices.launch.xml

# 2. 启动相机
ros2 launch astra_camera dabai_dcw.launch.xml

# 3. RViz 显示彩色 / 深度 / 点云
ros2 launch wheeltec_rviz2 wheeltec_camera.launch.py
```

## 注意事项

1. 必须先安装 udev 规则(SDK 提供 `99-obsensor-libusb.rules`),否则普通用户无权访问设备。
2. 多设备同时启动时,通过 `serial_number` 区分,避免相互干扰。
3. 高分辨率高帧率会消耗大量 USB 带宽,建议每条 USB 总线只挂一台。
4. 部分型号(Astra Pro)彩色流走 UVC,深度流走 OpenNI2,需保证 `libuvc` 与权限均正常。
5. 推出新 firmware 后请同步升级 SDK,否则可能出现卡帧。

## 在WSL Humble下编译
```bash
> colcon build --packages-select astra_camera
Starting >>> astra_camera
[Processing: astra_camera]                             
[Processing: astra_camera]                                     
[Processing: astra_camera]                                       
--- stderr: astra_camera                                         
gmake[2]: *** No rule to make target '/home/li/Lab/src/ros2_astra_camera-master/astra_camera/openni2_redist/x64/libOpenNI2_astra.so', needed by 'libastra_camera.so'.  Stop.
gmake[2]: *** Waiting for unfinished jobs....
gmake[1]: *** [CMakeFiles/Makefile2:143: CMakeFiles/astra_camera.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< astra_camera [1min 47s, exited with code 2]

Summary: 0 packages finished [1min 48s]
  1 package failed: astra_camera
  1 package had stderr output: astra_camera
```
