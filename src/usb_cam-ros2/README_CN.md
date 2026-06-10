# usb_cam (ROS2)

通用 USB / UVC 摄像头 ROS2 驱动包(`ros-drivers/usb_cam` 上游版本),将 V4L2 设备输出转换为标准 `sensor_msgs/Image` 与 `CameraInfo`。

## 概述

支持 YUYV / MJPEG / RGB24 / GREY 等多种像素格式,可调节分辨率、帧率、亮度、对比度、曝光、白平衡等参数,适配几乎所有 V4L2 兼容相机。

## 目录结构

```
usb_cam-ros2/
├── CMakeLists.txt
├── package.xml
├── README.md / CHANGELOG.rst / AUTHORS.md / LICENSE
├── include/
├── src/                              # 驱动源码
├── msg/                              # 自定义消息(如 Format)
├── config/
│   ├── params.yaml                   # 默认参数
│   └── camera_info.yaml              # 默认相机内参
├── scripts/
├── test/
└── launch/
    ├── usb_cam_launch.py
    └── demo.launch.py                # 启动 + image_view
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`rclcpp_components`、`sensor_msgs`、`std_msgs`、`std_srvs`、`builtin_interfaces`、`image_transport`、`camera_info_manager`、`cv_bridge`、`OpenCV`、`v4l-utils`、`ffmpeg`

## 节点说明

### `usb_cam_node_exe`(可执行) / `usb_cam::UsbCamNode`(组件)

发布:
- `/image_raw` (sensor_msgs/Image)
- `/image_raw/compressed`、`/image_raw/theora` 等 `image_transport` 后续话题
- `/camera_info` (sensor_msgs/CameraInfo)

服务:
- `~/set_capture_format`、`~/start_capture`、`~/stop_capture`

主要参数(`config/params.yaml`):
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `video_device` | `/dev/video9` | V4L2 设备路径 |
| `framerate` | `30.0` | 帧率 |
| `io_method` | `mmap` | I/O 方式(`mmap`/`read`/`userptr`) |
| `frame_id` | `camera` | 输出 frame |
| `pixel_format` | `mjpeg2rgb` | 输入格式或自动转换格式 |
| `image_width` | `640` | 分辨率 |
| `image_height` | `480` | 分辨率 |
| `camera_name` | `test_camera` | CameraInfoManager 名称 |
| `camera_info_url` | `package://usb_cam/config/camera_info.yaml` | 内参文件 |
| `brightness/contrast/saturation/sharpness/gain` | `-1` | -1 表示驱动自动 |
| `auto_white_balance/white_balance` | `true / 4000` | 白平衡 |
| `autoexposure/exposure` | `true / 100` | 曝光 |
| `autofocus/focus` | `false / -1` | 对焦 |

## 启动文件

- **`usb_cam_launch.py`** — 启动驱动节点,从 `config/params.yaml` 加载参数。
- **`demo.launch.py`** — 启动驱动并附带 `image_view`/`rqt_image_view` 显示。

## 编译与运行

```bash
colcon build --packages-up-to usb_cam
source install/setup.bash

ros2 launch usb_cam usb_cam_launch.py
```

## 使用示例

```bash
# 1. 确认设备
v4l2-ctl --list-devices

# 2. 修改 config/params.yaml 中 video_device 为实际设备(如 /dev/video0)

# 3. 启动
ros2 launch usb_cam usb_cam_launch.py
ros2 run rqt_image_view rqt_image_view /image_raw
```

## 注意事项

1. 默认 `video_device: /dev/video9` 是 WheelTec 的预设设备号,实际请按 `v4l2-ctl --list-devices` 调整。
2. 使用 `mjpeg2rgb` 时必须确保相机支持 MJPEG,否则切换为 `yuyv2rgb`。
3. `camera_info_url` 指向的 yaml 需提前标定,否则发布的 `CameraInfo` 不正确。
4. 多相机同时启动时,需要在不同 namespace 下运行并指定不同 `video_device`。
