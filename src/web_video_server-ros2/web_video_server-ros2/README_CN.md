# web_video_server (ROS2)

ROS2 图像话题 → HTTP 视频流 转换服务,把任意 `sensor_msgs/Image`、`CompressedImage` 通过浏览器以 MJPEG / H.264 / VP8 / VP9 / PNG 等格式实时查看。

## 概述

启动后会在指定端口起一个 HTTP server,提供:
- `/` — 索引页,列出可用图像话题
- `/stream?topic=/camera/color/image_raw&type=mjpeg` — 选定话题流
- 单图、压缩流、H264/VP8/VP9 等多种 streamer

适合远程查看机器人摄像头、调试视觉算法。

## 目录结构

```
web_video_server-ros2/
├── CMakeLists.txt
├── package.xml
├── README.md / CHANGELOG.rst / LICENSE / AUTHORS.md
├── include/
└── src/
    ├── web_video_server.cpp           # 主节点
    ├── image_streamer.cpp             # 抽象流
    ├── jpeg_streamers.cpp / png_streamers.cpp
    ├── h264_streamer.cpp / vp8_streamer.cpp / vp9_streamer.cpp / libav_streamer.cpp
    ├── ros_compressed_streamer.cpp    # 直接转发 CompressedImage
    └── multipart_stream.cpp           # MJPEG multipart 实现
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`sensor_msgs`、`image_transport`、`cv_bridge`、`OpenCV`、`async_web_server_cpp`、`Boost`、`libav`(可选,用于 H264/VP8/VP9)

## 节点说明

### `web_video_server`

订阅:
- 自动发现所有 `sensor_msgs/Image` 和 `sensor_msgs/CompressedImage` 话题

HTTP 路径:
- `GET /` — 自动生成的话题列表 + 在线预览
- `GET /stream?topic=<TOPIC>&type=<mjpeg|ros_compressed|png|h264|vp8|vp9>&quality=<0-100>&width=...&height=...`
- `GET /snapshot?topic=<TOPIC>` — 单帧图像
- `GET /stream_viewer?topic=<TOPIC>` — 简易 HTML 预览页

参数:
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `port` | `8080` | HTTP 端口 |
| `address` | `0.0.0.0` | 绑定地址 |
| `server_threads` | 内部默认 | HTTP 线程数 |
| `ros_threads` | 内部默认 | ROS 线程 |
| `default_stream_type` | `mjpeg` | 默认编码 |
| `verbose` | `false` | 是否打印详细日志 |

## 编译与运行

```bash
colcon build --packages-up-to web_video_server
source install/setup.bash
ros2 run web_video_server web_video_server --ros-args -p port:=8080
```

## 使用示例

```bash
# 1. 启动相机
ros2 launch usb_cam usb_cam_launch.py

# 2. 启动 web_video_server
ros2 run web_video_server web_video_server

# 3. 浏览器打开
#   http://<机器人IP>:8080/                  # 索引页
#   http://<机器人IP>:8080/stream?topic=/image_raw&type=mjpeg
```

## 注意事项

1. 默认绑定 `0.0.0.0`,会暴露到所有网卡,生产环境请绑定内网 IP 或加防火墙。
2. H264/VP8/VP9 依赖 libav,不同发行版可能需要额外安装 `libavcodec-dev` 等。
3. MJPEG 兼容性最好,适合默认使用;延迟取决于网络带宽。
4. 多客户端同时观看会按订阅复用同一份图像,延迟会略增加。
