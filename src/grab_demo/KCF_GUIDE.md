# KCF 跟踪抓取使用说明

`kcf_track_node` 是基于 **KCF (Kernelized Correlation Filter)** 的视觉**跟踪**节点，
作为 HSV / YOLO 之外的第三种视觉算法，接入同一套抓取流程。

## 一、KCF 和 HSV/YOLO 的区别

- HSV、YOLO 是**逐帧检测**：每一帧独立地找目标。
- KCF 是**跟踪**：需要先有一个"初始目标框"，之后逐帧跟踪同一个物体。
  优点是跟踪连续、对运动/部分遮挡更稳、计算量小；缺点是要先给初始框。

本节点的初始框来源（按优先级）：
1. 参数 `init_bbox = [x, y, w, h]`（`w,h>0` 时用，作为第一次播种）；
2. 否则用 **HSV 颜色阈值找最大色块自动播种**（无需手动框选，headless 友好）；
3. 运行中可调用 `~/reinit` 服务随时重新播种；跟丢时（`reinit_on_loss=true`）自动回到 HSV 重新播种。

## 二、统一接口（和 HSV/YOLO 完全一致）

| 项目 | 说明 |
|------|------|
| 彩色话题 | `/camera_arm/color/image_raw` |
| 深度话题 | `/camera_arm/depth/image_raw` (16UC1, mm) |
| 内参话题 | `/gemini_info` |
| 输出 TF | `camera_arm_depth_optical_frame → target_frame` |
| 反投影 | 针孔模型 + Z 补偿 0.07m |
| 距离 | 发布到 `/grab_target/distance` (std_msgs/Float32, 米)，并叠加到调试图 `dis=…m` |

三种算法都通过共享的 **`grab_demo::TargetTFPublisher`**（`include/grab_demo/target_tf_publisher.hpp`）
把"目标像素中心"统一转换成 `target_frame`，所以 `grab_service_node` 对 HSV/YOLO/KCF 一视同仁，无需改动。

> KCF 默认 **不弹窗**（`show_image=false`）；可用 `rqt_image_view` 订阅 `/kcf_node/tracking_image` 看跟踪框。

## 三、编译

```bash
cd ~/lebai
colcon build --packages-select grab_demo
source install/setup.bash
```

> KCF 需要 **opencv_contrib 的 tracking 模块**（`cv::TrackerKCF`，头文件 `opencv2/tracking.hpp`）。
> CMake 会自动探测：找不到就跳过 `kcf_track_node` 的编译并打印 WARNING，不影响其它节点。
> 若缺失，在 Ubuntu 上安装：`sudo apt install libopencv-contrib-dev`（Jetson 上需确认 OpenCV 编译时带了 contrib）。

## 四、运行

整套（相机 + 机械臂 + KCF + 抓取服务）：

```bash
ros2 launch grab_demo kcf_grab.launch.py
```

单独调试跟踪节点：

```bash
ros2 run grab_demo kcf_track_node --ros-args \
  -p init_bbox:="[300,200,80,80]"       # 指定初始框; 不指定则 HSV 自动播种

# 查看跟踪可视化
ros2 run rqt_image_view rqt_image_view /kcf_node/tracking_image
# 重新播种(更换跟踪目标)
ros2 service call /kcf_node/reinit std_srvs/srv/Trigger
```

## 五、参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `rgb_topic` / `depth_topic` / `camera_info_topic` | 同 HSV/YOLO | 相机话题 |
| `camera_frame` / `target_frame` | `camera_arm_depth_optical_frame` / `target_frame` | TF 坐标系 |
| `z_offset` | `0.07` | Z 补偿（m），与 HSV/YOLO 一致 |
| `init_bbox` | `[0,0,0,0]` | 初始框 `[x,y,w,h]`，全 0 表示用 HSV 自动播种 |
| `hue_min/max`、`sat_min/max`、`val_min/max` | 偏红 | HSV 自动播种阈值 |
| `min_area` | `400` | HSV 播种时色块最小面积（像素） |
| `reinit_on_loss` | `true` | 跟丢时是否自动用 HSV 重新播种 |
| `show_image` | `false` | 是否弹 OpenCV 窗口（默认否，headless 安全） |
| `publish_debug_image` | `true` | 是否发布跟踪可视化到 `~/tracking_image` |

## 六、在 Dashboard 里使用

打开 Dashboard 的"功能启动"页，**视觉抓取**分组里新增了 **KCF 跟踪抓取**，
点"启动"即可。它和 YOLO/HSV/ArUco 互斥（共享相机+机械臂+MoveIt），
启动其一后其余会自动禁用，换算法时先停掉当前的即可。
