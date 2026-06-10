# HSV 颜色检测抓取使用说明（整理重写版）

`hsv_range`（节点 `hsv_range_node`）按 HSV 颜色阈值找最大色块并抓取，是 YOLO / KCF 之外
最轻量的视觉算法。本版本已整理重写，修复了旧版的结构性 bug，并接入统一接口。

## 一、相比旧版的修复

- **同步器修复**：彩色+深度的 `message_filters` 同步器现在在构造函数里正确建立
  （旧版误放在回调内部，导致回调从不触发 / 反复重建）。
- **内参订阅修复**：旧版 `mera_info_sub` 拼写错误（未声明变量）已修正。
- **统一接口**：深度读取 + 针孔反投影 + 广播 `target_frame` 改用共享的
  `grab_demo::TargetTFPublisher`（与 YOLO/KCF 同一实现）。
- **默认 headless**：默认不弹任何窗口；HSV 阈值改为 ROS 参数。

## 二、统一接口（和 YOLO/KCF 完全一致）

| 项目 | 说明 |
|------|------|
| 彩色话题 | `/camera_arm/color/image_raw` |
| 深度话题 | `/camera_arm/depth/image_raw` (16UC1, mm) |
| 内参话题 | `/gemini_info` |
| 输出 TF | `camera_arm_depth_optical_frame → target_frame` |
| 反投影 | 针孔模型 + Z 补偿 0.07m（`z_offset` 运行时可调） |
| 距离 | 发布到 `/grab_target/distance` (std_msgs/Float32, 米)，并叠加到调试图 `dis=…m` |

所以抓取服务对 HSV/YOLO/KCF 一视同仁，无需改动。

> 本节点已按 **yolo_ros 方案的同一思路** 升级（详见 `YOLO_ROS_GUIDE.md`）：
> ① 抓取改为**闭环**（`color_grab.launch.py` 默认用 `closed_loop_grab_node`，看-动-再看-修正）；
> ② HSV 阈值与 `z_offset`（抓取深度）**运行时可调**（`ros2 param set` 即时生效，无需弹窗）；
> ③ 加了**异常与帧流健康日志**（回调异常捕获计数、长时间无帧 ERROR 告警）；
> ④ 接入 **Dashboard 参数面板**（`/color_node` 的 “HSV 颜色” 分组，在线拖阈值）。

## 三、运行

整套（相机 + 机械臂 + HSV + 抓取服务）：

```bash
ros2 launch grab_demo color_grab.launch.py
```

单独调试：

```bash
ros2 run grab_demo hsv_range --ros-args \
  -p hue_min:=0 -p hue_max:=10 -p sat_min:=100 -p val_min:=100

# 看检测可视化(不弹窗, 走话题; launch 里节点名是 color_node)
ros2 run rqt_image_view rqt_image_view /color_node/detection_image
```

## 四、调阈值

三种方式：

1. **运行时 ROS 参数（推荐，headless，即时生效）**：节点支持动态调参，无需重启：
   ```bash
   ros2 param set /color_node hue_min 100     # 改成抓蓝色物体
   ros2 param set /color_node hue_max 130
   ros2 param set /color_node min_area 800
   ros2 param set /color_node z_offset 0.10    # 抓更深
   ```
   或直接在 **Dashboard** 监控页的【HSV 颜色】分组里拖滑条。
2. **启动参数**：用 `-p hue_min:=...` 等在启动时设置。
3. **实时滑动条窗口**（需要显示器）：`show_image:=true`，弹出 "HSV Tuning" 滑动条 +
   "HSV Detection"/"HSV Mask" 预览（等价旧版 trackbar）。注意：滑条与 `ros2 param set`
   都会改同一组阈值，二者可并用。

   ```bash
   ros2 run grab_demo hsv_range --ros-args -p show_image:=true
   ```

## 五、参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `rgb_topic` / `depth_topic` / `camera_info_topic` | 同 YOLO/KCF | 相机话题 |
| `camera_frame` / `target_frame` | `camera_arm_depth_optical_frame` / `target_frame` | TF 坐标系 |
| `z_offset` | `0.07` | Z 补偿（m），沿相机光轴；**运行时可调**，越大抓得越深 |
| `hue_min/max`、`sat_min/max`、`val_min/max` | 偏红 | HSV 阈值（hue 0~255）；**运行时可调** |
| `min_area` | `200` | 色块最小面积（像素），过滤噪点；**运行时可调** |
| `show_image` | `false` | 是否弹滑动条/预览窗口（默认否，headless 安全） |
| `publish_debug_image` | `true` | 是否发布可视化到 `~/detection_image` |

## 六、闭环抓取 与 健康日志

- `color_grab.launch.py` 默认启动**闭环抓取** `closed_loop_grab_node`（看-动-再看-修正后再抓），
  与 yolo_ros / KCF 一致；想用回开环把它换成 `grab_service_node` 即可（服务名一致）。
- 节点对回调异常做了捕获（OpenCV/一般异常都会打印并计数，不会静默失效）；
  **帧流看门狗**：长时间收不到图像帧会 `ERROR` 告警，便于发现相机掉线/话题不匹配。

## 七、在 Dashboard 里使用

- **功能启动**页 →【HSV/颜色 抓取】启动整套流程。
- **监控**页 →【YOLO / KCF / HSV 识别 + 闭环抓取 参数】卡片的 **HSV 颜色** 分组，
  在线拖 `z_offset` / `hue_min/max` / `sat_min` / `val_min` / `min_area`，即时生效。
