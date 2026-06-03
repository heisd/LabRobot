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
| 反投影 | 针孔模型 + Z 补偿 0.07m |

所以 `grab_service_node` 对 HSV/YOLO/KCF 一视同仁，无需改动。

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

两种方式：

1. **参数**（headless，推荐）：用上面的 `-p hue_min:=...` 等设置。
2. **实时滑动条窗口**（需要显示器）：把 `show_image:=true`，会弹出 "HSV Tuning"
   滑动条窗口 + "HSV Detection"/"HSV Mask" 预览，等价旧版的 trackbar 工作流。

   ```bash
   ros2 run grab_demo hsv_range --ros-args -p show_image:=true
   ```

## 五、参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `rgb_topic` / `depth_topic` / `camera_info_topic` | 同 YOLO/KCF | 相机话题 |
| `camera_frame` / `target_frame` | `camera_arm_depth_optical_frame` / `target_frame` | TF 坐标系 |
| `z_offset` | `0.07` | Z 补偿（m） |
| `hue_min/max`、`sat_min/max`、`val_min/max` | 偏红 | HSV 阈值（hue 0~255） |
| `min_area` | `200` | 色块最小面积（像素），过滤噪点 |
| `show_image` | `false` | 是否弹滑动条/预览窗口（默认否，headless 安全） |
| `publish_debug_image` | `true` | 是否发布可视化到 `~/detection_image` |
