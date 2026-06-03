# YOLO (TensorRT) 抓取识别使用说明

`yolo_detect_node` 是基于 **TensorRT** 的 YOLOv8 目标检测节点，用来替代原来的
HSV 颜色识别 (`hsv_range`)。两者的**相机接口和输出完全一致**，所以
`grab_service_node` 不需要任何改动就能复用。

## 一、和 HSV 的对齐关系

| 项目 | HSV (`hsv_range`) | YOLO (`yolo_detect_node`) |
|------|-------------------|---------------------------|
| 彩色话题 | `/camera_arm/color/image_raw` | `/camera_arm/color/image_raw` |
| 深度话题 | `/camera_arm/depth/image_raw` | `/camera_arm/depth/image_raw` |
| 内参话题 | `/gemini_info` | `/gemini_info` |
| 输出 TF | `camera_arm_depth_optical_frame` → `target_frame` | 同左 |
| 反投影 | 针孔模型 + Z 补偿 0.07m | 同左 |
| 目标点 | 最大轮廓的质心 | 检测框中心(中心邻域深度中值，更稳) |
| 弹窗 | 会弹 `imshow` 窗口 | **默认不弹**，可选发布调试图像话题 |

> 关于"会不会弹提示框"：本节点默认 `show_image=false`，**不会弹任何窗口**，
> 在 Jetson 无显示器时也不会报 GTK/X11 错误。需要看效果时把 `show_image` 设为
> `true`，或保持默认并用 `rqt_image_view` 订阅 `/yolo_node/detection_image`。

## 二、准备模型 (yolov8n, 通用 COCO)

在装有 ultralytics 的机器上导出 ONNX：

```bash
pip install ultralytics
yolo export model=yolov8n.pt format=onnx opset=12   # 生成 yolov8n.onnx
```

把 `yolov8n.onnx` 拷到 Jetson 上，默认放在 `~/models/`：

```bash
mkdir -p ~/models
cp yolov8n.onnx ~/models/
```

引擎(engine)有两种获得方式，任选其一：

- **方式 A（推荐，自动）**：直接运行节点。首次启动若找不到 `~/models/yolov8n.engine`，
  会自动用 onnx 构建并缓存(首次较慢，之后秒启)。
- **方式 B（手动 trtexec）**：

  ```bash
  /usr/src/tensorrt/bin/trtexec \
    --onnx=$HOME/models/yolov8n.onnx \
    --saveEngine=$HOME/models/yolov8n.engine \
    --fp16
  ```

## 三、编译

```bash
cd ~/lebai        # 你的工作空间根目录
colcon build --packages-select grab_demo
source install/setup.bash
```

> CMake 会自动探测 CUDA + TensorRT。**找不到就跳过** `yolo_detect_node`
> 的编译（不影响其它节点），并打印一条 WARNING。所以在没有 GPU 的开发机上
> 也能正常构建整个包。

## 四、运行

整套流程（相机 + 机械臂 + YOLO 识别 + 抓取服务）：

```bash
ros2 launch grab_demo yolo_grab.launch.py
```

只单独跑识别节点调试：

```bash
ros2 run grab_demo yolo_detect_node \
  --ros-args \
  -p onnx_path:=$HOME/models/yolov8n.onnx \
  -p engine_path:=$HOME/models/yolov8n.engine \
  -p target_class:=-1 \
  -p show_image:=false
```

查看检测可视化（不弹窗，走话题）：

```bash
ros2 run rqt_image_view rqt_image_view /yolo_node/detection_image
```

确认 TF 是否正常输出：

```bash
ros2 run tf2_ros tf2_echo camera_arm_depth_optical_frame target_frame
```

## 五、参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `engine_path` | `""` | TensorRT engine 路径；存在则直接加载 |
| `onnx_path` | `""` | onnx 路径；engine 不存在时用它构建并缓存到 `engine_path` |
| `rgb_topic` | `/camera_arm/color/image_raw` | 彩色话题（与 HSV 一致） |
| `depth_topic` | `/camera_arm/depth/image_raw` | 深度话题（16UC1，mm） |
| `camera_info_topic` | `/gemini_info` | 相机内参话题 |
| `camera_frame` | `camera_arm_depth_optical_frame` | TF 父坐标系 |
| `target_frame` | `target_frame` | TF 子坐标系（抓取服务读取的就是它） |
| `target_class` | `-1` | `-1`=任意类别取最高分；指定 COCO id 只抓该类。瓶子=39，杯子=41 |
| `conf_threshold` | `0.25` | 置信度阈值 |
| `nms_threshold` | `0.45` | NMS IoU 阈值 |
| `z_offset` | `0.07` | Z 轴补偿，单位 m（与 HSV 一致，按实际标定调） |
| `show_image` | `false` | 是否弹 OpenCV 窗口（默认否，headless 安全） |
| `publish_debug_image` | `true` | 是否发布带框的调试图像到 `~/detection_image` |

## 六、只想抓某一类物体

比如只抓"瓶子"（COCO 类 id = 39）：

```bash
ros2 run grab_demo yolo_detect_node --ros-args -p target_class:=39 ...
```

常用 COCO id：person=0, bottle=39, wine glass=40, cup=41, bowl=45,
banana=46, apple=47, orange=49, scissors=76。
