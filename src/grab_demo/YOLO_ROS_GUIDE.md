# 官方 yolo_ros 识别 + 闭环抓取使用说明

本方案用 **官方 [yolo_ros](https://github.com/mgonzs13/yolo_ros)（mgonzs13）** 作为 YOLO 识别前端，
替代原来自研的 TensorRT 版 `yolo_detect_node`，并在抓取端引入 **闭环（闭环检测 / 位置闭环视觉伺服 PBVS）**。

> 一句话区别：
> - 旧方案 `yolo_detect_node`：**自研** TensorRT C++，直接加载 `.engine`/`.onnx` 推理，强依赖 CUDA+TensorRT（仅 Jetson）。
> - 新方案 `yolo_ros`：用社区维护的官方包（ultralytics 推理），**不依赖 TensorRT**，CPU/GPU 都能跑，模型切换更方便。

## 一、整体架构

```
                      官方 yolo_ros (yolo_node, ultralytics 推理)
   /camera_arm/color/image_raw  ───────────────►  /yolo/detections (yolo_msgs/DetectionArray)
                                                            │
                          grab_demo: yolo_ros_detect_node.py(桥接)
                          + 深度图 /camera_arm/depth/image_raw
                          + 内参   /gemini_info
                          (选目标 → 中心邻域深度中值 → 针孔反投影 + Z 补偿)
                                                            │
                          持续广播  camera_arm_depth_optical_frame → target_frame
                          + 距离 /grab_target/distance   （与 HSV/KCF/VLM 完全一致）
                                                            │
                          grab_demo: closed_loop_grab_node（闭环抓取）
                          看 target_frame → 移动到预抓取点 → 再看(刷新) → 修正 → 收敛后抓取
```

关键点：**抓取后端读取的依然是统一的 `target_frame`**，所以无论识别用 yolo_ros / HSV / KCF / VLM，抓取流程都不用改。

## 二、Docker（cv + humble）环境准备

在你现有的 **cv + humble** Docker 容器里安装 ultralytics 与依赖：

```bash
# 容器内
pip install ultralytics            # YOLO 推理 (会带上 torch / opencv 等)
# Jetson 上请装对应 JetPack 的 torch 轮子, 然后再 pip install ultralytics --no-deps 之类按需处理
```

> - **有 GPU（Jetson/独显）**：`device:=cuda:0`。
> - **纯 CPU 容器**：`device:=cpu`，并建议用最小模型 `model:=yolov8n.pt`。
> - yolo_ros 默认会自动下载 `yolov8*.pt` 权重；离线环境请提前把权重拷进容器并用绝对路径传给 `model:=`。

## 三、获取 yolo_ros 源码（git submodule）

本仓库已把 yolo_ros 作为 **子模块** 放在 `src/yolo_ros`。首次克隆或拉取后执行：

```bash
cd ~/lebai            # 工作空间根目录
git submodule update --init --recursive
```

这样 `src/yolo_ros` 下会有 `yolo_ros / yolo_msgs / yolo_bringup` 三个 ROS 2 包。

## 四、编译

```bash
cd ~/lebai
# 先编消息接口, 再编其它(避免找不到 yolo_msgs)
colcon build --packages-select yolo_msgs
colcon build --packages-select yolo_ros yolo_bringup grab_demo
source install/setup.bash
```

> - `grab_demo` 的桥接节点 `yolo_ros_detect_node.py` 运行时 `import yolo_msgs`，所以 **必须先编 `yolo_msgs`**。
> - 旧的 `yolo_detect_node`（TensorRT 版）依旧保留，找不到 CUDA/TensorRT 时会自动跳过编译，互不影响。

## 五、运行

整套流程（相机 + 机械臂 + yolo_ros 识别 + 闭环抓取）：

```bash
ros2 launch grab_demo yolo_ros_grab.launch.py
```

常用启动参数：

```bash
# 纯 CPU 容器 + 最小模型
ros2 launch grab_demo yolo_ros_grab.launch.py device:=cpu model:=yolov8n.pt

# 只抓“瓶子”(COCO id=39)，并选离相机最近的那个
ros2 launch grab_demo yolo_ros_grab.launch.py target_class:=39 select_mode:=nearest

# 用类名筛选(不区分大小写)，优先级高于 target_class
ros2 launch grab_demo yolo_ros_grab.launch.py target_label:=cup
```

只单独调试识别（不启动机械臂）时，可分别跑 yolo_ros 与桥接：

```bash
# 1) 官方 yolo_ros
ros2 launch yolo_bringup yolo.launch.py \
  model:=yolov8n.pt device:=cpu use_3d:=False use_tracking:=False \
  input_image_topic:=/camera_arm/color/image_raw

# 2) 桥接节点
ros2 run grab_demo yolo_ros_detect_node.py --ros-args \
  -p detections_topic:=/yolo/detections \
  -p select_mode:=confidence

# 3) 看检测可视化(带距离)
ros2 run rqt_image_view rqt_image_view /yolo_ros_node/detection_image
# 确认 TF
ros2 run tf2_ros tf2_echo camera_arm_depth_optical_frame target_frame
```

## 六、闭环（闭环检测）是怎么做的

`closed_loop_grab_node` 把原来 `grab_service_node` 的 **一次性开环**（查一次 TF → 规划一次 → 执行一次）
改成 **位置闭环（PBVS）**：

1. 张开夹爪，移动到观察点 `look`；
2. **看**：查最新 `base_link → target_frame`（识别节点以相机帧率持续刷新它）；
3. **动**：移动到目标正上方的预抓取点（hover，高度 `approach_height`）；
4. **再看**：在新视角下重新检测，目标位置被刷新；
5. **判收敛**：比较本轮与上一轮目标位置，若位移 < `pos_tolerance` 认为已对准，否则带修正量再来一轮（最多 `max_iters` 轮）；
6. 收敛后下降到抓取点（`grasp_z_offset`），闭合夹爪，退回 `look`。

这样即便存在手眼标定误差、深度噪声或物体被轻微移动，机械臂也能在靠近过程中不断修正，显著提升抓取成功率。

### 闭环参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `base_frame` | `base_link` | 抓取参考坐标系 |
| `look_target` | `look` | SRDF 里预设的观察点名 |
| `max_iters` | `4` | 最多修正轮数 |
| `pos_tolerance` | `0.008` | 目标位移收敛阈值（m），小于它视为对准 |
| `approach_height` | `0.10` | 预抓取悬停高度（m） |
| `grasp_z_offset` | `0.02` | 最终下降补偿（m，与开环版一致） |
| `settle_sec` | `0.6` | 每次移动后等识别刷新的时间（s） |

> 想用回开环抓取，直接用 `yolo_grab.launch.py` 或把 launch 里的 `closed_loop_grab_node` 换成 `grab_service_node` 即可（二者服务名都是 `obj_grab_service`）。

## 七、桥接节点（yolo_ros_detect_node.py）参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `detections_topic` | `/yolo/detections` | yolo_ros 输出的检测话题（`yolo_msgs/DetectionArray`）；想用带 id 的跟踪结果可改成 `/yolo/tracking` 并在 yolo_ros 开 `use_tracking:=True` |
| `rgb_topic` | `/camera_arm/color/image_raw` | 仅用于画调试图 |
| `depth_topic` | `/camera_arm/depth/image_raw` | 深度（16UC1，mm） |
| `camera_info_topic` | `/gemini_info` | 相机内参 |
| `camera_frame` | `camera_arm_depth_optical_frame` | TF 父系 |
| `target_frame` | `target_frame` | TF 子系（抓取服务读取的就是它） |
| `z_offset` | `0.07` | Z 补偿（m，与 HSV/YOLO 一致） |
| `target_class` | `-1` | 只抓某 COCO id，`-1`=不限（瓶子=39，杯子=41） |
| `target_label` | `""` | 只抓某类名（不区分大小写），非空时优先于 `target_class` |
| `select_mode` | `confidence` | 多目标选择：`confidence`/`nearest`/`center`/`largest` |
| `min_dist` / `max_dist` | `0.1` / `2.0` | 允许的目标距离范围（m），越界视为无效 |
| `publish_debug_image` | `True` | 发布带框+距离的调试图到 `~/detection_image` |

## 八、和旧 TensorRT 版（yolo_detect_node）的对照

| 项目 | 旧 `yolo_detect_node` | 新 `yolo_ros` + 桥接 |
|------|----------------------|---------------------|
| 推理后端 | 自研 TensorRT C++ | 官方 yolo_ros（ultralytics） |
| 硬件依赖 | 必须 CUDA + TensorRT（Jetson） | CPU/GPU 皆可 |
| 模型格式 | `.onnx` / `.engine` | `.pt`（ultralytics），含 v5~v12 / yolo-world |
| 跟踪 | 无 | yolo_ros 自带 ByteTrack（`use_tracking:=True`） |
| 输出接口 | `target_frame` + `/grab_target/distance` | **完全相同** |
| 抓取 | 开环 `grab_service_node` | 闭环 `closed_loop_grab_node` |
