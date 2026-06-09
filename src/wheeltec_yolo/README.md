# wheeltec_yolo

把 [yolo_ros](https://github.com/mgonzs13/yolo_ros)（mgonzs13，Ultralytics YOLO 的 ROS 2 封装）
集成到 Wheeltec S300：用 Wheeltec 相机做 YOLO 检测 / 跟踪，结果直接喂给 Web 仪表盘的
**「YOLO 检测」** 页。

本包只是一个 **便捷启动包**：把上游 `yolo_bringup/yolo.launch.py` 的参数换成
Wheeltec 默认值后转发。上游源码 **不并入本仓库**，用 `vcs` 按需拉取。

## 1. 拉取 yolo_ros 源码（vcs）

```bash
sudo apt install python3-vcstool          # 没有 vcs 时
cd <workspace_root>                        # 含 src/ 的工作区根目录
vcs import src < yolo_ros.repos            # -> src/yolo_ros (yolo_msgs / yolo_ros / yolo_bringup)
# 以后更新：vcs pull src
```

> 想要可复现的机器人构建，把 `yolo_ros.repos` 里的 `version: main` 钉到某个 tag / commit。

## 2. 安装推理依赖（ultralytics）

目标设备：**NVIDIA GPU / Jetson**（`device:=cuda:0`）。

- **独立显卡**：
  ```bash
  pip3 install ultralytics
  ```
- **Jetson**：先装好 JetPack 对应版本的 PyTorch（**不要**直接 `pip install torch`，
  否则装成 CPU 轮子），再：
  ```bash
  pip3 install ultralytics --no-deps
  pip3 install numpy pillow pyyaml requests   # 缺啥补啥, torch/torchvision 用 JetPack 的
  ```

首次运行会自动下载 `yolov8n.pt` 权重（需联网；离线则预先放好权重文件并用
`model:=/abs/path/yolov8n.pt`）。

## 3. 安装 ROS 依赖并编译

```bash
cd <workspace_root>
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to wheeltec_yolo
source install/setup.bash
```

## 4. 启动

先确保相机在跑（发布 `/camera/color/image_raw`），然后：

```bash
ros2 launch wheeltec_yolo yolo.launch.py
# 覆盖示例：
ros2 launch wheeltec_yolo yolo.launch.py device:=cpu model:=yolo11n.pt threshold:=0.4
ros2 launch wheeltec_yolo yolo.launch.py input_image_topic:=/usb_cam/image_raw
```

可覆盖参数：`input_image_topic` `model` `device` `threshold`
`use_tracking` `use_debug` `use_3d` `namespace`。

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/yolo/detections` | `yolo_msgs/DetectionArray` | 类别 `class_name` / 置信度 `score` / 2D 框 |
| `/yolo/tracking`   | `yolo_msgs/DetectionArray` | 带跟踪 `id`（`use_tracking`） |
| `/yolo/debug_image`| `sensor_msgs/Image`        | 标注可视化（仪表盘预览，`use_debug`） |

## 仪表盘

功能模块 → **YOLO 检测**。图像话题默认 `/yolo/debug_image`，检测话题默认
`/yolo/detections`；列表显示类别 + 置信度，开启跟踪时附带 `#id` 徽标。把检测话题
改成 `/yolo/tracking` 即可看带稳定 ID 的跟踪结果。

## 5. YOLO 3D 跟随（保持 ~20cm）

用相机的**深度**把 2D 检测投影成 3D（`/yolo/detections_3d`，坐标在 `base_link`），
`yolo_follow` 节点据此让小车跟随物体并保持设定距离。

### 前置：深度必须与彩色对齐
yolo_ros 的 3D 节点把**彩色图**上的检测框投到**深度图**取距离，所以深度要与彩色
配准。Astra 默认 `depth_registration:=false`，启动相机时要打开：

```bash
ros2 launch astra_camera <你的相机>.launch.xml depth_registration:=true
```

确认有这两个话题：`/camera/depth/image_raw`、`/camera/depth/camera_info`。

### 启动

```bash
ros2 launch wheeltec_yolo yolo_follow.launch.py
# 只跟人、保持 0.3m：
ros2 launch wheeltec_yolo yolo_follow.launch.py target_class:=person desired_distance:=0.3
# 速度交给仲裁器而不是直接给底盘：
ros2 launch wheeltec_yolo yolo_follow.launch.py cmd_vel_topic:=follow/cmd_vel
```

### `yolo_follow` 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `detections_topic` | `/yolo/detections_3d` | 3D 检测输入 |
| `target_class` | `` (空=任意) | 只跟某类，如 `person` |
| `desired_distance` | `0.20` | 保持的距离（米，base_link 下测量） |
| `distance_deadband` | `0.03` | 距离死区（米） |
| `yaw_deadband` | `0.08` | 方位角死区（弧度） |
| `kp_linear` / `kp_angular` | `0.6` / `1.5` | 前后 / 转向比例增益 |
| `max_linear` / `max_angular` | `0.15` / `0.6` | 限速（m/s、rad/s） |
| `lost_timeout` | `0.5` | 丢目标多久后停车（秒） |

控制：前后让 `bbox3d.center.position.x` 收敛到 `desired_distance`，转向让方位角
`atan2(y, x)` 归零（目标居中）；单帧漏检在 `lost_timeout` 内沿用上次目标，丢失后停车。

### 日志 / 排查

节点会打印诊断日志，出问题时按提示定位：

| 日志 | 含义 / 处理 |
|---|---|
| `锁定目标: class=… 距离=…m` | 已开始跟随某物体 |
| `跟随 … 距离=…m 方位=…° -> v=… w=…` | 跟随中（每秒一条） |
| `目标丢失, 停车` | 超过 `lost_timeout` 没看到目标 |
| `未检测到任何物体` | YOLO 没检到东西（光线/距离/模型） |
| `检测到 N 个物体, 但没有类别 "X"` | `target_class` 设的类别没出现 |
| **`N 个候选都没有有效 3D 深度 …`** | **最常见**：深度没和彩色对齐 → 相机加 `depth_registration:=true`，并确认深度话题在发布 |
| `尚未收到 /yolo/detections_3d …` | 3D 检测没起来：确认用的是 `yolo_follow.launch.py`（已开 `use_3d`） |
| `/yolo/detections_3d 超过 2s 未更新` | 检测中断：查相机 / 深度 / yolo 节点 |

把日志等级调高看更多：`ros2 run`/`launch` 时加 `--ros-args --log-level yolo_follow:=debug`。

> 距离在 `base_link`（车体中心）下测量。相机有前向安装偏移时，**车头**到物体的实际
> 间隙 ≈ `desired_distance − 相机前向偏移`，可据此把 `desired_distance` 调到让车头
> 真正离物体约 20cm。
