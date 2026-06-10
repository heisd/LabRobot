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

### 运行时调节 Z 轴 / 抓取深度（抓不同深度的物体）

有两个 Z 偏移共同决定"抓多深"，都支持 **运行中用 `ros2 param set` 即时调整，无需重启**：

| 参数 | 节点 | 方向 | 作用 |
|------|------|------|------|
| `z_offset` | 桥接 `yolo_ros_node` | 沿**相机光轴**（深度方向） | `z = 实测深度 + z_offset`。调大 → 目标点更深入物体内部；调小 / 负值 → 更靠近物体表面。适配不同**厚度/远近**的物体 |
| `grasp_z_offset` | 闭环 `grab_service_n` | 沿 **base_link 的 Z**（竖直） | 最终下降到 `目标z + grasp_z_offset`。调它控制夹爪竖直方向"压"多深 |
| `approach_height` | 闭环 `grab_service_n` | 沿 base_link Z（竖直） | 预抓取悬停高度，越大越安全、越小越快 |

示例（抓更深 / 更浅的物体）：

```bash
# 让目标点沿相机视线再往里 3cm(抓更靠后的厚物体)
ros2 param set /yolo_ros_node z_offset 0.10

# 抓很薄的物体, 几乎贴表面
ros2 param set /yolo_ros_node z_offset 0.02

# 最终竖直下降再多压 1cm
ros2 param set /grab_service_n grasp_z_offset 0.03

# 预抓取悬停降到 6cm(物体矮、空间紧时)
ros2 param set /grab_service_n approach_height 0.06

# 查看当前值
ros2 param get /yolo_ros_node z_offset
ros2 param list /grab_service_n
```

> 提示：闭环抓取节点在**一次抓取进行中**会阻塞参数服务，请在两次抓取之间设置参数（设置会在下次抓取生效）。桥接节点的 `z_offset` 则任何时候都即时生效。

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
| `center_mode` | `bbox` | 抓取中心：`bbox`=检测框几何中心；`mask`=分割掩码质心（不规则物体，需 `-seg` 模型） |
| `conf_threshold` | `0.0` | 桥接端**额外**置信度门槛，低于它的检测不抓；`0`=只用 yolo 的 `threshold` |
| `min_dist` / `max_dist` | `0.1` / `2.0` | 允许的目标距离范围（m），越界视为无效 |
| `publish_debug_image` | `True` | 发布带框+距离的调试图到 `~/detection_image` |

> 上表参数全部支持运行时 `ros2 param set /yolo_ros_node <名> <值>` 即时调整，并都有默认值。

## 八、抓取中心怎么算 & 不规则物体 & 置信度门槛

### 抓取中心怎么算

默认（`center_mode:=bbox`）：取**检测框的几何中心** `bbox.center`，再在该像素周围 11×11 邻域里取**非零深度的中值**作为距离，最后用针孔模型反投影成相机系 3D 点。取中值而不是单点，是为了抗深度噪声/空洞。

### 遇到不规则物体怎么办

对 L 形、环形、香蕉等不规则物体，检测框中心可能**不落在物体上**。这时用分割模型 + 掩码质心：

1. 模型换成分割版（文件名带 `-seg`），它会额外输出每个实例的掩码：
   ```bash
   ros2 launch grab_demo yolo_ros_grab.launch.py model:=yolov8n-seg.pt center_mode:=mask
   ```
2. 桥接节点会改用**掩码多边形的质心**作为抓取中心，并用**整个掩码区域内的深度中值**作为距离（比固定小窗更稳）。
   对 L 形/带孔等**凹形**物体，若质心落在了物体之外（掩码外），会自动改取掩码内**距边界最远的点**（距离变换峰值），确保抓取中心始终落在物体上。
3. 若某个检测没有掩码（用的是普通检测模型），会**自动回退**到 bbox 中心，不会报错。
4. 调试图里会把掩码轮廓也画出来，方便确认。

> 进一步（可选）：若需要更贴合“可抓取点”，后续可在掩码上做最小外接矩形/主轴分析来给出抓取朝向，目前先给出质心位置（姿态仍由抓取服务保持竖直下压）。

### 置信度门槛（只有足够确信才抓）

有两道门槛，**都可用 ROS 参数传入且都有默认值**：

1. **yolo_ros 自身的 `threshold`**（默认 `0.5`）——低于它的目标根本不会出现在 `/yolo/detections`：
   ```bash
   ros2 launch grab_demo yolo_ros_grab.launch.py threshold:=0.6
   ```
2. **桥接端的 `conf_threshold`**（默认 `0.0`，即只听 yolo 的）——再加一道“抓取门槛”，只有 `score ≥ conf_threshold` 才会被选为抓取目标，可运行时改：
   ```bash
   ros2 param set /yolo_ros_node conf_threshold 0.7
   ```

两者关系：`threshold` 决定“显示哪些框”，`conf_threshold` 决定“这些框里达到多少分才允许抓”。一般把 `threshold` 设低一点看全，再用 `conf_threshold` 卡抓取。

## 九、和旧 TensorRT 版（yolo_detect_node）的对照

| 项目 | 旧 `yolo_detect_node` | 新 `yolo_ros` + 桥接 |
|------|----------------------|---------------------|
| 推理后端 | 自研 TensorRT C++ | 官方 yolo_ros（ultralytics） |
| 硬件依赖 | 必须 CUDA + TensorRT（Jetson） | CPU/GPU 皆可 |
| 模型格式 | `.onnx` / `.engine` | `.pt`（ultralytics），含 v5~v12 / yolo-world |
| 跟踪 | 无 | yolo_ros 自带 ByteTrack（`use_tracking:=True`） |
| 输出接口 | `target_frame` + `/grab_target/distance` | **完全相同** |
| 抓取 | 开环 `grab_service_node` | 闭环 `closed_loop_grab_node` |

## 十、Dashboard 集成

Web Dashboard（`lebai_driver` 的 `dashboard_node`）已接入本方案：

- **功能启动页**新增一键任务【YOLO 抓取 (yolo_ros + 闭环)】，等价于
  `ros2 launch grab_demo yolo_ros_grab.launch.py`，启动/停止/看日志都在网页上完成。
- **监控页**新增卡片【YOLO 识别 / 闭环抓取 参数 (运行时可调)】，用滑条/下拉/输入框在线调：
  `z_offset`、`conf_threshold`、`select_mode`、`center_mode`、`target_class`、`target_label`
  （以上 → `/yolo_ros_node`）和 `grasp_z_offset`、`approach_height`、`max_iters`
  （以上 → `/grab_service_n`）。点【应用】即通过 `ros2 param set` 即时下发。
- 距离实时显示沿用既有的 `/grab_target/distance` 卡片。

> 启动 Dashboard：`ros2 launch lebai_driver dashboard.launch.py`，浏览器开 `http://<设备IP>:8080`。
> 参数面板是后端**白名单**渲染的，网页只能调这些预定义参数，不能设置任意节点/参数。

## 十一、异常日志与健康监控

为便于排障，识别与抓取两端都加了清晰的异常日志：

**桥接节点 `yolo_ros_node`：**
- 回调里任何未预料异常都打印**异常信息 + 堆栈**（节流 2s），并累计次数，不会静默失效；
- **检测流看门狗**（1Hz）：长时间（默认 `det_timeout=3s`）收不到 `/yolo/detections` 时
  打 `ERROR`，提示「yolo_ros 崩溃 / 相机掉线 / 话题不匹配」，并**停止广播过期 target_frame**，
  让抓取端及时发现 TF 失效（闭环安全）；
- 区分「画面里没有物体」与「有物体但被 `target_class/label/conf` 过滤掉」，分别给出提示；
- 目标**锁定/丢失**会各打一条日志；深度编码非 16UC1、mask 模式下彩色/深度分辨率不一致也会告警。

**闭环抓取 `grab_service_n`：**
- 整个抓取流程包在 `try/catch` 里，MoveIt/TF 等异常会被捕获并记录，服务**始终返回**，
  并在异常后尽量**张爪 + 退回观察点**，避免停在危险位姿；
- 夹爪服务（`io_service`）未就绪时带超时**循环告警**，而不是无限静默阻塞；
- TF 不可用、规划失败、执行失败、`obj_link` 为空等都有针对性日志。

查看日志：在 Dashboard 任务页点【日志】，或终端 `ros2 launch ...` 的输出里直接看。
