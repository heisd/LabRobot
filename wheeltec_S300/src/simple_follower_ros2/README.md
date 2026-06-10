# simple_follower_ros2

WheelTec 机器人多模态跟随包,提供视觉颜色跟随、激光跟随、ArUco 跟随、巡线(含随机分叉)等多种"跟随 + PID"实现。

## 概述

纯 Python 实现,包含若干"识别 + PID 跟随"节点:
- `visualFollower` — 基于 HSV 颜色块识别 + 距离估计的视觉跟随
- `visualTracker` — 视觉识别器(发布目标位置)
- `laserfollower` / `laserTracker` — 基于 2D 激光的目标跟随
- `ar_follow` — 基于 ArUco 的跟随
- `line_follow` / `line_follow_node` — 基于颜色线条的巡线
- `qr_detector` — 二维码检测(发布检测事件,用于路径选择)
- `cmd_arbiter` — 速度仲裁器(**QR 事件优先级 > 巡线**,检测到二维码先减速后停下)
- `qr_make` — 二维码生成工具
- `adjust_hsv` — HSV 阈值调试工具

## 目录结构

```
simple_follower_ros2/
├── package.xml
├── setup.py / setup.cfg
├── msg/
│   └── Position.msg                  # 目标角度 + 距离
├── param/
│   ├── PID_visual_param.yaml
│   └── pid_laser_param.yaml
├── parameters/
│   ├── PID_laser_param.yaml
│   ├── PID_visual_param.yaml
│   └── ar_param.yaml
├── resource/
├── simple_follower_ros2/
│   ├── visualFollower.py / visualTracker.py
│   ├── laserfollower.py / laserTracker.py
│   ├── ar_follow.py
│   ├── line_follow.py / line_follow_node.py
│   ├── line_follow_plain.py            # 纯巡线(无分叉处理)
│   ├── qr_detector.py                  # 二维码检测节点
│   ├── cmd_arbiter.py                  # 速度仲裁器(QR 优先)
│   ├── qr_make.py                      # 二维码生成工具(CLI)
│   ├── qr_make_gui.py                  # 二维码可视化生成工具(GUI)
│   ├── qr_codes/                       # qr_make 生成的二维码图片(与 QR 节点同级)
│   └── adjust_hsv.py
└── launch/
    ├── visual_follower.launch.py
    ├── laser_follower.launch.py
    ├── aruco_follower.launch.py
    ├── line_follower.launch.py
    ├── line_follow_random.launch.py
    ├── line_follow_qr.launch.py        # 巡线 + 二维码路径选择(QR 优先)
    ├── line_follow_qr_fixed.launch.py  # 纯巡线 + 二维码固定转角
    └── adjust_hsv.launch.py
```

## 依赖项

- buildtool: `ament_python`
- depend: `rclpy`、`geometry_msgs`、`nav_msgs`(里程计闭环转角)、`sensor_msgs`、`std_msgs`、`cv_bridge`、`OpenCV`、`numpy`、`aruco_msgs`(用于 ArUco 跟随);可选 `pyzbar`(更鲁棒的二维码识别)

## 消息定义

### `msg/Position.msg`

```
float32 angleX     # 目标方位角(rad,左正右负)
float32 angleY     # 目标俯仰角
float32 distance   # 估算距离(m)
```

## 节点说明

### `visualFollower`、`visualTracker`(视觉颜色跟随)

- `visualTracker` 订阅 RGB 图像,基于 HSV 阈值分割颜色块,发布 `Position`(目标位置)。
- `visualFollower` 订阅 `Position`,通过 PID 输出 `cmd_vel`。

参数(`PID_visual_param.yaml`):
- `P: [1.4, 0.4]` — 线速度 / 角速度 P 增益
- `I: [0, 0]`
- `D: [0.03, 0]`

### `laserfollower`、`laserTracker`(激光跟随)

- `laserTracker` 订阅 `/scan`,选取最近障碍物方向 / 距离作为目标。
- `laserfollower` 用 PID 跟随该目标。

参数(`pid_laser_param.yaml`):
- `P: [1.6, 0.5]`
- `D: [0.03, 0.005]`

### `ar_follow`(ArUco 跟随)

订阅 `aruco_msgs/MarkerArray` 或 `aruco_msgs/Marker`,锁定指定 ID 的 Marker 并跟随。

参数(`ar_param.yaml`):
- ArUco ID、目标距离、PID 等

### `line_follow` / `line_follow_node` / `line_follow_plain`(巡线)

订阅 RGB 图像,基于颜色阈值提取线条质心,输出 `cmd_vel` 巡线。
- `line_follow` — 含左叉确认的分叉处理。
- `line_follow_node`(`line_follow_random`)— 在 T 字 / Y 字分叉处随机左右选择。
- `line_follow_plain` — **纯巡线,无任何分叉处理**(只做"阈值→底部质心→PID")。路口左右转交给二维码 + `cmd_arbiter` 决定,适合配合"固定转角"二维码使用。

### 二维码路径选择(`qr_detector` + `cmd_arbiter`)

用于"巡线过程中检测到二维码先减速后停下",且 **二维码事件优先级高于巡线**。
两节点配合工作,巡线节点本身无需改动(launch 中把它的 `cmd_vel` 重映射到中间话题):

```
                 ┌──────────────┐  line_follow/cmd_vel
camera/image ───►│ line_follow  │──────────────────────┐
                 └──────────────┘                       ▼
                 ┌──────────────┐  qr_code/detected ┌──────────────┐
camera/image ───►│ qr_detector  │──────────────────►│ cmd_arbiter  │──► cmd_vel
                 └──────────────┘  qr_code/data      └──────────────┘
```

**`qr_detector`** — 订阅相机图像,用 `cv2.QRCodeDetector` 检测/解码二维码,发布:
- `qr_code/detected` (`std_msgs/Bool`) — 是否**确认**检测到二维码
- `qr_code/data` (`std_msgs/String`) — 解码内容(如 `path:left`,供路径选择)
- `qr_code/area_ratio` (`std_msgs/Float32`) — 二维码面积占比(距离的粗略代理,与缩放无关)

检测后端自动选择(越靠前越鲁棒):**pyzbar(ZBar)** > **cv2.QRCodeDetector**。`cv2.QRCodeDetector` 对带角度/弯曲/密集的二维码识别率较低,**强烈建议安装 pyzbar** 以提高现场识别率:

```bash
sudo apt install libzbar0
pip3 install pyzbar
```

> 为避免"检测过程本身打断/卡顿巡线",检测做了三重处理:**抽帧**(`detect_every_n`,把算力让给巡线)、**缩放**(`detect_scale`,默认 `1.0` 不缩放以保证识别率;CPU 吃紧再调小,但密集二维码可能识别不到)、**解码+连续确认**(`min_consecutive`,必须连续多帧成功解码才算确认)。因此只是"在检测"时不会让小车停车,**只有真正确认到二维码命令**,`cmd_arbiter` 才会减速停车。

`show_image=True` 时会弹出 **`QR Check`** 可视化窗口,实时显示:检测状态(红`searching`/黄`detecting`/绿`CONFIRMED`)、解码内容、当前后端 `[pyzbar/opencv]`、确认进度 `confirm=c/N`、面积占比以及检测框,方便现场调试。`line_follow_qr.launch.py` 中默认已开启。

参数:`image_topic`(默认 `/camera/color/image_raw`)、`min_area_ratio`(默认 `0.005`,过滤远处误检)、`show_image`(默认 `False`,launch 中开为 `True`)、`detect_every_n`(默认 `3`)、`detect_scale`(默认 `1.0`)、`min_consecutive`(默认 `3`)。

**`cmd_arbiter`** — 速度仲裁器(优先级 MUX)+ 二维码路径动作:
- 正常时透传 `line_follow/cmd_vel` → `cmd_vel`(巡线)
- 一旦检测到二维码立即进入更高优先级流程:`FOLLOW → DECELERATING(先减速)→ STOPPED(后停下)`,期间忽略巡线指令
- 停稳后按二维码内容执行动作(状态机增加 `TURNING`):

| 二维码内容(命中关键字即可) | 动作 |
| --- | --- |
| `path:left`  | **左转**:原地左转,直到重新发现线 → 恢复巡线 |
| `path:right` | **右转**:原地右转,直到重新发现线 → 恢复巡线 |
| `path:left30` | **固定左转 30°**:原地左转固定角度 → 恢复巡线(数字可改, 如 `left45`) |
| `path:right30` | **固定右转 30°**:原地右转固定角度 → 恢复巡线 |
| `path:stop`  | **停止**:保持停车(二维码移走后按 `resume_after_clear` 恢复) |
| `path:straight` | **直行**:停一下后继续巡线 |
| 其它/无法识别 | 安全起见按 **停止** 处理 |

> **固定转角**默认用 **里程计(`/odom`)闭环**精确转到目标角度:转向时累计 `odom` 的 yaw 变化(已处理 ±π 翻转),达到目标弧度即停,角度与速度/地面无关,更准。若拿不到里程计(`use_odom_turn=False` 或没有 `/odom`)则自动退回**开环按时间**(时长 = 角度弧度 / `turn_angular_speed`);两种模式都有安全超时(期望时长×2+2s)。`path:left`/`path:right`(不带数字)仍是"转到重新发现线"的旧行为。
>
> **同一二维码冷却**:同一**内容**的二维码在 `same_qr_cooldown` 秒(默认 `5.0`)内只会触发一次动作,避免靠近/经过同一张码时被反复识别;不同内容的二维码不受影响。

> "重新发现线" 的判据复用巡线节点:`line_follow` 看到线时 `linear.x>0`,丢线时为 `0`,所以**无需改动巡线节点**即可知道线是否重新出现。左/右转会先"盲转" `turn_min_time` 秒离开路口,再开始找线,避免在路口原地旧线上误判;并有 `turn_max_time` 安全超时。处理完一张码后会"解除武装",必须等该码彻底离开才允许再次触发,避免对同一张码反复触发。

参数:
- 减速/停车:`decel_duration`(默认 `1.2`s)、`publish_rate`(默认 `20`Hz)、`detect_timeout`(默认 `0.5`s)、`clear_hold`(默认 `1.0`s)、`resume_after_clear`(默认 `True`)、`stop_dwell`(停稳停留,默认 `0.5`s)、`same_qr_cooldown`(同一码冷却,默认 `5.0`s)
- 路径动作:`enable_path_action`(默认 `True`)、`turn_angular_speed`(默认 `0.4` rad/s)、`turn_min_time`(默认 `1.0`s)、`turn_max_time`(默认 `8.0`s)、`line_found_eps`(默认 `0.005`)、`line_confirm`(默认 `3` 帧)
- 固定转角闭环:`use_odom_turn`(默认 `True`)、`odom_topic`(默认 `/odom`)

### `qr_make`(二维码生成工具)

生成可打印的二维码图片,默认保存在 **QR 节点同级目录** 的 `qr_codes/` 文件夹下。
后端自动选择 `qrcode` / `segno` / `cv2.QRCodeEncoder` 中任意一个可用项。

```bash
# 生成一组默认路径选择二维码(含 left/right/left30/right30/straight/stop)
ros2 run simple_follower_ros2 qr_make --all

# 生成单个自定义二维码
ros2 run simple_follower_ros2 qr_make --data "path:left" --name turn_left

# 生成任意角度的固定转角二维码
ros2 run simple_follower_ros2 qr_make --turn left --angle 45
# 随机角度(范围可配 --min-angle / --max-angle)
ros2 run simple_follower_ros2 qr_make --turn right --random --min-angle 20 --max-angle 90
```

仓库已预生成 `qr_codes/{turn_left,turn_right,turn_left_30,turn_right_30,go_straight,stop}.png`,可直接打印张贴在线路上。

### `qr_make_gui`(二维码可视化生成工具)

基于 OpenCV 滑条的可视化工具:实时调方向 / 角度 / 类型,窗口里**实时预览**二维码,按键保存。

```bash
ros2 run simple_follower_ros2 qr_make_gui      # 或 qr_make --gui
```

窗口 `QR Maker` 滑条:`angle`(0~180)、`dir 0L/1R`、`mode`(0 固定转角 / 1 转到发现线 / 2 直行 / 3 停止)、`box`(清晰度)。按键:`s` 保存到 `qr_codes/`、`r` 随机角度+方向、`q`/`ESC` 退出。

### `adjust_hsv`(调参工具)

弹出 OpenCV trackbar,实时调整 HSV 阈值并显示分割结果。

## 启动文件

| 文件 | 用途 |
| --- | --- |
| `visual_follower.launch.py` | 启动底盘 + 相机 + 视觉跟随 |
| `laser_follower.launch.py` | 启动底盘 + 雷达 + 激光跟随 |
| `aruco_follower.launch.py` | 启动相机 + ArUco 节点 + 跟随 |
| `line_follower.launch.py` | 启动底盘 + 相机 + 巡线 |
| `line_follow_random.launch.py` | 巡线 + 分叉随机选择 |
| `line_follow_qr.launch.py` | 巡线 + 二维码路径选择(QR 优先,先减速后停下) |
| `line_follow_qr_fixed.launch.py` | 纯巡线(无分叉)+ 二维码固定转角(left30/right30) |
| `adjust_hsv.launch.py` | 启动 HSV 调试 |

## 编译与运行

```bash
colcon build --packages-up-to simple_follower_ros2
source install/setup.bash

# 视觉跟随(默认颜色)
ros2 launch simple_follower_ros2 visual_follower.launch.py
```

## 使用示例

```bash
# 1. 调 HSV 阈值
ros2 launch simple_follower_ros2 adjust_hsv.launch.py

# 2. 把 HSV 写入对应 .py 后,启动视觉跟随
ros2 launch simple_follower_ros2 visual_follower.launch.py

# 3. 也可以激光跟随(任何最近的腿/物体)
ros2 launch simple_follower_ros2 laser_follower.launch.py

# 4. 巡线 + 二维码路径选择(检测到二维码先减速后停下, QR 优先级高于巡线)
ros2 run simple_follower_ros2 qr_make --all   # 先生成并打印二维码
ros2 launch simple_follower_ros2 line_follow_qr.launch.py

# 5. 纯巡线(无分叉) + 二维码固定转角(left30/right30)
ros2 run simple_follower_ros2 qr_make --all   # 含 turn_left_30 / turn_right_30
ros2 launch simple_follower_ros2 line_follow_qr_fixed.launch.py
```

## 注意事项

1. 视觉跟随依赖正确的 HSV 阈值,请先使用 `adjust_hsv` 标定。
2. 激光跟随会跟"最近障碍物",出门口或大空间需谨慎。
3. PID 增益是按 S300 默认尺寸/速度调好的,小车体或地毯地面需重新调参。
4. 多个跟随节点会同时争抢 `cmd_vel`,一次只能启动一个 launch。
5. `截图 2026-05-23 18-19-05.png` 为示例图,仅作参考。
