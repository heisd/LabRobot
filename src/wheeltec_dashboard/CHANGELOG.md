# wheeltec_dashboard 变更记录

本文件记录 `wheeltec_dashboard` 面板的 bug 修复与改进，按时间倒序排列。
第 1–3 轮在分支 `claude/inspiring-thompson-LU7ql` 上完成，第 4–5 轮在
`claude/happy-ride-lzphg0` 上完成。

---

## 第 9 轮 — 地图选点航点管理（后端持久化）+ 导航仲裁（手动打断 Nav2/VLA）

### 背景

第 8 轮的"航点标定助手"只能生成 YAML 片段让人手工粘贴+重编译。本轮把
VLA 后端补成**运行时可增删 + 文件持久化**，前端升级为**在已建地图上选点**，
并新增**导航仲裁节点**让手动遥控随时打断 Nav2/VLA 自主导航。

### 后端 vla_navigation（需重编译）

- **运行时航点管理**：新订阅 `vla/waypoint_cmd`（String JSON，
  add 同名覆盖 / remove），变更**立即生效**（下一条指令的模型提示词就含
  新航点，原子换引用对推理线程安全）；新发布 `vla/waypoints`（列表 JSON，
  变更即发 + 3s 周期重发，晚连的面板也能拿到）。
- **文件持久化**：每次变更整表写入 `user_waypoints_file`
  （默认 `~/.ros/vla_waypoints.yaml`，不被 colcon build 覆盖）；启动时该文件
  存在则**优先于包内 config/waypoints.yaml 加载**——标定一次永久生效。
  `waypoints.py` 增加 `to_dict/upsert/remove/to_dict_list/save`（含离线
  roundtrip 自测通过）。
- **新增 `nav_arbiter` 导航仲裁节点**（目标监督式，不改 Nav2 launch）：
  订阅 `cmd_vel_manual`/`cmd_vel_keyboard`，收到手动速度立即向
  `navigate_to_pose/_action/cancel_goal` 发零 UUID（=取消全部目标，
  同时覆盖 RViz/dashboard 的 /goal_pose 目标与 VLA 目标）；手动滑动窗口
  `manual_timeout`（2s）内若 `cmd_vel_nav` 又有输出则限频重复取消；
  窗口结束发零速兜底并恢复 AUTO；状态发 `nav_arbiter/status`。
  随 `vla_bringup.launch.py` 自动启动（`start_arbiter:=false` 可关）。

### 前端 dashboard

- **"航点标定助手"升级为"地图航点管理"**：
  - 地图显示：`/map_server/map`（GetMap 服务，子页首次可见自动加载）+
    `/map` 话题兜底（SLAM 建图中实时刷新；map_server 的 transient_local
    帧 rosbridge 可能收不到，故以服务为主）；OccupancyGrid 渲染为位图
    （空闲/占用/未知三色，y 轴预翻转），`data` 兼容数组与 base64 两种
    rosbridge 编码。
  - 叠加层：小车实时位姿（绿箭头，wpTf）、已存航点（蓝点+名字）、当前
    选点（黄箭头）。
  - 交互：**按下选位置、按住拖动定朝向**（同 RViz 2D Goal Pose，
    setPointerCapture 触屏可用，拖 3 格以上才改 yaw 防手抖）。
  - 【保存到机器人】发 `/vla/waypoint_cmd`，列表（`/vla/waypoints`）显示
    全部航点与持久化文件路径，行内【导航】直发 `/goal_pose`（二次确认，
    不经大模型）、【删除】同步持久化；备用"生成 YAML 片段"手动流保留
    （优先用地图选点，其次当前位姿）。
- **手动打断接入**：遥控 publishCmd 同步双发 `/cmd_vel_manual`（未跑仲裁
  节点时无人订阅、零副作用）；VLA 卡新增"导航仲裁"指标
  （`/nav_arbiter/status`，MANUAL 橙色），接管/释放边沿写入 VLA 时间线。

### 文件改动汇总（第 9 轮）

| 文件 | 变化 |
| --- | --- |
| `../vla_navigation/vla_navigation/waypoints.py` | to_dict/upsert/remove/save |
| `../vla_navigation/vla_navigation/vla_navigator.py` | 用户航点文件优先加载、/vla/waypoints 广播、/vla/waypoint_cmd 处理 |
| `../vla_navigation/vla_navigation/nav_arbiter.py` | 新增导航仲裁节点 |
| `../vla_navigation/setup.py` / `launch/vla_bringup.launch.py` / `config/vla_params.yaml` / `README.md` | 入口、start_arbiter、参数与文档 |
| `web/index.html` | 地图航点管理卡（画布/工具条/列表）、VLA 卡仲裁指标 |
| `web/app.js` | OccupancyGrid 渲染与选点、/vla/waypoints //vla/waypoint_cmd //nav_arbiter/status、/cmd_vel_manual 双发 |
| `web/style.css` | `.wp-map-*` / `.wp-row` / `.wp-list` |

### 验证清单

- [ ] 重编译 `colcon build --packages-select vla_navigation wheeltec_dashboard` 后启动 `vla_bringup.launch.py`：VLA 子页地图自动出现，绿箭头跟随小车移动。
- [ ] 地图上按下拖动选点，填"测试点"保存：列表 1s 内出现该点，`cat ~/.ros/vla_waypoints.yaml` 包含它；对小车说/输入"去测试点"能导航。
- [ ] 重启 vla_navigator（不重新标定）：启动日志显示"从用户航点文件加载"，列表还在——下次无需重新设置。
- [ ] 列表【导航】到某点途中，按住面板 W 键：小车立即响应手动、Nav2 停止输出（`/nav_arbiter/status` 显示 MANUAL，VLA 时间线记录接管）；松手 2s 后恢复 AUTO，再发"去 X"正常。
- [ ] 手动期间发"去厨房"：VLA 的目标被仲裁立即取消，小车不抢方向。
- [ ] 【删除】航点后 `~/.ros/vla_waypoints.yaml` 同步少一条。

---

## 第 8 轮 — 下发命令按协议解析进事件栏 + 骨架识别子页 + VLA 航点标定助手

### 下位机事件显示"已解析的命令"（对照通信协议表）

用户提供了 S 系列通信协议表（C63A↔ROS 串口部分），按表实现：

- **驱动**：`Cmd_Vel_Callback` / `Red_Vel_Callback` / `Set_LightRgb_Callback`
  每次串口写帧成功后，把 11 字节控制帧原样回发到新话题
  `/robot_serial_tx`（UInt8MultiArray）。析构时的停车/复位帧不回发
  （节点正在关闭）。
- **面板**：`parseStm32Tx()` 按协议解析——帧头 `0x7B`/帧尾 `0x7D` 校验、
  BCC（前 9 字节异或 = 第 9 字节）、模式选择位
  （0=速度控制 / 1、2=自动回充 / 3=红外对接速度 / 4=灯带 RGB）、
  三轴目标速度（short, mm/s → m/s）、安全级（速度帧第 2 字节）。
- **防刷屏**：STM32 卡新增"最近下发指令（已解析）"指标实时刷新（BCC 错
  标红）；事件栏只在**命令签名变化**时追加一条（含解析文本 + 原始 hex），
  连续速度帧数值变化不重复记录。

### 新增"骨架识别 / 体感跟随"子页（功能模块，wheeltec_bodyreader）

读包源码对齐接口（`main/bodydata_process/follower/interaction/display.py`）：

- 骨架叠加画面 `/body/body_display`（display.py 发布，走 web_video_server
  MJPEG，复用 fn-img 懒加载）。
- 状态卡：`/body_posture` → 锁定状态（0 无人/1 检测到未锁定/2 已锁定）、
  锁定 ID、目标距离（centerofmass_z mm→m）、横向偏角（atan2(x,z)）、
  活跃姿态（叉腰锁定/举左右手/平举左右臂/抬左右脚）、跌倒告警；
  `/bodylist` → 视野人数；3 秒无数据自动回 "—"。
- 控制：发布 `/mode`（2=跟随【二次确认，会动真车】、1=姿态交互）、
  `/recoveryid`（Int16，找回锁定目标）。
- `/body_follower` 的 bodyfollow_x_p/x_d/z_p/z_d 接入既有 param-group
  在线调参。
- 架构卡与功能子导航加"骨架识别"入口。

### VLA 子页新增"航点标定助手"

回答"怎么建立 VLA 导航点"：航点是 `vla_navigation/config/waypoints.yaml`
里 map 坐标系的静态位姿，节点启动时加载。助手卡把标定流程工具化：

- 独立 `makeTfClient(ros, 'map')` 在浏览器端合成 map→base_footprint，
  500ms 刷新当前实测位姿（x/y/yaw，rad+deg）；未定位时显示提示；
  子页不可见时不刷新。
- 填航点名/别名 →"用当前位姿生成 YAML"产出可直接追加进
  `waypoints.yaml` 的片段 + 一键复制；hint 写明完整流程
  （建图→定位→开到点→生成→追加→colcon build→重启 vla_navigator）。

### 文件改动汇总（第 8 轮）

| 文件 | 变化 |
| --- | --- |
| `web/index.html` | 骨架识别子页、航点标定助手卡、STM32"最近下发指令"指标、导航/架构卡入口 |
| `web/app.js` | parseStm32Tx/ingestStm32Tx、bodyreader 订阅与控制、wpTf 航点助手、teardown 清理 |
| `web/style.css` | `.stm32-lastcmd` / `.body-recover` / `.wp-form` / `.wp-yaml` |
| `../turn_on_wheeltec_robot/...` | `/robot_serial_tx` 发布（hpp + cpp 三处写帧后回发） |
| `README.md` | 功能清单与明细更新 |

### 验证清单

- [ ] 重编译驱动后遥控小车：STM32 卡"最近下发指令"实时显示"速度控制 Vx=… Vy=… Vz=…"；事件栏出现一条"↓ 下发: 速度控制 …
 [7B 00 00 …]"，持续遥控不重复刷。
- [ ] 面板设置灯带红色：事件栏出现"↓ 下发: 设置灯带颜色 R255 G0 B0 [7B 04 01 FF 00 00 00 00 00 81 7D]"（与协议表示例一致）。
- [ ] 开始自动回充：事件栏出现"↓ 下发: 自动回充模式 …"。
- [ ] `ros2 launch bodyreader bodyfollow.launch.py` 后：骨架子页有叠加画面，人进入视野"锁定状态/人数"变化，叉腰后显示"已锁定"+ID；切"姿态交互模式"抬脚/举手时"活跃姿态"跟随显示。
- [ ] 启动 Nav2 定位后打开 VLA 子页：航点助手显示当前 x/y/yaw 并随小车移动刷新；填名字生成 YAML 片段、复制、追加到 waypoints.yaml、重编译重启后"去新航点"可导航。
- [ ] 未启动 Nav2 时航点助手显示"未定位（map TF 不可用）"。

---

## 第 7 轮 — 对齐新固件 KeilSingleChipProject：回充红外语义修正 + 自动回充 / RGB 灯带 / 安全等级 / 固件使能位

### 背景

仓库新增了下位机 STM32F407 的完整 Keil 工程源码
（`firmware/KeilSingleChipProject`，FreeRTOS）。逐文件比对固件协议
（`data_task.c` 上行三包 / `SerialControl_task.c` 下行命令 /
`RobotControl_task.c` 使能逻辑 / `AutoRecharge_task.c` 回充 /
`RGBStripControl_task.c` 灯带）与驱动 `turn_on_wheeltec_robot` 后，
把固件实际暴露但面板没接的能力补齐，并修正一处语义错误。

### 语义修正：`/robot_red_flag` 不是急停

固件回充帧 `autorechargerbuffer[3] = ChargeDev.RedNum` —— 是**收到充电桩
红外信号的对管个数（0–4）**，驱动转成 Bool 发布。此前面板标成
"急停 (red flag)"并按 err 渲染是错的。现统一改为"回充红外信号"，
检测到=ok / 未检测=warn（总览遥测、STM32 卡、自动回充卡三处）。

### 新增"自动回充"卡（底盘控制页）

- 【开始自动回充】发布 `1` 到 `/robot_recharge_flag`（二次确认），
  【退出回充】发布 `0`。**关键细节**：驱动只把该标志存进变量、随下一帧
  cmd_vel 序列化进串口帧 frame[1]，所以发布后面板自动补发一帧零速
  cmd_vel 把标志带下去（固件 `roscmdBuf[1]==1||2 → ChargeMode=1`）。
- 卡内显示：回充模式（**固件回读**，见下）、回充红外、充电中、充电电流。
- 固件行为已写进卡片提示：寻桩由充电桩 CAN 设备控制（Charger_CMD
  优先级最低，手动遥控可打断）；无红外信号或已充电时寻桩速度为 0；
  **回充模式中低压禁动豁免**（`robot_en_check`: `Vol<20 && ChargeMode==0`
  才置 LowPower 错）；灯带转充电指示。

### 新增"RGB 灯带"卡（底盘控制页）

- 颜色选择器 + R/G/B 读数 +【设置颜色】/【关闭灯带】，调驱动服务
  `/set_rgb_color`（robot_interfaces/SetRgb → 固件 `7B 04 en R G B … 7D`）。
- 提示固件灯带优先级（充电指示 > 低电量 > 超声波警示 > 用户自定义），
  自定义色在这些状态活跃时会被暂时覆盖。

### 速度控制卡新增"安全等级"开关

`/chassis_security`（Int8）→ 串口帧 frame[2] → 固件 SecurityLevel：
0 = 速度流中断时固件主动停车（看门狗，默认）；1 = 保持最后速度。
切到 1 需二次确认（断网不自停，明确标danger）；应用后补发一帧当前速度
使其立即生效。

### STM32F407 卡新增"固件使能位"与"回充模式回读"

- 新指标 **固件使能 (en_flag)**：来自 24 字节帧 rx[1]（固件
  `RobotControlParam.en_flag`），是固件真实的"允许移动"信号，涵盖
  低压 / 急停开关 / 软件急停 / 驱动器离线或报错 全部失能条件 ——
  比面板原来仅按电压推断的"底盘移动"更准确。失能/恢复边沿写事件日志
  并列出固件 errCode 枚举的可能原因。
- 新指标 **回充模式（固件确认）**：回充帧 rx[5] 的回读，区别于上位机
  意图 `/robot_recharge_flag`；进入/退出边沿写事件日志。
- "底盘移动"指标在 低压+回充中 时显示"允许（回充中低压豁免）"(warn)，
  对齐固件逻辑。
- 卡片提示更新：固件源码位置、20Hz 三包帧结构（24B 基础 `0x7B…0x7D` +
  19B 超声波 `0xFA…0xFC` + 8B 回充 `0x7C…0x7F`）、新固件自检字段恒 0
  （`/self_check_data` 显示 0x0 属正常）。

### 配套驱动改动（turn_on_wheeltec_robot，需重编译）

- **启用 `set_rgb_color` 服务**：回调原为 ROS1 风格签名且注册行被注释，
  改为 rclcpp shared_ptr 签名并注册；修复串口异常后仍返回
  "Set successfully" 的 bug（catch 分支 return）。
- **新发布 `/robot_enable_flag`**（Bool，随 24 字节帧 ~20Hz）：rx[1]
  此前已解析进 `Receive_Data.Flag_Stop` 但从未发布。
- **新发布 `/robot_recharge_mode`**（Bool，随回充帧）：解析此前被忽略的
  回充帧 rx[5]（固件 ChargeMode 回读）。
- 面板对旧驱动向后兼容：两个新话题没有时对应指标保持 "—"，
  RGB 服务不存在时按钮报"调用失败（驱动是否已重编译…）"。

### 文件改动汇总（第 7 轮）

| 文件 | 变化 |
| --- | --- |
| `web/index.html` | red flag 标签修正、STM32 卡 en_flag/回充模式指标 + hint 重写、自动回充卡、RGB 灯带卡、安全等级行 |
| `web/app.js` | red flag/充电状态多处广播、/robot_enable_flag //robot_recharge_mode 订阅与边沿日志、回充/安全等级发布（补发 cmd_vel 推帧）、RGB 服务调用、低压回充豁免显示 |
| `web/style.css` | `.sec-row` / `.rc-btns` / `.rgb-row` 样式 |
| `../turn_on_wheeltec_robot/include/.../wheeltec_robot.hpp` | SetRgb 回调签名、新发布者/成员/函数声明 |
| `../turn_on_wheeltec_robot/src/wheeltec_robot.cpp` | 启用 set_rgb_color 服务、Publish_EnableFlag / Publish_RechargeMode、回充帧 rx[5] 解析 |
| `README.md` | 遥测/控制功能清单更新 |

### 验证清单

- [ ] 重编译 `colcon build --packages-select turn_on_wheeltec_robot` 后：`ros2 topic echo /robot_enable_flag` 有 ~20Hz 数据；按下急停开关 → false，STM32 卡"固件使能"变红并写事件日志。
- [ ] `ros2 service call /set_rgb_color robot_interfaces/srv/SetRgb "{en: true, r: 255, g: 0, b: 0}"` 灯带变红；面板 RGB 卡选色"设置颜色"灯带跟随，"关闭灯带"熄灭。
- [ ] 面板"开始自动回充"后：`/robot_recharge_flag` 收到 1、随后一帧零速 cmd_vel；下位机进入回充（卡内"回充模式（固件确认）"变"回充中"，事件日志记录）；遥控打断后再"退出回充"恢复。
- [ ] 小车靠近充电桩：三处"回充红外"显示"检测到充电桩"（绿色，不再是"急停触发"红色）。
- [ ] 安全等级切 1 时弹危险确认；`ros2 topic echo /chassis_security` 收到 1，且随后有一帧 cmd_vel。
- [ ] 旧驱动（未重编译）下打开面板：新指标保持 "—"，无 JS 报错；RGB 按钮提示需重编译。

---

## 第 6 轮 — arm_demo 能力接入 + 下位机 STM32F407 状态卡 + 低压禁动提醒

### arm_demo（FK/IK 运动学演示）接入机械臂页

arm_demo 是两个一次性 MoveIt 演示程序（`fk_demo` 关节空间 / `ik_demo`
笛卡尔位姿），本身不暴露话题/服务，因此把**能力**而非节点接入面板，走
lebai 驱动既有服务：

- **关节运动卡新增预设位**：观察位 look / 零位 zero（来自 MoveIt SRDF
  命名位姿）与 FK 演示位（`fk_demo` 的硬编码目标关节角），点按钮只填值，
  仍走统一的"执行运动"二次确认路径。
- **新增"位姿运动 IK"卡**：末端 X/Y/Z + RPY（角度制，`quatFromRpy` 转
  四元数）经 `/motion_service/move_joint`（关节插补）或 `move_line`
  （笛卡尔直线）下发，`is_joint_pose=false` —— 逆解由乐白控制器完成。
  "IK 演示位"按钮把 `ik_demo` 的目标位姿（四元数经既有 `rpyFromQuat`
  反算成 RPY）填入。提示中明确：**不经 MoveIt、无碰撞检查**，要碰撞
  规划仍用 `ros2 launch arm_demo fk_demo/ik_demo.launch.py`。

### 下位机 STM32F407 状态卡（组件状态页）

- 读 `turn_on_wheeltec_robot/src/wheeltec_robot.cpp` 确认：`/odom`、
  `/PowerVoltage` 等话题**只有串口帧（帧头 0x7B）校验通过才发布**，
  因此用"话题数据流动"作下位机在线信号：`/odom`、`/PowerVoltage`
  回调刷新 `stm32Time`，2 秒无帧判离线，在线/离线**边沿**写入卡内
  事件时间线（复用 `appendTimeline`）。
- 卡内含本地 SVG 示意图（`web/stm32f407.svg`，LQFP 封装俯视，无外链）、
  串口在线状态、电池电压、底盘移动（<20V 禁动）、急停 red flag
  （`/robot_red_flag` 回调扩展为同时驱动总览与本卡）。
- **低压禁动提醒**：固件在电压 <20V（Plus 型）禁止底盘移动。
  - 前端：电压跌破/恢复 20V 边沿写事件日志，持续低压每 60s 重复提醒；
    "底盘移动"指标变红显示"禁动 (<20V)"；总览电压阈值同步改为
    <20 err（禁动）/ <22 warn（原 22/23）。
  - 驱动：`wheeltec_robot.cpp` 低压分支原本只 `cout` 到本地终端，
    现同时 `RCLCPP_WARN` 进 `/rosout`——Dashboard 日志面板可见。
- 架构卡底盘侧新增"下位机 STM32"芯片（跳组件状态页）。

### 文件改动汇总（第 6 轮）

| 文件 | 变化 |
| --- | --- |
| `web/index.html` | 关节预设位按钮、位姿运动 IK 卡、STM32 卡、架构卡芯片 |
| `web/app.js` | mj-preset 填充、quatFromRpy + armCartesianMove、stm32 在线/低压状态机与事件日志、/odom //PowerVoltage //robot_red_flag 回调扩展 |
| `web/style.css` | `.stm32-*` 样式 |
| `web/stm32f407.svg` | 新增本地芯片示意图 |
| `../turn_on_wheeltec_robot/src/wheeltec_robot.cpp` | 低压告警补 RCLCPP_WARN 进 /rosout |
| `README.md` | 组件状态/机械臂功能清单更新 |

### 验证清单

- [ ] 组件状态页出现 STM32F407 卡：底盘驱动启动后"串口数据流"变绿"在线"，拔串口/停驱动 2 秒后变红"离线"，事件栏各记录一条。
- [ ] 模拟低压（或真实低电量）：电压 <20V 时总览电压与卡内电压变红、"底盘移动"显示"禁动"，事件栏出现低压提醒且 60s 重复；`/rosout` 日志面板出现驱动的 WARN（需重编译 turn_on_wheeltec_robot）。
- [ ] 机械臂"关节运动"点"观察位 look"填入 [-1.14,-1.75,-2.47,-0.46,1.56,3.58]，执行后机械臂到观察位。
- [ ] "位姿运动 IK"点"IK 演示位"后执行，机械臂到 (0.6, 0.17, 0.46)；不可达位姿返回失败并写事件时间线。

---

## 第 5 轮 — 代码评审修复 + 导航重组为"底盘 / 机械臂"两部分

### 背景

对第 4 轮（机械臂接入）做了一次多视角代码评审（逐行 / 删除行为 / 跨文件
契约 / 语言陷阱 / 生命周期 / 复用 / 简化 / 效率 / 架构层次），并按
"机器人 = Wheeltec 底盘 + Lebai 机械臂 两部分"重组界面信息架构。

### 评审修复（app.js / kcf_track_node.cpp）

- **断线状态残留**：`teardownTopics()` 现在清空 `armJointMap`、
  `armYoloSelected`、YOLO 按钮区 —— 重连到另一台机器人不会再看到上一台的
  关节行 / 目标选中态；`armJointMap` 也因此不会跨重连无限增长。
- **隐藏面板的无效渲染**：关节表 200ms 渲染循环加 `offsetParent` 可见性
  门控（隐藏时保留 dirty，切回再画）；YOLO 物体按钮渲染加 HTML 串比对，
  检测结果没变化时不重建 DOM（保留 hover/焦点）。
- **空值守卫**：HSV 滑条"读取当前值"对 `res.values[i]` 加空值守卫；
  夹爪"设置位置/力度"按钮绑定同时要求滑块元素存在。
- **kcf_track_node `~/select_bbox` 上限防御**：uint32 字段超大值强转 int
  会变负/溢出，现在 >100000 的框直接拒收并告警。
- **流懒加载泛化（去特例）**：顶层页签 onActivate 不再特判 `'arm'`，统一走
  `kickVisibleFnStreams()` —— 只启动"可见且还停在占位图"的流；已在播放的
  流不重启（修掉了之前来回切页时 MJPEG 闪断的问题），三组 tab（顶层 /
  功能子页 / 机械臂子页）行为一致。
- **去重**：`addArmLog` 与 `addVlaStatus` 合并为 `appendTimeline()`；
  仲裁状态双 id（`arm-arb-state`/`arm-arb-state2`）改为 `.arm-arb` 类广播
  （与 `.arm-dist` 同模式）；删除未使用的 `arm-target-dist` id。

评审中核实为误报（保持原样）的：pointer capture 在 pointerup 后由浏览器
自动释放；`contentBox()` 已有 naturalWidth/Height 零值守卫；离线看门狗因
`armStatusTime=0` 复位只触发一次；`ROSLIB.Service` 按次创建与既有
`paramService()` 模式一致。

已记录未改（权衡后接受）的：`/kcf_node/select_bbox`、`/kcf_node/reinit`
节点名硬编码（流话题可改但框选发布固定 —— 重命名节点需同步改前端）；
机械臂订阅在未打开机械臂页时也保持活跃（与全文件订阅架构一致）。

### 界面重组（底盘 / 机械臂 两部分）

- 标题改为 **LabRobot Dashboard**，副标题"Wheeltec S300 底盘 · Lebai LM3
  机械臂"；导航栏加分组标签：`系统总览 │ Wheeltec 底盘（组件状态 / 底盘
  控制 / 功能模块）│ Lebai 机械臂（监控与抓取）│ 联系作者`。锚点不变。
- **系统总览新增"系统架构"卡**：左右两栏分别列出底盘与机械臂的子模块
  （点击芯片直接跳到对应页签/子页，含功能与抓取子页），中间标注两部分的
  连接关系（同一工作空间 / rosbridge / video 端口）；三个在线状态点由
  底盘遥测（`/PowerVoltage`）、机械臂驱动（`/robot_status`）、抓取目标
  （`/grab_target/distance`）数据流驱动，1s 刷新。
- 联系作者卡描述同步改为"底盘 + 机械臂"双部分口径。

### 文件改动汇总（第 5 轮）

| 文件 | 变化 |
| --- | --- |
| `web/app.js` | teardown 清理机械臂状态、渲染可见性门控、HTML 比对跳过重建、空值守卫、kickVisibleFnStreams 泛化、appendTimeline 合并、.arm-arb 类广播、chassisTime + 架构卡在线点 + 跳转 |
| `web/index.html` | 标题/副标题、导航分组标签、系统架构卡、冗余 id 清理、联系卡文案 |
| `web/style.css` | `.h1-sub`、`.nav-group-label`、`.arch-*` 架构卡样式（含窄屏纵排） |
| `../grab_demo/src/kcf_track_node.cpp` | select_bbox 尺寸上限防御 |

### 验证清单

- [ ] 导航栏显示"Wheeltec 底盘 / Lebai 机械臂"分组标签，原 `#...` 锚点深链全部可用。
- [ ] 系统总览顶部出现"系统架构"卡：连接 rosbridge 且底盘遥测到达后左侧绿点亮；启动 lebai_driver 后右侧绿点亮；启动任一抓取方案且识别到目标后"识别→抓取"流程点亮。
- [ ] 点架构卡里的"KCF 抓取"芯片：直接落在 机械臂→KCF 子页。
- [ ] 断开再重连 rosbridge：关节表清空重建、YOLO 按钮选中态清除。
- [ ] 在机械臂页与其他页之间来回切换：已在播放的 MJPEG 流不再闪断。

---

## 第 4 轮 — 机械臂接入（新增"机械臂"导航页）

### 背景

仓库由移动底盘（Wheeltec S300）与机械臂（Lebai LM3）合并而来，但 dashboard
此前只覆盖底盘。机械臂侧 lebai_driver 自带一个独立的小型网页面板
（`lebai_driver/dashboard`，自建 HTTP API），与主面板割裂：两套入口、两个端口。
本轮把机械臂功能直接接进主面板——所有交互均走既有的 rosbridge，对齐
lebai_driver / grab_demo 暴露的 ROS2 接口，不需要任何新后端。

### 改动

**导航栏新增"机械臂"标签页**（`#arm` 锚点深链可用），与"功能模块"同构：
自带子导航，分 **监控与控制 / HSV / YOLO / KCF / ArUco / VLM** 六个子页。
机械臂子页用独立的 `.arm-subtab-btn`/`.arm-subtab-panel` 类 +
`data-armsubtab` 键复用 `initTabGroup`，与功能模块的 `.subtab-*` 组互不干扰
（若共用类名，一组的 activate 会把另一组的 active 面板全部关掉）。

**监控与控制子页**：

- **机械臂状态**：订阅 `/robot_status`（lebai_interfaces/RobotStatus），
  TriState（1/0/-1）渲染急停/上电/可运动/运动中/错误 + 错误码 + 模式；
  另显示 `/arm_arbiter/state` 抓取控制权。3 秒无数据自动回 "—"（离线水位，
  避免陈旧状态误导操作）。
- **系统控制**：`/system_service/*`（std_srvs/Empty）十个按钮 + 急停大按钮。
  危险操作（断电/去使能/中止/关机）`confirm()` 二次确认；急停立即下发不确认。
- **夹爪**：订阅 `/gripper_status`；滑块 + 张开/闭合快捷键调
  `/io_service/set_gripper_position|set_gripper_force`（SetGripper，0–100）。
- **关节状态**：订阅 `/joint_states` 进 `armJointMap`（按关节名合并，
  底盘/机械臂同名话题共存也不互踩），最高 5Hz 重绘表格（度/弧度/速度）。
- **关节运动**：6 关节角(rad) + acc/vel 调 `/motion_service/move_joint`
  （MoveJoint）。请求按 rosbridge 习惯完整填充（cartesian_pose 单位四元数、
  common.time/radius=0）。"填入当前关节角"优先取 `lebai_joint_*`（排序后
  正好按编号），过滤掉夹爪/底盘关节。执行前二次确认。
- **抓取与仲裁**（各方案共用）：目标深度 `/grab_target/distance`（2.5s 过期
  回 "—"，经 `.arm-dist` 类同步到各子页）；TF 抓取调 `/obj_grab_service`
  （GrabObject，二次确认，提示规划可能数十秒）；仲裁接管/释放调
  `/arm_arbiter/manual_takeover|manual_release`（Trigger）。所有命令与结果
  写入"机械臂事件"时间线（textContent 渲染，不注入标记）。
- **机械臂相机**：复用 fn-img MJPEG 模式预览 `/camera_arm/color/image_raw`。

**抓取方案子页**（对齐 grab_demo 各 launch 的节点名与私有调试话题；每页含
调试画面 + 目标距离 + "抓取目标"快捷键，快捷键经 `.arm-grab-quick` 类共用
`requestArmGrab()`）：

- **HSV**（`color_grab` / `color_node`）：调试图 `/color_node/detection_image`；
  **HSV 阈值滑条**（`.hsv-group` 组件：H 0–180、S/V 0–255、min_area 0–5000，
  拖动 120ms 防抖即发 `set_parameters`(int)，"读取当前值"经 `get_parameters`
  同步回滑条）。
- **YOLO**（`yolo_ros_grab` / `yolo_ros_node`）：调试图
  `/yolo_ros_node/detection_image`；**识别物体按钮** —— 订阅
  `/yolo/detections`（yolo_msgs/DetectionArray，300ms 节流），按类别合并
  渲染成按钮（数量 + 最高置信度），点击把类别名写进桥接节点的
  `target_label` 参数（节点名取自子页参数卡的 `.pg-node`，按钮高亮选中态，
  事件委托避免重渲染丢绑定），"清除筛选"写空串恢复任意类别；
  `target_label`(string) / `target_class`(int) / `conf_threshold`(double)
  仍可在参数卡手动调。
- **KCF**（`kcf_grab` / `kcf_node`）：跟踪画面 `/kcf_node/tracking_image`，
  **画面上拖拽框选目标** —— pointer 事件画选框（`.bbox-rect` 覆盖层 +
  `setPointerCapture`，触屏可用），松手后把显示坐标经 object-fit:contain
  内容区换算成原始图像像素（`naturalWidth/Height`），发布
  `sensor_msgs/RegionOfInterest` 到 `/kcf_node/select_bbox`；流未加载
  （placeholder）或框小于 8×8 像素时忽略并提示。【重新播种 (HSV)】调
  `/kcf_node/reinit`（Trigger）；HSV 播种阈值滑条同 HSV 页组件。
- **ArUco**（`aruco_grab` / `aruco_node`）：节点无调试图发布，子页显示
  机械臂相机原图。
- **VLM**（`vlm_grab` / `vlm_node`）：框选画面 `/vlm_node/vlm_image`；指令发
  `/vlm/instruction`、结果订 `/vlm/result`、确认/取消发 `/vlm/confirm`。

**配套后端改动（grab_demo/kcf_track_node.cpp）**：新增 `~/select_bbox` 订阅
（sensor_msgs/RegionOfInterest），收到手动框即放弃当前跟踪、下一帧优先用
该框播种（一次性，优先级高于 init_bbox/HSV）；`~/reinit` 同时丢弃未消费的
手动框。默认单线程执行器下与图像回调串行，无需加锁。接口文档见
`grab_demo/KCF_GUIDE.md`。

**MJPEG 流的懒加载**：顶层 tab 切到机械臂时只拉当前激活子页的流；切换机械臂
子页时再拉对应子页的流（同功能模块策略），不会一次拉起五个方案的调试画面。

### 资源管理

- `/vlm/instruction`、`/vlm/confirm` 两个 publisher 随 `setupTopics()`
  advertise，`teardownTopics()` 时 unadvertise（同 cmd_vel 的处理）。
- 状态订阅均带 `throttle_rate`（100–200ms），关节表渲染与消息解耦
  （dirty 标记 + 200ms 定时器），高频 `/joint_states` 不会打爆 DOM。

### 文件改动汇总（第 4 轮）

| 文件 | 变化 |
| --- | --- |
| `web/index.html` | 导航栏加"机械臂" tab、新增 `#panel-arm`（子导航 + 监控与控制 / 五个抓取方案子页）、KCF 框选层、YOLO 物体按钮区、HSV 滑条组 |
| `web/app.js` | 机械臂订阅/发布、TriState 渲染、关节表、系统/夹爪/运动/抓取服务调用、事件时间线、机械臂子页 tab 组、流懒加载、KCF 拖拽框选、YOLO 目标按钮、HSV 滑条组件 |
| `web/style.css` | 子页样式扩展到 `.arm-subtab-*`、`.sys-btns`、`.grip-row`、`.arm-joint*`、`.mj-*`、`.bbox-*`、`.hsv-*`、`.arm-yolo-buttons` |
| `README.md` | 功能板块更新为六个，补机械臂子页明细与启动前提 |
| `../grab_demo/src/kcf_track_node.cpp` | 新增 `~/select_bbox` 手动框选接口 |
| `../grab_demo/KCF_GUIDE.md` | 记录框选接口与主面板用法 |

### 验证清单

- [ ] 导航栏出现"机械臂"，`http://<host>:8080/#arm` 直达；子页六个 tab 可切换，且不影响"功能模块"页的子页状态。
- [ ] 启动 `lebai_driver robot_state.launch.py` 后状态卡片有值；停掉 3 秒后回 "—"。
- [ ] 夹爪滑块"设置位置"能开合真实夹爪（`/gripper_status` 数值跟随）。
- [ ] "填入当前关节角"填进 `lebai_joint_1..6` 实时值；"执行运动"先弹确认。
- [ ] 启动 `grab_demo color_grab.launch.py`：HSV 子页能看到调试画面，"读取当前值"把滑条同步到节点当前阈值，拖动 H/S/V 滑条调试画面的色块掩码实时变化。
- [ ] 启动 `grab_demo yolo_ros_grab.launch.py`：YOLO 子页调试图有框+距离，识别到的物体出现为按钮（同类合并计数），点击某个按钮后调试图只跟该类别且按钮高亮，"清除筛选"恢复；各子页"目标距离"同步有读数，"抓取目标"能完成一次抓取并在时间线显示结果。
- [ ] 启动 `grab_demo kcf_grab.launch.py`（重编译 grab_demo 后）：KCF 子页在跟踪画面上拖拽框选一个物体，节点日志出现"收到手动框选"，跟踪框跳到所选物体；【重新播种 (HSV)】后回到色块播种。
- [ ] VLM 子页发指令后 `/vlm/result` 显示理解结果，"确认抓取"触发执行。
- [ ] "手动接管"后 `/arm_arbiter/state` 显示"手动接管"，自动抓取被拒绝；"释放控制权"恢复。
- [ ] 断开 rosbridge 时点任何按钮：时间线提示"未连接"，不报 JS 错。

---

## 第 3 轮 — 3D 视图重写（去掉 ros3djs / tf2_web_republisher）

### 背景

实车联调时 3D 视图全黑、激光不显示。浏览器控制台报：

```
Uncaught TypeError: Cannot read properties of undefined (reading 'getUniforms')   ros3d.min.js:1
THREE.WebGLRenderer 89
```

根因是 **ros3djs 1.1.0 与页面加载的 three.js 版本冲突**：ros3djs 自带/期望的是
旧版 THREE(r89)，而 index.html 显式加载了 three 0.118，两个 THREE 实例混用，
导致 ros3djs 的 `LaserScan` 自定义着色器材质在渲染时崩溃（`getUniforms`
读到 undefined），整个渲染循环挂掉。

同时 ros3djs 的 `TFClient` 依赖 `tf2_web_republisher`，而 roslibjs 在 ROS 2
下无法和它对接（节点跑着却收不到请求），就算渲染不崩，激光也没有坐标变换可用。

### 改动

**彻底放弃 ros3djs，3D 视图改用原生 three.js 重写：**

- `index.html`：移除 `ros3d.min.js`，改为加载 `three@0.118.3` +
  `examples/js/controls/OrbitControls.js`（单一 THREE 实例，消除版本冲突）。
- `buildViewer()`：自建 `THREE.Scene` / `PerspectiveCamera`（Z-up，匹配 ROS）
  / `WebGLRenderer` / `OrbitControls` / `GridHelper`(XY 平面) / `AxesHelper`，
  自己跑 `requestAnimationFrame` 渲染循环。
- **客户端 TF**：新增 `makeTfClient()`，直接订阅 `/tf` + `/tf_static`
  （rosbridge 原生转发，已验证可用），用四元数自己做变换合成
  （`quatMul`/`quatRotateVec`/`tfCompose`/`tfInverse`），提供
  `lookup(frame)` 返回该 frame 在 fixed frame 下的位姿。**不再需要
  tf2_web_republisher。**
- **激光层**：新增 `makeScanLayer()`，订阅 `/scan`，把 ranges 转成点、
  用 TF 变换到 fixed frame，用普通 `THREE.Points` + `THREE.PointsMaterial`
  渲染（避开 ros3djs 那个会崩的自定义着色器）。
- fixed frame 缺失提示改为基于 `makeTfClient` 的 `knows()`，5 秒内没出现就提示。
- `/map`(OccupancyGrid) 与 URDF 两个图层依赖 ros3djs，一并移除；对应的 UI
  输入框也从 `index.html` 删掉。Odom 轨迹本来就是原生 THREE 画的，保留。

### 实车排障记录（导致本轮的过程）

1. 远程连不上、报 `Can "Upgrade" only to "WebSocket"` → 是在浏览器直接开了
   rosbridge 端口 9090，应开面板的 **8080**。
2. 相机无画面 → `web_video_server` 因 `libboost_thread.so.1.75.0` 缺失启动即崩，
   系统实为 boost 1.74，**重新 `colcon build web_video_server`** 链接到 1.74 解决。
3. 激光无数据 → 双雷达融合 `double_lidar_fusion` 用 `ApproximateTime` 同步器，
   **必须两台雷达都有数据才输出 `/scan`**；雷达1(`/scan1`)故障 → `/scan` 一直空。
   临时方案：面板 `/scan` 改成 `/scan2` 看正常的雷达2。
4. 即便指向 `/scan2` 仍全黑 → 即本轮根因（ros3djs/THREE 版本冲突），故重写。

### 验证清单

- [ ] 面板 3D 视图能看到网格 + 坐标轴，可鼠标拖拽旋转/缩放（OrbitControls）。
- [ ] `/scan` 填 `/scan2`（或修好雷达1后填 `/scan`）点应用，能看到红色激光点云。
- [ ] 控制台不再有 `getUniforms` / ros3d 报错。
- [ ] 不启动 `tf2_web_republisher` 也能显示激光。
- [ ] fixed frame 填一个不存在的名字，5 秒后出现 TF 缺失提示。

---

## 第 2 轮 — 参数/相机/移动端改进

### 参数面板：类型感知 + 节点自动发现

**问题**
旧实现把所有参数都按 `double` (PT_DOUBLE=3) 写入：

```js
value: { type: 3, double_value: value, bool_value: false, ... }
```

对当前 `odom_*_scale` 四个 float 参数没问题，但一旦扩展到 int / bool /
string 就要改前端代码。读取时也只是把类型 dump 出原始 JS 值，没存回 row
上，所以下一次 apply 仍会按 double 发。

另外节点名输入框写死 `/wheeltec_robot`，多 namespace 部署时容易输错。

**修复**

- HTML 中给每个 `.param-row` 加 `data-type="double|int|bool|string"`
  ([web/index.html](web/index.html))。
- `app.js` 引入 `buildParameterValue(typeName, raw)`，按类型构造完整的
  `rcl_interfaces/ParameterValue`；同时引入 `PT_*` / `TYPE_NAME` /
  `NAME_TYPE` 常量表。
- `refreshParams()` 在收到服务响应后，根据 `val.type` 反向更新
  `row.dataset.type`，并在 `param-current` 旁显示 `value [type]`，这样
  即便 HTML 默认类型写错，下次 apply 也会用真实类型。
- 节点输入框加 `<datalist id="node-list">`，连接成功后调用
  `ros.getNodes()` 自动填入。`refreshNodeList()` 在 `setupTopics()`
  末尾触发。

### 相机：支持自定义 Base URL（修复 HTTPS 反向代理 mixed-content）

**问题**
`buildStreamUrl` 强制拼 `http://<location.hostname>:<port>/stream?...`。
当 dashboard 被 HTTPS 反向代理（nginx / cloudflare）后，浏览器会因
mixed content 拒绝加载 `http://` 的 MJPEG，整个相机区域全黑。

**修复**

- HTML 工具栏新增 `Base URL (覆盖端口)` 输入框（id=`cam-base`）。
- `videoBase()` 优先用 base URL，否则用 `${location.protocol}//host:port`
  （主动跟随当前页面协议，不再硬编码 `http:`）。
- 加 `change` 事件触发所有相机重载。
- CSS 中 `.cam-base-wrap` 自适应剩余宽度。

### roslibjs CDN 升级

`roslib@1.3.0` → `roslib@1.4.1`。1.3.0 在 ROS 2 上 `getNodes` /
`Service` 字段对接有几个已知小坑，升级一并解决；同时新本节点发现
特性需要 1.4+。

### 移动端 / 窄屏适配

`@media (max-width: 640px)` 段：

- header 允许换行，连接输入框占满宽度，避免被挤出右侧；
- 遥控按键由 56×56 放大到 72×72；急停按钮字号、padding 加大；
- 主网格 padding 缩到 10px；
- 日志行隐藏 `node` 列，让消息列拿到更多宽度。

### 文件改动汇总（第 2 轮）

| 文件 | 变化 |
| --- | --- |
| `web/index.html` | 参数行加 `data-type`、新增 `node-list` datalist、加 `cam-base` 输入框、CDN 改 1.4.1 |
| `web/app.js` | 类型感知参数读写、`refreshNodeList()`、`videoBase()` 支持 base URL |
| `web/style.css` | `cam-base-wrap` 样式、640px 移动端 media query |
| `CHANGELOG.md` | 新增本文件 |

---

## 第 1 轮 — 安全 / 资源管理 / 性能

提交：`dashboard: fix teleop safety, throttle high-rate topics, viewer feedback`

### 🔴 安全

1. **键盘 WASD 在输入框里也触发遥控**（重大）
   旧代码在 `document` 上挂 keydown/keyup，没判断 `e.target`。在
   `ws-url`、参数节点、相机 topic 等输入框打字时只要敲到 W/A/S/D，
   机器人就会走。
   **修复**：引入 `isTypingTarget(t)` 守卫，遇到 INPUT/TEXTAREA/SELECT/
   contenteditable 直接 return。

2. **键盘控制不连发，单次发送 + OS 自动重复不可靠**
   旧实现 keydown 只发一次 Twist，丢一帧就停。
   **修复**：用 `heldKeys: Set` 跟踪按下的 W/A/S/D，每 100ms (10Hz)
   重发；松开所有键时统一发 `(0,0)`；支持对角组合（W+A → 前左）。

3. **窗口失焦未释放按键**
   切到其它 tab 时不会收到 keyup，旧代码会让机器人一直跑。
   **修复**：监听 `window.blur` 清空 heldKeys 并发 `(0,0)`。

4. **新增急停按钮**（红色，遥控卡片下方）
   连发 3 次 `(0,0)`（立即 / 50ms / 150ms），抵御丢包。

### 🟡 资源管理

5. **`/cmd_vel` 断开时未 unadvertise**
   旧 teardown 只置 `cmdVelPub = null`，rosbridge 端的 advertise
   记录残留，反复断重连会泄漏。
   **修复**：teardown 时调用 `cmdVelPub.unadvertise()`。

6. **连接按钮无法主动断开**
   旧实现只支持 connect，再点击会重连。
   **修复**：按钮变成 "连接 / 断开" 二态，用 `userWantsConnected`
   跟踪用户意图；快速断开-重连时旧 `close` handler 通过 `if (ros !== r)
   return;` 忽略，不会把新连接的 `ros` 置 null。

7. **`setupTopics` monkey-patch 是定时炸弹**
   旧代码用 `setupTopics = function () { ... }` 在运行时改写函数
   声明，未来改成 `const`/箭头函数就直接报错（已经被 eslint 警告，
   靠 `// eslint-disable-next-line no-func-assign` 压住）。
   **修复**：把 `rebuildViewer()` 和 `subscribeRosout()` 直接写进
   `setupTopics()` 末尾。

### ⚡ 性能

8. **高频订阅未 throttle**
   `/odom`、`/imu/data_raw` 默认 ~50Hz，每条消息都触发 DOM 写入。
   **修复**：在 rosbridge 层加 `throttle_rate: 100`（约 10Hz），
   `/Distance` 同样处理。

### 🐞 正确性

9. **超声波 G / H 永远显示 0 且误标颜色**
   `wheeltec_robot.cpp:497-498` 注释掉了 G/H 字段（那两字节被自检
   payload 复用）。前端却一直按"距离"渲染，恒小于 0.3m → 红色。
   **修复**：HTML 删掉 G/H 两格；JS 的 keys 数组只保留 a-f；
   sonar grid CSS 改 3×2 更整齐；并加 `typeof v === 'number'` 守卫，
   避免 undefined 触发着色。

### 🗺️ 3D 视图

10. **viewer 尺寸只跟 `window.resize`**
    相邻卡片折叠/展开时 host 容器尺寸变了但 window 没变。
    **修复**：用 `ResizeObserver` 监听 `#viewer` 容器；浏览器不支持
    时降级回 `window.resize`。

11. **fixed frame 缺失时无任何反馈**
    EKF / robot_state_publisher 没启动时，LaserScan/Map/Odom 全部
    画不出来，用户没线索。
    **修复**：在 `rebuildViewer` 里同时订阅 `/tf` 和 `/tf_static`，
    扫描 `transforms[].header.frame_id` / `child_frame_id`，找到
    `fixedFrame` 才清掉提示；4 秒内没收到就显示 `未收到 frame "xxx"
    的 TF — 检查 robot_state_publisher / EKF 是否启动`。HTML 加
    `#viewer-status` 元素，CSS 给警告色样式。

### 文件改动汇总（第 1 轮）

| 文件 | 变化 |
| --- | --- |
| `web/app.js` | 连接按钮二态、teardown unadvertise、键盘守卫+连发、急停、throttle、setupTopics 内联、ResizeObserver、TF frame 探测、sonar A–F |
| `web/index.html` | 删 sonar G/H、加 viewer-status、加急停按钮 |
| `web/style.css` | sonar grid 3 列、`.e-stop`、`.viewer-status` |

---

## 验证清单（手动）

下次回归时可以照这个跑：

- [ ] 浏览器打开 `http://<host>:8080/`，自动连接 rosbridge 并显示"已连接"；按钮变 "断开"。
- [ ] 点 "断开" 后再点 "连接"，能正常重连；不会有第二个 advertise 残留（`ros2 topic info /cmd_vel`）。
- [ ] 把焦点放进 `ws-url` 或参数节点输入框，敲 `wasd`，**机器人不动**。
- [ ] 焦点离开输入框，按住 W 不松，`ros2 topic echo /cmd_vel` 看到稳定 10Hz；松开后立刻发 (0,0)。
- [ ] 按住 W 然后按 A：vx + wz 同时正；松 A 留 W：只剩 vx。
- [ ] 按 Space 或点 "急停 STOP"：立即收到三条 (0,0)。
- [ ] 切到其它浏览器 tab：`/cmd_vel` 立刻收到 (0,0)。
- [ ] 关掉 robot_state_publisher / EKF，刷新 viewer：4 秒后出现 TF 缺失提示。
- [ ] 参数面板点 "读取当前值"：显示形如 `1  [double]`，节点输入框下拉能看到当前所有 ROS 2 节点。
- [ ] 把 dashboard 放到 HTTPS 反向代理后，在 Base URL 填 `https://.../video`，相机能正常加载。
- [ ] 手机浏览器打开：header 不溢出，遥控键变大，急停可点。
