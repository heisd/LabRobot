# wheeltec_dashboard 变更记录

本文件记录 `wheeltec_dashboard` 面板的 bug 修复与改进，按时间倒序排列。
第 1–3 轮在分支 `claude/inspiring-thompson-LU7ql` 上完成，第 4–5 轮在
`claude/happy-ride-lzphg0` 上完成。

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
