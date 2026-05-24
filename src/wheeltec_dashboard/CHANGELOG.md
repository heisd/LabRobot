# wheeltec_dashboard 变更记录

本文件记录 `wheeltec_dashboard` 面板的 bug 修复与改进，按时间倒序排列。
所有修改都在分支 `claude/inspiring-thompson-LU7ql` 上完成。

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
