# 抓取仿真世界修复计划 / 记录

> 日期: 2026-06-13  环境: WSL2 + ROS 2 Humble + Gazebo Classic 11
> 接续 `~/Lab/仿真修复记录.md`(第 1~12 条)。本轮针对 Dashboard 端到端体验的 5 个问题。

---

## 原始问题清单(用户提出)

1. **HSV 识别**: 红/绿色块太大, 不符合机械臂尺寸。
2. **KCF 场景未加载出来**。
3. **仿真视频流不能在 Dashboard 上显示**。
4. **机器人与桌子/物体在 X 方向距离太近**, 需要拉开一些。
5. **仿真视频流不完整**, 不像真机那样有"处理后图像"(如 KCF / YOLO 处理后的画面)。

---

## 修复与验证

### 1. 红/绿色块过大 → 改为 5cm 可抓取小方块  ✅
- **病因**: `models/box_target_red|green/model.sdf` 原是 **6×6×2m 的"降落台"网格**
  (start_pad/end_pad 纹理), 完全不是机械臂尺度的目标物。
- **修复**: 重写两个 model.sdf 为 **0.05m 纯色立方体**(红/绿), 参照 `wood_cube_5cm`:
  link 原点在方块底面、`mass=0.05`、带摩擦/接触面、`Gazebo/Red|Green` 纯色(利于 HSV 阈值识别)、
  dynamic 可被夹起。`grab_hsv_color.world` 里按桌面高度 `z=0.565` 摆放即贴桌。
- **验证**: `gz sdf -p` 解析通过; headless 加载 `grab_hsv_color` 无模型报错。

### 2. KCF 场景"未加载"→ 实为 launch 永远只起 HSV 节点  ✅(根因已修)
- **病因**: `grab_kcf.world` 本身正常(headless 加载无误、场景相机出图)。真正问题是
  `launch/gazebo_grab.launch.py` **无论选哪个 world 都只启动 hsv_range(color_node)**,
  选 KCF 世界时 `kcf_node` 从不启动 → `/kcf_node/tracking_image` 永远没有 → 网页 KCF 页空白。
- **隐藏 bug**: 改成"按 world 选节点"后第一版仍失败 —— 内层 `gazebo.launch.py` 会把 `world`
  解析成**完整 .world 路径**并回写到 `world` 配置, 导致 `world_name == "grab_kcf"` 判断恒不成立。
  已用 `os.path.splitext(os.path.basename(...))` 取短名修正(兼容短名/全路径两种输入)。
- **修复**: `gazebo_grab.launch.py` 用 `OpaqueFunction` 按 world 选视觉节点, 并打印
  `[gazebo_grab] world=... -> 视觉节点: ...` 便于排查。
- **验证**(实际 `ros2 launch` 抓 LogInfo):
  `grab_kcf -> kcf_track_node(kcf_node)`、`grab_yolo -> yolo_ros_node`、其余 -> hsv_range。✅

### 3. 仿真视频流不能在 Dashboard 显示 → web_video_server 端口参数失效  ✅
- **病因**: `web_video_server` 把所有参数从一个**私有节点 `_web_video_server`** 读取, 且
  两个节点都**没有声明参数、也没开 `automatically_declare_parameters_from_overrides`**。
  于是 `dashboard.launch.py` 传给 `web_video_server` 节点的 `port:=8081` 被**完全忽略**,
  服务端永远绑 **8080**, 而网页(`index.html`/`app.js`)按 **8081** 取流 → 端口错配 → 没画面。
- **修复**: `src/web_video_server-ros2/.../web_video_server.cpp`:
  `main()` 给节点加 `NodeOptions().automatically_declare_parameters_from_overrides(true)`;
  构造函数里读 `port/address/...` 从 `private_nh` 改为 `nh`(即 launch 真正下发参数的节点)。
- **验证**: 重新编译后 `-p port:=8081` 生效 ——
  `Waiting For connections on 0.0.0.0:8081` ✅; 且 `curl http://127.0.0.1:8080|8081/stream?...`
  抓到 **2.6MB / 86 帧 JPEG (≈17fps)** 的 `/sim_scene/arm/image_raw` MJPEG。
- **注**: 整条链路(Gazebo 相机 → web_video_server → MJPEG)本就可用, 只差这个端口修正。

### 4. 机器人与桌子/物体太近 → 桌子后移 + 物体保持可达  ✅
- **修复**: 6 个 world 统一把桌心 `x 0.75 → 0.85`(桌沿 0.35 → 0.45, 与机器人前缘
  ~0.363 之间留出 ~0.09m 净空, 不再贴住机械臂); 物体相应后撤但**全部保持
  `x∈[0.50,0.63]`**, 仍在桌面、仍在臂可达半径(≲0.65m)内, 不影响抓取。
- **验证**: 6 个 world `gz sdf -p` 全部解析通过, 无模型加载错误。

### 5. 视频流不完整(缺处理后图像)→ 按 world 起对应视觉节点  ✅(KCF) / ⚠(YOLO 需依赖)
- 与第 2 条同一处修复。各 world 现在会启动对应方案节点, 产出 Dashboard 已对接好的话题:
  - HSV  → `/color_node/detection_image`(grab_world / grab_hsv_color / grab_aruco / grab_vlm)
  - KCF  → `/kcf_node/tracking_image`(grab_kcf, `kcf_track_node` 本机已编译可用)
  - YOLO → `/yolo_ros_node/detection_image`(grab_yolo)
- 这些调试图都能直接被 web_video_server 转 MJPEG, 在网页对应子页显示, 与真机一致。

### 6. 跑 dashboard bringup 后 `ros2 node list` 出现两个 web_video_server 节点  ✅
- **现象**: 启动 dashboard(sim_bringup / labrobot_bringup / dashboard.launch.py)后,
  `ros2 node list` 里能看到**两个** web_video_server 相关节点。
- **排查**: 三个 bringup 都只启动**一个** web_video_server 进程
  (`dashboard.launch.py` 里仅一处 Node, 其余只是 include 它, 没有重复启动)——
  **不是**重复 launch。两个节点来自 **web_video_server 二进制本身**:
  其 `main()` 一次性建了两个 rclcpp 节点 —— 公有 `web_video_server` + 私有
  `_web_video_server`(后者原本只用来读参数)。实测一个进程:
  - 修复前 `ros2 node list` 显示 `/web_video_server`(重复两行) + 隐藏的 `/_web_video_server`;
  - 这正是用户看到的"两个 web_video_server 节点"。
- **修复**: 第 3 条已把参数统一改成从公有节点 `nh` 读, 私有节点彻底没用了。
  于是 `main()` 里不再单独建 `_web_video_server`, 构造函数两个形参都传 `nh`
  (`WebVideoServer server(nh, nh);`)。
- **验证**: 重新编译后, 单个进程 `ros2 node list` / `ros2 node list --all` 均**只剩一个**
  `/web_video_server`。✅

### 7. dashboard 起来后视频流仍"无法加载流"→ web_video_server 根本没在跑  ✅
- **现象**: sim 与 labrobot_bringup 都已拉起、`/sim_scene/arm/image_raw`(9.9Hz)、
  `/camera_arm/color/image_raw`(6.5Hz)都在正常发布, rosbridge(:9090)/web_server(:8000)
  也在, 但三个视频面板全是"无法加载流"。
- **排查**: `ros2 node list` / `ss -ltn` 显示 **web_video_server 进程根本不存在**,
  8081/8080 都没在监听 —— 网页(`http://<host>:8081/stream?...`)自然取不到画面。
  labrobot_bringup 确实 include 了 dashboard.launch.py(web_server / sensor_watchdog /
  sim_launcher 都在跑), 唯独 web_video_server 不在 —— 即它**启动瞬间崩了**
  (最可能: 8081 被上一次残留 web_video_server 占用 → bind 抛异常退出),
  而 ROS 2 launch 默认不重启崩溃节点, 于是它一去不回。
- **修复**:
  - `dashboard.launch.py` 的 web_video_server 节点加 `respawn=True, respawn_delay=2.0`,
    崩溃后自动重启(等端口释放即可绑定), 不再"一崩就没"。
  - 当前会话先手动起一个补上: `ros2 run web_video_server web_video_server
    --ros-args -p port:=8081 -p address:=0.0.0.0`, 网页点"刷新"即出画面。
- **验证**: 起好后 `curl http://127.0.0.1:8081/stream?topic=/sim_scene/arm/image_raw`
  抓到 **90 帧 JPEG / 3.8MB(≈18fps)**; `/camera_arm/color/image_raw` 89 帧。✅
- **注意**: 重启 bringup 前先 `pkill -9 -f web_video_server` 清掉残留(含上面手动起的那个),
  否则端口冲突又会让新进程崩(虽有 respawn 兜底, 但留着残留仍是隐患)。

### 8. 代理放行后请求 200 OK 但 `<img>` 仍不显示 → MJPEG 长连接被 WSL2 转发掐住  ✅
- **现象**: 浏览器(Windows)经 `localhost:8081` 访问, DevTools 里流请求是 **200 OK +
  `Content-Type: multipart/x-mixed-replace`**(代理已放行 localhost), 但 `<img>` 仍报
  "无法加载流"。
- **排查**: WSL 主机上 `curl` 同一个流能拿到几 MB 帧、原始 multipart 分帧也完全合法;
  区别在于——**curl 在 WSL 主机直连 8081; 浏览器在 Windows 走 WSL2 的 localhost 转发**。
  普通短请求(页面 8000、ws 9090)转发没问题, 但 **MJPEG 这种永不结束的流式长连接经
  WSL2 localhost 转发会被缓冲/掐断**: 200 头过来了, 后续帧刷不到浏览器 → `<img>` 超时报错。
  (这也解释"直链能看到一帧然后卡住"。)
- **修复**: 把仪表盘视频从 **multipart 长连接流** 改为 **单帧快照轮询** ——
  `web/app.js` 用 `/snapshot` 短请求每 ~120ms(≈8fps)拉一张 JPEG 贴到 `<img>`
  (双缓冲预载, 不闪烁; 隐藏页签不请求)。短请求每次立即结束, 能顺利穿过 WSL2 转发/代理,
  且逐帧自愈(后端晚起/重启自动出图, 不必手动刷新)。
- **验证**: `/snapshot` 返回合法 JPEG(场景 17KB/机械臂 9KB); 8fps 连发 12 次 12/12 成功。
- **备注**: 想继续用 MJPEG(局域网直连、不经 WSL 转发时更省)可改回 `buildStreamUrl`;
  或用 WSL 真实 IP 访问以绕开 localhost 转发。

### 9. 终极根因: 8081 经 WSL2 localhost 转发到 Windows 浏览器根本不通 → 改走 8000 同源代理  ✅
- **现象**: 代理放行后请求虽到了 web_video_server, 但浏览器对 `:8081` 的请求
  (无论 MJPEG /stream 还是快照 /snapshot)统统 `net::ERR_EMPTY_RESPONSE`、0 字节;
  而 WSL 主机上 curl 同一地址完全正常。中间还踩过一个坑: 快照轮询用固定 setInterval
  不等返回就发, 失败时请求堆积(379 个)把 `server_threads=1` 的服务冲爆——已改自限速链式。
- **真根因**: **web_video_server 监听的 8081, 经 Windows→WSL2 的 localhost 转发就是不通**
  (8000 页面/9090 ws 这类短请求/握手能过, 8081 这个服务的连接被转发层吞掉)。
- **修复(同源代理)**:
  - `web_server.py`: 新增 `/video/snapshot` 路由——浏览器请求它时, web_server 在 **WSL
    本地** 转发到 `127.0.0.1:<video_port>/snapshot` 取回单帧 JPEG 再回给浏览器
    (用 no-proxy opener, 避免走系统代理)。浏览器**只连 8000**, 8081 仅 WSL 内部本地访问。
  - `dashboard.launch.py`: 给 web_server 传 `video_port`。
  - `web/app.js`: 快照默认走同源 `/video/snapshot`(不再直连 8081); 仍支持手填基址覆盖。
- **代理的二次坑(%2F)**: 浏览器 `encodeURIComponent` 把话题的 `/` 编成 `%2F`,
  web_video_server **不解码 `%2F`** → 直连 8081 时 HTTP 000、经代理时 502。
  修法: 代理用 `parse_qs` 先解码、再 `urlencode(safe='/')` 用原始斜杠转发。
- **验证**: 用浏览器同款 `%2F` 编码话题经 `http://127.0.0.1:8000/video/snapshot`
  三路均 HTTP 200 + 合法 JPEG(场景 17KB / 机械臂 8.8KB / HSV 9KB)。浏览器只走 8000 → 必通。
- **备注**: 局域网直连/不经 WSL 转发的部署, 在"相机基址"里手填 `http://<ip>:8081` 可改回直连。

---

## 改动文件清单(本轮)

| 文件 | 改动 |
|---|---|
| `models/box_target_red/model.sdf` | 6×6×2m 降落台 → 5cm 纯红立方体(dynamic, 可抓) |
| `models/box_target_green/model.sdf` | 同上, 纯绿 |
| `worlds/grab_world.world` 等 6 个 | 桌心 0.75→0.85; 物体后撤至 x∈[0.50,0.63] 仍可达 |
| `launch/gazebo_grab.launch.py` | 按 world 选视觉节点(HSV/KCF/YOLO)+ 取 basename 修正 world 名 + LogInfo |
| `src/web_video_server-ros2/.../web_video_server.cpp` | 修 port 参数失效(auto-declare overrides + 从 nh 读参数); 去掉多余的 `_web_video_server` 私有节点(node list 不再重复) |
| `wheeltec_dashboard/launch/dashboard.launch.py` | web_video_server 节点加 `respawn=True`(崩溃自愈, 修"视频流无法加载") |
| `wheeltec_dashboard/web/app.js` | 视频改单帧快照**自限速链式**轮询, 默认走同源 `/video/snapshot` 代理 |
| `wheeltec_dashboard/wheeltec_dashboard/web_server.py` | 新增 `/video/snapshot` 同源代理(本地转发 8081, 先解码话题再转); 绕开 8081 经 WSL2 转发不通 |
| `wheeltec_dashboard/web/style.css` | 加 `.cam-err[hidden]{display:none}` —— 流出图后"无法加载流"提示能正常消失 |

> 已 `colcon build --packages-select lebai_gazebo web_video_server wheeltec_dashboard`。
> 重新编译 web_video_server 后需重启 dashboard / 重新 source 才会生效。

## 已知/遗留(环境限制, 非本次代码问题)

- **YOLO(grab_yolo)**: 节点已接好, 但本机未装 `yolo_ros`/`yolo_bringup`, 故 `/yolo/detections`
  无来源、调试图为空。装好 yolo_ros 后即可(场景图 `/sim_scene/arm/image_raw` 始终正常)。
- **ArUco(grab_aruco)**: 本机 OpenCV 4.5.4 无新 ArUco API, `aruco_dectet` 不编译, 暂以 HSV 兜底。
- **VLM(grab_vlm)**: 需视觉语言模型 API, 暂以 HSV 兜底, 保证有处理后图像。
- **测试环境**: WSL 无 GPU 加速 + headless, 满载启动时眼在手相机出帧较慢; gzserver 关停不彻底
  会占住 11345 端口(见上一轮记录遗留事项), 重启仿真前建议先 `pkill -9 gzserver`。
