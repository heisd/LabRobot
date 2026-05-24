# wheeltec_dashboard

A lightweight web-based dashboard for the Wheeltec S300 robot. Visualizes
telemetry from `turn_on_wheeltec_robot`, drives the chassis through
`/cmd_vel`, and tunes ROS 2 parameters on the running robot node.

## 组成

- `rosbridge_websocket` — 浏览器 ↔ ROS 2 桥接 (默认 `ws://<host>:9090`)
- 一个 Python 节点 `web_server` — 用 `http.server` 静态托管前端 (默认 `http://<host>:8080`)
- 前端：原生 HTML/CSS/JS + [`roslibjs`](https://github.com/RobotWebTools/roslibjs) + [`Chart.js`](https://www.chartjs.org/) (走 CDN)

## 依赖

```bash
sudo apt install ros-humble-rosbridge-suite
```

构建：

```bash
colcon build --packages-select wheeltec_dashboard
source install/setup.bash
```

## 使用

```bash
ros2 launch wheeltec_dashboard dashboard.launch.py
```

然后在浏览器打开 `http://<机器人IP>:8080/`，页面默认会连接到
`ws://<同一host>:9090` (rosbridge)。

启动参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `http_port` | `8080` | dashboard HTTP 端口 |
| `ws_port` | `9090` | rosbridge_websocket 端口 |
| `address` | `0.0.0.0` | HTTP 监听地址 |

例：`ros2 launch wheeltec_dashboard dashboard.launch.py http_port:=8000`

## 功能

- 实时遥测：`/PowerVoltage`、`/robot_charging_flag`、`/robot_charging_current`、
  `/robot_red_flag`、`/self_check_data`、`/odom`、`/imu/data_raw`、`/Distance`
- 速度控制：方向按键 + 线/角速度上限滑块 + 键盘 WASD/空格 (停)
- 参数调节：通过 `rcl_interfaces/GetParameters`/`SetParameters` 服务读写
  `/wheeltec_robot` 上的 `odom_x_scale`、`odom_y_scale`、
  `odom_z_scale_positive`、`odom_z_scale_negative`
- 折线图：电压、cmd_vel (vx / wz)

## 网络说明

前端用到的 `roslibjs` 与 `Chart.js` 通过 CDN 加载，第一次使用需联网。
如果是离线环境，把对应 JS 下载到 `web/` 并改 `index.html` 中的引用即可。
