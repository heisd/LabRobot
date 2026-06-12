"""sim_launcher — 从 Dashboard(网页)启动/停止 Gazebo 仿真的"launch 服务"。

为什么用话题而不是自定义 service:
  前端经 rosbridge 用 roslibjs 收发 std_msgs/String 最省事(无需编译自定义接口包)。
  - 订阅 ``/sim_launch/cmd`` (std_msgs/String): 收命令
  - 发布 ``/sim_launch/status`` (std_msgs/String, JSON): 广播各仿真在线状态(1Hz + 变更即发)

命令语法(空格分隔, 大小写不敏感):
  ``start <key> <world>``   启动某仿真的某世界(同 key 已在跑会先停再起)
  ``stop  <key>``           停止某仿真
  ``stop_all``              全部停止
  ``status``                立即广播一次状态

安全: **只允许白名单内的 key/world**, 拼出的命令固定为
``ros2 launch <pkg> <launch_file> world:=<world> [固定附加参数]``, 不接受任意命令,
避免网页侧任意命令执行。每个仿真以独立进程组启动, 停止时给整组发 SIGINT(等价 Ctrl+C),
让 gazebo 干净退出。

参数:
  ``enable`` (bool, 默认 True) — 置 False 则只广播状态、忽略 start/stop(纯只读)。
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ---- 白名单: key -> (包, launch 文件, {world: 额外固定参数列表}) ----
# world 列表即各功能世界; value 是该世界要附加的固定参数(如巡线从线路起点出生)。
CATALOG = {
    "arm": {
        "pkg": "lebai_gazebo",
        "launch": "gazebo_grab.launch.py",
        "label": "机械臂抓取仿真 (lebai_gazebo)",
        "worlds": {
            "grab_world": [],
            "grab_hsv_color": [],
            "grab_yolo": [],
            "grab_kcf": [],
            "grab_aruco": [],
            "grab_vlm": [],
        },
    },
    "chassis": {
        "pkg": "wheeltec_gazebo",
        "launch": "gazebo.launch.py",
        "label": "机器人底盘仿真 (wheeltec_gazebo)",
        "worlds": {
            "wheeltec_slam_nav": [],
            "wheeltec_rrt_explore": ["x:=-3.0", "y:=-3.0"],
            "wheeltec_line_follow": ["x:=-3.0", "y:=0.0"],
            "wheeltec_target_follow": [],
            "wheeltec_vla_nav": [],
            "wheeltec_nav": ["x:=-3.0", "y:=-3.0"],
        },
    },
    # Gazebo 导航世界 + Nav2 一起起(nav2_sim.launch.py 默认 slam:=True 边建图边导航,
    # /goal_pose 直接驱动 Nav2, 免存图免初始位姿)。与 chassis 互斥使用(都开 Gazebo)。
    "nav": {
        "pkg": "wheeltec_gazebo",
        "launch": "nav2_sim.launch.py",
        "label": "Nav2 导航仿真 (Gazebo + Nav2)",
        "worlds": {
            "wheeltec_nav": [],
            "wheeltec_slam_nav": [],
            "wheeltec_vla_nav": [],
        },
    },
}


class _Proc:
    def __init__(self, popen: subprocess.Popen, world: str, cmd: str):
        self.popen = popen
        self.world = world
        self.cmd = cmd


class SimLauncher(Node):
    def __init__(self) -> None:
        super().__init__("sim_launcher")
        self.declare_parameter("enable", True)
        self._enable = bool(self.get_parameter("enable").value)

        self._lock = threading.Lock()
        self._procs: dict[str, _Proc] = {}

        self._status_pub = self.create_publisher(String, "/sim_launch/status", 10)
        self.create_subscription(String, "/sim_launch/cmd", self._on_cmd, 10)
        self.create_timer(1.0, self._tick)

        mode = "可启停" if self._enable else "只读(enable=false)"
        self.get_logger().info(
            f"sim_launcher 就绪({mode})。命令→/sim_launch/cmd, 状态←/sim_launch/status。"
        )
        self._publish_status()

    # ---------------- 命令处理 ----------------
    def _on_cmd(self, msg: String) -> None:
        parts = (msg.data or "").strip().split()
        if not parts:
            return
        verb = parts[0].lower()
        try:
            if verb == "status":
                self._publish_status()
            elif verb == "stop_all":
                self._guard() and self._stop_all()
            elif verb == "stop" and len(parts) >= 2:
                self._guard() and self._stop(parts[1].lower())
            elif verb == "start" and len(parts) >= 3:
                self._guard() and self._start(parts[1].lower(), parts[2])
            else:
                self.get_logger().warn(f"无法识别的命令: {msg.data!r}")
        except Exception as exc:  # noqa: BLE001 - 任何异常都不该让节点崩
            self.get_logger().error(f"处理命令 {msg.data!r} 出错: {exc}")
        self._publish_status()

    def _guard(self) -> bool:
        if not self._enable:
            self.get_logger().warn("enable=false, 忽略启停命令(仅广播状态)。")
            return False
        return True

    def _start(self, key: str, world: str) -> None:
        spec = CATALOG.get(key)
        if not spec:
            self.get_logger().warn(f"未知仿真 key: {key}")
            return
        if world not in spec["worlds"]:
            self.get_logger().warn(f"{key} 不允许的世界: {world}")
            return
        # 同 key 已在跑 -> 先停再起(切换世界)
        self._stop(key)
        extra = spec["worlds"][world]
        # argv 各元素来源: 常量 + CATALOG 白名单常量; world 已通过上方白名单成员检查,
        # 列表参数且不经 shell, 外部输入只能"从白名单选一项", 无法注入任意命令。
        argv = ["ros2", "launch", spec["pkg"], spec["launch"], f"world:={world}", *extra]
        cmd_str = " ".join(argv)
        self.get_logger().info(f"启动 {key}: {cmd_str}")
        # 独立进程组, 停止时可整组 SIGINT; 继承当前(已 source)的环境
        popen = subprocess.Popen(  # nosemgrep: python.lang.security.audit.dangerous-subprocess-use-audit
            argv,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
            env=os.environ.copy(),
        )
        with self._lock:
            self._procs[key] = _Proc(popen, world, cmd_str)

    def _stop(self, key: str) -> None:
        with self._lock:
            proc = self._procs.pop(key, None)
        if not proc:
            return
        if proc.popen.poll() is None:
            self.get_logger().info(f"停止 {key} (SIGINT 进程组 {proc.popen.pid})")
            try:
                os.killpg(os.getpgid(proc.popen.pid), signal.SIGINT)
            except ProcessLookupError:
                pass

    def _stop_all(self) -> None:
        for key in list(self._procs.keys()):
            self._stop(key)

    # ---------------- 状态广播 ----------------
    def _tick(self) -> None:
        # 回收已退出的进程(用户在终端关了 gazebo, 或自己崩了)
        changed = False
        with self._lock:
            for key, proc in list(self._procs.items()):
                if proc.popen.poll() is not None:
                    self._procs.pop(key, None)
                    changed = True
        self._publish_status()
        if changed:
            self.get_logger().info("有仿真进程已退出, 状态已更新。")

    def _publish_status(self) -> None:
        items = {}
        with self._lock:
            for key, spec in CATALOG.items():
                proc = self._procs.get(key)
                alive = bool(proc and proc.popen.poll() is None)
                items[key] = {
                    "label": spec["label"],
                    "pkg": spec["pkg"],
                    "launch": spec["launch"],
                    "worlds": list(spec["worlds"].keys()),
                    "running": alive,
                    "world": proc.world if alive else None,
                    "pid": proc.popen.pid if alive else None,
                }
        msg = String()
        msg.data = json.dumps({"enable": self._enable, "items": items}, ensure_ascii=False)
        self._status_pub.publish(msg)

    def destroy_node(self) -> bool:
        try:
            self._stop_all()
        except Exception:  # pragma: no cover - best effort
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimLauncher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
