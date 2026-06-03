#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""乐白机械臂 Web Dashboard 节点.

提供一个网页控制面板, 严格对齐 lebai-ros-sdk-humble-dev 暴露的 ROS2 接口:

订阅(状态显示, 来自 robot_state 节点):
    /joint_states    sensor_msgs/JointState
    /robot_status    lebai_interfaces/RobotStatus
    /io_status       lebai_interfaces/IOStatus
    /gripper_status  lebai_interfaces/GripperStatus

服务客户端(按钮命令):
    system_service (std_srvs/Empty):
        emergency_stop / power_on / power_off / enable / disable /
        pause_motion / resume_motion / abort_motion /
        entry_teach_mode / exit_teach_mode / turn_off_robot
    io_service:
        /io_service/set_gripper_position  lebai_interfaces/SetGripper
        /io_service/set_gripper_force     lebai_interfaces/SetGripper
        /io_service/set_robot_do          lebai_interfaces/SetDO
        /io_service/set_robot_ao          lebai_interfaces/SetAO
    motion_service:
        /motion_service/move_joint        lebai_interfaces/MoveJoint
        /motion_service/move_line         lebai_interfaces/MoveLine

设计说明:
    - 仅使用 Python 标准库(http.server), 无额外 pip 依赖, 适合 Jetson。
    - 纯网页形式, 不弹任何本地窗口(headless 安全), 浏览器远程访问即可。
    - HTTP 服务跑在守护线程, rclpy 在主线程 spin; 状态用锁保护的缓存共享。
"""

import json
import math
import os
import signal
import subprocess
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String, Bool
from std_srvs.srv import Empty
from lebai_interfaces.msg import RobotStatus, IOStatus, GripperStatus
from lebai_interfaces.srv import SetGripper, SetDO, SetAO, MoveJoint, MoveLine


# 系统服务(std_srvs/Empty)命令列表: 命令名 -> 中文标签
SYSTEM_COMMANDS = [
    ("power_on", "上电"),
    ("power_off", "断电"),
    ("enable", "使能"),
    ("disable", "去使能"),
    ("pause_motion", "暂停运动"),
    ("resume_motion", "恢复运动"),
    ("abort_motion", "中止运动"),
    ("entry_teach_mode", "进入示教"),
    ("exit_teach_mode", "退出示教"),
    ("emergency_stop", "急停"),
    ("turn_off_robot", "关闭控制器"),
]

# 被认为是"危险"的命令, 网页端会二次确认
DANGEROUS_COMMANDS = {"power_off", "emergency_stop", "turn_off_robot", "disable"}


# 功能启动页可一键启动/停止的任务(固定白名单, 不接受网页传入任意命令)。
# 每项通过 ros2 launch 启动一整套功能, 由 Dashboard 以子进程方式管理。
#
# "resources" 声明该任务会占用哪些资源, 两个任务只要资源有交集即视为冲突,
# 前端据此禁用"启动"并提示。资源对应实际会拉起的东西:
#   camera                = 机械臂相机 (gemini_arm)
#   robot_state/motion/io_service/system_service = 对应驱动节点
#   moveit                = MoveIt move_group
#   grab                  = camera_info + 抓取服务等抓取专用节点
# 视觉抓取 launch 内部包含 lm3.launch.py(= robot_interface 全部驱动 + MoveIt) + 相机,
# 所以它们占用上述几乎所有资源, 互相之间以及与驱动/MoveIt 单独启动都会冲突。
LAUNCH_TASKS = [
    # ---- 视觉抓取(各自包含相机 + 机械臂驱动 + MoveIt + 抓取服务) ----
    {"id": "yolo_grab", "label": "YOLO 抓取", "group": "视觉抓取",
     "cmd": ["ros2", "launch", "grab_demo", "yolo_grab.launch.py"],
     "resources": ["camera", "robot_state", "motion", "io_service", "system_service", "moveit", "grab"]},
    {"id": "color_grab", "label": "HSV/颜色 抓取", "group": "视觉抓取",
     "cmd": ["ros2", "launch", "grab_demo", "color_grab.launch.py"],
     "resources": ["camera", "robot_state", "motion", "io_service", "system_service", "moveit", "grab"]},
    {"id": "kcf_grab", "label": "KCF 跟踪抓取", "group": "视觉抓取",
     "cmd": ["ros2", "launch", "grab_demo", "kcf_grab.launch.py"],
     "resources": ["camera", "robot_state", "motion", "io_service", "system_service", "moveit", "grab"]},
    {"id": "vlm_grab", "label": "VLM 语言抓取", "group": "视觉抓取",
     "cmd": ["ros2", "launch", "grab_demo", "vlm_grab.launch.py"],
     "resources": ["camera", "robot_state", "motion", "io_service", "system_service", "moveit", "grab"]},
    {"id": "aruco_grab", "label": "ArUco 抓取", "group": "视觉抓取",
     "cmd": ["ros2", "launch", "grab_demo", "aruco_grab.launch.py"],
     "resources": ["camera", "robot_state", "motion", "io_service", "system_service", "moveit", "grab"]},
    {"id": "hand_eye", "label": "手眼标定", "group": "视觉抓取",
     "cmd": ["ros2", "launch", "grab_demo", "hand_eye.launch.py"],
     "resources": ["camera", "robot_state", "motion", "io_service", "system_service", "moveit", "grab"]},
    # ---- 机器人驱动(底层节点, 可单独启动; 互不冲突, 但与视觉抓取/MoveIt 冲突) ----
    {"id": "robot_state", "label": "机器人状态 robot_state", "group": "机器人驱动",
     "cmd": ["ros2", "launch", "lebai_driver", "robot_state.launch.py"],
     "resources": ["robot_state"]},
    {"id": "io_service", "label": "IO 服务 io_service", "group": "机器人驱动",
     "cmd": ["ros2", "launch", "lebai_driver", "io_service.launch.py"],
     "resources": ["io_service"]},
    {"id": "system_service", "label": "系统服务 system_service", "group": "机器人驱动",
     "cmd": ["ros2", "launch", "lebai_driver", "system_service.launch.py"],
     "resources": ["system_service"]},
    {"id": "motion", "label": "运动服务 motion", "group": "机器人驱动",
     "cmd": ["ros2", "launch", "lebai_driver", "motion.launch.py"],
     "resources": ["motion"]},
    # ---- 运动规划(= robot_interface 全部驱动 + MoveIt) ----
    {"id": "moveit_lm3", "label": "MoveIt (lm3)", "group": "运动规划",
     "cmd": ["ros2", "launch", "lebai_lm3_moveit_config", "lm3.launch.py"],
     "resources": ["robot_state", "motion", "io_service", "system_service", "moveit"]},
    # ---- 仿真(Gazebo, 与真机任务资源不冲突, 仿真任务之间互斥) ----
    {"id": "sim_gazebo", "label": "Gazebo 仿真场景 + 机械臂", "group": "仿真",
     "cmd": ["ros2", "launch", "lebai_gazebo", "gazebo.launch.py"],
     "resources": ["sim"]},
]

TASKS_BY_ID = {t["id"]: t for t in LAUNCH_TASKS}


class TaskManager:
    """以子进程方式启动/停止预定义的 ros2 launch 任务, 并跟踪运行状态。

    - 每个任务用独立进程组启动(start_new_session=True), 停止时对整个进程组发信号,
      以便干净地关闭 ros2 launch 拉起的所有子节点。
    - 子进程输出重定向到日志文件, 供网页查看。
    """

    def __init__(self, logger, log_dir=None):
        self._logger = logger
        self._lock = threading.Lock()
        self._procs = {}  # id -> {proc, logpath, logf, started}
        self._log_dir = log_dir or os.path.join(
            tempfile.gettempdir(), "lebai_dashboard_logs")
        os.makedirs(self._log_dir, exist_ok=True)

    @staticmethod
    def _running(info):
        return info is not None and info["proc"].poll() is None

    def start(self, task):
        tid = task["id"]
        with self._lock:
            if self._running(self._procs.get(tid)):
                pid = self._procs[tid]["proc"].pid
                return False, f"[{task['label']}] 已在运行 (pid={pid})"
            logpath = os.path.join(self._log_dir, f"{tid}.log")
            try:
                logf = open(logpath, "wb")
                proc = subprocess.Popen(
                    task["cmd"], stdout=logf, stderr=subprocess.STDOUT,
                    start_new_session=True, env=os.environ.copy())
            except FileNotFoundError as e:
                return False, f"启动失败: {e} (ros2 是否在 PATH / 工作空间是否 source?)"
            except Exception as e:  # noqa: BLE001
                return False, f"启动失败: {e}"
            self._procs[tid] = {"proc": proc, "logpath": logpath,
                                "logf": logf, "started": time.time()}
            self._logger.info(f"启动任务 [{task['label']}] pid={proc.pid}: "
                              f"{' '.join(task['cmd'])}")
            return True, f"[{task['label']}] 已启动 (pid={proc.pid})"

    def stop(self, task):
        tid = task["id"]
        with self._lock:
            info = self._procs.get(tid)
            if not self._running(info):
                return False, f"[{task['label']}] 未在运行"
            proc = info["proc"]
            try:  # 先对进程组发 SIGINT, 让 ros2 launch 优雅关闭
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
        # 在锁外等待退出, 超时则强杀
        try:
            proc.wait(timeout=6)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        self._logger.info(f"停止任务 [{task['label']}]")
        return True, f"[{task['label']}] 已停止"

    def status(self):
        now = time.time()
        out = []
        with self._lock:
            for task in LAUNCH_TASKS:
                info = self._procs.get(task["id"])
                running = self._running(info)
                out.append({
                    "id": task["id"],
                    "label": task["label"],
                    "group": task["group"],
                    "cmd": " ".join(task["cmd"]),
                    "resources": task.get("resources", []),
                    "running": running,
                    "pid": info["proc"].pid if running else None,
                    "uptime": round(now - info["started"], 1) if running else None,
                })
        return out

    def log(self, tid, lines=120):
        info = self._procs.get(tid)
        path = info["logpath"] if info else os.path.join(self._log_dir, f"{tid}.log")
        if not os.path.exists(path):
            return "(暂无日志)"
        try:
            with open(path, "rb") as f:
                data = f.read()
        except OSError as e:
            return f"(读取日志失败: {e})"
        text = data.decode("utf-8", "replace")
        return "\n".join(text.splitlines()[-lines:]) or "(日志为空)"

    def shutdown_all(self):
        with self._lock:
            items = list(self._procs.items())
        for tid, info in items:
            if self._running(info):
                try:
                    os.killpg(os.getpgid(info["proc"].pid), signal.SIGINT)
                except ProcessLookupError:
                    pass


class DashboardNode(Node):
    """聚合机械臂状态与控制服务, 对外提供 HTTP 接口。"""

    def __init__(self):
        super().__init__("dashboard_node")

        # ---- 参数 ----
        self.http_host_ = self.declare_parameter("http_host", "0.0.0.0").value
        self.http_port_ = int(self.declare_parameter("http_port", 8080).value)
        self.system_ns_ = self.declare_parameter("system_service_ns", "/system_service").value
        self.io_ns_ = self.declare_parameter("io_service_ns", "/io_service").value
        self.motion_ns_ = self.declare_parameter("motion_service_ns", "/motion_service").value

        # ---- 状态缓存(被订阅回调更新, HTTP 线程读取) ----
        self._lock = threading.Lock()
        self._cache = {
            "joint_states": (None, 0.0),
            "robot_status": (None, 0.0),
            "io_status": (None, 0.0),
            "gripper_status": (None, 0.0),
            "target_distance": (None, 0.0),
            "vlm_result": (None, 0.0),
        }

        # ---- 订阅状态话题 ----
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: self._store("joint_states", m), 10)
        self.create_subscription(RobotStatus, "/robot_status",
                                 lambda m: self._store("robot_status", m), 10)
        self.create_subscription(IOStatus, "/io_status",
                                 lambda m: self._store("io_status", m), 10)
        self.create_subscription(GripperStatus, "/gripper_status",
                                 lambda m: self._store("gripper_status", m), 10)
        # 目标距离(米), 由当前视觉算法(HSV/YOLO/KCF)经 TargetTFPublisher 发布
        self.create_subscription(Float32, "/grab_target/distance",
                                 lambda m: self._store("target_distance", m), 10)
        # VLM 自然语言抓取: 发送指令 + 接收结果
        self._vlm_pub = self.create_publisher(String, "/vlm/instruction", 10)
        self._vlm_confirm_pub = self.create_publisher(Bool, "/vlm/confirm", 10)
        self.create_subscription(String, "/vlm/result",
                                 lambda m: self._store("vlm_result", m), 10)

        # ---- 服务客户端 ----
        self._sys_clients = {
            name: self.create_client(Empty, f"{self.system_ns_}/{name}")
            for name, _ in SYSTEM_COMMANDS
        }
        self._cli_gripper_pos = self.create_client(
            SetGripper, f"{self.io_ns_}/set_gripper_position")
        self._cli_gripper_force = self.create_client(
            SetGripper, f"{self.io_ns_}/set_gripper_force")
        self._cli_set_do = self.create_client(SetDO, f"{self.io_ns_}/set_robot_do")
        self._cli_set_ao = self.create_client(SetAO, f"{self.io_ns_}/set_robot_ao")
        self._cli_move_joint = self.create_client(MoveJoint, f"{self.motion_ns_}/move_joint")
        self._cli_move_line = self.create_client(MoveLine, f"{self.motion_ns_}/move_line")

        # ---- 功能启动任务管理器(YOLO/HSV 抓取等一键启停) ----
        self.tasks_ = TaskManager(self.get_logger())

        # ---- HTTP 服务 ----
        self._httpd = None
        self._http_thread = None
        self._start_http_server()

        self.get_logger().info(
            f"Dashboard 已启动, 浏览器访问 http://<本机IP>:{self.http_port_}/")

    # ------------------------------------------------------------------
    # 状态缓存
    # ------------------------------------------------------------------
    def _store(self, key, msg):
        with self._lock:
            self._cache[key] = (msg, time.time())

    @staticmethod
    def _tri(tristate):
        """TriState.val -> 字符串 ('ON'/'OFF'/'UNKNOWN')。"""
        v = getattr(tristate, "val", -1)
        if v == 1:
            return "ON"
        if v == 0:
            return "OFF"
        return "UNKNOWN"

    def build_status(self):
        """汇总当前状态为可 JSON 化的字典。"""
        now = time.time()
        with self._lock:
            cache = dict(self._cache)

        def fresh(key, max_age=1.5):
            msg, ts = cache.get(key, (None, 0.0))
            return msg, (msg is not None and (now - ts) < max_age), round(now - ts, 2) if msg else None

        out = {"online": {}, "age": {}}

        # robot_status
        rs, ok, age = fresh("robot_status")
        out["online"]["robot_status"] = ok
        out["age"]["robot_status"] = age
        if rs is not None:
            mode_map = {-1: "UNKNOWN", 1: "MANUAL", 2: "AUTO"}
            out["robot"] = {
                "e_stopped": self._tri(rs.e_stopped),
                "drives_powered": self._tri(rs.drives_powered),
                "motion_possible": self._tri(rs.motion_possible),
                "in_motion": self._tri(rs.in_motion),
                "in_error": self._tri(rs.in_error),
                "error_code": int(rs.error_code),
                "mode": mode_map.get(int(rs.mode.val), str(rs.mode.val)),
            }

        # joint_states
        js, ok, age = fresh("joint_states")
        out["online"]["joint_states"] = ok
        out["age"]["joint_states"] = age
        if js is not None:
            out["joints"] = {
                "names": list(js.name),
                "positions_rad": [round(p, 4) for p in js.position],
                "positions_deg": [round(p * 180.0 / math.pi, 2) for p in js.position],
                "velocities": [round(v, 4) for v in js.velocity],
            }

        # gripper_status
        gs, ok, age = fresh("gripper_status")
        out["online"]["gripper_status"] = ok
        out["age"]["gripper_status"] = age
        if gs is not None:
            out["gripper"] = {"position": round(gs.position, 2), "force": round(gs.force, 2)}

        # io_status
        ios, ok, age = fresh("io_status")
        out["online"]["io_status"] = ok
        out["age"]["io_status"] = age
        if ios is not None:
            out["io"] = {
                "robot_din": [bool(b) for b in ios.robot_din],
                "robot_dout": [bool(b) for b in ios.robot_dout],
                "robot_ain": [round(float(a), 3) for a in ios.robot_ain],
                "flange_din": [bool(b) for b in ios.flange_din],
                "extend_din": [bool(b) for b in ios.extend_din],
            }

        # 目标距离(米); 视觉算法检测到目标时才会持续刷新, 无目标则很快变 stale
        td, ok, age = fresh("target_distance")
        out["online"]["target_distance"] = ok
        out["age"]["target_distance"] = age
        if td is not None:
            out["target_distance"] = round(float(td.data), 3)

        # VLM 最近一次结果(常驻显示, 不按 1.5s 过期)
        vr, _, vr_age = fresh("vlm_result", max_age=1e12)
        if vr is not None:
            out["vlm_result"] = {"text": vr.data, "age": vr_age}

        out["stamp"] = round(now, 3)
        return out

    # ------------------------------------------------------------------
    # 命令分发(均为非阻塞 call_async, 立即返回)
    # ------------------------------------------------------------------
    def dispatch(self, payload):
        """处理来自网页的命令, 返回 (ok, message)。"""
        cmd = payload.get("type", "")
        try:
            if cmd == "system":
                return self._call_system(payload.get("name", ""))
            if cmd == "gripper_position":
                return self._call_gripper(self._cli_gripper_pos, payload, "夹爪位置")
            if cmd == "gripper_force":
                return self._call_gripper(self._cli_gripper_force, payload, "夹爪力度")
            if cmd == "set_do":
                return self._call_set_do(payload)
            if cmd == "set_ao":
                return self._call_set_ao(payload)
            if cmd == "move_joint":
                return self._call_move_joint(payload)
            if cmd == "vlm":
                return self._send_vlm(payload)
            if cmd == "vlm_confirm":
                return self._send_vlm_confirm(payload)
            return False, f"未知命令类型: {cmd}"
        except Exception as e:  # noqa: BLE001 - 网页错误需返回给前端
            self.get_logger().error(f"命令执行异常: {e}")
            return False, f"异常: {e}"

    def dispatch_task(self, payload):
        """处理功能启动页的启动/停止请求, 返回 (ok, message)。"""
        tid = payload.get("id", "")
        action = payload.get("action", "")
        task = TASKS_BY_ID.get(tid)
        if task is None:
            return False, f"未知任务: {tid}"
        if action == "start":
            return self.tasks_.start(task)
        if action == "stop":
            return self.tasks_.stop(task)
        return False, f"未知操作: {action}"

    def _ready(self, cli, label, timeout=1.0):
        # 在 HTTP 线程里用非阻塞的 service_is_ready() 轮询, 避免从非执行器线程
        # 调用 wait_for_service 带来的潜在线程问题(rclpy 在主线程 spin)。
        deadline = time.time() + timeout
        while time.time() < deadline:
            if cli.service_is_ready():
                return True
            time.sleep(0.05)
        return cli.service_is_ready()

    def _call_system(self, name):
        cli = self._sys_clients.get(name)
        if cli is None:
            return False, f"未知系统命令: {name}"
        if not self._ready(cli, name):
            return False, f"服务未就绪: {self.system_ns_}/{name} (驱动是否已启动?)"
        cli.call_async(Empty.Request())
        return True, f"已发送: {self.system_ns_}/{name}"

    def _call_gripper(self, cli, payload, label):
        if not self._ready(cli, label):
            return False, f"{label}服务未就绪 (io_service 是否已启动?)"
        req = SetGripper.Request()
        req.val = float(payload.get("val", 0.0))
        cli.call_async(req)
        return True, f"已发送: {label} = {req.val}"

    def _call_set_do(self, payload):
        if not self._ready(self._cli_set_do, "set_robot_do"):
            return False, "set_robot_do 服务未就绪"
        req = SetDO.Request()
        req.pin = int(payload.get("pin", 0))
        req.value = bool(payload.get("value", False))
        self._cli_set_do.call_async(req)
        return True, f"已发送: DO[{req.pin}] = {req.value}"

    def _call_set_ao(self, payload):
        if not self._ready(self._cli_set_ao, "set_robot_ao"):
            return False, "set_robot_ao 服务未就绪"
        req = SetAO.Request()
        req.pin = int(payload.get("pin", 0))
        req.value = float(payload.get("value", 0.0))
        self._cli_set_ao.call_async(req)
        return True, f"已发送: AO[{req.pin}] = {req.value}"

    def _send_vlm(self, payload):
        text = str(payload.get("text", "")).strip()
        if not text:
            return False, "指令为空"
        msg = String()
        msg.data = text
        self._vlm_pub.publish(msg)
        return True, f"已发送 VLM 指令: {text}"

    def _send_vlm_confirm(self, payload):
        ok = bool(payload.get("confirm", False))
        msg = Bool()
        msg.data = ok
        self._vlm_confirm_pub.publish(msg)
        return True, ("已确认抓取" if ok else "已取消抓取")

    def _call_move_joint(self, payload):
        if not self._ready(self._cli_move_joint, "move_joint"):
            return False, "move_joint 服务未就绪 (motion 是否已启动?)"
        joints = payload.get("joint_pose", [])
        if not isinstance(joints, list) or len(joints) == 0:
            return False, "joint_pose 不能为空"
        req = MoveJoint.Request()
        req.is_joint_pose = True
        req.joint_pose = [float(x) for x in joints]
        req.common.acc = float(payload.get("acc", 1.0))
        req.common.vel = float(payload.get("vel", 1.0))
        req.common.time = float(payload.get("time", 0.0))
        req.common.radius = float(payload.get("radius", 0.0))
        self._cli_move_joint.call_async(req)
        return True, f"已发送: move_joint -> {req.joint_pose}"

    # ------------------------------------------------------------------
    # HTTP 服务
    # ------------------------------------------------------------------
    def _start_http_server(self):
        node = self

        # 把系统命令列表与危险命令集合注入网页(供前端渲染按钮)
        html = INDEX_HTML.replace(
            "__SYS_COMMANDS__", json.dumps(SYSTEM_COMMANDS, ensure_ascii=False))
        html = html.replace("__DANGER__", json.dumps(sorted(DANGEROUS_COMMANDS)))
        self._index_html = html.encode("utf-8")

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):  # 静默 http.server 默认日志
                pass

            def _send_json(self, obj, code=200):
                body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _send_text(self, text, code=200):
                body = text.encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                parsed = urlparse(self.path)
                path = parsed.path
                if path in ("/", "/index.html"):
                    body = node._index_html
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif path == "/api/status":
                    self._send_json(node.build_status())
                elif path == "/api/tasks":
                    self._send_json(node.tasks_.status())
                elif path == "/api/task_log":
                    qs = parse_qs(parsed.query)
                    tid = (qs.get("id") or [""])[0]
                    if tid not in TASKS_BY_ID:
                        self._send_text("(未知任务)", 404)
                    else:
                        self._send_text(node.tasks_.log(tid))
                else:
                    self._send_json({"error": "not found"}, 404)

            def do_POST(self):
                path = urlparse(self.path).path
                if path not in ("/api/command", "/api/task"):
                    self._send_json({"error": "not found"}, 404)
                    return
                try:
                    length = int(self.headers.get("Content-Length", 0))
                    payload = json.loads(self.rfile.read(length) or b"{}")
                except Exception as e:  # noqa: BLE001
                    self._send_json({"ok": False, "message": f"请求解析失败: {e}"}, 400)
                    return
                if path == "/api/command":
                    ok, msg = node.dispatch(payload)
                else:  # /api/task : 启动/停止功能任务
                    ok, msg = node.dispatch_task(payload)
                self._send_json({"ok": ok, "message": msg})

        self._httpd = ThreadingHTTPServer((self.http_host_, self.http_port_), Handler)
        self._http_thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._http_thread.start()

    def stop_http_server(self):
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()


# 内嵌的单页 Dashboard 网页(状态轮询 + 命令按钮)
INDEX_HTML = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>乐白机械臂 Dashboard</title>
<style>
  body { font-family: -apple-system, "Segoe UI", Roboto, "PingFang SC", sans-serif;
         margin: 0; background:#0f1419; color:#e6e6e6; }
  header { background:#1b2430; padding:12px 20px; font-size:20px; font-weight:600;
           border-bottom:1px solid #2a3744; display:flex; align-items:center; gap:12px;}
  #conn { font-size:13px; font-weight:400; }
  .dot { display:inline-block; width:10px; height:10px; border-radius:50%; margin-right:4px;}
  .ok { background:#3fb950; } .bad { background:#f85149; }
  .wrap { display:grid; grid-template-columns: 1fr 1fr; gap:16px; padding:16px; }
  .card { background:#161b22; border:1px solid #2a3744; border-radius:10px; padding:14px 16px; }
  .card h2 { margin:0 0 10px; font-size:15px; color:#9db4cc; border-bottom:1px solid #2a3744; padding-bottom:6px;}
  table { width:100%; border-collapse:collapse; font-size:13px; }
  td { padding:3px 6px; } td.k { color:#8b9bb0; width:45%; }
  .badge { padding:2px 8px; border-radius:10px; font-size:12px; }
  .b-on { background:#1f6f3f; } .b-off { background:#5a3030; } .b-unk { background:#444; }
  button { background:#21304a; color:#e6e6e6; border:1px solid #34507a; border-radius:6px;
           padding:7px 12px; margin:3px; cursor:pointer; font-size:13px; }
  button:hover { background:#2d4366; }
  button.danger { background:#5a2230; border-color:#8a3344; }
  button.danger:hover { background:#7a2e40; }
  input { background:#0f1419; color:#e6e6e6; border:1px solid #34507a; border-radius:5px;
          padding:5px; width:70px; }
  #toast { position:fixed; bottom:18px; right:18px; background:#21304a; border:1px solid #34507a;
           padding:10px 16px; border-radius:8px; opacity:0; transition:opacity .3s; max-width:360px; }
  .row { display:flex; align-items:center; flex-wrap:wrap; gap:6px; margin:6px 0; }
  small { color:#8b9bb0; }
  nav { display:flex; gap:6px; margin-left:18px; }
  .navbtn { background:transparent; border:1px solid transparent; border-bottom:2px solid transparent;
            color:#9db4cc; border-radius:4px 4px 0 0; }
  .navbtn.active { color:#fff; border-bottom-color:#4f8cff; background:#21304a; }
  .taskrow { border-bottom:1px solid #20303f; padding:8px 0; }
  .taskrow b { font-size:14px; }
  .taskbtns { float:right; }
  .cmd { color:#6b7d92; font-family:monospace; }
  .log { background:#0a0e12; border:1px solid #2a3744; border-radius:6px; padding:8px;
         max-height:240px; overflow:auto; font-family:monospace; font-size:12px;
         white-space:pre-wrap; margin-top:6px; color:#b8c4d0; }
  button:disabled { opacity:0.4; cursor:not-allowed; }
</style>
</head>
<body>
<header>
  <span style="font-size:18px;">乐白机械臂 Dashboard</span>
  <nav>
    <button id="nav-monitor" class="navbtn active" onclick="showView('monitor')">监控与控制</button>
    <button id="nav-launch" class="navbtn" onclick="showView('launch')">功能启动</button>
    <button id="nav-sim" class="navbtn" onclick="showView('sim')">仿真</button>
  </nav>
  <span id="conn" style="margin-left:auto;"></span>
</header>

<div id="view-monitor" class="view">
<div class="wrap">
  <div class="card" style="grid-column:1 / span 2; text-align:center;">
    <h2>当前目标距离 (/grab_target/distance)</h2>
    <div id="distance" style="font-size:34px; font-weight:700; color:#8b9bb0;">—</div>
    <small>相机到目标的深度(米), 由当前运行的视觉算法(HSV/YOLO/KCF)发布; 无目标时显示 —</small>
  </div>

  <div class="card" style="grid-column:1 / span 2;">
    <h2>VLM 自然语言抓取</h2>
    <div class="row">
      <input id="vlmtext" type="text" style="width:50%;"
             placeholder="例如: 把红色的瓶子递给我 / 我渴了 / 拿起最大的物体"
             onkeydown="if(event.key==='Enter')sendVlm()"/>
      <button onclick="sendVlm()">发送指令</button>
      <button onclick="confirmVlm(true)">确认抓取</button>
      <button class="danger" onclick="confirmVlm(false)">取消</button>
    </div>
    <div id="vlmresult" style="margin-top:6px; min-height:20px; color:#cfe3ff;">
      <small>等待指令... (需先在"功能启动"页启动【VLM 语言抓取】)</small>
    </div>
    <small>安全机制: 默认 require_confirm, 节点理解到目标后会等你点【确认抓取】才动机械臂。</small>
  </div>

  <div class="card">
    <h2>机器人状态</h2>
    <table id="robot"><tr><td class="k">等待 /robot_status ...</td></tr></table>
  </div>

  <div class="card">
    <h2>夹爪状态</h2>
    <table id="gripper"><tr><td class="k">等待 /gripper_status ...</td></tr></table>
  </div>

  <div class="card" style="grid-column:1 / span 2;">
    <h2>IO 状态 (/io_status)</h2>
    <table id="io"><tr><td class="k">等待 /io_status ...</td></tr></table>
  </div>

  <div class="card" style="grid-column:1 / span 2;">
    <h2>关节状态 (/joint_states)</h2>
    <table id="joints"><tr><td class="k">等待数据 ...</td></tr></table>
  </div>

  <div class="card" style="grid-column:1 / span 2;">
    <h2>系统控制 (system_service)</h2>
    <div id="sysbtns"></div>
  </div>

  <div class="card">
    <h2>夹爪控制 (io_service)</h2>
    <div class="row">
      位置 <input id="gpos" type="number" min="0" max="100" value="100"/>
      <button onclick="gripper('gripper_position','gpos')">设置位置 (0闭合~100张开)</button>
    </div>
    <div class="row">
      力度 <input id="gforce" type="number" value="50"/>
      <button onclick="gripper('gripper_force','gforce')">设置力度</button>
    </div>
  </div>

  <div class="card">
    <h2>数字输出 DO (io_service)</h2>
    <div class="row">
      引脚 <input id="dopin" type="number" min="0" value="0"/>
      <button onclick="setdo(true)">置 ON</button>
      <button onclick="setdo(false)">置 OFF</button>
    </div>
  </div>

  <div class="card" style="grid-column:1 / span 2;">
    <h2>关节运动 (motion_service/move_joint) <small>— 会真实移动机械臂, 请谨慎</small></h2>
    <div class="row" id="jointinputs"></div>
    <div class="row">
      acc <input id="macc" type="number" value="1.0"/>
      vel <input id="mvel" type="number" value="1.0"/>
      <button class="danger" onclick="movej()">执行关节运动</button>
      <button onclick="fillCurrent()">填入当前关节角</button>
    </div>
    <small>单位: 弧度(rad)。"填入当前关节角"会把上面的关节状态填进输入框。</small>
  </div>
</div>
</div><!-- /view-monitor -->

<div id="view-launch" class="view" style="display:none">
  <div class="wrap" id="tasklist"><div class="card">加载中 ...</div></div>
  <div style="padding:0 16px 16px;"><small>
    提示: 视觉抓取(YOLO/HSV/ArUco/KCF/VLM)各自已包含相机 + 机械臂 + MoveIt + 抓取服务, 选一个启动即可,
    不要和"机器人驱动""MoveIt"重复启动以免节点冲突。点"日志"可查看该任务输出。
  </small></div>
</div>

<div id="view-sim" class="view" style="display:none">
  <div class="wrap" id="tasklist-sim"><div class="card">加载中 ...</div></div>
  <div style="padding:0 16px 16px;">
    <div class="card">
      <h2>Gazebo 仿真说明</h2>
      <small>
        1. 启动上面的【Gazebo 仿真场景 + 机械臂】, 会打开 Gazebo GUI, 自动加载场景(地面/桌子/可乐罐/木块/啤酒)和机械臂模型。<br/>
        2. 用 Gazebo GUI 左侧 <b>Insert</b> 面板可继续拖入更多标准物体(Fuel 模型库)或你自己的模型。<br/>
        3. 仿真任务与真机任务资源不冲突, 但请勿同时连真机, 以免混淆。<br/>
        4. 仿真相机发布 <code>/camera_arm/color/image_raw</code> 等话题, 可在仿真里跑视觉/VLM(相机安装位与内参需按手眼标定微调)。<br/>
        5. 依赖: gazebo_ros / gazebo_ros2_control / controller_manager(详见 lebai_gazebo/GAZEBO_GUIDE.md)。
      </small>
    </div>
  </div>
</div>

<div id="toast"></div>

<script>
let lastJoints = [];

// 渲染系统按钮
const SYS = __SYS_COMMANDS__;
const DANGER = __DANGER__;
const sysDiv = document.getElementById('sysbtns');
SYS.forEach(([name, label]) => {
  const b = document.createElement('button');
  b.textContent = label;
  if (DANGER.includes(name)) b.className = 'danger';
  b.onclick = () => sysCmd(name, label, DANGER.includes(name));
  sysDiv.appendChild(b);
});

// 6 个关节输入框
const ji = document.getElementById('jointinputs');
for (let i=0;i<6;i++){
  const inp = document.createElement('input');
  inp.id = 'j'+i; inp.type='number'; inp.step='0.01'; inp.value='0.0';
  ji.appendChild(inp);
}

function toast(msg, ok=true){
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.style.borderColor = ok ? '#34507a' : '#8a3344';
  t.style.opacity = 1;
  setTimeout(()=> t.style.opacity = 0, 2500);
}

async function post(payload){
  try {
    const r = await fetch('/api/command', {method:'POST', headers:{'Content-Type':'application/json'},
                          body: JSON.stringify(payload)});
    const j = await r.json();
    toast(j.message, j.ok);
  } catch(e){ toast('请求失败: '+e, false); }
}

function sysCmd(name, label, danger){
  if (danger && !confirm('确认执行【'+label+'】?')) return;
  post({type:'system', name:name});
}
function gripper(type, inputId){
  post({type:type, val: parseFloat(document.getElementById(inputId).value)});
}
function setdo(v){
  post({type:'set_do', pin: parseInt(document.getElementById('dopin').value), value: v});
}
function sendVlm(){
  const t = document.getElementById('vlmtext').value.trim();
  if (!t){ toast('请输入指令', false); return; }
  post({type:'vlm', text:t});
}
function confirmVlm(ok){
  post({type:'vlm_confirm', confirm: ok});
}
function movej(){
  if (!confirm('确认执行关节运动? 机械臂会真实移动!')) return;
  const jp = [];
  for (let i=0;i<6;i++) jp.push(parseFloat(document.getElementById('j'+i).value));
  post({type:'move_joint', joint_pose: jp,
        acc: parseFloat(document.getElementById('macc').value),
        vel: parseFloat(document.getElementById('mvel').value)});
}
function fillCurrent(){
  for (let i=0;i<6 && i<lastJoints.length;i++) document.getElementById('j'+i).value = lastJoints[i];
}

function badge(state){
  const cls = state==='ON' ? 'b-on' : (state==='OFF' ? 'b-off' : 'b-unk');
  return '<span class="badge '+cls+'">'+state+'</span>';
}

async function refresh(){
  let s;
  try { s = await (await fetch('/api/status')).json(); }
  catch(e){ document.getElementById('conn').innerHTML =
    '<span class="dot bad"></span>无法连接 dashboard'; return; }

  // 顶部连接指示
  const on = s.online || {};
  const allok = on.robot_status;
  document.getElementById('conn').innerHTML =
    '<span class="dot '+(allok?'ok':'bad')+'"></span>' +
    (allok ? '驱动在线' : '等待 robot_state 节点...');

  // VLM 结果(用 textContent 防注入)
  const vr = document.getElementById('vlmresult');
  if (vr && s.vlm_result){
    vr.textContent = s.vlm_result.text + '  (' + s.vlm_result.age + 's前)';
  }

  // 目标距离(大字显示, 无目标/数据过期则显示 —)
  const distEl = document.getElementById('distance');
  if (distEl){
    if (on.target_distance && s.target_distance != null){
      distEl.textContent = s.target_distance.toFixed(3) + ' m';
      distEl.style.color = '#3fb950';
    } else {
      distEl.textContent = '—';
      distEl.style.color = '#8b9bb0';
    }
  }

  // 机器人状态
  if (s.robot){
    const r = s.robot;
    document.getElementById('robot').innerHTML =
      row('急停 e_stopped', badge(r.e_stopped)) +
      row('上电 drives_powered', badge(r.drives_powered)) +
      row('可运动 motion_possible', badge(r.motion_possible)) +
      row('运动中 in_motion', badge(r.in_motion)) +
      row('错误 in_error', badge(r.in_error)) +
      row('错误码 error_code', r.error_code) +
      row('模式 mode', r.mode);
  }

  // 夹爪
  if (s.gripper){
    document.getElementById('gripper').innerHTML =
      row('位置 position', s.gripper.position) +
      row('力度 force', s.gripper.force);
  }

  // IO
  if (s.io){
    const bits = (arr) => (arr && arr.length) ?
      arr.map((b,i)=> i+':'+badge(b?'ON':'OFF')).join(' ') : '<small>无</small>';
    document.getElementById('io').innerHTML =
      row('机器人 DI', bits(s.io.robot_din)) +
      row('机器人 DO', bits(s.io.robot_dout)) +
      row('机器人 AI', (s.io.robot_ain||[]).join(', ') || '<small>无</small>') +
      row('法兰 DI', bits(s.io.flange_din)) +
      row('扩展 DI', bits(s.io.extend_din));
  }

  // 关节
  if (s.joints){
    lastJoints = s.joints.positions_rad;
    let html = '<tr><td class="k">关节</td><td>角度(rad)</td><td>角度(°)</td><td>速度</td></tr>';
    for (let i=0;i<s.joints.names.length;i++){
      html += '<tr><td class="k">'+s.joints.names[i]+'</td><td>'+
              (s.joints.positions_rad[i]??'-')+'</td><td>'+
              (s.joints.positions_deg[i]??'-')+'</td><td>'+
              (s.joints.velocities[i]??'-')+'</td></tr>';
    }
    document.getElementById('joints').innerHTML = html;
  }
}
function row(k, v){ return '<tr><td class="k">'+k+'</td><td>'+v+'</td></tr>'; }

// ---------------- 导航 / 视图切换 ----------------
function showView(name){
  ['monitor','launch','sim'].forEach(v => {
    document.getElementById('view-'+v).style.display = (name===v) ? 'block' : 'none';
    document.getElementById('nav-'+v).classList.toggle('active', name===v);
  });
  if (name==='launch' || name==='sim') refreshTasks();
}

// ---------------- 功能启动页 ----------------
let tasksRendered = false;
let TASKS = [];

// 返回与给定任务"资源冲突"且正在运行的任务标签列表
function conflictsFor(task){
  const res = new Set(task.resources || []);
  return TASKS.filter(o => o.id !== task.id && o.running &&
            (o.resources || []).some(r => res.has(r)))
              .map(o => o.label);
}

async function refreshTasks(){
  let tasks;
  try { tasks = await (await fetch('/api/tasks')).json(); }
  catch(e){ return; }
  TASKS = tasks;
  if (!tasksRendered){ buildTaskList(tasks); tasksRendered = true; }
  tasks.forEach(t => {
    const dot = document.getElementById('dot-'+t.id);
    const st = document.getElementById('st-'+t.id);
    if (dot) dot.className = 'dot ' + (t.running ? 'ok' : 'bad');
    if (st) st.innerHTML = t.running
      ? '<small style="color:#3fb950">运行中 pid='+t.pid+' ('+t.uptime+'s)</small>'
      : '<small>已停止</small>';

    // 冲突检测: 禁用会与运行中任务冲突的"启动"按钮并提示
    const startBtn = document.querySelector(
      'button[data-id="'+t.id+'"][data-act="start"]');
    const hint = document.getElementById('hint-'+t.id);
    const conf = conflictsFor(t);
    if (startBtn){
      if (t.running){
        startBtn.disabled = true; startBtn.title = '已在运行';
      } else if (conf.length){
        startBtn.disabled = true; startBtn.title = '与运行中的任务冲突: ' + conf.join(', ');
      } else {
        startBtn.disabled = false; startBtn.title = '';
      }
    }
    if (hint){
      hint.innerHTML = (!t.running && conf.length)
        ? '<small style="color:#e3b341">⚠ 与运行中的【'+conf.join('、')+'】冲突, 已禁用启动</small>'
        : '';
    }
  });
}
function taskRowHtml(t){
  return '<div class="taskrow">'+
    '<span class="taskbtns">'+
      '<button data-id="'+t.id+'" data-act="start">启动</button>'+
      '<button class="danger" data-id="'+t.id+'" data-act="stop">停止</button>'+
      '<button data-id="'+t.id+'" data-act="log">日志</button>'+
    '</span>'+
    '<span class="dot bad" id="dot-'+t.id+'"></span>'+
    '<b>'+t.label+'</b> &nbsp;<span id="st-'+t.id+'"></span> '+
    '<span id="hint-'+t.id+'"></span>'+
    '<div class="cmd">'+t.cmd+'</div>'+
    '<pre id="log-'+t.id+'" class="log" style="display:none"></pre>'+
  '</div>';
}
function buildTaskList(tasks){
  const groups = {};
  tasks.forEach(t => { (groups[t.group] = groups[t.group] || []).push(t); });
  let mainHtml = '', simHtml = '';
  for (const g in groups){
    let html = '<div class="card"><h2>'+g+'</h2>';
    groups[g].forEach(t => { html += taskRowHtml(t); });
    html += '</div>';
    if (g === '仿真') simHtml += html; else mainHtml += html;
  }
  document.getElementById('tasklist').innerHTML = mainHtml || '<div class="card">无任务</div>';
  document.getElementById('tasklist-sim').innerHTML = simHtml || '<div class="card">无仿真任务</div>';
}
// 事件委托(两个任务容器共用)
function taskClickHandler(e){
  const b = e.target.closest('button'); if (!b) return;
  const id = b.dataset.id, act = b.dataset.act;
  if (!id) return;
  if (act === 'log') toggleLog(id);
  else taskCmd(id, act);
}
document.getElementById('tasklist').addEventListener('click', taskClickHandler);
document.getElementById('tasklist-sim').addEventListener('click', taskClickHandler);
async function taskCmd(id, action){
  // 启动前再做一次冲突拦截(双保险, 防止禁用态被绕过)
  if (action === 'start'){
    const t = TASKS.find(x => x.id === id);
    const conf = t ? conflictsFor(t) : [];
    if (conf.length){
      toast('无法启动: 与运行中的【'+conf.join('、')+'】冲突, 请先停止它们', false);
      return;
    }
  }
  if (action === 'stop' && !confirm('确认停止该任务?')) return;
  try {
    const r = await fetch('/api/task', {method:'POST', headers:{'Content-Type':'application/json'},
                          body: JSON.stringify({id:id, action:action})});
    const j = await r.json(); toast(j.message, j.ok);
  } catch(e){ toast('请求失败: '+e, false); }
  setTimeout(refreshTasks, 400);
}
async function toggleLog(id){
  const pre = document.getElementById('log-'+id);
  if (pre.style.display === 'none'){
    try { pre.textContent = await (await fetch('/api/task_log?id='+encodeURIComponent(id))).text(); }
    catch(e){ pre.textContent = '读取日志失败: '+e; }
    pre.style.display = 'block';
    pre.scrollTop = pre.scrollHeight;
  } else { pre.style.display = 'none'; }
}

setInterval(refresh, 500);
setInterval(() => { if (document.getElementById('view-launch').style.display !== 'none') refreshTasks(); }, 1500);
refresh();
</script>
</body>
</html>
"""


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.tasks_.shutdown_all()
        node.stop_http_server()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
