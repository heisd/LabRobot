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
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
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
            return False, f"未知命令类型: {cmd}"
        except Exception as e:  # noqa: BLE001 - 网页错误需返回给前端
            self.get_logger().error(f"命令执行异常: {e}")
            return False, f"异常: {e}"

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

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    body = node._index_html
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path == "/api/status":
                    self._send_json(node.build_status())
                else:
                    self._send_json({"error": "not found"}, 404)

            def do_POST(self):
                if self.path != "/api/command":
                    self._send_json({"error": "not found"}, 404)
                    return
                try:
                    length = int(self.headers.get("Content-Length", 0))
                    payload = json.loads(self.rfile.read(length) or b"{}")
                except Exception as e:  # noqa: BLE001
                    self._send_json({"ok": False, "message": f"请求解析失败: {e}"}, 400)
                    return
                ok, msg = node.dispatch(payload)
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
</style>
</head>
<body>
<header>
  乐白机械臂 Dashboard
  <span id="conn"></span>
</header>

<div class="wrap">
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

setInterval(refresh, 500);
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
        node.stop_http_server()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
