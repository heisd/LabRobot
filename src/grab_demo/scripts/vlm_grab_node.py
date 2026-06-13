#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VLM (视觉语言模型) 抓取节点.

让机械臂"看见画面 + 理解一句自然语言指令 -> 选出该抓的物体 -> 抓取"。

流程:
  1. 持续接收相机彩色/深度/内参(与 HSV/YOLO/KCF 同样的话题);
  2. 收到一条指令(/vlm/instruction, std_msgs/String, 由 Dashboard 发送)后,
     把"当前彩色帧 + 指令"发给 VLM, 让它返回该抓物体的像素框(JSON);
  3. 用框中心 + 深度做针孔反投影, 持续广播 camera_frame->target_frame,
     并发布距离到 /grab_target/distance(与其它视觉算法统一);
  4. 把"理解结果"发到 /vlm/result(String) 供 Dashboard 显示;
  5. 若 auto_grab=true, 自动调用 /obj_grab_service 让机械臂抓取。

VLM 接口支持三种(provider 参数):
  - "ollama"   : 本地 Ollama 原生接口 /api/generate;
  - "openai"   : OpenAI 兼容 /chat/completions;
  - "anthropic": Anthropic Claude /v1/messages。
仅使用 Python 标准库 urllib, 无额外 pip 依赖。
"""

import base64
import json
import math
import os
import threading
import time
import urllib.request
import urllib.error

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
from tf2_ros import TransformBroadcaster
import message_filters

try:
    from cv_bridge import CvBridge
except Exception:  # noqa: BLE001
    CvBridge = None

# 抓取服务(可选自动抓取); 在 grab_demo 里由 rosidl 生成
try:
    from grab_demo.srv import GrabObject
except Exception:  # noqa: BLE001
    GrabObject = None


PROMPT_TEMPLATE = (
    "你是机械臂的视觉抓取助手。下面这张图是机械臂相机看到的画面, 尺寸为 {w}x{h} 像素"
    "(左上角为原点, x 向右, y 向下)。\n"
    "用户指令: \"{instruction}\"\n"
    "请理解用户的真实意图(指令可能是间接的, 例如\"我渴了\"通常对应饮料/瓶子/杯子), "
    "在画面中选出最应该抓取的【单个】物体。\n"
    "只输出一个 JSON 对象, 不要输出任何额外文字, 格式如下:\n"
    '{{"found": true, "label": "物体的简短名称", "reason": "选它的简短理由", '
    '"bbox": [x, y, w, h]}}\n'
    "其中 bbox 是该物体外接矩形的像素坐标(整数, x,y 为左上角, w,h 为宽高)。"
    "如果画面里找不到合适的物体, 返回 {{\"found\": false, \"reason\": \"原因\"}}。"
)


class VlmGrabNode(Node):
    def __init__(self):
        super().__init__("vlm_grab_node")

        # ---- 参数 ----
        self.rgb_topic = self.declare_parameter("rgb_topic", "/camera_arm/color/image_raw").value
        self.depth_topic = self.declare_parameter("depth_topic", "/camera_arm/depth/image_raw").value
        self.info_topic = self.declare_parameter("camera_info_topic", "/gemini_info").value
        self.camera_frame = self.declare_parameter("camera_frame", "camera_arm_depth_optical_frame").value
        self.target_frame = self.declare_parameter("target_frame", "target_frame").value
        self.z_offset = float(self.declare_parameter("z_offset", 0.07).value)

        # VLM 接口
        self.provider = self.declare_parameter("provider", "ollama").value  # ollama | openai | anthropic
        self.api_base = self.declare_parameter("api_base", "http://localhost:11434").value
        self.model = self.declare_parameter("model", "qwen3-vl:2b").value
        self.api_key_env = self.declare_parameter("api_key_env", "").value  # 留空则按 provider 取默认
        self.timeout = float(self.declare_parameter("request_timeout", 60.0).value)

        # 抓取
        self.auto_grab = bool(self.declare_parameter("auto_grab", True).value)
        self.grab_service = self.declare_parameter("grab_service", "/obj_grab_service").value
        self.publish_debug_image = bool(self.declare_parameter("publish_debug_image", True).value)

        self.instruction_topic = self.declare_parameter("instruction_topic", "/vlm/instruction").value
        self.result_topic = self.declare_parameter("result_topic", "/vlm/result").value
        self.distance_topic = self.declare_parameter("distance_topic", "/grab_target/distance").value
        self.confirm_topic = self.declare_parameter("confirm_topic", "/vlm/confirm").value

        # ---- 安全加固参数 ----
        self.max_instruction_len = int(self.declare_parameter("max_instruction_len", 200).value)
        self.min_dist = float(self.declare_parameter("min_dist", 0.1).value)   # 允许的最近距离(m)
        self.max_dist = float(self.declare_parameter("max_dist", 1.5).value)   # 允许的最远距离(m)
        self.require_confirm = bool(self.declare_parameter("require_confirm", True).value)
        self.force_json = bool(self.declare_parameter("force_json", True).value)  # openai 强制 JSON 输出

        if CvBridge is None:
            self.get_logger().fatal("缺少 cv_bridge, 无法运行 VLM 节点")
            raise RuntimeError("cv_bridge missing")
        self.bridge = CvBridge()

        # ---- 状态 ----
        self._lock = threading.Lock()
        self._rgb = None        # 最新彩色帧(bgr)
        self._depth = None      # 最新深度帧(uint16, mm)
        self._fx = self._fy = self._cx = self._cy = 0.0
        self._intrinsics_ready = False
        self._target = None     # (x, y, dis) 相机系下目标点(z 偏移广播时再加); None 表示无目标
        self._busy = False
        self._pending_grab = False  # 已理解到目标, 等待 Dashboard 确认抓取
        # ---- 健康监控 / 仲裁状态 ----
        self._last_img_time = 0.0   # 最近一次收到相机帧的时刻(秒)
        self._stale_warned = False
        self._manual_active = False  # 仲裁: 手动接管中则不触发自动抓取

        # ---- 订阅 ----
        self.create_subscription(CameraInfo, self.info_topic, self._info_cb, 1)
        rgb_sub = message_filters.Subscriber(self, Image, self.rgb_topic)
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        self._sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 0.3)
        self._sync.registerCallback(self._image_cb)
        self.create_subscription(String, self.instruction_topic, self._instruction_cb, 10)
        # 抓取确认(安全加固): True=确认抓取, False=取消
        self.create_subscription(Bool, self.confirm_topic, self._confirm_cb, 10)
        # 抓取仲裁: 手动接管期间(/arm_arbiter/manual_active=True)不触发自动抓取(latched QoS)
        _latched = QoSProfile(depth=1)
        _latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Bool, "/arm_arbiter/manual_active",
                                 self._manual_cb, _latched)

        # ---- 发布 ----
        self.result_pub = self.create_publisher(String, self.result_topic, 10)
        self.dist_pub = self.create_publisher(Float32, self.distance_topic, 10)
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(Image, "~/vlm_image", 1)
        else:
            self.debug_pub = None
        self.tf_pub = TransformBroadcaster(self)

        # 抓取服务客户端(可选)
        self.grab_client = None
        if self.auto_grab and GrabObject is not None:
            self.grab_client = self.create_client(GrabObject, self.grab_service)

        # 以 15Hz 持续广播 target_frame + 距离(确保抓取服务能查到 TF)
        self.create_timer(1.0 / 15.0, self._publish_target)

        # 相机帧流健康监控: 长时间收不到帧就告警(便于排查相机掉线/话题不匹配)
        self.det_timeout = float(self.declare_parameter("det_timeout", 3.0).value)
        self.create_timer(1.0, self._watchdog)

        # 运行时动态调参(ros2 param set 即时生效), 重点是 z_offset(抓取深度)与安全/抓取开关
        self.add_on_set_parameters_callback(self._on_set_params)

        self.get_logger().info(
            "vlm_grab_node 已启动 (provider=%s, model=%s, auto_grab=%s). "
            "向 %s 发送指令开始。" % (self.provider, self.model, self.auto_grab, self.instruction_topic))
        # 启动即输出 VLM 大模型接入情况(未接入会 ERROR 提示)
        self._log_vlm_connectivity()

    # 仲裁: 缓存手动接管状态
    def _manual_cb(self, msg: Bool):
        self._manual_active = bool(msg.data)

    # 相机帧流看门狗
    def _watchdog(self):
        if self._last_img_time == 0.0:
            self.get_logger().warn("尚未收到相机图像 (%s): 相机是否启动? 话题是否匹配?"
                                   % self.rgb_topic, throttle_duration_sec=5.0)
            return
        gap = time.time() - self._last_img_time
        if gap > self.det_timeout and not self._stale_warned:
            self.get_logger().error("已 %.1fs 未收到相机帧(>%.1fs): 相机掉线 / 话题不匹配?"
                                    % (gap, self.det_timeout))
            self._stale_warned = True
        elif gap <= self.det_timeout:
            self._stale_warned = False

    # 运行时动态参数回调
    def _on_set_params(self, params):
        for p in params:
            try:
                if p.name == "z_offset":
                    self.z_offset = float(p.value)
                    self.get_logger().info("z_offset -> %.3f m" % self.z_offset)
                elif p.name == "min_dist":
                    self.min_dist = float(p.value)
                elif p.name == "max_dist":
                    self.max_dist = float(p.value)
                elif p.name == "auto_grab":
                    self.auto_grab = bool(p.value)
                    self.get_logger().info("auto_grab -> %s" % self.auto_grab)
                elif p.name == "require_confirm":
                    self.require_confirm = bool(p.value)
                    self.get_logger().info("require_confirm -> %s" % self.require_confirm)
                elif p.name == "max_instruction_len":
                    self.max_instruction_len = int(p.value)
            except (TypeError, ValueError) as e:
                return SetParametersResult(successful=False, reason=str(e))
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------
    def _info_cb(self, msg: CameraInfo):
        if self._intrinsics_ready:
            return
        if not any(msg.k):
            return
        self._fx, self._cx, self._fy, self._cy = msg.k[0], msg.k[2], msg.k[4], msg.k[5]
        self._intrinsics_ready = True
        self.get_logger().info("已获取相机内参 fx=%.1f fy=%.1f cx=%.1f cy=%.1f"
                               % (self._fx, self._fy, self._cx, self._cy))

    def _image_cb(self, rgb_msg: Image, depth_msg: Image):
        self._last_img_time = time.time()   # 健康监控: 记录帧到达
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error("cv_bridge 转换失败: %s" % e, throttle_duration_sec=2.0)
            return
        with self._lock:
            self._rgb = rgb
            self._depth = depth

    def _instruction_cb(self, msg: String):
        text = (msg.data or "").strip()
        if not text:
            return
        # 安全加固: 限制指令长度
        if len(text) > self.max_instruction_len:
            text = text[:self.max_instruction_len]
            self._publish_result("(指令过长, 已截断到 %d 字)" % self.max_instruction_len)
        if self._busy:
            self._publish_result("⏳ 正在处理上一条指令, 请稍候...")
            return
        # 在线程里调用 VLM(网络阻塞), 不卡住 ROS 执行器
        threading.Thread(target=self._process, args=(text,), daemon=True).start()

    def _confirm_cb(self, msg: Bool):
        # 安全加固: 抓取需 Dashboard 二次确认
        if msg.data:
            if self._pending_grab and self._target is not None:
                self._pending_grab = False
                self._publish_result("👍 已确认, 开始抓取")
                self._call_grab()
            else:
                self._publish_result("(当前没有待确认的抓取)")
        else:
            self._pending_grab = False
            self._publish_result("已取消本次抓取")

    # ------------------------------------------------------------------
    # 主流程
    # ------------------------------------------------------------------
    def _process(self, instruction: str):
        self._busy = True
        try:
            # 未接入 VLM 大模型时, 直接给出明确日志与提示, 不做无谓的网络请求
            miss = self._vlm_missing_reason()
            if miss:
                self.get_logger().error("%s, 拒绝处理指令: %s" % (miss, instruction))
                self._publish_result("❌ %s; 请配置 API Key 后重启, 或把 api_base 指向本地模型" % miss)
                return
            if not self._intrinsics_ready:
                self._publish_result("❌ 还没收到相机内参 (%s)" % self.info_topic)
                return
            with self._lock:
                rgb = None if self._rgb is None else self._rgb.copy()
                depth = None if self._depth is None else self._depth.copy()
            if rgb is None or depth is None:
                self._publish_result("❌ 还没收到相机图像")
                return

            self.get_logger().info("VLM 处理指令: %s" % instruction)
            self._publish_result("🤔 正在理解: %s" % instruction)

            ans = self._call_vlm(rgb, instruction)
            if ans is None:
                self.get_logger().error("VLM 调用失败(可能未接入大模型/网络不可达/Key 无效): %s" % instruction)
                self._publish_result("❌ VLM 调用失败(检查是否已接入大模型: provider/api_base/api_key/网络)")
                return
            if not ans.get("found", False):
                self._target = None
                self._publish_result("🚫 未找到合适物体: %s" % ans.get("reason", ""))
                return

            img_w, img_h = rgb.shape[1], rgb.shape[0]
            box = self._parse_box(ans, img_w, img_h)
            if box is None:
                self._publish_result("❌ VLM 返回的框无法解析: %s" % json.dumps(ans, ensure_ascii=False))
                return
            x, y, w, h = box
            # 安全加固: 框中心像素裁剪到图像范围内
            px = int(min(max(x + w / 2.0, 0), img_w - 1))
            py = int(min(max(y + h / 2.0, 0), img_h - 1))
            label = ans.get("label", "目标")
            reason = ans.get("reason", "")

            # 安全加固: 计算并校验目标(深度有效 + 距离在允许范围 + 坐标有限)
            ok, dis, why = self._compute_target(px, py, depth)
            if self.publish_debug_image:
                self._publish_debug(rgb, box, label, dis if ok else -1.0)
            if not ok:
                self._target = None
                self._publish_result("⚠ 已识别 [%s] 但无法定位: %s" % (label, why))
                return

            base = "✅ 理解为 [%s] (%s), 距离 %.3fm" % (label, reason, dis)
            self.get_logger().info(base)

            # 安全加固: 自动抓取前二次确认
            if self.auto_grab and self.require_confirm:
                self._pending_grab = True
                self._publish_result(base + "  → 请在 Dashboard 点【确认抓取】执行")
            elif self.auto_grab:
                self._publish_result(base + "  → 自动抓取中")
                self._call_grab()
            else:
                self._publish_result(base)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error("VLM 处理异常: %s" % e)
            self._publish_result("❌ 处理异常: %s" % e)
        finally:
            self._busy = False

    # ------------------------------------------------------------------
    # VLM 调用
    # ------------------------------------------------------------------
    def _call_vlm(self, bgr, instruction):
        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            return None
        b64 = base64.b64encode(buf.tobytes()).decode("ascii")
        prompt = PROMPT_TEMPLATE.format(w=bgr.shape[1], h=bgr.shape[0], instruction=instruction)
        try:
            if self.provider == "anthropic":
                text = self._call_anthropic(prompt, b64)
            elif self.provider == "ollama":
                text = self._call_ollama(prompt, b64)
            else:
                text = self._call_openai(prompt, b64)
        except urllib.error.HTTPError as e:
            self.get_logger().error("VLM HTTP 错误 %s: %s" % (e.code, e.read()[:200]))
            return None
        except Exception as e:  # noqa: BLE001
            self.get_logger().error("VLM 请求失败: %s" % e)
            return None
        return self._extract_json(text)

    def _api_key(self, default_env):
        env = self.api_key_env or default_env
        return os.environ.get(env, "")

    def _default_key_env(self):
        if self.provider == "anthropic": return "ANTHROPIC_API_KEY"
        if self.provider == "ollama": return ""
        return "OPENAI_API_KEY"

    def _vlm_missing_reason(self):
        """未接入 VLM 大模型时返回原因字符串, 已接入(或本地模型)返回 None。"""
        if self.provider == "ollama":
            return None
        env_name = self.api_key_env or self._default_key_env()
        key = os.environ.get(env_name, "")
        is_cloud = ("api.openai.com" in self.api_base) or ("api.anthropic.com" in self.api_base)
        if (not key) and is_cloud:
            return "未接入 VLM 大模型(云端 api_base=%s 但环境变量 %s 为空)" % (self.api_base, env_name)
        return None

    def _log_vlm_connectivity(self):
        """启动时输出 VLM 大模型接入情况, 便于一眼看出"是否接了大模型"。"""
        if self.provider == "ollama":
            self.get_logger().info("VLM 使用本地 Ollama: model=%s base=%s" % (self.model, self.api_base))
            return
        env_name = self.api_key_env or self._default_key_env()
        key = os.environ.get(env_name, "")
        is_cloud = ("api.openai.com" in self.api_base) or ("api.anthropic.com" in self.api_base)
        if key:
            self.get_logger().info(
                "VLM 大模型已配置: provider=%s model=%s api_base=%s (API Key 来自环境变量 %s)"
                % (self.provider, self.model, self.api_base, env_name))
        elif is_cloud:
            self.get_logger().error(
                "未接入 VLM 大模型: 云端 api_base=%s 但环境变量 %s 为空 —— 自然语言抓取不可用。"
                "请先 `export %s=...` 再启动, 或把 api_base 指向本地模型(Ollama/vLLM)。"
                % (self.api_base, env_name, env_name))
        else:
            self.get_logger().warn(
                "VLM 未检测到 API Key(环境变量 %s 为空)。若 api_base=%s 为本地模型(Ollama/vLLM)可忽略, "
                "否则自然语言抓取将不可用。" % (env_name, self.api_base))

    def _post(self, url, headers, body):
        req = urllib.request.Request(url, data=json.dumps(body).encode("utf-8"),
                                     headers=headers, method="POST")
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return json.loads(r.read().decode("utf-8"))

    def _call_ollama(self, prompt, b64):
        url = self.api_base.rstrip("/") + "/api/generate"
        body = {
            "model": self.model,
            "prompt": prompt,
            "images": [b64],
            "stream": False,
            "format": "json",
            "options": {"temperature": 0}
        }
        resp = self._post(url, {}, body)
        return resp.get("response", "")

    def _call_openai(self, prompt, b64):
        url = self.api_base.rstrip("/") + "/chat/completions"
        headers = {"Content-Type": "application/json"}
        key = self._api_key("OPENAI_API_KEY")
        if key:
            headers["Authorization"] = "Bearer " + key
        body = {
            "model": self.model,
            "temperature": 0,
            "max_tokens": 300,
            "messages": [{
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url",
                     "image_url": {"url": "data:image/jpeg;base64," + b64}},
                ],
            }],
        }
        # 安全加固: 让 API 层强制返回合法 JSON(本地服务若不支持可设 force_json:=false)
        if self.force_json:
            body["response_format"] = {"type": "json_object"}
        resp = self._post(url, headers, body)
        return resp["choices"][0]["message"]["content"]

    def _call_anthropic(self, prompt, b64):
        url = self.api_base.rstrip("/") + "/v1/messages"
        headers = {
            "Content-Type": "application/json",
            "x-api-key": self._api_key("ANTHROPIC_API_KEY"),
            "anthropic-version": "2023-06-01",
        }
        body = {
            "model": self.model,
            "max_tokens": 300,
            "messages": [{
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image", "source": {
                        "type": "base64", "media_type": "image/jpeg", "data": b64}},
                ],
            }],
        }
        resp = self._post(url, headers, body)
        return resp["content"][0]["text"]

    @staticmethod
    def _extract_json(text):
        if not text:
            return None
        s, e = text.find("{"), text.rfind("}")
        if s < 0 or e < 0 or e <= s:
            return None
        try:
            return json.loads(text[s:e + 1])
        except Exception:  # noqa: BLE001
            return None

    @staticmethod
    def _clamp_box(x, y, w, h, img_w, img_h):
        # 安全加固: 数值有限性 + 裁剪到图像内
        if not all(math.isfinite(v) for v in (x, y, w, h)):
            return None
        x = min(max(x, 0.0), img_w - 1.0)
        y = min(max(y, 0.0), img_h - 1.0)
        w = min(max(w, 1.0), img_w - x)
        h = min(max(h, 1.0), img_h - y)
        return [x, y, w, h]

    @classmethod
    def _parse_box(cls, ans, img_w, img_h):
        # 支持 bbox=[x,y,w,h]; 兼容归一化(0~1)坐标; 或 point=[x,y]
        try:
            if "bbox" in ans and isinstance(ans["bbox"], (list, tuple)) and len(ans["bbox"]) == 4:
                x, y, w, h = [float(v) for v in ans["bbox"]]
                if max(x, y, w, h) <= 1.5:  # 归一化坐标 -> 像素
                    x, y, w, h = x * img_w, y * img_h, w * img_w, h * img_h
                return cls._clamp_box(x, y, w, h, img_w, img_h)
            if "point" in ans and isinstance(ans["point"], (list, tuple)) and len(ans["point"]) == 2:
                x, y = [float(v) for v in ans["point"]]
                if max(x, y) <= 1.5:
                    x, y = x * img_w, y * img_h
                return cls._clamp_box(x - 20, y - 20, 40, 40, img_w, img_h)
        except (TypeError, ValueError):
            return None
        return None

    # ------------------------------------------------------------------
    # 投影 + 发布(与 TargetTFPublisher 同一套数学)
    # ------------------------------------------------------------------
    def _median_depth(self, depth, px, py, r=5):
        h, w = depth.shape[:2]
        px = max(0, min(px, w - 1))
        py = max(0, min(py, h - 1))
        patch = depth[max(0, py - r):min(h, py + r + 1), max(0, px - r):min(w, px + r + 1)]
        vals = patch[patch > 0]
        if vals.size == 0:
            return 0.0
        return float(np.median(vals)) / 1000.0  # mm -> m

    def _compute_target(self, px, py, depth):
        """计算并(若合法)设置目标点。返回 (ok, dis, reason)。安全加固集中在这里。

        只缓存与 z_offset 无关的量 (相机系 x,y 和实测深度 dis), z=dis+z_offset 推迟到
        广播时实时计算, 这样运行中 ros2 param set z_offset能立刻生效。
        """
        dis = self._median_depth(depth, px, py, 5)
        if dis <= 0:
            return False, 0.0, "中心深度无效(可能是无效深度区域)"
        if not (self.min_dist <= dis <= self.max_dist):
            return False, dis, "距离 %.3fm 超出允许范围 [%.2f, %.2f]m" % (
                dis, self.min_dist, self.max_dist)
        x = (px - self._cx) / self._fx * dis
        y = (py - self._cy) / self._fy * dis
        if not all(math.isfinite(v) for v in (x, y, dis)):
            return False, dis, "投影坐标非法(NaN/Inf)"
        self._target = (x, y, dis)   # 存原始量, z 偏移广播时再加
        return True, dis, ""

    def _publish_target(self):
        t = self._target              # 先取本地引用, 避免与处理线程置 None 竞争
        if t is None:
            return
        x, y, dis = t
        z = dis + self.z_offset       # 沿相机光轴(深度方向)实时施加 Z 偏移
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.camera_frame
        tf.child_frame_id = self.target_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.w = 1.0
        self.tf_pub.sendTransform(tf)
        d = Float32()
        d.data = float(dis)
        self.dist_pub.publish(d)

    def _publish_debug(self, bgr, box, label, dis):
        vis = bgr.copy()
        x, y, w, h = [int(v) for v in box]
        cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.drawMarker(vis, (x + w // 2, y + h // 2), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
        txt = label if dis <= 0 else "%s  dis=%.3fm" % (label, dis)
        cv2.putText(vis, txt, (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(vis, "bgr8"))
        except Exception:  # noqa: BLE001
            pass

    def _call_grab(self):
        # 仲裁: 手动接管期间不触发自动抓取(闭环抓取端也会拒绝, 这里提前给出提示)
        if self._manual_active:
            self.get_logger().warn("手动接管中, VLM 不触发自动抓取")
            self._publish_result("✋ 手动接管中, 暂不自动抓取(请在 Dashboard 释放后重试)")
            return
        if self.grab_client is None:
            self._publish_result("(auto_grab 开启但抓取服务客户端不可用)")
            return
        if not self.grab_client.service_is_ready():
            self.grab_client.wait_for_service(timeout_sec=2.0)
        if not self.grab_client.service_is_ready():
            self._publish_result("⚠ 抓取服务 %s 未就绪" % self.grab_service)
            return
        req = GrabObject.Request()
        req.obj_link = self.target_frame
        self.grab_client.call_async(req)
        self.get_logger().info("已请求抓取服务 %s (obj_link=%s)" % (self.grab_service, self.target_frame))

    def _publish_result(self, text):
        msg = String()
        msg.data = text
        self.result_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VlmGrabNode()
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
