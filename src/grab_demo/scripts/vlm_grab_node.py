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

VLM 接口支持两种(provider 参数):
  - "openai"   : OpenAI 兼容 /chat/completions(可指向本地 Ollama/vLLM 或云端 OpenAI);
  - "anthropic": Anthropic Claude /v1/messages。
仅使用 Python 标准库 urllib, 无额外 pip 依赖。API Key 从环境变量读取(不写进参数)。
"""

import base64
import json
import os
import threading
import urllib.request
import urllib.error

import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Float32
from geometry_msgs.msg import TransformStamped
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
        self.provider = self.declare_parameter("provider", "openai").value  # openai | anthropic
        self.api_base = self.declare_parameter("api_base", "https://api.openai.com/v1").value
        self.model = self.declare_parameter("model", "gpt-4o-mini").value
        self.api_key_env = self.declare_parameter("api_key_env", "").value  # 留空则按 provider 取默认
        self.timeout = float(self.declare_parameter("request_timeout", 30.0).value)

        # 抓取
        self.auto_grab = bool(self.declare_parameter("auto_grab", True).value)
        self.grab_service = self.declare_parameter("grab_service", "/obj_grab_service").value
        self.publish_debug_image = bool(self.declare_parameter("publish_debug_image", True).value)

        self.instruction_topic = self.declare_parameter("instruction_topic", "/vlm/instruction").value
        self.result_topic = self.declare_parameter("result_topic", "/vlm/result").value
        self.distance_topic = self.declare_parameter("distance_topic", "/grab_target/distance").value

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
        self._target = None     # (x, y, z) 相机系下目标点; None 表示无目标
        self._busy = False

        # ---- 订阅 ----
        self.create_subscription(CameraInfo, self.info_topic, self._info_cb, 1)
        rgb_sub = message_filters.Subscriber(self, Image, self.rgb_topic)
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        self._sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 0.3)
        self._sync.registerCallback(self._image_cb)
        self.create_subscription(String, self.instruction_topic, self._instruction_cb, 10)

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

        self.get_logger().info(
            "vlm_grab_node 已启动 (provider=%s, model=%s, auto_grab=%s). "
            "向 %s 发送指令开始。" % (self.provider, self.model, self.auto_grab, self.instruction_topic))

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
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error("cv_bridge 转换失败: %s" % e)
            return
        with self._lock:
            self._rgb = rgb
            self._depth = depth

    def _instruction_cb(self, msg: String):
        text = (msg.data or "").strip()
        if not text:
            return
        if self._busy:
            self._publish_result("⏳ 正在处理上一条指令, 请稍候...")
            return
        # 在线程里调用 VLM(网络阻塞), 不卡住 ROS 执行器
        threading.Thread(target=self._process, args=(text,), daemon=True).start()

    # ------------------------------------------------------------------
    # 主流程
    # ------------------------------------------------------------------
    def _process(self, instruction: str):
        self._busy = True
        try:
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
                self._publish_result("❌ VLM 调用失败(检查 provider/api_base/api_key/网络)")
                return
            if not ans.get("found", False):
                self._target = None
                self._publish_result("🚫 未找到合适物体: %s" % ans.get("reason", ""))
                return

            box = self._parse_box(ans, rgb.shape[1], rgb.shape[0])
            if box is None:
                self._publish_result("❌ VLM 返回的框无法解析: %s" % json.dumps(ans, ensure_ascii=False))
                return
            x, y, w, h = box
            px, py = int(x + w / 2), int(y + h / 2)

            dis = self._set_target(px, py, depth)
            label = ans.get("label", "目标")
            reason = ans.get("reason", "")
            if self.publish_debug_image:
                self._publish_debug(rgb, box, label, dis)

            if dis <= 0:
                self._publish_result("⚠ 已识别 [%s] 但中心深度无效, 无法定位" % label)
                return

            msg = "✅ 理解为 [%s] (%s), 距离 %.3fm" % (label, reason, dis)
            self.get_logger().info(msg)
            self._publish_result(msg)

            if self.auto_grab:
                self._call_grab()
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

    def _post(self, url, headers, body):
        req = urllib.request.Request(url, data=json.dumps(body).encode("utf-8"),
                                     headers=headers, method="POST")
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return json.loads(r.read().decode("utf-8"))

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
    def _parse_box(ans, img_w, img_h):
        # 支持 bbox=[x,y,w,h]; 兼容归一化(0~1)坐标; 或 point=[x,y]
        if "bbox" in ans and isinstance(ans["bbox"], (list, tuple)) and len(ans["bbox"]) == 4:
            x, y, w, h = [float(v) for v in ans["bbox"]]
            if max(x, y, w, h) <= 1.5:  # 归一化
                x, y, w, h = x * img_w, y * img_h, w * img_w, h * img_h
            return [x, y, max(w, 1.0), max(h, 1.0)]
        if "point" in ans and isinstance(ans["point"], (list, tuple)) and len(ans["point"]) == 2:
            x, y = [float(v) for v in ans["point"]]
            if max(x, y) <= 1.5:
                x, y = x * img_w, y * img_h
            return [x - 20, y - 20, 40, 40]  # 给个小框
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

    def _set_target(self, px, py, depth):
        dis = self._median_depth(depth, px, py, 5)
        if dis <= 0:
            self._target = None
            return 0.0
        x = (px - self._cx) / self._fx * dis
        y = (py - self._cy) / self._fy * dis
        z = dis + self.z_offset
        self._target = (x, y, z)
        return dis

    def _publish_target(self):
        if self._target is None:
            return
        x, y, z = self._target
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
        d.data = float(z - self.z_offset)
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
