#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""语音指令节点 voice_instruction_node.

把"说话"接到现有 VLM 自然语言抓取上:
    麦克风采集 --(能量VAD分句)--> 本地 Whisper 转写 --(文字)--> /vlm/instruction
VLM 节点(vlm_grab_node) **无需任何改动** —— 它照常订阅 /vlm/instruction 理解并抓取。

特点(与项目其它节点同思路):
  - **本地离线**: 用 faster-whisper(优先) 或 openai-whisper, 不依赖外网;
  - **依赖缺失优雅降级**: 没装 sounddevice / whisper 时打印清晰 ERROR 并保持存活
    (发布 /voice/status=error), 不会让整套 launch 崩;
  - 支持 **持续聆听(能量 VAD 自动分句)** 与 **按一下说一句(~/listen_once 服务)** 两种;
  - 把识别文字同时发到 /voice/text(供 Dashboard 显示)与 /voice/status(状态)。

依赖(按需 pip 安装, 见 VOICE_GUIDE.md):
    pip install sounddevice faster-whisper      # 推荐(CPU/GPU 都快)
    # 或   pip install sounddevice openai-whisper
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

import numpy as np

# ---- 可选依赖: 缺失时不崩, 运行时给出清晰提示 ----
try:
    import sounddevice as sd
except Exception as e:  # noqa: BLE001
    sd = None
    _SD_ERR = e


class VoiceInstructionNode(Node):
    def __init__(self):
        super().__init__("voice_instruction_node")

        # ---- 话题 ----
        self.instruction_topic = self.declare_parameter(
            "instruction_topic", "/vlm/instruction").value
        self.text_topic = self.declare_parameter("text_topic", "/voice/text").value
        self.status_topic = self.declare_parameter("status_topic", "/voice/status").value
        self.publish_to_instruction = bool(
            self.declare_parameter("publish_to_instruction", True).value)
        self.min_text_len = int(self.declare_parameter("min_text_len", 1).value)

        # ---- ASR 后端 ----
        self.backend = str(self.declare_parameter("backend", "faster-whisper").value)
        self.model = str(self.declare_parameter("model", "base").value)  # tiny/base/small/medium
        self.language = str(self.declare_parameter("language", "zh").value)
        self.device = str(self.declare_parameter("device", "cpu").value)  # cpu / cuda
        self.compute_type = str(self.declare_parameter("compute_type", "int8").value)  # faster-whisper

        # ---- 采集 / VAD ----
        self.sample_rate = int(self.declare_parameter("sample_rate", 16000).value)
        self.mic_device = int(self.declare_parameter("mic_device", -1).value)  # -1=默认设备
        self.block_ms = int(self.declare_parameter("block_ms", 30).value)
        self.vad_threshold = float(self.declare_parameter("vad_threshold", 0.012).value)  # RMS
        self.silence_sec = float(self.declare_parameter("silence_sec", 0.8).value)
        self.min_speech_sec = float(self.declare_parameter("min_speech_sec", 0.3).value)
        self.max_speech_sec = float(self.declare_parameter("max_speech_sec", 12.0).value)
        self.listen_timeout = float(self.declare_parameter("listen_timeout", 8.0).value)  # 按一下说一句的等待上限
        self.enabled = bool(self.declare_parameter("enabled", True).value)  # 是否持续聆听

        # ---- 发布 / 订阅 / 服务 ----
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.instr_pub = self.create_publisher(String, self.instruction_topic, 10)
        self.text_pub = self.create_publisher(String, self.text_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, latched)
        self.create_subscription(Bool, "/voice/enable", self._enable_cb, 10)
        self.create_service(Trigger, "~/listen_once", self._listen_once)

        # ---- 状态 ----
        self._model = None
        self._stream = None
        self._force = threading.Event()   # 按一下说一句
        self._status = "init"
        self._set_status("init")

        if sd is None:
            self.get_logger().error(
                "未安装 sounddevice, 语音功能不可用: %s  (请 `pip install sounddevice`)" % _SD_ERR)
            self._set_status("error: no sounddevice")
            return

        # 后台线程加载模型(慢, 不阻塞启动) + 采集/识别循环
        threading.Thread(target=self._load_model, daemon=True).start()
        threading.Thread(target=self._worker, daemon=True).start()

        self.get_logger().info(
            "voice_instruction_node 已启动 (backend=%s, model=%s, device=%s, language=%s, enabled=%s)"
            % (self.backend, self.model, self.device, self.language, self.enabled))

    # ------------------------------------------------------------------
    def _enable_cb(self, msg: Bool):
        self.enabled = bool(msg.data)
        self.get_logger().info("持续聆听 -> %s" % self.enabled)
        if not self.enabled:
            self._set_status("idle")

    def _listen_once(self, _req, res):
        if sd is None or self._model is None:
            res.success = False
            res.message = "语音未就绪(依赖缺失或模型未加载)"
            return res
        self._force.set()
        res.success = True
        res.message = "已开始录音, 请说话"
        return res

    def _set_status(self, s):
        self._status = s
        try:
            self.status_pub.publish(String(data=s))
        except Exception:  # noqa: BLE001
            pass

    # ------------------------------------------------------------------
    def _load_model(self):
        self._set_status("loading_model")
        try:
            if self.backend == "faster-whisper":
                from faster_whisper import WhisperModel
                self._model = WhisperModel(self.model, device=self.device,
                                           compute_type=self.compute_type)
            else:  # openai-whisper
                import whisper
                self._model = whisper.load_model(
                    self.model, device=(None if self.device == "cpu" else self.device))
            self.get_logger().info("语音模型已加载 (backend=%s model=%s)" % (self.backend, self.model))
            self._set_status("idle")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(
                "加载语音模型失败(是否已 `pip install %s`?): %s" % (self.backend, e))
            self._set_status("error: model load failed")

    def _open_stream(self):
        kwargs = dict(samplerate=self.sample_rate, channels=1, dtype="float32")
        if self.mic_device >= 0:
            kwargs["device"] = self.mic_device
        self._stream = sd.InputStream(**kwargs)
        self._stream.start()

    def _worker(self):
        while rclpy.ok():
            forced = self._force.is_set()
            if self._model is None or not (self.enabled or forced):
                time.sleep(0.1)
                continue
            if forced:
                self._force.clear()
            if self._stream is None:
                try:
                    self._open_stream()
                except Exception as e:  # noqa: BLE001
                    self.get_logger().error("打开麦克风失败(检查设备/权限): %s" % e,
                                            throttle_duration_sec=5.0)
                    self._set_status("error: no microphone")
                    time.sleep(1.0)
                    continue
            self._set_status("listening")
            audio = self._capture_utterance(forced)
            if audio is None:
                self._set_status("idle")
                continue
            self._set_status("transcribing")
            text = self._transcribe(audio)
            self._set_status("idle")
            if text:
                self._publish_text(text)
            else:
                self.get_logger().info("未识别到有效语音内容")

    # 能量 VAD 采集一句话, 返回 float32 单声道数组(失败/无语音返回 None)
    def _capture_utterance(self, forced):
        sr = self.sample_rate
        block = max(1, int(sr * self.block_ms / 1000.0))
        silence_need = max(1, int(self.silence_sec * 1000.0 / self.block_ms))
        max_blocks = max(1, int(self.max_speech_sec * 1000.0 / self.block_ms))
        pre, pre_max = [], 5
        collecting = False
        voiced = []
        silence_blocks = 0
        deadline = time.time() + self.listen_timeout
        while rclpy.ok():
            if not (self.enabled or forced):
                return None
            try:
                data, _ = self._stream.read(block)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error("麦克风读取失败: %s" % e, throttle_duration_sec=5.0)
                return None
            mono = data[:, 0] if getattr(data, "ndim", 1) > 1 else np.asarray(data).ravel()
            rms = float(np.sqrt(np.mean(np.square(mono))) + 1e-12)
            is_voice = rms >= self.vad_threshold
            if not collecting:
                pre.append(mono)
                if len(pre) > pre_max:
                    pre.pop(0)
                if is_voice:
                    collecting = True
                    voiced = list(pre)          # 带上前导, 避免吞掉开头
                    silence_blocks = 0
                elif forced and time.time() > deadline:
                    return None                 # 按一下说一句: 等了一会儿没人说话就退出
            else:
                voiced.append(mono)
                silence_blocks = 0 if is_voice else silence_blocks + 1
                if silence_blocks >= silence_need or len(voiced) >= max_blocks:
                    break
        if not voiced:
            return None
        audio = np.concatenate(voiced).astype(np.float32)
        if len(audio) / float(sr) < self.min_speech_sec:
            return None                          # 太短, 多半是噪声
        return audio

    def _transcribe(self, audio):
        try:
            lang = self.language or None
            if self.backend == "faster-whisper":
                segments, _info = self._model.transcribe(audio, language=lang, beam_size=1)
                return "".join(seg.text for seg in segments).strip()
            res = self._model.transcribe(audio, language=lang, fp16=(self.device != "cpu"))
            return str(res.get("text", "")).strip()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error("语音转写失败: %s" % e, throttle_duration_sec=2.0)
            return ""

    def _publish_text(self, text):
        self.text_pub.publish(String(data=text))
        self.get_logger().info("识别到: %s" % text)
        if self.publish_to_instruction and len(text) >= self.min_text_len:
            self.instr_pub.publish(String(data=text))
            self.get_logger().info("已转为指令发往 %s" % self.instruction_topic)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceInstructionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node._stream is not None:
                node._stream.stop()
                node._stream.close()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
