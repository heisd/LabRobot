"""与本地 Ollama 多模态大模型交互的客户端.

Ollama is talked to over its HTTP API (default http://localhost:11434).
给定一条自然语言指令(可附带摄像头图像)与已知航点清单,
让视觉语言模型(VLM)输出结构化的导航决策 JSON.
"""

import json
import re

import requests


# 要求大模型严格按这个 schema 输出 JSON
DECISION_SCHEMA = """{
  "action": "goto_waypoint | relative_move | stop | speak | unknown",
  "waypoint": "当 action=goto_waypoint 时, 目标航点的名称(必须取自给定清单)",
  "distance": "当 action=relative_move 时, 相对前进距离(米, 正数向前, 负数后退)",
  "angle": "当 action=relative_move 时, 需要转过的角度(度, 正数左转/逆时针, 负数右转)",
  "speak": "无论哪种 action, 都用一句简短中文告诉用户你将要做什么"
}"""

SYSTEM_PROMPT = """你是一台轮式机器人的导航大脑(Vision-Language-Action)。
用户会给你一条中文指令, 并可能附带机器人前方摄像头拍到的画面。
你要把指令解析成机器人能执行的导航动作, 并且只输出一个 JSON 对象, 不要输出多余文字。

可选的 action 含义:
- goto_waypoint: 前往一个已知的命名航点。只有当目标能对应到下面"已知航点"清单中的某一个时才用它, 并把 waypoint 设为清单里的名称。
- relative_move: 根据当前画面做相对移动, 例如"向前走一点""左转去那扇门"。用 distance(米)和 angle(度)描述。
- stop: 停止当前导航/让机器人停下。
- speak: 只需要语音回答, 不需要移动。
- unknown: 指令无法理解或与导航无关。

输出 JSON 必须符合以下结构(只填用得到的字段):
%s

注意:
1. 优先使用 goto_waypoint, 只有当指令明显是"相对当前位置/画面"的移动时才用 relative_move。
2. distance 一般不超过 2.0 米, angle 在 -180~180 度之间。
3. speak 用简短自然的中文。
""" % DECISION_SCHEMA


class VLMClient:
    """Ollama 多模态客户端 (生成 /api/generate)."""

    def __init__(self, base_url='http://localhost:11434', model='qwen2.5vl:3b',
                 timeout=60.0, temperature=0.2, logger=None):
        self.base_url = base_url.rstrip('/')
        self.model = model
        self.timeout = float(timeout)
        self.temperature = float(temperature)
        self.logger = logger

    def _log(self, level, msg):
        if self.logger is not None:
            getattr(self.logger, level, self.logger.info)(msg)

    def list_models(self):
        """返回 Ollama 当前已拉取的模型名列表; 出错时返回空列表."""
        try:
            resp = requests.get('%s/api/tags' % self.base_url, timeout=5.0)
            if resp.status_code == 200:
                return [m['name'] for m in resp.json().get('models', [])]
        except requests.RequestException as exc:
            self._log('warn', 'list_models failed: %s' % exc)
        return []

    def build_prompt(self, instruction, waypoint_text):
        return (
            '已知航点清单:\n%s\n\n'
            '用户指令: %s\n\n'
            '请只输出符合要求的 JSON。'
        ) % (waypoint_text, instruction)

    def query(self, instruction, waypoint_text='(无)', image_b64=None):
        """调用大模型, 返回解析后的决策 dict; 失败时返回带 action=unknown 的兜底结果."""
        payload = {
            'model': self.model,
            'system': SYSTEM_PROMPT,
            'prompt': self.build_prompt(instruction, waypoint_text),
            'stream': False,
            'format': 'json',
            'options': {'temperature': self.temperature},
        }
        if image_b64:
            payload['images'] = [image_b64]

        try:
            resp = requests.post('%s/api/generate' % self.base_url,
                                 json=payload, timeout=self.timeout)
        except requests.RequestException as exc:
            self._log('error', 'Ollama request failed: %s' % exc)
            return {'action': 'speak', 'speak': '我连不上大模型, 请检查 Ollama 是否启动。'}

        if resp.status_code != 200:
            self._log('error', 'Ollama HTTP %d: %s' % (resp.status_code, resp.text[:200]))
            return {'action': 'speak', 'speak': '大模型返回异常, 请稍后再试。'}

        raw = resp.json().get('response', '')
        decision = self.parse_decision(raw)
        if decision is None:
            self._log('warn', 'cannot parse model output: %s' % raw[:200])
            return {'action': 'unknown', 'speak': '抱歉, 我没理解这条指令。'}
        return decision

    @staticmethod
    def parse_decision(text):
        """从模型输出中尽量解析出 JSON 对象; 解析不出返回 None."""
        if not text:
            return None
        text = text.strip()
        # 去掉可能的 ```json ... ``` 代码块包裹
        if text.startswith('```'):
            text = re.sub(r'^```[a-zA-Z]*', '', text).strip()
            if text.endswith('```'):
                text = text[:-3].strip()
        try:
            return json.loads(text)
        except (ValueError, TypeError):
            pass
        # 退化: 抓取第一个 {...} 片段再尝试
        match = re.search(r'\{.*\}', text, re.DOTALL)
        if match:
            try:
                return json.loads(match.group(0))
            except (ValueError, TypeError):
                return None
        return None
