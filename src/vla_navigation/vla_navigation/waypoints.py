"""命名航点的加载与匹配工具 / Named waypoint loading and matching helpers."""

import math
import os

import yaml


class Waypoint:
    """单个命名航点 (地图坐标系 map frame 下的位姿)."""

    def __init__(self, name, x, y, yaw=0.0, aliases=None):
        self.name = str(name)
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.aliases = [str(a) for a in (aliases or [])]

    def all_names(self):
        return [self.name] + self.aliases

    def quaternion(self):
        """把 yaw(弧度) 转成四元数的 (z, w) 分量 (平面机器人只绕 z 轴转)."""
        return math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)

    def __repr__(self):
        return 'Waypoint(name=%r, x=%.2f, y=%.2f, yaw=%.2f)' % (
            self.name, self.x, self.y, self.yaw)


class WaypointMap:
    """一组命名航点, 支持按名称/别名做容错匹配."""

    def __init__(self, waypoints=None):
        self.waypoints = list(waypoints or [])

    @classmethod
    def from_yaml(cls, path):
        """从 yaml 文件加载航点; 文件不存在或为空时返回空地图."""
        if not path or not os.path.isfile(path):
            return cls([])
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        items = data.get('waypoints', []) or []
        waypoints = []
        for item in items:
            try:
                waypoints.append(Waypoint(
                    name=item['name'],
                    x=item['x'],
                    y=item['y'],
                    yaw=item.get('yaw', 0.0),
                    aliases=item.get('aliases', []),
                ))
            except (KeyError, TypeError, ValueError):
                # 跳过格式不正确的条目, 不影响其余航点
                continue
        return cls(waypoints)

    def names(self):
        return [wp.name for wp in self.waypoints]

    def describe(self):
        """生成给大模型看的航点清单文本."""
        if not self.waypoints:
            return '(当前没有预设航点)'
        lines = []
        for wp in self.waypoints:
            alias = ('，别名: ' + '/'.join(wp.aliases)) if wp.aliases else ''
            lines.append('- %s (x=%.2f, y=%.2f)%s' % (wp.name, wp.x, wp.y, alias))
        return '\n'.join(lines)

    def match(self, query):
        """按名称/别名匹配航点; 支持大小写无关与子串包含, 找不到返回 None."""
        if not query:
            return None
        q = str(query).strip().lower()
        if not q:
            return None
        # 1) 完全匹配优先
        for wp in self.waypoints:
            for name in wp.all_names():
                if q == name.lower():
                    return wp
        # 2) 退化为子串包含 (双向), 处理 "去厨房" / "kitchen room" 之类
        for wp in self.waypoints:
            for name in wp.all_names():
                nl = name.lower()
                if nl and (nl in q or q in nl):
                    return wp
        return None
