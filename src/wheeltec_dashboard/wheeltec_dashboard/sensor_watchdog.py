#!/usr/bin/env python3
"""传感器在线监控(看门狗): 相机 / 雷达 / IMU / 下位机 的连接与掉线日志.

各厂商驱动对"设备打不开/拔线"的报错五花八门(有的只打一次, 有的不打),
本节点用统一口径补齐: 订阅每个传感器的关键数据话题, 以"话题有数据 = 设备
在线"为判据 —— 与 dashboard 各状态卡同一思路, 但落成 /rosout 日志,
终端与面板日志面板(按 sensor_watchdog 过滤)都能看到:

  - 首次收到数据      -> INFO  "[传感器] 相机 已上线 (/camera/color/image_raw)"
  - 断流超过 timeout  -> ERROR "[传感器] 相机 设备不在线 …"(每 repeat_sec 重复提醒)
  - 启动 grace 秒后仍无数据 -> 同样按"不在线"报 ERROR(开机就没插的情况)
  - 恢复数据          -> INFO  "[传感器] 相机 恢复在线"

监控项用字符串参数配置, 每项格式:  标签|话题|消息类型|超时秒
默认覆盖: 车载相机 / 雷达1 / 雷达2 / IMU / 下位机STM32(串口遥测)。
机械臂相机等可按需追加, 例如:
  机械臂相机|/camera_arm/color/image_raw|sensor_msgs/msg/Image|3.0

同时扫描上位机的串口设备并发布到 sensor_watchdog/status (String JSON,
dashboard "传感器与串口设备"卡渲染):
  - /dev/wheeltec_* 等 udev 别名 -> 实际占用的串口号(realpath, 如 /dev/ttyACM0)
  - /dev/serial/by-id/* -> USB 设备 ID(含厂商/产品/序列号) 与对应串口
设备拔掉后符号链接消失, 表里即只剩"已连接"的设备。

实现细节:
  - 订阅用 sensor-data QoS(BEST_EFFORT): 兼容相机/雷达常用的 best-effort
    发布者(RELIABLE 订阅与 BEST_EFFORT 发布者不匹配会收不到数据)。
  - raw=True 只收序列化字节不做反序列化, 监控大分辨率图像话题零开销。
"""

import glob
import json
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from rosidl_runtime_py.utilities import get_message

from std_msgs.msg import String


DEFAULT_SENSORS = [
    '车载相机|/camera/color/image_raw|sensor_msgs/msg/Image|3.0',
    '雷达1|/scan1|sensor_msgs/msg/LaserScan|3.0',
    '雷达2|/scan2|sensor_msgs/msg/LaserScan|3.0',
    'IMU|/imu/data_raw|sensor_msgs/msg/Imu|3.0',
    '下位机STM32|/PowerVoltage|std_msgs/msg/Float32|3.0',
    # 机械臂走以太网不占串口, 在线判据 = lebai_driver 的状态流
    'Lebai机械臂|/robot_status|lebai_interfaces/msg/RobotStatus|3.0',
    '机械臂相机|/camera_arm/color/image_raw|sensor_msgs/msg/Image|3.0',
]


class SensorEntry:
    """单个被监控的传感器: 在线状态机 + 日志节流."""

    def __init__(self, label, topic, type_name, timeout):
        self.label = label
        self.topic = topic
        self.type_name = type_name
        self.timeout = timeout
        self.last_stamp = None    # 最近一次收到数据的时间
        self.online = None        # None=从未收到, True/False=当前判定
        self.last_error_log = None


class SensorWatchdog(Node):

    def __init__(self):
        super().__init__('sensor_watchdog')

        self.declare_parameter('sensors', DEFAULT_SENSORS)
        self.declare_parameter('grace_sec', 10.0)    # 启动宽限期, 之后仍无数据才报不在线
        self.declare_parameter('repeat_sec', 30.0)   # 持续不在线的重复提醒间隔
        self.declare_parameter('check_period', 1.0)  # 检查周期
        # 串口设备别名匹配模式(udev 符号链接), 拔掉即从表中消失
        self.declare_parameter('device_globs', ['/dev/wheeltec_*', '/dev/lebai*'])
        self.declare_parameter('status_topic', 'sensor_watchdog/status')

        g = self.get_parameter
        self.grace_sec = float(g('grace_sec').value)
        self.repeat_sec = float(g('repeat_sec').value)
        self.device_globs = list(g('device_globs').value or [])
        self.start_time = self.get_clock().now()
        self.status_pub = self.create_publisher(String, g('status_topic').value, 10)
        self._devices = []
        self._device_scan_count = 0

        self.entries = []
        for spec in (g('sensors').value or []):
            entry = self._parse_spec(spec)
            if entry is None:
                continue
            try:
                msg_type = get_message(entry.type_name)
            except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                self.get_logger().error(
                    '[传感器] 配置项 "%s" 的消息类型 %s 无法加载, 跳过: %s' % (
                        entry.label, entry.type_name, exc))
                continue
            # raw=True: 只收序列化字节, 不反序列化(图像话题零拷贝判活)
            self.create_subscription(
                msg_type, entry.topic,
                (lambda e: (lambda _msg: self._data_cb(e)))(entry),
                qos_profile_sensor_data, raw=True)
            self.entries.append(entry)

        self.create_timer(float(g('check_period').value), self._check)
        self.get_logger().info(
            '传感器在线监控启动, 共 %d 项: %s (无数据宽限 %.0fs, 离线每 %.0fs 重复提醒)' % (
                len(self.entries), ', '.join(e.label for e in self.entries),
                self.grace_sec, self.repeat_sec))

    def _parse_spec(self, spec):
        parts = [p.strip() for p in str(spec or '').split('|')]
        if len(parts) < 3 or not parts[0] or not parts[1]:
            self.get_logger().error('[传感器] 配置项格式错误, 跳过: "%s" '
                                    '(应为 标签|话题|消息类型|超时秒)' % spec)
            return None
        try:
            timeout = float(parts[3]) if len(parts) > 3 and parts[3] else 3.0
        except ValueError:
            timeout = 3.0
        return SensorEntry(parts[0], parts[1], parts[2], timeout)

    def _data_cb(self, entry):
        entry.last_stamp = self.get_clock().now()
        if entry.online is not True:
            recovered = entry.online is False
            entry.online = True
            entry.last_error_log = None
            self.get_logger().info('[传感器] %s %s (%s)' % (
                entry.label, '恢复在线' if recovered else '已上线', entry.topic))

    def _check(self):
        now = self.get_clock().now()
        sec = lambda d: d.nanoseconds / 1e9  # noqa: E731
        for e in self.entries:
            if e.last_stamp is None:
                # 从未收到过数据: 过了启动宽限期才按"不在线"报
                if sec(now - self.start_time) < max(self.grace_sec, e.timeout):
                    continue
                offline_reason = '启动后一直无数据'
            elif sec(now - e.last_stamp) > e.timeout:
                offline_reason = '数据流中断'
            else:
                continue

            first_report = e.online is not False
            if first_report or (e.last_error_log is not None and
                                sec(now - e.last_error_log) >= self.repeat_sec):
                e.online = False
                e.last_error_log = now
                self.get_logger().error(
                    '[传感器] %s 设备不在线 (%s, 话题 %s 超 %.1fs 无数据)'
                    ' —— 检查供电/USB·串口连线/驱动是否启动' % (
                        e.label, offline_reason, e.topic, e.timeout))

        # 串口设备表 5s 扫一次(检查周期的第 5 拍), 状态 JSON 每拍都发
        self._device_scan_count += 1
        if self._device_scan_count >= 5 or self._device_scan_count == 1:
            if self._device_scan_count >= 5:
                self._device_scan_count = 0
            self._devices = self._scan_devices()
        self._publish_status(now)

    # ------------------------------------------------------------ device scan
    def _scan_devices(self):
        """枚举上位机串口设备: udev 别名 -> 实际串口, USB by-id -> 设备 ID.

        只有当前插着的设备才有符号链接, 因此表内容即"已连接"清单。
        """
        # /dev/serial/by-id/* 文件名内含 厂商_产品_序列号, 即 USB 设备 ID
        byid = {}   # realpath(串口) -> by-id 名称
        for p in glob.glob('/dev/serial/by-id/*'):
            try:
                byid[os.path.realpath(p)] = os.path.basename(p)
            except OSError:
                continue

        devices = []
        seen_ports = set()
        for pattern in self.device_globs:
            for alias_path in sorted(glob.glob(pattern)):
                try:
                    port = os.path.realpath(alias_path)
                except OSError:
                    continue
                devices.append({
                    'alias': os.path.basename(alias_path),
                    'port': port,
                    'usb_id': byid.get(port, ''),
                })
                seen_ports.add(port)
        # 没有 wheeltec 别名但插着的其它 USB 串口也列出来(别名留空)
        for port, usb_id in sorted(byid.items()):
            if port not in seen_ports:
                devices.append({'alias': '', 'port': port, 'usb_id': usb_id})
        return devices

    def _publish_status(self, now):
        sec = lambda d: d.nanoseconds / 1e9  # noqa: E731
        sensors = []
        for e in self.entries:
            sensors.append({
                'label': e.label,
                'topic': e.topic,
                # online: true/false/null(启动宽限期内还没数据)
                'online': e.online,
                'age': round(sec(now - e.last_stamp), 1) if e.last_stamp else None,
            })
        payload = {'sensors': sensors, 'devices': self._devices}
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = SensorWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
