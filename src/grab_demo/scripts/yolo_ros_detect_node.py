#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yolo_ros 桥接节点 (yolo_ros_detect_node).

把 **官方 yolo_ros (https://github.com/mgonzs13/yolo_ros)** 的检测结果接入到本项目
既有的抓取流程里, 取代原来自研的 TensorRT 版 `yolo_detect_node`。

设计目标: 让 yolo_ros 成为"识别前端", 而抓取后端 (`grab_service_node` /
`closed_loop_grab_node`) **完全不用改动** —— 它们读取的依然是统一的
`target_frame` TF。

数据流:
    yolo_ros/yolo_node  --(yolo_msgs/DetectionArray)-->  本节点
                                                          + 深度图 + 相机内参
                                                          |
                          (选目标 + 中心邻域深度中值 + 针孔反投影)
                                                          v
                    持续广播  camera_frame -> target_frame  (与 HSV/YOLO/VLM 一致)
                    + 距离发布到 /grab_target/distance

与原 `yolo_detect_node` 的对齐关系:
    * 输出 TF / 距离话题 / select_mode 选择策略 全部保持一致;
    * 反投影数学与 `target_tf_publisher.hpp` / `vlm_grab_node.py` 完全相同;
    * 不再自己跑推理, 推理交给 yolo_ros(ultralytics), 因此 **不依赖 TensorRT**,
      在装了 ultralytics 的 cv + humble Docker 环境里 CPU/GPU 都能跑。

闭环 (闭环检测) 说明:
    本节点本身就是闭环的"感知"一环 —— 它以相机帧率持续重新检测并刷新 target_frame。
    配合 `closed_loop_grab_node` (它在抓取过程中反复读取最新 target_frame 并修正
    机械臂位姿), 构成 "看 -> 动 -> 再看 -> 再修正" 的位置闭环 (PBVS)。
"""

import math
import threading
import time
import traceback

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
from tf2_ros import TransformBroadcaster
import message_filters

try:
    from cv_bridge import CvBridge
    import cv2
except Exception:  # noqa: BLE001
    CvBridge = None
    cv2 = None

# yolo_ros 的消息类型. 没装 yolo_msgs 时给出清晰的报错而不是 import 崩溃。
try:
    from yolo_msgs.msg import DetectionArray
except Exception as _e:  # noqa: BLE001
    DetectionArray = None
    _YOLO_MSGS_IMPORT_ERROR = _e


class YoloRosDetectNode(Node):
    def __init__(self):
        super().__init__("yolo_ros_detect_node")

        # ---- 话题 / 坐标系参数 (默认与 HSV/YOLO/VLM 完全一致) ----
        self.detections_topic = self.declare_parameter(
            "detections_topic", "/yolo/detections").value
        self.depth_topic = self.declare_parameter(
            "depth_topic", "/camera_arm/depth/image_raw").value
        self.info_topic = self.declare_parameter(
            "camera_info_topic", "/gemini_info").value
        self.camera_frame = self.declare_parameter(
            "camera_frame", "camera_arm_depth_optical_frame").value
        self.target_frame = self.declare_parameter("target_frame", "target_frame").value
        self.z_offset = float(self.declare_parameter("z_offset", 0.07).value)
        self.distance_topic = self.declare_parameter(
            "distance_topic", "/grab_target/distance").value

        # ---- 目标筛选 ----
        # target_class: COCO 类 id, -1 表示不限类别 (瓶子=39, 杯子=41 ...)
        # target_label: 类别名(不区分大小写), 非空时优先于 target_class; "" 表示不限
        self.target_class = int(self.declare_parameter("target_class", -1).value)
        self.target_label = str(self.declare_parameter("target_label", "").value).strip().lower()
        # 额外置信度过滤(yolo_node 自身已有 threshold, 这里默认不再过滤)
        self.conf_threshold = float(self.declare_parameter("conf_threshold", 0.0).value)

        # ---- 多目标选择策略 ----
        #   confidence = 置信度最高(默认)  nearest = 离相机最近
        #   center     = 最靠近画面中心     largest = 检测框最大
        self.select_mode = str(self.declare_parameter("select_mode", "confidence").value)
        if self.select_mode not in ("confidence", "nearest", "center", "largest"):
            self.get_logger().warn("未知 select_mode='%s', 回退为 confidence" % self.select_mode)
            self.select_mode = "confidence"

        # ---- 抓取中心的计算方式 ----
        #   bbox = 检测框几何中心(默认, 适合规则物体)
        #   mask = 分割掩码的质心 + 掩码内深度中值(适合不规则物体, 需 -seg 分割模型)
        # 用 mask 时若该检测没有掩码(用的是检测模型而非分割模型), 自动回退到 bbox。
        self.center_mode = str(self.declare_parameter("center_mode", "bbox").value)
        if self.center_mode not in ("bbox", "mask"):
            self.get_logger().warn("未知 center_mode='%s', 回退为 bbox" % self.center_mode)
            self.center_mode = "bbox"

        # ---- 安全 / 距离约束 ----
        self.min_dist = float(self.declare_parameter("min_dist", 0.1).value)
        self.max_dist = float(self.declare_parameter("max_dist", 2.0).value)

        # ---- 可视化 ----
        self.publish_debug_image = bool(
            self.declare_parameter("publish_debug_image", True).value)

        # ---- 时间同步参数 ----
        self.sync_queue = int(self.declare_parameter("sync_queue", 10).value)
        self.sync_slop = float(self.declare_parameter("sync_slop", 0.3).value)

        if DetectionArray is None:
            self.get_logger().fatal(
                "无法导入 yolo_msgs.msg.DetectionArray, 请先编译 yolo_ros 的 yolo_msgs 包: %s"
                % _YOLO_MSGS_IMPORT_ERROR)
            raise RuntimeError("yolo_msgs not available")
        if CvBridge is None:
            self.get_logger().fatal("缺少 cv_bridge, 无法运行")
            raise RuntimeError("cv_bridge missing")
        self.bridge = CvBridge()

        # ---- 状态 ----
        self._lock = threading.Lock()
        self._fx = self._fy = self._cx = self._cy = 0.0
        self._intrinsics_ready = False
        self._target = None          # (x, y, dis) 相机系下目标; None 表示当前无目标
        self._last_dis = -1.0
        self._last_label = ""
        # ---- 异常/健康监控状态 ----
        self._last_det_time = 0.0    # 最近一次收到检测消息的时刻(秒); 0=从未收到
        self._had_target = False     # 上一帧是否有有效目标(用于打印"丢失"日志)
        self._cb_errors = 0          # 回调累计异常次数
        self._warn_no_det = True     # 是否还需要提示"未收到检测"(收到后复位)
        self._stale_warned = False   # 检测中断告警去抖

        # ---- 订阅 ----
        self.create_subscription(CameraInfo, self.info_topic, self._info_cb, 1)
        det_sub = message_filters.Subscriber(self, DetectionArray, self.detections_topic)
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [det_sub, depth_sub], self.sync_queue, self.sync_slop)
        self._sync.registerCallback(self._sync_cb)

        # ---- 发布 ----
        self.dist_pub = self.create_publisher(Float32, self.distance_topic, 10)
        self.tf_pub = TransformBroadcaster(self)
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(Image, "~/detection_image", 1)
            # 仅画框时也需要彩色帧, 单独订阅(不参与同步, 取最新一帧即可)
            rgb_topic = self.declare_parameter(
                "rgb_topic", "/camera_arm/color/image_raw").value
            self._rgb = None
            self.create_subscription(Image, rgb_topic, self._rgb_cb, 1)
        else:
            self.debug_pub = None
            self._rgb = None

        # 以 15Hz 持续广播最近一次的 target_frame + 距离, 保证抓取服务随时能查到 TF
        self.create_timer(1.0 / 15.0, self._publish_target)

        # 检测健康监控: 若超过 det_timeout 秒收不到检测消息, 打印告警(便于排查
        # yolo_ros 未启动 / 模型未加载 / 话题不匹配等异常)。
        self.det_timeout = float(self.declare_parameter("det_timeout", 3.0).value)
        self.create_timer(1.0, self._watchdog)

        # 运行时可动态调节的参数 (ros2 param set 立即生效), 重点是 z_offset:
        # 沿相机光轴的深度偏移, 调大=抓得更深(物体内部), 调小/负值=更靠近相机表面,
        # 从而适配不同深度/不同厚度的物体。
        self.add_on_set_parameters_callback(self._on_set_params)

        self.get_logger().info(
            "yolo_ros_detect_node 已启动: 订阅 %s, select_mode=%s, "
            "target_class=%d, target_label='%s', z_offset=%.3f"
            % (self.detections_topic, self.select_mode, self.target_class,
               self.target_label, self.z_offset))

    # 动态参数回调: 允许运行中用 `ros2 param set /yolo_ros_node <name> <value>` 调整
    def _on_set_params(self, params):
        for p in params:
            try:
                if p.name == "z_offset":
                    with self._lock:
                        self.z_offset = float(p.value)
                    self.get_logger().info("z_offset -> %.3f m" % self.z_offset)
                elif p.name == "min_dist":
                    self.min_dist = float(p.value)
                elif p.name == "max_dist":
                    self.max_dist = float(p.value)
                elif p.name == "conf_threshold":
                    self.conf_threshold = float(p.value)
                    self.get_logger().info("conf_threshold -> %.2f" % self.conf_threshold)
                elif p.name == "center_mode":
                    mode = str(p.value)
                    if mode not in ("bbox", "mask"):
                        return SetParametersResult(
                            successful=False, reason="center_mode 取值非法(bbox/mask)")
                    self.center_mode = mode
                    self.get_logger().info("center_mode -> %s" % self.center_mode)
                elif p.name == "target_class":
                    self.target_class = int(p.value)
                    self.get_logger().info("target_class -> %d" % self.target_class)
                elif p.name == "target_label":
                    self.target_label = str(p.value).strip().lower()
                    self.get_logger().info("target_label -> '%s'" % self.target_label)
                elif p.name == "select_mode":
                    mode = str(p.value)
                    if mode not in ("confidence", "nearest", "center", "largest"):
                        return SetParametersResult(
                            successful=False, reason="select_mode 取值非法")
                    self.select_mode = mode
                    self.get_logger().info("select_mode -> %s" % self.select_mode)
            except (TypeError, ValueError) as e:  # noqa: PERF203
                return SetParametersResult(successful=False, reason=str(e))
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # 回调
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

    def _rgb_cb(self, msg: Image):
        try:
            self._rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:  # noqa: BLE001
            pass

    def _sync_cb(self, det_msg, depth_msg: Image):
        # 健康监控: 记录"确实收到了检测消息"(即便本帧无目标也说明 yolo_ros 在工作)
        self._last_det_time = time.time()
        if self._warn_no_det:
            self.get_logger().info("已开始接收 yolo_ros 检测消息 (%s)" % self.detections_topic)
            self._warn_no_det = False
        self._stale_warned = False
        # 任何未预料的异常都记录(含堆栈), 避免回调静默失效导致整条链路"假死"
        try:
            self._process_frame(det_msg, depth_msg)
        except Exception as e:  # noqa: BLE001
            self._cb_errors += 1
            self.get_logger().error(
                "检测处理回调异常(累计 %d 次): %s\n%s"
                % (self._cb_errors, e, traceback.format_exc()),
                throttle_duration_sec=2.0)

    def _process_frame(self, det_msg, depth_msg: Image):
        if not self._intrinsics_ready:
            self.get_logger().warn("等待相机内参 %s ..." % self.info_topic, throttle_duration_sec=5.0)
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error("深度图转换失败(编码是否为 16UC1?): %s" % e,
                                    throttle_duration_sec=2.0)
            return

        img_h, img_w = depth.shape[:2]
        img_cx, img_cy = img_w / 2.0, img_h / 2.0

        # mask 模式下掩码像素坐标基于彩色图, 与深度图分辨率不一致会错位, 这里给出告警
        if self.center_mode == "mask" and self._rgb is not None:
            rh, rw = self._rgb.shape[:2]
            if (rh, rw) != (img_h, img_w):
                self.get_logger().warn(
                    "mask 模式下彩色(%dx%d)与深度(%dx%d)分辨率不一致, 掩码可能错位"
                    % (rw, rh, img_w, img_h), throttle_duration_sec=5.0)

        # ---- 候选过滤 + 选目标 ----
        # 每个候选都按 center_mode 算出抓取中心像素 (px,py) 和该中心深度 dis,
        # 再按 select_mode 比较。这样 mask/bbox 两种中心方式对所有策略都一致生效。
        best = None          # (metric, det, px, py, dis)
        candidate_count = 0

        for det in det_msg.detections:
            if not self._match_class(det):
                continue
            if det.score < self.conf_threshold:
                continue
            candidate_count += 1

            px, py, dis = self._object_center(det, depth)

            if self.select_mode == "nearest":
                if dis <= 0.0:
                    continue                # 无有效深度的候选跳过
                metric = -dis               # 越近越优
            elif self.select_mode == "center":
                metric = -((px - img_cx) ** 2 + (py - img_cy) ** 2)
            elif self.select_mode == "largest":
                metric = float(det.bbox.size.x * det.bbox.size.y)
            else:  # confidence
                metric = float(det.score)

            if best is None or metric > best[0]:
                best = (metric, det, px, py, dis)

        if best is None:
            # 区分"画面里啥都没有" vs "有检测但都被类别/置信度过滤掉了", 便于排查
            if len(det_msg.detections) == 0:
                self.get_logger().info("画面中未检测到任何物体", throttle_duration_sec=2.0)
            else:
                self.get_logger().info(
                    "检测到 %d 个物体, 但无一满足筛选(target_class=%d/label='%s'/conf>=%.2f)"
                    % (len(det_msg.detections), self.target_class,
                       self.target_label, self.conf_threshold),
                    throttle_duration_sec=2.0)
            self._lose_target()
            if self.publish_debug_image:
                self._publish_debug(det_msg, None, None, -1.0)
            return

        # ---- 抓取中心(像素) + 深度 -> 针孔反投影 ----
        _, target, px, py, dis_sel = best
        ok, dis, why = self._set_target(px, py, dis_sel)
        label = target.class_name or ("id_%d" % target.class_id)
        with self._lock:
            self._last_label = label
            self._last_dis = dis if ok else -1.0

        if self.publish_debug_image:
            self._publish_debug(det_msg, target, (px, py), dis if ok else -1.0)

        if not ok:
            with self._lock:
                self._target = None
            self.get_logger().warn("目标 [%s] 无法定位: %s" % (label, why),
                                   throttle_duration_sec=2.0)
            self._lose_target()
            return

        # 成功定位: 首次锁定时打一条 info, 之后节流
        if not self._had_target:
            self.get_logger().info("已锁定目标 [%s] conf=%.2f dis=%.3fm" % (label, target.score, dis))
        self._had_target = True
        if candidate_count > 1:
            self.get_logger().info(
                "检测到 %d 个候选, 按 %s 选中 [%s] conf=%.2f dis=%.3fm"
                % (candidate_count, self.select_mode, label, target.score, dis),
                throttle_duration_sec=1.0)
        else:
            self.get_logger().info("检测到 [%s] conf=%.2f dis=%.3fm"
                                   % (label, target.score, dis), throttle_duration_sec=1.0)

    def _lose_target(self):
        """目标从"有"变"无"时打一条告警(只打一次)。"""
        if self._had_target:
            self.get_logger().warn("目标丢失, 已停止刷新 target_frame")
        self._had_target = False

    def _watchdog(self):
        """1Hz 健康检查: 检测流中断时告警, 并停止广播过期目标(闭环安全)。"""
        if not self._intrinsics_ready:
            return
        if self._last_det_time == 0.0:
            self.get_logger().warn(
                "尚未收到任何 yolo_ros 检测 (%s): 确认已启动 yolo_ros 且 input_image_topic 正确"
                % self.detections_topic, throttle_duration_sec=5.0)
            return
        gap = time.time() - self._last_det_time
        if gap > self.det_timeout and not self._stale_warned:
            self.get_logger().error(
                "已 %.1fs 未收到检测消息(>%.1fs): yolo_ros 崩溃 / 相机掉线 / 话题不匹配?"
                % (gap, self.det_timeout))
            self._stale_warned = True
            with self._lock:        # 停止广播过期目标, 让抓取端及时发现 TF 失效
                self._target = None
            self._lose_target()

    # ------------------------------------------------------------------
    # 工具
    # ------------------------------------------------------------------
    def _match_class(self, det):
        if self.target_label:
            return (det.class_name or "").strip().lower() == self.target_label
        if self.target_class >= 0:
            return det.class_id == self.target_class
        return True

    @staticmethod
    def _center_px(det, img_w, img_h):
        px = int(min(max(det.bbox.center.position.x, 0), img_w - 1))
        py = int(min(max(det.bbox.center.position.y, 0), img_h - 1))
        return px, py

    def _object_center(self, det, depth):
        """按 center_mode 计算物体抓取中心像素与深度, 返回 (px, py, dis米)。

        - bbox: 检测框几何中心 + 中心 11x11 邻域深度中值 (规则物体足够)。
        - mask: 分割掩码的质心 + 掩码内深度中值 (不规则物体更准, 质心落在物体上,
                深度取整个物体表面的中值, 抗噪更强)。掩码缺失时自动回退到 bbox。
        """
        img_h, img_w = depth.shape[:2]
        if self.center_mode == "mask" and cv2 is not None \
                and getattr(det, "mask", None) is not None and len(det.mask.data) >= 3:
            pts = np.array([[p.x, p.y] for p in det.mask.data], dtype=np.float64)
            # 1) 填充掩码(一次), 后续质心校验与深度都复用它
            m = np.zeros((img_h, img_w), dtype=np.uint8)
            cv2.fillPoly(m, [pts.astype(np.int32)], 255)
            # 2) 面积加权质心(精确公式; 避免 cv2.moments 把 (N,2) 点集误当成图像)
            cx, cy = self._polygon_centroid(pts)
            px = max(0, min(int(round(cx)), img_w - 1))
            py = max(0, min(int(round(cy)), img_h - 1))
            # 3) 凹形/带孔物体的质心可能落在物体之外(掩码值为0),
            #    此时取掩码内"最深"的点(距离变换峰值), 保证抓取中心确实落在物体上。
            if m[py, px] == 0:
                ipx, ipy = self._deepest_mask_point(m)
                if ipx is not None:
                    px, py = ipx, ipy
            # 4) 深度: 掩码区域内非零深度中值(比单点更稳)
            vals = depth[(m > 0) & (depth > 0)]
            dis = float(np.median(vals)) / 1000.0 if vals.size else 0.0  # mm -> m
            return px, py, dis
        # 默认 / 回退: bbox 中心
        px, py = self._center_px(det, img_w, img_h)
        return px, py, self._median_depth(depth, px, py)

    @staticmethod
    def _polygon_centroid(pts):
        """多边形面积加权质心(shoelace 公式)。pts: Nx2 float。退化时回退顶点均值。"""
        x = pts[:, 0]
        y = pts[:, 1]
        x1 = np.roll(x, -1)
        y1 = np.roll(y, -1)
        cross = x * y1 - x1 * y
        area = cross.sum() / 2.0
        if abs(area) < 1e-6:                 # 共线/重合点 -> 面积为 0, 用顶点均值
            return float(x.mean()), float(y.mean())
        cx = ((x + x1) * cross).sum() / (6.0 * area)
        cy = ((y + y1) * cross).sum() / (6.0 * area)
        return float(cx), float(cy)

    @staticmethod
    def _deepest_mask_point(m):
        """掩码内距边界最远的点(距离变换峰值), 用作凹形物体的稳妥抓取中心。"""
        try:
            dist = cv2.distanceTransform(m, cv2.DIST_L2, 5)
        except Exception:  # noqa: BLE001
            return None, None
        _, _, _, maxloc = cv2.minMaxLoc(dist)
        return int(maxloc[0]), int(maxloc[1])

    def _median_depth(self, depth, px, py, r=5):
        h, w = depth.shape[:2]
        px = max(0, min(px, w - 1))
        py = max(0, min(py, h - 1))
        patch = depth[max(0, py - r):min(h, py + r + 1), max(0, px - r):min(w, px + r + 1)]
        vals = patch[patch > 0]
        if vals.size == 0:
            return 0.0
        return float(np.median(vals)) / 1000.0  # mm -> m

    def _set_target(self, px, py, dis):
        """用给定的抓取中心像素 + 深度 dis(米)反投影并缓存目标。返回 (ok, dis, reason)。

        注意: 这里只缓存与 z_offset 无关的量 (相机系 x, y 和实测深度 dis),
        z = dis + z_offset 推迟到广播时实时计算, 这样运行中 ros2 param set z_offset
        能立刻生效, 无需等待下一帧检测。
        """
        if dis <= 0:
            return False, 0.0, "中心深度无效"
        if not (self.min_dist <= dis <= self.max_dist):
            return False, dis, "距离 %.3fm 超出允许范围 [%.2f, %.2f]m" % (
                dis, self.min_dist, self.max_dist)
        x = (px - self._cx) / self._fx * dis
        y = (py - self._cy) / self._fy * dis
        if not all(math.isfinite(v) for v in (x, y, dis)):
            return False, dis, "投影坐标非法(NaN/Inf)"
        with self._lock:
            self._target = (x, y, dis)   # 存原始量, z 偏移广播时再加
        return True, dis, ""

    def _publish_target(self):
        with self._lock:
            target = self._target
            dis = self._last_dis
            z_off = self.z_offset
        if target is None:
            return
        x, y, raw_dis = target
        z = raw_dis + z_off            # 沿相机光轴(深度方向)实时施加 Z 偏移
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.camera_frame
        tf.child_frame_id = self.target_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.w = 1.0   # 只给位置, 姿态由抓取服务决定
        self.tf_pub.sendTransform(tf)
        if dis > 0:
            d = Float32()
            d.data = float(dis)
            self.dist_pub.publish(d)

    def _publish_debug(self, det_msg, target, center, dis):
        if cv2 is None or self._rgb is None:
            return
        vis = self._rgb.copy()
        for det in det_msg.detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            w = det.bbox.size.x
            h = det.bbox.size.y
            x0, y0 = int(cx - w / 2), int(cy - h / 2)
            x1, y1 = int(cx + w / 2), int(cy + h / 2)
            is_target = target is not None and det is target
            color = (0, 0, 255) if is_target else (0, 255, 0)
            cv2.rectangle(vis, (x0, y0), (x1, y1), color, 3 if is_target else 2)
            # 有分割掩码时把轮廓也画出来, 便于确认不规则物体的范围
            if cv2 is not None and getattr(det, "mask", None) is not None \
                    and len(det.mask.data) >= 3:
                poly = np.array([[int(p.x), int(p.y)] for p in det.mask.data], dtype=np.int32)
                cv2.polylines(vis, [poly], True, color, 1)
            label = "%s %.2f" % (det.class_name or det.class_id, det.score)
            cv2.putText(vis, label, (x0, max(0, y0 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        # 用真正的抓取中心(可能是掩码质心)画十字, 而不是固定 bbox 中心
        if target is not None and center is not None:
            cx, cy = int(center[0]), int(center[1])
            cv2.drawMarker(vis, (cx, cy), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
            if dis > 0:
                cv2.putText(vis, "dis=%.3fm" % dis, (cx - 40, cy + 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        try:
            msg = self.bridge.cv2_to_imgmsg(vis, "bgr8")
            msg.header = det_msg.header
            self.debug_pub.publish(msg)
        except Exception:  # noqa: BLE001
            pass


def main(args=None):
    rclpy.init(args=args)
    node = YoloRosDetectNode()
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
