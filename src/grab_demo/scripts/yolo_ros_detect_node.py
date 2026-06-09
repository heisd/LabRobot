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
        self._target = None          # (x, y, z) 相机系下目标; None 表示当前无目标
        self._last_dis = -1.0
        self._last_label = ""

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
        if not self._intrinsics_ready:
            self.get_logger().warn("等待相机内参 %s ..." % self.info_topic, throttle_duration_sec=5.0)
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error("深度图转换失败: %s" % e)
            return

        img_h, img_w = depth.shape[:2]
        img_cx, img_cy = img_w / 2.0, img_h / 2.0

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
            self.get_logger().info("未检测到目标物体", throttle_duration_sec=2.0)
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
            return

        if candidate_count > 1:
            self.get_logger().info(
                "检测到 %d 个候选, 按 %s 选中 [%s] conf=%.2f dis=%.3fm"
                % (candidate_count, self.select_mode, label, target.score, dis))
        else:
            self.get_logger().info("检测到 [%s] conf=%.2f dis=%.3fm"
                                   % (label, target.score, dis), throttle_duration_sec=1.0)

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
            pts = np.array([[p.x, p.y] for p in det.mask.data], dtype=np.int32)
            m = cv2.moments(pts)
            if m["m00"] > 0:                       # 多边形质心(面积加权)
                px = int(round(m["m10"] / m["m00"]))
                py = int(round(m["m01"] / m["m00"]))
            else:                                  # 退化(共线)时用边界点均值
                px = int(round(float(np.mean(pts[:, 0]))))
                py = int(round(float(np.mean(pts[:, 1]))))
            px = max(0, min(px, img_w - 1))
            py = max(0, min(py, img_h - 1))
            return px, py, self._mask_depth(depth, pts)
        # 默认 / 回退: bbox 中心
        px, py = self._center_px(det, img_w, img_h)
        return px, py, self._median_depth(depth, px, py)

    def _median_depth(self, depth, px, py, r=5):
        h, w = depth.shape[:2]
        px = max(0, min(px, w - 1))
        py = max(0, min(py, h - 1))
        patch = depth[max(0, py - r):min(h, py + r + 1), max(0, px - r):min(w, px + r + 1)]
        vals = patch[patch > 0]
        if vals.size == 0:
            return 0.0
        return float(np.median(vals)) / 1000.0  # mm -> m

    def _mask_depth(self, depth, pts):
        """掩码区域内非零深度的中值(米); 无有效深度返回 0。"""
        m = np.zeros(depth.shape[:2], dtype=np.uint8)
        cv2.fillPoly(m, [pts], 255)
        vals = depth[(m > 0) & (depth > 0)]
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
