#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""detection_marker.py —— 把 2D 检测框投影成 RViz 里的 3D Marker（M3）

数据流：
    /detections (Detection2DArray, RELIABLE)
        │  ① 取每框 center/size（像素，原点左上角，y 向下）
        │  ② 针孔模型投影到"固定深度平面" Z=fixed_depth：
        │       X = (u - cx) * Z / fx
        │       Y = (v - cy) * Z / fy
        ▼
    /detection_markers (MarkerArray)
        - ns="box"   ：LINE_STRIP 矩形框，画在 camera_optical_link 的 Z 平面上
        - ns="label" ：TEXT_VIEW_FACING 类别名+置信度，始终面向屏幕

为什么 frame_id 用 camera_optical_link：
    针孔公式的数学前提就是光学系（z 前/x 右/y 下），算出来的点天然属于该系；
    光学系→机器人系→map 的旋转平移全部由 TF 链完成（URDF camera_optical_joint
    + RViz 自动查链），本节点一个旋转矩阵都不写。

单目无深度的处理（10.11 Q3 方案一）：
    所有目标假设在 fixed_depth（默认 1m）深的平面上。方向（左/右/上/下）准确，
    距离是假的——M3 只做可视化，不做测量。后续"检测到 person 就停"联动
    可改用 /scan 在对应方位角的射线距离替换固定 Z。

Marker 消失机制（10.11 Q5）：
    每个 Marker 带 lifetime=0.5s，检测频率 ~11Hz 会持续刷新保质期；
    目标走出画面后不再重发，RViz 倒计时结束自动删除，无需手动发 DELETE。
"""
import zlib

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


# 【我】类别配色板：颜色不随帧序变，而是由类别名确定性 hash 决定，
# 于是 person 在整个运行期间永远是同一种颜色，便于肉眼跟踪
COLOR_PALETTE = [
    (0.95, 0.30, 0.30),  # 红
    (0.30, 0.80, 0.30),  # 绿
    (0.30, 0.55, 0.95),  # 蓝
    (0.95, 0.75, 0.10),  # 黄
    (0.75, 0.30, 0.90),  # 紫
    (0.10, 0.80, 0.80),  # 青
    (0.95, 0.50, 0.10),  # 橙
    (0.90, 0.35, 0.60),  # 粉
]


def class_color(class_id):
    """【我】crc32 而不用内置 hash()：后者受 PYTHONHASHSEED 影响，每次启动颜色会变"""
    idx = zlib.crc32(class_id.encode("utf-8")) % len(COLOR_PALETTE)
    r, g, b = COLOR_PALETTE[idx]
    return ColorRGBA(r=r, g=g, b=b, a=1.0)


class DetectionMarker(Node):
    def __init__(self):
        super().__init__("detection_marker")

        # 【我】内参（10.11 Q2 路线A：FOV 反推的近似值，非标定结果）
        #   fx = (W/2) / tan(hfov/2)，800x600 + hfov≈60° → fx≈693
        #   cx/cy 主点近似图像中心；正式做法是 camera_calibration 棋盘格标定
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("marker_topic", "/detection_markers")
        self.declare_parameter("image_width", 800)
        self.declare_parameter("image_height", 600)
        self.declare_parameter("fx", 693.0)
        self.declare_parameter("fy", 693.0)
        self.declare_parameter("cx", 400.0)
        self.declare_parameter("cy", 300.0)
        # 【我】Q3：单目没深度，所有目标先钉在这个深度（米）上
        self.declare_parameter("fixed_depth", 1.0)
        self.declare_parameter("marker_lifetime", 0.5)
        self.declare_parameter("line_width", 0.01)
        self.declare_parameter("text_height", 0.08)

        detections_topic = self.get_parameter("detections_topic").value
        marker_topic = self.get_parameter("marker_topic").value
        self.img_w = int(self.get_parameter("image_width").value)
        self.img_h = int(self.get_parameter("image_height").value)
        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)
        self.fixed_depth = float(self.get_parameter("fixed_depth").value)
        self.lifetime = float(self.get_parameter("marker_lifetime").value)
        self.line_width = float(self.get_parameter("line_width").value)
        self.text_height = float(self.get_parameter("text_height").value)

        # 【我】检测结果低频小消息，沿用发布端的 RELIABLE（默认 QoS depth=10）
        self.sub = self.create_subscription(
            Detection2DArray, detections_topic, self.on_detections, 10
        )
        self.pub = self.create_publisher(MarkerArray, marker_topic, 10)

        self.get_logger().info(
            f"检测投影就绪: {detections_topic} → {marker_topic}; "
            f"内参 fx={self.fx:.1f} fy={self.fy:.1f} "
            f"cx={self.cx:.1f} cy={self.cy:.1f}, 固定深度 Z={self.fixed_depth}m"
        )

    def project(self, u, v, z):
        """【我】Q1 针孔公式：像素 (u,v) + 假设深度 Z → 光学系 3D 点"""
        return Point(
            x=(u - self.cx) * z / self.fx,
            y=(v - self.cy) * z / self.fy,  # 图像 y 向下 → 光学系 y 也向下，直接对应
            z=z,
        )

    def on_detections(self, msg):
        # 【我】header 整个抄检测消息：frame_id=camera_optical_link，stamp=图像拍摄时刻
        frame_id = msg.header.frame_id
        stamp = msg.header.stamp
        # 【我】rclpy Duration(seconds=) → builtin_interfaces/Duration 消息；
        # 注意别直接 new 消息类——它的字段名是 sec/nanosec，不是 nanoseconds
        lifetime = Duration(seconds=self.lifetime).to_msg()
        markers = MarkerArray()

        for i, det in enumerate(msg.detections):
            # 【我】像素框四角：center±half-size（图像系：u 向右，v 向下）
            u_c = det.bbox.center.position.x
            v_c = det.bbox.center.position.y
            half_w = det.bbox.size_x / 2.0
            half_h = det.bbox.size_y / 2.0
            u1, u2 = u_c - half_w, u_c + half_w
            v1, v2 = v_c - half_h, v_c + half_h

            # 【我】像素坐标超出配置分辨率说明内参分辨率对不上，节流告警避免刷屏
            if u2 > self.img_w or v2 > self.img_h:
                self.get_logger().warn(
                    f"检测框像素({u1:.0f},{v1:.0f})-({u2:.0f},{v2:.0f}) "
                    f"超出配置图像 {self.img_w}x{self.img_h}，请检查 image_width/height",
                    throttle_duration_sec=3.0,
                )

            z = self.fixed_depth
            color = class_color(self._class_name(det))

            # 【我】① 矩形框：5 点 LINE_STRIP（首尾相同点闭合）
            box = Marker()
            box.header.frame_id = frame_id
            box.header.stamp = stamp
            box.ns = "box"
            box.id = i
            box.type = Marker.LINE_STRIP
            box.action = Marker.ADD
            box.scale.x = self.line_width  # LINE_STRIP 只用 scale.x=线宽（米）
            box.color = color
            box.lifetime = lifetime
            box.points = [
                self.project(u1, v1, z),  # 左上
                self.project(u2, v1, z),  # 右上
                self.project(u2, v2, z),  # 右下
                self.project(u1, v2, z),  # 左下
                self.project(u1, v1, z),  # 回到左上闭合
            ]
            # pose 留默认（全 0）：点坐标已经是光学系下的绝对位置
            markers.markers.append(box)

            # 【我】② 文字标签：放在框上边中点，再沿光学系 -y（画面上方）偏移半行
            label = Marker()
            label.header.frame_id = frame_id
            label.header.stamp = stamp
            label.ns = "label"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING  # 始终面向观察者，不会因视角变扁
            label.action = Marker.ADD
            label.color = color
            label.lifetime = lifetime
            label.scale.z = self.text_height  # TEXT_VIEW_FACING 用 scale.z=字高（米）
            top_center = self.project(u_c, v1, z)
            label.pose.position = Point(
                x=top_center.x,
                y=top_center.y - self.text_height * 0.6,
                z=top_center.z,
            )
            label.text = self._label_text(det)
            markers.markers.append(label)

        # 【我】注意：目标全部消失时这里发出的是空数组，
        # 旧 Marker 靠 lifetime 自动过期，不需要显式发 DELETEALL
        self.pub.publish(markers)

    @staticmethod
    def _class_name(det):
        # 【我】humble 的 Detection2D：results 至少含 1 个假设，class_id 是字符串
        return det.results[0].hypothesis.class_id if det.results else "unknown"

    def _label_text(self, det):
        name = self._class_name(det)
        score = det.results[0].hypothesis.score if det.results else 0.0
        return f"{name} {score:.2f}"


def main(args=None):
    rclpy.init(args=args)
    node = DetectionMarker()
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
