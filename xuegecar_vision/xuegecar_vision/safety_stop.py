#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""safety_stop.py —— M4「见人停」安全闸门节点（10.17）

【我】拓扑（详见 week6.md 10.16，固件零改动）：
    Nav2 controller_server ──/cmd_vel_raw──┐
    teleop(可选) ───────────/cmd_vel_raw──┤
                                           ├─▶ safety_stop ──/cmd_vel──▶ leap_low_driver(车)
    /detections (YOLO) ───────────────────┤        ▲ /cmd_vel 的唯一发布者
    /ultrasonic (固件) ───────────────────┘

为什么是「串联闸门」而不是「也往 /cmd_vel 发零速」：
    ROS2 话题允许多发布者，固件每收到一条就 xQueueOverwrite 最新命令；
    两个发布者会交错覆盖 → 车抽搐。改道后 /cmd_vel 只有本节点一个发布者。

判定逻辑（双条件 AND，防误触发）：
    连续 trigger_frames 帧，每帧都满足
      ① 检测里存在 class_id=="person" 且框面积占画面 ≥ area_ratio 阈值
      ② 超声波 range < dist_threshold（NAN/收不到 = 无障碍，不误触发）
    → STOPPED；恢复需连续 recover_frames 帧安全（迟滞，防边界反复启停）

输出：10Hz 定时闸门
    安全态 → 转发最新 /cmd_vel_raw（超过 0.5s 没收到新命令则发零，兼做上位机看门狗）
    停止态 → 持续发零速（固件无超时看门狗，必须持续占住出口）
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from vision_msgs.msg import Detection2DArray


class SafetyStop(Node):
    def __init__(self):
        super().__init__("safety_stop")

        # 【我】话题名参数：改道方案里 Nav2 出口改成 /cmd_vel_raw，固件入口仍是 /cmd_vel
        self.declare_parameter("raw_cmd_topic", "/cmd_vel_raw")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("ultrasonic_topic", "/ultrasonic")
        # 【我】判定参数（初值见 week6.md 10.16，实车再调）
        self.declare_parameter("target_class", "person")
        self.declare_parameter("image_width", 800)
        self.declare_parameter("image_height", 600)
        self.declare_parameter("area_ratio", 0.10)       # 框面积/画面面积
        self.declare_parameter("dist_threshold", 0.6)   # 米
        self.declare_parameter("trigger_frames", 3)     # ≈0.3s @10Hz
        self.declare_parameter("recover_frames", 10)    # ≈1s @10Hz
        self.declare_parameter("gate_hz", 10.0)
        self.declare_parameter("cmd_timeout", 0.5)      # 原始速度超时秒数
        self.declare_parameter("sensor_timeout", 0.5)   # 检测/超声消息新鲜度超时秒数

        self.target_class = self.get_parameter("target_class").value
        self.image_area = (
            float(self.get_parameter("image_width").value)
            * float(self.get_parameter("image_height").value
        ))
        self.area_ratio = float(self.get_parameter("area_ratio").value)
        self.dist_threshold = float(self.get_parameter("dist_threshold").value)
        self.trigger_frames = int(self.get_parameter("trigger_frames").value)
        self.recover_frames = int(self.get_parameter("recover_frames").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.sensor_timeout = float(self.get_parameter("sensor_timeout").value)

        # 【我】输入①：Nav2/teleop 的原始速度（默认 QoS 即可，与 controller 对齐）
        self.raw_sub = self.create_subscription(
            Twist, self.get_parameter("raw_cmd_topic").value,
            self.on_raw_cmd, 10,
        )
        # 【我】输入②：YOLO 检测（低频小消息，发布方用 RELIABLE depth10）
        self.det_sub = self.create_subscription(
            Detection2DArray, self.get_parameter("detections_topic").value,
            self.on_detections, 10,
        )
        # 【我】输入③：超声波（传感器话题，必须用 sensor_data=BEST_EFFORT）
        self.ultra_sub = self.create_subscription(
            Range, self.get_parameter("ultrasonic_topic").value,
            self.on_ultrasonic, qos_profile_sensor_data,
        )
        # 【我】唯一出口：给固件的安全速度
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_topic").value, 10
        )

        # 【我】闸门内部状态
        self.latest_raw = None                 # 缓存最新原始速度
        self.latest_raw_time = self.get_clock().now()
        self.person_close = False              # 最新一帧：有足够近（面积够大）的目标类
        self.ultra_close = False               # 最新一帧：超声波在阈值内（NAN=False）
        self.det_fresh = False                 # /detections 新鲜度看门狗
        self.ultra_fresh = False               # /ultrasonic 新鲜度看门狗
        self.latest_det_time = self.get_clock().now()
        self.latest_ultra_time = self.get_clock().now()
        self.stopped = False                   # 状态机：是否处于停车态
        self.danger_count = 0                  # 连续危险帧
        self.safe_count = 0                    # 连续安全帧

        gate_period = 1.0 / float(self.get_parameter("gate_hz").value)
        self.timer = self.create_timer(gate_period, self.gate)

        self.get_logger().info(
            f"safety_stop 就绪: {self.get_parameter('raw_cmd_topic').value} → "
            f"{self.get_parameter('cmd_topic').value}; 目标={self.target_class}, "
            f"面积阈值={self.area_ratio}, 距离阈值={self.dist_threshold}m, "
            f"防抖={self.trigger_frames}帧/恢复={self.recover_frames}帧"
        )

    # ---------- 输入回调：只更新状态，不做决策（决策统一在 10Hz 定时器）----------

    def on_raw_cmd(self, msg):
        self.latest_raw = msg
        self.latest_raw_time = self.get_clock().now()

    def _stamp_age(self, stamp):
        # 【我】消息头 stamp 年龄（秒）；stamp=0（伪造/异常消息）直接返回无穷大
        if stamp.sec == 0 and stamp.nanosec == 0:
            return float("inf")
        return (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds * 1e-9

    def on_detections(self, msg):
        age = self._stamp_age(msg.header.stamp)
        if age > self.sensor_timeout:
            # 【我】stamp 缺失/过老：不认这帧，person_close 保持上一有效值但标记不新鲜
            self.det_fresh = False
            self.get_logger().warn(
                f"/detections 不新鲜（stamp 年龄={age:.2f}s），本路暂不参与判定",
                throttle_duration_sec=2.0,
            )
            return
        self.det_fresh = True
        self.latest_det_time = self.get_clock().now()
        # 【我】最新帧是否存在「目标类 + 面积占比达标」的框
        self.person_close = any(
            h.hypothesis.class_id == self.target_class
            and (d.bbox.size_x * d.bbox.size_y) / self.image_area >= self.area_ratio
            for d in msg.detections
            for h in d.results
        )

    def on_ultrasonic(self, msg):
        age = self._stamp_age(msg.header.stamp)
        if age > self.sensor_timeout:
            self.ultra_fresh = False
            self.get_logger().warn(
                f"/ultrasonic 不新鲜（stamp 年龄={age:.2f}s），本路暂不参与判定",
                throttle_duration_sec=2.0,
            )
            return
        self.ultra_fresh = True
        self.latest_ultra_time = self.get_clock().now()
        # 【我】NAN（测距失败）视为无障碍；注意必须 math.isnan，NAN != NAN
        r = msg.range
        self.ultra_close = (not math.isnan(r)) and r < self.dist_threshold

    # ---------- 10Hz 定时闸门：状态机 + 出口发布 ----------

    def gate(self):
        # 【我】看门狗：按到达时间再兜底一次（发布者死亡时 stamp 检查不会再触发回调）
        now = self.get_clock().now()
        if (now - self.latest_det_time).nanoseconds * 1e-9 > self.sensor_timeout:
            self.det_fresh = False
        if (now - self.latest_ultra_time).nanoseconds * 1e-9 > self.sensor_timeout:
            self.ultra_fresh = False

        # 【我】双条件 AND，且两路输入都必须新鲜，缺一不触发（故障安全偏向不误停）
        frame_danger = (
            self.det_fresh and self.ultra_fresh
            and self.person_close and self.ultra_close
        )

        if not self.stopped:
            # 【我】安全态：连续 N 帧双条件命中才进停车态
            if frame_danger:
                self.danger_count += 1
                self.safe_count = 0
                if self.danger_count >= self.trigger_frames:
                    self.stopped = True
                    self.get_logger().warn(
                        "⚠️ 触发停车：person 近距离 + 超声波确认，持续发零速"
                    )
            else:
                self.danger_count = 0
        else:
            # 【我】停车态：连续 M 帧双条件都解除才恢复（迟滞）
            if frame_danger:
                self.safe_count = 0
            else:
                self.safe_count += 1
                if self.safe_count >= self.recover_frames:
                    self.stopped = False
                    self.danger_count = 0
                    self.get_logger().info("✅ 恢复通行：危险解除，转发原始速度")

        out = Twist()
        if not self.stopped and self.latest_raw is not None:
            age = (self.get_clock().now() - self.latest_raw_time).nanoseconds * 1e-9
            # 【我】安全但原始速度超时（上游挂了/没在发）→ 发零，兼做上位机看门狗
            if age <= self.cmd_timeout:
                out = self.latest_raw

        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
