#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yolo_detector.py —— xuegecar_vision 包节点本体

数据流（JD 第3条"视觉算法接入 ROS2"的最小完整链路）：
    /camera/image_raw/compressed (CompressedImage, BEST_EFFORT)
        │  ① jpeg 字节 → numpy BGR（imdecode）
        ▼
    YOLOv8 推理（~50ms/帧, CPU）
        │  ② 遍历 results[0].boxes：xyxy → center/size（10.9 公式）
        ▼
    /detections (Detection2DArray, RELIABLE)          给机器消费
    /detections_debug/image_raw/compressed (BEST_EFFORT)  画框图，给人看

注意：推理直接在订阅回调里执行（单线程 executor，~50ms 阻塞）。
本节点只有这一个高频回调，且订阅 depth=1 会自动丢弃积压旧帧，
所以可接受；以后若增加多回调/多传感器，需改成像 xuegecar_camera
那样的"后台线程 + 最新帧共享"模型。
"""
import numpy as np
import cv2
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose
from ultralytics import YOLO


class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        # 【我】参数声明：模型路径/输入话题/置信度阈值/是否发画框调试图
        self.declare_parameter("model_path", "/home/jay/leap_ros/yolov8n.pt")
        self.declare_parameter("image_topic", "/camera/image_raw/compressed")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("publish_debug", True)

        model_path = self.get_parameter("model_path").value
        self.image_topic = self.get_parameter("image_topic").value
        self.conf = float(self.get_parameter("conf").value)
        self.publish_debug = bool(self.get_parameter("publish_debug").value)

        # 【我】参数热更新回调：ros2 param set 时即时生效，不必重启节点
        self.add_on_set_parameters_callback(self.on_set_parameters)

        # 【我】模型在节点启动时一次性加载（加载要几秒），之后每帧只做推理
        self.model = YOLO(model_path)

        # 【我】订阅 QoS 必须与上游 camera 发布者兼容：KEEP_LAST/depth1/BEST_EFFORT
        video_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.sub = self.create_subscription(
            CompressedImage, self.image_topic, self.on_image, video_qos
        )

        # 【我】检测结果：低频小消息、下游一帧不想丢 → 默认 RELIABLE + depth=10
        self.det_pub = self.create_publisher(Detection2DArray, "/detections", 10)
        # 【我】画框调试图：视频类消息，沿用 BEST_EFFORT
        self.debug_pub = (
            self.create_publisher(
                CompressedImage, "/detections_debug/image_raw/compressed", video_qos
            )
            if self.publish_debug
            else None
        )

        self.get_logger().info(
            f"YOLO 检测就绪: model={model_path}, conf={self.conf}, "
            f"订阅 {self.image_topic} → /detections"
        )

    def on_set_parameters(self, params):
        # 【我】只处理 conf：改完立刻影响下一帧推理的阈值
        for p in params:
            if p.name == "conf":
                self.conf = float(p.value)
                self.get_logger().info(f"conf 热更新 → {self.conf}")
        return SetParametersResult(successful=True)

    def on_image(self, msg):
        # 【我】① Q4 解码：jpeg 字节 → 一维 uint8 数组 → BGR 图像
        frame = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if frame is None:
            self.get_logger().warn("jpeg 解码失败，丢弃本帧", throttle_duration_sec=2.0)
            return

        # 【我】推理：verbose=False 关掉 ultralytics 每帧刷屏
        results = self.model(frame, conf=self.conf, verbose=False)
        result = results[0]

        # 【我】② 组装 Detection2DArray
        arr = Detection2DArray()
        # 【我】10.18 修复：stamp 用发布时刻而非拍摄时刻。
        # 拍摄时刻会把 CPU 推理延迟算进"年龄"，导致 safety_stop 新鲜度看门狗误判过期。
        # 检测结果的有效时刻 = 产出时刻；frame_id 仍沿用相机坐标系。
        arr.header.frame_id = msg.header.frame_id
        arr.header.stamp = self.get_clock().now().to_msg()

        debug_frame = frame.copy() if self.debug_pub is not None else None

        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cls_id = int(box.cls[0])
            score = float(box.conf[0])
            name = result.names[cls_id]  # int 索引 → COCO 类别名字符串

            det = Detection2D()
            det.header = msg.header
            # 【我】10.9 Q4 公式：两角 → 中心点+宽高
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)
            # 【我】Q3：YOLO 单类别，results 只填 1 个最佳假设
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = score
            det.results.append(hyp)
            arr.detections.append(det)

            if debug_frame is not None:
                self.draw_box(debug_frame, x1, y1, x2, y2, name, score)

        self.det_pub.publish(arr)

        # 【我】③ 发画框调试图（header 同样抄原图）
        if debug_frame is not None:
            ok, encoded = cv2.imencode(".jpg", debug_frame)
            if ok:
                dbg = CompressedImage()
                dbg.header = msg.header
                dbg.format = "jpeg"
                dbg.data = encoded.tobytes()
                self.debug_pub.publish(dbg)

    @staticmethod
    def draw_box(img, x1, y1, x2, y2, name, score):
        p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
        cv2.rectangle(img, p1, p2, (0, 255, 0), 2)
        label = f"{name} {score:.2f}"
        # 【我】标签底色块 + 文字，防止框贴边时文字看不清
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img, p1, (p1[0] + tw + 4, p1[1] - th - 6), (0, 255, 0), -1)
        cv2.putText(
            img, label, (p1[0] + 2, p1[1] - 4),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA,
        )


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
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
