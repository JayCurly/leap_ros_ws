import time
import threading

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header


class HttpVideoPublisher(Node):
    def __init__(self):
        super().__init__("http_video_publisher")

        # 【我】declare_parameter：声明节点参数。运行时可被 launch/命令行覆盖，
        # 比如 ros2 run ... --ros-args -p url:=http://x.x.x.x:81/
        # 这里给的全是默认值；url 默认空串，L47 会强制检查
        self.declare_parameter("url", "")
        self.declare_parameter("topic", "/camera/image_raw/compressed")
        self.declare_parameter("raw_topic", "/camera/image_raw")
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("reconnect_delay", 2.0)
        self.declare_parameter("use_ffmpeg", True)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("publish_raw", False)

        # 【我】读取参数值：get_parameter 返回的是"参数对象"，
        # 要再 .get_parameter_value().xxx_value 取出实际类型
        self.url = self.get_parameter("url").get_parameter_value().string_value
        self.topic = self.get_parameter("topic").get_parameter_value().string_value
        self.raw_topic = self.get_parameter("raw_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.fps = self.get_parameter("fps").get_parameter_value().double_value
        self.reconnect_delay = (
            self.get_parameter("reconnect_delay").get_parameter_value().double_value
        )
        self.use_ffmpeg = (
            self.get_parameter("use_ffmpeg").get_parameter_value().bool_value
        )
        self.jpeg_quality = (
            self.get_parameter("jpeg_quality").get_parameter_value().integer_value
        )
        self.publish_raw = (
            self.get_parameter("publish_raw").get_parameter_value().bool_value
        )

        # 【我】url 是必填项：不传就直接抛异常让节点启动失败，早失败好过静默跑空
        if not self.url:
            raise ValueError("Parameter 'url' must be set in the camera YAML file.")
        self.jpeg_quality = max(1, min(100, self.jpeg_quality))  # 钳到合法范围 1-100

        # 【我】线程模型核心状态（Q2 的答案都在这几个变量上）：
        #   capture        当前视频捕获对象（cv2.VideoCapture）
        #   stop_event     线程停止信号（节点销毁时通知后台线程退出）
        #   frame_lock     锁：保护 latest_frame 不被两个线程同时读写
        #   latest_frame   后台线程刚读到的最新一帧（numpy 图像）
        #   latest_frame_seq    后台线程读帧计数（每读一帧 +1）
        #   published_frame_seq timer 已发布计数（用来跳过重复帧）
        self.capture = None
        self.stop_event = threading.Event()
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.latest_frame_seq = 0
        self.published_frame_seq = 0
        self.last_frame_time = 0.0
        self.frame_period = 1.0 / self.fps if self.fps > 0.0 else 0.0  # 发布间隔上限

        # 【我】Q3 答案在这里：视频用 KEEP_LAST + depth=1 + BEST_EFFORT
        #   depth=1：只留最新一帧，旧帧直接丢（视频宁可丢帧也不要积压延迟）
        #   BEST_EFFORT：不保证送达，UDP 丢包不重传——视频流丢一帧无感，
        #   重传反而会把延迟的雪球滚大。代价：订阅端 QoS 必须兼容（也要 BEST_EFFORT）
        video_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(CompressedImage, self.topic, video_qos)
        # 【我】publish_raw 开关：True 才额外发未压缩的 Image（调试/省事用，默认关省带宽）
        self.raw_publisher = (
            self.create_publisher(Image, self.raw_topic, video_qos)
            if self.publish_raw and self.raw_topic
            else None
        )
        # 【我】timer 周期 = min(5ms, 发布间隔)：5ms 一个 tick 检查"有没有新帧可发"，
        # 但真正的限速由 L122 的 frame_period 比较完成（timer tick 快≠发布快）
        timer_period = 0.005 if self.frame_period <= 0.0 else min(0.005, self.frame_period)
        self.timer = self.create_timer(timer_period, self.publish_next_frame)
        # 【我】Q2 关键：拉流循环放【后台线程】，daemon=True 表示主线程退出时它跟着死
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()

        self.get_logger().info(
            f"Publishing video stream {self.url} to {self.topic} as compressed jpeg "
            f"with frame_id={self.frame_id}; publish_raw={self.publish_raw}"
        )

    def open_capture(self):
        # 【我】打开/重连视频流的封装：先释放旧对象再新建，防止句柄泄漏
        if self.capture is not None:
            self.capture.release()
            self.capture = None

        backend = cv2.CAP_FFMPEG if self.use_ffmpeg else cv2.CAP_ANY
        capture = cv2.VideoCapture(self.url, backend)
        if not capture.isOpened():
            self.get_logger().warn(
                f"Failed to open video stream: {self.url}; retrying in "
                f"{self.reconnect_delay:.1f}s"
            )
            # 【我】注意：这里 sleep 阻塞的是【后台线程】，不影响主线程的 spin/timer
            # ——这正是 capture_loop 必须在后台线程的原因之一
            time.sleep(max(0.1, self.reconnect_delay))
            return False

        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 内部缓冲只留1帧，降低端到端延迟
        self.capture = capture
        self.get_logger().info(f"Connected to video stream: {self.url}")
        return True

    def capture_loop(self):
        # 【我】后台线程主循环：节点活着且没收到停止信号就一直拉帧
        while rclpy.ok() and not self.stop_event.is_set():
            if self.capture is None and not self.open_capture():
                continue

            ok, frame = self.capture.read()
            if not ok or frame is None:
                # 【我】断流自愈：释放 → 置空 → 等待 → 下轮循环重新 open_capture
                self.get_logger().warn(
                    f"Lost video stream: {self.url}; retrying in "
                    f"{self.reconnect_delay:.1f}s"
                )
                self.capture.release()
                self.capture = None
                self.stop_event.wait(max(0.1, self.reconnect_delay))
                continue

            # 【我】只干一件事：把最新帧存进共享区，seq +1
            # 不加锁的话 timer 线程可能读到半张图（竞态）
            with self.frame_lock:
                self.latest_frame = frame
                self.latest_frame_seq += 1

    def publish_next_frame(self):
        """【我】timer 回调：spin 里每 5ms 调用一次，负责限速+编码+发布

        为什么不在这里 capture.read()？（Q2 核心）
        read() 是阻塞 IO——网络流等下一帧可能卡几十 ms。
        timer 回调阻塞 = 整个 spin 的事件循环被卡住，
        节点里其他 timer/subscription 全部跟着饿死。
        所以拆分：后台线程专职阻塞拉帧，timer 回调只做"有就拿，没有就走"。
        """
        now = time.monotonic()
        # 【我】限速：距上次发布不足 frame_period 就直接返回（比如 fps=30 → 33ms 一帧）
        if self.frame_period > 0.0 and now - self.last_frame_time < self.frame_period:
            return

        with self.frame_lock:
            frame = self.latest_frame
            frame_seq = self.latest_frame_seq
        # 【我】Q2 两个 seq 的作用：frame_seq == published_frame_seq 说明
        # 这帧已经发过了（拉流 fps 低于发布 tick 时会反复命中），直接跳过，
        # 防止同一帧被重复发布刷爆订阅端
        if frame is None or frame_seq == self.published_frame_seq:
            return

        # 【我】Q4 的逆操作：numpy 图像 → jpeg 字节（下游用 imdecode 还原）
        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            self.get_logger().warn("Failed to encode frame as jpeg.")
            return

        msg = CompressedImage()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self.publisher.publish(msg)

        # 【我】可选的未压缩通道：编码(raw)比压缩版多一步 BGR→RGB 转换
        # （cv2 内部是 BGR，ROS Image 约定 rgb8）
        if self.raw_publisher is not None:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            raw_msg = Image()
            raw_msg.header = msg.header
            raw_msg.height = rgb_frame.shape[0]
            raw_msg.width = rgb_frame.shape[1]
            raw_msg.encoding = "rgb8"
            raw_msg.is_bigendian = False
            raw_msg.step = raw_msg.width * 3  # 每行字节数 = 宽 × 3通道
            raw_msg.data = rgb_frame.tobytes()
            self.raw_publisher.publish(raw_msg)

        self.published_frame_seq = frame_seq
        self.last_frame_time = now

    def destroy_node(self):
        # 【我】有序关停四步：①发停止信号 ②join 等后台线程退出（最多 2s）
        # ③释放摄像头句柄 ④才轮到父类销毁。顺序错了可能线程还在写已销毁的资源
        self.stop_event.set()
        if hasattr(self, "capture_thread") and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        super().destroy_node()


def main(args=None):
    # 【我】Q1 的 entry point 本体：setup.py 里 :main 指向的就是这个函数
    rclpy.init(args=args)
    node = HttpVideoPublisher()
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
