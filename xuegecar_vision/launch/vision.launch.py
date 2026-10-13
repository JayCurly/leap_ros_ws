import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    vision_dir = get_package_share_directory("xuegecar_vision")
    camera_dir = get_package_share_directory("xuegecar_camera")

    # 【我】复用暑假写好的拉流节点（url 在 camera 包自己的 config yaml 里）
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_dir, "launch", "http_video_publisher.launch.py")
        )
    )

    # 【我】YOLO 检测节点，加载 vision 包自己的参数文件
    yolo_node = Node(
        package="xuegecar_vision",
        executable="yolo_detector",
        name="yolo_detector",
        output="screen",
        parameters=[os.path.join(vision_dir, "config", "yolo.yaml")],
    )

    # 【我】M3：检测框 → 3D MarkerArray 投影节点；
    # 同一个 yolo.yaml 里有 yolo_detector/detection_marker 两段，
    # ROS2 按节点名各取各的段，互不干扰
    marker_node = Node(
        package="xuegecar_vision",
        executable="detection_marker",
        name="detection_marker",
        output="screen",
        parameters=[os.path.join(vision_dir, "config", "yolo.yaml")],
    )

    return LaunchDescription([camera_launch, yolo_node, marker_node])
