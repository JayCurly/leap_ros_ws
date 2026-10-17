import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    vision_dir = get_package_share_directory("xuegecar_vision")

    # 【我】单独启动安全闸门；Nav2 需在 nav2.yaml 把 cmd_vel_topic 改成 /cmd_vel_raw
    safety_node = Node(
        package="xuegecar_vision",
        executable="safety_stop",
        name="safety_stop",
        output="screen",
        parameters=[os.path.join(vision_dir, "config", "safety_stop.yaml")],
    )

    return LaunchDescription([safety_node])
