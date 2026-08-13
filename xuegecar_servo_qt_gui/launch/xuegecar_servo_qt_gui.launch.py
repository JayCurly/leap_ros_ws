from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="xuegecar_servo_qt_gui",
                executable="xuegecar_servo_qt_gui",
                name="xuegecar_servo_qt_gui",
                output="screen",
                parameters=[{"servo_topic": "/servo_cmd"}],
            )
        ]
    )
