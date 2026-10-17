from glob import glob
import os

from setuptools import find_packages, setup

package_name = "xuegecar_vision"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jay",
    maintainer_email="jay@todo.todo",
    description="YOLOv8 object detection node for xuegecar.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 【我】可执行名 yolo_detector → ros2 run xuegecar_vision yolo_detector
            "yolo_detector = xuegecar_vision.yolo_detector:main",
            # 【我】M3：Detection2DArray → MarkerArray 投影节点
            "detection_marker = xuegecar_vision.detection_marker:main",
            # 【我】10.17：见人停安全闸门（/cmd_vel_raw → /cmd_vel）
            "safety_stop = xuegecar_vision.safety_stop:main",
        ],
    },
)
