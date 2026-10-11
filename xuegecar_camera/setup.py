from glob import glob
import os

from setuptools import find_packages, setup

package_name = "xuegecar_camera"

# 【我】ament_python 包的构建脚本：不是编译，而是"把 Python 文件摆进 install 目录"
setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),  # 自动发现带 __init__.py 的子目录
    data_files=[
        # 【我】data_files = "把非代码文件也拷进 install/share/包名/"：
        # resource 标记文件（ament 索引靠它找到这个包）
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # package.xml 本体
        (f"share/{package_name}", ["package.xml"]),
        # launch/config 目录下的文件原样拷贝（FindPackageShare 才能找到）
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xuegeros",
    maintainer_email="xuegeros@todo.todo",
    description="ROS2 HTTP video stream publisher for xuegecar.",
    license="MIT",
    tests_require=["pytest"],
    # 【我】Q1 答案本体：console_scripts 三段的含义
    #   "可执行名 = 包名.模块文件名:入口函数名"
    #   colcon build 后 setuptools 在 install/.../lib/包名/ 下生成同名引导脚本，
    #   ros2 run xuegecar_camera http_video_publisher → 执行 shim → import 模块 → 调 main()
    entry_points={
        "console_scripts": [
            "http_video_publisher = xuegecar_camera.http_video_publisher:main",
        ],
    },
)
