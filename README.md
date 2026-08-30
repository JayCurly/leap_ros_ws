# leap_ros_ws

leap_ros 的 ROS 2 工作空间。

## 工作空间包

- `camsense_lidar`：串口雷达驱动。
- `leap_interfaces`：公共接口。
- `openslam_gmapping`：上游 gmapping 源码。
- `slam_gmapping`：gmapping 的 ROS 2 封装。
- `xuegecar_bringup`：底盘 bringup 节点和启动文件。
- `xuegecar_camera`：HTTP/MJPEG 摄像头流发布节点。
- `xuegecar_cartographer`：Cartographer 相关包。
- `xuegecar_description`：URDF、模型和 RViz 资源。
- `xuegecar_mavlink_bridge`：MAVLink 桥接包。
- `xuegecar_navigation2`：Nav2 配置和地图。
- `xuegeros_demo`：综合演示启动文件。

## 构建

```bash
cd ~/leap_ros_ws
colcon build
source install/setup.bash
```

只构建单个包：

```bash
colcon build --packages-select xuegecar_description
```

## 常用启动

- 底盘启动：

```bash
ros2 launch xuegecar_bringup xuegecar_bringup.launch.py
```

- 完整演示：

```bash
ros2 launch xuegeros_demo bringup_all.launch.py
```

- 机器人模型预览：

```bash
ros2 launch xuegecar_description display.launch.py
```

- 摄像头流发布：

```bash
ros2 launch xuegecar_camera http_video_publisher.launch.py
```

## 说明

- 当前使用的机器人描述文件是 `xuegecar_description/urdf/leap_ros.urdf`。
- 视觉资源位于 `xuegecar_description/meshes/`。
- 导航地图位于 `xuegecar_navigation2/maps/`。
