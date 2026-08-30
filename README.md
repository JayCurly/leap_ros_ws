# leap_ros_ws

ROS 2 workspace for the xuegecar / leap_ros robot stack.

## Workspace packages

- `camsense_lidar`: serial lidar driver.
- `leap_interfaces`: shared interfaces.
- `openslam_gmapping`: upstream gmapping sources.
- `slam_gmapping`: ROS 2 wrapper for gmapping.
- `xuegecar_bringup`: robot bringup node and launch files.
- `xuegecar_camera`: HTTP/MJPEG camera stream publisher.
- `xuegecar_cartographer`: Cartographer-related package.
- `xuegecar_description`: URDF, meshes, and RViz assets.
- `xuegecar_mavlink_bridge`: MAVLink bridge package.
- `xuegecar_navigation2`: Nav2 configs and maps.
- `xuegeros_demo`: combined demo launch files.

## Build

```bash
cd ~/leap_ros_ws
colcon build
source install/setup.bash
```

Build a single package if needed:

```bash
colcon build --packages-select xuegecar_description
```

## Main launch files

- Robot bringup:

```bash
ros2 launch xuegecar_bringup xuegecar_bringup.launch.py
```

- Full demo:

```bash
ros2 launch xuegeros_demo bringup_all.launch.py
```

- Robot model preview:

```bash
ros2 launch xuegecar_description display.launch.py
```

- Camera stream node:

```bash
ros2 launch xuegecar_camera http_video_publisher.launch.py
```

## Notes

- The active robot description is `xuegecar_description/urdf/leap_ros.urdf`.
- Visual assets are stored in `xuegecar_description/meshes/`.
- Navigation maps are stored in `xuegecar_navigation2/maps/`.
