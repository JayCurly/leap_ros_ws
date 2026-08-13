# xuegecar_servo_qt_gui

Qt5 + ROS2 舵机调试页面，向 `/servo_cmd` 发布 `geometry_msgs/msg/Vector3`：

- `x`: 舵机通道 1..4
- `y`: 角度
- `z`: 固定为 0

## 构建

```bash
cd /home/chuyun/xuegeros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select xuegecar_servo_qt_gui --symlink-install
source install/setup.bash
```

## 启动

```bash
ros2 launch xuegecar_servo_qt_gui xuegecar_servo_qt_gui.launch.py
```

界面包含：

- 1-4 通道切换
- 角度输入
- `+/-` 微调按钮
- `回正` 一键归零
- `发送` 手动发布当前值
