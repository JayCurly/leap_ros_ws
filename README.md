# leap_ros_ws

基于开源小车平台的自主导航与视觉联动安全车 ROS 2 工作空间。

打通 **ESP32 固件 → micro-ROS → ROS 2 Humble → Nav2 → YOLOv8** 全链路，
完成实车集成、调试与验证。所有方案均经实车验证。

## 核心成果

- **运动控制**：精读差速运动控制与 PID 模块源码，用阶跃响应法整定速度环
  `kp=0.7 / ki=0.5 / kd=0`，上升时间 **140 ms**、零振荡；完成参数 NVS 持久化，
  并用冻结策略抗积分饱和。
- **导航集成**：基于 Nav2 完成实车集成（planner → controller → behavior →
  velocity_smoother），定位多节点争抢 `/cmd_vel` 导致的导航抽搐，
  用 launch 层 remap 收敛为安全闸门单发布者，路径跟踪稳定。
- **视觉安全闸门**：将 YOLOv8n 集成到 ROS 2，CPU 推理 **50 ms/帧（11–14 Hz）**；
  设计 `safety_stop` 闸门，检测框面积 ≥ 10% 且超声 < 0.6 m 双条件触发，
  3 帧防抖、10 帧迟滞恢复，实车验证见人停车。
- **多传感器融合**：定位航向漂移根因——固件将 IMU 协方差标为不可用致融合器
  静默丢弃该路数据；实现运行时零偏标定与 EKF 重构（仅融 `vx` + 去偏 `vyaw`），
  闭环航向误差 **15° → 0.9°**。

## 技术栈

| 类别 | 技术 |
|------|------|
| 机器人框架 | ROS 2 Humble、Nav2、micro-ROS |
| 建图定位 | Cartographer、GMapping、robot_localization (EKF) |
| 视觉 | YOLOv8、OpenCV、USB/MJPEG 摄像头 |
| 嵌入式 | ESP32、FreeRTOS、PID、UART/SPI/I2C |
| 传感器 | 激光雷达（欢创 Camsense）、IMU、超声波 |
| 工具 | colcon、rviz2、rqt、ros2 bag、Docker |

## 系统架构

```
┌─────────────┐  micro-ROS   ┌──────────────────────┐
│  ESP32 固件  │ ───────────▶ │  ROS 2 Humble 上位机   │
│ (C++, FreeRTOS)│ ◀─────────── │  ┌────────────────┐  │
└─────────────┘   /cmd_vel    │  │ Nav2 (导航)      │  │
                              │  └────────────────┘  │
                              │  ┌────────────────┐  │
    激光雷达 ───────────────▶ │  │ SLAM / EKF 融合  │  │
    IMU ───────────────────▶ │  └────────────────┘  │
    超声波 ─────────────────▶ │  ┌────────────────┐  │
                              │  │ safety_stop 闸门 │──┼──▶ /cmd_vel
    摄像头 ──YOLOv8────────▶ │  └────────────────┘  │
                              └──────────────────────┘
```

## 工作空间包

| 包 | 说明 |
|----|------|
| `camsense_lidar` | 欢创激光雷达串口驱动 |
| `leap_interfaces` | 自定义消息与服务接口 |
| `openslam_gmapping` | 上游 GMapping 源码 |
| `slam_gmapping` | GMapping 的 ROS 2 封装 |
| `xuegecar_bringup` | 底盘 bringup 启动文件与运行节点 |
| `xuegecar_camera` | HTTP/MJPEG 摄像头流发布节点 |
| `xuegecar_cartographer` | Cartographer 建图配置 |
| `xuegecar_description` | URDF 模型与 RViz 资源 |
| `xuegecar_mavlink_bridge` | MAVLink 桥接（无人机协同） |
| `xuegecar_navigation2` | Nav2 配置、地图、EKF 参数 |
| `xuegecar_vision` | YOLOv8 检测节点 + safety_stop 安全闸门 |
| `xuegeros_demo` | 综合演示启动文件 |

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+
- ESP-IDF（固件开发，可选）

## 构建

```bash
cd ~/leap_ros_ws
colcon build
source install/setup.bash
```

只构建单个包：

```bash
colcon build --packages-select xuegecar_vision
```

## 运行

**1. 启动底盘与传感器**

```bash
ros2 launch xuegecar_bringup xuegecar_bringup.launch.py
```

**2. 启动视觉检测与安全闸门**

```bash
ros2 launch xuegecar_vision safety_stop.launch.py
```

**3. 启动导航**

```bash
ros2 launch xuegecar_navigation2 navigation2.launch.py
```

## 关键文件

- EKF 配置：`xuegecar_navigation2/config/ekf_improved.yaml`
- 安全闸门参数：`xuegecar_vision/config/safety_stop.yaml`
- 机器人模型：`xuegecar_description/urdf/leap_ros.urdf`
- 导航地图：`xuegecar_navigation2/maps/`

## 演示

> 🎬 实车演示视频：待补充
> 📷 实物照片：待补充

## 作者

汪勇 — 机器人工程专业本科生，ROS 2 开发者。
