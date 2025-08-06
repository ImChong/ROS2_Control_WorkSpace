# 1. ros2_urdf_viewer

- [1. ros2\_urdf\_viewer](#1-ros2_urdf_viewer)
  - [1.1. 编译 ros2\_urdf\_viewer](#11-编译-ros2_urdf_viewer)
  - [1.2. 运行 ros2\_urdf\_viewer](#12-运行-ros2_urdf_viewer)
  - [一条指令运行 ros2\_urdf\_viewer](#一条指令运行-ros2_urdf_viewer)

此包的目的是:

- 练习使用rviz显示机器人URDF
- 通过joint_state_publisher_gui移动机器人的可动关节

## 1.1. 编译 ros2_urdf_viewer

```bash
colcon build --packages-select ros2_urdf_viewer --symlink-install
```

## 1.2. 运行 ros2_urdf_viewer

```bash
source install/setup.bash
ros2 launch ros2_urdf_viewer view_robot.launch.py
```

## 一条指令运行 ros2_urdf_viewer

```bash
colcon build --packages-select ros2_urdf_viewer --symlink-install && source install/setup.bash && ros2 launch ros2_urdf_viewer view_robot.launch.py
```
