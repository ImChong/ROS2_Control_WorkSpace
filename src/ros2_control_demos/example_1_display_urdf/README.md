# 1. example 1 display urdf

- [1. example 1 display urdf](#1-example-1-display-urdf)
  - [1.1. 编译 ros2\_control\_demos\_example\_1\_display\_urdf](#11-编译-ros2_control_demos_example_1_display_urdf)
  - [运行 ros2\_control\_demos\_example\_1\_display\_urdf](#运行-ros2_control_demos_example_1_display_urdf)

此包的目的是:

- 练习使用rviz显示机器人URDF
- 通过joint_state_publisher_gui移动机器人的可动关节

## 1.1. 编译 ros2_control_demos_example_1_display_urdf

```bash
colcon build --packages-select ros2_control_demos_example_1_display_urdf --symlink-install
```

## 运行 ros2_control_demos_example_1_display_urdf

```bash
source install/setup.bash
ros2 launch ros2_control_demos_example_1_display_urdf view_robot.launch.py
```
