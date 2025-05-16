# 1. example 1 hardware

- [1. example 1 hardware](#1-example-1-hardware)
  - [1.1. 创建 ros2 包：ros2\_control\_demos\_example\_1\_hardware](#11-创建-ros2-包ros2_control_demos_example_1_hardware)

此包的目的是：

- 使用rviz显示机器人URDF
- 调用rrbot机器人硬件
- 通过terminal与rrbot机器人硬件进行交互

## 1.1. 创建 ros2 包：ros2_control_demos_example_1_hardware

```bash
ros2 pkg create example_1_hardware --build-type ament_cmake --dependencies rclcpp std_msgs
```
