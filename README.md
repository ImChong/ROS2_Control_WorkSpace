# 1. ROS2 control 相关工作空间

- [1. ROS2 control 相关工作空间](#1-ros2-control-相关工作空间)
  - [1.1. 移除所有编译包](#11-移除所有编译包)
  - [1.2. 编译案例1](#12-编译案例1)

## 1.1. 移除所有编译包

```bash
rm -rf build/ install/ log/
```

## 1.2. 编译案例1

```bash
colcon build --packages-select ros2_control_demo_example_1 ros2_control_demo_description
```
