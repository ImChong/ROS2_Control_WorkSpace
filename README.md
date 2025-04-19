# 1. ROS2 control 相关工作空间

- [1. ROS2 control 相关工作空间](#1-ros2-control-相关工作空间)
  - [1.1. 移除所有编译包](#11-移除所有编译包)
  - [1.2. ros2\_control\_demo\_example\_1](#12-ros2_control_demo_example_1)

## 1.1. 移除所有编译包

```bash
rm -rf build/ install/ log/
```

## 1.2. ros2_control_demo_example_1

编译 ros2_control_demo_example_1

```bash
colcon build --packages-select ros2_control_demo_example_1 ros2_control_demo_description
```

xacro 转换至 urdf 文件

```bash
xacro src/ros2_control_demos/example_1/description/urdf/rrbot.urdf.xacro > src/ros2_control_demos/example_1/description/urdf/rrbot.urdf
```
