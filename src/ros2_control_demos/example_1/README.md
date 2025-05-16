# 1. ros2_control_demo_example_1

*RRBot* - or ''Revolute-Revolute Manipulator Robot'' - a simple position controlled robot with one hardware interface. This example also demonstrates the switching between different controllers.

Find the documentation in [doc/userdoc.rst](doc/userdoc.rst) or on [control.ros.org](https://control.ros.org/master/doc/ros2_control_demos/example_1/doc/userdoc.html).

- [1. ros2\_control\_demo\_example\_1](#1-ros2_control_demo_example_1)
  - [1.1. 编译 ros2\_control\_demo\_example\_1](#11-编译-ros2_control_demo_example_1)
  - [1.2. xacro 转换至 urdf 文件](#12-xacro-转换至-urdf-文件)
  - [1.3. 发送测试指令](#13-发送测试指令)

## 1.1. 编译 ros2_control_demo_example_1

```bash
colcon build --packages-select ros2_control_demo_example_1 ros2_control_demo_description
```

## 1.2. xacro 转换至 urdf 文件

```bash
xacro src/ros2_control_demos/example_1/description/urdf/rrbot.urdf.xacro > src/ros2_control_demos/example_1/description/urdf/rrbot.urdf
```

## 1.3. 发送测试指令

```bash
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
- 2.0
- -2.0"
```

```bash
ros2 topic pub --once /joint_trajectory_position_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{joint_names: ["joint1", "joint2"], points: [{positions: [2.0, -2.0], time_from_start: {sec: 1, nanosec: 0}}]}'
```
