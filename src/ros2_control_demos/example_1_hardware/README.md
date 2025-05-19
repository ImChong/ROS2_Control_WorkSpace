# 1. example 1 hardware

- [1. example 1 hardware](#1-example-1-hardware)
  - [1.1. 创建 ros2 包：ros2\_control\_demo\_example\_1\_hardware](#11-创建-ros2-包ros2_control_demo_example_1_hardware)
  - [1.2. 编译 ros2\_control\_demo\_example\_1\_hardware](#12-编译-ros2_control_demo_example_1_hardware)
  - [1.3. 运行 ros2\_control\_demo\_example\_1\_hardware](#13-运行-ros2_control_demo_example_1_hardware)
    - [1.3.1. view\_robot.launch.py 查看RRBot机器人URDF](#131-view_robotlaunchpy-查看rrbot机器人urdf)
    - [1.3.2. rrbot.launch.py 运行RRBot机器人硬件以及控制器](#132-rrbotlaunchpy-运行rrbot机器人硬件以及控制器)
    - [1.3.3. 一键启动所有运行代码](#133-一键启动所有运行代码)
  - [1.4. 查看机器人控制器相关信息](#14-查看机器人控制器相关信息)
  - [1.5. 发送位置测试指令](#15-发送位置测试指令)

此包的目的是：

- 使用rviz显示机器人URDF
- 调用rrbot机器人硬件
- 通过terminal与rrbot机器人硬件进行交互

## 1.1. 创建 ros2 包：ros2_control_demo_example_1_hardware

```bash
ros2 pkg create example_1_hardware --build-type ament_cmake --dependencies rclcpp
```

## 1.2. 编译 ros2_control_demo_example_1_hardware

```bash
colcon build --packages-select ros2_control_demo_example_1_hardware
```

## 1.3. 运行 ros2_control_demo_example_1_hardware

### 1.3.1. view_robot.launch.py 查看RRBot机器人URDF

```bash
source install/setup.bash
ros2 launch ros2_control_demo_example_1_hardware view_robot.launch.py
```

### 1.3.2. rrbot.launch.py 运行RRBot机器人硬件以及控制器

```bash
source install/setup.bash
ros2 launch ros2_control_demo_example_1_hardware rrbot.launch.py
```

### 1.3.3. 一键启动所有运行代码

```bash
./src/ros2_control_demos/example_1_hardware/run.sh
```

## 1.4. 查看机器人控制器相关信息

查看当前加载的控制器列表:

```bash
ros2 control list_controllers
ros2 control list_controllers --verbose
```

查看当前加载的硬件组件

```bash
ros2 control list_hardware_components
ros2 control list_hardware_components --verbose
```

查看硬件接口列表

```bash
ros2 control list_hardware_interfaces
```

图形界面方式查看

```bash
ros2 run rqt_controller_manager rqt_controller_manager
```

## 1.5. 发送位置测试指令

```bash
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
- 2.0
- -2.0"
```

```bash
ros2 topic pub --once /joint_trajectory_position_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{joint_names: ["joint1", "joint2"], points: [{positions: [2.0, -2.0], time_from_start: {sec: 1, nanosec: 0}}]}'
```
