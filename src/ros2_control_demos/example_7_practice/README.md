# 1. example 7 practice

- [1. example 7 practice](#1-example-7-practice)
  - [1.1. 创建 ros2 包：r6bot\_control\_system](#11-创建-ros2-包r6bot_control_system)
  - [1.2. 文件夹结构](#12-文件夹结构)
  - [1.3. 编译 r6bot\_control\_system](#13-编译-r6bot_control_system)
  - [1.4. 运行 r6bot\_control\_system](#14-运行-r6bot_control_system)
    - [1.4.1. 一键启动所有运行代码](#141-一键启动所有运行代码)

此包的目的是：

- 使用rviz显示6轴机器人URDF
- 创建6轴机器人硬件
- 创建6轴机器人控制
- 调用6轴机器人控制器进行运动控制

## 1.1. 创建 ros2 包：r6bot_control_system

```bash
cd src/ros2_control_demos/
ros2 pkg create example_7_practice --build-type ament_cmake --dependencies rclcpp
```

## 1.2. 文件夹结构

```bash
tree src/ros2_control_demos/example_7_practice/ -L 4
```

## 1.3. 编译 r6bot_control_system

```bash
colcon build --packages-select r6bot_control_system
```

## 1.4. 运行 r6bot_control_system

### 1.4.1. 一键启动所有运行代码

```bash
./src/ros2_control_demos/example_7_practice/run.sh
```
