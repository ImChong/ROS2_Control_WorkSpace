# 1. ros2_control_demo_example_7

- [1. ros2\_control\_demo\_example\_7](#1-ros2_control_demo_example_7)
  - [1.1. 编译 ros2\_control\_demo\_example\_7](#11-编译-ros2_control_demo_example_7)
  - [1.2. 推荐的机器人开发顺序](#12-推荐的机器人开发顺序)
  - [1.3. 运行 ros2\_control\_demo\_example\_7](#13-运行-ros2_control_demo_example_7)
    - [1.3.1. view\_robot.launch.py 查看 r6bot 机器人 URDF](#131-view_robotlaunchpy-查看-r6bot-机器人-urdf)

A full tutorial for a 6 DOF robot for intermediate ROS 2 users.

It consists of the following:

- bringup: launch files and ros2_controller configuration
- controller: a controller for the 6-DOF robot
- description: the 6-DOF robot description
- hardware: ros2_control hardware interface
- reference_generator: A KDL-based reference generator for a fixed trajectory

Find the documentation in [doc/userdoc.rst](doc/userdoc.rst) or on [control.ros.org](https://control.ros.org/master/doc/ros2_control_demos/example_7/doc/userdoc.html).

## 1.1. 编译 ros2_control_demo_example_7

```bash
rm -rf build/ install/ log/
colcon build --packages-select ros2_control_demo_example_7 ros2_control_demo_description
```

## 1.2. 推荐的机器人开发顺序

1. URDF/机器人描述 - 首先定义机器人结构和接口
2. 基本硬件接口 - 实现最小功能的硬件接口
3. RViz配置 - 设置可视化，以便在开发过程中验证
4. 控制器实现 - 开发控制算法
5. 轨迹生成 - 创建测试命令或轨迹
6. 完善和优化 - 迭代改进所有组件

## 1.3. 运行 ros2_control_demo_example_7

### 1.3.1. view_robot.launch.py 查看 r6bot 机器人 URDF

```bash
source install/setup.bash
ros2 launch ros2_control_demo_example_7 view_r6bot.launch.py
```
