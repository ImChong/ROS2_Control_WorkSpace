# 1. ROS2 control 使用案例

- [1. ROS2 control 使用案例](#1-ros2-control-使用案例)
  - [1.1. 移除所有编译包](#11-移除所有编译包)
  - [1.2. ros2\_control\_demos](#12-ros2_control_demos)
    - [1.2.1. example\_1 - RRBot](#121-example_1---rrbot)
    - [1.2.2. example\_2 - DiffBot](#122-example_2---diffbot)
    - [1.2.3. example\_3 - 具有多个接口的 RRBot](#123-example_3---具有多个接口的-rrbot)
    - [1.2.4. example\_4 - 集成传感器的工业机器人](#124-example_4---集成传感器的工业机器人)
    - [1.2.5. example\_5 - 外接传感器的工业机器人](#125-example_5---外接传感器的工业机器人)
    - [1.2.6. example\_6 - 每个执行器单独通信的模块化机器人](#126-example_6---每个执行器单独通信的模块化机器人)
    - [1.2.7. example\_7 - 6 自由度机器人](#127-example_7---6-自由度机器人)
    - [1.2.8. example\_8 - 使用传动装置](#128-example_8---使用传动装置)
    - [1.2.9. example\_9 - Gazebo 经典版](#129-example_9---gazebo-经典版)
    - [1.2.10. example\_10 - GPIO 接口](#1210-example_10---gpio-接口)
    - [1.2.11. example\_11 - CarlikeBot](#1211-example_11---carlikebot)
    - [1.2.12. example\_12 - 控制器链](#1212-example_12---控制器链)
    - [1.2.13. example\_13 - 具有硬件生命周期管理的多机器人系统](#1213-example_13---具有硬件生命周期管理的多机器人系统)
    - [1.2.14. example\_14 - 具有不提供状态的执行器和附加传感器的模块化机器人](#1214-example_14---具有不提供状态的执行器和附加传感器的模块化机器人)
    - [1.2.15. example\_15 - 使用多个控制器管理器](#1215-example_15---使用多个控制器管理器)
  - [1.3. gazebo\_ros2\_control](#13-gazebo_ros2_control)
    - [1.3.1. cart\_example (gazebo)](#131-cart_example-gazebo)
    - [1.3.2. pendulum\_example (gazebo)](#132-pendulum_example-gazebo)
  - [1.4. gz\_ros2\_control](#14-gz_ros2_control)
    - [1.4.1. cart\_example (gz)](#141-cart_example-gz)
    - [1.4.2. pendulum\_example (gz)](#142-pendulum_example-gz)

## 1.1. 移除所有编译包

```bash
rm -rf build/ install/ log/
```

## 1.2. ros2_control_demos

### 1.2.1. example_1 - RRBot

- [x] [exmaple_1 [official]](/src/ros2_control_demos/example_1/README.md)
- [x] [example_1_display_urdf](/src/ros2_control_demos/example_1_display_urdf/README.md)
- [x] [example_1_hardware](/src/ros2_control_demos/example_1_hardware/README.md)

### 1.2.2. example_2 - DiffBot

- [ ] [example_2 [official]](/src/ros2_control_demos/example_2/README.md)

### 1.2.3. example_3 - 具有多个接口的 RRBot

- [ ] [example_3 [official]](/src/ros2_control_demos/example_3/README.md)

### 1.2.4. example_4 - 集成传感器的工业机器人

- [ ] [example_4 [official]](/src/ros2_control_demos/example_4/README.md)

### 1.2.5. example_5 - 外接传感器的工业机器人

- [ ] [example_5 [official]](/src/ros2_control_demos/example_5/README.md)

### 1.2.6. example_6 - 每个执行器单独通信的模块化机器人

- [ ] [example_6 [official]](/src/ros2_control_demos/example_6/README.md)

### 1.2.7. example_7 - 6 自由度机器人

- [ ] [example_7 [official]](/src/ros2_control_demos/example_7/README.md)

### 1.2.8. example_8 - 使用传动装置

- [ ] [example_8 [official]](/src/ros2_control_demos/example_8/README.md)

### 1.2.9. example_9 - Gazebo 经典版

- [ ] [example_9 [official]](/src/ros2_control_demos/example_9/README.md)

### 1.2.10. example_10 - GPIO 接口

- [ ] [example_10 [official]](/src/ros2_control_demos/example_10/README.md)

### 1.2.11. example_11 - CarlikeBot

- [ ] [example_11 [official]](/src/ros2_control_demos/example_11/README.md)

### 1.2.12. example_12 - 控制器链

- [ ] [example_12 [official]](/src/ros2_control_demos/example_12/README.md)

### 1.2.13. example_13 - 具有硬件生命周期管理的多机器人系统

- [ ] [example_13 [official]](/src/ros2_control_demos/example_13/README.md)

### 1.2.14. example_14 - 具有不提供状态的执行器和附加传感器的模块化机器人

- [ ] [example_14 [official]](/src/ros2_control_demos/example_14/README.md)

### 1.2.15. example_15 - 使用多个控制器管理器

- [ ] [example_15 [official]](/src/ros2_control_demos/example_15/README.md)

## 1.3. gazebo_ros2_control

### 1.3.1. cart_example (gazebo)

- [ ] [cart_example_position](/src/gazebo_ros2_control/gazebo_ros2_control_demos/launch/cart_example_effort.launch.py)
- [ ] [cart_example_velocity](/src/gazebo_ros2_control/gazebo_ros2_control_demos/launch/cart_example_velocity.launch.py)
- [ ] [cart_example_effort](/src/gazebo_ros2_control/gazebo_ros2_control_demos/launch/cart_example_effort.launch.py)

### 1.3.2. pendulum_example (gazebo)

- [ ] [pendulum_example_position](/src/gazebo_ros2_control/gazebo_ros2_control_demos/launch/pendulum_example_position.launch.py)
- [ ] [pendulum_example_effort](/src/gazebo_ros2_control/gazebo_ros2_control_demos/launch/pendulum_example_effort.launch.py)

## 1.4. gz_ros2_control

### 1.4.1. cart_example (gz)

- [ ] [cart_example_position](/src/gz_ros2_control/gz_ros2_control_demos/launch/cart_example_position.launch.py)
- [ ] [cart_example_velocity](/src/gz_ros2_control/gz_ros2_control_demos/launch/cart_example_velocity.launch.py)
- [ ] [cart_example_effort](/src/gz_ros2_control/gz_ros2_control_demos/launch/cart_example_effort.launch.py)

### 1.4.2. pendulum_example (gz)

- [ ] [pendulum_example_position](/src/gz_ros2_control/gz_ros2_control_demos/launch/pendulum_example_position.launch.py)
- [ ] [pendulum_example_effort](/src/gz_ros2_control/gz_ros2_control_demos/launch/pendulum_example_effort.launch.py)
