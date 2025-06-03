# 1. example 7 practice (r6bot_control_system)

- [1. example 7 practice (r6bot\_control\_system)](#1-example-7-practice-r6bot_control_system)
  - [1.1. 创建 ros2 包：r6bot\_control\_system](#11-创建-ros2-包r6bot_control_system)
  - [1.2. 推荐的机器人开发顺序](#12-推荐的机器人开发顺序)
  - [1.3. xacro 转 URDF](#13-xacro-转-urdf)
  - [1.4. 文件夹结构](#14-文件夹结构)
  - [1.5. 编译 r6bot\_control\_system](#15-编译-r6bot_control_system)
  - [1.6. 运行 r6bot\_control\_system](#16-运行-r6bot_control_system)
    - [1.6.1. view\_r6bot.launch.py 查看 r6bot 机器人 URDF](#161-view_r6botlaunchpy-查看-r6bot-机器人-urdf)
    - [1.6.2. r6bot\_control\_system.launch.py 启动控制器](#162-r6bot_control_systemlaunchpy-启动控制器)
      - [1.6.2.1. forward\_position\_controller 位置控制器配置](#1621-forward_position_controller-位置控制器配置)
      - [1.6.2.2. forward\_position\_controller 交互指令](#1622-forward_position_controller-交互指令)
      - [1.6.2.3. forward\_velocity\_controller 速度控制器配置](#1623-forward_velocity_controller-速度控制器配置)
      - [1.6.2.4. forward\_velocity\_controller 交互指令](#1624-forward_velocity_controller-交互指令)
    - [1.6.3. 一键启动](#163-一键启动)

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

## 1.2. 推荐的机器人开发顺序

- [x] 1. URDF/机器人描述 - 首先定义机器人结构和接口
- [x] 2. RViz配置 - 设置可视化，以便在开发过程中验证
  - 所需最少文件：
    - 机器人描述文件：```r6bot.urdf.xacro```
    - rviz 配置文件：```view_r6bot.rviz```
    - launch 文件：```view_r6bot.launch.py```
- [x] 3. 基本硬件接口 - 实现最小功能的硬件接口
  - 所需最少文件：
    - 机器人描述文件：```r6bot.urdf.xacro```
    - rviz 配置文件：```view_r6bot.rviz```
    - 硬件接口文件：```r6bot_hardware.hpp```
    - 硬件源文件：```r6bot_hardware.cpp```
    - yaml 文件：```r6bot_controller.yaml```
    - 插件配置文件：```r6bot_control_system_plugins.xml```
    - launch 文件：```r6bot_control_system.launch.py```
- [ ] 4. 控制器实现 - 开发控制器算法
  - 所需最少文件：
    - 机器人描述文件：```r6bot.urdf.xacro```
    - rviz 配置文件：```view_r6bot.rviz```
    - 硬件接口文件：```r6bot_hardware.hpp```
    - 硬件源文件：```r6bot_hardware.cpp```
    - 控制器接口文件：```r6bot_controller.hpp```
    - 控制器源文件：```r6bot_controller.cpp```
    - yaml 文件：```r6bot_controller.yaml```
    - 插件配置文件：```r6bot_control_system_plugins.xml```
    - launch 文件：```r6bot_control_system.launch.py```
- [ ] 5. 轨迹生成 - 创建测试命令或轨迹
- [ ] 6. 完善和优化 - 迭代改进所有组件

## 1.3. xacro 转 URDF

```bash
cd path/to/ros2_ctrl_ws
source install/setup.bash
xacro src/ros2_control_demos/example_7_practice/description/urdf/r6bot.urdf.xacro > src/ros2_control_demos/example_7_practice/description/urdf/r6bot.urdf
```

## 1.4. 文件夹结构

```bash
tree src/ros2_control_demos/example_7_practice/ -L 4
```

```bash
src/ros2_control_demos/example_7_practice/
├── CMakeLists.txt  # 编译文件
├── controller
│   ├── include
│   │   └── r6bot_controller.hpp    # 控制器头文件
│   ├── r6bot_controller.cpp        # 控制器源文件
│   └── r6bot_controller.yaml       # 控制器配置文件
├── description
│   ├── meshes                     # 模型文件
│   │   ├── collision              # 碰撞文件
│   │   │   ├── link_0.stl
│   │   │   ├── link_1.stl
│   │   │   ├── link_2.stl
│   │   │   ├── link_3.stl
│   │   │   ├── link_4.stl
│   │   │   ├── link_5.stl
│   │   │   └── link_6.stl
│   │   └── visual                 # 可视化文件
│   │       ├── link_0.dae
│   │       ├── link_1.dae
│   │       ├── link_2.dae
│   │       ├── link_3.dae
│   │       ├── link_4.dae
│   │       ├── link_5.dae
│   │       └── link_6.dae
│   ├── rviz
│   │   └── view_r6bot.rviz        # rviz 配置文件
│   ├── srdf
│   │   └── r6bot.srdf             # srdf 文件
│   └── urdf
│       ├── inc
│       │   └── create_link.xacro
│       ├── r6bot_description.urdf.xacro
│       ├── r6bot.ros2_control.xacro
│       ├── r6bot.urdf               # urdf 文件
│       └── r6bot.urdf.xacro         # xacro 文件
├── hardware
│   ├── include
│   │   └── r6bot_hardware.hpp      # 硬件头文件
│   └── r6bot_hardware.cpp         # 硬件源文件
├── launch
│   ├── r6bot_control_system.launch.py # 控制器启动文件
│   └── view_r6bot.launch.py           # joint_state_publisher_gui 启动文件
├── package.xml                      # 包配置文件
├── r6bot_control_system_plugins.xml  # 插件配置文件
├── README.md                        # 说明文件
├── run.sh                           # 运行脚本
└── trajectory_generator             # 轨迹生成器
    └── main.cpp                     # 轨迹生成器源文件
```

## 1.5. 编译 r6bot_control_system

```bash
cd path/to/ros2_ctrl_ws
colcon build --packages-select r6bot_control_system
```

## 1.6. 运行 r6bot_control_system

### 1.6.1. view_r6bot.launch.py 查看 r6bot 机器人 URDF

```bash
cd path/to/ros2_ctrl_ws
source install/setup.bash
ros2 launch r6bot_control_system view_r6bot.launch.py
```

通过 joint_state_publisher_gui 可以控制 r6bot 机器人关节角度。

### 1.6.2. r6bot_control_system.launch.py 启动控制器

```bash
cd path/to/ros2_ctrl_ws
source install/setup.bash
ros2 launch r6bot_control_system r6bot_control_system.launch.py
```

#### 1.6.2.1. forward_position_controller 位置控制器配置

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    forward_position_controller:
      type: forward_command_controller/ForwardCommandController

forward_position_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
    interface_name: position
```

#### 1.6.2.2. forward_position_controller 交互指令

```bash
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

#### 1.6.2.3. forward_velocity_controller 速度控制器配置

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    forward_velocity_controller:
      type: forward_command_controller/ForwardCommandController

forward_velocity_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
    interface_name: velocity
```

#### 1.6.2.4. forward_velocity_controller 交互指令

```bash
ros2 topic pub --once /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

### 1.6.3. 一键启动

```bash
cd path/to/ros2_ctrl_ws
./src/ros2_control_demos/example_7_practice/run.sh
```
