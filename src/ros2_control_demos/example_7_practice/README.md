# 1. example 7 practice

- [1. example 7 practice](#1-example-7-practice)
  - [1.1. 创建 ros2 包：r6bot\_control\_system](#11-创建-ros2-包r6bot_control_system)
  - [1.2. 推荐的机器人开发顺序](#12-推荐的机器人开发顺序)
  - [1.3. 文件夹结构](#13-文件夹结构)
  - [1.4. 编译 r6bot\_control\_system](#14-编译-r6bot_control_system)
  - [1.5. 运行 r6bot\_control\_system](#15-运行-r6bot_control_system)
    - [1.5.1. 一键启动所有运行代码](#151-一键启动所有运行代码)

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
- [ ] 2. 基本硬件接口 - 实现最小功能的硬件接口
- [ ] 3. RViz配置 - 设置可视化，以便在开发过程中验证
- [ ] 4. 控制器实现 - 开发控制算法
- [ ] 5. 轨迹生成 - 创建测试命令或轨迹
- [ ] 6. 完善和优化 - 迭代改进所有组件

## 1.3. 文件夹结构

```bash
tree src/ros2_control_demos/example_7_practice/ -L 4
```

```bash
src/ros2_control_demos/example_7_practice/
├── CMakeLists.txt                    # CMake构建配置文件
├── controller                        # 控制器目录
│   ├── include                      # 控制器头文件目录
│   │   └── r6bot_controller.hpp     # R6Bot控制器头文件
│   ├── r6bot_controller.cpp         # R6Bot控制器实现文件
│   └── r6bot_controller.yaml        # R6Bot控制器配置文件
├── description                       # 机器人描述文件目录
│   ├── meshes                       # 机器人模型文件目录
│   │   ├── collision               # 碰撞模型目录
│   │   │   ├── link_0.stl         # 基座碰撞模型
│   │   │   ├── link_1.stl         # 第一关节碰撞模型
│   │   │   ├── link_2.stl         # 第二关节碰撞模型
│   │   │   ├── link_3.stl         # 第三关节碰撞模型
│   │   │   ├── link_4.stl         # 第四关节碰撞模型
│   │   │   ├── link_5.stl         # 第五关节碰撞模型
│   │   │   └── link_6.stl         # 第六关节碰撞模型
│   │   └── visual                  # 视觉模型目录
│   │       ├── link_0.dae         # 基座视觉模型
│   │       ├── link_1.dae         # 第一关节视觉模型
│   │       ├── link_2.dae         # 第二关节视觉模型
│   │       ├── link_3.dae         # 第三关节视觉模型
│   │       ├── link_4.dae         # 第四关节视觉模型
│   │       ├── link_5.dae         # 第五关节视觉模型
│   │       └── link_6.dae         # 第六关节视觉模型
│   ├── rviz                        # RViz配置目录
│   │   └── view_robot.rviz        # RViz配置文件
│   ├── srdf                        # SRDF文件目录
│   │   └── r6bot.srdf             # R6Bot机器人SRDF文件
│   └── urdf                        # URDF文件目录
│       ├── inc                     # URDF包含文件目录
│       │   └── create_link.xacro   # 创建机器人连杆的宏文件
│       └── r6bot_description.urdf.xacro  # R6Bot机器人URDF主文件
├── hardware                         # 硬件接口目录
│   ├── include                     # 硬件接口头文件目录
│   │   └── r6bot_hardware.hpp     # R6Bot硬件接口头文件
│   └── r6bot_hardware.cpp         # R6Bot硬件接口实现文件
├── launch                          # 启动文件目录
│   ├── r6bot_control_system.launch.py  # R6Bot控制系统启动文件
│   └── view_r6bot.launch.py       # R6Bot可视化启动文件
├── package.xml                     # ROS2包配置文件
├── r6bot_control_system_plugins.xml  # R6Bot插件描述文件
├── README.md                       # 项目说明文档
├── run.sh                          # 一键启动脚本
└── trajectory_generator            # 轨迹生成器目录
    └── main.cpp                    # 轨迹生成器主程序
```

## 1.4. 编译 r6bot_control_system

```bash
colcon build --packages-select r6bot_control_system
```

## 1.5. 运行 r6bot_control_system

### 1.5.1. 一键启动所有运行代码

```bash
./src/ros2_control_demos/example_7_practice/run.sh
```
