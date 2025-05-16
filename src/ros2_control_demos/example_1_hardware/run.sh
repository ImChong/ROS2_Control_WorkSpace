#!/bin/bash
set -e

# 清理
echo "===== 清理 build/ install/ log/ 目录 ====="
rm -rf build/ install/ log/

# 编译
echo "===== 编译 ====="
colcon build --packages-select ros2_control_demo_example_1_hardware

# 设置环境变量
echo "===== 设置环境变量 ====="
source install/setup.bash

# 启动
echo "===== 启动 ====="
ros2 launch ros2_control_demo_example_1_hardware rrbot.launch.py