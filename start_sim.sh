#!/bin/bash

# 启动仿真脚本

cd /home/zhan/train_robot-main/train_robot/ros2_ws

echo "========================================="
echo "启动仿真环境..."
echo "========================================="

# 源代码环境
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "环境已加载，启动 Gazebo..."

# 启动launch文件
ros2 launch robot_description gazebo_display.launch.py

echo "仿真已启动！"
