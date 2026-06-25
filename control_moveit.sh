cd train_robot/ros2_ws/
python3 -m colcon build
source install/setup.sh
ros2 launch train_robot robot_control.launch.py | tee ros2_log.log

# moveit_controller 节点命令
# ros2 topic pub --once /control_order std_msgs/msg/String "{data: 'decouple'}" # 执行摘钩操作
