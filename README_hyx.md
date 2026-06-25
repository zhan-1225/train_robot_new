## usd文件修改
1. 增加tf发布：world -> robot_v2/base_link
2. 增加tf_static发布：robot_v2/link6 -> robot_v2/grasp/base_link, robot_v2/baselink -> hand_camera
3. grasp的base_link改为grasp_base_link，避免base_link命名重复

## robot_description
包含robot.urdf文件的依赖包

## robot_moveit_config
使用moveit2和robot_grasp_moveit.urdf配置的控制机械臂的moveit2包

## 其他
- start_stage.py中发布力传感器数据/ee_contact和钩提杆下摘钩点位置/hookrod_b_pos
- whole_stage_velocity_control.usd中配置了速度控制模式的机械臂，moveit2也是根据该模式建立的

## 使用
- 启动节点的两种方式
```bash
$ ros2 launch train_robot robot_control.launch.py
```
```bash
$ bash control_moveit.sh
```
- 启动start_stage.py和moveit相关节点后，在新终端发送命令可以执行一次摘钩操作
```bash
$ ros2 topic pub --once /control_order std_msgs/msg/String "{data: 'decouple'}"
```
- 可用rosbag保存topic：/ee_contact（及其他topic）并在Foxglove中查看
```bash
ros2 bag record /ee_contact
```