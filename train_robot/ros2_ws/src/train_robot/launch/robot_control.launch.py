from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    moveit_package_name = "arm_grasp_moveit_config"
    moveit_package_name = "robot_moveit_config"
    moveit_config = (
        MoveItConfigsBuilder('robot', package_name=moveit_package_name)
        .robot_description(file_path="config/robot.urdf.xacro")
        .robot_description_semantic(file_path="config/robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory(moveit_package_name)
            + "/config/moveit_py.yaml") # ***
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"], 
            default_planning_pipeline="ompl" # 避免ompl的采样
        )
        .to_moveit_configs()
    )

    # control node
    robot_control_node = Node(
        package='train_robot',
        executable='control',
        name='robot_control_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # move group
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ]
    )

    # moveit controller node
    self_moveit_plan_node = Node(
        package='train_robot',
        executable='moveit_node',
        name='moveit_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': True
            },
        ]
    )

    # rviz配置文件
    rviz_config_file = os.path.join(moveit_config.package_path, 'config', 'moveit_define.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file], # 视觉配置文件
        parameters=[ # moveit所需参数
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            # moveit_config.planning_pipelines,
            {'use_sim_time': True},
        ],
    )

    # rviz Robot State Publisher, TF
    robot_state_publish_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        respawn=False,
        name='robot_state_publisher',
        output='screen',

        parameters=[
            moveit_config.robot_description,
            {
                "publish_frequency": 100.0,
                'use_sim_time': True
            },
        ],
        remappings=[('/joint_states', '/joint_states_sub')],
    )
    
    # joint state broadcaster, 启动器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': True}]
    )
    
    # ros2_control node, 服务器
    joint_driver_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            {'use_sim_time': True}
        ],
    )

    # arm controller spawner
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': True}]
    )


    # 联动关停：control节点退出时，触发launch全局Shutdown，确保所有托管节点一起停止
    shutdown_on_control_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_control_node,
            on_exit=[
                LogInfo(msg="robot_control_node exited, triggering global shutdown for all managed nodes."),
                EmitEvent(event=Shutdown(reason="robot_control_node exited")),
            ],
        )
    )
    
    return LaunchDescription([
        joint_state_broadcaster_spawner, # 关节状态发布到joint_states
        robot_state_publish_node, # 发布tf
        rviz_node,
        arm_controller_spawner,
        move_group_node,
        joint_driver_node,
        # robot_control_node, # 自定义node
        shutdown_on_control_exit, # control退出触发全局关停
        self_moveit_plan_node, # moveit规划接口node
    ])
