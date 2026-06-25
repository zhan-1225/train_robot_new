from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_train_robot = get_package_share_directory('train_robot')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gazebo_control_node = Node(
        package='train_robot',
        executable='gazebo_control',
        name='gazebo_robot_control_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    shutdown_on_control_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo_control_node,
            on_exit=[
                LogInfo(msg="gazebo_robot_control_node exited, triggering global shutdown."),
                EmitEvent(event=Shutdown(reason="gazebo_robot_control_node exited")),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        gazebo_control_node,
        shutdown_on_control_exit,
    ])
