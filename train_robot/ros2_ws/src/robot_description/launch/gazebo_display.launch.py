import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_file = PathJoinSubstitution([pkg_robot_description, 'urdf', 'robot_complete.urdf'])
    carriage_urdf_path = os.path.join(pkg_robot_description, 'urdf', 'single_carriage_gazebo.urdf')
    
    world_file_default = os.path.join(pkg_robot_description, 'worlds', 'empty_sensors.sdf')

    robot_description_val = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', urdf_file]),
        value_type=str
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'train_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
        ],
    )

    gz_spawn_carriage = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_carriage',
        output='screen',
        arguments=[
            '-file', carriage_urdf_path,
            '-name', 'single_carriage',
            '-x', '3.0',
            '-y', '0.0',
            '-z', '0.65',
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', '/home/zhan/train_robot-main/train_robot/assets/urdf/single_carriage/meshes'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ['-v 4 ', world_file_default],
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_description_val,
            }],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        ),
        gz_spawn_entity,
        gz_spawn_carriage,
    ])
