import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # Get gz_ros2_control install prefix for the patched plugin
    gz_ros2_control_prefix = os.path.dirname(os.path.dirname(get_package_share_directory('gz_ros2_control')))
    gz_plugin_path = os.path.join(gz_ros2_control_prefix, 'lib')

    urdf_file = PathJoinSubstitution([pkg_robot_description, 'urdf', 'robot_complete.urdf'])
    carriage_urdf_path = os.path.join(pkg_robot_description, 'urdf', 'single_carriage_gazebo.urdf')
    controller_config = PathJoinSubstitution([pkg_robot_description, 'config', 'gazebo_controllers.yaml'])
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_default = os.path.join(pkg_robot_description, 'worlds', 'empty_sensors.sdf')
    world_file = LaunchConfiguration('world', default=world_file_default)

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

    # Spawn single carriage (train car)
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

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller'],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', gz_plugin_path),
        SetEnvironmentVariable('GZ_PLUGIN_PATH', gz_plugin_path),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=world_file_default,
            description='Gazebo world file'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ['-v 4 ', world_file],
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
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
        # Camera sensor bridges
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_camera_down',
            output='screen',
            arguments=['/camera_down@sensor_msgs/msg/Image[gz.msgs.Image'],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_camera_up',
            output='screen',
            arguments=['/camera_up@sensor_msgs/msg/Image[gz.msgs.Image'],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_camera_hand',
            output='screen',
            arguments=['/camera_hand@sensor_msgs/msg/Image[gz.msgs.Image'],
        ),
        # Lidar sensor bridges
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_lidar_down',
            output='screen',
            arguments=['/lidar_down@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_lidar_up',
            output='screen',
            arguments=['/lidar_up@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        ),
        gz_spawn_entity,
        gz_spawn_carriage,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[velocity_controller_spawner],
            )
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
