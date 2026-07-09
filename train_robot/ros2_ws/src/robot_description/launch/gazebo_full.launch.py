import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_file_path = os.path.join(pkg_robot_description, 'urdf', 'robot_complete.urdf')
    carriage_urdf_path = os.path.join(pkg_robot_description, 'urdf', 'single_carriage_gazebo.urdf')
    hump_urdf_path = os.path.join(pkg_robot_description, 'urdf', 'hump_gazebo.urdf')
    hook_urdf_path = os.path.join(pkg_robot_description, 'urdf', 'hook_gazebo.urdf')
    power_cart_urdf_path = os.path.join(pkg_robot_description, 'urdf', 'power_cart_export', 'power_cart.urdf')
    controller_config = PathJoinSubstitution([pkg_robot_description, 'config', 'gazebo_controllers.yaml'])
    robot_pose = {'x': '2.87', 'y': '-168.884903', 'z': '3.492518'}
    carriage_poses = [
        {'name': 'single_carriage', 'x': '0.0', 'y': '-176.106', 'z': '4.78'},
        {'name': 'single_carriage_01', 'x': '0.0', 'y': '-162.212', 'z': '4.78'},
        {'name': 'single_carriage_02', 'x': '0.0', 'y': '-148.318', 'z': '4.78'},
    ]
    hump_pose = {'x': '0.0', 'y': '0.0', 'z': '0.0'}
    power_cart_pose = {'x': '0.0', 'y': '-190.0', 'z': '4.78'}
    hook_pose = {'x': '1.333', 'y': '-169.42049', 'z': '4.277699', 'yaw': '-1.5708'}
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    load_hump = LaunchConfiguration('load_hump', default='true')
    world_file_default = os.path.join(pkg_robot_description, 'worlds', 'empty_sensors.sdf')
    world_file = LaunchConfiguration('world', default=world_file_default)

    robot_description_val = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', urdf_file_path]),
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
            '-x', robot_pose['x'],
            '-y', robot_pose['y'],
            '-z', robot_pose['z'],
        ],
    )

    gz_spawn_carriages = [
        Node(
            package='ros_gz_sim',
            executable='create',
            name=f"spawn_{pose['name']}",
            output='screen',
            arguments=[
                '-file', carriage_urdf_path,
                '-name', pose['name'],
                '-x', pose['x'],
                '-y', pose['y'],
                '-z', pose['z'],
            ],
        )
        for pose in carriage_poses
    ]

    gz_spawn_hump = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_hump',
        condition=IfCondition(load_hump),
        output='screen',
        arguments=[
            '-file', hump_urdf_path,
            '-name', 'hump',
            '-x', hump_pose['x'],
            '-y', hump_pose['y'],
            '-z', hump_pose['z'],
        ],
    )

    gz_spawn_hook = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_hook',
        output='screen',
        arguments=[
            '-file', hook_urdf_path,
            '-name', 'hook',
            '-x', hook_pose['x'],
            '-y', hook_pose['y'],
            '-z', hook_pose['z'],
            '-Y', hook_pose['yaw'],
        ],
    )

    gz_spawn_power_cart = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_power_cart',
        output='screen',
        arguments=[
            '-file', power_cart_urdf_path,
            '-name', 'power_cart',
            '-x', power_cart_pose['x'],
            '-y', power_cart_pose['y'],
            '-z', power_cart_pose['z'],
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
        DeclareLaunchArgument(
            'load_hump',
            default_value='true',
            description='Spawn the large hump model'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ['-r -v 4 ', world_file],
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
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_wrist_ft',
            output='screen',
            arguments=['/wrist_ft@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench'],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_grasp_contacts',
            output='screen',
            arguments=['/grasp_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_hook_contacts',
            output='screen',
            arguments=['/hook_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],
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
        gz_spawn_hump,
        *gz_spawn_carriages,
        gz_spawn_power_cart,
        gz_spawn_hook,
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
