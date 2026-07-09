import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_robot_description = get_package_share_directory('robot_description')

    gazebo_urdf_path = os.path.join(pkg_robot_description, 'urdf', 'robot_complete_gazebo.urdf')
    carriage_urdf_path = os.path.join(pkg_robot_description, 'urdf', 'single_carriage_gazebo.urdf')
    hump_urdf_path = os.path.join(pkg_robot_description, 'urdf', 'hump_gazebo.urdf')
    hook_urdf_path = os.path.join(pkg_robot_description, 'urdf', 'hook_gazebo.urdf')
    power_cart_urdf_path = os.path.join(pkg_robot_description, 'urdf', 'power_cart_export', 'power_cart.urdf')
    world_file_default = os.path.join(pkg_robot_description, 'worlds', 'empty_sensors.sdf')

    load_hump = LaunchConfiguration('load_hump', default='true')

    robot_pose = {'x': '2.87', 'y': '-168.884903', 'z': '3.47'}
    carriage_poses = [
        {'name': 'single_carriage', 'x': '0.0', 'y': '-176.106', 'z': '4.78'},
        {'name': 'single_carriage_01', 'x': '0.0', 'y': '-162.212', 'z': '4.78'},
        {'name': 'single_carriage_02', 'x': '0.0', 'y': '-148.318', 'z': '4.78'},
    ]
    hump_pose = {'x': '0.0', 'y': '0.0', 'z': '0.0'}
    power_cart_pose = {'x': '0.0', 'y': '-190.0', 'z': '4.78'}
    hook_poses = [
        {'name': 'hook', 'x': '3.5', 'y': '-170', 'z': '4.735', 'yaw': '-1.5708'},
        {
            'name': 'hook_carriage_joint',
            'x': '1.333',
            'y': '-169.42049',
            'z': '4.277699',
            'yaw': '-1.5708',
        },
    ]

    robot_description_val = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', gazebo_urdf_path]),
        value_type=str,
    )

    gz_spawn_robot = Node(
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

    gz_spawn_hooks = [
        Node(
            package='ros_gz_sim',
            executable='create',
            name=f"spawn_{pose['name']}",
            output='screen',
            arguments=[
                '-file', hook_urdf_path,
                '-name', pose['name'],
                '-x', pose['x'],
                '-y', pose['y'],
                '-z', pose['z'],
                '-Y', pose['yaw'],
            ],
        )
        for pose in hook_poses
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'load_hump',
            default_value='true',
            description='Spawn the large hump model',
        ),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            '/home/zhan/train_robot-main/train_robot/assets/urdf/single_carriage/meshes',
        ),
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v', '4', world_file_default],
            name='gazebo',
            output='screen',
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
        gz_spawn_robot,
        gz_spawn_hump,
        *gz_spawn_carriages,
        gz_spawn_power_cart,
        *gz_spawn_hooks,
    ])
