from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_description',
            executable='virtual_coupler_force_node',
            name='virtual_coupler_force_node',
            output='screen',
            parameters=[{
                'robot_pose': [2.87, -168.884903, 3.47, 0.0],
                'coupler_pose': [3.5, -170.0, 4.735, -1.5708],
                'tool_contact_offset': [-0.3305, 0.017, 1.284],
                'coupler_contact_offset': [0.0, -0.652, 1.304],
                'contact_axis_world': [0.0, 1.0, 0.0],
                'free_gap': 0.12,
                'lateral_window': 0.35,
                'k_load': 8000.0,
                'k_unload': 4500.0,
                'd_load': 180.0,
                'd_unload': 90.0,
                'mu': 0.28,
                'force_limit': 2500.0,
                'filter_alpha': 0.25,
                'publish_rate': 100.0,
            }],
        ),
    ])
