#!/usr/bin/env python3
"""Launch file for key_teleop_ros."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the key_drive node."""
    update_rate = 50.0
    max_linear_vel = 2.0
    max_angular_vel = 5.0
    linear_vel_start = 1.5
    angular_vel_start = 2.5
    cmd_vel_topic = '/cmd_vel'

    return LaunchDescription([
        Node(
            package='key_teleop_ros',
            executable='key_drive',
            name='key_drive_node',
            output='screen',
            parameters=[
                {'update_rate': update_rate},
                {'max_linear_vel': max_linear_vel},
                {'max_angular_vel': max_angular_vel},
                {'linear_vel_start': linear_vel_start},
                {'angular_vel_start': angular_vel_start},
            ],
            remappings=[
                ('/cmd_vel', cmd_vel_topic)
            ]

        )
    ])
