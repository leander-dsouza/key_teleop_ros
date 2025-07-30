#!/usr/bin/env python3

# Copyright (c) 2025, Leander Stephen Desouza
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Launch file for key_teleop_ros."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the key_drive node."""
    update_rate = 50.0
    max_linear_vel = 2.0
    max_angular_vel = 5.0
    linear_vel_start = 1.5
    angular_vel_start = 2.5
    cmd_vel_topic = '/cmd_vel'

    key_drive_node = Node(
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

    ld = LaunchDescription()
    ld.add_action(key_drive_node)

    return ld
