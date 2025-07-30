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

"""Teleoperation using arrow keys for ROS2."""

from typing import Optional, Union

from geometry_msgs.msg import Twist, Vector3
from pynput import keyboard
from pynput.keyboard import Key, KeyCode

import rclpy
from rclpy.node import Node


class KeyDrive(Node):
    """Class to teleoperate the robot using arrow keys."""

    def __init__(self) -> None:
        """Initialize the KeyDrive class."""
        super().__init__('key_teleop_node')

        # Parameters
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('max_linear_vel', 5.0)
        self.declare_parameter('max_angular_vel', 5.0)
        self.declare_parameter('linear_vel_start', 1.5)
        self.declare_parameter('angular_vel_start', 2.5)

        self.update_rate = self.get_parameter('update_rate').value
        self.time_period = 1.0 / self.update_rate

        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.linear_vel = self.get_parameter('linear_vel_start').value
        self.angular_vel = self.get_parameter('angular_vel_start').value

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(self.time_period, self.keyboard_update)

    def forward(self) -> None:
        """Move Forward."""
        self.pub.publish(Twist(linear=Vector3(x=self.linear_vel)))

    def backward(self) -> None:
        """Move Backward."""
        self.pub.publish(Twist(linear=Vector3(x=-self.linear_vel)))

    def left(self) -> None:
        """Move Left."""
        self.pub.publish(Twist(angular=Vector3(z=self.angular_vel)))

    def right(self) -> None:
        """Move Right."""
        self.pub.publish(Twist(angular=Vector3(z=-self.angular_vel)))

    def brutestop(self) -> None:
        """Stop the robot."""
        self.pub.publish(Twist())

    def key_press(self, key: Optional[Union[Key, KeyCode]]) -> None:
        """Listen for key press."""
        if key is None:
            return

        if isinstance(key, Key):
            if key == Key.up:
                self.forward()
            elif key == Key.down:
                self.backward()
            elif key == Key.right:
                self.right()
            elif key == Key.left:
                self.left()

        elif isinstance(key, KeyCode):
            if key.char == 'w':
                if self.linear_vel < self.max_linear_vel:
                    self.linear_vel += 0.1
                    self.linear_vel = round(self.linear_vel, 1)
                    self.get_logger().info(f'Linear Velocity: {self.linear_vel}')
                else:
                    self.get_logger().info('Reached Max Linear Velocity')

            elif key.char == 's':
                if self.linear_vel > 0.1:
                    self.linear_vel -= 0.1
                    self.linear_vel = round(self.linear_vel, 1)
                    self.get_logger().info(f'Linear Velocity: {self.linear_vel}')
                else:
                    self.get_logger().info('Reached Minimal Linear Velocity')

            elif key.char == 'd':
                if self.angular_vel < self.max_angular_vel:
                    self.angular_vel += 0.1
                    self.angular_vel = round(self.angular_vel, 1)
                    self.get_logger().info(f'Angular Velocity: {self.angular_vel}')
                else:
                    self.get_logger().info('Reached Max Angular Velocity')

            elif key.char == 'a':
                if self.angular_vel > 0.1:
                    self.angular_vel -= 0.1
                    self.angular_vel = round(self.angular_vel, 1)
                    self.get_logger().info(f'Angular Velocity: {self.angular_vel}')
                else:
                    self.get_logger().info('Reached Minimal Angular Velocity')

            elif key.char == 'q':
                self.get_logger().info('Shutting down...')
                if rclpy.ok():
                    self.destroy_node()
                    rclpy.shutdown()

    def key_release(self, _: Optional[Union[Key, KeyCode]]) -> None:
        """Listen for key release."""
        self.brutestop()

    def keyboard_update(self) -> None:
        """Keyboard Listener for a press and release event."""
        listener = None
        try:
            listener = keyboard.Listener(
                on_press=self.key_press,
                on_release=self.key_release)
            listener.start()

            while rclpy.ok() and listener.running:
                rclpy.spin_once(self, timeout_sec=0.1)

        finally:
            if listener is not None:
                listener.stop()


def main(args: Optional[list[str]] = None) -> None:
    """Initialize the ROS2 Node and KeyDrive class."""
    rclpy.init(args=args)
    node = KeyDrive()

    node.get_logger().info('\n\
        ↑ ↓ ← → : Arrow keys for movement\n\
        w s : Increase/Decrease linear velocity (0.1 m/s)\n\
        d a : Increase/Decrease angular velocity (0.1 rad/s)\n\
        q: Quit\n')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
