#!/usr/bin/env python3
"""Teleoperation using arrow keys for ROS2."""

from geometry_msgs.msg import Twist, Vector3
from pynput import keyboard
from pynput.keyboard import Key

import rclpy
from rclpy.node import Node


class KeyDrive(Node):
    """Class to teleoperate the robot using arrow keys."""

    def __init__(self):
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

        # Publisher
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Timer
        self.create_timer(self.time_period, self.keyboard_update)

    def forward(self):
        """Move Forward."""
        self.pub.publish(Twist(linear=Vector3(x=self.linear_vel)))

    def backward(self):
        """Move Backward."""
        self.pub.publish(Twist(linear=Vector3(x=-self.linear_vel)))

    def left(self):
        """Move Left."""
        self.pub.publish(Twist(angular=Vector3(z=self.angular_vel)))

    def right(self):
        """Move Right."""
        self.pub.publish(Twist(angular=Vector3(z=-self.angular_vel)))

    def brutestop(self):
        """Stop the robot."""
        self.pub.publish(Twist())

    def key_press(self, key):
        """Listen for key press."""
        try:
            if key == Key.up:
                self.forward()
            elif key == Key.down:
                self.backward()
            elif key == Key.right:
                self.right()
            elif key == Key.left:
                self.left()
            elif key.char == 'w':
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
                return False

        except AttributeError:
            pass
        return False

    def key_release(self, _):
        """Listen for key release."""
        self.brutestop()
        return False

    def keyboard_update(self):
        """Keyboard Listener for a press and release event."""
        with keyboard.Listener(on_press=self.key_press, on_release=self.key_release) as listener:
            listener.join()


def main(args=None):
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
