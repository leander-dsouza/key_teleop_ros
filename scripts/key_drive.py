#! /usr/bin/env python3
"""Teleoperation using arrow keys for ROS1."""

import argparse
import rospy
from geometry_msgs.msg import Twist, Vector3
from pynput import keyboard
from pynput.keyboard import Key


class KeyDrive():
    """Class to teleoperate the robot using arrow keys."""

    def __init__(self, args):
        """Initialize the KeyDrive class."""
        self.update_rate = args.rate
        self.time_period = 1./self.update_rate

        self.max_linear_velocity = args.max_linear_velocity
        self.max_angular_velocity = args.max_angular_velocity
        self.linear_velocity = args.linear_velocity
        self.angular_velocity = args.angular_velocity
        self.obj = Twist()

        # Publishers
        self.pub = rospy.Publisher(args.topic_name, Twist, queue_size=10)

        # Timers
        rospy.Timer(rospy.Duration(self.time_period), self.keyboard_update)

    def forward(self):
        """Move Forward."""
        self.pub.publish(Twist(linear=Vector3(x=self.linear_velocity)))

    def backward(self):
        """Move Backward."""
        self.pub.publish(Twist(linear=Vector3(x=-self.linear_velocity)))

    def left(self):
        """Move Left."""
        self.pub.publish(Twist(angular=Vector3(z=self.angular_velocity)))

    def right(self):
        """Move Right."""
        self.pub.publish(Twist(angular=Vector3(z=-self.angular_velocity)))

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
                if self.linear_velocity < self.max_linear_velocity:
                    self.linear_velocity += 0.1
                    self.linear_velocity = round(self.linear_velocity, 1)
                    rospy.loginfo(f"Linear Velocity: {self.linear_velocity}")
                else:
                    rospy.loginfo("Reached Max Linear Velocity")
            elif key.char == 's':
                if self.linear_velocity > 0.1:
                    self.linear_velocity -= 0.1
                    self.linear_velocity = round(self.linear_velocity, 1)
                    rospy.loginfo(f"Linear Velocity: {self.linear_velocity}")
                else:
                    rospy.loginfo("Reached Minimal Linear Velocity")
            elif key.char == 'd':
                if self.angular_velocity < self.max_angular_velocity:
                    self.angular_velocity += 0.1
                    self.angular_velocity = round(self.angular_velocity, 1)
                    rospy.loginfo(f"Angular Velocity: {self.angular_velocity}")
                else:
                    rospy.loginfo("Reached Max Angular Velocity")
            elif key.char == 'a':
                if self.angular_velocity > 0.1:
                    self.angular_velocity -= 0.1
                    self.angular_velocity = round(self.angular_velocity, 1)
                    rospy.loginfo(f"Angular Velocity: {self.angular_velocity}")
                else:
                    rospy.loginfo("Reached Minimal Angular Velocity")
            elif key.char == 'q':
                rospy.signal_shutdown("Done")

        except AttributeError:
            pass
        return False

    def key_release(self, _):
        """Listen for key release."""
        self.brutestop()
        return False

    def keyboard_update(self, _):
        """Keyboard Listener for a press and release event."""
        with keyboard.Listener(on_press=self.key_press, suppress=True) \
                as listener_for_key_press:
            listener_for_key_press.join()

        with keyboard.Listener(on_release=self.key_release) \
                as listener_for_key_release:
            listener_for_key_release.join()


def main():
    """Mimic Main Function to initialize the ROS Node and KeyDrive class."""
    parser = argparse.ArgumentParser(
        description='ROS1 Teleoperation using Arrow Keys')
    parser.add_argument('-r', '--rate', type=int, default=50,
                        help='Update Rate for the Teleoperation')
    parser.add_argument('-t', '--topic_name', type=str, default='/cmd_vel',
                        help='Topic to publish Twist messages')
    parser.add_argument('-l', '--linear_velocity', type=float, default=1.5,
                        help='Maximum Linear Velocity')
    parser.add_argument('-a', '--angular_velocity', type=float, default=2.5,
                        help='Maximum Angular Velocity')
    parser.add_argument('-ml', '--max_linear_velocity', type=float,
                        default=5.0,
                        help='Maximum Linear Velocity')
    parser.add_argument('-ma', '--max_angular_velocity', type=float,
                        default=5.0,
                        help='Maximum Angular Velocity')
    args = parser.parse_args()

    # Initialize ROS Node
    rospy.init_node('key_teleop_node')

    rospy.loginfo("\n\
        ↑ ↓ ← → : Arrow keys for movement\n\
        w s : Increase/Decrease linear velocity (0.1 m/s)\n\
        d a : Increase/Decrease angular velocity (0.1 rad/s)\n\
        q: Quit\n")

    # Initialize KeyDrive
    KeyDrive(args)
    rospy.spin()
    rospy.signal_shutdown("Done")


if __name__ == '__main__':
    main()
