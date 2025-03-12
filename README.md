# key_teleop_ros
![ROS](https://img.shields.io/badge/-ROS-22314E?style=plastic&logo=ROS)
![Python](https://img.shields.io/badge/-Python-black?style=plastic&logo=Python)

A ROS2 package to drive a robot around using arrow keys.

The following video is recorded with the [atreus](https://github.com/leander-dsouza/atreus) robot, embedding the key presses using [screenkey](https://www.thregr.org/wavexx/software/screenkey/).

https://github.com/leander-dsouza/key_teleop_ros/assets/45683974/e2bec100-d9aa-4712-b843-7f26a5b68428

## Installation

* Install all the dependencies using `rosdep`:

   ```bash
   rosdep install --from-paths $ROS_WS/src --ignore-src -r -y
   ```

* Build the workspace:

   ```bash
   colcon build --packages-select key_teleop_ros
   ```

## Usage

* After launching your robot, you can execute the following command to drive it around using the arrow keys:

   ```bash
   ros2 run key_teleop_ros key_drive
   ```
    Or you can execute the node using a launch file:

   ```bash
   ros2 launch key_teleop_ros key_drive.launch.py
   ```

* The teleoperation node's parameters are as follows:
    * `~update_rate` - The rate at which the node updates the robot's velocity. (default: 50 Hz)
    * `~linear_vel_start` - The initial linear velocity of the robot. (default: 1.5 m/s)
    * `~angular_vel_start` - The initial angular velocity of the robot. (default: 2.5 rad/s)
    * `~max_linear_vel` - The maximum linear velocity of the robot. (default: 5.0 m/s)
    * `~max_angular_vel` - The maximum angular velocity of the robot. (default: 5.0 rad/s)

    The topic name can be remapped using the `cmd_vel` argument.

* The following keys are used to drive and configure the robot:
    * `↑` - Move forward
    * `↓` - Move backward
    * `←` - Turn left
    * `→` - Turn right
    * `w` - Increase linear velocity
    * `s` - Decrease linear velocity
    * `d` - Increase angular velocity
    * `a` - Decrease angular velocity
    * `q` - Quit the program

## Standardization

* The entire package is formatted against [pycodestyle](https://pypi.org/project/pycodestyle/) and [pydocstyle](https://pypi.org/project/pydocstyle/).

###### 💾 EOF