# Gazebo ROS Demos

* Author: Dave Coleman <davetcoleman@gmail.com>
* License: GNU General Public License, version 3 (GPL-3.0)

Example robots and code for interfacing Gazebo with ROS

## Tutorials

[ROS URDF](http://gazebosim.org/tutorials/?tut=ros_urdf)

## Quick Start

Rviz:

    roslaunch doris_description doris_rviz.launch

Gazebo:

    roslaunch doris_gazebo doris_world.launch

ROS Control:

    roslaunch doris_control doris_control.launch

Example of Moving Joints:

    rostopic pub /doris/joint2_position_controller/command std_msgs/Float64 "data: -0.9"

## Develop and Contribute

We welcome any contributions to this repo and encourage you to fork the project then send pull requests back to this parent repo. Thanks for your help!
