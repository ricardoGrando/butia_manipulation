# Gazebo ROS Doris

## Quick Start

Gazebo:

    roslaunch doris_gazebo doris_world.launch

ROS Control:

    roslaunch doris_control doris_control.launch

Example of Moving Joints:

    rostopic pub /doris/joint2_position_controller/command std_msgs/Float64 "data: -0.9"


