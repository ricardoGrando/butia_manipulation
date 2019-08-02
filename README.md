# Gazebo Butia Manipulation and Doris Manipulation

## Gazebo arm

Running Gazebo world:

    roslaunch butia_manipulation_gazebo butia_manipulation_world.launch

Running Manipulation Control for Gazebo:

    roslaunch butia_manipulation_control butia_manipulation_control.launch

Example of Moving Joints:

    rostopic pub /butia_manipulation/joint2_position_controller/command std_msgs/Float64 "data: -0.9"

## Manipulation

Running kinematics package:

    rosrun butia_manipulation_kinematics butia_manipulation_arm_control.py

Publishing a target position:

    rostopic pub -1 /butia_manipulation_arm_target_position butia_manipulation_msgs/CartesianMsg "target_end_effector: [0.5, 0.1, 1.1, 0, 0.0, 0]
step_size: 0.01"

    Message type: butia_manipulation_msgs/CartesianMsg

Verifying if the target position was achieved

   rostopic echo /butia_manipulation_kinematics_finished

   Message type: std_msgs bool

   Example of return:
   ...
   data: True
   ...

## THE INTERFACE WITH THE ACTUATORS YET TO BE DONE


