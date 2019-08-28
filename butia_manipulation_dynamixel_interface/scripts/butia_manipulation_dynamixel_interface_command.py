#!/usr/bin/env python
import rospy
from std_msgs.msg import *
import geometry_msgs.msg
import PyDynamixel_v2 as pd
from defines import *

def shoulder_yaw_joint_position_controller(data):
    # serial.joints[0].enable_torque()

def shoulder_pitch_joint_up_position_controller(data):
    print("porra")

def shoulder_pitch_joint_down_position_controller(data):
    print("porra")

def elbow_1_pitch_joint_up_position_controller(data):
    print("porra")

def elbow_1_pitch_joint_down_position_controller(data):
    print("porra")

def elbow_2_pitch_joint_up_position_controller(data):
    print("porra")

def elbow_2_pitch_joint_down_position_controller(data):
    print("porra")

def gripper_pitch_joint_position_controller(data):
    serial.joints[-2].send_angle(180)

def gripper_yaw_joint_position_controller(data):
    print("porra")

def gripper_roll_joint_position_controller(data):
    print("porra")

if __name__ == "__main__": 
    rospy.init_node("dynamixel_interface_command", anonymous=False)

    rospy.Subscriber("/butia_manipulation/"+dict_topics[SHOULDER_YAW_ID]+"/command", Float64, shoulder_yaw_joint_position_controller)
    rospy.Subscriber("/butia_manipulation/"+dict_topics[SHOULDER_PITCH_UP_ID]+"/command", Float64, shoulder_pitch_joint_up_position_controller)
    rospy.Subscriber("/butia_manipulation/"+dict_topics[SHOULDER_PITCH_DOWN_ID]+"/command", Float64, shoulder_pitch_joint_down_position_controller)
    rospy.Subscriber("/butia_manipulation/"+dict_topics[ELBOW_1_PITCH_UP_ID]+"/command", Float64, elbow_1_pitch_joint_up_position_controller)
    rospy.Subscriber("/butia_manipulation/"+dict_topics[ELBOW_1_PITCH_DOWN_ID]+"/command", Float64, elbow_1_pitch_joint_down_position_controller)
    rospy.Subscriber("/butia_manipulation/"+dict_topics[ELBOW_2_PITCH_UP_ID]+"/command", Float64, elbow_2_pitch_joint_up_position_controller)
    rospy.Subscriber("/butia_manipulation/"+dict_topics[ELBOW_2_PITCH_DOWN_ID]+"/command", Float64, elbow_2_pitch_joint_down_position_controller)
    rospy.Subscriber("/butia_manipulation/"+dict_topics[GRIPPER_PITCH_ID]+"/command", Float64, gripper_pitch_joint_position_controller)
    rospy.Subscriber("/butia_manipulation/"+dict_topics[GRIPPER_YAW_ID]+"/command", Float64, gripper_yaw_joint_position_controller)
    rospy.Subscriber("/butia_manipulation/"+dict_topics[GRIPPER_ROLL_ID]+"/command", Float64, gripper_roll_joint_position_controller)
        
    rospy.spin()
