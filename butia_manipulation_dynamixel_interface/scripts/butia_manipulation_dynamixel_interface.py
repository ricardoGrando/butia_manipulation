#!/usr/bin/env python
import rospy
from std_msgs.msg import *
import geometry_msgs.msg
import PyDynamixel_v2 as pd
from defines import *

def shoulder_yaw_joint_position_controller(data):
    serial.joints[0].send_angle(data.data)
    print("Send "+str(data.data))

def shoulder_pitch_joint_up_position_controller(data):
    serial.joints[1].send_angle(data.data)
    print("Send "+str(data.data))

def shoulder_pitch_joint_down_position_controller(data):
    serial.joints[2].send_angle(data.data)
    print("Send "+str(data.data))

def elbow_1_pitch_joint_up_position_controller(data):
    serial.joints[3].send_angle(data.data)
    print("Send "+str(data.data))

def elbow_1_pitch_joint_down_position_controller(data):
    serial.joints[4].send_angle(data.data)
    print("Send "+str(data.data))

def elbow_2_pitch_joint_up_position_controller(data):
    serial.joints[5].send_angle(data.data)
    print("Send "+str(data.data))

def elbow_2_pitch_joint_down_position_controller(data):
    serial.joints[6].send_angle(data.data)
    print("Send "+str(data.data))

def gripper_pitch_joint_position_controller(data):
    serial.joints[7].send_angle(data.data)
    print("Send "+str(data.data))

def gripper_yaw_joint_position_controller(data):
    serial.joints[8].send_angle(data.data)
    print("Send "+str(data.data))

def gripper_roll_joint_position_controller(data):
    #serial.joints[9].send_angle(data.data)
    print("Send "+str(data.data))

pub_list = []

def dynamixel_interface_state():
    pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[SHOULDER_YAW_ID]+"/state", Float64, queue_size=10))
    pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[SHOULDER_PITCH_UP_ID]+"/state", Float64, queue_size=10))
    pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[SHOULDER_PITCH_DOWN_ID]+"/state", Float64, queue_size=10))
    pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[ELBOW_1_PITCH_UP_ID]+"/state", Float64, queue_size=10))
    pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[ELBOW_1_PITCH_DOWN_ID]+"/state", Float64, queue_size=10))
    pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[ELBOW_2_PITCH_UP_ID]+"/state", Float64, queue_size=10))
    pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[ELBOW_2_PITCH_DOWN_ID]+"/state", Float64, queue_size=10))
    pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[GRIPPER_PITCH_ID]+"/state", Float64, queue_size=10))
    pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[GRIPPER_YAW_ID]+"/state", Float64, queue_size=10))
    #pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[GRIPPER_ROLL_ID]+"/state", Float64, queue_size=10))
    
    rate = rospy.Rate(5)

    # rospy.init_node('dynamixel_interface_state', anonymous=False)
    while not rospy.is_shutdown():
        for i in range(0, len(pub_list)):
            joint_value = serial.joints[i].get_angle()   
            pub_list[i].publish(joint_value) 
            #pass

        rate.sleep()

if __name__ == "__main__": 
    rospy.init_node("dynamixel_interface", anonymous=False)

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
        
    # rospy.spin()

    try:
        dynamixel_interface_state()
    except rospy.ROSInterruptException:
        pass