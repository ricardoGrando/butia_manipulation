#!/usr/bin/env python
import rospy
from std_msgs.msg import *
import geometry_msgs.msg
import PyDynamixel_v2 as pd
from defines import *

pub_list = []

def dynamixel_interface_state():
    #pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[SHOULDER_YAW_ID]+"/state", Float64, queue_size=10))
    pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[SHOULDER_PITCH_UP_ID]+"/state", Float64, queue_size=10))
    pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[SHOULDER_PITCH_DOWN_ID]+"/state", Float64, queue_size=10))
    pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[ELBOW_1_PITCH_UP_ID]+"/state", Float64, queue_size=10))
    pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[ELBOW_1_PITCH_DOWN_ID]+"/state", Float64, queue_size=10))
    pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[ELBOW_2_PITCH_UP_ID]+"/state", Float64, queue_size=10))
    pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[ELBOW_2_PITCH_DOWN_ID]+"/state", Float64, queue_size=10))
    pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[GRIPPER_PITCH_ID]+"/state", Float64, queue_size=10))
    pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[GRIPPER_YAW_ID]+"/state", Float64, queue_size=10))
    #pub_list.append(rospy.Publisher("/butia_manipulation/"+dict_topics[GRIPPER_ROLL_ID]+"/state", Float64, queue_size=10))
    
    rospy.init_node('dynamixel_interface_state', anonymous=False)
    while not rospy.is_shutdown():
        for i in range(0, len(pub_list)):
            joint_value = serial.joints[i].get_angle()   
            pub_list[i].publish(joint_value) 

        
if __name__ == "__main__": 
    try:
        dynamixel_interface_state()
    except rospy.ROSInterruptException:
        pass
