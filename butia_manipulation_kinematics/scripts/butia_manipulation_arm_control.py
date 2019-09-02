#! /usr/bin/env python
import time
import math
import numpy as np

import rospy
from std_msgs.msg import *
from butia_manipulation_msgs.msg import *

from kinematics import *

from gazebo_msgs.srv import GetLinkState 
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetLinkState 
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import LinkState # For getting information about link states
import time
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from gazebo_msgs.srv import SpawnModel
import geometry_msgs.msg

# negative right
SHOULDER_YAW_UP_DOWN_ZERO = 180.0
SHOULDER_YAW_UP_DOWN_MAX = 225.0  
SHOULDER_YAW_UP_DOWN_MIN = 135.0

SHOULDER_PITCH_UP_DOWN_ZERO = 180.0
SHOULDER_PITCH_UP_DOWN_MAX = 225.0 # 45 degrees (down) 
SHOULDER_PITCH_UP_DOWN_MIN = 135.0 # -45 degrees (up) 

ELBOW_1_PITCH_UP_ZERO = 180.0
ELBOW_1_PITCH_UP_MAX = 225.0 # -45 degrees (up) 
ELBOW_1_PITCH_UP_MIN = 125.0 # 45 degrees (down) 

ELBOW_1_PITCH_DOWN_ZERO = 190.0
ELBOW_1_PITCH_DOWN_MAX = 235.0 # -45 degrees (up) 
ELBOW_1_PITCH_DOWN_MIN = 135.0 # 45 degrees (down) 

ELBOW_2_PITCH_UP_DOWN_ZERO = 170.0 
ELBOW_2_PITCH_UP_DOWN_MAX = 215.0 # 45 degrees (down) 
ELBOW_2_PITCH_UP_DOWN_MIN = 125.0 # 45 degrees (up) 

GRIPPER_PITCH_ZERO = 180.0
GRIPPER_PITCH_MAX = 225.0 # -45 degrees (up)
GRIPPER_PITCH_MIN = 135.0 # 45 degrees (down)

#negative right
GRIPPER_YAW_ZERO = 180.0
GRIPPER_YAW_MAX = 225.0 # 45 degrees (up)
GRIPPER_YAW_MIN = 180.0 # -45 degrees (down)

GRIPPER_ROLL_ZERO = 180.0
GRIPPER_ROLL_MAX = 225.0 
GRIPPER_ROLL_MIN = 180.0 

angles_raw = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def shoulder_yaw_joint_position_controller(data):
    angles_raw[0] = ((data.data - SHOULDER_YAW_UP_DOWN_MIN)/(SHOULDER_YAW_UP_DOWN_MAX-SHOULDER_YAW_UP_DOWN_MIN))*(-math.pi/2) + math.pi/4

def shoulder_pitch_joint_up_position_controller(data):
    angles_raw[1] = ((data.data - SHOULDER_PITCH_UP_DOWN_MIN)/(SHOULDER_PITCH_UP_DOWN_MAX-SHOULDER_PITCH_UP_DOWN_MIN))*(math.pi/2) + -math.pi/4

def shoulder_pitch_joint_down_position_controller(data):
    angles_raw[2] = ((data.data - SHOULDER_PITCH_UP_DOWN_MIN)/(SHOULDER_PITCH_UP_DOWN_MAX-SHOULDER_PITCH_UP_DOWN_MIN))*(math.pi/2) + -math.pi/4

def elbow_1_pitch_joint_up_position_controller(data):
    angles_raw[3] = ((data.data - ELBOW_1_PITCH_UP_MIN)/(ELBOW_1_PITCH_UP_MAX-ELBOW_1_PITCH_UP_MIN))*(-math.pi/2) + math.pi/4

def elbow_1_pitch_joint_down_position_controller(data):
    angles_raw[4] = ((data.data - ELBOW_1_PITCH_DOWN_MIN)/(ELBOW_1_PITCH_DOWN_MAX-ELBOW_1_PITCH_DOWN_MIN))*(-math.pi/2) + math.pi/4

def elbow_2_pitch_joint_up_position_controller(data):
    angles_raw[5] = ((data.data - ELBOW_2_PITCH_UP_DOWN_MIN)/(ELBOW_2_PITCH_UP_DOWN_MAX-ELBOW_2_PITCH_UP_DOWN_MIN))*(math.pi/2) + -math.pi/4

def elbow_2_pitch_joint_down_position_controller(data):
    angles_raw[6] = ((data.data - ELBOW_2_PITCH_UP_DOWN_MIN)/(ELBOW_2_PITCH_UP_DOWN_MAX-ELBOW_2_PITCH_UP_DOWN_MIN))*(math.pi/2) + -math.pi/4

def gripper_pitch_joint_position_controller(data):
    angles_raw[7] = ((data.data - GRIPPER_PITCH_MIN)/(GRIPPER_PITCH_MAX-GRIPPER_PITCH_MIN))*(-math.pi/2) + math.pi/4

def gripper_yaw_joint_position_controller(data):
    angles_raw[8] = ((data.data - GRIPPER_YAW_MIN)/(GRIPPER_YAW_MAX-GRIPPER_YAW_MIN))*(-math.pi/2) + math.pi/4

def gripper_roll_joint_position_controller(data):
    angles_raw[9] = ((data.data - GRIPPER_ROLL_MIN)/(GRIPPER_ROLL_MAX-GRIPPER_ROLL_MIN))*(-math.pi/2) + math.pi/4

gazeboPubList = [       '/butia_manipulation/shoulder_yaw_joint_position_controller/command',
                        '/butia_manipulation/shoulder_pitch_joint_up_position_controller/command',  
                        '/butia_manipulation/shoulder_pitch_joint_down_position_controller/command',                               
                        '/butia_manipulation/elbow_1_pitch_joint_up_position_controller/command',   
                        '/butia_manipulation/elbow_1_pitch_joint_down_position_controller/command',   
                        '/butia_manipulation/elbow_2_pitch_joint_up_position_controller/command',   
                        '/butia_manipulation/elbow_2_pitch_joint_down_position_controller/command',                                        
                        '/butia_manipulation/gripper_pitch_joint_position_controller/command',
                        '/butia_manipulation/gripper_yaw_joint_position_controller/command',
                        '/butia_manipulation/gripper_roll_joint_position_controller/command'
                ]

publishers = []

for topic in gazeboPubList:
    publishers.append(rospy.Publisher(topic, Float64, queue_size=10))

finished_topic = rospy.Publisher('/butia_manipulation_kinematics_finished', Bool, queue_size=10)                

rospy.Subscriber("/butia_manipulation/shoulder_yaw_joint_position_controller/state", Float64, shoulder_yaw_joint_position_controller)
rospy.Subscriber("/butia_manipulation/shoulder_pitch_joint_up_position_controller/state", Float64, shoulder_pitch_joint_up_position_controller)
rospy.Subscriber("/butia_manipulation/shoulder_pitch_joint_down_position_controller/state", Float64, shoulder_pitch_joint_down_position_controller)
rospy.Subscriber("/butia_manipulation/elbow_1_pitch_joint_up_position_controller/state", Float64, elbow_1_pitch_joint_up_position_controller)
rospy.Subscriber("/butia_manipulation/elbow_1_pitch_joint_down_position_controller/state", Float64, elbow_1_pitch_joint_down_position_controller)
rospy.Subscriber("/butia_manipulation/elbow_2_pitch_joint_up_position_controller/state", Float64, elbow_2_pitch_joint_up_position_controller)
rospy.Subscriber("/butia_manipulation/elbow_2_pitch_joint_down_position_controller/state", Float64, elbow_2_pitch_joint_down_position_controller)
rospy.Subscriber("/butia_manipulation/gripper_pitch_joint_position_controller/state", Float64, gripper_pitch_joint_position_controller)
rospy.Subscriber("/butia_manipulation/gripper_yaw_joint_position_controller/state", Float64, gripper_yaw_joint_position_controller)
rospy.Subscriber("/butia_manipulation/gripper_roll_joint_position_controller/state", Float64, gripper_roll_joint_position_controller)

def inverse_kinematics_callback(data):
    # Pegar o target_end_effector da mensagem recebida e converter para array de numpy
    target_position = np.array([data.target_end_effector[0], data.target_end_effector[1], data.target_end_effector[2], data.target_end_effector[3], data.target_end_effector[4], data.target_end_effector[5]])

    ############################################################
    # MUST BE READ FROM THE MOTORS
    # Angulos da posicao inicial
    #angles = np.array([angles_raw[0], angles_raw[1], angles_raw[5], angles_raw[7], angles_raw[8], angles_raw[9]]) 
    ###########################################################
    angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 
    
    # Taxa de sleep
    rate = rospy.Rate(1)

    # Realizar a cinematica direta para obter a posicao e orientacao cartesiana do end effector
    atual_position = forwardKinematics(angles)
    distance = target_position - atual_position
   
    while(max(abs(distance)) > 0.01):
        ############################################################
        # MUST BE READ FROM THE MOTORS
        angles = angles
        ############################################################
        # Realizar a cinematica direta para obter a posicao e orientacao cartesiana do end effector        
        atual_position = forwardKinematics(angles)
        distance = target_position - atual_position

        #print(atual_position)
        #print(angles)

        J = calc_jacobian(angles)
        J_inv = np.linalg.pinv(J)
        
        delta_end_effector = ((distance)*data.step_size)/np.max(max(abs(distance)))

        delta_angles = J_inv.dot(delta_end_effector)
        
        if max(delta_angles) > 0.1:
            while(max(delta_angles) > 0.1):
                delta_angles = delta_angles/10
                print("adsfasd")

        angles += delta_angles			
        
        # publishers[0].publish(180.0) 
        # publishers[1].publish(SHOULDER_PITCH_UP_DOWN_MIN + (SHOULDER_PITCH_UP_DOWN_MAX-SHOULDER_PITCH_UP_DOWN_MIN)*(angles[1] - (-math.pi/4))/(math.pi/2))  
        # publishers[2].publish(SHOULDER_PITCH_UP_DOWN_MIN + (SHOULDER_PITCH_UP_DOWN_MAX-SHOULDER_PITCH_UP_DOWN_MIN)*(angles[1] - (-math.pi/4))/(math.pi/2)) 
        # publishers[3].publish(ELBOW_1_PITCH_UP_MIN + (ELBOW_1_PITCH_UP_MAX-ELBOW_1_PITCH_UP_MIN)*(-angles[1] - (math.pi/4))/(-math.pi/2))
        # publishers[4].publish(ELBOW_1_PITCH_DOWN_MIN + (ELBOW_1_PITCH_DOWN_MAX-ELBOW_1_PITCH_DOWN_MIN)*(-angles[1] - (math.pi/4))/(-math.pi/2))
        # publishers[5].publish(ELBOW_2_PITCH_UP_DOWN_MIN + (ELBOW_2_PITCH_UP_DOWN_MAX-ELBOW_2_PITCH_UP_DOWN_MIN)*(angles[2] - (-math.pi/4))/(math.pi/2))
        # publishers[6].publish(ELBOW_2_PITCH_UP_DOWN_MIN + (ELBOW_2_PITCH_UP_DOWN_MAX-ELBOW_2_PITCH_UP_DOWN_MIN)*(angles[2] - (-math.pi/4))/(math.pi/2))
        # publishers[7].publish(GRIPPER_PITCH_MIN + (GRIPPER_PITCH_MAX-GRIPPER_PITCH_MAX)*(angles[3] - (math.pi/4))/(-math.pi/2))
        # publishers[8].publish(GRIPPER_YAW_MIN + (GRIPPER_YAW_MAX-GRIPPER_YAW_MAX)*(angles[4] - (math.pi/4))/(-math.pi/2))
        # publishers[9].publish(180)
        print (angles_raw)
        finished_topic.publish(False)

        raw_input()

    print("Arrived!!!")
    finished_topic.publish(True)
    rate.sleep()

def butia_manipulation_control():
	rospy.init_node("butia_manipulation_arm_control", anonymous=False)

	rospy.Subscriber('/butia_manipulation_arm_target_position', CartesianMsg, inverse_kinematics_callback)
	
	rospy.spin()

####################################### Exemplo para testar a aplicacao #######################################
# rostopic pub -1 /cartesian_position butia_manipulation_control_msgs/CartesianMsg "target_end_effector: [-17, 2, 20, -2.7, 1.46, 2.67] step_size: 0.1"		

if __name__ == "__main__": 
    butia_manipulation_control()      
    