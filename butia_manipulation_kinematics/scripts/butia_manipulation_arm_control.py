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

from defines import *

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

finished_topic = rospy.Publisher('/butia_manipulation_kinematics_finished', Bool, queue_size=10)

finished_topic = rospy.Publisher('/butia_manipulation_kinematics_finished', Bool, queue_size=10)  

def get_angles():
    read_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    read_angles[0] = ((serial.joints[0].get_angle() - SHOULDER_YAW_UP_DOWN_MIN)/(SHOULDER_YAW_UP_DOWN_MAX-SHOULDER_YAW_UP_DOWN_MIN))*(-math.pi/2) + math.pi/4
    read_angles[1] = ((serial.joints[1].get_angle() - SHOULDER_PITCH_UP_DOWN_MIN)/(SHOULDER_PITCH_UP_DOWN_MAX-SHOULDER_PITCH_UP_DOWN_MIN))*(math.pi/2) + -math.pi/4
    read_angles[2] = ((serial.joints[2].get_angle() - SHOULDER_PITCH_UP_DOWN_MIN)/(SHOULDER_PITCH_UP_DOWN_MAX-SHOULDER_PITCH_UP_DOWN_MIN))*(math.pi/2) + -math.pi/4
    read_angles[3] = ((serial.joints[3].get_angle() - ELBOW_1_PITCH_UP_MIN)/(ELBOW_1_PITCH_UP_MAX-ELBOW_1_PITCH_UP_MIN))*(-math.pi/2) + math.pi/2
    read_angles[4] = ((serial.joints[4].get_angle() - ELBOW_1_PITCH_DOWN_MIN)/(ELBOW_1_PITCH_DOWN_MAX-ELBOW_1_PITCH_DOWN_MIN))*(-math.pi/2) + math.pi/4
    read_angles[5] = ((serial.joints[5].get_angle() - ELBOW_2_PITCH_UP_DOWN_MIN)/(ELBOW_2_PITCH_UP_DOWN_MAX-ELBOW_2_PITCH_UP_DOWN_MIN))*(math.pi/2) + -math.pi/4
    read_angles[6] = ((serial.joints[6].get_angle() - ELBOW_2_PITCH_UP_DOWN_MIN)/(ELBOW_2_PITCH_UP_DOWN_MAX-ELBOW_2_PITCH_UP_DOWN_MIN))*(math.pi/2) + -math.pi/4
    read_angles[7] = ((serial.joints[7].get_angle() - GRIPPER_PITCH_MIN)/(GRIPPER_PITCH_MAX-GRIPPER_PITCH_MIN))*(-math.pi/2) + math.pi/4
    read_angles[8] = ((serial.joints[8].get_angle() - GRIPPER_YAW_MIN)/(GRIPPER_YAW_MAX-GRIPPER_YAW_MIN))*(-math.pi/2)
    # read_angles[9] = ((serial.joints[9].get_angle() - GRIPPER_ROLL_MIN)/(GRIPPER_ROLL_MAX-GRIPPER_ROLL_MIN))*(-math.pi/2) + math.pi/4

    return read_angles

def inverse_kinematics_callback(data):
    # Pegar o target_end_effector da mensagem recebida e converter para array de numpy
    target_position = np.array([data.target_end_effector[0], data.target_end_effector[1], data.target_end_effector[2], data.target_end_effector[3], data.target_end_effector[4], data.target_end_effector[5]])

    ############################################################
    # MUST BE READ FROM THE MOTORS
    # Angulos da posicao inicial
    #angles = np.array([read_angles[0], read_angles[1], read_angles[5], read_angles[7], read_angles[8], read_angles[9]]) 
    ###########################################################
    read_angles = get_angles()
    angles = np.array([read_angles[1], read_angles[5], read_angles[7], read_angles[8], read_angles[9]])  
    
    # Taxa de sleep
    rate = rospy.Rate(1)

    # Realizar a cinematica direta para obter a posicao e orientacao cartesiana do end effector
    atual_position = forwardKinematics(angles)
    distance = target_position - atual_position

    backup_gripper_roll = 0.0
    backup_gripper_yaw = 0.0
    backup_gripper_pitch = 0.0

    raw_input()

    while(max(abs(distance)) > 0.01):
        ############################################################
        read_angles = get_angles()
        angles = np.array([read_angles[1], read_angles[5], read_angles[7], read_angles[8], read_angles[9]]) 

        # angles[3] = backup_gripper_yaw
        # angles[4] = backup_gripper_roll 
        # angles[2] = backup_gripper_pitch
        ############################################################
        # Realizar a cinematica direta para obter a posicao e orientacao cartesiana do end effector        
        atual_position = forwardKinematics(angles)
        distance = target_position - atual_position

        print(atual_position)
        #print(angles)

        # print(SHOULDER_PITCH_UP_DOWN_MIN + (SHOULDER_PITCH_UP_DOWN_MAX-SHOULDER_PITCH_UP_DOWN_MIN)*(angles[0] - (-math.pi/4))/(math.pi/2), \
        #     SHOULDER_PITCH_UP_DOWN_MIN + (SHOULDER_PITCH_UP_DOWN_MAX-SHOULDER_PITCH_UP_DOWN_MIN)*(angles[0] - (-math.pi/4))/(math.pi/2), \
        #     ELBOW_1_PITCH_UP_MIN + (ELBOW_1_PITCH_UP_MAX-ELBOW_1_PITCH_UP_MIN)*(-angles[0] - (math.pi/2))/(-math.pi/2), \
        #     ELBOW_1_PITCH_DOWN_MIN + (ELBOW_1_PITCH_DOWN_MAX-ELBOW_1_PITCH_DOWN_MIN)*(-angles[0] - (math.pi/4))/(-math.pi/2), \
        #     ELBOW_2_PITCH_UP_DOWN_MIN + (ELBOW_2_PITCH_UP_DOWN_MAX-ELBOW_2_PITCH_UP_DOWN_MIN)*(angles[1] - (-math.pi/4))/(math.pi/2), \
        #     ELBOW_2_PITCH_UP_DOWN_MIN + (ELBOW_2_PITCH_UP_DOWN_MAX-ELBOW_2_PITCH_UP_DOWN_MIN)*(angles[1] - (-math.pi/4))/(math.pi/2), \
        #     GRIPPER_PITCH_MIN + (GRIPPER_PITCH_MAX-GRIPPER_PITCH_MIN)*(angles[2] - (math.pi/4))/(-math.pi/2), \
        #     GRIPPER_YAW_MIN + (GRIPPER_YAW_MAX-GRIPPER_YAW_MIN)*(angles[3])/(-math.pi/2)
        # )
        
        # print("#########################################################")
        # print(serial.joints[1].get_angle(), serial.joints[2].get_angle(), serial.joints[3].get_angle(),\
        #         serial.joints[4].get_angle(), serial.joints[5].get_angle(), serial.joints[6].get_angle(), \
        #         serial.joints[7].get_angle(), serial.joints[8].get_angle())
        # print("#########################################################")
        
        J = calc_jacobian(angles)
        J_inv = np.linalg.pinv(J)
        
        delta_end_effector = ((distance)*data.step_size)/np.max(max(abs(distance)))
        print("End effector: "+str(delta_end_effector))

        delta_angles = J_inv.dot(delta_end_effector)
        
        if max(delta_angles) > 0.1:
            while(max(delta_angles) > 0.1):
                delta_angles = delta_angles/10
                print("adsfasd")

        print(angles)
        angles += delta_angles	
        print("#########################################################")
        print(delta_angles)
        print("#########################################################")
        # backup_gripper_roll = angles[4]
        # backup_gripper_yaw = angles[3]
        # backup_gripper_pitch = angles[2]
        
        serial.joints[0].send_angle(180.0)
        # print((SHOULDER_PITCH_UP_DOWN_MIN + (SHOULDER_PITCH_UP_DOWN_MAX-SHOULDER_PITCH_UP_DOWN_MIN)*(angles[1] - (-math.pi/4))/(math.pi/2)))
        serial.joints[1].send_angle(SHOULDER_PITCH_UP_DOWN_MIN + (SHOULDER_PITCH_UP_DOWN_MAX-SHOULDER_PITCH_UP_DOWN_MIN)*(angles[0] - (-math.pi/4))/(math.pi/2))
        serial.joints[2].send_angle(SHOULDER_PITCH_UP_DOWN_MIN + (SHOULDER_PITCH_UP_DOWN_MAX-SHOULDER_PITCH_UP_DOWN_MIN)*(angles[0] - (-math.pi/4))/(math.pi/2)) 
        serial.joints[3].send_angle(ELBOW_1_PITCH_UP_MIN + (ELBOW_1_PITCH_UP_MAX-ELBOW_1_PITCH_UP_MIN)*(-angles[0] - (math.pi/2))/(-math.pi/2))
        serial.joints[4].send_angle(ELBOW_1_PITCH_DOWN_MIN + (ELBOW_1_PITCH_DOWN_MAX-ELBOW_1_PITCH_DOWN_MIN)*(-angles[0] - (math.pi/4))/(-math.pi/2))
        serial.joints[5].send_angle(ELBOW_2_PITCH_UP_DOWN_MIN + (ELBOW_2_PITCH_UP_DOWN_MAX-ELBOW_2_PITCH_UP_DOWN_MIN)*(angles[1] - (-math.pi/4))/(math.pi/2))
        serial.joints[6].send_angle(ELBOW_2_PITCH_UP_DOWN_MIN + (ELBOW_2_PITCH_UP_DOWN_MAX-ELBOW_2_PITCH_UP_DOWN_MIN)*(angles[1] - (-math.pi/4))/(math.pi/2))
        serial.joints[7].send_angle(GRIPPER_PITCH_MIN + (GRIPPER_PITCH_MAX-GRIPPER_PITCH_MIN)*(angles[2] - (math.pi/4))/(-math.pi/2))
        serial.joints[8].send_angle(GRIPPER_YAW_MIN + (GRIPPER_YAW_MAX-GRIPPER_YAW_MIN)*(angles[3])/(-math.pi/2))
        # serial.joints[9].send_angle(180)        		
        #print ([read_angles[0], read_angles[1], read_angles[5], read_angles[7], read_angles[8], read_angles[9]])
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
    