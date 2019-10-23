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
GRIPPER_ROLL_MAX = 270.0
GRIPPER_ROLL_MIN = 90.0

LEFT_OPEN = 220
RIGHT_OPEN = 50

MAX_GRIPPER_CLOSE = 135

MAX_EFFORT_LEFT = 70
MAX_EFFORT_RIGHT = 1090

finished_topic = rospy.Publisher('/butia_manipulation_kinematics_finished', Bool, queue_size=10)

finished_open_gripper = rospy.Publisher('butia_manipulation_arm_gripper/open/finished', Bool, queue_size=10)  

finished_close_gripper = rospy.Publisher('butia_manipulation_arm_gripper/close/finished', Bool, queue_size=10) 

finished_turn_gripper = rospy.Publisher('butia_manipulation_arm_gripper/turn/finished', Bool, queue_size=10)  

def get_angles():
    read_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    read_angles[0] = ((serial.joints[0].get_angle() - SHOULDER_YAW_UP_DOWN_MIN)/(SHOULDER_YAW_UP_DOWN_MAX-SHOULDER_YAW_UP_DOWN_MIN))*(-math.pi/2) + math.pi/4
    read_angles[1] = ((serial.joints[1].get_angle() - SHOULDER_PITCH_UP_DOWN_MIN)/(SHOULDER_PITCH_UP_DOWN_MAX-SHOULDER_PITCH_UP_DOWN_MIN))*(math.pi/2) + -math.pi/4
    read_angles[2] = ((serial.joints[2].get_angle() - SHOULDER_PITCH_UP_DOWN_MIN)/(SHOULDER_PITCH_UP_DOWN_MAX-SHOULDER_PITCH_UP_DOWN_MIN))*(math.pi/2) + -math.pi/4
    read_angles[3] = ((serial.joints[3].get_angle() - ELBOW_1_PITCH_UP_MIN)/(ELBOW_1_PITCH_UP_MAX-ELBOW_1_PITCH_UP_MIN))*(-math.pi/2) + math.pi/2
    read_angles[4] = ((serial.joints[4].get_angle() - ELBOW_1_PITCH_DOWN_MIN)/(ELBOW_1_PITCH_DOWN_MAX-ELBOW_1_PITCH_DOWN_MIN))*(-math.pi/2) + math.pi/4
    read_angles[5] = ((serial.joints[5].get_angle() - ELBOW_2_PITCH_UP_DOWN_MIN)/(ELBOW_2_PITCH_UP_DOWN_MAX-ELBOW_2_PITCH_UP_DOWN_MIN))*(math.pi/2) + -math.pi/4
    read_angles[6] = ((serial.joints[6].get_angle() - ELBOW_2_PITCH_UP_DOWN_MIN)/(ELBOW_2_PITCH_UP_DOWN_MAX-ELBOW_2_PITCH_UP_DOWN_MIN))*(math.pi/2) + -math.pi/4
    read_angles[7] = ((serial.joints[7].get_angle() - GRIPPER_PITCH_MIN)/(GRIPPER_PITCH_MAX-GRIPPER_PITCH_MIN))*(-math.pi/2) + math.pi/4
    read_angles[8] = ((serial.joints[8].get_angle() - GRIPPER_YAW_MIN)/(GRIPPER_YAW_MAX-GRIPPER_YAW_MIN))*(-math.pi/2)
    read_angles[9] = ((serial.joints[9].get_angle() - GRIPPER_ROLL_MIN)/(GRIPPER_ROLL_MAX-GRIPPER_ROLL_MIN))*(-math.pi/2) + math.pi/4
    read_angles[10] = serial.joints[10].get_angle()
    read_angles[11] = serial.joints[11].get_angle()

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

    counter = 0

    while(max(abs(distance)) > 0.04):
        ############################################################
        read_angles = get_angles()
        angles = np.array([read_angles[1], read_angles[5], read_angles[7], read_angles[8], read_angles[9]]) 

        ############################################################
        # Realizar a cinematica direta para obter a posicao e orientacao cartesiana do end effector        
        atual_position = forwardKinematics(angles)
        distance = target_position - atual_position

        print(atual_position)
        #print(angles)

        print(SHOULDER_PITCH_UP_DOWN_MIN + (SHOULDER_PITCH_UP_DOWN_MAX-SHOULDER_PITCH_UP_DOWN_MIN)*(angles[0] - (-math.pi/4))/(math.pi/2), \
            SHOULDER_PITCH_UP_DOWN_MIN + (SHOULDER_PITCH_UP_DOWN_MAX-SHOULDER_PITCH_UP_DOWN_MIN)*(angles[0] - (-math.pi/4))/(math.pi/2), \
            ELBOW_1_PITCH_UP_MIN + (ELBOW_1_PITCH_UP_MAX-ELBOW_1_PITCH_UP_MIN)*(-angles[0] - (math.pi/4))/(-math.pi/2), \
            ELBOW_1_PITCH_DOWN_MIN + (ELBOW_1_PITCH_DOWN_MAX-ELBOW_1_PITCH_DOWN_MIN)*(-angles[0] - (math.pi/4))/(-math.pi/2), \
            ELBOW_2_PITCH_UP_DOWN_MIN + (ELBOW_2_PITCH_UP_DOWN_MAX-ELBOW_2_PITCH_UP_DOWN_MIN)*(angles[1] - (-math.pi/4))/(math.pi/2), \
            ELBOW_2_PITCH_UP_DOWN_MIN + (ELBOW_2_PITCH_UP_DOWN_MAX-ELBOW_2_PITCH_UP_DOWN_MIN)*(angles[1] - (-math.pi/4))/(math.pi/2), \
            GRIPPER_PITCH_MIN + (GRIPPER_PITCH_MAX-GRIPPER_PITCH_MIN)*(angles[2] - (math.pi/4))/(-math.pi/2), \
            GRIPPER_YAW_MIN + (GRIPPER_YAW_MAX-GRIPPER_YAW_MIN)*(angles[3])/(-math.pi/2), \
            GRIPPER_ROLL_MIN + (GRIPPER_ROLL_MAX-GRIPPER_ROLL_MIN)*(angles[4] - (-math.pi/4))/(math.pi/2)
        )

        print(angles)
        
        # print("#########################################################")
        # print(serial.joints[1].get_angle(), serial.joints[2].get_angle(), serial.joints[3].get_angle(),\
        #         serial.joints[4].get_angle(), serial.joints[5].get_angle(), serial.joints[6].get_angle(), \
        #         serial.joints[7].get_angle(), serial.joints[8].get_angle())
        # print("#########################################################")
        
        J = calc_jacobian(angles)
        J_inv = np.linalg.pinv(J)

        step = data.step_size #- (data.step_size*counter)/50.0
        
        delta_end_effector = ((distance)*step)/np.max(max(abs(distance)))
        # print("End effector: "+str(delta_end_effector))

        delta_angles = J_inv.dot(delta_end_effector)
        
        if max(delta_angles) > 0.1:
            while(max(delta_angles) > 0.1):
                delta_angles = delta_angles/10
                print("adsfasd")

        #print(serial.joints[9].get_angle(), serial.joints[10].get_angle(), serial.joints[11].get_angle())
        print("#########################################################")
        #print(serial.joints[10].get_effort(), serial.joints[11].get_effort())
        angles += delta_angles	
        print("#########################################################")
        # print(delta_angles)
        print("#########################################################")
                
        serial.joints[0].send_angle(180.0)
        # print((SHOULDER_PITCH_UP_DOWN_MIN + (SHOULDER_PITCH_UP_DOWN_MAX-SHOULDER_PITCH_UP_DOWN_MIN)*(angles[1] - (-math.pi/4))/(math.pi/2)))
        if (angles[0] < math.pi/3 and angles[0] > -math.pi/3):
            serial.joints[1].send_angle(SHOULDER_PITCH_UP_DOWN_MIN + (SHOULDER_PITCH_UP_DOWN_MAX-SHOULDER_PITCH_UP_DOWN_MIN)*(angles[0] - (-math.pi/4))/(math.pi/2))
            serial.joints[2].send_angle(SHOULDER_PITCH_UP_DOWN_MIN + (SHOULDER_PITCH_UP_DOWN_MAX-SHOULDER_PITCH_UP_DOWN_MIN)*(angles[0] - (-math.pi/4))/(math.pi/2)) 
            serial.joints[3].send_angle(ELBOW_1_PITCH_UP_MIN + (ELBOW_1_PITCH_UP_MAX-ELBOW_1_PITCH_UP_MIN)*(-angles[0] - (math.pi/2))/(-math.pi/2))
            serial.joints[4].send_angle(ELBOW_1_PITCH_DOWN_MIN + (ELBOW_1_PITCH_DOWN_MAX-ELBOW_1_PITCH_DOWN_MIN)*(-angles[0] - (math.pi/4))/(-math.pi/2))
        if (angles[1] < math.pi/2 and angles[1] > -math.pi/2):
            serial.joints[5].send_angle(ELBOW_2_PITCH_UP_DOWN_MIN + (ELBOW_2_PITCH_UP_DOWN_MAX-ELBOW_2_PITCH_UP_DOWN_MIN)*(angles[1] - (-math.pi/4))/(math.pi/2))
            serial.joints[6].send_angle(ELBOW_2_PITCH_UP_DOWN_MIN + (ELBOW_2_PITCH_UP_DOWN_MAX-ELBOW_2_PITCH_UP_DOWN_MIN)*(angles[1] - (-math.pi/4))/(math.pi/2))
        if (angles[2] < math.pi/2 and angles[2] > -math.pi/2):        
            serial.joints[7].send_angle(GRIPPER_PITCH_MIN + (GRIPPER_PITCH_MAX-GRIPPER_PITCH_MIN)*(angles[2] - (math.pi/4))/(-math.pi/2))
        if (angles[3] < math.pi/6 and angles[3] > -math.pi/6):
            serial.joints[8].send_angle(GRIPPER_YAW_MIN + (GRIPPER_YAW_MAX-GRIPPER_YAW_MIN)*(angles[3])/(-math.pi/2))
        # if (angles[4] < math.pi/2 and angles[4] > -math.pi/2):
        #     print(GRIPPER_ROLL_MIN + (GRIPPER_ROLL_MAX-GRIPPER_ROLL_MIN)*(angles[4] - (-math.pi/4))/(math.pi/2))
        #     #serial.joints[9].send_angle(GRIPPER_ROLL_MIN + (GRIPPER_ROLL_MAX-GRIPPER_ROLL_MIN)*(angles[4] - (-math.pi/4))/(math.pi/2))
        roll_angle = serial.joints[9].get_angle()

        while (roll_angle/180.0 > 1.1):
            serial.joints[9].send_angle(roll_angle + 5.0) 

            roll_angle = serial.joints[9].get_angle()
       		
        #print ([read_angles[0], read_angles[1], read_angles[5], read_angles[7], read_angles[8], read_angles[9]])
        #finished_topic.publish(False)
        raw_input()

        print(read_angles[9])

        counter += 1

        if counter == 70:
            finished_topic.publish(False)
            break

    if counter < 70:    
        print("Arrived!!!")
        finished_topic.publish(True)
        rate.sleep()

def open_gripper_callback(data):
    rate = rospy.Rate(5)
    
    if (data.data == True):
        print("Opening gripper")
        read_angles = get_angles()
        while(read_angles[10] < LEFT_OPEN or read_angles[11] > RIGHT_OPEN):
            if (read_angles[10] < LEFT_OPEN):
                serial.joints[10].send_angle(read_angles[10] + 5.0)

            if (read_angles[11] > RIGHT_OPEN):
                serial.joints[11].send_angle(read_angles[11] - 5.0)

            rate.sleep()

            print(read_angles[10], read_angles[11])

            read_angles = get_angles()

        print("Opened")
        finished_open_gripper.publish(True)

def close_gripper_callback(data):
    rate = rospy.Rate(10)
    
    counter = 1.0

    base_read_angles = get_angles()
    
    if (data.data == True):
        while((serial.joints[10].get_effort() < MAX_EFFORT_LEFT or serial.joints[10].get_effort() > 1000) or (serial.joints[11].get_effort() < MAX_EFFORT_RIGHT or serial.joints[11].get_effort() < 100)):

            if ((serial.joints[10].get_effort() < MAX_EFFORT_LEFT or serial.joints[10].get_effort() > 1000) and serial.joints[10].get_angle() > MAX_GRIPPER_CLOSE+5.0):
                serial.joints[10].send_angle(base_read_angles[10] - counter)     

            if ((serial.joints[11].get_effort() < MAX_EFFORT_RIGHT or serial.joints[11].get_effort() < 100) and serial.joints[11].get_angle() < MAX_GRIPPER_CLOSE+5.0):
                serial.joints[11].send_angle(base_read_angles[11] + counter)  

            counter += 3.0

            read_angles = get_angles()

            rate.sleep()

            print(serial.joints[10].get_effort(), serial.joints[11].get_effort())
            
        print("Grabbed")
        finished_close_gripper.publish(True)

def turn_gripper_callback(data):
    rate = rospy.Rate(10)
    
    print("Turning")
    read_angles = get_angles()

    if (read_angles[9] > data.data):
        while(read_angles[9] > data.data):
            read_angles = get_angles()
            serial.joints[9].send_angle(read_angles[9] - 5.0)
    else:
        while(read_angles[9] < data.data):
            read_angles = get_angles()
            serial.joints[9].send_angle(read_angles[9] + 5.0)

    finished_turn_gripper.publish(True)


def butia_manipulation_control():
    
    rospy.init_node("butia_manipulation_arm_control", anonymous=False)
    
    rospy.Subscriber('/butia_manipulation_arm_target_position', CartesianMsg, inverse_kinematics_callback)
    
    rospy.Subscriber('/butia_manipulation_arm_gripper/open', Bool, open_gripper_callback)

    rospy.Subscriber('/butia_manipulation_arm_gripper/close', Bool, close_gripper_callback)

    rospy.Subscriber('/butia_manipulation_arm_gripper/turn', Float64, turn_gripper_callback)
    
    rospy.spin()

####################################### Exemplo para testar a aplicacao #######################################
# rostopic pub -1 /cartesian_position butia_manipulation_control_msgs/CartesianMsg "target_end_effector: [-17, 2, 20, -2.7, 1.46, 2.67] step_size: 0.1"		

if __name__ == "__main__": 
    butia_manipulation_control()      
    
# [0.60, 0.0, 0.8, 0.0, 0.0, 0.0]
# step_size: 0.02"

# [0.60, 0.0, 1.3, 0.0, 0.0, 0.0]
# step_size: 0.02" 

# [0.50, 0.0, 1.0, 0.0, 0.0, 0.0]
# step_size: 0.02"

# [0.70, 0.0, 1.0, 0.0, 0.0, 0.0]
# step_size: 0.02"