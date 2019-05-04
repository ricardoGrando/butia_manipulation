import sys
import rospy
import roslib
from std_msgs.msg import String
import threading
import random
import time
from random import randint

from topicCartesianState import *
from std_msgs.msg import Float64
from std_srvs.srv import Empty

import numpy as np

PUBLISHER_RATE_SLEEP = 2

pubList =  [    '/doris/shoulder_yaw_joint_position_controller/command',
                '/doris/shoulder_pitch_joint_up_position_controller/command',
                '/doris/shoulder_pitch_joint_down_position_controller/command',
                '/doris/shoulder_elbow_joint_position_controller/command',
                '/doris/elbow_pitch_joint_up_position_controller/command',
                '/doris/elbow_pitch_joint_down_position_controller/command',
                '/doris/gripper_pitch_joint_position_controller/command',
                '/doris/gripper_yaw_joint_position_controller/command',
                '/doris/gripper_roll_joint_position_controller/command',
                '/doris/gripper_right_joint_position_controller/command',
                '/doris/gripper_left_joint_position_controller/command'
            ]

value = [   [0, 1.5, -1.5, 0, 0, 0, 0, 0] 
            #[0.1, 0.1, 0, 0, 0, 0, 0.5, -0.5],
            #[0.2, 0.2, 0, 0, 0, 0, 0.5, -0.5],
            #[0.3, 0.3, 0, 0, 0, 0, 0.5, -0.5],
            #[0.4, 0.4, 0, 0, 0, 0, 0.5, -0.5],
            #[0.5, 0.5, 0, 0, 0, 0, 0.5, -0.5]            
]

class main(object):

    def __init__(self):
        self.linkThreads = []
        self.mutex = threading.Condition()

        for i in range(0, len(pubList)):
            self.linkThreads.append(topicCartesianState(pubList[i], PUBLISHER_RATE_SLEEP, self.mutex, False))
                    
            # init node
            rospy.init_node('cartesianService', anonymous = True)  

        for i in range(0, len(pubList)):
            self.linkThreads[i].start()

        k = 0
        p = 0        
        
        while True:        
            
            self.mutex.acquire()
            time.sleep(0.07)
            publishersFlag = True
            for i in range(0, len(self.linkThreads)):
                if self.linkThreads[i].flag == True:
                    publishersFlag = False
                    break    
            
            if publishersFlag == True:                
           
                self.linkThreads[1].setValue(0.78)      
                self.linkThreads[2].setValue(0.78)  
                self.linkThreads[3].setValue(-0.78)     
                self.linkThreads[4].setValue(-0.78)
                self.linkThreads[5].setValue(-0.78)   
                self.linkThreads[6].setValue(0.78)   
                self.linkThreads[7].setValue(k/4)   
                self.linkThreads[8].setValue(k/2)
                self.linkThreads[9].setValue(k*2)
                self.linkThreads[10].setValue(k-0.78)
                self.linkThreads[11].setValue(-k+0.78)      
                
                if k >= 0.78 or k <= -0.78:
                    p += 1                                     

                if p % 2 == 0:
                    k += 0.02
                else:
                    k += -0.02

                for i in range(0, len(self.linkThreads)):                                        
                    self.linkThreads[i].setFlag()
                
                self.mutex.notify_all()  

            else:
                self.mutex.wait() 
            
            self.mutex.release()        

        for i in range(0, len(linkList)):
            self.linkThreads[i].join()

m = main()