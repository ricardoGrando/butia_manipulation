import PyDynamixel_v2 as pd

SHOULDER_YAW_ID = 254 # to be found

SHOULDER_PITCH_UP_ID = 2 # to be found
SHOULDER_PITCH_DOWN_ID = 1

ELBOW_1_PITCH_UP_ID = 22
ELBOW_1_PITCH_DOWN_ID = 220

ELBOW_2_PITCH_UP_ID = 16
ELBOW_2_PITCH_DOWN_ID = 160

GRIPPER_PITCH_ID = 14
GRIPPER_YAW_ID = 12
GRIPPER_ROLL_ID = 253 # to be found

PORT = '/dev/ttyUSB0'
BAUDRATE = 1000000
serial = pd.DxlComm(port=PORT, baudrate=BAUDRATE)

dict_topics = { SHOULDER_YAW_ID: 'shoulder_yaw_joint_position_controller',
                SHOULDER_PITCH_UP_ID: 'shoulder_pitch_joint_up_position_controller',
                SHOULDER_PITCH_DOWN_ID: 'shoulder_pitch_joint_down_position_controller',
                ELBOW_1_PITCH_UP_ID: 'elbow_1_pitch_joint_up_position_controller',
                ELBOW_1_PITCH_DOWN_ID: 'elbow_1_pitch_joint_down_position_controller',
                ELBOW_2_PITCH_UP_ID: 'elbow_2_pitch_joint_up_position_controller',
                ELBOW_2_PITCH_DOWN_ID: 'elbow_2_pitch_joint_down_position_controller',
                GRIPPER_PITCH_ID: 'gripper_pitch_joint_position_controller',
                GRIPPER_YAW_ID: 'gripper_yaw_joint_position_controller',
                GRIPPER_ROLL_ID: 'gripper_roll_joint_position_controller'
                }

dyns = []
#dyns.append(pd.Joint(SHOULDER_YAW_ID))
#dyns.append(0)
dyns.append(pd.Joint(SHOULDER_PITCH_UP_ID))
dyns.append(pd.Joint(SHOULDER_PITCH_DOWN_ID))
dyns.append(pd.Joint(ELBOW_1_PITCH_UP_ID))
dyns.append(pd.Joint(ELBOW_1_PITCH_DOWN_ID))
dyns.append(pd.Joint(ELBOW_2_PITCH_UP_ID))
dyns.append(pd.Joint(ELBOW_2_PITCH_DOWN_ID))
dyns.append(pd.Joint(GRIPPER_PITCH_ID))
dyns.append(pd.Joint(GRIPPER_YAW_ID))
#dyns.append(pd.Joint(GRIPPER_ROLL_ID))
#dyns.append(0)

serial.attach_joints(dyns)