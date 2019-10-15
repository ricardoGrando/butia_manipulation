import PyDynamixel_v2 as pd

SHOULDER_YAW_ID = 56 # to be found

SHOULDER_PITCH_UP_ID = 2
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

dyns = []
dyns.append(pd.Joint(SHOULDER_YAW_ID))
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