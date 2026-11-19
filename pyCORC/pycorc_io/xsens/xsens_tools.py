import numpy as np

"""
Base Parse Function
"""
def convert_4_bytes(message:bytes,base_offsets,add_offsets,dtype):
    message_array = np.frombuffer(message, dtype=np.uint8)
    converted_msg = message_array[np.add.outer(base_offsets+add_offsets, np.arange(4))].reshape(-1, 4) # extract the required bytes
    converted_msg = np.frombuffer(converted_msg.tobytes(), dtype=dtype)
    return converted_msg

"""
Parse Functions
"""
def parse_string(message:bytes):
    return message.decode('utf-8')
def parse_COM_kinematics(message:bytes):
    COM_kinematics = np.frombuffer(message, dtype='>f4').tolist()
    return COM_kinematics
# MVN body segments in stream order (index = position in the type-21 packet)
SEGMENT_NAMES = ['Pelvis','L5','L3','T12','T8','Neck','Head',
                 'RightShoulder','RightUpperArm','RightForeArm','RightHand',
                 'LeftShoulder','LeftUpperArm','LeftForeArm','LeftHand',
                 'RightUpperLeg','RightLowerLeg','RightFoot','RightToe',
                 'LeftUpperLeg','LeftLowerLeg','LeftFoot','LeftToe']
SEGMENT_FIELDS = ['x','y','z','vx','vy','vz','ax','ay','az']
SEGMENT_RIGHT_FOOT = 17
SEGMENT_LEFT_FOOT = 21
SEGMENT_KIN_COLUMNS = [f"{seg}_{f}" for seg in SEGMENT_NAMES for f in SEGMENT_FIELDS]

def parse_segment_kinematics(message: bytes):
    # Linear segment kinematics (type 21): 40 bytes per segment
    # [segment ID, x, y, z, vx, vy, vz, ax, ay, az], global, Z-Up
    # returns a (23, 9) float array
    # missing ones are NaN so the logged row width never changes
    n = len(SEGMENT_NAMES)
    raw = np.frombuffer(message, dtype='>f4', count=(len(message)//40)*10).reshape(-1, 10)[:n, 1:]
    if raw.shape[0] < n:
        raw = np.vstack([raw, np.full((n-raw.shape[0], 9), np.nan)])
    return raw.astype(np.float64)

def parse_foot_kinematics(message: bytes):
    # kept for callers that only need the feet: [x,y,z,vx,vy,vz,ax,ay,az] each
    values = parse_segment_kinematics(message)
    return values[SEGMENT_RIGHT_FOOT].tolist(), values[SEGMENT_LEFT_FOOT].tolist()
# MVN joints in stream order (index = position in the type-20 packet): 22 body joints + 6 ergonomic
JOINT_NAMES = ['jL5S1','jL4L3','jL1T12','jT9T8','jT1C7','jC1Head',
               'jRightT4Shoulder','jRightShoulder','jRightElbow','jRightWrist',
               'jLeftT4Shoulder','jLeftShoulder','jLeftElbow','jLeftWrist',
               'jRightHip','jRightKnee','jRightAnkle','jRightBallFoot',
               'jLeftHip','jLeftKnee','jLeftAnkle','jLeftBallFoot',
               'T8_Head','T8_LeftUpperArm','T8_RightUpperArm',
               'Pelvis_T8','Vertical_Pelvis','Vertical_T8']
# raw MVN axes (deg), same as an MVN export: for most joints x = abd/add (lateral bend),
# y = int/ext (axial) rotation, z = flex/ext; shoulders use a different Euler order
JOINT_FIELDS = ['x','y','z']
JOINT_ANGLE_COLUMNS = [f"{j}_{f}" for j in JOINT_NAMES for f in JOINT_FIELDS]

def parse_all_joint_angles(message: bytes):
    # Joint angles (type 20): 20 bytes per joint
    # [parent connection ID, child connection ID, x, y, z]
    # returns a (28, 3) float array; missing joints are NaN so the logged row width never changes
    n = len(JOINT_NAMES)
    raw = np.frombuffer(message, dtype='>f4', count=(len(message)//20)*5).reshape(-1, 5)[:n, 2:]
    if raw.shape[0] < n:
        raw = np.vstack([raw, np.full((n-raw.shape[0], 3), np.nan)])
    return raw.astype(np.float64)

def parse_LL_joint_angle(message: bytes):
    # Compute starting offsets for each joint
    JOINT_RIGHT_HIP = 14
    JOINT_RIGHT_KNEE = 15
    JOINT_RIGHT_ANKLE = 16
    JOINT_LEFT_HIP = 18
    JOINT_LEFT_KNEE = 19
    JOINT_LEFT_ANKLE = 20
    JOINT_TRUNK_B = 27      #vertical to T8

    joints_list = [JOINT_RIGHT_HIP,JOINT_RIGHT_KNEE,JOINT_RIGHT_ANKLE,
                    JOINT_LEFT_HIP,JOINT_LEFT_KNEE,JOINT_LEFT_ANKLE,
                    JOINT_TRUNK_B] # get the order from Analyze Pro
    packet_size = 20
    base_offsets = np.array(joints_list) * packet_size

    x_values = convert_4_bytes(message=message,base_offsets=base_offsets,add_offsets=8,dtype='>f4')
    y_values = convert_4_bytes(message=message,base_offsets=base_offsets,add_offsets=12,dtype='>f4')
    z_values = convert_4_bytes(message=message,base_offsets=base_offsets,add_offsets=16,dtype='>f4')

    # Convert to lists for compatibility
    x_arr = x_values.tolist() #Abd/add  
    y_arr = y_values.tolist() #int/ext
    z_arr = z_values.tolist() #flex/ext

    # extract the joint angles
    JOINT_RIGHT_HIP= 0
    JOINT_RIGHT_KNEE = 1
    JOINT_RIGHT_ANKLE = 2
    JOINT_LEFT_HIP = 3
    JOINT_LEFT_KNEE = 4
    JOINT_LEFT_ANKLE = 5
    JOINT_TRUNK_B = 6

    right_dict = {
        'trunk_ie':y_arr[JOINT_TRUNK_B],
        'trunk_aa':x_arr[JOINT_TRUNK_B],
        'trunk_fe':z_arr[JOINT_TRUNK_B],
        'hip_fe':z_arr[JOINT_RIGHT_HIP],
        'hip_aa':x_arr[JOINT_RIGHT_HIP],
        'hip_ie':y_arr[JOINT_RIGHT_HIP],
        'knee_fe':z_arr[JOINT_RIGHT_KNEE],
        'knee_aa':x_arr[JOINT_RIGHT_KNEE], # probably not useful
        'knee_ie':y_arr[JOINT_RIGHT_KNEE],
        'ankle_fe':z_arr[JOINT_RIGHT_ANKLE],
        'ankle_aa':x_arr[JOINT_RIGHT_ANKLE],
        'ankle_ie':y_arr[JOINT_RIGHT_ANKLE]       
    }

    left_dict = {
        'trunk_ie':y_arr[JOINT_TRUNK_B],
        'trunk_aa':x_arr[JOINT_TRUNK_B],
        'trunk_fe':z_arr[JOINT_TRUNK_B],
        'hip_fe':z_arr[JOINT_LEFT_HIP],
        'hip_aa':x_arr[JOINT_LEFT_HIP],
        'hip_ie':y_arr[JOINT_LEFT_HIP],
        'knee_fe':z_arr[JOINT_LEFT_KNEE],
        'knee_aa':x_arr[JOINT_LEFT_KNEE],
        'knee_ie':y_arr[JOINT_LEFT_KNEE],
        'ankle_fe':z_arr[JOINT_LEFT_ANKLE],
        'ankle_aa':x_arr[JOINT_LEFT_ANKLE],
        'ankle_ie':y_arr[JOINT_LEFT_ANKLE]
    }
    return {"dict":right_dict,"list":list(right_dict.values())},{"dict":left_dict,"list":list(left_dict.values())}
