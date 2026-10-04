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
def parse_foot_kinematics(message: bytes):
    # Linear segment kinematics (type 21): 40 bytes per segment
    # [segment ID, x, y, z, vx, vy, vz, ax, ay, az], global, Z-Up
    SEGMENT_RIGHT_FOOT = 17
    SEGMENT_LEFT_FOOT = 21

    segments_list = [SEGMENT_RIGHT_FOOT, SEGMENT_LEFT_FOOT]
    packet_size = 40
    base_offsets = np.array(segments_list) * packet_size

    # position (x,y,z), velocity (vx,vy,vz), acceleration (ax,ay,az)
    values = [convert_4_bytes(message=message,base_offsets=base_offsets,add_offsets=4*i,dtype='>f4') for i in range(1,10)]
    values = np.stack(values, axis=1) # rows: segment, cols: x,y,z,vx,vy,vz,ax,ay,az

    right_foot = values[0].tolist()
    left_foot = values[1].tolist()
    return right_foot, left_foot
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
