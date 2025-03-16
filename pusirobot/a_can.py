import can
import time
import struct
import math
import numpy as np

# Mendapatkan waktu saat ini dalam detik sejak epoch
last_time = time.time()

bus = can.Bus(channel='can0', interface='socketcan')

MICROSTEP = 32

MAX_FAILED_CNT = 20
RECV_WAIT = 0.5

pvt_time_interval = 50

STEPPER_PPR = 4096
SERVO_PPR = 10000
STEPPER_RATIO = STEPPER_PPR / 360
SERVO_RATIO = SERVO_PPR / 360

def stepper_degrees_to_pulses(degrees):
    return int(degrees * STEPPER_RATIO)


def stepper_pulses_to_degrees(pulses):
    return float(pulses / STEPPER_RATIO)


def servo_degrees_to_pulses(degrees):
    return int(degrees * SERVO_RATIO)


def servo_pulses_to_degrees(pulses):
    return float(pulses / SERVO_RATIO)


# System Information (RO)
OD_STEPPER_DEVICE_NAME = 0x1008
OD_STEPPER_HARDWARE_VERSION = 0x1009
OD_STEPPER_SOFTWARE_VERSION = 0x100A

# Communication Configuration
# need to reset power to enable (cansend can0 000#82<CAN_ID>)
OD_STEPPER_NODE_ID = 0x2002
OD_STEPPER_BAUD_RATE = 0x2003
OD_STEPPER_GROUP_ID = 0x2006
OD_STEPPER_SYSTEM_CONTROL = 0x2007

# Motor Control Parameter
OD_STEPPER_ERROR_STATUS = 0x6000
OD_STEPPER_CONTROLLER_STATUS = 0x6001
OD_STEPPER_ROTATION_DIRECTION = 0x6002
OD_STEPPER_MAX_SPEED = 0x6003
OD_STEPPER_RELATIVE_DISPLACEMENT = 0x6004
OD_STEPPER_ABSOLUTE_DISPLACEMENT = 0x601C
OD_STEPPER_STOP_STEPPING = 0x6020
OD_STEPPER_OPERATION_MODE = 0x6005
OD_STEPPER_START_SPEED = 0x6006
INEDX_STOP_SPEED = 0x6007
OD_STEPPER_ACCEL_COEF = 0x6008
OD_STEPPER_DECEL_COEF = 0x6009
OD_STEPPER_MICROSTEPPING = 0x600A
OD_STEPPER_MAX_PHASE_CURRENT = 0x600B
OD_STEPPER_MOTOR_POSITION = 0x600C
OD_STEPPER_CALIBRATION_ZERO = 0x6034
OD_STEPPER_ENCODER_POSITION = 0x6035
OD_STEPPER_CURRENT_REDUCTION = 0x600D
OD_STEPPER_MOTOR_ENABLE = 0X600E
OD_STEPPER_STALL_SET = 0X601B
OD_STEPPER_STALL_PARAMETERS = 0X6017
OD_STEPPER_REAL_TIME_SPEED = 0X6030
OD_STEPPER_EXTERNAL_EMERGENCY_STOP = 0X600F

# CLOSE LOOP CONTROL
OD_STEPPER_ENCODER_RESOLUTION = 0X6021
OD_STEPPER_KP_PARAMETER = 0X6023
OD_STEPPER_KI_PARAMETER = 0X6024
OD_STEPPER_KD_PARAMETER = 0X6025
OD_STEPPER_PRE_FILTERING_PARAMETER = 0X6026
OD_STEPPER_POST_FILTERING_PARAMETER = 0X6027
OD_STEPPER_STALL_LENGTH = 0X6028
OD_STEPPER_TORQUE_RING_ENABLE = 0X6029
OD_STEPPER_AUTOSAVE_WHEN_POWEROFF = 0X602A

# SYNC POSITION MOTION MODE
OD_STEPPER_SP_MOTION = 0X601D
OD_STEPPER_PVT_MOTION = 0X6010
OD_STEPPER_PP_MOTION = 0X602D
OD_STEPPER_PP_MOTION_2 = 0X602E
OD_STEPPER_ANALOG_MODE = 0X602F

# BRAKE CONTROL
OD_STEPPER_BRAKE_CONTROL = 0X6016
OD_STEPPER_ANALOGUE_INPUT = 0X602B

# STEP NOTIFICATION
OD_STEPPER_STEP_NOTIFICATION = 0X602C

#power loss behavior
OD_STEPPER_POWER_LOSS_BEHAVIOR = 0X6031

# PUSIROBOT LIBRARY
SET_1_BYTE = 0x2F
SET_2_BYTE = 0x2B
SET_3_BYTE = 0x27
SET_4_BYTE = 0x23
SET_OK = 0x60
SET_ERROR = 0x80
READ_REQ = 0x40
RECV_1_BYTE = 0x4F
RECV_2_BYTE = 0x4B
RECV_3_BYTE = 0x47
RECV_4_BYTE = 0x43

ID1 = 0x601
ID2 = 0x602
ID3 = 0x603
ID4 = 0x604

response_id_map = {
    ID1: 0x581,
    ID2: 0x582,
    ID3: 0x583,
    ID4: 0x584
}

NO_ERROR = 0x00
MSG_ERROR = 0x01
TIMEOUT_ERROR = 0x02


def send_can_command(command):
    # print(command)
    can_id, can_data = command.split('#')
    can_id = int(can_id, 16)
    can_data = bytes.fromhex(can_data)

    msg = can.Message(arbitration_id=can_id, data=can_data, is_extended_id=False)
    bus.send(msg)
    time.sleep(0.1)


def read_sdo(request_id):
    response_id = response_id_map.get(request_id)
    error_code = NO_ERROR
    can_id = 0
    value = 0
    
    while(can_id != response_id):
        message = bus.recv(RECV_WAIT)  # Wait up to 0.5 seconds for a message
        if message:
            can_id = message.arbitration_id
        else:
            error_code = TIMEOUT_ERROR
            return error_code, value
            
   # print(f"Data CAN Diterima: can id { can_id:03X} data-> {message.data.hex()} (Panjang: {len(message.data)} byte)")
        
    msg = message.data 
    cs = msg[0]
         
    if(cs == SET_ERROR):
        error_code = MSG_ERROR
    else:
        index = (msg[2] << 8) | msg[1]
        sub_index = msg[3]
        value = struct.unpack('<i', msg[4:8])[0]

    return error_code, value

def write_sdo(request_id, cs, index_id, sub_index_id, data):
    # Convert index_id and sub_index_id to bytes
    index_bytes = struct.pack('<H', index_id)
    sub_index_byte = struct.pack('B', sub_index_id)
    
    # Convert data to bytes (assuming 4-byte data for this example)
    data_bytes = struct.pack('<i', data)
    
    # Construct the CAN data payload
    can_data = bytearray()
    can_data.append(cs)                 # Command Specifier
    can_data.extend(index_bytes)        # Index (2 bytes)
    can_data.extend(sub_index_byte)     # Sub-index (1 byte)
    can_data.extend(data_bytes)         # Data (4 bytes)
    
    # Pad the remaining bytes with zeros if necessary to make it 8 bytes
    while len(can_data) < 8:
        can_data.append(0)
    
    # Create and send the CAN message
    msg = can.Message(arbitration_id=request_id, data=can_data, is_extended_id=False)
    bus.send(msg)
    
def set_sdo(request_id, cs, index_id, sub_index_id, data):
    write_sdo(request_id, cs, index_id, sub_index_id, data)
    error_code, ret = read_sdo(request_id)
    
    return error_code

def req_sdo(request_id, index_id, sub_index_id):
    write_sdo(request_id, READ_REQ, index_id, sub_index_id, 0)
    error_code, ret = read_sdo(request_id)
    
    return ret

def set_req_sdo(request_id, cs, index_id, sub_index_id, data):
    error_code = set_sdo(request_id, cs, index_id, sub_index_id, data)
    ret = 0
    if error_code == NO_ERROR:
        ret = req_sdo(request_id, index_id, sub_index_id)

    return error_code, ret

failed_cnt = 0

def ensure_set_req_sdo(request_id, cs, index_id, sub_index_id, data):
    global failed_cnt
    
    error_code, ret = set_req_sdo(request_id, cs, index_id, sub_index_id, data)
    if ( (error_code != NO_ERROR) or (data != ret) ):
        failed_cnt += 1
        
        if (failed_cnt > MAX_FAILED_CNT):
            failed_cnt = 0  # Reset counter after exceeding limit
            return 1
        
        # Call the function recursively
        return ensure_set_req_sdo(request_id, cs, index_id, sub_index_id, data)
    
    failed_cnt = 0  # Reset counter if successful
    
    return 0

def safe_set_sdo(request_id, cs, index_id, sub_index_id, data):
    error_code = ensure_set_req_sdo(request_id, cs, index_id, sub_index_id, data)
    if (error_code != NO_ERROR):
        shutdown()
        print(f"emergency off, something wrong with can bus communication")
    
######################################### INIT #######################################
def init_motor_enable(enable):
    for id in [ID2, ID3, ID4]:
        ensure_set_req_sdo(id, SET_1_BYTE, OD_STEPPER_MOTOR_ENABLE, 0x00, enable)

def init_torque_ring_enable(enable):
    for id in [ID2, ID3, ID4]:
        ensure_set_req_sdo(id, SET_1_BYTE, OD_STEPPER_TORQUE_RING_ENABLE, 0x00, enable)
        
def init_set_max_current(max_current):
    for id in [ID2, ID3, ID4]:
        ensure_set_req_sdo(id, SET_2_BYTE, OD_STEPPER_MAX_PHASE_CURRENT, 0x00, max_current) # the unit is in miliAmpere (mA) (0-6000)
        
def init_microstepping(sub_division):
    for id in [ID2, ID3, ID4]:
        ensure_set_req_sdo(id, SET_2_BYTE, OD_STEPPER_MICROSTEPPING, 0x00, sub_division)
    print(f"microstep set to {sub_division}") 

def init_operation_mode(mode):
    for id in [ID2, ID3, ID4]:
        ensure_set_req_sdo(id, SET_1_BYTE, OD_STEPPER_OPERATION_MODE, 0x00, mode)
    #print(f"set operation mode to {mode}")

def init_change_baudrate(baudrate_option):
    for id in [ID2, ID3, ID4]:
        ensure_set_req_sdo(id, SET_1_BYTE, OD_STEPPER_BAUD_RATE, 0x00, baudrate_option)

def init_change_group_id(group_id):
    for id in [ID2, ID3, ID4]:
        ensure_set_req_sdo(id, SET_1_BYTE, OD_STEPPER_GROUP_ID, 0x00, group_id)
        
def init_set_accel_coef(accel_coef_option):
    for id in [ID2, ID3, ID4]:
        _,ret = set_req_sdo(id, SET_1_BYTE, OD_STEPPER_ACCEL_COEF, 0x00, accel_coef_option)
    print(f"accel coef = {ret}") 
             
def init_set_decel_coef(decel_coef_option):  
    for id in [ID2, ID3, ID4]:
        _,ret = set_req_sdo(id, SET_1_BYTE, OD_STEPPER_DECEL_COEF, 0x00, decel_coef_option)
    print(f"decel coef = {ret}")  
 
def emergency_stop_stepping():
    for id in [ID2, ID3, ID4]:
        set_sdo(id, SET_1_BYTE, OD_STEPPER_STOP_STEPPING, 0x00, 0)
            
def reset_node():
    for id in [0x02, 0x03, 0x04]:
        send_can_command(f"000#81{id:02X}")

def reset_communication():
    for id in [0x02, 0x03, 0x04]:
        send_can_command(f"000#82{id:02X}")
        

def stall_on():
    for id in [ID2, ID3, ID4]:
        set_sdo(id, SET_1_BYTE, OD_STEPPER_STALL_SET, 0x00, 0x01)
        _, ret = set_req_sdo(id, SET_2_BYTE, OD_STEPPER_STALL_LENGTH, 0x00, 64)
    print(f"stall length set to {ret}")
    print(f"stall (open-loop) activated")
    save_settings()


def stall_off():
    for id in [ID2, ID3, ID4]:
        set_sdo(id, SET_1_BYTE, OD_STEPPER_STALL_SET, 0x00, 0x00)
    print(f"stall (open-loop) deactivated, be carefull")
    save_settings()


        
######################################### PVT #######################################
def pvt_mode_stop_pvt_motion():
    for node_id in [ID2, ID3, ID4]:
        set_sdo(node_id, SET_1_BYTE, OD_STEPPER_PVT_MOTION, 0x01, 0x00)


def pvt_mode_start_pvt_motion(node_id):
    set_sdo(node_id, SET_1_BYTE, OD_STEPPER_PVT_MOTION, 0x01, 0x01)

def pvt_mode_store_queue():
    for node_id in [ID2, ID3, ID4]:
        set_sdo(node_id, SET_1_BYTE, OD_STEPPER_PVT_MOTION, 0x01, 0x02)
                
def pvt_mode_reset_queue():
    for node_id in [ID2, ID3, ID4]:
        set_sdo(node_id, SET_1_BYTE, OD_STEPPER_PVT_MOTION, 0x01, 0x03)
        


def pvt_mode_set_pvt_operation_mode(pvt_operation_mode):
    for node_id in [ID2, ID3, ID4]:
        _, ret = set_req_sdo(node_id, SET_1_BYTE, OD_STEPPER_PVT_MOTION, 0x02, pvt_operation_mode)
    #print(f"set in PVT mode -> operation mode is mode {ret + 1}")


def pvt_mode_set_pvt_max_point(max_point):
    for node_id in [ID2, ID3, ID4]:
        _, ret = set_req_sdo(node_id, SET_2_BYTE, OD_STEPPER_PVT_MOTION, 0x03, max_point)
    #print(f"set max PVT points set to {ret}")


def pvt_mode_set_pvt_1_start(start_idx):
    for node_id in [ID2, ID3, ID4]:
        _, ret = set_req_sdo(node_id, SET_2_BYTE, OD_STEPPER_PVT_MOTION, 0x05, start_idx)
        print(f"Node {node_id:03X} PVT start index set to {ret}")


def pvt_mode_set_pvt_1_end(end_idx):
    for node_id in [ID2, ID3, ID4]:
        _, ret = set_req_sdo(node_id, SET_2_BYTE, OD_STEPPER_PVT_MOTION, 0x06, end_idx)
        print(f"Node {node_id:03X} PVT stop index set to {ret}")


def pvt_mode_start_pvt_step(group_id):
    send_can_command(f"000#0B{group_id:02X}")
    
def pvt_mode_stop_pvt_step(group_id):
    send_can_command(f"000#0C{group_id:02X}")



def pvt_mode_read_index():
    for node_id in [ID2, ID3, ID4]:
        ret = req_sdo(node_id, OD_STEPPER_PVT_MOTION, 0x04)
        print(f"Node {node_id:03X} PVT current index is {ret}")

def pvt_mode_write_pvt(node_id, pos, vel, timess):

    error_code = ensure_set_req_sdo(node_id, SET_4_BYTE, OD_STEPPER_PVT_MOTION, 0x11, pos)
    error_code |= ensure_set_req_sdo(node_id, SET_4_BYTE, OD_STEPPER_PVT_MOTION, 0x12, vel)
    error_code |= ensure_set_req_sdo(node_id, SET_4_BYTE, OD_STEPPER_PVT_MOTION, 0x13, timess)
    error_code |= set_sdo(node_id, SET_1_BYTE, OD_STEPPER_PVT_MOTION, 0x01, 0x02)
    
    return error_code

def pvt_mode_read_pvt(node_id):
    p = req_sdo(node_id, READ_REQ, OD_STEPPER_PVT_MOTION, 0x11, 0x00)
    v = req_sdo(node_id, READ_REQ, OD_STEPPER_PVT_MOTION, 0x12, 0x00)
    t = req_sdo(node_id, READ_REQ, OD_STEPPER_PVT_MOTION, 0x13, 0x00)
    return p,v,t

def pvt_mode_write_read(node_id, wr_p, wr_v, wr_t):
    # pvt_mode_read_index()
    wr_p_will_be = int((wr_p/(MICROSTEP* 200)) * 4096)
    # wr_p = (pvt_wb * (MICROSTEP * 200)) / 4096
    

    # wr_p_will_be = int((wr_p * (MICROSTEP* 200)) / 4096)
    # wr_v_will_be = int((wr_v * (MICROSTEP* 200)) / 4096)
    
    
    error_code = pvt_mode_write_pvt(node_id, wr_p, wr_v, wr_t)
    if (error_code == NO_ERROR):
        print(f"motor{node_id-0x600} pvt wr: {wr_p},{wr_v},{wr_t} will be: {wr_p_will_be} -> OK")
    else:
        print(f"motor{node_id-0x600} pvt wr -> ERROR")

def generate_pvt_trajectory(cur_pulse, tar_pulse, travel_time):
    # Define the number of intervals (100ms steps)
    dt = pvt_time_interval / 1000  # 100ms = 0.1s
    num_steps = int(travel_time / dt) + 1
    time_points = np.full(num_steps, pvt_time_interval)

    # Generate sinusoidal acceleration-based motion profile
    t_norm = np.linspace(0, np.pi, num_steps)  # Normalize to sinusoidal shape
    position_points = cur_pulse + (tar_pulse - cur_pulse) * (0.5 * (1 - np.cos(t_norm)))  # Sinusoidal motion

    # Compute velocity by differentiating position
    velocity_points = np.gradient(position_points, dt)

    # Convert to integers
    position_points = np.round(position_points).astype(int)
    velocity_points = np.round(velocity_points).astype(int)
    # time_points = np.round(time_points).astype(int)
    
    # return position_points, velocity_points, time_points
    
    # Filter out small changes in position
    filtered_position_points = []
    filtered_velocity_points = []
    filtered_time_points = []
    last_position = cur_pulse

    tt = 0
    for pos, vel, time in zip(position_points, velocity_points, time_points):
        if abs(pos - last_position) >= STEPPER_RATIO/ 2:  # Half degree threshold
            filtered_position_points.append(pos)
            filtered_velocity_points.append(vel)
            filtered_time_points.append(time + tt)
            last_position = pos
            tt = 0
        else:
            tt += time

    return filtered_position_points, filtered_velocity_points, filtered_time_points



def generate_pvt_trajectory_round_trip(cur_pulse, tar_pulse, time_travel):
    # Define the number of intervals (100ms steps) for one direction
    dt = pvt_time_interval / 1000  # 100ms = 0.1s
    num_steps = int(time_travel / dt) + 1
    time_points_one_way = np.linspace(0, time_travel, num_steps)

    # Generate sinusoidal acceleration-based motion profile for forward motion
    t_norm = np.linspace(0, np.pi, num_steps)  # Normalize to sinusoidal shape
    position_forward = cur_pulse + (tar_pulse - cur_pulse) * (0.5 * (1 - np.cos(t_norm)))  # Sinusoidal motion

    # Compute velocity for forward motion
    velocity_forward = np.gradient(position_forward, dt)

    # Generate return motion using the same profile
    position_backward = position_forward[::-1]  # Reverse the motion
    velocity_backward = -velocity_forward[::-1]  # Reverse the velocity

    # Concatenate forward and backward motion
    time_points = np.concatenate([time_points_one_way, time_travel + time_points_one_way[1:]])
    position_points = np.concatenate([position_forward, position_backward[1:]])
    velocity_points = np.concatenate([velocity_forward, velocity_backward[1:]])
    
    # Convert to integers
    position_points = np.round(position_points).astype(int)
    velocity_points = np.round(velocity_points).astype(int)
    time_points = np.round(time_points).astype(int)


    return position_points, velocity_points, time_points

# def generate_pvt_trajectory_triangle_1(cur_pulse, tar_pulse, travel_time):
#     # Define the number of intervals (100ms steps)
#     dt = pvt_time_interval / 1000  # 100ms = 0.1s
#     num_steps = int(travel_time / dt) + 1
#     time_points = np.linspace(0, travel_time, num_steps)

#     # Generate triangular acceleration-based motion profile
#     t_norm = np.linspace(0, 1, num_steps)  # Normalize to triangular shape
#     position_points = cur_pulse + (tar_pulse - cur_pulse) * t_norm

#     # Compute velocity by differentiating position
#     velocity_points = np.gradient(position_points, dt)

#     # Convert to integers
#     position_points = np.round(position_points).astype(int)
#     velocity_points = np.round(velocity_points).astype(int)
#     time_points = np.round(time_points).astype(int)

#     return position_points, velocity_points, time_points

def generate_pvt_trajectory_triangle_2(cur_pulse, tar_pulse, travel_time):
    # Menghitung interval waktu dt dalam detik
    dt = pvt_time_interval / 1000  # 100 ms = 0.1 s
    num_steps = int(travel_time / dt) + 1
    


    # Hitung total perpindahan dan waktu setengah siklus
    total_pulse = tar_pulse - cur_pulse
    half_time = travel_time / 2

    # Hitung percepatan maksimum (a = 2 * s / t^2)
    max_acc = 2 * total_pulse / (travel_time * half_time)

    # Inisialisasi list untuk menyimpan data
    position_points = []
    velocity_points = []
    time_points = []

    # Inisialisasi variabel
    time = 0.0
    position = cur_pulse

    # Fase akselerasi
    while time < half_time:
        velocity = max_acc * time
        position += velocity * dt
        
        position_points.append(position)
        velocity_points.append(velocity)
        time_points.append(time)
        
        time += dt

    # Fase deselerasi
    while time < travel_time:
        velocity = max_acc * (travel_time - time)
        position += velocity * dt
        
        position_points.append(position)
        velocity_points.append(velocity)
        time_points.append(time)
        
        time += dt

    # Pastikan posisi akhir tepat di target
    position_points.append(tar_pulse)
    velocity_points.append(0.0)
    time_points.append(travel_time)
    
    
    position_points = np.round(position_points).astype(int)
    velocity_points = np.round(velocity_points).astype(int)
    time_points = np.full(num_steps, pvt_time_interval)
    
    return position_points, velocity_points, time_points

    # filtered_position_points = []
    # filtered_velocity_points = []
    # filtered_time_points = []
    # last_position = cur_pulse

    # for pos, vel, time in zip(position_points, velocity_points, time_points):
    #     if abs(pos - last_position) >= STEPPER_RATIO / 2:  # Half degree threshold
    #         filtered_position_points.append(pos)
    #         filtered_velocity_points.append(vel)
    #         filtered_time_points.append(time)
    #         last_position = pos

    # return filtered_position_points, filtered_velocity_points, filtered_time_points


def generate_pvt_trajectory_triangle_3(cur_pulse, tar_pulse, travel_time):
    dt = pvt_time_interval / 1000  # Konversi interval waktu ke detik
    total_pulse = tar_pulse - cur_pulse
    
    # Menghitung percepatan maksimum (a = 4 * s / t^2)
    max_acc = 4 * total_pulse / (travel_time ** 2)
    print(max_acc)
    half_time = travel_time / 2  # Waktu mencapai kecepatan maksimum

    # Variabel untuk menyimpan data
    position_points = []
    velocity_points = []
    time_points = []

    # Fase akselerasi
    for i in range(1, 201):
        curr_time = (i - 1) * dt
        curr_position = 0.5 * max_acc * curr_time ** 2
        curr_velocity = max_acc * curr_time
        
        position_wr = math.floor(curr_position + 0.5) + cur_pulse
        velocity_wr = math.floor(curr_velocity + 0.5)
        
        position_points.append(position_wr)
        velocity_points.append(velocity_wr)
        time_points.append(curr_time)
        
        if curr_time >= half_time:
            break

    last_position = curr_position
    last_velocity = curr_velocity

    # Fase deselerasi
    for i in range(1, 201):
        curr_time = i * dt
        curr_velocity = last_velocity - max_acc * curr_time
        curr_position = last_position + last_velocity * curr_time - 0.5 * max_acc * curr_time ** 2
        
        position_wr = math.floor(curr_position + 0.5) + cur_pulse
        velocity_wr = math.floor(curr_velocity + 0.5)
        
        position_points.append(position_wr)
        velocity_points.append(velocity_wr)
        time_points.append(time_points[-1] + dt)
        
        if abs(curr_velocity) <= 0:
            break
        
    
    position_points = np.round(position_points).astype(int)
    velocity_points = np.round(velocity_points).astype(int)
    position_points_size = len(position_points)
    print(position_points_size)
    time_points = np.full(position_points_size, pvt_time_interval)

    return position_points, velocity_points, time_points
    


def pvt_mode_try_pvt_1(cur_joints, tar_joints, travel_time):
    global last_time
    group_id = 0x05
    tar_pulses = []
    
    pvt_mode_init()
    
    for tar_joint in tar_joints:
        tar_pulses.append(stepper_degrees_to_pulses(tar_joint))
    
    pvt_mode_reset_queue()
    p2, v2, t2 = generate_pvt_trajectory_round_trip(0 , tar_pulses[1], travel_time)
    p3, v3, t3 = generate_pvt_trajectory_round_trip(0 , tar_pulses[2], travel_time)
    p4, v4, t4 = generate_pvt_trajectory_round_trip(0 , tar_pulses[3], travel_time)


    # send_can_command(f"000#{fake_group_id:02X}{ID4-0x600:02X}")
    pt_idx = 0
    for pos, vel in zip(p4, v4):
        # print(f"{pos:.2f}           | {vel:.2f}        | {100}")
        pvt_mode_write_read(ID4, int(pos), int(vel), pvt_time_interval)
        pt_idx += 1
        
    pt_idx = 0
    for pos, vel in zip(p3, v3):
        # print(f"{pos:.2f}           | {vel:.2f}        | {100}")
        pvt_mode_write_read(ID3, int(pos), int(vel), pvt_time_interval)
        pt_idx += 1

    pt_idx = 0
    for pos, vel in zip(p2, v2):
        # print(f"{pos:.2f}           | {vel:.2f}        | {100}")
        pvt_mode_write_read(ID2, int(pos), int(vel), pvt_time_interval)
        pt_idx += 1
    
    
    pvt_mode_read_index()
    pvt_mode_set_pvt_1_start(0)
    pvt_mode_set_pvt_1_end(pt_idx-1)
    pvt_mode_start_pvt_step(group_id)
    # pvt_mode_start_pvt_motion(ID3)
    last_time = time.time()
    
def pvt_mode_read_pvt_3_depth():
    for node_id in [ID2, ID3, ID4]:
        ret = req_sdo(node_id, OD_STEPPER_PVT_MOTION, 0x0E)
        print(f"Node {node_id:03X} PVT3 depth is {ret}")
        
def read_pvt_3_depth(node_id):
    ret = req_sdo(node_id, OD_STEPPER_PVT_MOTION, 0x0E)
    return ret

def pvt_mode_set_pvt_3_fifo_threshold_1(th1):
    for node_id in [ID2, ID3, ID4]:
        _, ret = set_req_sdo(node_id, SET_2_BYTE, OD_STEPPER_PVT_MOTION, 0x0F, th1)
        print(f"Node {node_id:03X} PVT3 lower limit set to {th1}")

def pvt_mode_set_pvt_3_fifo_threshold_2(th2):
    for node_id in [ID2, ID3, ID4]:
        _, ret = set_req_sdo(node_id, SET_2_BYTE, OD_STEPPER_PVT_MOTION, 0x10, th2)
        print(f"Node {node_id:03X} PVT3 upper limit set to {th2}")


def calculate_position_from_velocity(velocity_points, dt, start_position=0):
    position_points = [start_position]
    for vel in velocity_points:
        new_position = position_points[-1] + vel * dt
        position_points.append(new_position)
        
    position_points = np.round(position_points).astype(int)
    
    return position_points

def pvt_mode_try_pvt_3(cur_joints, tar_joints, travel_time):
    global last_time, stop_watch, time_out
    group_id = 0x05
    tar_pulses = []
    node_ids = [ID1, ID2, ID3, ID4]
    pvt_3_lower_limit = 60
    pvt_3_upper_limit = 80
    
    pvt_type = 3
    reset_node()
    time.sleep(1)
    init_operation_mode(0x02)
    init_change_group_id(group_id)
    pvt_mode_set_pvt_max_point(400)
    pvt_mode_set_pvt_operation_mode(pvt_type-1)
    
    pvt_mode_set_pvt_3_fifo_threshold_1(pvt_3_lower_limit)
    pvt_mode_set_pvt_3_fifo_threshold_2(pvt_3_upper_limit)
    
    print(f"pvt init : operation mode pvt, max point 400, pvt mode {pvt_type}")
    
    for tar_joint in tar_joints:
        tar_pulses.append(stepper_degrees_to_pulses(tar_joint))
    
    
    # Initialize p, v, and t as empty lists
    p = [None] * 4
    v = [None] * 4
    t = [None] * 4
    
    #qq
    for i in range(1, 4):
        tar_step = pulse_to_step(tar_pulses[i])
        p[i], v[i], t[i] = generate_pvt_trajectory_triangle_2(0 ,  tar_step, travel_time) 
    
    for i in range(1, 4):
        for pos, vel, tim in zip(p[i], v[i], t[i]):
            pvt_mode_write_read(node_ids[i], pos, vel, tim)
            # print(f"motor {i+1} write {pos}, {vel}, {tim}")
                
                
    pvt_mode_read_pvt_3_depth()
    pvt_mode_start_pvt_step(group_id)
    last_time = time.time()
    stop_watch = last_time
    time_out = travel_time
    
    
            #     last_point = len(p[i])-1
            # if ((cnt == 0) or (cnt ==  1)):# or (cnt==last_point)):
            #     print(f"motor {i+1} no write for first and last point")
            # else:
    
    # for i in range(0, 2):
    #     last_pos = 0
    #     last_vel = 0 
    #     while(depth > pvt_3_lower_limit):
    #         depth = read_pvt_3_depth(ID3)
    #         print(f"depth: {depth}")
    #         read_present_position()
    #         time.sleep(0.1)
    #     for i in range(2, 3):
    #         cnt = 0
    #         for pos, vel in zip(p[i], v[i]):
    #             pos_will_be = int((pos * (MICROSTEP* 200)) / 4096)
    #             if ((cnt < 10) and (pos_will_be == 0)):# or (pos_will_be == last_pos)):
    #                 # pvt_mode_write_read(node_ids[i], int(pos), int(vel), pvt_time_interval)
    #                 print(f"no write")
    #             else:
    #                 pvt_mode_write_read(node_ids[i], int(pos), int(vel), pvt_time_interval)
    #             cnt += 1
    #             last_pos = pos_will_be
    #             last_vel = vel
    #     depth = read_pvt_3_depth(ID3)
        
    # while(depth > 0):
    #     depth = read_pvt_3_depth(ID3)
    #     print(f"depth: {depth}")
    #     read_present_position()
    #     time.sleep(0.1)


    


def read_pdo_1(request_id):
    response_id = (request_id - 0x600) + 0x180
    error_code = NO_ERROR
    can_id = 0
    error_state = 0
    controller_status = 0
    motor_position = 0
    
    while(can_id != response_id):
        message = bus.recv(0.1)  # Wait up to 0.5 seconds for a message
        if message:
            can_id = message.arbitration_id
        else:
            error_code = TIMEOUT_ERROR
            return error_code, error_state, controller_status, motor_position
        
    msg = message.data 
    error_state = msg[0]
    controller_status = msg[1]
    motor_position = struct.unpack('<i', msg[2:6])[0]

    return error_code, error_state, controller_status, motor_position

def extract_controller_status(controller_status):
    controller_status = controller_status & 0xFF
    
    is_stall = ((controller_status >> 2) & 0x01) == 0x01
    is_busy = ((controller_status >> 3) & 0x01) == 0x01
    is_pvt_3_fifo_empty = ((controller_status >> 5) & 0x01) == 0x01
    is_pvt_3_fifo_lower_limit = ((controller_status >> 6) & 0x01) == 0x01
    is_pvt_3_fifo_upper_limit = ((controller_status >> 7) & 0x01) == 0x01
    
    return is_pvt_3_fifo_upper_limit, is_pvt_3_fifo_lower_limit, is_pvt_3_fifo_empty, is_busy, is_stall

stop_watch = 0
time_out = 0
is_motor_2_busy = True
is_motor_3_busy = True
is_motor_4_busy = True
is_motor_2_fifo_empty = False
is_motor_3_fifo_empty = False
is_motor_4_fifo_empty = False

def read_sp_mode_arrival_status():
    global is_motor_2_busy, is_motor_3_busy, is_motor_4_busy
    global stop_watch, time_out
    
    is_not_busy = not (is_motor_2_busy or is_motor_3_busy or is_motor_4_busy)
    is_time_out = (time.time() - stop_watch) > time_out
    arrival_status = is_not_busy or is_time_out
    
    return arrival_status

def read_pvt_mode_arrival_status():
    global is_motor_2_fifo_empty, is_motor_3_fifo_empty, is_motor_4_fifo_empty
    global stop_watch, time_out
    
    is_empty = is_motor_2_fifo_empty and is_motor_3_fifo_empty and is_motor_4_fifo_empty
    is_time_out = (time.time() - stop_watch) > time_out
    
    arrival_status = is_empty or is_time_out
    
    return arrival_status

def gen_circular(cur_pos, center_pos, end_angle, travel_time, direction="CCW"):
    dt = pvt_time_interval / 1000  # Konversi ke detik
    num_steps = int(travel_time / dt) + 1

    start_angle = math.atan2(cur_pos[1] - center_pos[1], cur_pos[0] - center_pos[0])
    end_angle = start_angle + math.radians(end_angle) if direction.upper() == "CCW" else start_angle - math.radians(end_angle)

    # Radius lingkaran
    radius = math.sqrt((cur_pos[0] - center_pos[0]) ** 2 + (cur_pos[1] - center_pos[1]) ** 2)

    # Sudut tiap titik
    theta_points = np.linspace(start_angle, end_angle, num_steps)

    # Hitung posisi x, y berdasarkan sudut
    x_points = center_pos[0] + radius * np.cos(theta_points)
    y_points = center_pos[1] + radius * np.sin(theta_points)
    
    # Nilai konstan untuk z dan yaw
    z_points = np.zeros_like(x_points)
    yaw_points = np.zeros_like(x_points)
    
    trajectory_points = [x_points, y_points, z_points, yaw_points]
    
    return trajectory_points

def generate_pvt_points(joint_pulses):
    dt = pvt_time_interval / 1000  # Konversi ke detik
    
    # Hitung velocity menggunakan numpy gradient
    velocity_points = np.gradient(joint_pulses, dt)

    position_points = np.round(joint_pulses).astype(int)
    velocity_points = np.round(velocity_points).astype(int)
    position_points_size = len(position_points)
    print(position_points_size)
    time_points = np.full(position_points_size, pvt_time_interval)
    
    return position_points, velocity_points, time_points

def pvt_circular(cur_pos, center_pos, end_angle, travel_time, direction="CCW"):
    reset_node()
    time.sleep(2)
    trajectory_points = gen_circular(cur_pos, center_pos, end_angle, travel_time, "CCW")
    trajectory_points_2 = gen_circular(cur_pos, center_pos, end_angle, travel_time, "CW")
    # x_points, y_points, z_points, yaw_points = trajectory_points
    
    joint1_angles, joint2_angles, joint3_angles, joint4_angles = [], [], [], []
    for tar_coor in zip(*trajectory_points):  # zip untuk mengambil titik per titik (x, y, z, yaw)
        try:
            tar_joint = inverse_kinematics(tar_coor)
            joint1, joint2, joint3, joint4 = check_limit(tar_joint)

            joint1_angles.append(joint1)
            joint2_angles.append(joint2)
            joint3_angles.append(joint3)
            joint4_angles.append(joint4)
        except ValueError as e:
            print(f"Inverse kinematics error at {tar_coor}: {e}")
            joint1_angles.append(None)
            joint2_angles.append(None)
            joint3_angles.append(None)
            joint4_angles.append(None)
    for tar_coor in zip(*trajectory_points_2):  # zip untuk mengambil titik per titik (x, y, z, yaw)
        try:
            tar_joint = inverse_kinematics(tar_coor)
            joint1, joint2, joint3, joint4 = check_limit(tar_joint)

            joint1_angles.append(joint1)
            joint2_angles.append(joint2)
            joint3_angles.append(joint3)
            joint4_angles.append(joint4)
        except ValueError as e:
            print(f"Inverse kinematics error at {tar_coor}: {e}")
            joint1_angles.append(None)
            joint2_angles.append(None)
            joint3_angles.append(None)
            joint4_angles.append(None)
    
    # Normalisasi nilai joint agar mulai dari 0
    joint1_angles = [val - joint1_angles[0] for val in joint1_angles]
    joint2_angles = [val - joint2_angles[0] for val in joint2_angles]
    joint3_angles = [val - joint3_angles[0] for val in joint3_angles]
    joint4_angles = [val - joint4_angles[0] for val in joint4_angles]    
    
    #convert to pulses
    joint1_pulses = [stepper_degrees_to_pulses(val) for val in joint1_angles]
    joint2_pulses = [stepper_degrees_to_pulses(val) for val in joint2_angles]
    joint3_pulses = [stepper_degrees_to_pulses(val) for val in joint3_angles]
    joint4_pulses = [stepper_degrees_to_pulses(val) for val in joint4_angles]
    
    joint1_steps = [pulse_to_step(val) for val in joint1_pulses]
    joint2_steps = [pulse_to_step(val) for val in joint2_pulses]
    joint3_steps = [pulse_to_step(val) for val in joint3_pulses]
    joint4_steps = [pulse_to_step(val) for val in joint4_pulses]
    #convert pvt
    
    
    joint1_p, joint1_v, joint1_t = generate_pvt_points(joint1_steps)
    joint2_p, joint2_v, joint2_t = generate_pvt_points(joint2_steps)
    joint3_p, joint3_v, joint3_t = generate_pvt_points(joint3_steps)
    joint4_p, joint4_v, joint4_t = generate_pvt_points(joint4_steps)
    
    global last_time, stop_watch, time_out
    group_id = 0x05
    tar_pulses = []
    node_ids = [ID1, ID2, ID3, ID4]
    pvt_3_lower_limit = 60
    pvt_3_upper_limit = 80
    
    pvt_type = 3
    
    init_operation_mode(0x02)
    init_change_group_id(group_id)
    pvt_mode_set_pvt_max_point(400)
    pvt_mode_set_pvt_operation_mode(pvt_type-1)
    pvt_mode_set_pvt_3_fifo_threshold_1(pvt_3_lower_limit)
    pvt_mode_set_pvt_3_fifo_threshold_2(pvt_3_upper_limit)
    
    print(f"pvt init : operation mode pvt, max point 400, pvt mode {pvt_type}")
    
    #qw
    # Initialize p, v, and t as empty lists
    p = [None] * 4
    v = [None] * 4
    t = [None] * 4
    
    p[0], v[0], t[0] = joint1_p, joint1_v, joint1_t
    p[1], v[1], t[1] = joint2_p, joint2_v, joint2_t
    p[2], v[2], t[2] = joint3_p, joint3_v, joint3_t
    p[3], v[3], t[3] = joint4_p, joint4_v, joint4_t
    
    for i in range(1, 3):
        for pos, vel, tim in zip(p[i], v[i], t[i]):
            pvt_mode_write_read(node_ids[i], pos, vel, tim)
            # print(f"motor {i+1} write {pos}, {vel}, {tim}")
                
                
    pvt_mode_read_pvt_3_depth()
    pvt_mode_start_pvt_step(group_id)
    last_time = time.time()
    stop_watch = last_time
    time_out = travel_time

def pulse_to_step(tar_pulse):
    step = int(int((tar_pulse * (MICROSTEP* 200)) / 4096))
    return step
##################################################################
##################################################################     
##################################################################     
##################################################################       
##################################################################     



#X
def on_closing():
    print("closing")
    shutdown()
    bus.shutdown()

#0
def save_settings():
    #save settings
    for id in [ID2, ID3, ID4]:
        set_sdo(id, SET_1_BYTE, OD_STEPPER_SYSTEM_CONTROL, 0x00, 2)

#1

#2

#3


def wake_up():
    init_motor_enable(1)
    init_torque_ring_enable(1)
    init_set_max_current(3000)
    init_microstepping(MICROSTEP)
    init_set_accel_coef(1)
    init_set_decel_coef(1)
    stall_on()
    print(f"wake_up")
    save_settings()
    
def shutdown():
    emergency_stop_stepping()
    init_motor_enable(0)  
    init_torque_ring_enable(0)  
    init_set_max_current(0)
    reset_node()
    # motor_1_shutdown()
    print(f"motor shutdown")

#13
def pvt_mode_init():
    pvt_type = 1
    reset_node()
    time.sleep(1)
    init_operation_mode(0x02)
    init_change_group_id(0x05)
    pvt_mode_set_pvt_max_point(400)
    pvt_mode_set_pvt_operation_mode(pvt_type-1)
    
    print(f"pvt init : operation mode pvt, max point 400, pvt mode {pvt_type}")

         
           
#19
def read_present_position():
    servo_ids = ID1
    servo_pulse = 0#can_tx(ID1, READ_REQ, OD_SERVO_POSITION_ACTUAL_VALUE, 0)
    servo_angle = stepper_pulses_to_degrees(servo_pulse)
    
    stepper_ids = [ID2, ID3, ID4]
    stepper_angles = []
    
    for stepper_id in stepper_ids:
        stepper_pulse = req_sdo(stepper_id, OD_STEPPER_MOTOR_POSITION, 0x00)
        # print(f"stepper {stepper_id:03X} pulse is {stepper_pulse}")
        stepper_angles.append(stepper_pulses_to_degrees(stepper_pulse))
    
    motor_angles = [servo_angle, stepper_angles[0], stepper_angles[1], stepper_angles[2]]
    
    cur_coor = forward_kinematics(motor_angles)
    
    cur_x, cur_y, cur_z, cur_yaw = cur_coor
    
    print(f"cur coor : x:{cur_x:.1f} mm, y:{cur_y:.1f} mm, z:{cur_z:.1f} mm, yaw:{cur_yaw:.1f} degree")
    # print(f"cur joint : m2:{m2_angle:.1f}, m3:{m3_angle:.1f}, m4:{m4_angle:.1f} degree")
    is_sp_mode_arrive = read_sp_mode_arrival_status()
    delta_time = time.time() - last_time
    formatted_angles = ", ".join([f"{angle:.2f}" for angle in motor_angles])
    print(f"cur joint : {formatted_angles} degree")
    print(f"time : {delta_time:.2f}, is sp mode arrive : {is_sp_mode_arrive}")
    return motor_angles

#20
    
def sp_angle(tar_joints, travel_time):
    global stop_watch, time_out, last_time
    group_id = 5
    init_operation_mode(0)
    #group id need to be set after changing operation mode
    init_change_group_id(group_id)
    # init_set_accel_coef(1)
    # init_set_decel_coef(1)
    
    cur_joints = read_present_position()
    delta_joints = [tar - cur for tar, cur in zip(tar_joints, cur_joints)]
    tar_speeds = [stepper_degrees_to_pulses(int(delta / travel_time)) for delta in delta_joints]
        
    
    
    tar_pulses = []
    for tar_joint in tar_joints:
        tar_pulses.append(stepper_degrees_to_pulses(tar_joint))

    # Set speed
    for id, speed in zip([ID2, ID3, ID4], [tar_speeds[1], tar_speeds[2], tar_speeds[3]]):
        speed_have_to_write = int(int((speed * (MICROSTEP* 200)) / 4096))
        _,ret = set_req_sdo(id, SET_4_BYTE, OD_STEPPER_SP_MOTION, 0x01, speed_have_to_write)
        print(f"{id:03X} sp speed is {ret}")  
    #set position
    for id, pulse in zip([ID2, ID3, ID4], [tar_pulses[1], tar_pulses[2], tar_pulses[3]]):
        _,ret = set_req_sdo(id, SET_4_BYTE, OD_STEPPER_SP_MOTION, 0x02, pulse)
        print(f"{id:03X} sp position is {ret}")
    
    send_can_command(f"000#0A{group_id:02X}")
    last_time = time.time()
    stop_watch = last_time
    time_out = travel_time
    
def sp_coor(tar_coor, travel_time):
    cur_joints = read_present_position()
    tar_joints = inverse_kinematics(tar_coor)
    tar_joints = check_limit(tar_joints)
    print(f"tar joint = {tar_joints} degree")
    sp_angle(tar_joints, travel_time)
    
def check_limit(tar_joints):
    
    tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4 = tar_joints
    tar_joint_4 *= -1
    #limit2 = 0 to 178 degree
    #limit3 = -135 to 0 degree
    #limit4 = 0 to 196 degree
    
    if tar_joint_1 > (13004):
        tar_joint_1 = 13004
        print(f"tar_joint_1 greater than {tar_joint_1}")
    elif tar_joint_1 < 0:
        tar_joint_1 = 0
        print(f"tar_joint_1 lower than {tar_joint_1}")
    
    if tar_joint_2 > (178 * 5):
        tar_joint_2 = 178 * 5
        print(f"tar_joint_2 greater than {tar_joint_2}")
    elif tar_joint_2 < 0:
        tar_joint_2 = 0
        print(f"tar_joint_2 lower than {tar_joint_2}")
        
    if tar_joint_3 > (0 + tar_joint_2):
        tar_joint_3 = (0 + tar_joint_2)
        print(f"tar_joint_3 greater than {tar_joint_3}")
    elif tar_joint_3 < ((-135 * 5) + tar_joint_2):
        tar_joint_3 = (-135 * 5) + tar_joint_2
        print(f"tar_joint_3 lower than {tar_joint_3}")
        
    if tar_joint_4 > ((196 * 5)+tar_joint_3):
        tar_joint_4 = (196 * 5) + tar_joint_3
        print(f"tar_joint_4 greater than {tar_joint_4}")
    elif tar_joint_4 < (0 + tar_joint_3):
        tar_joint_4 = 0 + tar_joint_3
        print(f"tar_joint_4 lower than {tar_joint_4}")
    
    tar_joint_4 *= -1
    
    tar_joints = [tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4]
    
    return tar_joints

def inverse_kinematics(tar_coor):
    x, y, z, yaw = tar_coor
    # max area
    L2 = 137.0
    L3 = 121.0
    L4 = 56.82
    OFFSET_2 = -96.5
    OFFSET_3 = 134
    OFFSET_4 = -52.5
    #ration joint = 5:1

    distance = math.sqrt(x**2 + y**2)

    if distance > (L2 + L3):
        raise ValueError("out of boundary")

    # joint_3
    cos_theta3 = (x**2 + y**2 - L2**2 - L3**2) / (2 * L2 * L3)
    theta3 = math.acos(cos_theta3)  #angle in radian

    # joint_2
    k1 = L2 + L3 * cos_theta3
    k2 = L3 * math.sin(theta3)
    theta2 = math.atan2(y, x) - math.atan2(k2, k1)

    # rad to deg
    theta2 = math.degrees(theta2)
    theta3 = math.degrees(theta3)

    joint_1 = z * (360/90)
    joint_2 = (theta2-OFFSET_2)*5
    joint_3 = (theta3-OFFSET_3)*5 + joint_2
    joint_4 = (yaw-OFFSET_4)*5# + joint_3;
    
    if joint_1 > (13004):
        joint_1 = 13004
       # raise ValueError("out of joint_1 max limit")
    elif joint_1 < 0:
        joint_1 = 0
    if joint_2 > (178 * 5):
        joint_2 = 178 * 5
       # raise ValueError("out of joint_2 max limit")
    elif joint_2 < 0:
        joint_2 = 0
        #raise ValueError("out of joint_2 min limit")
    if joint_3 > (0 + joint_2):
        joint_3 = (0 + joint_2)
        #raise ValueError("out of joint_3 max limit")
    elif joint_3 < ((-135 * 5) + joint_2):
        joint_3 = (-135 * 5) + joint_2
        #raise ValueError("out of joint_2 min limit")
    if joint_4 > ((196 * 5) + joint_3):
        joint_4 = (196 * 5) + joint_3
        #raise ValueError("out of joint_4 max limit")
    elif joint_4 < (0 + joint_3):
        joint_4 = (0 + joint_3)
        #raise ValueError("out of joint_2 min limit")
        
    joint_4 *= -1
        
    return [joint_1, joint_2, joint_3, joint_4]

# Forward kinematics function
def forward_kinematics(cur_joints):
    
    cur_deg1, cur_deg2, cur_deg3, cur_deg4 = cur_joints
    cur_deg4 *= -1.0
    #x:258, y:0, c:0 = joint2:482.5, joint3:-187.5, joint4:262.5
    L2 = 137.0
    L3 = 121.0
    L4 = 56.82
    OFFSET_2 = -96.5
    OFFSET_3 = 134.0
    OFFSET_4 = -52.5
    theta2_rad = math.radians((cur_deg2 / 5) + OFFSET_2)
    theta3_rad = math.radians((cur_deg3 / 5) + OFFSET_3 - (cur_deg2 / 5))
    x2 = L2 * math.cos(theta2_rad)
    y2 = L2 * math.sin(theta2_rad)
    x3 = x2 + L3 * math.cos(theta2_rad + theta3_rad)
    y3 = y2 + L3 * math.sin(theta2_rad + theta3_rad)
    z = (cur_deg1/360)*90
    yaw = (cur_deg4 / 5) + OFFSET_4
    
    cur_coor = [x3, y3, z, yaw]  
    return cur_coor
    
#21

def encoder_position():
    enc2 = req_sdo(ID2, OD_STEPPER_ENCODER_POSITION, 0x00)
    enc3 = req_sdo(ID3, OD_STEPPER_ENCODER_POSITION, 0x00)
    enc4 = req_sdo(ID4, OD_STEPPER_ENCODER_POSITION, 0x00)
    print(f"enc: {enc2}, {enc3}, {enc4}")
    
    return enc2, enc3, enc4

def calib_0():
    enc2, enc3, enc4 = encoder_position()
    for id, enc in zip([ID2, ID3, ID4], [enc2, enc3, enc4]):
        error_code = set_sdo(id, SET_4_BYTE, OD_STEPPER_CALIBRATION_ZERO, 0x00, -enc)
    
    save_settings()

# suggestion:
# motor position pada pdo1 sepertinya tidak dibutuhkan, kalo dihapus program kalkulasinya bisa hemat waktu
# coba rangkap data yang dikitim lewat pvt ke pdo saja bisa hemat waktu

#note:
#speed in sp mode is relative to microstep
#reach position information in sp mode can be read from controller status (busy_state)
#pvt mode is more like speed based rather thank position based


#to do:
# 1. robot harus bisa menjalankan s-shape motion
# bagaimana cara ngetes s-shape motion?
# 2. buat pvt mode relative terhadap current position
# 3. pvt mode with sp correction
# 4. make xyz with time control (4D)
# 5. try PP mode again with pulse_to_step() function
# 6. make the third file


#key role:
# Target : The robot must follow a predefined trajectory to pick up boxes.
# must to have: 
# 1. Organic movement (smooth acceleration & deceleration)
#    - make a 4D trajectory tester
#    - try PP mode again with pulse_to_step() function
#               
# 2. Precision & no incremental drift
#    - maybe can be combining with sp mode after
#    - found what cause the drift in PVT mode.
#               
# 3. anomaly detection & activate Emergency response
#    - decide the pin for the servo brake (GPIO2 - Pin3)
#    - read the position frequently, if the motor out of tolerance, activaate the emergency function


# walalu dan dan danali wa qablu tob tobali
# tob tobi tob tob tobi tob tob tobi tob tob tobali
# wasyaqqu syaq syaq syaqali wara'su qa qabaili
# syawa syawa wasyaisu 'alal wara'suqarjali
# wagharradal qimriyasil hulmalalin fi malali