import can
import time
import struct
import math
import numpy as np

# Mendapatkan waktu saat ini dalam detik sejak epoch
last_time = time.time()

bus = can.Bus(channel='can0', interface='socketcan')

MICROSTEP = 16

MAX_FAILED_CNT = 20
RECV_WAIT = 0.5

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
    print(command)
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
        print(f"{id:03X} accel coef = {ret}") 
             
def init_set_decel_coef(decel_coef_option):  
    for id in [ID2, ID3, ID4]:
        _,ret = set_req_sdo(id, SET_1_BYTE, OD_STEPPER_DECEL_COEF, 0x00, decel_coef_option)
        print(f"{id:03X} decel coef = {ret}")  
 
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
        _, ret = set_req_sdo(id, SET_2_BYTE, OD_STEPPER_STALL_LENGTH, 0x00, 256)
        print(f"id {id:03X} stall length set to {ret}")
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
    # pvt_wb = (wr_p/(MICROSTEP* 200)) * 4096
    # wr_p = (pvt_wb * (MICROSTEP * 200)) / 4096
    pvt_wb = int((wr_p * (MICROSTEP* 200)) / 4096)
    error_code = pvt_mode_write_pvt(node_id, pvt_wb, wr_v, wr_t)
    if (error_code == NO_ERROR):
        # print(f"motor{node_id-0x600} pvt wr: {wr_p},{wr_v},{wr_t} and pvt rd: {rd_p},{rd_v},{rd_t} -> OK")
        print(f"motor{node_id-0x600} pvt wr: {pvt_wb},{wr_v},{wr_t} and it will be: {wr_p} -> OK")
    else:
        print(f"motor{node_id-0x600} pvt wr: {pvt_wb},{wr_v},{wr_t} -> ERROR")

def generate_pvt_trajectory(cur_pulse, tar_pulse, travel_time):
    # Define the number of intervals (100ms steps)
    dt = 0.1  # 100ms = 0.1s
    num_steps = int(travel_time / dt) + 1
    time_points = np.linspace(0, travel_time, num_steps)

    # Generate sinusoidal acceleration-based motion profile
    t_norm = np.linspace(0, np.pi, num_steps)  # Normalize to sinusoidal shape
    position_points = cur_pulse + (tar_pulse - cur_pulse) * (0.5 * (1 - np.cos(t_norm)))  # Sinusoidal motion

    # Compute velocity by differentiating position
    velocity_points = np.gradient(position_points, dt)

    return position_points, velocity_points, time_points

def generate_pvt_trajectory_round_trip(cur_pulse, tar_pulse, time_travel):
    # Define the number of intervals (100ms steps) for one direction
    dt = 0.1  # 100ms = 0.1s
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

    # Round values for better readability
    position_points = np.round(position_points, 2)
    velocity_points = np.round(velocity_points, 2)
    time_points = np.round(time_points, 2)

    return position_points, velocity_points, time_points


def pvt_triangle_trajectory(cur_joints, tar_joints, travel_time):
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
        pvt_mode_write_read(ID4, int(pos), int(vel), 100)
        pt_idx += 1
        
    pt_idx = 0
    for pos, vel in zip(p3, v3):
        # print(f"{pos:.2f}           | {vel:.2f}        | {100}")
        pvt_mode_write_read(ID3, int(pos), int(vel), 100)
        pt_idx += 1

    pt_idx = 0
    for pos, vel in zip(p2, v2):
        # print(f"{pos:.2f}           | {vel:.2f}        | {100}")
        pvt_mode_write_read(ID2, int(pos), int(vel), 100)
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

def pvt_mode_try_pvt_3(cur_joints, tar_joints, travel_time):
    global last_time
    group_id = 0x05
    tar_pulses = []
    node_ids = [ID1, ID2, ID3, ID4]
    
    pvt_type = 3
    reset_node()
    time.sleep(2)
    init_operation_mode(0x02)
    init_change_group_id(group_id)
    pvt_mode_set_pvt_max_point(400)
    pvt_mode_set_pvt_operation_mode(pvt_type-1)
    pvt_mode_set_pvt_3_fifo_threshold_1(40)
    pvt_mode_set_pvt_3_fifo_threshold_2(80)
    
    print(f"pvt init : operation mode pvt, max point 400, pvt mode {pvt_type}")
    
    for tar_joint in tar_joints:
        tar_pulses.append(stepper_degrees_to_pulses(tar_joint))
    
    
    
    pvt_mode_reset_queue()
    
    # Initialize p, v, and t as empty lists
    p = [None] * 4
    v = [None] * 4
    t = [None] * 4
    
    for i in range(1, 4):
        p[i], v[i], t[i] = generate_pvt_trajectory_round_trip(0 , tar_pulses[i], travel_time)
    
    
    time_1 = time.time()
    
    
    
    for i in range(0, 2):
        for i in range(1, 4):
            cnt = 0
            for pos, vel in zip(p[i], v[i]):
                if cnt == 0:
                    print(f"pos: {pos}, vel: {vel} will not written")
                else:
                    pvt_mode_write_read(node_ids[i], int(pos), int(vel), 100)
                cnt += 1
            
    time_2 = time.time()
    time.sleep(0.01)
    print(f"write pvt motor 4 time: {(time_2-time_1)*1000:.2f} ms")
        
    depth = read_pvt_3_depth(ID4)
    # pvt_mode_read_pvt_3_depth()
    pvt_mode_start_pvt_step(group_id)
    # pvt_mode_start_pvt_motion(ID4)
    last_time = time.time()
    
    last_pos = 0
    last_vel = 0
    
    for i in range(0, 2):
        while(depth > 40):
            depth = read_pvt_3_depth(ID4)
            print(f"depth: {depth}")
            read_present_position()
            time.sleep(0.1)
        for i in range(1, 4):
            cnt = 0
            for pos, vel in zip(p[i], v[i]):
                if cnt == 0:
                    print(f"pos: {pos}, vel: {vel} will not written")
                else:
                    pvt_mode_write_read(node_ids[i], int(pos), int(vel), 100)
                cnt += 1
                last_pos = pos
                last_vel = vel
        depth = read_pvt_3_depth(ID4)
    
    
    


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

def read_pvt_3_threshold(request_id):
    error_code, error_state, controller_status, motor_position = read_pdo_1(request_id)
    if (error_code == NO_ERROR):
        is_lower_limit = ((controller_status >> 6) & 0x01) == 0x01
        is_upper_limit = ((controller_status >> 7) & 0x01) == 0x01

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
    # print(f"motor shutdown")

#13
def pvt_mode_init():
    pvt_type = 1
    reset_node()
    time.sleep(2)
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
        stepper_angles.append(stepper_pulses_to_degrees(stepper_pulse))
    
    motor_angles = [servo_angle, stepper_angles[0], stepper_angles[1], stepper_angles[2]]
    
    # cur_x, cur_y, cur_z, cur_yaw = forward_kinematics(m1_angle, m2_angle, m3_angle, m4_angle)
    
    # print(f"cur coordinate : x:{cur_x:.1f} mm, y:{cur_y:.1f} mm, z:{cur_z:.1f} mm, yaw:{cur_yaw:.1f} degree")
    # print(f"cur joint : m2:{m2_angle:.1f}, m3:{m3_angle:.1f}, m4:{m4_angle:.1f} degree")
    delta_time = time.time() - last_time
    formatted_angles = ", ".join([f"{angle:.2f}" for angle in motor_angles])
    print(f"cur joint : {formatted_angles} degree -> time : {delta_time:.2f}")
    return motor_angles

#20
def homing():
    group_id = 5
    speed = 1000
    init_operation_mode(0)
    #group id need to be set after changing operation mode
    init_change_group_id(group_id)
    init_set_accel_coef(1)
    init_set_decel_coef(1)

    # Set speed
    for id, speed in zip([ID2, ID3, ID4], [speed, speed, speed]):
        _,ret = set_req_sdo(id, SET_4_BYTE, OD_STEPPER_SP_MOTION, 0x01, speed)
        print(f"{id:03X} sp speed is {ret}")  
    #set position
    for id, pulse in zip([ID2, ID3, ID4], [0, 0, 0]):
        _,ret = set_req_sdo(id, SET_4_BYTE, OD_STEPPER_SP_MOTION, 0x02, pulse)
        print(f"{id:03X} sp position is {ret}")
    
    send_can_command(f"000#0A{group_id:02X}")
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

# motor position pada pdo1 sepertinya tidak dibutuhkan, kalo dihapus program kalkulasinya bisa hemat waktu