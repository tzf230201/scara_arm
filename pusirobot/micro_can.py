import can
import time
import struct
import math
import numpy as np
import getpass

# Mendapatkan waktu saat ini dalam detik sejak epoch
last_time = time.time()

user = getpass.getuser()
if user == "peter":
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

def stepper_pulses_to_steps(pulses):
    steps = int(int((pulses * (MICROSTEP * 200)) / 4096))
    return steps

def stepper_steps_to_pulses(steps):
    pulses = int((steps / (MICROSTEP * 200)) * 4096)
    return pulses





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

######################################## STATUS ######################################

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

def get_controller_status(node_id):
    controller_status = req_sdo(node_id, OD_STEPPER_CONTROLLER_STATUS, 0x00)
    return controller_status

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



def read_pvt_mode_arrival_status():
    global is_motor_2_fifo_empty, is_motor_3_fifo_empty, is_motor_4_fifo_empty
    global stop_watch, time_out
    
    is_empty = is_motor_2_fifo_empty and is_motor_3_fifo_empty and is_motor_4_fifo_empty
    is_time_out = (time.time() - stop_watch) > time_out
    
    arrival_status = is_empty or is_time_out
    
    return arrival_status

def read_sp_mode_arrival_status():
    global is_motor_2_busy, is_motor_3_busy, is_motor_4_busy
    global stop_watch, time_out
    
    is_not_busy = not (is_motor_2_busy or is_motor_3_busy or is_motor_4_busy)
    is_time_out = (time.time() - stop_watch) > time_out
    arrival_status = is_not_busy or is_time_out
    
    return arrival_status
        

##################################################################
##################################################################     
##################################################################     
##################################################################       
##################################################################     
def save_settings():
    #save settings
    for id in [ID2, ID3, ID4]:
        set_sdo(id, SET_1_BYTE, OD_STEPPER_SYSTEM_CONTROL, 0x00, 2)
        

#X
def on_closing():
    print("closing")
    shutdown()
    bus.shutdown()

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
    
    # cur_coor = forward_kinematics(motor_angles)
    
    # cur_x, cur_y, cur_z, cur_yaw = cur_coor
    
    # print(f"cur coor : x:{cur_x:.1f} mm, y:{cur_y:.1f} mm, z:{cur_z:.1f} mm, yaw:{cur_yaw:.1f} degree")
    # print(f"cur joint : m2:{m2_angle:.1f}, m3:{m3_angle:.1f}, m4:{m4_angle:.1f} degree")
    is_sp_mode_arrive = read_sp_mode_arrival_status()
    delta_time = time.time() - last_time
    formatted_angles = ", ".join([f"{angle:.2f}" for angle in motor_angles])
    print(f"cur joint : {formatted_angles} degree")
    print(f"time : {delta_time:.2f}, is sp mode arrive : {is_sp_mode_arrive}")
    return motor_angles

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