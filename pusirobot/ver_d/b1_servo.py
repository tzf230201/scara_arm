from b0_can import *

# System Information (RO)
OD_SERVO_DEVICE_TYPE = 0x1000
OD_SERVO_ERROR_REGISTER = 0x1001
OD_SERVO_DEVICE_STATUS_REGISTER = 0x1002 #not available
OD_SERVO_PREDEFINED_ERROR_FIELD = 0x1003
OD_SERVO_COB_ID_SYNC_MSG = 0x1005
OD_SERVO_SYNC_PERIOD = 0x1006
OD_SERVO_SYNC_WINDOW_LENGTH = 0x1007
OD_SERVO_DEVICE_NAME = 0x1008 #not available
OD_SERVO_HARDWARE_VERSION = 0x1009 #not available
OD_SERVO_SOFTWARE_VERSION = 0x100A #not available
OD_SERVO_STORE_PARAMETERS = 0x1010
OD_SERVO_RESET_THE_FAULT_PARAMETERS = 0x1011
OD_SERVO_EMERGENCY_MSG_COB_ID = 0x1014
OD_SERVO_EMERGENCY_MSG_PROHIBITED_TIME = 0x1015 #not available
OD_SERVO_PRODUCER_HEARTH_BEAT = 0x1017
OD_SERVO_OBJECT_IDENTIFIER = 0x1018 #not available
OD_SERVO_SYNC_CNT_OVERFLOW = 0x1019 #not available
OD_SERVO_ERROR_BEHAVIOR = 0x1029 #not available
OD_SERVO_SDO_SERVER_PARAMETERS = 0x1200
OD_SERVO_RPDO_COMMUNICATION_0 = 0x1400
OD_SERVO_RPDO_COMMUNICATION_1 = 0x1401
OD_SERVO_RPDO_COMMUNICATION_2 = 0x1402
OD_SERVO_RPDO_COMMUNICATION_3 = 0x1403
OD_SERVO_RPDO_MAPPING_0 = 0x1600
OD_SERVO_RPDO_MAPPING_1 = 0x1601
OD_SERVO_RPDO_MAPPING_2 = 0x1602
OD_SERVO_RPDO_MAPPING_3 = 0x1603
OD_SERVO_TPDO_COMMUNICATION_0 = 0x1800
OD_SERVO_TPDO_COMMUNICATION_1 = 0x1801
# OD_SERVO_TPDO_COMMUNICATION_2 = 0x1802
# OD_SERVO_TPDO_COMMUNICATION_3 = 0x1803
OD_SERVO_TPDO_MAPPING_0 = 0x1A00
OD_SERVO_TPDO_MAPPING_1 = 0x1A01
# OD_SERVO_TPDO_MAPPING_2 = 0x1A02
# OD_SERVO_TPDO_MAPPING_3 = 0x1A03

# Device Protocol
OD_SERVO_DSP_ERROR_CODE = 0x603F #not available
OD_SERVO_CONTROL_WORD = 0x6040
OD_SERVO_STATUS_WORD = 0x6041
OD_SERVO_QUICK_STOP_CODE = 0x605A
OD_SERVO_HALT_CODE = 0x605D
OD_SERVO_MODE_OF_OPERATION = 0x6060
OD_SERVO_MODE_CODE_RESPONSE = 0X6061
OD_SERVO_POSITION_ACTUAL_VALUE = 0X6064
OD_SERVO_VELOCITY_ACTUAL_VALUE= 0X606C
OD_SERVO_TARGET_POSITION = 0x607A
OD_SERVO_HOME_OFFSET = 0X607C
OD_SERVO_TARGET_VELOCITY = 0X6081
OD_SERVO_ACCELERATION = 0X6083
OD_SERVO_DECELERATION = 0X6084
OD_SERVO_QUICK_STOP_DECELERATION = 0X6085
OD_SERVO_PROFILE_TYPE = 0X6086
OD_SERVO_HOMING_METHOD = 0X6098
OD_SERVO_HOMING_SPEEDS = 0X6099
OD_SERVO_HOMING_ACCELERATION = 0X609A

# 0 :Linear interpolation with a constant time.
# -1 : Linear interpolation with a variable time
# -2 : PVT (Position, Velocity, Time) interpolation
OD_SERVO_INTERPOLATION_SUB_MODE = 0x60C0
#sub_index_1 : position, sub_index_2 : time, sub_index_3 : velocity
OD_SERVO_INTERPOLATION_DATA_RECORD = 0x60C1 
OD_SERVO_IP_SEGMENT_MOVE_COMMAND = 0x2010
OD_SERVO_TRAJECTORY_BUFFER_FREE_COUNT = 0x2011
OD_SERVO_TRAJECTORY_BUFFER_STATUS = 0x2012
OD_SERVO_NEXT_TRAJECTORY_SEGMENT_ID = 0x2013


SERVO_PPR = 131072
SERVO_RATIO = SERVO_PPR / 360

def servo_degrees_to_pulses(degrees):
    return int(degrees * SERVO_RATIO)

def servo_pulses_to_degrees(pulses):
    return float(pulses / SERVO_RATIO)

def servo_pps_to_rps(pulse):
    return (int)((pulse / SERVO_PPR) * 10)

def servo_rps_to_pps(rps):
    return (int)((rps * SERVO_PPR) / 10)


import RPi.GPIO as GPIO

def servo_brake_on():
    GPIO.output(17, GPIO.LOW)
    
def servo_brake_off():
    GPIO.output(17, GPIO.HIGH)

def is_brake_on():
    brake_off = GPIO.input(17)
    return not(brake_off)

    
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
servo_brake_on()



    

def servo_get_motor_position(node_id):
    servo_position = req_sdo(node_id, OD_SERVO_POSITION_ACTUAL_VALUE, 0x00)
    return servo_position

def servo_get_motor_velocity(node_id):
    servo_velocity = req_sdo(node_id, OD_SERVO_VELOCITY_ACTUAL_VALUE, 0x00)
    return servo_velocity

def servo_get_status_word(node_id):
    status_word = req_sdo(node_id, OD_SERVO_STATUS_WORD, 0x00)
    return status_word


def servo_accel_decel_calc(d_total, t_travel_ms):
    """
    Hitung akselerasi dan kecepatan maksimum:
    - accel_scaled: untuk register 0x6083 dan 0x6084 (10 count/s²)
    - v_max_scaled: untuk register 0x60FF (0.1 count/s)
    """
    t_accel = (t_travel_ms / 2) / 1000.0  # seconds
    d_accel = d_total / 2                # counts

    accel_pps2 = (2 * d_accel) / (t_accel ** 2)  # counts/s²
    v_max_pps = accel_pps2 * t_accel             # counts/s

    accel_scaled = int(abs(accel_pps2 / 10))     # 10 count/s²
    v_max_scaled = int(abs(v_max_pps * 10))      # 0.1 count/s

    # print(f"accel_scaled (10 count/s²): {accel_scaled}")
    # print(f"v_max_scaled (0.1 count/s): {v_max_scaled}")
    return accel_scaled, v_max_scaled










# Mode of Operation Definition
SERVO_OPERATION_MODE = {
	0: "not defined",
	1: "Profile Position mode",
	2: "not defined",
	3: "Profile Velocity mode",
	4: "Profile Torque mode",
	5: "not defined",
    6: "homing mode",
    7: "Interpolated Position mode",
    8: "Cyclic Synchronous Position mode",
    9: "Cyclic Synchronous Velocity mode",
    10: "Cyclic Synchronous Torque mode"
	}

def decode_opeation_mode(operation_mode):
    print("Decoded operation mode:")
    for bit, description in SERVO_OPERATION_MODE.items():
        if operation_mode == bit:
            print(f"  - {description}")

def servo_get_operation_mode():
    operation_mode = req_sdo(ID1, OD_SERVO_MODE_OF_OPERATION, 0x00)
    # print(f"Operation mode: {operation_mode:02X}")
    decode_opeation_mode(operation_mode)


SERVO_SUB_MODE = {
	0: "Linear interpolation with a constant time.",
	-1: "Linear interpolation with a variable time",
	-2: "PVT (Position, Velocity, Time) interpolation"
	}

def decode_sub_mode(sub_mode):
    print("Decoded sub mode:")
    for bit, description in SERVO_SUB_MODE.items():
        if sub_mode == bit:
            print(f"  - {description}")

def servo_get_sub_mode():
    sub_mode = req_sdo(ID1, OD_SERVO_INTERPOLATION_SUB_MODE, 0x00)
    # print(f"sub mode: {sub_mode:02X}")
    decode_sub_mode(sub_mode)
    
def servo_shutdown():
    servo_brake_on()
    time.sleep(1)
    set_sdo(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x06)
    send_can_command(f"000#8101")
    
    

def servo_switch_on():
    set_sdo(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x07)
    
    
def servo_set_operation_mode(operation_mode):
    set_sdo(ID1, SET_1_BYTE, OD_SERVO_MODE_OF_OPERATION, 0x00,  operation_mode)
    
def servo_goto_operational():
    id = 0x01
    send_can_command(f"000#01{id:02X}")
    
def servo_init(OPERATION_MODE=1):
    print(f"servo init")
    servo_enable_heartbeat()
    _,val = req_nmt(ID1)
    # print(f"val: {val:02X}")
    if val == 0x7F:
        print(f"servo is not operational, going to operational mode")
        servo_goto_operational()
        servo_switch_on()
        time.sleep(1)
        print(f"ok, servo in oeprational mode")
    elif val == 0x05:
        print(f"servo is already in operational mode")
        
    
    servo_set_operation_mode(OPERATION_MODE)
    servo_get_operation_mode()
    
    if (OPERATION_MODE == 7):
        servo_set_interpolation_sub_mode(-2)
        servo_get_sub_mode()
        servo_get_buffer_free_count()
        servo_get_next_trajectory_segment_id()
    # print(f"servo wake_up")

def servo_set_profile_type(profile_type):
    set_sdo(ID1, SET_2_BYTE, OD_SERVO_PROFILE_TYPE, 0x00,  profile_type)

def servo_set_acceleration(accel_1):
    set_sdo(ID1, SET_4_BYTE, OD_SERVO_ACCELERATION, 0x00,  accel_1)
    
def servo_set_deceleration(decel_1):
    set_sdo(ID1, SET_4_BYTE, OD_SERVO_DECELERATION, 0x00,  decel_1)
    
def servo_set_max_speed(max_speed):
    set_sdo(ID1, SET_4_BYTE, OD_SERVO_TARGET_VELOCITY, 0x00,  max_speed)
    
def servo_set_tar_pulse(tar_pulse_1):
    set_sdo(ID1, SET_4_BYTE, OD_SERVO_TARGET_POSITION, 0x00,  tar_pulse_1)
    
def servo_enable_heartbeat():
    _, ret = set_req_sdo(ID1, SET_2_BYTE, OD_SERVO_PRODUCER_HEARTH_BEAT, 0x00,  1000)  # 1000 ms heartbeat
    print(f"Heartbeat enabled: {ret} ms")
    
def servo_disable_heartbeat():
    _, ret = set_req_sdo(ID1, SET_2_BYTE, OD_SERVO_PRODUCER_HEARTH_BEAT, 0x00,  0)  # 1000 ms heartbeat
    print(f"Heartbeat enabled: {ret} ms (disabled)")
    
def servo_set_interpolation_sub_mode(sub_mode):
    set_sdo(ID1, SET_2_BYTE, OD_SERVO_INTERPOLATION_SUB_MODE, 0x00,  sub_mode)

def servo_set_interpolation_data(position, time, velocity):
    """
    Set interpolation data for servo.
    position: target position in counts
    time: time in ms
    velocity: target velocity in counts/s
    """
    set_sdo(ID1, SET_4_BYTE, OD_SERVO_INTERPOLATION_DATA_RECORD, 0x01,  position)  # sub_index 1: position
    set_sdo(ID1, SET_1_BYTE, OD_SERVO_INTERPOLATION_DATA_RECORD, 0x02,  time)      # sub_index 2: time
    set_sdo(ID1, SET_4_BYTE, OD_SERVO_INTERPOLATION_DATA_RECORD, 0x03,  velocity)  # sub_index 3: velocity
    
    print(f"servo{ID1-0x600} pvt wr: {position},{velocity},{time}")

def servo_get_buffer_free_count():
    """
    Get the number of free segments in the trajectory buffer.
    Returns:
        int: The number of free segments.
    """
    free_count = req_sdo(ID1, OD_SERVO_TRAJECTORY_BUFFER_FREE_COUNT, 0x00)
    print(f"Free trajectory buffer segments: {free_count}")
    return free_count

def servo_get_next_trajectory_segment_id():
    """
    Get the next trajectory segment ID.
    Returns:
        int: The next trajectory segment ID.
    """
    next_id = req_sdo(ID1, OD_SERVO_NEXT_TRAJECTORY_SEGMENT_ID, 0x00)
    print(f"Next trajectory segment ID: {next_id}")
    return next_id

def servo_get_trajectory_buffer_status():
    buffer_status = req_sdo(ID1, OD_SERVO_TRAJECTORY_BUFFER_STATUS, 0x00)
    print(f"trajectory buffer status is: {buffer_status}")
    return buffer_status

# def servo_set_ip_segment_move_command(segment_id, position, time, velocity):
#     """
#     Set the IP segment move command for the servo.
#     segment_id: The ID of the segment to set.
#     position: Target position in counts.
#     time: Time in ms.
#     velocity: Target velocity in counts/s.
#     """
#     set_sdo(ID1, SET_2_BYTE, OD_SERVO_IP_SEGMENT_MOVE_COMMAND, 0x01, segment_id)  # sub_index 1: segment ID
#     set_sdo(ID1, SET_4_BYTE, OD_SERVO_IP_SEGMENT_MOVE_COMMAND, 0x02, position)     # sub_index 2: position
#     set_sdo(ID1, SET_4_BYTE, OD_SERVO_IP_SEGMENT_MOVE_COMMAND, 0x03, time)         # sub_index 3: time
#     set_sdo(ID1, SET_4_BYTE, OD_SERVO_IP_SEGMENT_MOVE_COMMAND, 0x04, velocity)     # sub_index 4: velocity

def servo_pvt_position(cur_po, tar_pos, travel_time):
    """
    Set PVT (Position, Velocity, Time) for servo.
    cur_pos: Current position in counts.
    tar_pos: Target position in counts.
    tar_time: Time in ms.
    tar_vel: Target velocity in counts/s.
    """
    position = tar_pos - cur_pos
    servo_set_interpolation_data(position, tar_time, tar_vel)
    
    # Send the command to start the movement
    set_sdo(ID1, SET_2_BYTE, OD_SERVO_IP_SEGMENT_MOVE_COMMAND, 0x01,  0x01)  # Start the segment move command
    

    
def servo_execute():
    set_sdo(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x1F)
    
    
