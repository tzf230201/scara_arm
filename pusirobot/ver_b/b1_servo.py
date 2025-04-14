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
OD_SERVO_HOMING_METHOD = 0X6098
OD_SERVO_HOMING_SPEEDS = 0X6099
OD_SERVO_HOMING_ACCELERATION = 0X609A

SERVO_PPR = 10000
SERVO_RATIO = SERVO_PPR / 360

def servo_degrees_to_pulses(degrees):
    return int(degrees * SERVO_RATIO)

def servo_pulses_to_degrees(pulses):
    return float(pulses / SERVO_RATIO)

def servo_pps_to_rps(pulse):
    return (int)((pulse / SERVO_PPR) * 10)

def servo_rps_to_pps(rps):
    return (int)((rps * SERVO_PPR) / 10)



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
    t_accel_ms = t_travel_ms / 2  # Accel and decel time (ms)
    d_accel = d_total / 2  # Distance during accel and decel (pulses)

    # Convert time from ms to seconds for calculations
    t_accel = t_accel_ms / 1000  # Accel and decel time (s)
    
    # Calculate acceleration in pulses per second squared (pps²)
    accel_pps_squared = (2 * d_accel) / (t_accel ** 2)
    
    # Convert acceleration to revolutions per second squared (rps²)
    accel_rps_squared = (int)(abs((accel_pps_squared / SERVO_PPR) * 10))
    
    v_max_rps = (int)(abs(accel_rps_squared * t_accel))

    return accel_rps_squared, v_max_rps









# Mode of Operation Definition
SERVO_OPERATION_MODE = {
	0: "not defined",
	1: "position mode",
	2: "pulse direction mode",
	3: "velocity mode",
	4: "torque mode (servo)",
	5: "not defined",
    6: "homing mode"
	}

def decode_opeation_mode(operation_mode):
    print("Decoded operation mode:")
    for bit, description in SERVO_OPERATION_MODE.items():
        if operation_mode == bit:
            print(f"  - {description}")

def servo_read_operation_mode():
    operation_mode = set_sdo(ID1, READ_REQ, OD_SERVO_MODE_OF_OPERATION, 0x00,  0x00)
    decode_opeation_mode(operation_mode)

def servo_shutdown():
    set_sdo(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x06)

def servo_switch_on():
    set_sdo(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x07)
    
def servo_set_operation_mode(operation_mode):
    set_sdo(ID1, SET_1_BYTE, OD_SERVO_MODE_OF_OPERATION, 0x00,  operation_mode)
    
def servo_goto_operational():
    id = 0x01
    send_can_command(f"000#01{id:02X}")
    
def servo_init():
    servo_status = req_nmt(ID1)
    print(f"servo status (hex): {servo_status:08X}")
    if servo_status == 0x7F:
        servo_goto_operational()
        servo_switch_on()
        # servo_set_operation_mode(1)
        # servo_read_operation_mode()
        # print(f"servo wake_up")

def servo_set_acceleration(accel_1):
    set_sdo(ID1, SET_2_BYTE, OD_SERVO_ACCELERATION, 0x00,  accel_1)
    
def servo_set_deceleration(decel_1):
    set_sdo(ID1, SET_2_BYTE, OD_SERVO_DECELERATION, 0x00,  decel_1)
    
def servo_set_max_speed(max_speed):
    set_sdo(ID1, SET_4_BYTE, OD_SERVO_TARGET_VELOCITY, 0x00,  max_speed)
    
def servo_set_tar_pulse(tar_pulse_1):
    set_sdo(ID1, SET_4_BYTE, OD_SERVO_TARGET_POSITION, 0x00,  tar_pulse_1)

def servo_position_mode(tar_joints):
    
    cur_joint_1, cur_joint_2, cur_joint_3, cur_joint_4 = read_present_position()
    
    tar_joints = check_limit(tar_joints)
    tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4 = tar_joints
    
    tar_pulse_1 = servo_degrees_to_pulses(tar_joint_1)
    delta_pulse_1 = servo_degrees_to_pulses(tar_joint_1 - cur_joint_1)
    accel_decel_1 = servo_accel_decel_calc(delta_pulse_1, travel_time)
    
    set_sdo(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x0F)
    servo_max_speed_ppr = (int)((max_speed / SERVO_PPR) * 10)
    servo_set_acceleration(accel_decel_1)
    servo_set_deceleration(accel_decel_1)
    servo_set_max_speed(servo_max_speed_ppr)
    servo_set_tar_pulse(tar_pulse_1)

    set_sdo(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x1F)
    