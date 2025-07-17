from b0_can import *

# System Information (RO)
OD_STEPPER_DEVICE_NAME = 0x1008
OD_STEPPER_HARDWARE_VERSION = 0x1009
OD_STEPPER_SOFTWARE_VERSION = 0x100A
OD_STEPPER_PRODUCER_HEARTH_BEAT = 0x1017

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
OD_STEPPER_STOP_SPEED = 0x6007
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

STEPPER_MAX_CURRENT = 3000

MICROSTEP = 128

STEPPER_PPR = 4096
STEPPER_RATIO = STEPPER_PPR / 360

SP_MODE = 0x00
PVT_MODE = 0x02
PP_MODE = 0x04


def stepper_degrees_to_pulses(degrees):
    return int(degrees * STEPPER_RATIO)

def stepper_pulses_to_degrees(pulses):
    return float(pulses / STEPPER_RATIO)

def stepper_pulses_to_steps(pulses):
    steps = int(int((pulses * (MICROSTEP * 200)) / 4096))
    return steps

def stepper_steps_to_pulses(steps):
    pulses = int((steps / (MICROSTEP * 200)) * 4096)
    return pulses


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

def init_single_motor_operation_mode(id, mode):
    ensure_set_req_sdo(id, SET_1_BYTE, OD_STEPPER_OPERATION_MODE, 0x00, mode)
    
def init_change_baudrate(baudrate_option):
    for id in [ID2, ID3, ID4]:
        ensure_set_req_sdo(id, SET_1_BYTE, OD_STEPPER_BAUD_RATE, 0x00, baudrate_option)

def init_change_group_id(group_id):
    for id in [ID2, ID3, ID4]:
        ensure_set_req_sdo(id, SET_1_BYTE, OD_STEPPER_GROUP_ID, 0x00, group_id)

def init_single_motor_change_group_id(id, group_id):
    ensure_set_req_sdo(id, SET_1_BYTE, OD_STEPPER_GROUP_ID, 0x00, group_id)
        
def init_set_accel_coef(accel_coef_option):
    for id in [ID2, ID3, ID4]:
        _,ret = set_req_sdo(id, SET_1_BYTE, OD_STEPPER_ACCEL_COEF, 0x00, accel_coef_option)
    # print(f"accel coef = {ret}") 
             
def init_set_decel_coef(decel_coef_option):  
    for id in [ID2, ID3, ID4]:
        _,ret = set_req_sdo(id, SET_1_BYTE, OD_STEPPER_DECEL_COEF, 0x00, decel_coef_option)
    # print(f"decel coef = {ret}")  
 
def emergency_stop_stepping():
    for id in [ID2, ID3, ID4]:
        set_sdo(id, SET_1_BYTE, OD_STEPPER_STOP_STEPPING, 0x00, 0)
            
def reset_node():
    for id in [0x02, 0x03, 0x04]:
        send_can_command(f"000#81{id:02X}")

def reset_communication():
    for id in [0x02, 0x03, 0x04]:
        send_can_command(f"000#82{id:02X}")


def save_settings():
    #save settings
    for id in [ID2, ID3, ID4]:
        print(f"save settings for node {id:03X}")
        set_sdo(id, SET_1_BYTE, OD_STEPPER_SYSTEM_CONTROL, 0x00, 2)  

def stall_on():
    for id in [ID2, ID3, ID4]:
        set_sdo(id, SET_1_BYTE, OD_STEPPER_STALL_SET, 0x00, 0x01)
        _, ret = set_req_sdo(id, SET_2_BYTE, OD_STEPPER_STALL_LENGTH, 0x00, 19)
    print(f"stall length set to {ret}")
    print(f"stall (open-loop) activated")
    save_settings()

def stall_off():
    for id in [ID2, ID3, ID4]:
        set_sdo(id, SET_1_BYTE, OD_STEPPER_STALL_SET, 0x00, 0x00)
    print(f"stall (open-loop) deactivated, be carefull")
    save_settings()
    


######################################## STATUS ######################################s
def stepper_get_controller_status(node_id):
    controller_status = req_sdo(node_id, OD_STEPPER_CONTROLLER_STATUS, 0x00)
    return controller_status

def stepper_get_motor_position(node_id):
    motor_position = req_sdo(node_id, OD_STEPPER_MOTOR_POSITION, 0x00)
    return motor_position

def stepper_get_encoder_position(node_id):
    encoder_position = req_sdo(node_id, OD_STEPPER_ENCODER_POSITION, 0x00)
    return encoder_position

def stepper_calibration_zero(node_id):
    encoder = stepper_get_encoder_position(node_id)
    set_sdo(node_id, SET_4_BYTE, OD_STEPPER_CALIBRATION_ZERO, 0x00, -encoder)
    save_settings()

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



def stepper_init():
    init_motor_enable(1)
    init_torque_ring_enable(1)
    init_set_max_current(STEPPER_MAX_CURRENT)
    init_microstepping(MICROSTEP)
    init_set_accel_coef(1)
    init_set_decel_coef(1)
    stall_on()
    # save_settings() #ini udah save dalam fungsi stall_on()
    # print(f"stepper_wake_up")
    
def stepper_shutdown():
    emergency_stop_stepping()
    init_motor_enable(0)  
    init_torque_ring_enable(0)  
    init_set_max_current(0)
    reset_node()
    
def stepper_enable_heartbeat():
    for id in [ID2, ID3, ID4]:
        _, ret = set_req_sdo(id, SET_2_BYTE, OD_STEPPER_PRODUCER_HEARTH_BEAT, 0x00,  1000)  # 1000 ms heartbeat
    print(f"Heartbeat enabled: {ret} ms")
    
def stepper_disable_heartbeat():
    for id in [ID2, ID3, ID4]:
        _, ret = set_req_sdo(id, SET_2_BYTE, OD_STEPPER_PRODUCER_HEARTH_BEAT, 0x00,  0)  # 1000 ms heartbeat
    print(f"Heartbeat enabled: {ret} ms (disabled)")