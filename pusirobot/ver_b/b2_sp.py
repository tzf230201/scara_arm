import time
import math
from b1_stepper import *

def sp_mode_set_speed(id, speed):
    error_code, ret = set_req_sdo(id, SET_4_BYTE, OD_STEPPER_SP_MOTION, 0x01, speed)
    return error_code, ret
    
def sp_mode_set_pulse(id, pulse):
    error_code, ret = set_req_sdo(id, SET_4_BYTE, OD_STEPPER_SP_MOTION, 0x02, pulse)
    return error_code, ret
    
def sp_mode_start_motion(group_id):
    send_can_command(f"000#0A{group_id:02X}")
    
def sp_mode_init(group_id):
    init_operation_mode(0)
    #group id need to be set after changing operation mode
    init_change_group_id(group_id)
    init_set_accel_coef(1)
    init_set_decel_coef(1)
    
def sp_mode_get_arrival_status(node_id):
    controller_status = stepper_get_controller_status(node_id)
    _, _, _, is_busy, _ = extract_controller_status(controller_status)
    is_arrive = not is_busy
    return (is_arrive)