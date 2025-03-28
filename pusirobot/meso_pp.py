import time
import math
from micro_stepper import *

# pp motion1's parameters 

def pp_mode_set_acceleration(accel_2, accel_3, accel_4):
    for id, accel in zip([ID2, ID3, ID4], [accel_2, accel_3, accel_4]):
        set_sdo(id, SET_4_BYTE, OD_STEPPER_PP_MOTION, 0x01, accel)
        
def pp_mode_set_deceleration(decel_2, decel_3, decel_4):
    for id, decel in zip([ID2, ID3, ID4], [decel_2, decel_3, decel_4]):
        set_sdo(id, SET_4_BYTE, OD_STEPPER_PP_MOTION, 0x02, decel)

def pp_mode_set_start_speed(start_speed):
    for id in [ID2, ID3, ID4]:
        set_sdo(id, SET_4_BYTE, OD_STEPPER_PP_MOTION, 0x03, start_speed)

def pp_mode_set_stop_speed(stop_speed):
    for id in [ID2, ID3, ID4]:
        set_sdo(id, SET_4_BYTE, OD_STEPPER_PP_MOTION, 0x04, stop_speed)
        
# pp motion2's parameters 
        
def pp_mode_get_status():
    status_2 = set_sdo(ID2, READ_REQ, OD_STEPPER_PP_MOTION_2, 0x02, 0x00)
    status_3 = set_sdo(ID3, READ_REQ, OD_STEPPER_PP_MOTION_2, 0x02, 0x00)
    status_4 = set_sdo(ID4, READ_REQ, OD_STEPPER_PP_MOTION_2, 0x02, 0x00)
    return status_2, status_3, status_4

def get_bit(value, bit_position):
    return (value >> bit_position) & 1

def pp_mode_get_arrival_status():
    status_2, status_3, status_4 = pp_mode_get_status()
    reach_2 = get_bit(status_2, 10)
    reach_3 = get_bit(status_3, 10)
    reach_4 = get_bit(status_4, 10)
        
    return reach_2, reach_3, reach_4
    
def pp_mode_set_max_speed(max_speed):
    for id in [ID2, ID3, ID4]:
        set_sdo(id, SET_4_BYTE, OD_STEPPER_PP_MOTION_2, 0x03, max_speed)
        
def pp_mode_set_tar_pulse(tar_pulse_2, tar_pulse_3, tar_pulse_4):
    for id, pulse in zip([ID2, ID3, ID4], [tar_pulse_2, tar_pulse_3, tar_pulse_4]):
        set_sdo(id, SET_4_BYTE, OD_STEPPER_PP_MOTION_2, 0x04, pulse)
        
def pp_mode_start_absolute_motion():
    for id in [ID2, ID3, ID4]:
        send_can_command(f"{id:03X}#2b2e600150000000")

def pp_mode_start_realtive_motion():
    for id in [ID2, ID3, ID4]:
        send_can_command(f"{id:03X}#2b2e600110000000")
        
        
################################## function #############################################3

def pp_mode_init():
    print(f"start pp mode initialization")
    
    init_motor_enable(1)
    init_torque_ring_enable(1)
    init_set_max_current(1500)
    init_microstepping(MICROSTEP)
    init_operation_mode(4)

    pp_mode_set_acceleration(8192, 8192, 8192)
    pp_mode_set_deceleration(8192, 8192, 8192)
    pp_mode_set_start_speed(150)
    pp_mode_set_stop_speed(150)
    
    print(f"the motors is in pp mode")
    
def stepper_accel_decel_calc(d_total, t_travel_ms):
    t_accel_ms = t_travel_ms / 2  # Accel and decel time (ms)
    d_accel = d_total / 2  # Distance during accel and decel (pulses)

    # Convert time from ms to seconds for calculations
    t_accel = t_accel_ms / 1000  # Accel and decel time (s)
    
    # Calculate acceleration and max speed
    accel = (2 * d_accel) / (t_accel ** 2)  # Acceleration (pps^2)
    # v_max = accel * t_accel  # Maximum speed (pps)
    int_accel = (int)(abs(accel))
    # print(int_accel)
    return int_accel
    
def pp_angle(tar_joints, travel_time, max_speed):
    
    cur_joints = read_present_position()
    
    cur_joint_2, cur_joint_3, cur_joint_4 = cur_joints
    tar_joint_2, tar_joint_3, tar_joint_4 = tar_joints
    
    tar_pulse_2 = stepper_degrees_to_pulses(tar_joint_2)
    tar_pulse_3 = stepper_degrees_to_pulses(tar_joint_3)
    tar_pulse_4 = stepper_degrees_to_pulses(tar_joint_4)
    
    delta_pulse_2 = stepper_degrees_to_pulses(tar_joint_2 - cur_joint_2)
    delta_pulse_3 = stepper_degrees_to_pulses(tar_joint_3 - cur_joint_3)
    delta_pulse_4 = stepper_degrees_to_pulses(tar_joint_4 - cur_joint_4)
    
    accel_decel_2 = stepper_accel_decel_calc(delta_pulse_2, travel_time)
    accel_decel_3 = stepper_accel_decel_calc(delta_pulse_3, travel_time)
    accel_decel_4 = stepper_accel_decel_calc(delta_pulse_4, travel_time)
    
    pp_mode_set_acceleration(accel_decel_2, accel_decel_3, accel_decel_4)
    pp_mode_set_deceleration(accel_decel_2, accel_decel_3, accel_decel_4)
    pp_mode_set_max_speed(max_speed)
    pp_mode_set_tar_pulse(tar_pulse_2, tar_pulse_3, tar_pulse_4)
    pp_mode_start_absolute_motion()
    
    