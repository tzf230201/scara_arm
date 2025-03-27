import time
import math
from micro_can import *

# Sync PVT commands
def pvt_mode_start_pvt_step(group_id):
    send_can_command(f"000#0B{group_id:02X}")
    
def pvt_mode_stop_pvt_step(group_id):
    send_can_command(f"000#0C{group_id:02X}")
    

# PVT Parameters Index 0x01
def pvt_mode_stop_pvt_motion(node_id):
    set_sdo(node_id, SET_1_BYTE, OD_STEPPER_PVT_MOTION, 0x01, 0x00)

def pvt_mode_start_pvt_motion(node_id):
    set_sdo(node_id, SET_1_BYTE, OD_STEPPER_PVT_MOTION, 0x01, 0x01)

def pvt_mode_store_queue():
    for node_id in [ID2, ID3, ID4]:
        set_sdo(node_id, SET_1_BYTE, OD_STEPPER_PVT_MOTION, 0x01, 0x02)
                
def pvt_mode_reset_queue():
    for node_id in [ID2, ID3, ID4]:
        set_sdo(node_id, SET_1_BYTE, OD_STEPPER_PVT_MOTION, 0x01, 0x03)


# PVT Parameters Index 0x02 - 0x04

def pvt_mode_set_pvt_operation_mode(pvt_operation_mode):
    for node_id in [ID2, ID3, ID4]:
        _, ret = set_req_sdo(node_id, SET_1_BYTE, OD_STEPPER_PVT_MOTION, 0x02, pvt_operation_mode)
    #print(f"set in PVT mode -> operation mode is mode {ret + 1}")

def pvt_mode_set_pvt_max_point(max_point):
    for node_id in [ID2, ID3, ID4]:
        _, ret = set_req_sdo(node_id, SET_2_BYTE, OD_STEPPER_PVT_MOTION, 0x03, max_point)
    #print(f"set max PVT points set to {ret}")

def pvt_mode_read_index():
    for node_id in [ID2, ID3, ID4]:
        ret = req_sdo(node_id, OD_STEPPER_PVT_MOTION, 0x04)
        print(f"Node {node_id:03X} PVT current index is {ret}")
        

# PVT1's Parameters 0x05 - 0x06

def pvt_mode_set_pvt_1_start(start_idx):
    for node_id in [ID2, ID3, ID4]:
        _, ret = set_req_sdo(node_id, SET_2_BYTE, OD_STEPPER_PVT_MOTION, 0x05, start_idx)
        print(f"Node {node_id:03X} PVT start index set to {ret}")


def pvt_mode_set_pvt_1_end(end_idx):
    for node_id in [ID2, ID3, ID4]:
        _, ret = set_req_sdo(node_id, SET_2_BYTE, OD_STEPPER_PVT_MOTION, 0x06, end_idx)
        print(f"Node {node_id:03X} PVT stop index set to {ret}")

# PVT2's Parameters 0x07 - 0x0D
#         unused for now


# PVT3's Parameters 0x0E - 0x10
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

# PVT Parameters Index 0x11 - 0x13

def pvt_mode_write_pvt(node_id, pos, vel, tim):

    error_code = ensure_set_req_sdo(node_id, SET_4_BYTE, OD_STEPPER_PVT_MOTION, 0x11, pos)
    error_code |= ensure_set_req_sdo(node_id, SET_4_BYTE, OD_STEPPER_PVT_MOTION, 0x12, vel)
    error_code |= ensure_set_req_sdo(node_id, SET_4_BYTE, OD_STEPPER_PVT_MOTION, 0x13, tim)
    error_code |= set_sdo(node_id, SET_1_BYTE, OD_STEPPER_PVT_MOTION, 0x01, 0x02)
    
    return error_code

def pvt_mode_read_pvt(node_id):
    p = req_sdo(node_id, READ_REQ, OD_STEPPER_PVT_MOTION, 0x11, 0x00)
    v = req_sdo(node_id, READ_REQ, OD_STEPPER_PVT_MOTION, 0x12, 0x00)
    t = req_sdo(node_id, READ_REQ, OD_STEPPER_PVT_MOTION, 0x13, 0x00)
    return p,v,t

def pvt_mode_write_read(node_id, wr_p, wr_v, wr_t):

    arrival_pulse = stepper_steps_to_pulses(wr_p)
    
    error_code = pvt_mode_write_pvt(node_id, wr_p, wr_v, wr_t)
    if (error_code == NO_ERROR):
        print(f"motor{node_id-0x600} pvt wr: {wr_p},{wr_v},{wr_t} will be: {arrival_pulse} -> OK")
    else:
        print(f"motor{node_id-0x600} pvt wr -> ERROR")



