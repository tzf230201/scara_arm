import time
import math
import numpy as np
from b1_stepper import *

pvt_time_interval = 50

PVT_1 = 0x00
PVT_2 = 0x01
PVT_3 = 0x02

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
    p = req_sdo(node_id, OD_STEPPER_PVT_MOTION, 0x11)
    v = req_sdo(node_id, OD_STEPPER_PVT_MOTION, 0x12)
    t = req_sdo(node_id, OD_STEPPER_PVT_MOTION, 0x13)
    return p,v,t

def pvt_mode_write_read(node_id, wr_p, wr_v, wr_t):

    arrival_pulse = stepper_steps_to_pulses(wr_p)
    
    wr_p = stepper_pulses_to_steps(wr_p)
    wr_v = stepper_pulses_to_steps(abs(wr_v))
    
    
    error_code = pvt_mode_write_pvt(node_id, wr_p, wr_v, wr_t)
    if (error_code == NO_ERROR):
        print(f"motor{node_id-0x600} pvt wr: {wr_p},{wr_v},{wr_t} will be: {arrival_pulse} -> OK")
    else:
        print(f"motor{node_id-0x600} pvt wr -> ERROR")



############################################ function #############################################################

def pvt_mode_init(group_id, pvt_type = PVT_1, pvt_max_point = 400, pvt_3_lower_limit = 40, pvt_3_upper_limit = 80):
    reset_node()
    time.sleep(3)
    init_operation_mode(PVT_MODE)
    init_change_group_id(group_id)
    init_single_motor_change_group_id(ID4, 0x06)
    pvt_mode_set_pvt_max_point(pvt_max_point)
    pvt_mode_set_pvt_operation_mode(pvt_type)
    print(f"pvt init : pvt_mode {pvt_type+1}, max point {pvt_max_point}, pvt mode {pvt_type}")
    pvt_mode_set_pvt_3_fifo_threshold_1(pvt_3_lower_limit)
    pvt_mode_set_pvt_3_fifo_threshold_2(pvt_3_upper_limit)
    print(f"pvt init : lower_limit {pvt_3_lower_limit}, upper_limit {pvt_3_upper_limit}")
    
    
def pvt_mode_get_arrival_status(node_id):
    controller_status = stepper_get_controller_status(node_id)
    _, _, is_pvt_3_fifo_empty, _, _ = extract_controller_status(controller_status)
    return (is_pvt_3_fifo_empty)


########################## TRAJECTORY ################################################################3

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
    travel_time = travel_time / 1000  # Konversi dari ms ke detik
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

    # # Pastikan posisi akhir tepat di target
    # position_points.append(tar_pulse)
    # velocity_points.append(0.0)
    # time_points.append(travel_time)
    
    
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

####################################################### MOTION ##########################################



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
    



def calculate_position_from_velocity(velocity_points, dt, start_position=0):
    position_points = [start_position]
    for vel in velocity_points:
        new_position = position_points[-1] + vel * dt
        position_points.append(new_position)
        
    position_points = np.round(position_points).astype(int)
    
    return position_points

