import time
import math
from b1_servo import *
from b1_stepper import *
from b2_pvt import *
import numpy as np
# enc: 131744993 -794543, -802111, -802962
#enc: 131876599 -794544, -802109, -802962

import os
import json

def print_red(text):
    # ANSI escape code untuk warna merah
    RED = '\033[31m'  # Kode warna 256-color mode untuk merah
    RESET = '\033[0m'  # Untuk mengembalikan warna ke default
    print(f"{RED}{text}{RESET}")

def print_yellow(text):
    # ANSI escape code untuk warna kuning
    YELLOW = '\033[93m'
    RESET = '\033[0m'  # Untuk mengembalikan warna ke default
    print(f"{YELLOW}{text}{RESET}")

def print_orange(text):
    # ANSI escape code untuk warna kuning yang menyerupai oranye
    ORANGE = '\033[38;5;214m'  # Kode warna 256-color mode untuk oranye
    RESET = '\033[0m'  # Untuk mengembalikan warna ke default
    print(f"{ORANGE}{text}{RESET}")

def load_origin_from_config():
    """
    Load origin values from config_origin.json.
    If file does not exist, create it with all zeros.
    Returns a tuple (origin_1, origin_2, origin_3, origin_4).
    """
    # Get the directory of the current script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "config_origin.json")
    print(f"{config_path}")
    
    default_config = {
         "origin_1": -2864476,
        "origin_2": -807440,
        "origin_3": -802157,
        "origin_4": -792346
    }
    
    try:
        with open(config_path, "r") as f:
            config_data = json.load(f)
        
        origin_1 = config_data.get("origin_1", 0)
        origin_2 = config_data.get("origin_2", 0)
        origin_3 = config_data.get("origin_3", 0)
        origin_4 = config_data.get("origin_4", 0)
        
        print(f"Loaded origins: {origin_1}, {origin_2}, {origin_3}, {origin_4}")
        return origin_1, origin_2, origin_3, origin_4
    
    except FileNotFoundError:
        # File not found, create new file with defaults
        with open(config_path, "w") as f:
            json.dump(default_config, f, indent=4)
        print(f"config_origin.json not found. Created new file with defaults.")
        return 0, 0, 0, 0
    
    except json.JSONDecodeError:
        print("Invalid JSON format in config_origin.json. Returning default (0,0,0,0).")
        return 0, 0, 0, 0

def save_origin_to_config(encoders):
    """
    Save encoder values to config_origin.json as a dictionary with keys: origin_1..origin_4.
    """
    config_data = {
        "origin_1": encoders[0],
        "origin_2": encoders[1],
        "origin_3": encoders[2],
        "origin_4": encoders[3],
    }
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config_origin.json")
    with open(config_path, "w") as f:
        json.dump(config_data, f, indent=4)

    print(f"Origin saved to {config_path}")
    
origins = load_origin_from_config()

def get_origins():
    global origins
    
    return origins

def set_origins(encoders):
    global origins
    
    # Save the new origins to the config file
    save_origin_to_config(encoders)
    
    # Update the global origins variable
    origins = encoders
    
    print(f"Origins updated: {origins}")
    
    
    

def get_cur_joints(selection):
    
    origins = get_origins()       
    
    servo_id = ID1
    
    if selection != "stepper_only":  
        servo_pulse = servo_get_motor_position(servo_id) - origins[0]
    else:
        servo_pulse = 0
        
    servo_angle = servo_pulses_to_degrees(servo_pulse)
    
    stepper_ids = [ID2, ID3, ID4]
    stepper_angles = []
    
    for stepper_id in stepper_ids:
        if selection != "servo_only":  
            stepper_pulse = stepper_get_motor_position(stepper_id)
        else:
            stepper_pulse = 0
        
        # print(f"stepper {stepper_id:03X} pulse is {stepper_pulse}")
        stepper_angles.append(stepper_pulses_to_degrees(stepper_pulse))
    
    cur_joints = [servo_angle, stepper_angles[0], stepper_angles[1], stepper_angles[2]]
    return cur_joints

# ######################################### SP MODE ######################################### #

from b2_sp import * 

def sp_angle(tar_joints, travel_time, selection):
    travel_time_s = (travel_time / 1000)  # Convert milliseconds to seconds
    global stop_watch, time_out, last_time
    group_id = 5
    sp_mode_init(group_id)
    
    cur_joints = get_cur_joints(selection)
    delta_joints = [tar - cur for tar, cur in zip(tar_joints, cur_joints)]
    tar_speeds = [stepper_degrees_to_pulses(int(delta / travel_time_s)) for delta in delta_joints]
    
    tar_pulses = []
    for tar_joint in tar_joints:
        tar_pulses.append(stepper_degrees_to_pulses(tar_joint))

    # Set speed
    for id, speed in zip([ID2, ID3, ID4], [tar_speeds[1], tar_speeds[2], tar_speeds[3]]):
        #in sp mode, speed is not pulse per second, but step per second
        speed_have_to_write = stepper_pulses_to_steps(speed)
        _,ret = sp_mode_set_speed(id, speed_have_to_write)
        print(f"{id:03X} sp speed is {ret}")  
    #set position
    for id, pulse in zip([ID2, ID3, ID4], [tar_pulses[1], tar_pulses[2], tar_pulses[3]]):
        _,ret = sp_mode_set_pulse(id, pulse)
        print(f"{id:03X} sp position is {ret}")
    
    sp_mode_start_motion(group_id)
    last_time = time.time()
    stop_watch = last_time
    time_out = travel_time
    
def sp_coor(tar_coor, travel_time, selection):
    tar_joints = inverse_kinematics(tar_coor)
    tar_joints = check_limit(tar_joints)
    print(f"tar joint = {tar_joints} degree")
    sp_angle(tar_joints, travel_time, selection)

    

# ######################################### PP MODE ######################################### #
from b2_pp import *

def pp_angle_servo(tar_joints, travel_time, selection):
    
    origins = get_origins()
        
    cur_joints = get_cur_joints(selection)
    
    cur_joint_1, cur_joint_2, cur_joint_3, cur_joint_4 = cur_joints
    tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4 = tar_joints
    
    tar_pulse_1 = servo_degrees_to_pulses(tar_joint_1) + origins[0]

    
    delta_pulse_1 = servo_degrees_to_pulses(tar_joint_1 - cur_joint_1)

    
    accel_decel_1, max_speed_1 = servo_accel_decel_calc(delta_pulse_1, travel_time)

    
    if selection != "stepper_only":
        set_sdo(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x0F)
        servo_set_profile_type(0x00)
        servo_set_acceleration(accel_decel_1)
        servo_set_deceleration(accel_decel_1)
        servo_set_max_speed(max_speed_1)
        servo_set_tar_pulse(tar_pulse_1)
        
    if selection != "stepper_only":  
        max_accel_servo = 583000 # 582549
        if (accel_decel_1 <= max_accel_servo):
            if (accel_decel_1 >= (max_accel_servo * 0.8)):
                print_yellow(f"warning : almost max acceleration")
            return 1
        else:
            print_red(f"motion denied, acceleration is too high, dangerous movement")
            return 0

def pp_angle(tar_joints, travel_time, selection):
    print(f"masuk pp angle")
    origins = get_origins()
    
    if selection != "servo_only": 
        pp_mode_init()
    
    cur_joints = get_cur_joints(selection)
    
    tar_joints = check_limit(tar_joints)
    
    cur_joint_1, cur_joint_2, cur_joint_3, cur_joint_4 = cur_joints
    tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4 = tar_joints
    
    tar_pulse_1 = servo_degrees_to_pulses(tar_joint_1) + origins[0]
    tar_pulse_2 = stepper_degrees_to_pulses(tar_joint_2)
    tar_pulse_3 = stepper_degrees_to_pulses(tar_joint_3)
    tar_pulse_4 = stepper_degrees_to_pulses(tar_joint_4)
    
    print(f"{tar_joint_1} {tar_pulse_1}")
    
    delta_pulse_1 = servo_degrees_to_pulses(tar_joint_1 - cur_joint_1)
    delta_pulse_2 = stepper_degrees_to_pulses(tar_joint_2 - cur_joint_2)
    delta_pulse_3 = stepper_degrees_to_pulses(tar_joint_3 - cur_joint_3)
    delta_pulse_4 = stepper_degrees_to_pulses(tar_joint_4 - cur_joint_4)
    
    # print(f"tar_puse {delta_pulse_2} {delta_pulse_3} {delta_pulse_4}")
    
    accel_decel_1, max_speed_1 = servo_accel_decel_calc(delta_pulse_1, travel_time)
    accel_decel_2, max_speed_2 = stepper_accel_decel_calc(delta_pulse_2, travel_time)
    accel_decel_3, max_speed_3 = stepper_accel_decel_calc(delta_pulse_3, travel_time)
    accel_decel_4, max_speed_4 = stepper_accel_decel_calc(delta_pulse_4, travel_time)
    
    # print(f"cur joint = {cur_joints} degree")
    # print(f"tar joint = {tar_joints} degree")
    # print(f"delta pulse = {delta_pulse_1}, {delta_pulse_2}, {delta_pulse_3}, {delta_pulse_4}")
    # print(f"tar pulse = {tar_pulse_1}, {tar_pulse_2}, {tar_pulse_3}, {tar_pulse_4}")
    # print(f"tar accel = {accel_decel_1}, {accel_decel_2}, {accel_decel_3}, {accel_decel_4}")
    # print(f"tar max speed = {max_speed_1}, {max_speed_2}, {max_speed_3}, {max_speed_4}")
    
    if selection != "servo_only":  
        max_speed_2 = stepper_pulses_to_steps(max_speed_2)
        max_speed_3 = stepper_pulses_to_steps(max_speed_3)
        max_speed_4 = stepper_pulses_to_steps(max_speed_4)
        accel_decel_2 = stepper_pulses_to_steps(accel_decel_2)
        accel_decel_3 = stepper_pulses_to_steps(accel_decel_3)
        accel_decel_4 = stepper_pulses_to_steps(accel_decel_4)
        pp_mode_set_acceleration(accel_decel_2, accel_decel_3, accel_decel_4)
        pp_mode_set_deceleration(accel_decel_2, accel_decel_3, accel_decel_4)
        pp_mode_set_max_speed(max_speed_2, max_speed_3, max_speed_4)
        pp_mode_set_tar_pulse(tar_pulse_2, tar_pulse_3, tar_pulse_4)
    
  
    
    if selection != "stepper_only":
        set_sdo(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x0F)
        servo_set_profile_type(0x00)
        servo_set_acceleration(accel_decel_1)
        servo_set_deceleration(accel_decel_1)
        servo_set_max_speed(max_speed_1)
        servo_set_tar_pulse(tar_pulse_1)
    
    if selection != "servo_only":  
        pp_mode_start_absolute_motion()
        # print(f"pp mode start absolute motion")
        
    if selection != "stepper_only":  
        # time.sleep(0.5)  # wait for servo to switch on
        max_accel_servo = 583000 # 582549
        if (accel_decel_1 <= max_accel_servo):
            print(f"done")
            set_sdo(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x1F)
            if (accel_decel_1 >= (max_accel_servo * 0.8)):
                print_yellow(f"warning : almost reaching max acceleration")
        else:
            print_red(f"motion denied, acceleration is too high, dangerous movement")
    
def pp_coor(tar_coor, travel_time, selection):
    tar_joints = inverse_kinematics(tar_coor)
    tar_joints = check_limit(tar_joints)
    formatted_tar_joints = "°, ".join([f"{tar_joint:.2f}" for tar_joint in tar_joints]) #6 may 2025
    print(f"tar joint = {formatted_tar_joints}°")
    pp_angle(tar_joints, travel_time, selection)
    
    
    
# ######################################### PVT ######################################### #

def pvt_angle(tar_joints, travel_time, selection):
    global stop_watch, time_out, last_time
    group_id = 0x05
    tar_pulses = []
    node_ids = [ID1, ID2, ID3, ID4]
    pvt_3_lower_limit = 60
    pvt_3_upper_limit = 80
    pvt_mode_init(group_id, PVT_3, 400, pvt_3_lower_limit, pvt_3_upper_limit)
    cur_joints = get_cur_joints(selection)
    servo_init(7)
    
    tar_pulses.append(servo_degrees_to_pulses(tar_joints[0]))
    for tar_joint in tar_joints[1:]:
        tar_pulses.append(stepper_degrees_to_pulses(tar_joint))
        
    # Initialize p, v, and t as empty lists
    p = [None] * 4
    v = [None] * 4
    t = [None] * 4
    
    p[0], v[0], t[0] = generate_pvt_trajectory_triangle_2(0, tar_pulses[0], travel_time)
    

    for pos, vel, tim in zip(p[0], v[0], t[0]):
        servo_set_interpolation_data(pos, vel, tim)

    if selection != "stepper_only":
        set_sdo(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x0F)
        set_sdo(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x1F)
    
    last_time = time.time()
    stop_watch = last_time
    time_out = travel_time / 1000

# ######################################### PVT 3 ######################################### #

def pvt_mode_try_pvt_3(cur_joints, tar_joints, travel_time):
    global last_time, stop_watch, time_out
    group_id = 0x05
    tar_pulses = []
    cur_pulses = []
    node_ids = [ID1, ID2, ID3, ID4]
    pvt_3_lower_limit = 60
    pvt_3_upper_limit = 80

    pvt_mode_init(group_id, PVT_3, 400, pvt_3_lower_limit, pvt_3_upper_limit)
    
    for tar_joint in tar_joints:
        tar_pulses.append(stepper_degrees_to_pulses(tar_joint))
        
    for cur_joint in cur_joints:
        cur_pulses.append(stepper_degrees_to_pulses(cur_joint))
    
    
    # Initialize p, v, and t as empty lists
    p = [None] * 4
    v = [None] * 4
    t = [None] * 4
    
    #qq
    for i in range(1, 4):
        # tar_step = stepper_pulses_to_steps(tar_pulses[i])
        tar_step = tar_pulses[i]
        p[i], v[i], t[i] = generate_pvt_trajectory_triangle_2(cur_pulses[i] ,  tar_step, travel_time) 
    
    for i in range(1, 4):
        for pos, vel, tim in zip(p[i], v[i], t[i]):
            # if (vel != 0):
            pvt_mode_write_read(node_ids[i], pos, vel, tim)
            # print(f"motor {i+1} write {pos}, {vel}, {tim}")
                
                
    pvt_mode_read_pvt_3_depth()
    # pvt_mode_start_pvt_step(group_id)
    last_time = time.time()
    stop_watch = last_time
    time_out = travel_time / 1000
    
    
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



# ######################################### PVT CIRCULAR ######################################### #
def gen_circular(cur_pos, center_pos, end_angle, travel_time, direction="CCW"):
    travel_time = travel_time / 1000  # Konversi dari ms ke detik
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
    
    joint1_steps = [stepper_pulses_to_steps(val) for val in joint1_pulses]
    joint2_steps = [stepper_pulses_to_steps(val) for val in joint2_pulses]
    joint3_steps = [stepper_pulses_to_steps(val) for val in joint3_pulses]
    joint4_steps = [stepper_pulses_to_steps(val) for val in joint4_pulses]
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
    time_out = travel_time / 1000





    
    
    
# ######################################### KINEMATICS ######################################### #

def check_limit(tar_joints):
    
    tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4 = tar_joints
    tar_joint_4 *= -1
    #limit2 = 0 to 178 degree
    #limit3 = -135 to 0 degree
    #limit4 = 0 to 196 degree
    
    # if tar_joint_1 > (13004): # 6 may 2025
    #     tar_joint_1 = 13004 # 6 may 2025
    if tar_joint_1 > (3551*4): # 6 may 2025
        tar_joint_1 = (3551*4) # 6 may 2025
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
        
    joint_4 *= -1
    
    joints = [joint_1, joint_2, joint_3, joint_4]
    joints = check_limit(joints)
    
    return joints

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

# ######################################### DANCING ######################################### #

def straight_line(travel_time):
    sleep = (travel_time / 1000) + 0.1 # in seconds

    
    coor_1 = [258, 0, 0, 0]
    coor_2 = [150, 0, 0, 0]
        
    for i in range(3):
        sp_coor(coor_1, travel_time)
        time.sleep(sleep)
        sp_coor(coor_2, travel_time)
        time.sleep(sleep)
    
def rectangular(travel_time):
    sleep = (travel_time / 1000) + 0.1 # in seconds

    
    coor_1 = [210, 40, 0, 0]
    coor_2 = [210, -40, 0, 0]
    coor_3 = [130, -40, 0, 0]
    coor_4 = [130, 40, 0, 0]
    
    for i in range(2):
        sp_coor(coor_1, travel_time)
        time.sleep(sleep)
        sp_coor(coor_2, travel_time)
        time.sleep(sleep)
        sp_coor(coor_3, travel_time)
        time.sleep(sleep)
        sp_coor(coor_4, travel_time)
        time.sleep(sleep)
    
    sp_coor(coor_1, travel_time)
    time.sleep(sleep)

        
def home_position(travel_time):
    sleep = (travel_time / 1000) + 0.1 # in seconds

    
    home_angles = [0, 0, 0, 0]
    sp_angle(home_angles, travel_time)
    time.sleep(sleep)

		
def shuttle_position(travel_time):
    sleep = (travel_time / 1000)
    
    shuttle_coor = [166.82, -168, 0, 0]
    sp_coor(shuttle_coor, travel_time)
    time.sleep(sleep)

def pre_past_shelf(travel_time):
    sleep = (travel_time / 1000)
    
    pre_past_shelf_coor = [107, 100, 0, 90]
    sp_coor(pre_past_shelf_coor, travel_time)
    time.sleep(sleep)


def pickup_from_shelf(travel_time):
    sleep = (travel_time / 1000)
    
    pickup_from_shelf_coor = [107, 224, 0, 90]
    sp_coor(pickup_from_shelf_coor, travel_time)
    time.sleep(sleep)

def place_onto_shelf(travel_time):
    sleep = (travel_time / 1000)
    
    place_from_shelf_coor = [107, 197, 0, 90]
    sp_coor(place_from_shelf_coor, travel_time)
    time.sleep(sleep)

        
def dancing(travel_time):
    sleep = (travel_time / 1000) + 0.1 # in seconds

    straight_line(travel_time)
    sp_coor([140, 0, 0, -20], travel_time)
    time.sleep(sleep)
    sp_coor([140, 0, 0, 90], travel_time)
    time.sleep(sleep)
    sp_coor([140, -168, 0, 0], travel_time)
    time.sleep(sleep)
    home_position(travel_time)
    shuttle_position(travel_time)
    home_position(travel_time)
    shuttle_position(travel_time)
    home_position(travel_time)
    rectangular(travel_time)
    sp_coor([130, 40, 0, 0], travel_time)
    time.sleep(sleep)
    sp_coor([130, 0, 0, 0], travel_time/2)
    time.sleep(sleep/2)
    
    cur_pos = (130, 0)  # (x, y) posisi awal
    center_pos = (170, 0)  # Pusat lingkaran
    end_angle = 360  # Gerakan setengah lingkaran
    circular_travel_time = 4  # dalam detik
    circular_sleep = circular_travel_time + 0.1
    direction = "CCW"  # Arah rotasi
    
    pvt_circular(cur_pos, center_pos, end_angle, circular_travel_time, direction)
    time.sleep(circular_sleep*2)
    time.sleep(sleep/2)
    #
    pre_past_shelf(travel_time)
    pickup_from_shelf(travel_time)
    pre_past_shelf(travel_time)
    pickup_from_shelf(travel_time)
    pre_past_shelf(travel_time)
    
    #
    sp_coor([150, 0, 0, 0], travel_time)
    time.sleep(sleep)
    sp_coor([258, 0, 0, 90], travel_time)
    time.sleep(sleep)
    sp_coor([258, 0, 0, -90], travel_time)
    time.sleep(sleep)
    sp_coor([258, 0, 0, 0], travel_time)


import matplotlib.pyplot as plt

def generate_coor_straight_trajectory(start, end, steps):
    x_points = np.linspace(start[0], end[0], steps)
    y_points = np.linspace(start[1], end[1], steps)
    z_points = np.linspace(start[2], end[2], steps)
    yaw_points = np.linspace(start[3], end[3], steps)
    trajectory = list(zip(x_points, y_points, z_points, yaw_points))
    return trajectory

def transform_time_to_angle(current_time, start_time, travel_time):
    elapsed_time = current_time - start_time
    angle = ((elapsed_time / travel_time) * 180) - 90
    angle = min(max(angle, -90), 90)
    return angle

def sine_wave(current_time, start_time, travel_time, cur_pos, tar_pos):
    angle = transform_time_to_angle(current_time, start_time, travel_time)
    angle_radians = math.radians(angle)
    sin_value = math.sin(angle_radians)
    output = cur_pos + (sin_value + 1) / 2 * (tar_pos - cur_pos)
    return output



def pvt_mode_try_pvt_4(cur_joints, tar_joints, travel_time):
    
    show = 0
    
    cur_coor = forward_kinematics(cur_joints)
    tar_coor = forward_kinematics(tar_joints)

    cur_x, cur_y, cur_z, cur_yaw = cur_coor
    tar_x, tar_y, tar_z, tar_yaw = tar_coor
    # # 
    # sp_angle(cur_joints, 500, "stepper_only")
    # time.sleep(1)
    
    start_coor = cur_x, cur_y, cur_z, cur_yaw
    end_coor = tar_x, tar_y, cur_z, tar_yaw

    steps = int(travel_time / pvt_time_interval)  # Make sure pvt_time_interval is defined

    # print(f"cur_coor: ({cur_x:.2f}, {cur_y:.2f}, {cur_z:.2f}, {cur_yaw:.2f})")
    # print(f"tar_coor: ({tar_x:.2f}, {tar_y:.2f}, {tar_z:.2f}, {tar_yaw:.2f})")
    # print(f"steps: {steps}")
    
    trajectory = generate_coor_straight_trajectory(start_coor, end_coor, steps)
    
    if show == 1:
        # Plot the trajectory in one window
        plt.figure(figsize=(8, 6))
        plt.plot([point[0] for point in trajectory], [point[1] for point in trajectory], label='Trajectory')
        plt.xlim(0, 258)
        plt.ylim(-258, 258)
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('Generated Trajectory')
        plt.grid(True)
        plt.legend()
        plt.show()
    
    start_time = 0
    time_values = np.linspace(start_time, start_time + travel_time, steps)
    
    # Calculate sine wave values for x and y positions
    x_over_time = [sine_wave(t, start_time, travel_time, cur_x, tar_x) for t in time_values]
    y_over_time = [sine_wave(t, start_time, travel_time, cur_y, tar_y) for t in time_values]
    z_over_time = [sine_wave(t, start_time, travel_time, cur_z, tar_z) for t in time_values]
    yaw_over_time = [sine_wave(t, start_time, travel_time, cur_yaw, tar_yaw) for t in time_values]

    trajectory_over_time = list(zip(x_over_time, y_over_time, z_over_time, yaw_over_time))
    
    # Calculate joint angles for each point in the trajectory
    joint_1_values = []
    joint_2_values = []
    joint_3_values = []
    joint_4_values = []

    for (x, y, z, yaw) in trajectory_over_time:
        try:
            joint_1, joint_2, joint_3, joint_4 = inverse_kinematics([x, y, z, yaw])
            joint_1_values.append(joint_1)
            joint_2_values.append(joint_2)
            joint_3_values.append(joint_3)
            joint_4_values.append(joint_4)
        except ValueError as e:
            print(f"Error calculating inverse kinematics for (x={x}, y={y}, c={c}): {e}")
            joint_1_values.append(None) # Use None to indicate out-of-bound values
            joint_2_values.append(None) 
            joint_3_values.append(None)
            joint_4_values.append(None)

    if show == 1:
    # Plot x, y, and c over time in one window
        plt.figure(figsize=(12, 8))

        # Plot x and y over time
        plt.subplot(3, 1, 1)
        plt.plot(time_values - start_time, x_over_time, label='X over Time')
        plt.xlabel('Time (ms)')
        plt.ylabel('X Position')
        plt.grid(True)
        plt.legend()
        plt.title('X and Y Position over Time')

        plt.subplot(3, 1, 2)
        plt.plot(time_values - start_time, y_over_time, label='Y over Time')
        plt.xlabel('Time (ms)')
        plt.ylabel('Y Position')
        plt.grid(True)
        plt.legend()

        plt.subplot(3, 1, 3)
        plt.plot(time_values - start_time, yaw_over_time, label='C over Time')
        plt.xlabel('Time (ms)')
        plt.ylabel('C Position')
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.show()
        
        # Plot joint angles over time
        plt.figure(figsize=(12, 8))

        # Plot joint2, joint3, and joint4 over time
        plt.subplot(3, 1, 1)
        plt.plot(time_values - start_time, joint_2_values, label='Joint 2')
        plt.xlabel('Time (ms)')
        plt.ylabel('Joint 2 Angle')
        plt.grid(True)
        plt.legend()
        plt.title('Joint Angles Over Time')

        plt.subplot(3, 1, 2)
        plt.plot(time_values - start_time, joint_3_values, label='Joint 3')
        plt.xlabel('Time (ms)')
        plt.ylabel('Joint 3 Angle')
        plt.grid(True)
        plt.legend()

        plt.subplot(3, 1, 3)
        plt.plot(time_values - start_time, joint_4_values, label='Joint 4')
        plt.xlabel('Time (ms)')
        plt.ylabel('Joint 4 Angle')
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.show()
    
    def calculate_relative_position(joint_values):
        base = joint_values[0]
        return [j - base if j is not None else None for j in joint_values]

    joint_1_relative = calculate_relative_position(joint_1_values)
    joint_2_relative = calculate_relative_position(joint_2_values)
    joint_3_relative = calculate_relative_position(joint_3_values)
    joint_4_relative = calculate_relative_position(joint_4_values)
    
    if show == 1:
    
        plt.figure(figsize=(12, 8))
        plt.subplot(3, 1, 1)
        plt.plot(time_values, joint_2_relative, label='Joint 2 Relative Position')
        plt.ylabel('Degrees')
        plt.xlabel('Time (s)')
        plt.title('Relative Joint Position')
        plt.grid(True)
        plt.legend()

        plt.subplot(3, 1, 2)
        plt.plot(time_values, joint_3_relative, label='Joint 3 Relative Position')
        plt.ylabel('Degrees')
        plt.xlabel('Time (s)')
        plt.grid(True)
        plt.legend()

        plt.subplot(3, 1, 3)
        plt.plot(time_values, joint_4_relative, label='Joint 4 Relative Position')
        plt.ylabel('Degrees')
        plt.xlabel('Time (s)')
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.show()


    def calculate_joint_displacement(joint_values):
        displacement = []
        for i in range(1, len(joint_values)):
            if joint_values[i] is not None and joint_values[i-1] is not None:
                delta_position = joint_values[i] - joint_values[i-1]
                displacement.append(delta_position)
            else:
                displacement.append(None)  # Jika ada nilai None, maka hasilnya None
        displacement.insert(0, 0)  # Tidak ada pergerakan pada titik awal, jadi displacement awal adalah 0
        return displacement
    
    # Menghitung diferensial posisi untuk setiap joint
    joint_1_displacement = calculate_joint_displacement(joint_1_values)
    joint_2_displacement = calculate_joint_displacement(joint_2_values)
    joint_3_displacement = calculate_joint_displacement(joint_3_values)
    joint_4_displacement = calculate_joint_displacement(joint_4_values)
    
    if show == 1:
    
        plt.figure(figsize=(12, 8))
        plt.subplot(3, 1, 1)
        plt.plot(time_values - start_time, joint_2_displacement, label='Joint 2 Displacement')
        plt.xlabel('Time (ms)')
        plt.ylabel('Joint 2 Displacement')
        plt.grid(True)
        plt.legend()
        plt.title('Joint Displacement Over Time')

        plt.subplot(3, 1, 2)
        plt.plot(time_values - start_time, joint_3_displacement, label='Joint 3 Displacement')
        plt.xlabel('Time (ms)')
        plt.ylabel('Joint 3 Displacement')
        plt.grid(True)
        plt.legend()

        plt.subplot(3, 1, 3)
        plt.plot(time_values - start_time, joint_4_displacement, label='Joint 4 Displacement')
        plt.xlabel('Time (ms)')
        plt.ylabel('Joint 4 Displacement')
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.show()
    
    # Fungsi untuk menghitung kecepatan joint dalam RPM
    def calculate_joint_speed(displacement, interval):
        speed_rpm = []
        for i in range(1, len(displacement)):
            if displacement[i] is not None and displacement[i-1] is not None:
                speed_dps = displacement[i] / interval
                speed_rpm.append((speed_dps * 60) / 360)
            else:
                speed_rpm.append(None)  # Jika ada nilai None, maka hasilnya None
        speed_rpm.insert(0, 0)  # Tidak ada kecepatan pada titik awal, jadi kecepatan awal adalah 0
        return speed_rpm
    
    # Interval waktu antara setiap langkah
    interval = (time_values[1] - time_values[0]) / 1000  # Konversi dari ms ke detik
    

    # Menghitung kecepatan dalam RPM
    joint_1_speed_rpm = calculate_joint_speed(joint_1_displacement, interval)
    joint_2_speed_rpm = calculate_joint_speed(joint_2_displacement, interval)
    joint_3_speed_rpm = calculate_joint_speed(joint_3_displacement, interval)
    joint_4_speed_rpm = calculate_joint_speed(joint_4_displacement, interval)
    
    if show == 1:
    # Plot kecepatan joint dalam RPM
        plt.figure(figsize=(12, 8))
        plt.subplot(3, 1, 1)
        plt.plot(time_values - start_time, joint_2_speed_rpm, label='Joint 2 Speed (RPM)')
        plt.xlabel('Time (ms)')
        plt.ylabel('Speed (RPM)')
        plt.grid(True)
        plt.legend()
        plt.title('Joint 2 Speed Over Time')

        plt.subplot(3, 1, 2)
        plt.plot(time_values - start_time, joint_3_speed_rpm, label='Joint 3 Speed (RPM)')
        plt.xlabel('Time (ms)')
        plt.ylabel('Speed (RPM)')
        plt.grid(True)
        plt.legend()

        plt.subplot(3, 1, 3)
        plt.plot(time_values - start_time, joint_4_speed_rpm, label='Joint 4 Speed (RPM)')
        plt.xlabel('Time (ms)')
        plt.ylabel('Speed (RPM)')
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.show()
        
    joint_2_relative_pulses = [stepper_degrees_to_pulses(d) if d is not None else None for d in joint_2_relative]
    joint_3_relative_pulses = [stepper_degrees_to_pulses(d) if d is not None else None for d in joint_3_relative]
    joint_4_relative_pulses = [stepper_degrees_to_pulses(d) if d is not None else None for d in joint_4_relative]
    
    if show == 1:
        plt.figure(figsize=(12, 8))
        plt.subplot(3, 1, 1)
        plt.plot(time_values, joint_2_relative_pulses, label='Joint 2 Relative (Pulse)')
        plt.xlabel('Time (s)')
        plt.ylabel('Pulse')
        plt.title('Joint 2 Relative Position (Pulse)')
        plt.grid(True)
        plt.legend()

        plt.subplot(3, 1, 2)
        plt.plot(time_values, joint_3_relative_pulses, label='Joint 3 Relative (Pulse)')
        plt.xlabel('Time (s)')
        plt.ylabel('Pulse')
        plt.title('Joint 3 Relative Position (Pulse)')
        plt.grid(True)
        plt.legend()

        plt.subplot(3, 1, 3)
        plt.plot(time_values, joint_4_relative_pulses, label='Joint 4 Relative (Pulse)')
        plt.xlabel('Time (s)')
        plt.ylabel('Pulse')
        plt.title('Joint 4 Relative Position (Pulse)')
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.show()
    
    def stepper_rpm_to_pps(rpm):
        return int(rpm * (STEPPER_PPR / 60))

    joint_2_speed_pps = [stepper_rpm_to_pps(r) if r is not None else None for r in joint_2_speed_rpm]
    joint_3_speed_pps = [stepper_rpm_to_pps(r) if r is not None else None for r in joint_3_speed_rpm]
    joint_4_speed_pps = [stepper_rpm_to_pps(r) if r is not None else None for r in joint_4_speed_rpm]

        
    def generate_pvt_points_from_pulse_and_pps(position_pulse, speed_pps, interval_ms):
        """
        Buat PVT point dalam format (position, velocity, time)
        - position_pulse: list posisi relatif dalam pulse
        - speed_pps: list kecepatan dalam pulse per second
        - interval_ms: waktu antar titik dalam milidetik
        """
        pvt_points = []
        for pos, vel in zip(position_pulse, speed_pps):
            if pos is not None and vel is not None:
                pvt_points.append((int(round(pos)), int(round(vel)), int(round(interval_ms))))
        return pvt_points

    interval_ms = pvt_time_interval  # langsung dalam ms, misalnya 25
    pvt_joint_2 = generate_pvt_points_from_pulse_and_pps(joint_2_relative_pulses, joint_2_speed_pps, interval_ms)
    pvt_joint_3 = generate_pvt_points_from_pulse_and_pps(joint_3_relative_pulses, joint_3_speed_pps, interval_ms)
    pvt_joint_4 = generate_pvt_points_from_pulse_and_pps(joint_4_relative_pulses, joint_4_speed_pps, interval_ms)

    global last_time, stop_watch, time_out
    group_id = 0x05
    tar_pulses = []
    cur_pulses = []
    node_ids = [ID1, ID2, ID3, ID4]
    pvt_3_lower_limit = 60
    pvt_3_upper_limit = 80

    #ww
    pvt_mode_init(group_id, PVT_3, 400, pvt_3_lower_limit, pvt_3_upper_limit)
    
    
    for pos, vel, tim in pvt_joint_2:
        if (vel != 0) and (pos != 0):
            pvt_mode_write_read(ID2, pos, vel, tim)
        
    for pos, vel, tim in pvt_joint_3:
        if (vel != 0):
            pvt_mode_write_read(ID3, pos, vel, tim)
        
    for pos, vel, tim in pvt_joint_4:
        if (vel != 0):
            pvt_mode_write_read(ID4, pos, vel, tim)
                
                
    pvt_mode_read_pvt_3_depth()
    pvt_mode_start_pvt_step(group_id)
    last_time = time.time()
    stop_watch = last_time
    time_out = travel_time / 1000

    return 0


def stepper_single_motor_pp_mode(id, tar_joint, travel_time,  selection):
    
    if selection != "servo_only": 
        init_single_motor_operation_mode(id, 4)
    
    cur_joint = stepper_get_motor_position(id)
    tar_pulse = stepper_degrees_to_pulses(tar_joint)
    
    delta_pulse = stepper_degrees_to_pulses(tar_joint - cur_joint)
    accel_decel, max_speed = stepper_accel_decel_calc(delta_pulse, travel_time)
    
    if selection != "servo_only":  
        max_speed = stepper_pulses_to_steps(max_speed)
        accel_decel = stepper_pulses_to_steps(accel_decel)
        pp_mode_single_motor_set_acceleration(id, accel_decel)
        pp_mode_single_motor_set_deceleration(id, accel_decel)
        pp_mode_single_motor_set_max_speed(id, max_speed)
        pp_mode_single_motor_set_tar_pulse(id, tar_pulse)
    
    if selection != "servo_only":  
        pp_mode_single_motor_start_absolute_motion(id)
        # print(f"pp mode start absolute motion")


