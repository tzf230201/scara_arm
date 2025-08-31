from motion import *
from servo import *
from stepper import *
from arm import *
import time
import math
import numpy as np
import matplotlib.pyplot as plt

def plot_xy_from_pt(pt1, pt2, pt3, pt4):
    x_list, y_list = [], []
    N = len(pt1)
    for i in range(N):
        # Pulse → degree
        j1 = stepper_pulse_to_deg(pt1[i])
        j2 = stepper_pulse_to_deg(pt2[i])
        j3 = stepper_pulse_to_deg(pt3[i])
        j4 = stepper_pulse_to_deg(pt4[i])
        # FK: panggil dengan satu list
        x, y, *_ = forward_kinematics([j1, j2, j3, j4])
        x_list.append(x)
        y_list.append(y)
    plt.figure()
    plt.plot(x_list, y_list, '-o', label='End Effector Path')
    plt.xlabel('X (mm)')
    plt.ylabel('Y (mm)')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.title("XY Trajectory from PT Path")
    plt.show()

def check_limit(tar_joints, source_info=""):
    tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4 = tar_joints
    tar_joint_4 *= -1

    if tar_joint_1 > (3510 * 4):
        print(f"[{source_info}] tar_joint_1 greater than {tar_joint_1}")
        tar_joint_1 = (3510 * 4)
    elif tar_joint_1 < -1:
        print(f"[{source_info}] tar_joint_1 lower than {tar_joint_1}")
        tar_joint_1 = -1

    if tar_joint_2 > (178 * 5):
        print(f"[{source_info}] tar_joint_2 greater than {tar_joint_2}")
        tar_joint_2 = 178 * 5
    elif tar_joint_2 < 0:
        print(f"[{source_info}] tar_joint_2 lower than {tar_joint_2}")
        tar_joint_2 = 0

    if tar_joint_3 > (0 + tar_joint_2):
        print(f"[{source_info}] tar_joint_3 greater than {tar_joint_3}")
        tar_joint_3 = (0 + tar_joint_2)
    elif tar_joint_3 < ((-135 * 5) + tar_joint_2):
        print(f"[{source_info}] tar_joint_3 lower than {tar_joint_3}")
        tar_joint_3 = (-135 * 5) + tar_joint_2

    if tar_joint_4 > ((196 * 5) + tar_joint_3):
        print(f"[{source_info}] tar_joint_4 greater than {tar_joint_4}")
        tar_joint_4 = (196 * 5) + tar_joint_3
    elif tar_joint_4 < (0 + tar_joint_3):
        print(f"[{source_info}] tar_joint_4 lower than {tar_joint_4}")
        tar_joint_4 = 0 + tar_joint_3

    tar_joint_4 *= -1
    return [tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4]

import math

def inverse_kinematics(tar_coor):
    x, y, z, yaw = tar_coor
    # Panjang link (mm)
    L2 = 137.0
    L3 = 121.0
    # Offset (derajat) dan rasio joint
    OFFSET_2 = -96.5
    OFFSET_3 = 134
    OFFSET_4 = -52.5
    RATIO = 5.0  # 5:1

    # Hitung jarak planar
    distance = math.hypot(x, y)
    max_reach = L2 + L3
    if distance > max_reach:
        # Jika benar-benar di luar jangkauan, optional: warn, lalu clamp ke batas terjauh
        print(f"Warning: target ({x:.1f},{y:.1f}) jarak {distance:.1f} > {max_reach:.1f}, akan di-clamp")
        # Arahkan ke arah tepi workspace
        scale = max_reach / distance
        x *= scale
        y *= scale

    # Hitung cos(theta3)
    cos_theta3 = (x**2 + y**2 - L2**2 - L3**2) / (2 * L2 * L3)
    # Clamp ke [-1,1]
    cos_theta3 = max(-1.0, min(1.0, cos_theta3))
    theta3_rad = math.acos(cos_theta3)
    theta3 = math.degrees(theta3_rad)

    # Hitung theta2
    k1 = L2 + L3 * cos_theta3
    k2 = L3 * math.sin(theta3_rad)
    theta2_rad = math.atan2(y, x) - math.atan2(k2, k1)
    theta2 = math.degrees(theta2_rad)

    # Konversi ke pulse
    joint_1 = z * (360.0 / 90.0)           # asumsi 90 mm → 360°
    joint_2 = (theta2 - OFFSET_2) * RATIO
    joint_3 = (theta3 - OFFSET_3) * RATIO + joint_2
    joint_4 = -((yaw - OFFSET_4) * RATIO)

    joints = [joint_1, joint_2, joint_3, joint_4]
    return check_limit(joints)

# def inverse_kinematics(tar_coor):
#     x, y, z, yaw = tar_coor
#     # max area
#     L2 = 137.0
#     L3 = 121.0
#     L4 = 56.82
#     OFFSET_2 = -96.5
#     OFFSET_3 = 134
#     OFFSET_4 = -52.5
#     #ration joint = 5:1

#     distance = math.sqrt(x**2 + y**2)

#     if distance > (L2 + L3):
#         raise ValueError("out of boundary")

#     # joint_3
#     cos_theta3 = (x**2 + y**2 - L2**2 - L3**2) / (2 * L2 * L3)
#     theta3 = math.acos(cos_theta3)  #angle in radian

#     # joint_2
#     k1 = L2 + L3 * cos_theta3
#     k2 = L3 * math.sin(theta3)
#     theta2 = math.atan2(y, x) - math.atan2(k2, k1)

#     # rad to deg
#     theta2 = math.degrees(theta2)
#     theta3 = math.degrees(theta3)

#     joint_1 = z * (360/90)
#     joint_2 = (theta2-OFFSET_2)*5
#     joint_3 = (theta3-OFFSET_3)*5 + joint_2
#     joint_4 = (yaw-OFFSET_4)*5# + joint_3;
        
#     joint_4 *= -1
    
#     joints = [joint_1, joint_2, joint_3, joint_4]
#     joints = check_limit(joints)
    
#     return joints


def forward_kinematics(cur_joints):
    cur_deg1, cur_deg2, cur_deg3, cur_deg4 = cur_joints
    cur_deg4 *= -1.0
    L2 = 137.0
    L3 = 121.0
    OFFSET_2 = -96.5
    OFFSET_3 = 134.0
    OFFSET_4 = -52.5

    theta2_rad = math.radians((cur_deg2 / 5) + OFFSET_2)
    theta3_rad = math.radians((cur_deg3 / 5) + OFFSET_3 - (cur_deg2 / 5))

    x2 = L2 * math.cos(theta2_rad)
    y2 = L2 * math.sin(theta2_rad)
    x3 = x2 + L3 * math.cos(theta2_rad + theta3_rad)
    y3 = y2 + L3 * math.sin(theta2_rad + theta3_rad)
    z = (cur_deg1 / 360) * 90
    yaw = (cur_deg4 / 5) + OFFSET_4

    return [x3, y3, z, yaw]

def generate_multi_straight_pt_points(start_coor, list_tar_coor, pt_time_interval=50):
    pt1_f, pt2_f, pt3_f, pt4_f = [], [], [], []
    last_coor = start_coor

    for tar_coor, traveltime in list_tar_coor:
        # jumlah step & total waktu
        n_step = max(int(round(traveltime / pt_time_interval)), 1)
        T = n_step * pt_time_interval

        for i in range(n_step):
            t = i * pt_time_interval
            tau = t / T  # normalisasi waktu [0,1)

            # triangle profile: alpha = s(t)/s_total
            if tau <= 0.5:
                alpha = 2 * tau**2
            else:
                alpha = 1 - 2 * (1 - tau)**2

            # interpolasi Cartesian
            coor = [
                last_coor[j] + (tar_coor[j] - last_coor[j]) * alpha
                for j in range(4)
            ]
            
            

            # IK → pulse
            joint = inverse_kinematics(coor)
            pulses = [stepper_deg_to_pulse(j) for j in joint]

            pt1_f.append(pulses[0])
            pt2_f.append(pulses[1])
            pt3_f.append(pulses[2])
            pt4_f.append(pulses[3])

        # beralih ke segmen berikutnya, pastikan posisinya pas di target
        last_coor = tar_coor

    # tambahkan titik akhir yang pasti sampai target terakhir
    final_joint = inverse_kinematics(list_tar_coor[-1][0])
    pulses = [stepper_deg_to_pulse(j) for j in final_joint]
    pt1_f.append(pulses[0])
    pt2_f.append(pulses[1])
    pt3_f.append(pulses[2])
    pt4_f.append(pulses[3])

    return pt1_f, pt2_f, pt3_f, pt4_f










def is_stepper_selected(selection):
    return selection != "servo_only"

def is_servo_selected(selection):
    return selection != "stepper_only"



def robot_init(selection):
    print(f"robot_init({selection})")
    
    
    
def robot_wake_up(selection):
    if is_stepper_selected(selection):
        arm_init()
    if is_servo_selected(selection):
        print(f"servo initialization skipped in this version")
        servo_init(1)  # 7 is PVT mode, 1 is PP mode
        servo_disable_heartbeat()
    print(f"robot_wake_up({selection})")
    
def robot_shutdown(selection):
    if is_stepper_selected(selection):
        arm_set_motor_off()
    if is_servo_selected(selection):
        # servo_shutdown()
        # print(f"servo shutdown, brake active")
        print(f"servo is not shutdown, development purpose only")
    print(f"robot_shutdown({selection})")

def robot_get_enc(selection):
    # Inisialisasi default
    enc_1 = None
    enc_2 = None
    enc_3 = None
    enc_4 = None

    # Baca encoder servo jika ter-select
    if is_servo_selected(selection):
        enc_1 = servo_get_encoder()

    # Baca encoder stepper jika ter-select
    if is_stepper_selected(selection):
        enc_2, enc_3, enc_4 = arm_get_enc()

    return enc_1, enc_2, enc_3, enc_4


def robot_get_angle(selection):
    # Inisialisasi default
    cur_angle_1 = None
    cur_angle_2 = None
    cur_angle_3 = None
    cur_angle_4 = None

    # Baca servo jika ter-select
    if is_servo_selected(selection):
        cur_angle_1 = servo_get_angle()

    # Baca stepper jika ter-select
    if is_stepper_selected(selection):
        cur_angle_2, cur_angle_3, cur_angle_4 = arm_get_angle()

    return cur_angle_1, cur_angle_2, cur_angle_3, cur_angle_4

def robot_set_origin(selection):
    if selection == "all":
        servo_set_origin()
        arm_set_origin()
        print(f"robot_set_origin({selection})")
    else:
        print(f"robot_set_origin({selection}) skipped, only available for 'all' selection")
    

def robot_pp_angle(tar_joints, t_ms, selection):
    angle_1, angle_2, angle_3, angle_4 = tar_joints
    if is_stepper_selected(selection) :
        arm_pp_angle(angle_2, angle_3, angle_4, t_ms)
    if is_servo_selected(selection):
        servo_pp_angle(angle_1, t_ms)
    
def robot_pp_coor(tar_coor,travel_time,selection):
    print(f"robot pp coor : {tar_coor}")
    x,y,z,yaw = tar_coor
    angle_1 = servo_inverse_kinematics(z)
    print(f"robot pp coor angle_1 : {angle_1}")
    angle_2, angle_3, angle_4 = arm_inverse_kinematics(x, y, yaw)
    tar_joints = angle_1, angle_2, angle_3, angle_4
    robot_pp_angle(tar_joints, travel_time, selection)

def robot_pt_angle(tar_angles, t_ms, selection):
    tar_angle_1, tar_angle_2, tar_angle_3, tar_angle_4 = tar_angles
    if is_servo_selected(selection):
        servo_pp_angle(tar_angle_1, t_ms)
    if is_stepper_selected(selection):
        arm_pt_angle(tar_angle_2, tar_angle_3, tar_angle_4, t_ms, PT_TIME_INTERVAL)

def robot_pt_coor(tar_coor, t_ms, selection):
    x, y, z, yaw = tar_coor
    tar_angle_1 = servo_inverse_kinematics(z)
    tar_angle_2, tar_angle_3, tar_angle_4 = arm_inverse_kinematics(x, y, yaw)
    tar_angles = tar_angle_1, tar_angle_2, tar_angle_3, tar_angle_4
    robot_pt_angle(tar_angles, t_ms, selection)















##########################################################################



#DANCING












import csv
import os

def read_motion_csv(filename):
    motions = []

    with open(filename, mode='r', newline='') as file:
        reader = csv.DictReader(file)
        print("CSV fieldnames:", reader.fieldnames)  # DEBUG HEADER

        for row in reader:
            motion_type = row['motion_type']
            travel_time = int(row['travel_time'])  # in ms

            # For pp_mode: interpret as x,y,z,yaw
            d1 = float(row['d1'])
            d2 = float(row['d2'])
            d3 = float(row['d3'])
            d4 = float(row['d4'])

            motion_data = {
                'motion_type': motion_type,
                'travel_time': travel_time,
                'd1': d1,
                'd2': d2,
                'd3': d3,
                'd4': d4
            }
            # print(f"Read motion entry: {motion_data}")  # DEBUG ENTRY
            motions.append(motion_data)

    return motions

def print_motion_data(motion_data):
    i = 0
    for entry in motion_data:
        motion_type = entry['motion_type']
        travel_time = entry['travel_time']
        d1 = entry['d1']
        d2 = entry['d2']
        d3 = entry['d3']
        d4 = entry['d4']

        print(f"{i} : {motion_type}, {travel_time} ms, d1: {d1}, d2: {d2}, d3: {d3}, d4: {d4}")
        i += 1


def convert_csv_to_list_tar_coor(filepath):
    list_tar_coor = []
    with open(filepath, mode='r', newline='') as file:
        reader = csv.DictReader(file)
        for row in reader:
            if row['motion_type'] == 'pvt':
                travel_time = int(row['travel_time'])
                x = float(row['d1'])
                y = float(row['d2'])
                z = float(row['d3'])
                yaw = float(row['d4'])
                list_tar_coor.append(([x, y, z, yaw], travel_time))
    return list_tar_coor


home_angle = [360,0,0,0]
shuttle_coor = [166.82, -168, 90, 0]
pre_past_shelf_coor = [107, 125, 90, 90]
pickup_from_shelf_coor = [107, 224, 90, 90]
place_onto_shelf_coor = [107, 197, 90, 90]

def execute_motion_data(entry):
    motion_type = entry['motion_type']
    travel_time = entry['travel_time']
    d1 = entry['d1']
    d2 = entry['d2']
    d3 = entry['d3']
    d4 = entry['d4']
    
    if motion_type == 'pp_coor':
        robot_pp_coor([d1, d2, d3, d4], travel_time, selection="all")
    elif motion_type == 'pp_angle':
        robot_pp_angle([d1, d2, d3, d4], travel_time, selection="all")
    elif motion_type == 'home':
        robot_pp_angle(home_angle, travel_time, selection="all")
    elif motion_type == 'shuttle':
        robot_pp_coor(shuttle_coor, travel_time, selection="all")
    elif motion_type == 'pre_past_shelf':
        robot_pp_coor(pre_past_shelf_coor, travel_time, selection="all")
    elif motion_type == 'pickup_from_shelf':
        robot_pp_coor(pickup_from_shelf_coor, travel_time, selection="all")
    elif motion_type == 'place_onto_shelf':
        robot_pp_coor(place_onto_shelf_coor, travel_time, selection="all")
    elif motion_type == 'pvt':
        servo_pp_coor(d3, travel_time)











def pre_start_dancing(selection):
    
    robot_pp_angle(home_angle, 4000, selection)
    time.sleep(4.1)
    
    # 1) Cartesian home
    start_coor = forward_kinematics([0, 0, 0, 0])
    # 2) Daftar target (Cartesian, durasi_ms)
    list_tar_coor = [
        ([150, -100, 90, 0], 1000),
        ([107, 125, 90, 90], 1000),
        ([107, 224, 90, 90], 1000),
        ([107, 224, 115, 90], 1000),
        ([107, 125, 115, 90], 3000),
    ]

    pt1, pt2, pt3, pt4 = generate_multi_straight_pt_points(start_coor, list_tar_coor, PT_TIME_INTERVAL)
    
    
    plot_xy_from_pt(pt1, pt2, pt3, pt4)
    
    arm_pt_init()
    for i in range(len(pt2)):
        arm_pt_set_point(pt2[i], pt3[i], pt4[i])
    
    arm_pt_get_index()
    arm_pt_execute()
    time.sleep(6)
    

pt_1 = []
pt_2 = []
pt_3 = []
pt_4 = []

filename = os.path.join(os.path.dirname(os.path.abspath(__file__)), "motion_data_4.csv")
motion_enable = True
motion_cnt = 0   
motion_data = read_motion_csv(filename)
motion_size = len(motion_data)  # Set how many times to run based on the number of entries in the CSV    
dancing_tar_coor = convert_csv_to_list_tar_coor(filename)
pt_1, pt_2, pt_3, pt_4 = generate_multi_straight_pt_points(shuttle_coor, dancing_tar_coor, PT_TIME_INTERVAL)

pvt_sended = 0
max_pvt_index = 0
cur_pvt = 0
tar_pvt = 0

def robot_start_dancing():
    global pvt_sended, max_pvt_index, tar_pvt, cur_pvt
    global motion_enable, motion_cnt, motion_data, motion_size
    #qq
    x,y,z,yaw = shuttle_coor
    arm_pp_coor(x, y, yaw, 3000)
    servo_pp_coor(90, 3000)
    time.sleep(4)
    
    pvt_sended = 0
    cur_pvt = 0
    
    
    arm_pt_init()
    for i in range(len(pt_2)):
        arm_pt_set_point(pt_2[pvt_sended], pt_3[pvt_sended], pt_4[pvt_sended])
        pvt_sended += 1
    
    max_pvt_index = stepper_pvt_get_queue(8)
    
    entry = motion_data[motion_cnt]
    execute_motion_data(entry)
    motion_cnt += 1
    travel_time = entry['travel_time']
    tar_pvt = int(travel_time/PT_TIME_INTERVAL)
    arm_pt_execute()

last_depth = 0
def is_pvt_decrease(depth):
    global last_depth
    ret = 0
    if depth < last_depth:
        ret = 1
    last_depth = depth
    return ret
        
    
    
def pt_routine():
    global pvt_sended, max_pvt_index, tar_pvt, cur_pvt
    global motion_cnt, motion_data, motion_size, motion_enable
    if motion_enable:
        depth = stepper_pvt_get_queue(8)
        if is_pvt_decrease(depth):
            cur_pvt += 1
        # print(f"depth: {depth}")
        if depth != 0:
            if (depth < 40):
                if pvt_sended < max_pvt_index:
                    arm_pt_set_point(pt_2[pvt_sended], pt_3[pvt_sended], pt_4[pvt_sended])
                    pvt_sended += 1
                                            
            if cur_pvt >= tar_pvt:
                entry = motion_data[motion_cnt]
#                 # motion_type = entry['motion_type']
                travel_time = entry['travel_time']           
                execute_motion_data(entry)
                motion_cnt += 1
                tar_pvt = int(travel_time/PT_TIME_INTERVAL)
                cur_pvt = 0
        else:
            print(f"motion selesai")
            return 1
    # else:
    #     stop()
    
    return 0
    
    




def pt_test():
    x, y, yaw = arm_forward_kinematics(0,0,0)
    start_coor = [x, y, 0, yaw]
    list_tar_coor = [
        ([166.82, -168,   0,   0], 2000),
        ([166.82, -168,   0,   0], 200),
        ([258,     0,     0,   0], 2000),
        ([258,     0,     0,   0], 200),
        ([107,    125,    0,  90], 2000),
        ([107,    125,    0,  90], 200),
        ([107,    224,    0,  90], 2000),
        ([107,    224,    0,  90], 500),
        ([107,    125,    0,  90], 2000),
        ([166.82, -168,   0,   0], 2000),
    ]
    pt1, pt2, pt3, pt4 = generate_multi_straight_pt_points(
        start_coor, list_tar_coor, PT_TIME_INTERVAL
    )
    plot_xy_from_pt(pt1, pt2, pt3, pt4)

    arm_pt_init()
    for i in range(len(pt2)):
        arm_pt_set_point(pt2[i], pt3[i], pt4[i])
    
    arm_pt_get_index()
    arm_pt_execute()
    # for t in range(100):
    #     arm_pt_get_index()
    #     time.sleep(0.2)
    
    
    
# 3) Hitung PVT untuk joint1–4
    # pvt1, pvt2, pvt3, pvt4 = generate_multi_straight_pvt_points(
    #     start_coor, list_tar_coor, PT_TIME_INTERVAL
    # )
    # # 4) Node IDs untuk joint2,3,4
    # node_ids = [6, 7, 8]
    # pvts     = [pvt2, pvt3, pvt4]

    # arm_pvt_init()

    # # 6) Isi PVT: posisi, kecepatan, waktu
    # for node, (pos, vel, times) in zip(node_ids, pvts):
    #     for row in range(len(pos)):
    #         stepper_pvt_set_position_row_n(node, 0, pos[row])
    #         stepper_pvt_set_velocity_row_n(node, 0, vel[row])
    #         stepper_pvt_set_time_row_n(node, 0, times[row])

    # # 7) Mulai motion serempak
    # for node in node_ids:
    #     stepper_pvt_start_motion(node, 0)
    # stepper_begin_motion(STEPPER_GROUP_ID)