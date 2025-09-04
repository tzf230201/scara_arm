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
        #add last val
        # pt1_f.append(pulses[0])
        # pt2_f.append(pulses[1])
        # pt3_f.append(pulses[2])
        # pt4_f.append(pulses[3])
        
        # pt1_f.append(pulses[0])
        # pt2_f.append(pulses[1])
        # pt3_f.append(pulses[2])
        # pt4_f.append(pulses[3])
        
        # pt1_f.append(pulses[0])
        # pt2_f.append(pulses[1])
        # pt3_f.append(pulses[2])
        # pt4_f.append(pulses[3])
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

def generate_multi_straight_pvt_points(start_coor, list_tar_coor, dt):
    def generate_trajectory_triangle(cur_coor, list_tar_coor, dt):
        def triangle_profile(p0, p1, T, dt):
            steps = int(T / dt)
            half = steps // 2
            a = 4 * (p1 - p0) / (T ** 2)

            positions = []
            for i in range(steps + 1):
                t = i * dt
                if i <= half:
                    pos = p0 + 0.5 * a * t**2
                else:
                    t1 = t - T / 2
                    vmax = a * (T / 2)
                    pmid = p0 + 0.5 * a * (T / 2)**2
                    pos = pmid + vmax * t1 - 0.5 * a * t1**2
                positions.append(pos)
            return positions

        time_vals = []
        total_time = 0
        current = cur_coor
        traj = [[] for _ in range(4)]

        for target, T in list_tar_coor:
            for j in range(4):  # x, y, z, yaw
                interp = triangle_profile(current[j], target[j], T, dt)
                traj[j].extend(interp)
            time_vals.extend(np.arange(total_time, total_time + T + dt, dt))
            total_time += T
            current = target

        return time_vals, traj[0], traj[1], traj[2], traj[3]
    
    def convert_cartesian_traj_to_joint_traj(x_list, y_list, z_list, yaw_list):
        joint1_list, joint2_list, joint3_list, joint4_list = [], [], [], []

        for i, (x, y, z, yaw) in enumerate(zip(x_list, y_list, z_list, yaw_list)):
            joints = inverse_kinematics([x, y, z, yaw])
            
            
            joint1_list.append(joints[0])
            joint2_list.append(joints[1])
            joint3_list.append(joints[2])
            joint4_list.append(joints[3])

        return joint1_list, joint2_list, joint3_list, joint4_list
    # 1) Trajektori Cartesian + konversi ke joint (deg)
    t_arr, x_arr, y_arr, z_arr, yaw_arr = generate_trajectory_triangle(start_coor, list_tar_coor, dt)
    j1_arr, j2_arr, j3_arr, j4_arr = convert_cartesian_traj_to_joint_traj(x_arr, y_arr, z_arr, yaw_arr)

    # 2) Degree → pulse
    p1 = [stepper_deg_to_pulse(d) for d in j1_arr]
    p2 = [stepper_deg_to_pulse(d) for d in j2_arr]
    p3 = [stepper_deg_to_pulse(d) for d in j3_arr]
    p4 = [stepper_deg_to_pulse(d) for d in j4_arr]

    # 3) Hitung velocity (pulse/sec)
    dt_s = dt / 1000.0
    def compute_vel(p):
        v = [int((p[i+1] - p[i]) / dt_s) for i in range(len(p)-1)]
        v.append(0)
        return v

    v1 = compute_vel(p1)
    v2 = compute_vel(p2)
    v3 = compute_vel(p3)
    v4 = compute_vel(p4)

    # 4) Waktu per row (ms)
    times = [dt] * len(p1)

    # 5) Gabungkan menjadi list of [pos, vel, time] per index
    pvt1 = [[p1[i], v1[i], times[i]] for i in range(len(p1))]
    pvt2 = [[p2[i], v2[i], times[i]] for i in range(len(p2))]
    pvt3 = [[p3[i], v3[i], times[i]] for i in range(len(p3))]
    pvt4 = [[p4[i], v4[i], times[i]] for i in range(len(p4))]

    return pvt1, pvt2, pvt3, pvt4








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
    print(f"xyzyaw = {[x,y,z,yaw]}")
    tar_angle_1 = servo_inverse_kinematics(z)
    tar_angle_2, tar_angle_3, tar_angle_4 = arm_inverse_kinematics(x, y, yaw)
    tar_angles = tar_angle_1, tar_angle_2, tar_angle_3, tar_angle_4
    robot_pt_angle(tar_angles, t_ms, selection)

def robot_pvt_angle(tar_angles, t_ms, selection):
    tar_angle_1, tar_angle_2, tar_angle_3, tar_angle_4 = tar_angles
    if is_servo_selected(selection):
        servo_pp_angle(tar_angle_1, t_ms)
    if is_stepper_selected(selection):
        arm_pvt_angle(tar_angle_2, tar_angle_3, tar_angle_4, t_ms, PT_TIME_INTERVAL)
    
def robot_pvt_coor(tar_coor, t_ms, selection):
    x, y, z, yaw = tar_coor
    print(f"xyzyaw = {[x,y,z,yaw]}")
    tar_angle_1 = servo_inverse_kinematics(z)
    tar_angle_2, tar_angle_3, tar_angle_4 = arm_inverse_kinematics(x, y, yaw)
    tar_angles = tar_angle_1, tar_angle_2, tar_angle_3, tar_angle_4
    robot_pvt_angle(tar_angles, t_ms, selection)













##########################################################################


import csv
import os

def _get_filepath(filename):
    base_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(base_dir, filename)


def read_motion_csv(filename):
    filepath = _get_filepath(filename)
    motions = []
    with open(filepath, mode='r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            motions.append({
                'motion_type': row['motion_type'],
                'x':           float(row['x']),
                'y':           float(row['y']),
                'yaw':         float(row['yaw']),
                't_arm':       int(row['t_arm']),
                'z':           float(row['z']),
                't_servo':     int(row['t_servo']),
            })
    return motions


def print_motion_data(motions):
    for i, m in enumerate(motions):
        print(
            f"{i}: {m['motion_type']}, "
            f"x={m['x']:.2f}, y={m['y']:.2f}, yaw={m['yaw']:.2f}, "
            f"t_arm={m['t_arm']}ms, z={m['z']:.2f}, t_servo={m['t_servo']}ms"
        )


def convert_csv_to_list_tar_coor(filename):
    motions = read_motion_csv(filename)
    arm_list   = []
    servo_list = []

    for m in motions:
        if m['motion_type'].lower() != 'pvt':
            continue

        # Arm trajectory (z-plane = 0.0)
        arm_list.append(([m['x'], m['y'], m['z'], m['yaw']], m['t_arm']))
        # Servo trajectory (joint 1)
        servo_list.append((m['z'], m['t_servo']))
        
        if m['t_arm'] < m['t_servo']:
            t_idle = m['t_servo'] - m['t_arm']
            arm_list.append(([m['x'], m['y'], m['z'], m['yaw']], t_idle))
            servo_list.append((m['z'], 0))

    return arm_list, servo_list

def read_motion_list(arm_list, servo_list):
    motions = []
    for i in range(len(arm_list)):
        arm_coor, t_arm = arm_list[i]
        servo_coor, t_servo = servo_list[i]
        motion = {
            'motion_type': 'pvt',
            'x': arm_coor[0],
            'y': arm_coor[1],
            'yaw': arm_coor[3],
            't_arm': t_arm,
            'z': servo_coor,
            't_servo': t_servo,
            'travel_time': max(t_arm, t_servo)
        }
        motions.append(motion)
    return motions





home_angle = [360,0,0,0]
shuttle_coor = [166.82, -168, 90, 0]
pre_past_shelf_coor = [107, 125, 90, 90]
pickup_from_shelf_coor = [107, 224, 90, 90]
place_onto_shelf_coor = [107, 197, 90, 90]

def execute_motion_data(entry):
    motion_type = entry['motion_type']
    x = entry['x']
    y = entry['y']
    yaw = entry['yaw']
    t_arm = entry['t_arm']
    z = entry['z']
    t_servo = entry['t_servo']

    if motion_type == 'pvt':
        if t_servo > 0:
            servo_pp_coor(z, t_servo)


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

filename = "motion_data_5.csv"  
robot_tar_coor,servo_tar_coor  = convert_csv_to_list_tar_coor(filename)
motion_enable = True
motion_cnt = 0   
motion_data = read_motion_list(robot_tar_coor, servo_tar_coor)
motion_size = len(motion_data)  # Set how many times to run based on the number of entries in the CSV  
pt_1, pt_2, pt_3, pt_4 = generate_multi_straight_pt_points(shuttle_coor, robot_tar_coor, PT_TIME_INTERVAL)
pvts_1, pvts_2, pvts_3, pvts_4 = generate_multi_straight_pvt_points(shuttle_coor, robot_tar_coor, PT_TIME_INTERVAL)

pvt_sended = 0
max_pvt_index = 0
cur_pvt = 0
tar_pvt = 0

last_depth = 0
def is_pvt_decrease(depth):
    global last_depth
    ret = 0
    if depth < last_depth:
        ret = 1
    last_depth = depth
    return ret
        

def robot_start_pt_dancing():
    global pvt_sended, max_pvt_index, tar_pvt, cur_pvt
    global motion_enable, motion_cnt, motion_data, motion_size
    #qq
    x,y,z,yaw = shuttle_coor
    arm_pp_coor(x, y, yaw, 2000)
    servo_pp_coor(90, 2000)
    time.sleep(3)
    
    pvt_sended = 0
    cur_pvt = 0
    motion_cnt = 0
    
    arm_pt_init()
    for i in range(80):
        arm_pt_set_point(pt_2[pvt_sended], pt_3[pvt_sended], pt_4[pvt_sended])
        pvt_sended += 1
    
    max_pvt_index = len(pt_2)
    
    entry = motion_data[motion_cnt]
    execute_motion_data(entry)
    motion_cnt += 1
    travel_time = entry['travel_time']
    tar_pvt = int(travel_time/PT_TIME_INTERVAL)
    arm_pt_execute()


    
    
def pt_routine():
    global pvt_sended, max_pvt_index, tar_pvt, cur_pvt
    global motion_cnt, motion_data, motion_size, motion_enable
    if motion_enable:
        depth = stepper_pvt_get_queue(7)
        if is_pvt_decrease(depth):
            cur_pvt += 1
        if depth != 0:
            if (depth < 40):
                if pvt_sended < max_pvt_index:
                    
                    arm_pt_set_point(pt_2[pvt_sended], pt_3[pvt_sended], pt_4[pvt_sended])
                    pvt_sended += 1
                                            
            if cur_pvt > tar_pvt:
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
    
    
def robot_start_pvt_dancing():
    global pvt_sended, max_pvt_index, tar_pvt, cur_pvt
    global motion_enable, motion_cnt, motion_data, motion_size
    #qq
    x,y,z,yaw = shuttle_coor
    arm_pp_coor(x, y, yaw, 2000)
    servo_pp_coor(90, 2000)
    time.sleep(3)
    
    pvt_sended = 0
    cur_pvt = 0
    motion_cnt = 0
    
    arm_pvt_init()
    for i in range(80):
        arm_pvt_set_pvt(pvts_2[pvt_sended], pvts_3[pvt_sended], pvts_4[pvt_sended])
        pvt_sended += 1

    max_pvt_index = len(pvts_2)

    entry = motion_data[motion_cnt]
    execute_motion_data(entry)
    motion_cnt += 1
    travel_time = entry['t_arm']
    tar_pvt = int(travel_time/PT_TIME_INTERVAL)
    arm_pvt_execute()
    
def pvt_routine():
    global pvt_sended, max_pvt_index, tar_pvt, cur_pvt
    global motion_cnt, motion_data, motion_size, motion_enable
    if motion_enable:
        depth = stepper_pvt_get_queue(7)
        if is_pvt_decrease(depth):
            cur_pvt += 1
        if depth != 0:
            if (depth < 40):
                if pvt_sended < max_pvt_index:
                    arm_pvt_set_pvt(pvts_2[pvt_sended], pvts_3[pvt_sended], pvts_4[pvt_sended])
                    pvt_sended += 1
                                            
            if cur_pvt >= tar_pvt:
                entry = motion_data[motion_cnt]
#                 # motion_type = entry['motion_type']
                travel_time = entry['t_arm']           
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

# def pt_test():
#     x, y, yaw = arm_forward_kinematics(0,0,0)
#     start_coor = [x, y, 0, yaw]
#     list_tar_coor = [
#         ([166.82, -168,   0,   0], 2000),
#         ([166.82, -168,   0,   0], 200),
#         ([258,     0,     0,   0], 2000),
#         ([258,     0,     0,   0], 200),
#         ([107,    125,    0,  90], 2000),
#         ([107,    125,    0,  90], 200),
#         ([107,    224,    0,  90], 2000),
#         ([107,    224,    0,  90], 500),
#         ([107,    125,    0,  90], 2000),
#         ([166.82, -168,   0,   0], 2000),
#     ]
#     pt1, pt2, pt3, pt4 = generate_multi_straight_pt_points(
#         start_coor, list_tar_coor, PT_TIME_INTERVAL
#     )
#     plot_xy_from_pt(pt1, pt2, pt3, pt4)

#     arm_pt_init()
#     for i in range(len(pt2)):
#         arm_pt_set_point(pt2[i], pt3[i], pt4[i])
    
#     arm_pt_get_index()
#     arm_pt_execute()





# def pvt_test():
#     x, y, yaw = arm_forward_kinematics(0,0,0)
#     start_coor = [x, y, 0, yaw]
#     list_tar_coor = [
#         ([166.82, -168,   0,   0], 2000),
#         ([258,     0,     0,   0], 2000),
#         ([107,    125,    0,  90], 2000),
#         ([107,    224,    0,  90], 2000),
#         ([107,    125,    0,  90], 2000),
#         ([166.82, -168,   0,   0], 2000),
#     ]
#     pvts_1, pvts_2, pvts_3, pvts_4 = generate_multi_straight_pvt_points(
#         start_coor, list_tar_coor, PT_TIME_INTERVAL
#     )
#     N = len(pvts_1)
#     arm_pvt_init()
#     # # 6) Isi PVT: posisi, kecepatan, waktu
#     for i in range(N):
#         arm_pvt_set_pvt(pvts_2[i], pvts_3[i], pvts_4[i])

#     arm_pvt_get_index()
#     arm_pvt_execute()
    