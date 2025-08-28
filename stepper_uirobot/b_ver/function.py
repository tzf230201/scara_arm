from stepper import *
import math
import time

STEPPER_GROUP_ID = 10
STEPPER_MICROSTEP = 128
STEPPER_CPR = 200 * STEPPER_MICROSTEP  # 200 step/rev x 128 microstep = 25600 pulse/rev

# Satuan: degree/pulse dan pulse/degree
STEPPER_DEG_PER_PULSE = 360.0 / STEPPER_CPR   # (ex: 0.0140625 deg/pulse)
STEPPER_PULSE_PER_DEG = STEPPER_CPR / 360.0   # (ex: 71.111... pulse/deg)

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

def stepper_pulse_to_deg(pulse):
    return float(pulse) * STEPPER_DEG_PER_PULSE

def stepper_deg_to_pulse(deg):
    return int(round(float(deg) * STEPPER_PULSE_PER_DEG))

def stepper_get_all_enc():
    pa2 = stepper_get_pa(6)
    pa3 = stepper_get_pa(7)
    pa4 = stepper_get_pa(8)
    return pa2, pa3, pa4

def stepper_get_all_angle():
    pa2, pa3, pa4 = stepper_get_all_enc()
    pa2_deg = stepper_pulse_to_deg(pa2)
    pa3_deg = stepper_pulse_to_deg(pa3)
    pa4_deg = stepper_pulse_to_deg(pa4)
    return pa2_deg, pa3_deg, pa4_deg

def stepper_set_all_motor_on():
    stepper_set_mo(6, 1)
    stepper_set_mo(7, 1)
    stepper_set_mo(8, 1)

def stepper_set_all_motor_off():
    stepper_set_mo(6, 0)
    stepper_set_mo(7, 0)
    stepper_set_mo(8, 0)

def stepper_set_all_group_id():
    stepper_set_group_id(6, STEPPER_GROUP_ID)
    stepper_set_group_id(7, STEPPER_GROUP_ID)
    stepper_set_group_id(8, STEPPER_GROUP_ID)

def stepper_set_all_unit_ac_dc():
    stepper_set_ac_dc_unit(6, 0)
    stepper_set_ac_dc_unit(7, 0)
    stepper_set_ac_dc_unit(8, 0)

def stepper_set_all_micro_stepping():
    stepper_set_micro_stepping_resolution(6, STEPPER_MICROSTEP)
    stepper_set_micro_stepping_resolution(7, STEPPER_MICROSTEP)
    stepper_set_micro_stepping_resolution(8, STEPPER_MICROSTEP)

def stepper_set_all_ptp_finish_notification_off():
    stepper_set_ptp_finish_notification(6, 0)
    stepper_set_ptp_finish_notification(7, 0)
    stepper_set_ptp_finish_notification(8, 0)
    
def stepper_set_all_working_current(current):
    stepper_set_working_current(6, current)
    stepper_set_working_current(7, current)
    stepper_set_working_current(8, current)

def init_stepper():
    stepper_set_all_motor_off()
    stepper_set_all_working_current(0.5)
    stepper_set_all_ptp_finish_notification_off()
    stepper_set_all_group_id()
    stepper_set_all_unit_ac_dc()
    stepper_set_all_micro_stepping()
    stepper_set_all_motor_on()
    
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

def arm_pp_angle(tar_joints, t_ms):
    tar_joints = check_limit(tar_joints)
    arm_tar_joints = tar_joints[1:4]
    node_ids = [6, 7, 8]
    pulses = [stepper_deg_to_pulse(j) for j in arm_tar_joints]
    pa_now = [stepper_get_pa(n) for n in node_ids]
    delta = [abs(p - p_now) for p, p_now in zip(pulses, pa_now)]
    t = max(t_ms / 1000.0, 1e-3)

    for node_id, pos, d in zip(node_ids, pulses, delta):
        v_m = int(2 * d / t) if t > 0 else 1
        ac_m = int(4 * d / (t ** 2)) if t > 0 else 1
        stepper_set_ac(node_id, ac_m)
        stepper_set_dc(node_id, ac_m)
        stepper_set_pa(node_id, pos)
        stepper_set_sp(node_id, v_m)
    stepper_begin_motion(STEPPER_GROUP_ID)
    
def arm_pp_coor(tar_coor, t_ms):
    tar_joints = inverse_kinematics(tar_coor)
    arm_pp_angle(tar_joints, t_ms)

############################################################## PVT
PT_TIME_INTERVAL = 50
    
def generate_multi_straight_pt_points(start_coor, list_tar_coor, pt_time_interval=PT_TIME_INTERVAL):
    pt1_f, pt2_f, pt3_f, pt4_f = [], [], [], []
    last_coor = start_coor

    for tar_coor, traveltime in list_tar_coor:
        n_step = max(int(round(traveltime / pt_time_interval)), 1)
        for i in range(n_step):
            alpha = i / n_step
            coor = [
                last_coor[j] + (tar_coor[j] - last_coor[j]) * alpha
                for j in range(4)
            ]
            # Panggil dengan satu list:
            joint = inverse_kinematics(coor)    # ← tar_coor adalah [x,y,z,yaw]
            pulses = [stepper_deg_to_pulse(j) for j in joint]
            pt1_f.append(pulses[0])
            pt2_f.append(pulses[1])
            pt3_f.append(pulses[2])
            pt4_f.append(pulses[3])
        last_coor = tar_coor

    # Titik akhir:
    final_joint = inverse_kinematics(list_tar_coor[-1][0])
    pulses = [stepper_deg_to_pulse(j) for j in final_joint]
    pt1_f.append(pulses[0])
    pt2_f.append(pulses[1])
    pt3_f.append(pulses[2])
    pt4_f.append(pulses[3])

    return pt1_f, pt2_f, pt3_f, pt4_f



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


# Panggil ini setelah generate pt1, pt2, pt3, pt4:

def arm_pt_init():
    stepper_set_all_group_id()
    stepper_pvt_clear_queue(STEPPER_GROUP_ID)
    stepper_pvt_set_first_valid_row(STEPPER_GROUP_ID, 0)
    stepper_pvt_set_last_valid_row(STEPPER_GROUP_ID, 500)
    stepper_pvt_set_management_mode(STEPPER_GROUP_ID, 0)
    stepper_pvt_set_pt_time(STEPPER_GROUP_ID, PT_TIME_INTERVAL)

def arm_pt_set_point(pt2, pt3, pt4):
    stepper_pvt_set_pt_data_row_n(6, 0, pt2)
    stepper_pvt_set_pt_data_row_n(7, 0, pt3)
    stepper_pvt_set_pt_data_row_n(8, 0, pt4)
    
def arm_pt_execute():
    stepper_pvt_start_motion(6, 0)
    stepper_pvt_start_motion(7, 0)
    stepper_pvt_start_motion(8, 0)
    stepper_begin_motion(STEPPER_GROUP_ID)

def arm_pt_get_index():
    n2 = stepper_pvt_get_queue(6)
    n3 = stepper_pvt_get_queue(7)
    n4 = stepper_pvt_get_queue(8)
    print(f"[dance] queue: n2={n2}, n3={n3}, n4={n4}")

def pre_start_dancing():
    start_coor = forward_kinematics([0, 0, 0, 0])
    list_tar_coor = [
        ([150, -100, 90, 0], 2000),
        ([150,   0,   90, 87], 2000),
        ([107,   50,  90, 87], 2000)
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
    for t in range(100):
        arm_pt_get_index()
        time.sleep(0.2)