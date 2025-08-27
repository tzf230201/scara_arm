from stepper import *
import math

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
    
def arm_pvt_init():
    stepper_pvt_clear_queue(8)
    stepper_pvt_set_first_valid_row(8, 0)
    stepper_pvt_set_last_valid_row(8,500)
    stepper_pvt_set_management_mode(8, 0)
    stepper_pvt_set_pt_time(8, 20)
    for n in range(100):
        pulse = stepper_deg_to_pulse(-n)
        stepper_pvt_set_pt_data_row_n(8, 0, pulse)

    row = stepper_pvt_get_queue(8)
    print(f"Initial PVT queue: {row} rows")
    
    for m in range(100):
        pulse = stepper_pvt_get_pt_data_row_n(8, m)
        print(f"Row {m}: {pulse} pulse, {stepper_pulse_to_deg(pulse):.2f} deg")
    stepper_pvt_start_motion(8, 0)
    
    