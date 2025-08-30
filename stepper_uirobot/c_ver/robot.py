from motion import *
from servo import *
from stepper import *
from arm import *
import time


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
        servo_shutdown()
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
    if is_stepper_selected() :
        arm_pp_angle(angle_2, angle_3, angle_4, t_ms)
    if is_servo_selected():
        servo_pp_angle(angle_1, t_ms)
    
def robot_pp_coor(tar_coor,travel_time,selection):
    x,y,z,yaw = tar_coor
    angle_1 = servo_inverse_kinematics(z)
    angle_2, angle_3, angle_4 = arm_inverse_kinematics(x, y, yaw)
    tar_joints = angle_1, angle_2, angle_3, angle_4
    robot_pp_angle(tar_joints, travel_time, selection)

def robot_pt_angle(tar_joints, travel_time, selection):
    print(f"entering robot pt angle")

def robot_pt_coor(tar_coor, travel_time, selection):
    print(f"entering robot pt coor")

def robot_dancing(selection):
    print(f"robot_dancing({selection})")
    
    
def pre_start_dancing():
    # 1) Cartesian home
    start_coor = forward_kinematics([0, 0, 0, 0])
    # 2) Daftar target (Cartesian, durasi_ms)
    list_tar_coor = [
        ([166.82, -168,   0,   0], 1000),
        ([258,     0,     0,   0], 1000),
        ([107,    125,    0,  90], 1000),
    ]

    # 3) Hitung PVT untuk joint1–4
    pvt1, pvt2, pvt3, pvt4 = generate_multi_straight_pvt_points(
        start_coor, list_tar_coor, PT_TIME_INTERVAL
    )

    # 4) Node IDs untuk joint2,3,4
    node_ids = [6, 7, 8]
    pvts     = [pvt2, pvt3, pvt4]

    arm_pvt_init()

    # 6) Isi PVT: posisi, kecepatan, waktu
    for node, (pos, vel, times) in zip(node_ids, pvts):
        for row in range(len(pos)):
            stepper_pvt_set_position_row_n(node, 0, pos[row])
            stepper_pvt_set_velocity_row_n(node, 0, vel[row])
            stepper_pvt_set_time_row_n(node, 0, times[row])

    # 7) Mulai motion serempak
    for node in node_ids:
        stepper_pvt_start_motion(node, 0)
    stepper_begin_motion(STEPPER_GROUP_ID)

def pt_test():
    x, y, yaw = arm_forward_kinematics(0,0,0)
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