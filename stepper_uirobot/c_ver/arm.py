from stepper import *
from motion import *

PT_TIME_INTERVAL = 100

STEPPER_GROUP_ID = 10
stepper_ids = [6, 7, 8]

def arm_set_motor_off():
    for node_id in stepper_ids:
        stepper_set_mo(node_id, 0)
        
def arm_set_group_id():
    for node_id in stepper_ids:
        stepper_set_group_id(node_id, STEPPER_GROUP_ID)

def arm_set_unit_ac_dc():
    stepper_set_ac_dc_unit(STEPPER_GROUP_ID, 0)



def arm_set_motor_on():
    stepper_set_mo(STEPPER_GROUP_ID, 1)



def arm_get_enc():
    pa2 = stepper_get_pa(6)
    pa3 = stepper_get_pa(7)
    pa4 = stepper_get_pa(8)
    return pa2, pa3, pa4

def arm_get_angle():
    pa2, pa3, pa4 = arm_get_enc()
    pa2_deg = stepper_pulse_to_deg(pa2)
    pa3_deg = stepper_pulse_to_deg(pa3)
    pa4_deg = stepper_pulse_to_deg(pa4)
    return pa2_deg, pa3_deg, pa4_deg



def arm_set_micro_stepping():
    stepper_set_micro_stepping_resolution(STEPPER_GROUP_ID, STEPPER_MICROSTEP)

def arm_set_ptp_finish_notification_off():
    stepper_set_ptp_finish_notification(STEPPER_GROUP_ID, 0)
    
def arm_set_working_current(current):
    stepper_set_working_current(STEPPER_GROUP_ID, current)

def arm_set_origin():
    stepper_set_origin(6)
    stepper_set_origin(7)
    stepper_set_origin(8)
    
def arm_init():
    arm_set_motor_off()
    arm_set_group_id()
    arm_set_working_current(1.6)
    arm_set_ptp_finish_notification_off()
    arm_set_unit_ac_dc()
    arm_set_micro_stepping()
    arm_set_motor_on()

def arm_pt_init():
    arm_set_group_id()
    stepper_pvt_clear_queue(STEPPER_GROUP_ID)
    stepper_pvt_set_first_valid_row(STEPPER_GROUP_ID, 0)
    stepper_pvt_set_last_valid_row(STEPPER_GROUP_ID, 500)
    stepper_pvt_set_management_mode(STEPPER_GROUP_ID, 0)
    stepper_pvt_set_pt_time(STEPPER_GROUP_ID, PT_TIME_INTERVAL)

def arm_pt_set_point(pt2, pt3, pt4):
    for (node_id, pt) in zip(stepper_ids, [pt2, pt3, pt4]):
        stepper_pvt_set_pt_data_row_n(node_id, 0, pt)
        
def arm_pt_execute():
    stepper_pvt_start_motion(STEPPER_GROUP_ID, 0)
    stepper_begin_motion(STEPPER_GROUP_ID)

def arm_pt_get_index():
    n2 = stepper_pvt_get_queue(6)
    n3 = stepper_pvt_get_queue(7)
    n4 = stepper_pvt_get_queue(8)
    print(f"[dance] queue: n2={n2}, n3={n3}, n4={n4}")
    
def pt_test():
    start_coor = forward_kinematics([0, 0, 0, 0])
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
    
def arm_pt_angle(tar_joints, t_ms, pt_time_interval=PT_TIME_INTERVAL):

    node_ids=[6,7,8]
    # 1. Pastikan joint dalam limit, lalu ambil 3 joint arm
    tar_joints = check_limit(tar_joints)
    target_deg = tar_joints[1:4]

    # 2. Konversi target → pulse, baca posisi sekarang (pulse)
    pulses_target = [stepper_deg_to_pulse(d) for d in target_deg]
    pulses_now    = [stepper_get_pa(n)       for n in node_ids]

    # 3. Hitung jumlah step untuk interpolation
    n_step = max(int(round(t_ms / pt_time_interval)), 1)

    # 4. Bangun trajectory pulse per node: list of [p1,p2,p3] tiap step
    trajectory = []
    for i in range(n_step+1):
        alpha = i / n_step
        traj = [
            int(p_now + (p_tgt - p_now) * alpha)
            for p_now, p_tgt in zip(pulses_now, pulses_target)
        ]
        trajectory.append(traj)

    # 5. Setup PVT queue di tiap node
    for node in node_ids:
        stepper_pvt_clear_queue(node)
        stepper_pvt_set_first_valid_row(node, 0)
        stepper_pvt_set_last_valid_row (node, len(trajectory))
        stepper_pvt_set_management_mode(node, 0)
        stepper_pvt_set_pt_time(node, pt_time_interval)

    # 6. Isi queue: setiap row untuk semua node
    for idx, row_pulses in enumerate(trajectory):
        for node, p in zip(node_ids, row_pulses):
            stepper_pvt_set_pt_data_row_n(node, idx, p)

    # 7. Mulai motion serempak
    for node in node_ids:
        stepper_pvt_start_motion(node, 0)
    stepper_begin_motion(STEPPER_GROUP_ID)

def arm_pt_coor(tar_coor, t_ms, pt_time_interval=PT_TIME_INTERVAL):
    tar_joints = inverse_kinematics(tar_coor)
    print(f"[arm_pt_coor] tar_coor={tar_coor} → tar_joints={tar_joints}")
    arm_pt_angle(tar_joints, t_ms, pt_time_interval)
    
def arm_pvt_init():
    stepper_set_all_group_id()
    stepper_pvt_clear_queue(STEPPER_GROUP_ID)
    stepper_pvt_set_first_valid_row(STEPPER_GROUP_ID, 0)
    stepper_pvt_set_last_valid_row(STEPPER_GROUP_ID, 250)
    stepper_pvt_set_management_mode(STEPPER_GROUP_ID, 0)
    stepper_pvt_set_pt_time(STEPPER_GROUP_ID, 0)
    

    #     time.sleep(0.2)

def stepper_pvt_set_pvt_value(node_id, pvt):
    p, v, t = pvt
    stepper_pvt_set_position_row_n(node_id, 0, p)
    stepper_pvt_set_velocity_row_n(node_id, 0, v)
    stepper_pvt_set_time_row_n(node_id, 0, t)


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


