from stepper import *
import math

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
    enc_2 = stepper_get_pa(6)
    enc_3 = stepper_get_pa(7)
    enc_4 = stepper_get_pa(8)
    return enc_2, enc_3, enc_4

def arm_get_angle():
    enc_2, enc_3, enc_4 = arm_get_enc()
    cur_angle_2 = stepper_pulse_to_deg(enc_2)
    cur_angle_3 = stepper_pulse_to_deg(enc_3)
    cur_angle_4 = stepper_pulse_to_deg(enc_4)
    return cur_angle_2, cur_angle_3, cur_angle_4

def arm_check_limit(angle_2, angle_3, angle_4):
    angle_2_upper_limit = 178 * 5
    angle_2_lower_limit = 0
    angle_3_upper_limit = 0 + angle_2
    angle_3_lower_limit = (-135 * 5) + angle_2
    angle_4_upper_limit = (196 * 5) + angle_3
    angle_4_lower_limit = 0 + angle_3
    angle_4 *= -1

    if angle_2 > angle_2_upper_limit:
        print(f"angle_2 greater than {angle_2}")
        angle_2 = angle_2_upper_limit
    elif angle_2 < angle_2_lower_limit:
        print(f"angle_2 lower than {angle_2}")
        angle_2 = angle_2_lower_limit

    if angle_3 > angle_3_upper_limit:
        print(f"angle_3 greater than {angle_3}")
        angle_3 = angle_3_upper_limit
    elif angle_3 < angle_3_lower_limit:
        print(f"angle_3 lower than {angle_3}")
        angle_3 = angle_3_lower_limit

    if angle_4 > angle_4_upper_limit:
        print(f"angle_4 greater than {angle_4}")
        angle_4 = angle_4_upper_limit
    elif angle_4 < angle_4_lower_limit:
        print(f"angle_4 lower than {angle_4}")
        angle_4 = angle_4_lower_limit

    angle_4 *= -1
    return angle_2, angle_3, angle_4

def arm_inverse_kinematics(x, y, yaw):
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

    joint_2 = (theta2 - OFFSET_2) * RATIO
    joint_3 = (theta3 - OFFSET_3) * RATIO + joint_2
    joint_4 = -((yaw - OFFSET_4) * RATIO)

    angle_2, angle_3, angle_4 = arm_check_limit(joint_2, joint_3, joint_4)
    return angle_2, angle_3, angle_4

def arm_forward_kinematics(angle_2, angle_3, angle_4):
    angle_4 *= -1.0
    L2 = 137.0
    L3 = 121.0
    OFFSET_2 = -96.5
    OFFSET_3 = 134.0
    OFFSET_4 = -52.5

    theta2_rad = math.radians((angle_2 / 5) + OFFSET_2)
    theta3_rad = math.radians((angle_3 / 5) + OFFSET_3 - (angle_2 / 5))

    x2 = L2 * math.cos(theta2_rad)
    y2 = L2 * math.sin(theta2_rad)
    x3 = x2 + L3 * math.cos(theta2_rad + theta3_rad)
    y3 = y2 + L3 * math.sin(theta2_rad + theta3_rad)
    yaw = (angle_4 / 5) + OFFSET_4

    return x3, y3, yaw

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

### PP

def arm_pp_angle(angle_2, angle_3, angle_4, t_ms):
    angle_2, angle_3, angle_4 = arm_check_limit(angle_2, angle_3, angle_4)
    arm_tar_joints = [angle_2, angle_3, angle_4]
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
    
def arm_pp_coor(x, y, yaw, t_ms):
    angle_2, angle_3, angle_4 = arm_inverse_kinematics(x, y, yaw)
    arm_pp_angle(angle_2, angle_3, angle_4, t_ms)


### PT
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
    
def arm_pt_angle(angle_2, angle_3, angle_4, t_ms, pt_time_interval=PT_TIME_INTERVAL):

    node_ids=[6,7,8]
    # 1. Pastikan joint dalam limit, lalu ambil 3 joint arm
    angle_2, angle_3, angle_4 = arm_check_limit(angle_2, angle_3, angle_4)
    target_deg = [angle_2, angle_3, angle_4]

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
    
    trajectory.append(traj)
    trajectory.append(traj)
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
            pt = stepper_pvt_get_pt_data_row_n(node, idx)
            pa = stepper_pulse_to_deg(pt)
            print(f"node {node} index {idx} is {pa}")

    # 7. Mulai motion serempak
    for node in node_ids:
        stepper_pvt_start_motion(node, 0)
    stepper_begin_motion(STEPPER_GROUP_ID)

def arm_pt_coor(x, y, yaw, t_ms, pt_time_interval=PT_TIME_INTERVAL):
    tar_joints = arm_inverse_kinematics([x, y, yaw])
    print(f"[arm_pt_coor] tar_coor={[x, y, yaw]} → tar_joints={tar_joints}")
    arm_pt_angle(tar_joints, t_ms, pt_time_interval)
    
### PVT 
   
def arm_pvt_init():
    arm_set_group_id()
    stepper_pvt_clear_queue(STEPPER_GROUP_ID)
    stepper_pvt_set_first_valid_row(STEPPER_GROUP_ID, 0)
    stepper_pvt_set_last_valid_row(STEPPER_GROUP_ID, 250)
    stepper_pvt_set_management_mode(STEPPER_GROUP_ID, 0)
    stepper_pvt_set_pt_time(STEPPER_GROUP_ID, 0)

def stepper_pvt_set_pvt_value(node_id, pvt):
    p, v, t = pvt
    stepper_pvt_set_position_row_n(node_id, 0, p)
    stepper_pvt_set_velocity_row_n(node_id, 0, v)
    stepper_pvt_set_time_row_n(node_id, 0, t)
    
def arm_pvt_execute():
    stepper_pvt_start_motion(STEPPER_GROUP_ID, 0)
    stepper_begin_motion(STEPPER_GROUP_ID)

def arm_pvt_get_index():
    n2 = stepper_pvt_get_queue(6)
    n3 = stepper_pvt_get_queue(7)
    n4 = stepper_pvt_get_queue(8)
    print(f"[dance] queue: n2={n2}, n3={n3}, n4={n4}")




