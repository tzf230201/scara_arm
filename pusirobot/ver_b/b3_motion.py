import time
import math
from b1_servo import *
from b1_stepper import *
from b2_pvt import *

def get_cur_joints():
    servo_id = ID1
    servo_pulse = servo_get_motor_position(servo_id)
    servo_angle = servo_pulses_to_degrees(servo_pulse)
    
    stepper_ids = [ID2, ID3, ID4]
    stepper_angles = []
    
    for stepper_id in stepper_ids:
        stepper_pulse = stepper_get_motor_position(stepper_id)
        # print(f"stepper {stepper_id:03X} pulse is {stepper_pulse}")
        stepper_angles.append(stepper_pulses_to_degrees(stepper_pulse))
    
    cur_joints = [servo_angle, stepper_angles[0], stepper_angles[1], stepper_angles[2]]
    return cur_joints

# ######################################### SP MODE ######################################### #

from b2_sp import * 

def sp_angle(tar_joints, travel_time):
    global stop_watch, time_out, last_time
    group_id = 5
    sp_mode_init(group_id)
    
    cur_joints = get_cur_joints()
    delta_joints = [tar - cur for tar, cur in zip(tar_joints, cur_joints)]
    tar_speeds = [stepper_degrees_to_pulses(int(delta / travel_time)) for delta in delta_joints]
    
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
    
def sp_coor(tar_coor, travel_time):
    tar_joints = inverse_kinematics(tar_coor)
    tar_joints = check_limit(tar_joints)
    print(f"tar joint = {tar_joints} degree")
    sp_angle(tar_joints, travel_time)

# ######################################### PVT 3 ######################################### #
from b2_pp import *

def pp_angle(tar_joints, travel_time, max_speed):
    
    pp_mode_init()
    
    cur_joints = get_cur_joints()
    
    cur_joint_1, cur_joint_2, cur_joint_3, cur_joint_4 = cur_joints
    tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4 = tar_joints
    
    tar_pulse_1 = servo_degrees_to_pulses(tar_joint_1)
    tar_pulse_2 = stepper_degrees_to_pulses(tar_joint_2)
    tar_pulse_3 = stepper_degrees_to_pulses(tar_joint_3)
    tar_pulse_4 = stepper_degrees_to_pulses(tar_joint_4)
    
    delta_pulse_1 = servo_degrees_to_pulses(tar_joint_1 - cur_joint_1)
    delta_pulse_2 = stepper_degrees_to_pulses(tar_joint_2 - cur_joint_2)
    delta_pulse_3 = stepper_degrees_to_pulses(tar_joint_3 - cur_joint_3)
    delta_pulse_4 = stepper_degrees_to_pulses(tar_joint_4 - cur_joint_4)
    
    accel_decel_1, max_speed_1 = servo_accel_decel_calc(delta_pulse_1, travel_time)
    accel_decel_2, max_speed_2 = stepper_accel_decel_calc(delta_pulse_2, travel_time)
    accel_decel_3, max_speed_3 = stepper_accel_decel_calc(delta_pulse_3, travel_time)
    accel_decel_4, max_speed_4 = stepper_accel_decel_calc(delta_pulse_4, travel_time)
    
    pp_mode_set_acceleration(accel_decel_2, accel_decel_3, accel_decel_4)
    pp_mode_set_deceleration(accel_decel_2, accel_decel_3, accel_decel_4)
    pp_mode_set_max_speed(max_speed_2, max_speed_3, max_speed_4)
    pp_mode_set_tar_pulse(tar_pulse_2, tar_pulse_3, tar_pulse_4)
    
    accel_decel_2 = stepper_pulses_to_steps(accel_decel_2)
    accel_decel_3 = stepper_pulses_to_steps(accel_decel_3)
    accel_decel_4 = stepper_pulses_to_steps(accel_decel_4)
    
    set_sdo(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x0F)
    #max speed is in pps, we need to convert it to ppr, and 1 ppr is 10 in the servo
    max_speed_rps = servo_pps_to_rps(max_speed)
    #coba ulang lagi yang stepper agar mengikuti si servo
    
    servo_set_acceleration(accel_decel_1)
    servo_set_deceleration(accel_decel_1)
    servo_set_max_speed(max_speed_1)
    servo_set_tar_pulse(tar_pulse_1)
    
    
    pp_mode_start_absolute_motion()
    set_sdo(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x1F)
    
def pp_coor(tar_coor, travel_time, max_speed):
    tar_joints = inverse_kinematics(tar_coor)
    tar_joints = check_limit(tar_joints)
    print(f"tar joint = {tar_joints} degree")
    pp_angle(tar_joints, travel_time, max_speed)

# ######################################### PVT 3 ######################################### #


def pvt_mode_try_pvt_3(cur_joints, tar_joints, travel_time):
    global last_time, stop_watch, time_out
    group_id = 0x05
    tar_pulses = []
    node_ids = [ID1, ID2, ID3, ID4]
    pvt_3_lower_limit = 60
    pvt_3_upper_limit = 80

    pvt_mode_init(group_id, PVT_3, 400, pvt_3_lower_limit, pvt_3_upper_limit)
    
    for tar_joint in tar_joints:
        tar_pulses.append(stepper_degrees_to_pulses(tar_joint))
    
    
    # Initialize p, v, and t as empty lists
    p = [None] * 4
    v = [None] * 4
    t = [None] * 4
    
    #qq
    for i in range(1, 4):
        tar_step = stepper_pulses_to_steps(tar_pulses[i])
        p[i], v[i], t[i] = generate_pvt_trajectory_triangle_2(0 ,  tar_step, travel_time) 
    
    for i in range(1, 4):
        for pos, vel, tim in zip(p[i], v[i], t[i]):
            pvt_mode_write_read(node_ids[i], pos, vel, tim)
            # print(f"motor {i+1} write {pos}, {vel}, {tim}")
                
                
    pvt_mode_read_pvt_3_depth()
    pvt_mode_start_pvt_step(group_id)
    last_time = time.time()
    stop_watch = last_time
    time_out = travel_time
    
    
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
    time_out = travel_time





    
    
    
# ######################################### KINEMATICS ######################################### #

def check_limit(tar_joints):
    
    tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4 = tar_joints
    tar_joint_4 *= -1
    #limit2 = 0 to 178 degree
    #limit3 = -135 to 0 degree
    #limit4 = 0 to 196 degree
    
    if tar_joint_1 > (13004):
        tar_joint_1 = 13004
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
    sleep = travel_time + 0.1
    
    coor_1 = [258, 0, 0, 0]
    coor_2 = [150, 0, 0, 0]
        
    for i in range(3):
        sp_coor(coor_1, travel_time)
        time.sleep(sleep)
        sp_coor(coor_2, travel_time)
        time.sleep(sleep)
    
def rectangular(travel_time):
    sleep = travel_time + 0.1
    
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
    sleep = travel_time + 0.1
    
    home_angles = [0, 0, 0, 0]
    sp_angle(home_angles, travel_time)
    time.sleep(sleep)

		
def shuttle_position(travel_time):
    sleep = travel_time
    
    shuttle_coor = [166.82, -168, 0, 0]
    sp_coor(shuttle_coor, travel_time)
    time.sleep(sleep)

def pre_past_shelf(travel_time):
    sleep = travel_time
    
    pre_past_shelf_coor = [107, 100, 0, 90]
    sp_coor(pre_past_shelf_coor, travel_time)
    time.sleep(sleep)


def pickup_from_shelf(travel_time):
    sleep = travel_time
    
    pickup_from_shelf_coor = [107, 224, 0, 90]
    sp_coor(pickup_from_shelf_coor, travel_time)
    time.sleep(sleep)

def place_onto_shelf(travel_time):
    sleep = travel_time
    
    place_from_shelf_coor = [107, 197, 0, 90]
    sp_coor(place_from_shelf_coor, travel_time)
    time.sleep(sleep)

        
def dancing(travel_time):
    sleep = travel_time + 0.1

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
    
    
def dancing2(travel_time):
    sleep = travel_time + 0.1
    pp_travel = sleep * 1000
    for i in range(10):
        pp_angle([1200, 0, 0, 0], pp_travel, 10000)
        time.sleep(sleep)
        pp_angle([0, 0, 0, 0], pp_travel, 10000)
        time.sleep(sleep)