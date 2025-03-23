import time
import math
from micro_can import *


    
# suggestion:
# motor position pada pdo1 sepertinya tidak dibutuhkan, kalo dihapus program kalkulasinya bisa hemat waktu
# coba rangkap data yang dikitim lewat pvt ke pdo saja bisa hemat waktu

#note:
#speed in sp mode is relative to microstep
#reach position information in sp mode can be read from controller status (busy_state)
#pvt mode is more like speed based rather thank position based


#to do:
# 1. robot harus bisa menjalankan s-shape motion
# bagaimana cara ngetes s-shape motion?
# 2. buat pvt mode relative terhadap current position
# 3. pvt mode with sp correction
# 4. make xyz with time control (4D)
# 5. try PP mode again with pulse_to_step() function
# 6. make the third file


#key role:
# Target : The robot must follow a predefined trajectory to pick up boxes.
# must to have: 
# 1. Organic movement (smooth acceleration & deceleration)
#    - make a 4D trajectory tester
#    - try PP mode again with pulse_to_step() function
#               
# 2. Precision & no incremental drift
#    - maybe can be combining with sp mode after
#    - found what cause the drift in PVT mode.
#               
# 3. anomaly detection & activate Emergency response
#    - decide the pin for the servo brake (GPIO2 - Pin3)
#    - read the position frequently, if the motor out of tolerance, activaate the emergency functio

# ######################################### PVT MODE ######################################### #
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
    
    joint1_steps = [pulse_to_step(val) for val in joint1_pulses]
    joint2_steps = [pulse_to_step(val) for val in joint2_pulses]
    joint3_steps = [pulse_to_step(val) for val in joint3_pulses]
    joint4_steps = [pulse_to_step(val) for val in joint4_pulses]
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

def pulse_to_step(tar_pulse):
    step = int(int((tar_pulse * (MICROSTEP* 200)) / 4096))
    return step


# ######################################### SP MODE ######################################### #

def sp_angle(tar_joints, travel_time):
    global stop_watch, time_out, last_time
    group_id = 5
    init_operation_mode(0)
    #group id need to be set after changing operation mode
    init_change_group_id(group_id)
    # init_set_accel_coef(1)
    # init_set_decel_coef(1)
    
    cur_joints = read_present_position()
    delta_joints = [tar - cur for tar, cur in zip(tar_joints, cur_joints)]
    tar_speeds = [stepper_degrees_to_pulses(int(delta / travel_time)) for delta in delta_joints]
    
    tar_pulses = []
    for tar_joint in tar_joints:
        tar_pulses.append(stepper_degrees_to_pulses(tar_joint))

    # Set speed
    for id, speed in zip([ID2, ID3, ID4], [tar_speeds[1], tar_speeds[2], tar_speeds[3]]):
        speed_have_to_write = int(int((speed * (MICROSTEP* 200)) / 4096))
        _,ret = set_req_sdo(id, SET_4_BYTE, OD_STEPPER_SP_MOTION, 0x01, speed_have_to_write)
        print(f"{id:03X} sp speed is {ret}")  
    #set position
    for id, pulse in zip([ID2, ID3, ID4], [tar_pulses[1], tar_pulses[2], tar_pulses[3]]):
        _,ret = set_req_sdo(id, SET_4_BYTE, OD_STEPPER_SP_MOTION, 0x02, pulse)
        print(f"{id:03X} sp position is {ret}")
    
    send_can_command(f"000#0A{group_id:02X}")
    last_time = time.time()
    stop_watch = last_time
    time_out = travel_time
    
def sp_coor(tar_coor, travel_time):
    cur_joints = read_present_position()
    tar_joints = inverse_kinematics(tar_coor)
    tar_joints = check_limit(tar_joints)
    print(f"tar joint = {tar_joints} degree")
    sp_angle(tar_joints, travel_time)
    
    
    
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
    
# sdsjd