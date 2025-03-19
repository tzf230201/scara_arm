import time
import math
from micro_can import save_settings, set_req_sdo

    
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
    
    if joint_1 > (13004):
        joint_1 = 13004
       # raise ValueError("out of joint_1 max limit")
    elif joint_1 < 0:
        joint_1 = 0
    if joint_2 > (178 * 5):
        joint_2 = 178 * 5
       # raise ValueError("out of joint_2 max limit")
    elif joint_2 < 0:
        joint_2 = 0
        #raise ValueError("out of joint_2 min limit")
    if joint_3 > (0 + joint_2):
        joint_3 = (0 + joint_2)
        #raise ValueError("out of joint_3 max limit")
    elif joint_3 < ((-135 * 5) + joint_2):
        joint_3 = (-135 * 5) + joint_2
        #raise ValueError("out of joint_2 min limit")
    if joint_4 > ((196 * 5) + joint_3):
        joint_4 = (196 * 5) + joint_3
        #raise ValueError("out of joint_4 max limit")
    elif joint_4 < (0 + joint_3):
        joint_4 = (0 + joint_3)
        #raise ValueError("out of joint_2 min limit")
        
    joint_4 *= -1
        
    return [joint_1, joint_2, joint_3, joint_4]

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
    
#21

def encoder_position():
    enc2 = req_sdo(ID2, OD_STEPPER_ENCODER_POSITION, 0x00)
    enc3 = req_sdo(ID3, OD_STEPPER_ENCODER_POSITION, 0x00)
    enc4 = req_sdo(ID4, OD_STEPPER_ENCODER_POSITION, 0x00)
    print(f"enc: {enc2}, {enc3}, {enc4}")
    
    return enc2, enc3, enc4

def calib_0():
    enc2, enc3, enc4 = encoder_position()
    for id, enc in zip([ID2, ID3, ID4], [enc2, enc3, enc4]):
        error_code = set_sdo(id, SET_4_BYTE, OD_STEPPER_CALIBRATION_ZERO, 0x00, -enc)
    
    save_settings()

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

        
def dancing():
    try:
        travel_time = int(entry_time.get())
    except ValueError:
        print("Please enter valid numbers for angles.")
    #while True:
    travel_time = travel_time/1000
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