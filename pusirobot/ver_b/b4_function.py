from b1_stepper import *
from b1_servo import *
from b3_motion import *

is_wake_up = False
def wake_up():
    global is_wake_up
    start_can()
    stepper_init()
    is_wake_up = True
    servo_init()

def is_already_wake_up():
    global is_wake_up
    return is_wake_up

def shutdown():
    stepper_shutdown()
    # motor_1_shutdown()
    # stop_can()

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
    
def read_present_position():
    cur_joints = get_cur_joints()
    cur_coor = forward_kinematics(cur_joints)
    
    formatted_angles = "°, ".join([f"{angle:.2f}" for angle in cur_joints])
    print_yellow(f"cur joint : {formatted_angles}°")#yellow
    
    cur_x, cur_y, cur_z, cur_yaw = cur_coor
    print_orange(f"cur coor : x:{cur_x:.1f} mm, y:{cur_y:.1f} mm, z:{cur_z:.1f} mm, yaw:{cur_yaw:.1f}°")#orange
    
    # servo_vel = servo_get_motor_velocity(ID1)
    # servo_status = servo_get_status_word(ID1)
    
    # print(f"servo status (hex): {servo_status:08X}, servo velocity: {servo_vel}")

    return cur_joints

def get_encoder_position():
    enc1 = servo_get_motor_position(ID1)
    enc2 = stepper_get_encoder_position(ID2)
    enc3 = stepper_get_encoder_position(ID3)
    enc4 = stepper_get_encoder_position(ID4)
    print(f"enc: {enc1} {enc2}, {enc3}, {enc4}")
    return enc1, enc2, enc3, enc4

def set_origin():
    enc1, enc2, enc3, enc4 = get_encoder_position()
    for node_id, enc in zip([ID2, ID3, ID4], [enc2, enc3, enc4]):
        stepper_calibration_zero(node_id, -enc)
    save_settings()
    
