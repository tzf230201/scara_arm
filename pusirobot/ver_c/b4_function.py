from b1_stepper import *
from b1_servo import *
from b3_motion import *

is_wake_up = False
motor_selection = "stepper_only" #"all", "stepper_only", "servo_only"
def set_motor_selection(selection):
    global motor_selection
    if selection in ["all", "stepper_only", "servo_only"]:
        motor_selection = selection
    else:
        raise ValueError("Invalid motor selection. Choose 'all', 'stepper', or 'servo'.")

def get_motor_selection():
    global motor_selection
    return motor_selection


def wake_up():
    global is_wake_up
    start_can()
    selection = get_motor_selection()
    if selection != "servo_only":
        stepper_init()
    if selection != "stepper_only":
        servo_init(1)  # 7 is PVT mode, 1 is PP mode
    is_wake_up = True
    

def is_already_wake_up():
    global is_wake_up
    return is_wake_up

def shutdown():
    selection = get_motor_selection()
    
    if selection != "servo_only":
        stepper_shutdown()
        print(f"stepper shutdown")
    if selection != "stepper_only":
        servo_shutdown()
        print(f"servo shutdown")
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
    selection = get_motor_selection()
    cur_joints = get_cur_joints(selection)
    cur_coor = forward_kinematics(cur_joints)
    
    formatted_angles = "°, ".join([f"{angle:.2f}" for angle in cur_joints]) #6 may 2025
    print_yellow(f"cur joint : {formatted_angles}°")#yellow #6 may 2025
    # j1, j2, j3, j4 = cur_joints #6 may 2025
    # print_yellow(f"cur servo's angle: {j1:.1f}°")#yellow #6 may 2025
    
    cur_x, cur_y, cur_z, cur_yaw = cur_coor
    print_orange(f"cur coor : x:{cur_x:.1f} mm, y:{cur_y:.1f} mm, z:{cur_z:.1f} mm, yaw:{cur_yaw:.1f}°")#orange #6 may 2025
    # print_orange(f"cur coor z:{cur_z:.1f} mm")#orange #6 may 2025
    
    # servo_vel = servo_get_motor_velocity(ID1)
    # servo_status = servo_get_status_word(ID1)
    
    # print(f"servo status (hex): {servo_status:08X}, servo velocity: {servo_vel}")
# import json

# def save_origin_to_config(encoder_value):
#     config_data = {
#         "origin_encoder": encoder_value
#     }
#     with open("config_origin.json", "w") as f:
#         json.dump(config_data, f, indent=4)
#     print("Origin saved to config_origin.json")

# def get_encoder_position():
#     selection = get_motor_selection()
#     if selection != "stepper_only":
#         enc1 = servo_get_motor_position(ID1)
#     else: 
#         enc1 = 0
        
#     if selection != "servo_only":
#         enc2 = stepper_get_encoder_position(ID2)
#         enc3 = stepper_get_encoder_position(ID3)
#         enc4 = stepper_get_encoder_position(ID4)
#     else:
#         enc2 = enc3 = enc4 = 0
    
#     print(f"enc: {enc1} {enc2}, {enc3}, {enc4}")
#     return enc1, enc2, enc3, enc4

# def set_origin():
#     selection = get_motor_selection()
#     encoders = get_encoder_position()
#     save_origin_to_config(encoders)
#     if selection != "servo_only":
#         for node_id in [ID2, ID3, ID4]:
#             stepper_calibration_zero(node_id)
#             print(f"node {node_id} set to zero")
#         save_settings()
#     else:
#         raise ValueError("set origin only for stepper, choose 'all' or 'only stepper'")
import json
import os


def save_origin_to_config(encoders):
    """
    Save encoder values to config_origin.json as a dictionary with clear keys.
    """
    config_data = {
        "origin_encoder_1": encoders[0],
        "origin_encoder_2": encoders[1],
        "origin_encoder_3": encoders[2],
        "origin_encoder_4": encoders[3],
    }
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config_origin.json")
    with open(config_path, "w") as f:
        json.dump(config_data, f, indent=4)

    print(f"Origin saved to {config_path}")


def get_encoder_position():
    """
    Get encoder position for servo (ID1) and steppers (ID2, ID3, ID4) based on motor selection.
    """
    selection = get_motor_selection()
    
    # Initialize encoders with default 0
    enc1 = enc2 = enc3 = enc4 = 0
    
    if selection in ["all", "servo_only"]:
        enc1 = servo_get_motor_position(ID1)
    
    if selection in ["all", "stepper_only"]:
        enc2 = stepper_get_encoder_position(ID2)
        enc3 = stepper_get_encoder_position(ID3)
        enc4 = stepper_get_encoder_position(ID4)
    
    print(f"Encoder readings: {enc1}, {enc2}, {enc3}, {enc4}")
    
    return enc1, enc2, enc3, enc4


def set_origin():
    """
    Set origin for stepper motors and save encoder values to config.
    Raises ValueError if motor selection does not include stepper.
    """
    selection = get_motor_selection()
    if selection == "all":
        encoders = get_encoder_position()
        
        # Save encoder readings to config file
        save_origin_to_config(encoders)
        
        for node_id in [ID2, ID3, ID4]:
            stepper_calibration_zero(node_id)
            print(f"Node {node_id} set to zero")
        
        save_settings()
       
    else:
        raise ValueError("Set origin only valid for stepper motors. Choose 'all'")


# Example usage in button:
# set_origin_button = tk.Button(root, text="set origin", command=set_origin)

    
    