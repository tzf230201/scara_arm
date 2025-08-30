from motion import *
from servo import *
from stepper import *
from arm import *
import time
import os
import json
import shutil

def is_stepper_selected(selection):
    return selection != "servo_only"

def is_servo_selected(selection):
    return selection != "stepper_only"

def load_origin_from_config():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "config_origin.json")
    print(f"{config_path}")
    
    default_config = {
        "origin_1": -3306079,
        "origin_2": 0,
        "origin_3": 0,
        "origin_4": 0
    } 

    if not os.path.exists(config_path):
        with open(config_path, "w") as f:
            json.dump(default_config, f, indent=4)
        print("config_origin.json not found. Created new file with defaults.")
        return tuple(default_config.values())

    try:
        with open(config_path, "r") as f:
            config_data = json.load(f)

        origin_1 = config_data.get("origin_1", 0)
        origin_2 = config_data.get("origin_2", 0)
        origin_3 = config_data.get("origin_3", 0)
        origin_4 = config_data.get("origin_4", 0)

        print(f"Loaded origins: {origin_1}, {origin_2}, {origin_3}, {origin_4}")
        return origin_1, origin_2, origin_3, origin_4

    except json.JSONDecodeError:
        # Backup the invalid file
        backup_path = config_path + ".backup_invalid"
        shutil.copy(config_path, backup_path)
        print(f"Invalid JSON format. Backup saved to {backup_path}. Rewriting with defaults.")
        
        with open(config_path, "w") as f:
            json.dump(default_config, f, indent=4)
        return tuple(default_config.values())


def save_origin_to_config(encoders):
    config_data = {
        "origin_1": encoders[0],
        "origin_2": encoders[1],
        "origin_3": encoders[2],
        "origin_4": encoders[3],
    }
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config_origin.json")
    with open(config_path, "w") as f:
        json.dump(config_data, f, indent=4)

    print(f"Origin saved to {config_path}")
    
origins = load_origin_from_config()

def get_origins():
    global origins
    
    return origins



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
        enc_1 = servo_get_motor_position(ID1)

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
        enc_1 = servo_get_motor_position(ID1)
        cur_angle_1 = servo_pulses_to_degrees(enc_1)

    # Baca stepper jika ter-select
    if is_stepper_selected(selection):
        cur_angle_2, cur_angle_3, cur_angle_4 = arm_get_angle()

    return cur_angle_1, cur_angle_2, cur_angle_3, cur_angle_4



def robot_set_origin(selection):
    if selection == "all":
        enc_1 = servo_get_motor_position(ID1)
        enc_2, enc_3, enc_4 = arm_get_enc()
        save_origin_to_config([enc_1, enc_2, enc_3, enc_4])
        arm_set_origin()
        print(f"robot_set_origin({selection})")
    else:
        print(f"robot_set_origin({selection}) skipped, only available for 'all' selection")
    
def robot_dancing(selection):
    print(f"robot_dancing({selection})")
    
    

    
    
    




def robot_pp_angle(tar_joints, travel_time, selection):
    print(f"entering robot pp angle")
    
def robot_pp_coor(tar_coor,travel_time,selection):
    # tar_joints = inverse_kinematics(tar_coor)
    # robot_pp_angle(tar_joints, travel_time, selection)
    print(f"entering robot pp coor")

def robot_pt_angle(tar_joints, travel_time, selection):
    print(f"entering robot pt angle")

def robot_pt_coor(tar_coor, travel_time, selection):
    print(f"entering robot pt coor")