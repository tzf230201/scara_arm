from motion import *
from servo import *
from stepper import *
import time

def robot_init(selection):
    print(f"robot_init({selection})")
def robot_wake_up(selection):
    print(f"robot_wake_up({selection})")
def robot_shutdown(selection):
    print(f"robot_shutdown({selection})")
def robot_dancing(selection):
    print(f"robot_dancing({selection})")
def robot_get_enc(selection):
    print(f"robot_get_enc({selection})")
def robot_get_angle(selection):
    print(f"robot_get_angle({selection})")
def robot_set_origin(selection):
    print(f"robot_set_origin({selection})")


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