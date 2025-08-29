from motion import *
from servo import *
from stepper import *
import time

def robot_init():
    print(f"robot_init()")
def robot_wake_up():
    print(f"robot_wake_up()")
def robot_shutdown():
    print(f"robot_shutdown()")
def robot_dancing():
    print(f"robot_dancing()")
def robot_get_enc():
    print(f"robot_get_enc()")
def robot_get_angle():
    print(f"robot_get_angle()")
def robot_set_origin():
    print(f"robot_set_origin()")


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