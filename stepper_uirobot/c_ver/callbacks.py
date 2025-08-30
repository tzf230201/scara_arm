# callbacks.py
from time import time
from arm import *
from utility import *
from motion import *
from servo import *
from robot import *

def wake_up(msg, state):

    selection = msg.get('motor')
    robot_wake_up(selection)
    state['motor_on'] = True
    print(f"[cb] Wake up: motor={selection}")  # Wake up/enable/prepare motors

def shutdown(msg, state):
    selection = msg.get('motor')
    robot_shutdown(selection)
    state['motor_on'] = False

def pp_joint(msg, state):
    # Ambil array lengkap [joint1, joint2, joint3, joint4]
    joints = msg.get("joints", [0, 0, 0, 0])
    t_ms = msg.get("time", 1000)
    arm_pp_angle(joints, t_ms)


def pp_coor(msg, state):
    coor = msg.get("coor", [258, 0, 0, 0])
    t_ms = msg.get("time", 1000)
    arm_pp_coor(coor, t_ms)

def pvt_joint(msg, state):
    joints = msg.get("joints", [])
    t_ms = msg.get("time", 1000)
    arm_pt_angle(joints, t_ms)
    # TODO: Implement PVT handling

def pvt_coor(msg, state):
    coor = msg.get("coor", [])
    t_ms = msg.get("time", 1000)
    arm_pt_coor(coor, t_ms)
    # TODO: Implement inverse kinematics/PVT

def read_position(msg, state):
    cur_angle_1 = cur_angle_2 = cur_angle_3 = cur_angle_4 = None
    m1_s = m2_s = m3_s = m4_s = "None"
    x_s = y_s = z_s = yaw_s = "None"
    
    selection = msg.get('motor')
    cur_angle_1, cur_angle_2, cur_angle_3, cur_angle_4 = robot_get_angle(selection)

    if cur_angle_1 != None:
        z = servo_forward_kinematics(cur_angle_1)
        m1_s = f"{cur_angle_1:.2f}°"
        z_s = f"{z:.2f}mm"
        
    if None not in (cur_angle_2, cur_angle_3, cur_angle_4):
        x, y, yaw = arm_forward_kinematics(cur_angle_2, cur_angle_3, cur_angle_4)
        m2_s = f"{cur_angle_2:.2f}°"
        m3_s = f"{cur_angle_3:.2f}°"
        m4_s = f"{cur_angle_4:.2f}°"
        x_s   = f"{x:.2f}mm"
        y_s   = f"{y:.2f}mm"
        yaw_s = f"{z:.2f}°"

    print_yellow(f"[cb] m1={m1_s} m2={m2_s} m3={m3_s} m4={m4_s}")
    print_orange(f"[cb] x={x_s}, y={y_s}, z={z_s}, yaw={yaw_s}")

def dancing(msg, state):
    # arm_pvt_init()
    # pre_start_dancing()
    pt_test()
    # print(f"[cb] Dancing: motor={msg.get('motor')}")
    # # TODO: Implement demo/dance pattern

def homing(msg, state):
    # joints = msg.get("joints", [0, 0, 0, 0])
    joints = [0, 0, 0, 0]
    t_ms = msg.get("time", 4000)
    arm_pp_angle(joints, t_ms)
    # print(f"[cb] Homing: motor={msg.get('motor')}")
    # # TODO: Implement homing

def stop(msg, state):
    arm_set_motor_off()
    print(f"[cb] STOP (from GUI)")
    # state['running'] = False

def read_encoder(msg, state):
    selection = msg.get('motor')
    enc1, enc2, enc3, enc4 = robot_get_enc(selection)
    pa2, pa3, pa4 = arm_get_enc()
    print(f"[cb] enc: {enc1}, {enc2}, {enc3}, {enc4}")
    # TODO: Implement reading encoder value

def set_origin(msg, state):
    selection = msg.get('motor')
    robot_set_origin(selection)
    print(f"[cb] Set Origin: motor={msg.get('motor')}")

# === Mapping ===
HANDLERS = {
    "wake_up": wake_up,
    "shutdown": shutdown,
    "pp_joint": pp_joint,
    "pp_coor": pp_coor,
    "pvt_joint": pvt_joint,
    "pvt_coor": pvt_coor,
    "read_position": read_position,
    "dancing": dancing,
    "homing": homing,
    "stop": stop,
    "read_encoder": read_encoder,
    "set_origin": set_origin,
}
