# callbacks.py
from time import time
from stepper import *
from function import *

def wake_up(msg, state):
    print(f"[cb] Wake up: motor={msg.get('motor')}")  # Wake up/enable/prepare motors
    init_stepper()
    state['motor_on'] = True

def shutdown(msg, state):
    print(f"[cb] Shutdown: motor={msg.get('motor')}")
    stepper_set_all_motor_off()
    state['motor_on'] = False

def pp_joint(msg, state):
    group_id = STEPPER_GROUP_ID
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
    t_ms = msg.get("time", 0)
    arm_pvt_angle(joints, t_ms)
    # TODO: Implement PVT handling

def pvt_coor(msg, state):
    coor = msg.get("coor", [])
    t_ms = msg.get("time", 0)
    arm_pvt_coor(coor, t_ms)
    # TODO: Implement inverse kinematics/PVT

def read_position(msg, state):
    # print(f"[cb] Read Position: motor={msg.get('motor')}")
    pa2_deg, pa3_deg, pa4_deg = stepper_get_all_angle()
    x,y,z,yaw = forward_kinematics([0,pa2_deg, pa3_deg, pa4_deg])
    print_yellow(f"[cb] pa2={pa2_deg:.2f}°, pa3={pa3_deg:.2f}°, pa4={pa4_deg:.2f}°")
    print_orange(f"[cb] x={x:.2f}mm, y={y:.2f}mm, z={z:.2f}mm, yaw={yaw:.2f}°")

    # TODO: Implement read pos & maybe update state["pos_abs"]

def dancing(msg, state):
    # arm_pvt_init()
    pre_start_dancing()
    # print(f"[cb] Dancing: motor={msg.get('motor')}")
    # # TODO: Implement demo/dance pattern

def homing(msg, state):
    home_coor = [0, 0, 0, 0]
    t_ms = msg.get("time", 0)
    arm_pvt_coor(home_coor, t_ms)
    # print(f"[cb] Homing: motor={msg.get('motor')}")
    # # TODO: Implement homing

def stop(msg, state):
    stepper_set_all_motor_off()
    print(f"[cb] STOP (from GUI)")
    # state['running'] = False

def read_encoder(msg, state):
    pa2, pa3, pa4 = stepper_get_all_enc()
    print(f"[cb] enc: pa2={pa2} pa3={pa3} pa4={pa4}")
    # TODO: Implement reading encoder value

def set_origin(msg, state):
    print(f"[cb] Set Origin: motor={msg.get('motor')}")
    stepper_set_origin(6)
    stepper_set_origin(7)
    stepper_set_origin(8)

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
