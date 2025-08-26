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
    joints = msg.get("joints", [])
    t_ms = msg.get("time", 0)
    print(f"[cb] PP Joint: joints={joints}, time={t_ms} ms, motor={msg.get('motor')}")
    # TODO: Implement trajectory/send to HW

def pp_coor(msg, state):
    coor = msg.get("coor", [])
    t_ms = msg.get("time", 0)
    print(f"[cb] PP Coor: coor={coor}, time={t_ms} ms, motor={msg.get('motor')}")
    # TODO: Implement inverse kinematics/send

def pvt_joint(msg, state):
    joints = msg.get("joints", [])
    t_ms = msg.get("time", 0)
    print(f"[cb] PVT Joint: joints={joints}, time={t_ms} ms, motor={msg.get('motor')}")
    # TODO: Implement PVT handling

def pvt_coor(msg, state):
    coor = msg.get("coor", [])
    t_ms = msg.get("time", 0)
    print(f"[cb] PVT Coor: coor={coor}, time={t_ms} ms, motor={msg.get('motor')}")
    # TODO: Implement inverse kinematics/PVT

def read_position(msg, state):
    # print(f"[cb] Read Position: motor={msg.get('motor')}")
    pa2_deg, pa3_deg, pa4_deg = stepper_get_all_angle()
    print(f"[cb] Read Position: pa2={pa2_deg}, pa3={pa3_deg}, pa4={pa4_deg} degree")

    # TODO: Implement read pos & maybe update state["pos_abs"]

def dancing(msg, state):
    print(f"[cb] Dancing: motor={msg.get('motor')}")
    # TODO: Implement demo/dance pattern

def homing(msg, state):
    print(f"[cb] Homing: motor={msg.get('motor')}")
    # TODO: Implement homing

def stop(msg, state):
    stepper_stop_motion(6)
    stepper_stop_motion(7)
    stepper_stop_motion(8)
    print(f"[cb] STOP (from GUI)")
    # state['running'] = False

def read_encoder(msg, state):
    pa2, pa3, pa4 = stepper_get_all_enc()
    print(f"[cb] Read Position: pa2={pa2} pa3={pa3} pa4={pa4}")
    # TODO: Implement reading encoder value

def set_origin(msg, state):
    print(f"[cb] Set Origin: motor={msg.get('motor')}")
    # TODO: Implement set origin

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
