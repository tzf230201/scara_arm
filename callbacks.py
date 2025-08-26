# callbacks.py
from time import time
from stepper import *

MICROSTEP = 16
STEPS_PER_REV = 200 * MICROSTEP # misal 3200 pulse/rev
DEG_PER_PULSE = 360.0 / STEPS_PER_REV

def stepper_pulse_to_deg(pulse):
    return pulse * DEG_PER_PULSE

def wake_up(msg, state):
    print(f"[cb] Wake up: motor={msg.get('motor')}")  # Wake up/enable/prepare motors
    stepper_set_mo(6,1)
    stepper_set_mo(7,1)
    stepper_set_mo(8,1)
    state['motor_on'] = True

def shutdown(msg, state):
    print(f"[cb] Shutdown: motor={msg.get('motor')}")
    stepper_set_mo(6,0)
    stepper_set_mo(7,0)
    stepper_set_mo(8,0)
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
    pa2 = stepper_get_pa(6)
    pa3 = stepper_get_pa(7)
    pa4 = stepper_get_pa(8)

    print(f"[cb] Read Position: pa2={pa2} pa3={pa3} pa4={pa4}")

    # TODO: Implement read pos & maybe update state["pos_abs"]

def dancing(msg, state):
    print(f"[cb] Dancing: motor={msg.get('motor')}")
    # TODO: Implement demo/dance pattern

def homing(msg, state):
    print(f"[cb] Homing: motor={msg.get('motor')}")
    # TODO: Implement homing

def stop(msg, state):
    print(f"[cb] STOP (from GUI)")
    stepper_stop_motion(6)
    stepper_stop_motion(7)
    stepper_stop_motion(8)
    # state['running'] = False

def read_encoder(msg, state):
    pa2 = stepper_get_pa(6)
    pa3 = stepper_get_pa(7)
    pa4 = stepper_get_pa(8)

    pa2_deg = stepper_pulse_to_deg(pa2)
    pa3_deg = stepper_pulse_to_deg(pa3)
    pa4_deg = stepper_pulse_to_deg(pa4)
    print(f"[cb] pa = {pa2_deg}, {pa3_deg}, {pa4_deg} degree")
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
