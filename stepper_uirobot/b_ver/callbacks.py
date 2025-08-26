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
    joints = msg.get("joints", [0, 0, 0])
    t_ms = msg.get("time", 1000)
    print(f"[cb] PP Joint: joints={joints}, time={t_ms} ms, motor={msg.get('motor')}")
    node_ids = [6, 7, 8]  # ganti jika urutan node kamu berbeda

    # 1. Konversi degree ke pulses
    pulses = [stepper_deg_to_pulse(j) for j in joints]

    # 2. Hitung max selisih (untuk cari kecepatan max yang aman)
    pa_now = [stepper_get_pa(node_id) for node_id in node_ids]
    delta = [abs(p - p_now) for p, p_now in zip(pulses, pa_now)]
    max_diff = max(delta)
    speed = int(max_diff / max(t_ms / 1000.0, 1e-3))   # pulses/sec, minimal 1
    speed = max(speed, 1)

    # 3. Set speed + posisi ke setiap motor
    for node_id, pos in zip(node_ids, pulses):
        stepper_set_sp(node_id, speed)
        stepper_set_pa(node_id, pos)
    # 4. Mulai gerak (BG) serempak
    for node_id in node_ids:
        stepper_begin_motion(node_id)

    print(f"PP Joint executed: pulses={pulses}, speed={speed}, time={t_ms}ms")


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
    stepper_set_all_motor_off()
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
