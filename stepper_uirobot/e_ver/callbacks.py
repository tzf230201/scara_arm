# callbacks.py
from time import time
from arm import *
from utility import *
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
    state['routine'] = False

def pp_joint(msg, state):
    # Ambil array lengkap [joint1, joint2, joint3, joint4]
    selection = msg.get('motor')
    joints = msg.get("joints", [0, 0, 0, 0])
    t_ms = msg.get("time", 1000)
    robot_pp_angle(joints, t_ms, selection)


def pp_coor(msg, state):
    selection = msg.get('motor')
    coor = msg.get("coor", [258, 0, 0, 0])
    t_ms = msg.get("time", 1000)
    robot_pp_coor(coor, t_ms, selection)

def pvt_joint(msg, state):
    selection = msg.get('motor')
    joints = msg.get("joints", [])
    t_ms = msg.get("time", 1000)
    # robot_pt_angle(joints, t_ms, "stepper_only")
    robot_pvt_angle(joints, t_ms, selection)
    # TODO: Implement PVT handling

def pvt_coor(msg, state):
    selection = msg.get('motor')
    coor = msg.get("coor", [])
    t_ms = msg.get("time", 1000)
    robot_pvt_coor(coor, t_ms, selection)
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
        yaw_s = f"{yaw:.2f}°"

    print_yellow(f"[cb] m1={m1_s} m2={m2_s} m3={m3_s} m4={m4_s}")
    print_orange(f"[cb] x={x_s}, y={y_s}, z={z_s}, yaw={yaw_s}")

def dancing(msg, state):
    # arm_pvt_init()
    selection = msg.get('motor')
    selection = "all"
    # pre_start_dancing(selection)
    # robot_start_dancing()
    # pvt_test()
    robot_start_pvt_dancing()
    state['routine'] = True
    # pt_test()
    # print(f"[cb] Dancing: motor={msg.get('motor')}")
    # # TODO: Implement demo/dance pattern
    
def routine(state):
    if state['routine'] == True:
        # ret = pt_routine()
        ret = pvt_routine()
        if ret == 1:
            state['routine'] = False

def homing(msg, state):
    joints = msg.get("joints", home_angle)
    t_ms = msg.get("time", 4000)
    selection = msg.get('motor')
    robot_pp_angle(joints, t_ms, selection)
    print(f"[cb] Homing: motor={msg.get('motor')}")
    # # TODO: Implement homing

def stop(msg, state):
    arm_set_motor_off()
    print(f"[cb] STOP (from GUI)")
    state['routine'] = False
    state['running'] = False

def read_encoder(msg, state):
    selection = msg.get('motor')
    enc1, enc2, enc3, enc4 = robot_get_enc(selection)
    print(f"[cb] enc: {enc1}, {enc2}, {enc3}, {enc4}")
    # TODO: Implement reading encoder value

def set_origin(msg, state):
    selection = msg.get('motor')
    robot_set_origin(selection)
    print(f"[cb] Set Origin: motor={msg.get('motor')}")

def out2_active(msg, state):
    print(f"out2 active")
    servo_get_digital_output_state()

def out2_nonactive(msg, state):
    print(f"out2 non-active")
    servo_get_output_pin_configuration()
    

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
    "out2_active": out2_active,
    "out2_nonactive": out2_nonactive,
}
