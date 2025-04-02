from b1_stepper import *
from b1_servo import *
from b3_motion import *

def wake_up():
    start_can()
    stepper_init()
    # servo_init()git
    
def shutdown():
    stepper_shutdown()
    # motor_1_shutdown()
    stop_can()

def read_present_position():
    cur_joints = get_cur_joints()
    cur_coor = forward_kinematics(cur_joints)
    
    cur_x, cur_y, cur_z, cur_yaw = cur_coor
    print(f"cur coor : x:{cur_x:.1f} mm, y:{cur_y:.1f} mm, z:{cur_z:.1f} mm, yaw:{cur_yaw:.1f} degree")
    
    # is_sp_mode_arrive = read_sp_mode_arrival_status()
    # delta_time = time.time() - last_time
    formatted_angles = ", ".join([f"{angle:.2f}" for angle in cur_joints])
    print(f"cur joint : {formatted_angles} degree")
    # print(f"time : {delta_time:.2f}, is sp mode arrive : {is_sp_mode_arrive}")
    return cur_joints

def get_encoder_position():
    enc1 = servo_get_motor_position(ID1)
    enc2 = stepper_get_encoder_position(ID2)
    enc3 = stepper_get_encoder_position(ID3)
    enc4 = stepper_get_encoder_position(ID4)
    print(f"enc: {enc1} {enc2}, {enc3}, {enc4}")
    return enc1, enc2, enc3, enc4

def set_origin():
    enc1, enc2, enc3, enc4 = get_encoder_position()
    for node_id, enc in zip([ID2, ID3, ID4], [enc2, enc3, enc4]):
        stepper_calibration_zero(node_id, -enc)
    save_settings()
    
