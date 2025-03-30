from b1_stepper import *
from b1_servo import *
from b3_motion import *

def wake_up():
    start_can()
    stepper_init()
    servo_init()
    
    
def shutdown():
    stepper_shutdown()
    # motor_1_shutdown()
    stop_can()

def read_present_position():
    servo_id = ID1
    servo_pulse = servo_get_motor_position(servo_id)
    servo_angle = servo_pulses_to_degrees(servo_pulse)
    
    stepper_ids = [ID2, ID3, ID4]
    stepper_angles = []
    
    for stepper_id in stepper_ids:
        stepper_pulse = stepper_get_motor_position(stepper_id)
        # print(f"stepper {stepper_id:03X} pulse is {stepper_pulse}")
        stepper_angles.append(stepper_pulses_to_degrees(stepper_pulse))
    
    cur_joints = [servo_angle, stepper_angles[0], stepper_angles[1], stepper_angles[2]]
    
    cur_coor = forward_kinematics(cur_joints)
    
    cur_x, cur_y, cur_z, cur_yaw = cur_coor
    
    print(f"cur coor : x:{cur_x:.1f} mm, y:{cur_y:.1f} mm, z:{cur_z:.1f} mm, yaw:{cur_yaw:.1f} degree")
    print(f"cur joint : m2:{m2_angle:.1f}, m3:{m3_angle:.1f}, m4:{m4_angle:.1f} degree")
    is_sp_mode_arrive = read_sp_mode_arrival_status()
    delta_time = time.time() - last_time
    formatted_angles = ", ".join([f"{angle:.2f}" for angle in cur_joints])
    print(f"cur joint : {formatted_angles} degree")
    print(f"time : {delta_time:.2f}, is sp mode arrive : {is_sp_mode_arrive}")
    return cur_joints