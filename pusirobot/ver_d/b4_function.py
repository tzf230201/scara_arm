from b1_stepper import *
from b1_servo import *
from b3_motion import *

is_wake_up = False
motor_selection = "stepper_only" #"all", "stepper_only", "servo_only"
def set_motor_selection(selection):
    global motor_selection
    if selection in ["all", "stepper_only", "servo_only"]:
        motor_selection = selection
    else:
        raise ValueError("Invalid motor selection. Choose 'all', 'stepper', or 'servo'.")

def get_motor_selection():
    global motor_selection
    return motor_selection


def wake_up():
    global is_wake_up
    start_can()
    selection = get_motor_selection()
    cur_joints = get_cur_joints(selection)
    if selection != "servo_only":
        print(f"stepper intialization")
        stepper_init()
        pp_mode_init()
        stepper_disable_heartbeat()
    if selection != "stepper_only":
        print(f"servo intialization")
        servo_init(1)  # 7 is PVT mode, 1 is PP mode
        servo_disable_heartbeat()
        if is_brake_on():
            ret = pp_angle_servo(cur_joints, 100, "servo_only")
            if ret == 1:
                servo_execute()  # Execute the servo command to start the movement
            
            time.sleep(0.2)
            servo_brake_off()
            print(f"turned off brake")
        else:
            print(f"brake already off")
            
    is_wake_up = True
    

def is_already_wake_up():
    global is_wake_up
    return is_wake_up

def shutdown():
    selection = get_motor_selection()
    
    if selection != "servo_only":
        stepper_shutdown()
        print(f"stepper shutdown")
    if selection != "stepper_only":
        servo_shutdown()
        print(f"servo shutdown, brake active")
    # stop_can()
    

def get_encoder_position():
    """
    Get encoder position for servo (ID1) and steppers (ID2, ID3, ID4) based on motor selection.
    """
    selection = get_motor_selection()
    
    # Initialize encoders with default 0
    enc1 = enc2 = enc3 = enc4 = 0
    
    # if selection in ["all", "servo_only"]:
    enc1 = servo_get_motor_position(ID1)
    
    # if selection in ["all", "stepper_only"]:
    enc2 = stepper_get_encoder_position(ID2)
    enc3 = stepper_get_encoder_position(ID3)
    enc4 = stepper_get_encoder_position(ID4)
    
    print(f"Encoder readings: {enc1}, {enc2}, {enc3}, {enc4}")
    
    return enc1, enc2, enc3, enc4


def set_origin():
    """
    Set origin for stepper motors and save encoder values to config.
    Raises ValueError if motor selection does not include stepper.
    """
    selection = get_motor_selection()
    encoders = get_encoder_position()
    
    # Save encoder readings to config file
    set_origins(encoders)

    for node_id in [ID2, ID3, ID4]:
        stepper_calibration_zero(node_id)
        print(f"Node {node_id} set to zero")
    
    save_settings()


# Example usage in button:
# set_origin_button = tk.Button(root, text="set origin", command=set_origin)

    
    