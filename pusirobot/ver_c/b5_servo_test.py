import signal
import tkinter as tk
import time
from b4_function import wake_up, shutdown, read_present_position, get_encoder_position, set_origin, is_already_wake_up,set_motor_selection, get_motor_selection
from b3_motion import dancing, sp_angle, sp_coor, pvt_circular, pvt_mode_try_pvt_3, pp_angle, pp_coor, inverse_kinematics, check_limit
from b1_servo import servo_get_motor_velocity, servo_get_status_word
from b0_can import send_can_command
import sys
#ege
# class Tee:
#     def __init__(self, file_name):
#         self.console = sys.stdout  # Standard output (console)
#         self.file = open(file_name, 'a')  # File to log the output

#     def write(self, message):
#         self.console.write(message)  # Print to console
#         self.file.write(message)     # Write to file

#     def flush(self):
#         self.console.flush()  # Flush the console
#         self.file.flush()     # Flush the file

# # Redirect sys.stdout to Tee class
# sys.stdout = Tee("output_terminal.txt")  # Specify your file name here

last_time = time.time()

def signal_handler():
    # if is_already_wake_up():
        #  shutdown()
        #  homing()
    print("SIGINT received, closing application...")
   
    root.quit()  # Hentikan event loop
    root.destroy()  # Hancurkan GUI
    exit(0)  # Keluar sepenuhnya dari program
    
def get_tar_joints():
    try:  
        tar_joint_1 = 0#float(entry_tar_joint_1.get()) #6 may 2025
        tar_joint_2 = 0#float(entry_tar_joint_2.get()) #6 may 2025
        tar_joint_3 = 0#float(entry_tar_joint_3.get()) #6 may 2025
        tar_joint_4 = 0#float(entry_tar_joint_4.get()) #6 may 2025
    except ValueError:
        print("Please enter valid numbers for angles.")
        
    tar_joints = [tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4]

    return tar_joints

def get_nor():
    nor = 1
    try:  
        nor = int(entry_nor.get())
    except ValueError:
        print("Please enter valid numbers for angles.")
        
    if nor < 1:
        nor = 1

    return nor


# def get_tar_coor(): #6 may 2025
#     try:
#         tar_x = float(entry_tar_x.get())
#         tar_y = float(entry_tar_y.get())
#         tar_z = float(entry_tar_z.get())
#         tar_yaw = float(entry_tar_yaw.get())
#     except ValueError:
#         print("Please enter valid numbers for angles.")
    
#     tar_coor = [tar_x, tar_y, tar_z, tar_yaw]

#     return tar_coor
def get_tar_coor(): #6 may 2025
    try:
        tar_x = 130
        tar_y = 0
        tar_z = float(entry_tar_z.get())
        tar_yaw = 0
    except ValueError:
        print("Please enter valid numbers for angles.")
    
    tar_coor = [tar_x, tar_y, tar_z, tar_yaw]

    return tar_coor

def get_travel_time():
    try:
        travel_time = int(entry_time.get())
    except ValueError:
        print("Please enter valid numbers for angles.")

    return travel_time
    
    

    
def pp_joint():
    global last_time
    selection = get_motor_selection()
    tar_joints = get_tar_joints()
    travel_time = get_travel_time()
    
    pp_angle(tar_joints, travel_time, selection)
    last_time = time.time()
    
def pp_move():  
    global last_time
    selection = get_motor_selection()
    tar_coor = get_tar_coor()
    travel_time = get_travel_time()
       
    pp_coor(tar_coor, travel_time, selection)
    last_time = time.time()
    

enable_motion = True
is_up = True
dancing_tar_joints = [40, 0, 0, 0]  # Default target angle for dancing
dancing_travel_time = 4000
how_many_times = 1  # Default number of times to run the dance routine
dancing_i = 0



def start_dancing():
    global last_time
    global enable_motion
    global dancing_tar_joints
    global is_up
    global dancing_travel_time
    global how_many_times
    global dancing_i
    
    travel_time = get_travel_time()
    dancing_travel_time = (int)(travel_time)
    
    
    how_many_times = get_nor()
    dancing_i = 0  # Reset the dance iteration counter
    
    # dancing(travel_time)
    tar_coor = get_tar_coor()
    tar_joints = inverse_kinematics(tar_coor)
    dancing_tar_joints = check_limit(tar_joints)
    
    is_up = True  # Reset is_up to True at the start of dancing  
    enable_motion = True
    
    # dancing2(tar_coor, travel_time, nor)
    last_time = time.time()
    # print(f"enter dancing2")
    root.after(500, routine)
    
def routine():
    global enable_motion
    global is_up
    global dancing_travel_time
    global dancing_i
    global how_many_times
    # print(f"enter routine")
    if is_already_wake_up():
        # read_present_position()
        # servo_get_motor_velocity(0x601)
        # servo_get_status_word(0x601)
        # Memanggil fungsi print_continuously lagi setelah 1000 ms (1 detik)
        if enable_motion and dancing_i < how_many_times:
            if is_up:
                pp_angle(dancing_tar_joints, dancing_travel_time, "servo_only")
                is_up = False
            else :
                pp_angle([40, 0, 0, 0], dancing_travel_time, "servo_only")
                is_up = True
                dancing_i += 1
                print(f"counter {dancing_i} of {how_many_times}")
                
                
            # delta_time = time.time() - last_time
            # print(f"time : {delta_time:.2f}")
            root.after(int(dancing_travel_time + 100), routine)
        else:
            stop()
    else:
        print(f"Robot is not awake, cannot perform motion.")

def stop():
    global enable_motion
    global dancing_i
    enable_motion = False
    dancing_i = 0
    print("stop")
    
    

def homing():
    global last_time
    selection = get_motor_selection()
    tar_joints = [40, 0, 0, 0]
    travel_time = get_travel_time()
    # sp_angle(tar_joints, travel_time)
    pp_angle(tar_joints, travel_time, selection)
    
    
    last_time = time.time()
def on_motor_selection_changed():
    # This function, in principle, simply avoids sending commands to non-selected motors.
    # This means if the user changes the motor selection in the middle of the program,
    # it doesn't shut down the previously selected motors — their previous commands will still remain active.
    selected = motor_type.get()
    set_motor_selection(selected)
    selected_motors = get_motor_selection()
    print(f"Current motor selection: {selected_motors}")



    
    
    
# Menangani sinyal SIGINT (Ctrl + C)
signal.signal(signal.SIGINT, lambda signum, frame: signal_handler())

# Buat GUI
root = tk.Tk()
root.title("Motor Control Panel")

# Radio button for motor selection
motor_type = tk.StringVar(value="servo_only")

radio_frame = tk.LabelFrame(root, text="Motor Selection", padx=10, pady=5)
radio_frame.grid(row=0, column=0, rowspan=2, columnspan=2, padx=10, pady=5, sticky="nsew")


# tk.Radiobutton(
#     radio_frame, text="All motors", variable=motor_type, value="all",
#     command=on_motor_selection_changed
# ).grid(row=0, column=0, padx=5)

# tk.Radiobutton(
#     radio_frame, text="Stepper only", variable=motor_type, value="stepper_only",
#     command=on_motor_selection_changed
# ).grid(row=0, column=1, padx=5)

# tk.Radiobutton(
#     radio_frame, text="Servo only", variable=motor_type, value="servo_only",
#     command=on_motor_selection_changed
# ).grid(row=0, column=2, padx=5)

on_motor_selection_changed()  # Set initial motor selection

#baris 0
# error_status_button = tk.Button(root, text="error status", command=error_status)
# error_status_button.grid(row=0, column=0, columnspan=1, pady=10,sticky="ew", padx=5)

# save_settings_button = tk.Button(root, text="save settings", command=save_settings)
# save_settings_button.grid(row=0, column=1, columnspan=1, pady=10, sticky="ew", padx=5)

#baris 1 dan 2

# tk.Label(root, text="Enter max speed:").grid(row=1, column=0, padx=5, pady=5, sticky="ew")
# entry_speed = tk.Entry(root)
# entry_speed.insert(0, "1000")
# entry_speed.grid(row=1, column=1, padx=5, pady=5, sticky="ew")

tk.Label(root, text="Enter time:").grid(row=12, column=0, padx=5, pady=5, sticky="ew")
entry_time = tk.Entry(root)
entry_time.insert(0, "4000")
entry_time.grid(row=12, column=1, padx=5, pady=5, sticky="ew")

# baris 3d
wake_up_button = tk.Button(root, text="Wake Up", bg="#CC2BE8", fg="white", command=wake_up)
wake_up_button.grid(row=3, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

shutdown_button = tk.Button(root, text="Shutdown", bg="maroon", fg="white", command=shutdown)
shutdown_button.grid(row=3, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

tk.Label(
    root,
    text="hold the robot before shutdown or pressing [x] button (close button)",
    fg="red",
    font=("Helvetica", 10, "bold")
).grid(row=4, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

tk.Label(
    root,
    text="except the robot is already at home position, because it will fall down",
    fg="red",
    font=("Helvetica", 10, "bold")
).grid(row=5, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

#baris 14 - 17

tk.Label(root, text="coor z (mm):").grid(row=16, column=0, padx=5, pady=5, sticky="ew")
entry_tar_z = tk.Entry(root)
entry_tar_z.insert(0, "100")
entry_tar_z.grid(row=16, column=1, padx=5, pady=5, sticky="ew")

tk.Label(root, text="number of run:").grid(row=17, column=0, padx=5, pady=5, sticky="ew")
entry_nor = tk.Entry(root)
entry_nor.insert(0, "5")
entry_nor.grid(row=17, column=1, padx=5, pady=5, sticky="ew")


pp_move_button = tk.Button(root, text="Go to Coordinate", command=pp_move)
pp_move_button.grid(row=18, column=0, columnspan=2, pady=10, padx=5, sticky="ew")


#baris 19
motor_position_button = tk.Button(root, text="read position", bg="orange",fg="black", command=read_present_position)
motor_position_button.grid(row=19, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

dancing_button = tk.Button(root, text="Run N Times",bg="green",fg="black", command=start_dancing)
dancing_button.grid(row=19, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

# #baris 20
homing_button = tk.Button(root, text="homing", bg="cyan",fg="black",  command=homing)
homing_button.grid(row=20, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

stop_button = tk.Button(root, text="stop", bg="red",fg="white", command=stop)
stop_button.grid(row=20, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

# jmc_button = tk.Button(root, text="run Servo Command", command=from_jmc_command)
# jmc_button.grid(row=21, column=0, columnspan=2, pady=10, padx=5, sticky="ew")

# jmc2_button = tk.Button(root, text="Servo homing", command=from_jmc_homing)
# jmc2_button.grid(row=22, column=0, columnspan=2, pady=10, padx=5, sticky="ew")


# def execute_custom_commands():
#     try:
#         commands = entry_custom.get("1.0", tk.END).strip().split('\n')
#         for line in commands:
#             parts = line.strip().split()
#             if len(parts) != 9:
#                 print(f"Invalid format (expecting 9 hex values): {line}")
#                 continue
#             can_id = parts[0]
#             data_bytes = ''.join(parts[1:])  # Gabungkan semua byte tanpa spasi
#             formatted_command = f"{can_id}#{data_bytes}"
#             send_can_command(formatted_command)
#         print(f"Custom commands executed successfully.")
#     except Exception as e:
#         print(f"Error executing custom commands: {e}")
        
        
# entry_custom = tk.Text(root, height=15, width=50)
# entry_custom.insert(tk.END, """\
# 601 2B 40 60 00 0F 00 00 00
# 601 2F 60 60 00 01 00 00 00
# 601 2B 83 60 00 64 00 00 00
# 601 2B 84 60 00 64 00 00 00
# 601 23 81 60 00 0A 00 00 00
# 601 23 7A 60 00 88 13 00 00
# 601 2B 40 60 00 1F 00 00 00""")
# entry_custom.grid(row=23, column=0, columnspan=2, padx=5, pady=5, sticky="ew")



# custom_button = tk.Button(root, text="Execute custom commands", command=execute_custom_commands)
# custom_button.grid(row=24, column=0, columnspan=2, pady=10, padx=5, sticky="ew")

set_origin_button = tk.Button(root, text="set origin", command=set_origin)
set_origin_button.grid(row=25, column=1, columnspan=1, pady=10, padx=5, sticky="ew")


# Menangani event saat jendela ditutup
root.protocol("WM_DELETE_WINDOW", signal_handler)

# Memulai fungsi print_continuously saat aplikasi dimulai
# root.after(500, routine)
# Jalankan GUI
root.mainloop()
