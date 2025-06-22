import signal
import tkinter as tk
import time
from b4_function import wake_up, shutdown, read_present_position, get_encoder_position, set_origin, is_already_wake_up
from b3_motion import dancing,dancing2, sp_angle, sp_coor, pvt_circular, pvt_mode_try_pvt_3, pp_angle, pp_coor
from b1_servo import servo_get_motor_velocity, servo_get_status_word
import sys

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
    if is_already_wake_up():
         shutdown()
        #  homing()
    print("SIGINT received, closing application...")
   
    root.quit()  # Hentikan event loop
    root.destroy()  # Hancurkan GUI
    exit(0)  # Keluar sepenuhnya dari program
    
def get_tar_joints():
    try:  
        tar_joint_1 = float(entry_tar_joint_1.get())
        tar_joint_2 = float(entry_tar_joint_2.get())
        tar_joint_3 = float(entry_tar_joint_3.get())
        tar_joint_4 = float(entry_tar_joint_4.get())
        
    except ValueError:
        print("Please enter valid numbers for angles.")
        
    tar_joints = [tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4]

    return tar_joints

def get_tar_coor():
    try:
        tar_x = float(entry_tar_x.get())
        tar_y = float(entry_tar_y.get())
        tar_z = float(entry_tar_z.get())
        tar_yaw = float(entry_tar_yaw.get())
    except ValueError:
        print("Please enter valid numbers for angles.")
    
    tar_coor = [tar_x, tar_y, tar_z, tar_yaw]

    return tar_coor

def get_travel_time():
    try:
        travel_time = int(entry_time.get())
    except ValueError:
        print("Please enter valid numbers for angles.")
    
    travel_time = travel_time/1000

    return travel_time

def pvt_joint():
    global last_time
    cur_joints = read_present_position()
    tar_joints = get_tar_joints()
    travel_time =get_travel_time()
    
    # pvt_mode_try_pvt_1(cur_joints, tar_joints, travel_time)
    pvt_mode_try_pvt_3(cur_joints, tar_joints, travel_time)
    last_time = time.time()

def pvt_move():
    global last_time
    travel_time = 1.0
    sleep = travel_time + 0.1
    
    coor_1 = [130, 0, 0, 0]
        
    sp_coor(coor_1, travel_time)
    time.sleep(sleep)
    
    cur_joint = read_present_position()
    # Contoh penggunaan
    cur_pos = (130, 0)  # (x, y) posisi awal
    center_pos = (170, 0)  # Pusat lingkaran
    end_angle = 360  # Gerakan setengah lingkaran
    travel_time = 4  # dalam detik
    direction = "CCW"  # Arah rotasi
    
    
    pvt_circular(cur_pos, center_pos, end_angle, travel_time, direction)
    last_time = time.time()
    
    
def sp_joint():
    global last_time
    tar_joints = get_tar_joints()
    travel_time = get_travel_time()
    
    sp_angle(tar_joints, travel_time)
    last_time = time.time()
    
def sp_move():  
    global last_time
    tar_coor = get_tar_coor()
    travel_time = get_travel_time()
    
    sp_coor(tar_coor, travel_time)
    last_time = time.time()
    
def pp_joint():
    global last_time
    tar_joints = get_tar_joints()
    travel_time = get_travel_time()
    travel_time = travel_time * 1000
    
    pp_angle(tar_joints, travel_time, 10000)
    last_time = time.time()
    
def pp_move():  
    global last_time
    tar_coor = get_tar_coor()
    travel_time = get_travel_time()
    travel_time = travel_time * 1000
       
    pp_coor(tar_coor, travel_time, 10000)
    last_time = time.time()
    

def start_dancing():
    global last_time
    travel_time = get_travel_time()
    # dancing(travel_time)
    dancing2(travel_time)
    last_time = time.time()
    
def homing():
    global last_time
    tar_joints = [0, 0, 0, 0]
    travel_time = get_travel_time()
    # sp_angle(tar_joints, travel_time)
    travel_time = travel_time * 1000
    pp_angle(tar_joints, travel_time, 10000)
    
    
    last_time = time.time()


def on_motor_selection_changed():
    selected = motor_type.get()
    print(f"Motor selection changed: {selected}")

# def routine():
# #     if is_already_wake_up():
# #         read_present_position()
# #         # servo_get_motor_velocity(0x601)
# #         # servo_get_status_word(0x601)
# #         # Memanggil fungsi print_continuously lagi setelah 1000 ms (1 detik)
# #         delta_time = time.time() - last_time
# #         print(f"time : {delta_time:.2f}")
    
#     root.after(500, routine)
    
# Menangani sinyal SIGINT (Ctrl + C)
signal.signal(signal.SIGINT, lambda signum, frame: signal_handler())

# Buat GUI
root = tk.Tk()
root.title("Motor Control Panel")

# Radio button for motor selection
motor_type = tk.StringVar(value="all")

radio_frame = tk.LabelFrame(root, text="Motor Selection", padx=10, pady=5)
radio_frame.grid(row=0, column=0, rowspan=2, columnspan=2, padx=10, pady=5, sticky="nsew")


tk.Radiobutton(
    radio_frame, text="All motors", variable=motor_type, value="all",
    command=on_motor_selection_changed
).grid(row=0, column=0, padx=5)

tk.Radiobutton(
    radio_frame, text="Stepper only", variable=motor_type, value="stepper",
    command=on_motor_selection_changed
).grid(row=0, column=1, padx=5)

tk.Radiobutton(
    radio_frame, text="Servo only", variable=motor_type, value="servo",
    command=on_motor_selection_changed
).grid(row=0, column=2, padx=5)



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

tk.Label(root, text="Enter time:").grid(row=2, column=0, padx=5, pady=5, sticky="ew")
entry_time = tk.Entry(root)
entry_time.insert(0, "4000")
entry_time.grid(row=2, column=1, padx=5, pady=5, sticky="ew")

# baris 3d
wake_up_button = tk.Button(root, text="Wake Up", command=wake_up)
wake_up_button.grid(row=3, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

shutdown_button = tk.Button(root, text="Shutdown", bg="red", fg="white", command=shutdown)
shutdown_button.grid(row=3, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

# baris 4 sampai 7
tk.Label(root, text="angle 1 (degree):").grid(row=4, column=0, padx=5, pady=5, sticky="ew")
entry_tar_joint_1 = tk.Entry(root)
entry_tar_joint_1.insert(0, "360")
entry_tar_joint_1.grid(row=4, column=1, padx=5, pady=5, sticky="ew")

tk.Label(root, text="angle 2 (degree):").grid(row=5, column=0, padx=5, pady=5, sticky="ew")
entry_tar_joint_2 = tk.Entry(root)
entry_tar_joint_2.insert(0, "482.5")
entry_tar_joint_2.grid(row=5, column=1, padx=5, pady=5, sticky="ew")

tk.Label(root, text="angle 3 (degree):").grid(row=6, column=0, padx=5, pady=5, sticky="ew")
entry_tar_joint_3 = tk.Entry(root)
entry_tar_joint_3.insert(0, "-187.5")
entry_tar_joint_3.grid(row=6, column=1, padx=5, pady=5, sticky="ew")

tk.Label(root, text="angle 4 (degree):").grid(row=7, column=0, padx=5, pady=5, sticky="ew")
entry_tar_joint_4 = tk.Entry(root)
entry_tar_joint_4.insert(0, "-262.5")#262.5
entry_tar_joint_4.grid(row=7, column=1, padx=5, pady=5, sticky="ew")

#baris 13
# pvt_mode_init_button = tk.Button(root, text="PVT Mode Init", command=pvt_mode_init)
# pvt_mode_init_button.grid(row=13, column=0, columnspan=1, pady=10, padx=5, sticky="ew")
# pvt_mode_init_button = tk.Button(root, text="read PVT3 depth", command=pvt_mode_read_pvt_3_depth)
# pvt_mode_init_button.grid(row=13, column=0, columnspan=1, pady=10, padx=5, sticky="ew")
sp_joint_button = tk.Button(root, text="SP try", command=sp_joint)
sp_joint_button.grid(row=13, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

pp_mode_button = tk.Button(root, text="PP try", command=pp_joint)
pp_mode_button.grid(row=13, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

# pvt_mode_button = tk.Button(root, text="PVT try", command=pvt_joint)
# pvt_mode_button.grid(row=13, column=1, columnspan=1, pady=10, padx=5, sticky="ew")


#baris 14 - 17

tk.Label(root, text="coor x (mm):").grid(row=14, column=0, padx=5, pady=5, sticky="ew")
entry_tar_x = tk.Entry(root)
entry_tar_x.insert(0, "130")
entry_tar_x.grid(row=14, column=1, padx=5, pady=5, sticky="ew")

tk.Label(root, text="coor y (mm):").grid(row=15, column=0, padx=5, pady=5, sticky="ew")
entry_tar_y = tk.Entry(root)
entry_tar_y.insert(0, "0")
entry_tar_y.grid(row=15, column=1, padx=5, pady=5, sticky="ew")

tk.Label(root, text="coor z (mm):").grid(row=16, column=0, padx=5, pady=5, sticky="ew")
entry_tar_z = tk.Entry(root)
entry_tar_z.insert(0, "0")
entry_tar_z.grid(row=16, column=1, padx=5, pady=5, sticky="ew")

tk.Label(root, text="coor yaw (degree):").grid(row=17, column=0, padx=5, pady=5, sticky="ew")
entry_tar_yaw = tk.Entry(root)
entry_tar_yaw.insert(0, "0")
entry_tar_yaw.grid(row=17, column=1, padx=5, pady=5, sticky="ew")

#baris 18
sp_move_button = tk.Button(root, text="SP move", command=sp_move)
sp_move_button.grid(row=18, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

pp_move_button = tk.Button(root, text="PP move", command=pp_move)
pp_move_button.grid(row=18, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

# pvt_move_button = tk.Button(root, text="PVT circular", command=pvt_move)
# pvt_move_button.grid(row=18, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

#baris 19
motor_position_button = tk.Button(root, text="read position", bg="orange",fg="black", command=read_present_position)
motor_position_button.grid(row=19, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

dancing_button = tk.Button(root, text="dancing", command=start_dancing)
dancing_button.grid(row=19, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

#baris 20
homing_button = tk.Button(root, text="homing", command=homing)
homing_button.grid(row=20, column=0, columnspan=2, pady=10, padx=5, sticky="ew")

#baris 21
encoder_position_button = tk.Button(root, text="read encoder", command=get_encoder_position)
encoder_position_button.grid(row=21, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

set_origin_button = tk.Button(root, text="set origin", command=set_origin)
set_origin_button.grid(row=21, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

# Menangani event saat jendela ditutup
root.protocol("WM_DELETE_WINDOW", signal_handler)

# Memulai fungsi print_continuously saat aplikasi dimulai
# root.after(500, routine)
# Jalankan GUI
root.mainloop()

# suggestion:
# motor position pada pdo1 sepertinya tidak dibutuhkan, kalo dihapus program kalkulasinya bisa hemat waktu
# coba rangkap data yang dikitim lewat pvt ke pdo saja bisa hemat waktu

#note:
#speed in sp mode is relative to microstep
#reach position information in sp mode can be read from controller status (busy_state)
#pvt mode is more like speed based rather thank position based


#to do:
# 1. robot harus bisa menjalankan s-shape motion
# bagaimana cara ngetes s-shape motion?
# 2. buat pvt mode relative terhadap current position
# 3. pvt mode with sp correction
# 4. make xyz with time control (4D)
# 5. try PP mode again with pulse_to_step() function
# 6. make the third file


#key role:
# Target : The robot must follow a predefined trajectory to pick up boxes.
# must to have: 
# 1. Organic movement (smooth acceleration & deceleration)
#    - make a 4D trajectory tester
#    - try PP mode again with pulse_to_step() function
#    - try pvt mode 1 dengan check point
#               
# 2. Precision & no incremental drift
#    - maybe can be combining with sp mode after
#    - found what cause the drift in PVT mode.
#               
# 3. anomaly detection & activate Emergency response
#    - decide the pin for the servo brake (GPIO2 - Pin3)
#    - read the position frequently, if the motor out of tolerance, activaate the emergency functio