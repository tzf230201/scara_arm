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
wake_up_button = tk.Button(root, text="Wake Up", command=wake_up)
wake_up_button.grid(row=3, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

shutdown_button = tk.Button(root, text="Shutdown", bg="red", fg="white", command=shutdown)
shutdown_button.grid(row=3, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

tk.Label(
    root,
    text="hold the robot before shutdown, except the robot is already at home position",
    fg="red",
    font=("Helvetica", 10, "bold")
).grid(row=4, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

#baris 14 - 17

tk.Label(root, text="coor z (mm):").grid(row=16, column=0, padx=5, pady=5, sticky="ew")
entry_tar_z = tk.Entry(root)
entry_tar_z.insert(0, "0")
entry_tar_z.grid(row=16, column=1, padx=5, pady=5, sticky="ew")


pp_move_button = tk.Button(root, text="go to coordinate", command=pp_move)
pp_move_button.grid(row=18, column=0, columnspan=2, pady=10, padx=5, sticky="ew")


#baris 19
motor_position_button = tk.Button(root, text="read position", bg="orange",fg="black", command=read_present_position)
motor_position_button.grid(row=19, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

dancing_button = tk.Button(root, text="10 times", command=start_dancing)
dancing_button.grid(row=19, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

#baris 20
homing_button = tk.Button(root, text="homing", command=homing)
homing_button.grid(row=20, column=0, columnspan=2, pady=10, padx=5, sticky="ew")



# Menangani event saat jendela ditutup
root.protocol("WM_DELETE_WINDOW", signal_handler)

# Memulai fungsi print_continuously saat aplikasi dimulai
# root.after(500, routine)
# Jalankan GUI
root.mainloop()
