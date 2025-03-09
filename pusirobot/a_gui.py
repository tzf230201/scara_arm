import signal
import tkinter as tk
from a_can import save_settings, on_closing, wake_up, shutdown, pvt_mode_init, pvt_mode_try_pvt_1, pvt_mode_try_pvt_3,pvt_mode_read_pvt_3_depth,  read_present_position, encoder_position, calib_0,sp_mode_linear_motion

def signal_handler():
    print("SIGINT received, closing application...")
    on_closing()
    root.quit()  # Hentikan event loop
    root.destroy()  # Hancurkan GUI
    exit(0)  # Keluar sepenuhnya dari program

def pvt_try():
    
    cur_joints = read_present_position()

    try:
        tar_joint_1 = float(entry_tar_joint_1.get())
        tar_joint_2 = float(entry_tar_joint_2.get())
        tar_joint_3 = float(entry_tar_joint_3.get())
        tar_joint_4 = float(entry_tar_joint_4.get())
        travel_time = int(entry_time.get())
    except ValueError:
        print("Please enter valid numbers for angles.")
    
    tar_joints = [tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4]
    
    travel_time = travel_time/1000
    
    # pvt_mode_try_pvt_1(cur_joints, tar_joints, travel_time)
    pvt_mode_try_pvt_3(cur_joints, tar_joints, travel_time)

def sp_try():
    cur_joints = read_present_position()

    try:
        tar_joint_1 = float(entry_tar_joint_1.get())
        tar_joint_2 = float(entry_tar_joint_2.get())
        tar_joint_3 = float(entry_tar_joint_3.get())
        tar_joint_4 = float(entry_tar_joint_4.get())
        travel_time = int(entry_time.get())
    except ValueError:
        print("Please enter valid numbers for angles.")
    
    tar_joints = [tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4]
    
    travel_time = travel_time/1000
    
    sp_mode_linear_motion(tar_joints, travel_time)

def homing():
    try:
        travel_time = int(entry_time.get())
    except ValueError:
        print("Please enter valid numbers for angles.")
    tar_joints = [0, 0, 0, 0]
    travel_time = travel_time/1000
    
    sp_mode_linear_motion(tar_joints, travel_time)
    
    
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

tk.Label(root, text="Enter time:").grid(row=2, column=0, padx=5, pady=5, sticky="ew")
entry_time = tk.Entry(root)
entry_time.insert(0, "4000")
entry_time.grid(row=2, column=1, padx=5, pady=5, sticky="ew")

# baris 3
wake_up_button = tk.Button(root, text="Wake Up", command=wake_up)
wake_up_button.grid(row=3, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

shutdown_button = tk.Button(root, text="Shutdown", bg="red", fg="white", command=shutdown)
shutdown_button.grid(row=3, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

# baris 4 sampai 7
tk.Label(root, text="Motor 1 angle (degree):").grid(row=4, column=0, padx=5, pady=5, sticky="ew")
entry_tar_joint_1 = tk.Entry(root)
entry_tar_joint_1.insert(0, "360")
entry_tar_joint_1.grid(row=4, column=1, padx=5, pady=5, sticky="ew")

tk.Label(root, text="Motor 2 angle (degree):").grid(row=5, column=0, padx=5, pady=5, sticky="ew")
entry_tar_joint_2 = tk.Entry(root)
entry_tar_joint_2.insert(0, "482.5")
entry_tar_joint_2.grid(row=5, column=1, padx=5, pady=5, sticky="ew")

tk.Label(root, text="Motor 3 angle (degree):").grid(row=6, column=0, padx=5, pady=5, sticky="ew")
entry_tar_joint_3 = tk.Entry(root)
entry_tar_joint_3.insert(0, "-187.5")
entry_tar_joint_3.grid(row=6, column=1, padx=5, pady=5, sticky="ew")

tk.Label(root, text="Motor 4 angle (degree):").grid(row=7, column=0, padx=5, pady=5, sticky="ew")
entry_tar_joint_4 = tk.Entry(root)
entry_tar_joint_4.insert(0, "-262.5")#262.5
entry_tar_joint_4.grid(row=7, column=1, padx=5, pady=5, sticky="ew")

#baris 13
# pvt_mode_init_button = tk.Button(root, text="PVT Mode Init", command=pvt_mode_init)
# pvt_mode_init_button.grid(row=13, column=0, columnspan=1, pady=10, padx=5, sticky="ew")
pvt_mode_init_button = tk.Button(root, text="read PVT3 depth", command=pvt_mode_read_pvt_3_depth)
pvt_mode_init_button.grid(row=13, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

pvt_mode_button = tk.Button(root, text="PVT try", command=pvt_try)
pvt_mode_button.grid(row=13, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

#baris 19
motor_position_button = tk.Button(root, text="motor position", bg="orange",fg="black", command=read_present_position)
motor_position_button.grid(row=19, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

sp_mode_button = tk.Button(root, text="SP try", command=sp_try)
sp_mode_button.grid(row=19, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

#baris 20
homing_button = tk.Button(root, text="homing", command=homing)
homing_button.grid(row=20, column=0, columnspan=2, pady=10, padx=5, sticky="ew")

#baris 21
encoder_position_button = tk.Button(root, text="encoder position", command=encoder_position)
encoder_position_button.grid(row=21, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

calib_0_button = tk.Button(root, text="calib_0", command=calib_0)
calib_0_button.grid(row=21, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

# Menangani event saat jendela ditutup
root.protocol("WM_DELETE_WINDOW", signal_handler)

# Jalankan GUI
root.mainloop()

# coba test pvt 3
# coba ganti triangle trajector