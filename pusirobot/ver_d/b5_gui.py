import signal
import tkinter as tk
import time
from b4_function import *
from b3_motion import *
from b1_servo import servo_execute
from b2_pvt import pvt_mode_start_pvt_step
import sys
from kinematics_and_trajectory import (
    generate_trajectory_triangle,
    check_limit,
    forward_kinematics,
    inverse_kinematics,
    convert_cartesian_traj_to_joint_traj
)

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

    return travel_time

def pvt_joint():
    global last_time
    selection = get_motor_selection()
    cur_joints = get_cur_joints(selection)
    tar_joints = get_tar_joints()
    travel_time = get_travel_time()
    
    # pvt_mode_try_pvt_1(cur_joints, tar_joints, travel_time)
    pvt_mode_try_pvt_4(cur_joints, tar_joints, travel_time)
    # pvt_mode_try_pvt_5(selection)
    _, _, _, tar_joint_4 = tar_joints
    stepper_single_motor_pp_mode(ID4, tar_joint_4, travel_time, selection)
    # print(f"run motor 4 {tar_joint_4}")
    # ret = pp_angle_servo(tar_joints, travel_time, selection)
    # if ret == 1:
    #     servo_execute()  # Execute the servo command to start the movement
    group_id = 0x05
    pvt_mode_start_pvt_step(group_id)
    last_time = time.time()

def pvt_move():
    global last_time
    selection = get_motor_selection()
    cur_joints = get_cur_joints(selection)
    tar_coor = get_tar_coor()
    travel_time = get_travel_time()
    
    tar_joints = inverse_kinematics(tar_coor)
    tar_joints = check_limit(tar_joints)
    print(f"tar joint = {tar_joints} degree")
    # pvt_mode_try_pvt_1(cur_joints, tar_joints, travel_time)
    # pvt_mode_try_pvt_3(cur_joints, tar_joints, travel_time)
    pvt_mode_try_pvt_4(cur_joints, tar_joints, travel_time)
    _, _, _, tar_joint_4 = tar_joints
    stepper_single_motor_pp_mode(ID4, tar_joint_4, travel_time, selection)
    # print(f"run motor 4 {tar_joint_4}")
    # ret = pp_angle_servo(tar_joints, travel_time, selection)
    # if ret == 1:
    #     servo_execute()  # Execute the servo command to start the movement
    group_id = 0x05
    pvt_mode_start_pvt_step(group_id)
    last_time = time.time()

def pvt_circular():
    global last_time
    travel_time = 1.0
    sleep = travel_time + 0.1
    
    coor_1 = [130, 0, 0, 0]
        
    sp_coor(coor_1, travel_time, "stepper_only")
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
    selection = get_motor_selection()
    tar_joints = get_tar_joints()
    travel_time = get_travel_time()
    
    sp_angle(tar_joints, travel_time, selection)
    last_time = time.time()
    
def sp_move():  
    global last_time
    selection = get_motor_selection()
    tar_coor = get_tar_coor()
    travel_time = get_travel_time()
    
    sp_coor(tar_coor, travel_time, selection)
    last_time = time.time()
    
def pp_joint():
    global last_time
    selection = get_motor_selection()
    tar_joints = get_tar_joints()
    travel_time = get_travel_time()
    # # print(f"masuk pp joint")
    pp_angle(tar_joints, travel_time, selection)
    last_time = time.time()
    
def pp_move():  
    global last_time
    selection = get_motor_selection()
    tar_coor = get_tar_coor()
    travel_time = get_travel_time()
       
    pp_coor(tar_coor, travel_time, selection)
    last_time = time.time()


import csv
import os

def read_motion_csv(filename):
    motions = []

    with open(filename, mode='r', newline='') as file:
        reader = csv.DictReader(file)
        print("CSV fieldnames:", reader.fieldnames)  # DEBUG HEADER

        for row in reader:
            motion_type = row['motion_type']
            travel_time = int(row['travel_time'])  # in ms

            # For pp_mode: interpret as x,y,z,yaw
            d1 = float(row['d1'])
            d2 = float(row['d2'])
            d3 = float(row['d3'])
            d4 = float(row['d4'])

            motion_data = {
                'motion_type': motion_type,
                'travel_time': travel_time,
                'd1': d1,
                'd2': d2,
                'd3': d3,
                'd4': d4
            }
            # print(f"Read motion entry: {motion_data}")  # DEBUG ENTRY
            motions.append(motion_data)

    return motions

def print_motion_data(motion_data):
    i = 0
    for entry in motion_data:
        motion_type = entry['motion_type']
        travel_time = entry['travel_time']
        d1 = entry['d1']
        d2 = entry['d2']
        d3 = entry['d3']
        d4 = entry['d4']

        print(f"{i} : {motion_type}, {travel_time} ms, d1: {d1}, d2: {d2}, d3: {d3}, d4: {d4}")
        i += 1

home_angle = [360, 0, 0, 0]  # home position in coor
shuttle_coor = [166.82, -168, 10, 0]
pre_past_shelf_coor = [107, 100, 10, 90]
pickup_from_shelf_coor = [107, 224, 10, 90]
place_onto_shelf_coor = [107, 197, 10, 90]
 
 
def execute_motion_data(entry):
    motion_type = entry['motion_type']
    travel_time = entry['travel_time']
    d1 = entry['d1']
    d2 = entry['d2']
    d3 = entry['d3']
    d4 = entry['d4']
    
    if motion_type == 'pp_coor':
        pp_coor([d1, d2, d3, d4], travel_time, selection=get_motor_selection())
    elif motion_type == 'pp_angle':
        pp_angle([d1, d2, d3, d4], travel_time, selection=get_motor_selection())
    elif motion_type == 'home':
        pp_angle(home_angle, travel_time, selection=get_motor_selection())
    elif motion_type == 'shuttle':
        pp_coor(shuttle_coor, travel_time, selection=get_motor_selection())
    elif motion_type == 'pre_past_shelf':
        pp_coor(pre_past_shelf_coor, travel_time, selection=get_motor_selection())
    elif motion_type == 'pickup_from_shelf':
        pp_coor(pickup_from_shelf_coor, travel_time, selection=get_motor_selection())
    elif motion_type == 'place_onto_shelf':
        pp_coor(place_onto_shelf_coor, travel_time, selection=get_motor_selection())
    elif motion_type == 'sp_coor':
        sp_coor([d1, d2, d3, d4], travel_time, selection=get_motor_selection())
    elif motion_type == 'sp_angle':
        sp_angle([d1, d2, d3, d4], travel_time, selection=get_motor_selection())
    elif motion_type == 'pvt':
        pp_coor([d1, d2, d3, d4], travel_time, selection="servo_only")

    # print(f"Executing: {motion_type}, {travel_time} ms, d1: {d1}, d2: {d2}, d3: {d3}, d4: {d4}")


motion_enable = True
motion_size = 0  # Initialize motion size
motion_cnt = 0   
motion_data = []
pvt_cnt = 0

def convert_csv_to_list_tar_coor(filepath):
    list_tar_coor = []
    with open(filepath, mode='r', newline='') as file:
        reader = csv.DictReader(file)
        for row in reader:
            if row['motion_type'] == 'pvt':
                travel_time = int(row['travel_time'])
                x = float(row['d1'])
                y = float(row['d2'])
                z = float(row['d3'])
                yaw = float(row['d4'])
                list_tar_coor.append(([x, y, z, yaw], travel_time))
    return list_tar_coor

cur_time = 0
tar_time = 0

routine_interval = 15

pvt_1 = []
pvt_2 = []
pvt_3 = []
pvt_4 = []

max_pvt_index = 0
cur_pvt = 0
tar_pvt = 0

def start_dancing():
    global last_time
    global motion_enable
    global motion_cnt
    global motion_data
    global motion_size
    global pvt_cnt
    global tar_time
    global pvt_1, pvt_2, pvt_3, pvt_4
    global max_pvt_index
    global cur_pvt
    global tar_pvt
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filename = os.path.join(script_dir, "motion_data_4.csv")
    
    motion_data = read_motion_csv(filename)
    motion_size = len(motion_data)  # Set how many times to run based on the number of entries in the CSV    
    motion_cnt = 0  # Reset the counter
    motion_enable = True
    print_motion_data(motion_data)  # DEBUG: Print all motion data
    
    
    
    start_coor = forward_kinematics([0,0,0,0])
    list_tar_coor = convert_csv_to_list_tar_coor(filename)
    
    # print(f"{list_tar_coor}")
    for i, (coord, travel_time) in enumerate(list_tar_coor):
        print(f"{i+1}. Coordinate: {coord}, Time: {travel_time} ms")
        

    pvt_1, pvt_2, pvt_3, pvt_4 = generate_multi_straight_pvt_points(start_coor, list_tar_coor, pvt_time_interval)

    max_pvt_index = len(pvt_2)
    print(f"max pvt index: {max_pvt_index}")

    group_id = 0x05
    pvt_3_lower_limit = 60
    pvt_3_upper_limit = 80
    
    selection = get_motor_selection()

    #init PVT mode
    # if selection != "stepper_only":
    #     servo_init(7)
    if selection != "servo_only":
        pvt_mode_init(group_id, PVT_3, 1000, pvt_3_lower_limit, pvt_3_upper_limit)
    
    
    
    #write PVT points
    for i in range(900):
        # if selection != "stepper_only":    
        #     pos_1, vel_1, tim_1 = pvt_1[i]
        #     servo_set_interpolation_data(pos_1, tim_1, vel_1)
            
        if selection != "servo_only":
            pos_2, vel_2, tim_2 = pvt_2[i]
            pos_3, vel_3, tim_3 = pvt_3[i]
            pos_4, vel_4, tim_4 = pvt_4[i]
            pvt_mode_write_read(ID2, pos_2, vel_2, tim_2)
            pvt_mode_write_read(ID3, pos_3, vel_3, tim_3)        
            pvt_mode_write_read(ID4, pos_4, vel_4, tim_4)
            
        pvt_cnt = pvt_cnt + 1
        
    if selection != "servo_only": 
        pvt_mode_read_pvt_3_depth()
        for node_id in (ID2, ID3, ID4):
            init_single_motor_change_group_id(node_id, group_id)
        pvt_mode_start_pvt_step(group_id)   
             
    if selection != "stepper_only":
        entry = motion_data[motion_cnt]
        travel_time = entry['travel_time']
        motion_cnt += 1
        tar_time += travel_time
        tar_pvt = int(travel_time/pvt_time_interval)
        execute_motion_data(entry)
        # servo_execute()
    #     servo_get_sub_mode()
    #     servo_get_buffer_free_count()
    #     servo_get_next_trajectory_segment_id()
    #     print(f"execute servo")        
        
    root.after(int(routine_interval), routine)
    last_time = time.time()
    



def routine():
    global motion_enable
    global motion_cnt
    global motion_size
    global motion_data
    global pvt_cnt
    global cur_time
    global tar_time
    global max_pvt_index
    global cur_pvt
    global tar_pvt
    
    # read_present_position()
    selection = get_motor_selection()
    # print(f"enter routine")
    if is_already_wake_up():
        if motion_enable and motion_cnt < motion_size:
            root.after(int(routine_interval), routine)
            
            
            if selection != "servo_only":
                depth = read_pvt_3_depth(ID2)
                if depth < 80:
                # for i in range(2):
                    # print(pvt_cnt)
                    if pvt_cnt < max_pvt_index:
                        pos_2, vel_2, tim_2 = pvt_2[pvt_cnt]
                        pos_3, vel_3, tim_3 = pvt_3[pvt_cnt]
                        pos_4, vel_4, tim_4 = pvt_4[pvt_cnt]
                        pvt_mode_write_read(ID2, pos_2, vel_2, tim_2)
                        pvt_mode_write_read(ID3, pos_3, vel_3, tim_3)        
                        pvt_mode_write_read(ID4, pos_4, vel_4, tim_4)
                
                        pvt_cnt = pvt_cnt + 1
                        cur_pvt += 1
                        
            # cur_time = (time.time() - last_time) * 1000
            # print(f"cur time: {cur_time:.2f}, pvt cnt = {pvt_cnt} / {(pvt_cnt/20):.2f} d={(pvt_cnt/20)-(cur_time/1000):.2f}")
            
            print(depth)
            
            if (depth == 0):
                stop()
            
            
            if cur_pvt >= tar_pvt:
                entry = motion_data[motion_cnt]
                # motion_type = entry['motion_type']
                # entry['travel_time'] = get_travel_time() #atur waktu
                travel_time = entry['travel_time']
                # d1 = entry['d1']
                # d2 = entry['d2']
                # d3 = entry['d3']
                # d4 = entry['d4']
                tar_time += travel_time
                tar_pvt = int(travel_time/pvt_time_interval)
                cur_pvt = 0
                # print(f"tar pvt = {tar_pvt}")
               
            
            
            
                # read_present_position()
                # print_red(f"{motion_type}, {travel_time} ms, d1: {d1}, d2: {d2}, d3: {d3}, d4: {d4}")
                # print(f"counter {motion_cnt + 2} of {motion_size}")
                # print_red(f"tar coor : x:{d1} mm, y:{d2} mm, z:{d3} mm, yaw:{d4}°")
                motion_cnt += 1
                execute_motion_data(entry)
                
        else:
            stop()
    else:
        print(f"Robot is not awake, cannot perform motion.")

def stop():
    global motion_enable
    global motion_cnt
    global pvt_cnt
    global cur_pvt
    motion_enable = False
    motion_cnt = 0
    pvt_cnt = 0
    cur_pvt = 0
    print("stop")
    homing()
    
def homing():
    global last_time
    selection = get_motor_selection()
    tar_joints = home_angle
    travel_time = get_travel_time()
    if selection != "stepper_only":
        pp_angle(tar_joints, 6000, "servo_only")
    if selection != "servo_only":
        sp_angle(tar_joints, 6000, selection)
    
    
    
    last_time = time.time()


def on_motor_selection_changed():
    # This function, in principle, simply avoids sending commands to non-selected motors.
    # This means if the user changes the motor selection in the middle of the program,
    # it doesn't shut down the previously selected motors — their previous commands will still remain active.
    selected = motor_type.get()
    set_motor_selection(selected)
    selected_motors = get_motor_selection()
    print(f"Current motor selection: {selected_motors}")

def read_present_position():
    global last_time
    selection = get_motor_selection()
    cur_joints = get_cur_joints(selection)
    cur_coor = forward_kinematics(cur_joints)
    
    formatted_angles = "°, ".join([f"{angle:.2f}" for angle in cur_joints])
    print_yellow(f"cur joint : {formatted_angles}°")
    # j1, j2, j3, j4 = cur_joints
    # print_yellow(f"cur servo's angle: {j1:.1f}°")
    
    cur_x, cur_y, cur_z, cur_yaw = cur_coor
    print_orange(f"cur coor : x:{cur_x:.1f} mm, y:{cur_y:.1f} mm, z:{cur_z:.1f} mm, yaw:{cur_yaw:.1f}°")
    
    delta_time = time.time() - last_time
    print(f"time : {delta_time:.2f} seconds")
    # print_orange(f"cur coor z:{cur_z:.1f} mm")
    
    # servo_vel = servo_get_motor_velocity(ID1)
    # servo_status = servo_get_status_word(ID1)
    
    # print(f"servo status (hex): {servo_status:08X}, servo velocity: {servo_vel}")
    
    # t1 = time.time()
    # pvt_mode_write_read(ID2, 1000, 100, 50)
    # pvt_mode_write_read(ID2, 2000, 100, 50)
    # pvt_mode_write_read(ID2, 3000, 100, 50)
    # pvt_mode_write_read(ID2, 4000, 100, 50)
    # pvt_mode_write_read(ID2, 5000, 100, 50)
    # pvt_mode_write_read(ID2, 6000, 100, 50)
    # t2 = time.time() - t1
    # print(f"total = {t2} s {t2*1000} ms")
    
# Menangani sinyal SIGINT (Ctrl + C)
signal.signal(signal.SIGINT, lambda signum, frame: signal_handler())

# Buat GUI
root = tk.Tk()
root.title("Motor Control Panel")

# Radio button for motor selection
motor_type = tk.StringVar(value="all")  # Default to "all motors"

radio_frame = tk.LabelFrame(root, text="Motor Selection", padx=10, pady=5)
radio_frame.grid(row=0, column=0, rowspan=2, columnspan=2, padx=10, pady=5, sticky="nsew")


tk.Radiobutton(
    radio_frame, text="All motors", variable=motor_type, value="all",
    command=on_motor_selection_changed
).grid(row=0, column=0, padx=5)

tk.Radiobutton(
    radio_frame, text="Stepper only", variable=motor_type, value="stepper_only",
    command=on_motor_selection_changed
).grid(row=0, column=1, padx=5)

tk.Radiobutton(
    radio_frame, text="Servo only", variable=motor_type, value="servo_only",
    command=on_motor_selection_changed
).grid(row=0, column=2, padx=5)

on_motor_selection_changed()  # Set initial motor selection

tk.Label(root, text="time (ms):").grid(row=2, column=0, padx=5, pady=5, sticky="ew")
entry_time = tk.Entry(root)
entry_time.insert(0, "4000")
entry_time.grid(row=2, column=1, padx=5, pady=5, sticky="ew")

# baris 3d
wake_up_button = tk.Button(root, text="Wake Up", bg="#CC2BE8", fg="white", command=wake_up)
wake_up_button.grid(row=3, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

shutdown_button = tk.Button(root, text="Shutdown", bg="maroon", fg="white", command=shutdown)
shutdown_button.grid(row=3, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

# baris 4 sampai 7
tk.Label(root, text="angle 1 (degree):").grid(row=4, column=0, padx=5, pady=5, sticky="ew")
entry_tar_joint_1 = tk.Entry(root)
entry_tar_joint_1.insert(0, "360")
entry_tar_joint_1.grid(row=4, column=1, padx=5, pady=5, sticky="ew")

tk.Label(root, text="angle 2 (degree):").grid(row=5, column=0, padx=5, pady=5, sticky="ew")
entry_tar_joint_2 = tk.Entry(root)
entry_tar_joint_2.insert(0, "730.68")#"482.5")
entry_tar_joint_2.grid(row=5, column=1, padx=5, pady=5, sticky="ew")

tk.Label(root, text="angle 3 (degree):").grid(row=6, column=0, padx=5, pady=5, sticky="ew")
entry_tar_joint_3 = tk.Entry(root)
entry_tar_joint_3.insert(0, "219.07")#"-187.5")
entry_tar_joint_3.grid(row=6, column=1, padx=5, pady=5, sticky="ew")

tk.Label(root, text="angle 4 (degree):").grid(row=7, column=0, padx=5, pady=5, sticky="ew")
entry_tar_joint_4 = tk.Entry(root)
entry_tar_joint_4.insert(0, "-712.50")#262.5
entry_tar_joint_4.grid(row=7, column=1, padx=5, pady=5, sticky="ew")

#baris 13
# pvt_mode_init_button = tk.Button(root, text="PVT Mode Init", command=pvt_mode_init)
# pvt_mode_init_button.grid(row=13, column=0, columnspan=1, pady=10, padx=5, sticky="ew")
# pvt_mode_init_button = tk.Button(root, text="read PVT3 depth", command=pvt_mode_read_pvt_3_depth)
# pvt_mode_init_button.grid(row=13, column=0, columnspan=1, pady=10, padx=5, sticky="ew")


# sp_joint_button = tk.Button(root, text="SP try", command=sp_joint)
# sp_joint_button.grid(row=13, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

pvt_mode_button = tk.Button(root, text="PVT angle", command=pvt_joint)
pvt_mode_button.grid(row=13, column=0, columnspan=1, pady=10, padx=5, sticky="ew")


pp_mode_button = tk.Button(root, text="PP angle", command=pp_joint)
pp_mode_button.grid(row=13, column=1, columnspan=1, pady=10, padx=5, sticky="ew")



#baris 14 - 17

tk.Label(root, text="coor x (mm):").grid(row=14, column=0, padx=5, pady=5, sticky="ew")
entry_tar_x = tk.Entry(root)
entry_tar_x.insert(0, "107")
entry_tar_x.grid(row=14, column=1, padx=5, pady=5, sticky="ew")

tk.Label(root, text="coor y (mm):").grid(row=15, column=0, padx=5, pady=5, sticky="ew")
entry_tar_y = tk.Entry(root)
entry_tar_y.insert(0, "125")
entry_tar_y.grid(row=15, column=1, padx=5, pady=5, sticky="ew")

tk.Label(root, text="coor z (mm):").grid(row=16, column=0, padx=5, pady=5, sticky="ew")
entry_tar_z = tk.Entry(root)
entry_tar_z.insert(0, "90")
entry_tar_z.grid(row=16, column=1, padx=5, pady=5, sticky="ew")

tk.Label(root, text="coor yaw (degree):").grid(row=17, column=0, padx=5, pady=5, sticky="ew")
entry_tar_yaw = tk.Entry(root)
entry_tar_yaw.insert(0, "90")
entry_tar_yaw.grid(row=17, column=1, padx=5, pady=5, sticky="ew")

# #baris 18
# sp_move_button = tk.Button(root, text="SP move", command=sp_move)
# sp_move_button.grid(row=18, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

pvt_move_button = tk.Button(root, text="PVT move", command=pvt_move)
pvt_move_button.grid(row=18, column=0, columnspan=1, pady=10, padx=5, sticky="ew")


pp_move_button = tk.Button(root, text="PP coor", command=pp_move)
pp_move_button.grid(row=18, column=1, columnspan=1, pady=10, padx=5, sticky="ew")


#baris 19
motor_position_button = tk.Button(root, text="read position", bg="orange",fg="black", command=read_present_position)
motor_position_button.grid(row=19, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

dancing_button = tk.Button(root, text="dancing",bg="green",fg="black", command=start_dancing)
dancing_button.grid(row=19, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

#baris 20
homing_button = tk.Button(root, text="homing", bg="cyan",fg="black",  command=homing)
homing_button.grid(row=20, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

stop_button = tk.Button(root, text="stop", bg="red",fg="white", command=stop)
stop_button.grid(row=20, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

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