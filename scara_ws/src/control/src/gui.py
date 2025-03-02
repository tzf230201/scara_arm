#!/usr/bin/env python3

import rospy
import tkinter as tk
from std_msgs.msg import String  # Import the custom message
from control.msg import Params  # Import the custom message
import signal
import sys
import json
import logging

# Setup logging
logging.basicConfig(level=logging.INFO)

# Global dictionary for parameters
params = Params()  # Initialize with Params message type

# Define ROS publishers for the topics
def initialize_publishers():
    global pub_button_command, pub_parameters
    pub_button_command = rospy.Publisher('button_command', String, queue_size=10)
    pub_parameters = rospy.Publisher('parameters', Params, queue_size=10)  # Update to use Params

def signal_handler(sig, frame):
    on_closing()
    rospy.signal_shutdown('Shutdown signal received')
    sys.exit(0)

def send_parameters():
    logging.info(f"Sending parameters: {params}")
    pub_parameters.publish(params)  # Publish the Params message

def send_button_command(command_name):
    logging.info(f"Sending button command: {command_name}")
    pub_button_command.publish(command_name)

def update_params_from_entries():
    global params
    params.x = validate_numeric(entry_x_pos.get())
    params.y = validate_numeric(entry_y_pos.get())
    params.z = validate_numeric(entry_z_pos.get())
    params.c = validate_numeric(entry_c_pos.get())
    params.motor1 = validate_numeric(entry_motor1.get())
    params.motor2 = validate_numeric(entry_motor2.get())
    params.motor3 = validate_numeric(entry_motor3.get())
    params.motor4 = validate_numeric(entry_motor4.get())
    params.duration_time = validate_numeric(entry_duration_time.get())
    params.max_speed = validate_numeric(entry_max_speed.get())
    params.kpP = validate_numeric(entry_kpP.get())
    params.kdP = validate_numeric(entry_kdP.get())
    params.kfP = validate_numeric(entry_kfP.get())

def take_data():
    update_params_from_entries()
    send_parameters()
    send_button_command("take_data")

def move_coor():
    update_params_from_entries()
    send_parameters()
    send_button_command("move_coor")

def move_deg():
    update_params_from_entries()
    send_parameters()
    send_button_command("move_deg")

def initialize_motors():
    update_params_from_entries()
    send_parameters()
    send_button_command("initialize_motors")

def send_kp_values():
    update_params_from_entries()
    send_parameters()
    send_button_command("set_kp_values")

def read_error():
    send_button_command("read_error")

def fault_reset():
    send_button_command("fault_reset")

def on_closing():
    rospy.signal_shutdown('GUI closed')
    root.destroy()

def validate_numeric(value, default=0):
    try:
        return float(value)
    except ValueError:
        logging.error(f"Invalid numeric value: {value}. Defaulting to {default}.")
        return default

# Initialize the ROS node
rospy.init_node('motor_control_gui', anonymous=True)
initialize_publishers()

# Setup signal handler for clean shutdown
signal.signal(signal.SIGINT, signal_handler)

# Tkinter GUI
root = tk.Tk()
root.title("Motor Control Panel")

# Frame for Initialize Motors, Duration Time, and Maximum Speed
frame_init_params = tk.Frame(root)
frame_init_params.grid(row=0, column=0, columnspan=3, pady=10)

# Duration Time Input
tk.Label(frame_init_params, text="Duration Time (ms):").grid(row=0, column=0, padx=5, pady=5)
entry_duration_time = tk.Entry(frame_init_params)
entry_duration_time.insert(0, "4000")
entry_duration_time.grid(row=0, column=1, padx=5, pady=5)

# Maximum Speed Input
tk.Label(frame_init_params, text="Maximum Speed (rpm):").grid(row=1, column=0, padx=5, pady=5)
entry_max_speed = tk.Entry(frame_init_params)
entry_max_speed.insert(0, "120")
entry_max_speed.grid(row=1, column=1, padx=5, pady=5)

# Initialize and Shutdown Motors Buttons
initialize_button = tk.Button(frame_init_params, text="Initialize Motors", command=initialize_motors)
initialize_button.grid(row=2, column=0, padx=5, pady=10)

shutdown_button = tk.Button(frame_init_params, text="Shutdown Motors", command=lambda: send_button_command("shutdown_motors"))
shutdown_button.grid(row=2, column=1, padx=5, pady=10)

# Input fields for x, y, z, and c positions
tk.Label(root, text="x position in mm").grid(row=3, column=0, padx=5, pady=5)
entry_x_pos = tk.Entry(root)
entry_x_pos.insert(0, "258")
entry_x_pos.grid(row=3, column=1, padx=5, pady=5)

tk.Label(root, text="y position in mm").grid(row=4, column=0, padx=5, pady=5)
entry_y_pos = tk.Entry(root)
entry_y_pos.insert(0, "0")
entry_y_pos.grid(row=4, column=1, padx=5, pady=5)

tk.Label(root, text="z position in mm").grid(row=5, column=0, padx=5, pady=5)
entry_z_pos = tk.Entry(root)
entry_z_pos.insert(0, "0")
entry_z_pos.grid(row=5, column=1, padx=5, pady=5)

tk.Label(root, text="c position in degrees").grid(row=6, column=0, padx=5, pady=5)
entry_c_pos = tk.Entry(root)
entry_c_pos.insert(0, "0")
entry_c_pos.grid(row=6, column=1, padx=5, pady=5)

# Move Coor Button
move_coor_button = tk.Button(root, text="Move Coor", command=move_coor)
move_coor_button.grid(row=7, column=0, columnspan=3, pady=10)

# Input fields for motor positions
tk.Label(root, text="Enter position for Motor 1 (degrees):").grid(row=8, column=0, padx=5, pady=5)
entry_motor1 = tk.Entry(root)
entry_motor1.insert(0, "0")
entry_motor1.grid(row=8, column=1, padx=5, pady=5)

tk.Label(root, text="Enter position for Motor 2 (degrees):").grid(row=9, column=0, padx=5, pady=5)
entry_motor2 = tk.Entry(root)
entry_motor2.insert(0, "0")
entry_motor2.grid(row=9, column=1, padx=5, pady=5)

tk.Label(root, text="Enter position for Motor 3 (degrees):").grid(row=10, column=0, padx=5, pady=5)
entry_motor3 = tk.Entry(root)
entry_motor3.insert(0, "0")
entry_motor3.grid(row=10, column=1, padx=5, pady=5)

tk.Label(root, text="Enter position for Motor 4 (degrees):").grid(row=11, column=0, padx=5, pady=5)
entry_motor4 = tk.Entry(root)
entry_motor4.insert(0, "0")
entry_motor4.grid(row=11, column=1, padx=5, pady=5)

# Move Deg Button
move_deg_button = tk.Button(root, text="Move Deg", command=move_deg)
move_deg_button.grid(row=12, column=0, columnspan=3, pady=10)

# Input fields for kp values
tk.Label(root, text="zlac706 kpP:").grid(row=13, column=0, padx=5, pady=5)
entry_kpP = tk.Entry(root)
entry_kpP.insert(0, "3000")
entry_kpP.grid(row=13, column=1, padx=5, pady=5)

tk.Label(root, text="zlac706 kdP:").grid(row=14, column=0, padx=5, pady=5)
entry_kdP = tk.Entry(root)
entry_kdP.insert(0, "1")
entry_kdP.grid(row=14, column=1, padx=5, pady=5)

tk.Label(root, text="zlac706 kfP:").grid(row=15, column=0, padx=5, pady=5)
entry_kfP = tk.Entry(root)
entry_kfP.insert(0, "0")
entry_kfP.grid(row=15, column=1, padx=5, pady=5)

# Button to send kp values
send_kp_button = tk.Button(root, text="Send KP Values", command=send_kp_values)
send_kp_button.grid(row=16, column=0, columnspan=3, pady=10)

# Take Data Button (second lower position)
take_data_button = tk.Button(root, text="Take Data", command=take_data)
take_data_button.grid(row=17, column=0, columnspan=3, pady=10)

# Frame for error handling
frame_error = tk.Frame(root)
frame_error.grid(row=18, column=0, columnspan=3, pady=10)

read_error_button = tk.Button(frame_error, text="Read Error", command=read_error)
read_error_button.grid(row=0, column=0, padx=5, pady=10)

fault_reset_button = tk.Button(frame_error, text="Fault Reset", command=fault_reset)
fault_reset_button.grid(row=0, column=1, padx=5, pady=10)

root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
