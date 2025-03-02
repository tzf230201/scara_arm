#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import time
import os
import tkinter as tk
from tkinter import simpledialog, messagebox
import pickle

DEGREES_TO_PULSES = 4000 / 360
accel_time= 10
decel_time= 10
I_speed= 0

#f = 'last_position'
#with open(f, 'rb') as file:
#    last = pickle.load(file)

degrees11= 0
degrees21= 0
degrees31= 0
dt= 0.1

def send_can_command(command):
    print(f"Sending CAN command: {command}")
    os.system(f'cansend can0 {command}')

def degrees_to_pulses(degrees):
    global DEGREES_TO_PULSES
    return int(degrees * DEGREES_TO_PULSES)

def convert_to_little_endian(value):
    hex_value = f"{value:08X}"
    little_endian_value = ''.join([hex_value[i:i+2] for i in range(6, -1, -2)])
    return little_endian_value

def convert_to_little_endian_signed(value):
    if value < 0:
        value = (1 << 32) + value
    hex_value = f"{value:08X}"
    little_endian_value = ''.join([hex_value[i:i+2] for i in range(6, -1, -2)])
    return little_endian_value

def initialize_motors():
    global accel_time
    global decel_time
    global I_speed
    try:
        accel_command_le = convert_to_little_endian(accel_time)
        decel_command_le = convert_to_little_endian(decel_time)
        speed_command_le = convert_to_little_endian(I_speed)
        for node in ['602', '603', '604']:
            send_can_command(f"{node}#2B40600000000000")  # Initialization step 0
            time.sleep(0.5)
            send_can_command(f"{node}#2B40600006000000")  # Initialization step 1
            time.sleep(0.5)
            send_can_command(f"{node}#2B40600007000000")  # Initialization step 2
            time.sleep(0.5)
            send_can_command(f"{node}#2B4060000F000000")  # Initialization step 3
            time.sleep(0.5)
            send_can_command(f"{node}#23836000{accel_command_le}")  # Set acceleration time (index 6083)
            time.sleep(0.5)
            send_can_command(f"{node}#23846000{decel_command_le}")  # Set deceleration time (index 6084)
            time.sleep(0.5)
            send_can_command(f"{node}#23FF6000{speed_command_le}")  # Set initial speed
            time.sleep(0.5)
            send_can_command(f"{node}#2F60600003000000")  # Switch to Speed Mode 
            time.sleep(0.5)

        messagebox.showinfo("Initialize Motors", "Motors have been initialized and are ready for operation.")
    except ValueError:
        messagebox.showerror("Invalid Input", "Please enter valid numerical values for acceleration, deceleration, and speed.")

def joint_state_callback(msg):
    global degrees11, degrees21, degrees31, dt

    degrees12 = (msg.position[1] / 3.14 * 180) * 5
    degrees22 = (msg.position[2] / 3.14 * 180) * 5 + degrees12
    degrees32 = (msg.position[3] / 3.14 * 180) * 5 + degrees22

    speed_1= (degrees12-degrees11)/dt
    speed_2= (degrees22-degrees21)/dt
    speed_3= (degrees32-degrees31)/dt

    speed_1= (1/6) * speed_1 #convert to rpm
    speed_2= (1/6) * speed_2 #convert to rpm
    speed_3= (1/6) * speed_3 #convert to rpm
    try:
        speed_1 = float(speed_1)
        speed_2 = float(speed_2)
        speed_3 = -1.0 * float(speed_3)

        pulses1 = degrees_to_pulses(speed_1)
        pulses2 = degrees_to_pulses(speed_2)
        pulses3 = degrees_to_pulses(speed_3)

        pulse_command1_le = convert_to_little_endian_signed(pulses1)
        pulse_command2_le = convert_to_little_endian_signed(pulses2)
        pulse_command3_le = convert_to_little_endian_signed(pulses3)


        send_can_command(f"{'602'}#23FF6000{pulse_command1_le}")  # Set target speed using index
        time.sleep(0.001)
        send_can_command(f"{'603'}#23FF6000{pulse_command2_le}")  # Set target speed using index 
        time.sleep(0.001)
        send_can_command(f"{'604'}#23FF6000{pulse_command3_le}")  # Set target speed using index 
        time.sleep(0.001)

    except ValueError:
        messagebox.showerror("Invalid Input", "Please enter valid numbers for angles.")

    degrees11 = degrees12
    degrees21 = degrees22
    degrees31 = degrees32

    print(speed_1, speed_2, speed_3)
    
def joint_state_listener():
    rospy.init_node('joint_state_listener', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, joint_state_callback, queue_size=1)
    rate = rospy.Rate(10)
    rospy.spin()

def shutdown_motors():
    for node in ['602', '603', '604']:
        send_can_command(f"{node}#2B40600000000000")  # Shutdown command
        time.sleep(0.1)
    messagebox.showinfo("Shutdown Motors", "Motors have been shut down properly.")

if __name__ == '__main__':
    try:
        initialize_motors()
        joint_state_listener()
        f = 'last_position'
        with open(f, 'wb') as file:
            pickle.dump([degrees11, degrees21, degrees31], file)

    except rospy.ROSInterruptException:
        shutdown_motors()
