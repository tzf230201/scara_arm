#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import time
import os
import tkinter as tk
from tkinter import simpledialog, messagebox
from moveit_msgs.msg import ExecuteTrajectoryActionGoal, ExecuteTrajectoryActionResult

# Conversion factor based on 360 degrees = 4000 pulses
current_d=[0,0,0]

DEGREES_TO_PULSES = 4000 / 360
accel_time= 100
decel_time= 100
max_speed_1= 60
max_speed_2= 2*max_speed_1
max_speed_3= 2*max_speed_2
joint1= 0
joint2= 0
joint3= 0
k_gain = 1000
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
    global k_gain
    try:
        accel_command_le = convert_to_little_endian(accel_time)
        decel_command_le = convert_to_little_endian(decel_time)
        speed_command_le_1 = convert_to_little_endian(max_speed_1)
        speed_command_le_2 = convert_to_little_endian(max_speed_2)
        speed_command_le_3 = convert_to_little_endian(max_speed_3)
        k_gain = convert_to_little_endian(k_gain)
        for node in ['602', '603', '604']:
            send_can_command(f"{node}#2B40600000000000")  # Initialization step 0
            time.sleep(0.5)
            send_can_command(f"{node}#2B40600006000000")  # Initialization step 1
            time.sleep(0.5)
            send_can_command(f"{node}#2B40600007000000")  # Initialization step 2
            time.sleep(0.5)
            send_can_command(f"{node}#2B4060000F000000")  # Initialization step 3
            time.sleep(0.5)
            send_can_command(f"{node}#2B4C2000{k_gain}")  # set pid k gain
            time.sleep(0.5)            
            send_can_command(f"{node}#23836000{accel_command_le}")  # Set acceleration time (index 6083)
            time.sleep(0.5)
            send_can_command(f"{node}#23846000{decel_command_le}")  # Set deceleration time (index 6084)
            time.sleep(0.5)
            if node == '602':
                speed_command_le= speed_command_le_1
            elif node == '603':
                speed_command_le= speed_command_le_2
            elif node == '604':
                speed_command_le= speed_command_le_3
            send_can_command(f"{node}#23816000{speed_command_le}")  # Set maximum speed (index 6081)
            time.sleep(0.5)
            send_can_command(f"{node}#2F60600001000000")  # Switch to position mode (index 6060)
            time.sleep(0.5)
        
        messagebox.showinfo("Initialize Motors", "Motors have been initialized and are ready for operation.")
    except ValueError:
        messagebox.showerror("Invalid Input", "Please enter valid numerical values for acceleration, deceleration, and speed.")

class JointStateSorter:
    def __init__(self):
        self.joint_states = []
        self.is_executing = False
        # Subscribers
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback, queue_size=1)
        self.execute_goal_sub = rospy.Subscriber('/execute_trajectory/goal', ExecuteTrajectoryActionGoal, self.execution_start_callback, queue_size=1)
        self.execute_result_sub = rospy.Subscriber('/execute_trajectory/result', ExecuteTrajectoryActionResult, self.execution_end_callback, queue_size=1)

    def joint_state_callback(self, msg):
        if self.is_executing:
            self.joint_states.append(msg)

    def execution_start_callback(self, msg):
        self.joint_states.clear()
        self.is_executing = True
        rospy.loginfo("Execution started")

    def execution_end_callback(self, msg):
        self.is_executing = False
        rospy.loginfo("Execution finished")

        # Sort joint states based on timestamp
        sorted_joint_states = sorted(self.joint_states, key=lambda msg: msg.header.stamp.to_sec())

        # Convert sorted joint states to an array
        joint_state_array = self.convert_to_array(sorted_joint_states)
        
        # Print or process the sorted joint states array
        rospy.loginfo("Sorted Joint States Array:")

        global current_d
        is_relative = False
        d1 = 1
        d2 = 1
        targets = joint_state_array
        i=1
        #print(target)
        for target in targets:
            target= target['position']
            print(target)
            print(len(targets))
            current_d = target
            degrees1 = (target[1] / 3.14 * 180) * 5
            degrees2 = (target[2] / 3.14 * 180) * 5 + d1 * degrees1
            degrees3 = (target[3] / 3.14 * 180) * 5 + d2 * degrees2
            try:
                degrees1 = float(degrees1)
                degrees2 = float(degrees2)
                degrees3 = -1.0 * float(degrees3)

                pulses1 = degrees_to_pulses(degrees1)
                pulses2 = degrees_to_pulses(degrees2)
                pulses3 = degrees_to_pulses(degrees3)

                pulse_command1_le = convert_to_little_endian_signed(pulses1)
                pulse_command2_le = convert_to_little_endian_signed(pulses2)
                pulse_command3_le = convert_to_little_endian_signed(pulses3)

                if is_relative:
                    start_command = "4F"
                    execute_command = "5F"
                else:
                    start_command = "0F"
                    execute_command = "1F"
                interupt_1= "2f"
                interupt_2= "3f"
                for node, pulse_command_le in zip(['602', '603', '604'], [pulse_command1_le, pulse_command2_le, pulse_command3_le]):
                    send_can_command(f"{node}#2B406000{start_command}000000")  # Prepare for motion
                    time.sleep(0.001)
                    send_can_command(f"{node}#237A6000{pulse_command_le}")  # Set target position using index 607A
                    time.sleep(0.001)
                for node in ['602', '603', '604']:
                    send_can_command(f"{node}#2B406000{execute_command}000000")  # Execute motion
                    time.sleep(0.001)
                
            except ValueError:
                messagebox.showerror("Invalid Input", "Please enter valid numbers for angles.")
            i=i+1
            
        joint_state_array=[]
	
    def convert_to_array(self, sorted_joint_states):
        joint_state_array = []
        for state in sorted_joint_states:
            joint_state_dict = {'position': state.position}
            joint_state_array.append(joint_state_dict)
        return joint_state_array
        
def shutdown_motors():
    for node in ['602', '603', '604']:
        send_can_command(f"{node}#2B40600000000000")  # Shutdown command
        time.sleep(0.5)
    messagebox.showinfo("Shutdown Motors", "Motors have been shut down properly.")

if __name__ == '__main__':
    initialize_motors()
    rospy.init_node('joint_state_sorter')
    sorter = JointStateSorter()
    rospy.spin()
