"""
author: teukuzikrifatahillah@gmail.com
about this file: 
to test motor1

todo list:
1. sometimes, when the IK too near to x:100, it will now moving in straight line
2. inverse kinematics and forward kinematics for motor1
3. inner working of motor1
4. merge pada c++
5. pengaman joitn
6. 
"""

import can
import threading
import time
import struct
import tkinter as tk
import subprocess
import re
import math
import numpy as np
import signal

#ZLAC706 LIBRARY
FUNCTION_WRITE = 0xDA
FUNCTION_READ = 0xDC
FUNCTION_OK = 0XDB
FUNCTION_ERROR = 0XFF
FUNCTION_HEARTHBEAT = 0XFE

ADDR_OPERATION_CONTROL_WORD = 0x10
ADDR_WORKING_MODE = 0x19
ADDR_POSITION_MODE_CONTROL_WORD = 0x17
ADDR_ORIGIN_CONTROL_WORD = 0x18
ADDR_POSITION_MODE_TIME_OF_ACC_DEC = 0x12
ADDR_TRAPEZOID_SPEED = 0x14
ADDR_ALARM_RESET = 0x15
ADDR_TARGET_POSITION = 0x16
ADDR_SOFTWARE_BRAKE = 0x30
ADDR_TARGET_SPEED = 0x11
ADDR_SPEED_MODE_TIME_OF_ACC_DEC = 0x13
ADDR_POSITION_LOOP_KP = 0x20
ADDR_POSITION_LOOP_KD = 0x21
ADDR_POSITION_LOOP_FEED_FORWARD_GAIN_KI = 0x22
ADDR_SPEED_LOOP_PROPORTIONAL_GAIN_KP = 0x23
ADDR_SPEED_LOOP_KI = 0x24
ADDR_SPEED_LOOP_KD = 0x25
ADDR_CURRENT_LOOP_KP = 0x26
ADDR_CURRENT_LOOP_KI = 0x27
ADDR_BUS_VOLTAGE = 0xE1
ADDR_OUTPUT_CURRENT = 0xE2
ADDR_ALARM_STATUS = 0xE3
ADDR_REAL_TIME_SPEED = 0xE4
ADDR_POSITION_GIVEN = 0xE6
ADDR_POSITION_FEEDBACK = 0xE8
ADDR_TORQUE_MODE = 0x2D
ADDR_TARGET_TORQUE = 0x08
ADDR_DRIVER_ALARM = 0xEB

WORKING_MODE_SPEED = 0x2F
WORKING_MODE_POSITION = 0x3F
WORKING_MODE_TORQUE = 0x8F

POSITION_MODE_ABSOLUTE = 0x4F
POSITION_MODE_RELATIVE = 0x5F

ID1 = 0x01
bus = can.Bus(channel='can1', interface='socketcan')
def send_can_command(command):
    can_id, can_data = command.split('#')
    can_id = int(can_id, 16)
    can_data = bytes.fromhex(can_data)

    msg = can.Message(arbitration_id=can_id, data=can_data, is_extended_id=False)
    bus.send(msg)
    
def send_can1(id, function_code, internal_addr, data):
    send_can_command(f"{id:03X}#00{function_code:02X}00{internal_addr:02X}{data:08X}")


time.sleep(1)

def pulse_int_to_hex(value):
	if value < 0:
		value =  (1 << 32) + value
	return value

def pulse_hex_to_int(value):
	if value >= 0x80000000:
		value -= (1 << 32)
	return value
	
def pulse_to_degree(pulses):
	# 1 revolution = 10000 pulses, 1 revolution = 360 degrees
	return (360 / 10000) * pulses

def degree_to_pulse(degrees):
	# 1 revolution = 360 degrees, 1 revolution = 10000 pulses
	return (10000 / 360) * degrees

def degree_to_length(degrees):
	# 1 revolution = 360 degrees, 1 revolution = 90mm vertical travel
	return (90 / 360) * degrees

def length_to_degree(length):
	# 1 revolution = 90mm vertical travel, 1 revolution = 360 degrees
	return (360 / 90) * length

def merge_acc_dec(acc_time, dec_time):
	acc = int(acc_time/100)
	dec = int(dec_time/100)
	acc_dec = (acc << 8) | (dec & 0xFF)
	return acc_dec

def speed_to_hex(speed):
	value = int((speed * 8192) / 3000)
	return value
	
def hex_to_speed(hex_val):
	speed = int((hex_val/8192)*3000)
	
def on_closing():
	print("closing")
	shutdown()
	stop_timer()
	root.destroy()
        #thread.stop()

def initialize():
	
	acc_dec_time = merge_acc_dec(1000, 1000)
	target_position = pulse_int_to_hex(10000)
	target_position2 = pulse_int_to_hex(-10000)
	max_speed = speed_to_hex(100)
	int_target_position = pulse_hex_to_int(target_position)
	int_target_position2 = pulse_hex_to_int(target_position)
	
	print(f"tar1 = {target_position:08X} tarr = {target_position2:08X}")
	print(f"tar1 = {int_target_position} tarr = {int_target_position2}")
	
	
	#print(f"acc time = {acc_time:X} dec time = {dec_time:X} acc_dec = {acc_dec_time:X}")
	
	send_can1(ID1, FUNCTION_WRITE, ADDR_WORKING_MODE, WORKING_MODE_POSITION)
	send_can1(ID1, FUNCTION_WRITE, ADDR_POSITION_MODE_TIME_OF_ACC_DEC, acc_dec_time)
	send_can1(ID1, FUNCTION_WRITE, ADDR_TRAPEZOID_SPEED, max_speed)
	send_can1(ID1, FUNCTION_WRITE, ADDR_POSITION_MODE_CONTROL_WORD,POSITION_MODE_RELATIVE)
	send_can1(ID1, FUNCTION_WRITE, ADDR_OPERATION_CONTROL_WORD,0x1F)
	send_can1(ID1, FUNCTION_WRITE, ADDR_TARGET_POSITION, target_position2)
	#send_can_command(f"{ID1:X}#00DA00190000003F") #position mode
	#send_can_command(f"{ID1:X}#00DA001200000A0A") #acc dec
	#send_can_command(f"{ID1:X}#00DA001400000111") #max speed
	#send_can_command(f"{ID1:X}#00DA00170000005F") #relative position
	#send_can_command(f"{ID1:X}#00DA00100000001F") #motor ready
	#send_can_command(f"{ID1:X}#00DA00160001E240") #target position
	

def shutdown():
	send_can_command(f"{ID1:X}#00DA00100000000F")
	
def hold():
	send_can_command(f"{ID1:X}#00DA00300000001F")
	
def send_pid():
	try:
		kpp = int(entry_kpp.get())
		kdp = int(entry_kdp.get())
		kfp = int(entry_kfp.get())
		
		send_can1(ID1, FUNCTION_WRITE, ADDR_POSITION_LOOP_KP, kpp)
		send_can1(ID1, FUNCTION_WRITE, ADDR_POSITION_LOOP_KD, kdp)
		send_can1(ID1, FUNCTION_WRITE, ADDR_POSITION_LOOP_FEED_FORWARD_GAIN_KI, kfp)
		
	except ValueError:
		print(f"error value")
	
def travel():
	try:
		target_degree = float(entry_relative_position.get())
	except ValueError:
		print(f"error value target pos")
		
	acc_dec_time = merge_acc_dec(1000, 1000)
	
	target_pulse = int(degree_to_pulse(target_degree))
	target_position = pulse_int_to_hex(target_pulse)
	max_speed = speed_to_hex(100)
	int_target_position = pulse_hex_to_int(target_position)

	
	#print(f"acc time = {acc_time:X} dec time = {dec_time:X} acc_dec = {acc_dec_time:X}")
	
	send_can1(ID1, FUNCTION_WRITE, ADDR_WORKING_MODE, WORKING_MODE_POSITION)
	send_can1(ID1, FUNCTION_WRITE, ADDR_POSITION_MODE_TIME_OF_ACC_DEC, acc_dec_time)
	send_can1(ID1, FUNCTION_WRITE, ADDR_TRAPEZOID_SPEED, max_speed)
	send_can1(ID1, FUNCTION_WRITE, ADDR_POSITION_MODE_CONTROL_WORD,POSITION_MODE_RELATIVE)
	send_can1(ID1, FUNCTION_WRITE, ADDR_OPERATION_CONTROL_WORD,0x1F)
	send_can1(ID1, FUNCTION_WRITE, ADDR_TARGET_POSITION, target_position)

def signal_handler(sig, frame):
    on_closing()  # Panggil fungsi cleanup atau fungsi lain yang diinginkan
    bus.shutdown()
    exit(0)    # Keluar dari program dengan status 0

# Menangkap sinyal Ctrl+C (SIGINT)
signal.signal(signal.SIGINT, signal_handler)
   
#q3
root = tk.Tk()
root.title("Motor Control Panel")

initialize_button = tk.Button(root, text="initialize", command=initialize)
initialize_button.grid(row=0, column=0, columnspan=3, pady=10)

shutdown_button = tk.Button(root, text="shutdown", command=shutdown)
shutdown_button.grid(row=1, column=0, columnspan=3, pady=10)

hold_button = tk.Button(root, text="hold", command=hold)
hold_button.grid(row=2, column=0, columnspan=3, pady=10)

# Input fields for positions
tk.Label(root, text="Enter kpP:").grid(row=3, column=0, padx=5, pady=5)
entry_kpp = tk.Entry(root)
entry_kpp.insert(0, "1000")
entry_kpp.grid(row=3, column=1, padx=5, pady=5)

tk.Label(root, text="Enter kdP:").grid(row=4, column=0, padx=5, pady=5)
entry_kdp = tk.Entry(root)
entry_kdp.insert(0, "0")
entry_kdp.grid(row=4, column=1, padx=5, pady=5)

tk.Label(root, text="Enter kfP:").grid(row=5, column=0, padx=5, pady=5)
entry_kfp = tk.Entry(root)
entry_kfp.insert(0, "0")
entry_kfp.grid(row=5, column=1, padx=5, pady=5)

send_pid_button = tk.Button(root, text="send pid coef", command=send_pid)
send_pid_button.grid(row=6, column=0, columnspan=3, pady=10)

tk.Label(root, text="Enter target:").grid(row=7, column=0, padx=5, pady=5)
entry_relative_position = tk.Entry(root)
entry_relative_position.insert(0, "0")
entry_relative_position.grid(row=7, column=1, padx=5, pady=5)

travel_button = tk.Button(root, text="travel", command=travel)
travel_button.grid(row=8, column=0, columnspan=3, pady=10)

root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
bus.shutdown()
