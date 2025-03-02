"""
author: teukuzikrifatahillah@gmail.com
about this file: 
the source code in demo video without motor1
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

# ZLIS42C LIBRARY
SET_1_BYTE = 0x2F
SET_2_BYTE = 0x2B
SET_3_BYTE = 0x27
SET_4_BYTE = 0x23
SET_OK = 0x60
READ_1_BYTE = 0x4F
READ_2_BYTE = 0x4B
READ_3_BYTE = 0x47
READ_4_BYTE = 0x43
SET_ERROR = 0x80

#ROM : need to reset power to enable
INDEX_POSITION_KP = 0x204B
INDEX_INITIAL_SPEED = 0x200E


#RAM
INDEX_ERROR_CODE = 0x603F
INDEX_CONTROL_WORD = 0x6040
INDEX_STATUS_WORD = 0x6041
INDEX_MODE_OF_OPERATION = 0x6060
INDEX_POSITION_ACTUAL_VALUE = 0x6064
INDEX_VELOCITY_ACTUAL_VALUE = 0x606C
INDEX_TARGET_POSITION = 0x607A
INDEX_PROFILE_VELOCITY = 0x6081
INDEX_END_VELOCITY = 0x6082
INDEX_PROFILE_ACCELERATION = 0x6083
INDEX_PROFILE_DECELERATION = 0x6084
INDEX_TARGET_VELOCITY = 0x60FF

ID2 = 0x602
ID3 = 0x603
ID4 = 0x604

response_id_map = {
    ID2: 0x582,
    ID3: 0x583,
    ID4: 0x584
}

# Use a dictionary to store current pulse values for different IDs
cur_pulse_dict = {}
is_data_exist = threading.Event()

def read_can_bus():
    def run_can_read():
        bus = can.Bus(channel='can0', interface='socketcan')

        try:
            while True:
                message = bus.recv(0.5)  # Wait up to 0.5 seconds for a message
                if message:
                    can_id = message.arbitration_id
                    # Block CAN IDs 602, 603, and 604
                    if can_id in {ID2, ID3, ID4}:
                        continue
                    
                    # Store the CAN data based on ID
                    data_receive = f"{can_id:X}#"
                    data_receive += "".join(f"{byte:02X}" for byte in message.data)
                    
                    # Update the pulse value for the CAN ID
                    cur_pulse_dict[can_id] = data_receive
                    
                    # print(f"Received: {data_receive}")  # For debugging purposes
                    is_data_exist.set()  # Signal that data was received

        except KeyboardInterrupt:
            print("Stopped reading CAN bus.")

    thread = threading.Thread(target=run_can_read)
    thread.daemon = True
    thread.start()
    

# Run the CAN bus reader
read_can_bus()

DEGREES_TO_PULSES = 4000 / 360
def int_to_little_endian(value):

    # Convert integer to 4-byte little-endian representation
    little_endian_bytes = value.to_bytes(4, byteorder='little', signed=True)
    
    # Convert bytes to a hexadecimal string and format
    little_endian_hex = ''.join(f'{byte:02X}' for byte in little_endian_bytes)
    
    return little_endian_hex
    
def degrees_to_pulses(degrees):
    return int(degrees * DEGREES_TO_PULSES)

def pulses_to_degrees(pulses):
    return float(pulses / (10000 / 360))
        
def forward_kinematics(cur_deg2, cur_deg3, cur_deg4):
	# max area
    	l2 = 137.0
    	l3 = 121.0
    	l4 = 56.82
    	offset2 = -96.5
    	offset3 = 134.0
    	offset4 = -52.5
    	#ration joint = 5:1
    	theta2_rad = math.radians((cur_deg2/5) + offset2)
    	theta3_rad = math.radians((cur_deg3/5) + offset3 - (cur_deg2/5))
    	x2 = l2 * math.cos(theta2_rad)
    	y2 = l2 * math.sin(theta2_rad)
    	x3 = l3 * math.cos(theta2_rad + theta3_rad)
    	y3 = l3 * math.sin(theta2_rad + theta3_rad)
    	x = x2 + x3
    	y = y2 + y3
    	c = (cur_deg4/5) + offset4 #- (cur_deg3/5)
    	return x, y, c

def inverse_kinematics(x, y, c):
    # max area
    l2 = 137.0
    l3 = 121.0
    l4 = 56.82
    offset2 = -96.5
    offset3 = 134
    offset4 = -52.5
    #ration joint = 5:1
    
    distance = math.sqrt(x**2 + y**2)
    
    if distance > (l2 + l3):
        raise ValueError("out of boundary")
    
    # joint3
    cos_theta3 = (x**2 + y**2 - l2**2 - l3**2) / (2 * l2 * l3)
    theta3 = math.acos(cos_theta3)  #angle in radian
    
    # joint2
    k1 = l2 + l3 * cos_theta3
    k2 = l3 * math.sin(theta3)
    theta2 = math.atan2(y, x) - math.atan2(k2, k1)
    
    # rad to deg
    theta2 = math.degrees(theta2)
    theta3 = math.degrees(theta3)
    
    joint2 = (theta2-offset2)*5;
    joint3 = (theta3-offset3)*5 + joint2;
    joint4 = (c-offset4)*5# + joint3;
    
    if joint2 > (178 * 5):
    	joint2 = 178 * 5
    	raise ValueError("out of joint2 max limit")
    elif joint2 < 0:
    	joint2 = 0
    	raise ValueError("out of joint2 min limit")
    if joint3 > (0 + joint2):
    	joint3 = (0 + joint2)
    	raise ValueError("out of joint3 max limit")
    elif joint3 < ((-135 * 5) +joint2):
    	joint3 = (-135 * 5) + joint2
    	raise ValueError("out of joint2 min limit")
    if joint4 > ((196 * 5) + joint3):
    	joint4 = (196 * 5) + joint3
    	raise ValueError("out of joint4 max limit")
    elif joint4 < (0 + joint3):
    	joint4 = (0 + joint3)
    	raise ValueError("out of joint2 min limit")

    return joint2, joint3, joint4
bus = can.Bus(channel='can0', interface='socketcan')

def format_data(can_id, set_length, index, data):
    # Convert ID and length to hexadecimal strings
    can_id_hex = format(can_id, '03X')  # Ensure ID is 3 characters long for proper formatting
    set_length_hex = format(set_length, '02X')  # Set length as 2 characters long

    # Convert index to little-endian and then to hex
    index_le = struct.pack('<H', index)
    index_hex = ''.join(f"{byte:02X}" for byte in index_le)

    # Convert data to little-endian 4-byte format and then to hex
    if data < 0:
        # Ensure data is treated as an unsigned 32-bit integer if negative
        data = (data + 0x100000000) % 0x100000000  # Wrap around for 32-bit unsigned
    data_le = struct.pack('<I', data)
    data_hex = ''.join(f"{byte:02X}" for byte in data_le)

    # Format the message
    message = f"{can_id_hex}#{set_length_hex}{index_hex}00{data_hex}"
    return message
    

def send_can_command(command):
    can_id, can_data = command.split('#')
    can_id = int(can_id, 16)
    can_data = bytes.fromhex(can_data)

    msg = can.Message(arbitration_id=can_id, data=can_data, is_extended_id=False)
    bus.send(msg)

def can_tx(request_id, set_length, index, data):
	command = format_data(request_id, set_length, index, data)
	is_data_exist.clear()
	send_can_command(command)
	is_data_exist.wait()
	response_id = response_id_map.get(request_id)
	data_receive = cur_pulse_dict[response_id]
	can_id, hex_data = data_receive.split('#')
	data_bytes = bytes.fromhex(hex_data)
	SET_OK = 0x60
	SET_ERROR = 0x80
	first_byte = data_bytes[0]
	if first_byte == SET_OK:
		return 0x00
	elif first_byte == SET_ERROR:
		return 0x01
	else:
		if len(data_bytes) >= 4:
			last_4_bytes = data_bytes[-4:]
			# Unpack data as little-endian (assuming 4 bytes for 32-bit value)
			data_int = struct.unpack('<i', last_4_bytes)[0]
		else:
			data_int = 0;
		return data_int

#ret = can_tx(ID3, READ_4_BYTE, INDEX_POSITION_ACTUAL_VALUE, 0)
#print(f"{ret}")

def read_servo_angle(request_id):
	pulse = can_tx(request_id, READ_4_BYTE, INDEX_POSITION_ACTUAL_VALUE, 0)
	joint = pulses_to_degrees(pulse)
	if (request_id != ID4):
		joint = joint * -1.0
	return joint

def read_servo_pulse(request_id):
	pulse = can_tx(request_id, READ_4_BYTE, INDEX_POSITION_ACTUAL_VALUE, 0)
	if (request_id != ID4):
		pulse = pulse * -1.0
	return pulse

def read_servo_speed(request_id):
	vel = can_tx(request_id, READ_4_BYTE, INDEX_VELOCITY_ACTUAL_VALUE, 0)
	if (request_id != ID4):
		vel = vel * -1.0
	return vel
#qt
def is_target_reached(request_id):
	ret = can_tx(request_id, READ_2_BYTE, INDEX_STATUS_WORD, 0)

	# Ensure ret is an integer if it's not already
	#halt = (ret >> 8) & 1
	target_reached = (ret >> 10) & 1

	#print(f"ret halt : {halt} tr : {target_reached}")

	return target_reached

def 	is_targets_reached():
	ret2 = is_target_reached(ID2)
	ret3 = is_target_reached(ID3)
	ret4 = is_target_reached(ID4)

	return ret2, ret3, ret4

def read_present_position():
    angles = []
    for request_id in [ID2, ID3, ID4]:
        response_id = response_id_map.get(request_id)
        if response_id is None:
            raise ValueError(f"No response ID mapping found for request ID {request_id}")

        is_data_exist.clear()
        send_can_command(f"{request_id:X}#4364600000000000")  # Read encoder 32bit 0x6064
        is_data_exist.wait()

        if response_id in cur_pulse_dict:
            data_receive = cur_pulse_dict[response_id]
            #print(f"Received: {data_receive}")

            # Extract the hex data (remove 'XX#' and convert to bytes)
            data_hex = data_receive.split('#')[1]
            data_bytes = bytes.fromhex(data_hex)

            # Pick the last 4 bytes
            if len(data_bytes) >= 4:
                last_4_bytes = data_bytes[-4:]

                # Unpack data as little-endian (assuming 4 bytes for 32-bit value)
                data_int = struct.unpack('<i', last_4_bytes)[0]  # '<I' is format for little-endian unsigned int
                angles.append(pulses_to_degrees(data_int))
            else:
                angles.append(0)
        else:
            angles.append(0)
    
    angles[0] = angles[0] * -1.0
    angles[1] = angles[1] * -1.0
    
   
    return angles



#qr
def read_joint():
	start_time = time.time()
	# Automatically determine the response ID
	cur_speed2 = read_servo_speed(ID2)
	cur_speed3 = read_servo_speed(ID3)
	cur_speed4 = read_servo_speed(ID4)
	cur_joint2, cur_joint3, cur_joint4 = read_present_position()   
	end_time = time.time()
	delta_time = (end_time - start_time) * 1000
	print(f"\033[94mcur_joint: ({cur_joint2:.2f}, {cur_joint3:.2f}, {cur_joint4:.2f}) dt: {delta_time:.2f} ms\033[0m")
	print(f"\033[93mcur_speed: ({cur_speed2:.2f}, {cur_speed3:.2f}, {cur_speed4:.2f}) dt: {delta_time:.2f} ms\033[0m")
	x, y, c = forward_kinematics(cur_joint2, cur_joint3, cur_joint4)
	ret2 = is_target_reached(ID2)
	ret3 = is_target_reached(ID3)
	ret4 = is_target_reached(ID4)
	print(f"fk x, y, c = ({x:.1f}, {y:.1f}, {c:.1f})")
	print(f"ret = ({ret2}, {ret3}, {ret4})")

def init_motor(accel_time, decel_time, speed2, speed3, speed4):
	for id in [ID2, ID3, ID4]:
		can_tx(id, SET_2_BYTE, INDEX_CONTROL_WORD, 0x00) #init 0
		can_tx(id, SET_4_BYTE, INDEX_PROFILE_ACCELERATION, accel_time) #profile accel
		can_tx(id, SET_4_BYTE, INDEX_PROFILE_DECELERATION, decel_time) #profile decel
		can_tx(id, SET_1_BYTE, INDEX_MODE_OF_OPERATION, 0x01) #position mode
	
	for id, speed in zip([ID2, ID3, ID4], [speed2, speed3, speed4]):
		can_tx(id, SET_4_BYTE, INDEX_PROFILE_VELOCITY, speed) #profile speed

	for id in [ID2, ID3, ID4]:
		can_tx(id, SET_2_BYTE, INDEX_CONTROL_WORD, 0x06) #init 1
		can_tx(id, SET_2_BYTE, INDEX_CONTROL_WORD, 0x07) #init 2
		#can_tx(id, SET_2_BYTE, INDEX_CONTROL_WORD, 0x4F) #init 2
    
def initialize_motors():
    try:
        #accel_time = int(entry_accel_time.get())
        #decel_time = int(entry_decel_time.get())
        #max_speed = int(entry_max_speed.get())
        
        duration_time = int(entry_accel_time.get())
        accel_time = int(duration_time/2)
        decel_time = int(duration_time/2)
        max_speed = int(entry_max_speed.get())

        init_motor(accel_time, decel_time, max_speed, max_speed, max_speed)
        print(f"set accel : {accel_time} ms")
        print(f"set decel : {decel_time} ms")
        print(f"set speed : {max_speed} r/min")
        print("Motors Ready")
    except ValueError:
    	print("Please enter valid numerical values for acceleration, deceleration, and speed.")

        
def shutdown_motors():
    for node in ['602', '603', '604']:
        send_can_command(f"{node}#2B40600000000000")  # Shutdown command
        time.sleep(0.05)
    print("shutdown complete")

def relative_command(cur_joint2, cur_joint3, cur_joint4, tar_joint2, tar_joint3, tar_joint4):
	try:
	    	#limit2 = 0 to 178 degree
	    	#limit3 = -135 to 0 degree
	    	#limit4 = 0 to 196 degree 
	    	if tar_joint2 > (178 * 5):
	    		tar_joint2 = 178 * 5
	    		raise ValueError("out of boundary")
	    	elif tar_joint2 < 0:
	    		tar_joint2 = 0
	    		raise ValueError("out of boundary")
	    		
	    	if tar_joint3 > (0 + tar_joint2):
	    		tar_joint3 = (0 + tar_joint2)
	    		raise ValueError("out of boundary")
	    	elif tar_joint3 < ((-135 * 5) + tar_joint2):
	    		tar_joint3 = (-135 * 5) + tar_joint2
	    		raise ValueError("out of boundary")
	    		
	    	if tar_joint4 > ((196 * 5)+tar_joint3):
	    		tar_joint4 = (196 * 5) + tar_joint3
	    		raise ValueError("out of boundary")
	    	elif tar_joint4 < (0 + tar_joint3):
	    		tar_joint4 = 0 + tar_joint3
	    		raise ValueError("out of boundary")
	    	
	    	joint2 = tar_joint2 - cur_joint2
	    	joint3 = tar_joint3 - cur_joint3
	    	joint4 = tar_joint4 - cur_joint4
	    	
	    	#print(f"cur_joint : {cur_joint2:.2f}, {cur_joint3:.2f}, {cur_joint3:.2f}")
	    	#print(f"tar_joint : {tar_joint2:.2f}, {tar_joint3:.2f}, {tar_joint3:.2f}")
	    	#print(f"del_joint : {joint2:.2f}, {joint3:.2f}, {joint3:.2f}")
	    	
	    	degree2 = float(joint2)
	    	degree3 = float(joint3)
	    	degree4 = -1.0*float(joint4)
	    	
	    	pulse2 = degrees_to_pulses(degree2)
	    	pulse3 = degrees_to_pulses(degree3)
	    	pulse4 = degrees_to_pulses(degree4)
	    	
	    	start_command = 0x4F
	    	execute_command = 0x5F
	    	
	    	#pulse2 = int_to_little_endian(pulse2)
	    	#pulse3 = int_to_little_endian(pulse3)
	    	#pulse4 = int_to_little_endian(pulse4)
	    	
	    	for id in [ID2, ID3, ID4]:
	    		#send_can_command(f"{id:03X}#2B4060004F000000")
	    		can_tx(id, SET_2_BYTE, INDEX_CONTROL_WORD, start_command)
	    	
	    	for id, pulse in zip([ID2, ID3, ID4], [pulse2, pulse3, pulse4]):
	    		#send_can_command(f"{id:03X}#237A6000{pulse}")  # Set target position using index 607A
	    		can_tx(id, SET_4_BYTE, INDEX_TARGET_POSITION, pulse)
	    	
	    	for id in [ID2, ID3, ID4]:
	    		#send_can_command(f"{id:03X}#2B4060005F000000")
	    		can_tx(id, SET_2_BYTE, INDEX_CONTROL_WORD, execute_command)
	    	
	    	

		#messagebox.showinfo("Send Position Command", "Position commands sent to all motors.")
	except ValueError:
		print("Please enter valid numbers for angles.")


def send_relative_command():
    try:
        tar_joint2 = float(entry_motor1.get())
        tar_joint3 = float(entry_motor2.get())
        tar_joint4 = float(entry_motor3.get())
        
        start_time = time.time()
  
	
        
        cur_joint2, cur_joint3, cur_joint4 = read_present_position()
        relative_command(cur_joint2, cur_joint3, cur_joint4, tar_joint2, tar_joint3, tar_joint4)
        
        end_time = time.time()
        delta_time = (end_time - start_time) * 1000
        print(f"\033[94mtime cost : {delta_time:.2f} ms\033[0m")
#
#        time.sleep(7)
#        read_joint()
    except ValueError:
        print("Please enter valid numbers for angles.")


def generate_trajectory(start, end, steps):
    x_points = np.linspace(start[0], end[0], steps)
    y_points = np.linspace(start[1], end[1], steps)
    c_points = np.linspace(start[2], end[2], steps)
    trajectory = list(zip(x_points, y_points, c_points))
    return trajectory

def plot_trajectory(trajectory):
	cnt = 0
	previous_time = 0
	interval = 0.02
	for (x, y, c) in trajectory:
		start_time = time.time()
		try:
			current_time = time.time()
			delta_time = current_time - previous_time
			previous_time = current_time
			
			cur_joint2, cur_joint3, cur_joint4 = read_present_position()       
			tar_joint2, tar_joint3, tar_joint4 = inverse_kinematics(x, y, c)
			
			#print(f"Tar xyc {cnt}: {x:.2f}, {y:.2f}, {c:.2f} (dt: {delta_time:.2f} s)")
			#print(f"\033[94mcur: ({cur_joint2:.2f}, {cur_joint3:.2f}, {cur_joint4:.2f})\033[0m")
			print(f"Tar: ({tar_joint2:.2f}, {tar_joint3:.2f}, {tar_joint4:.2f}) (dt: {delta_time:.2f} s)")

			relative_command(cur_joint2, cur_joint3, cur_joint4, tar_joint2, tar_joint3, tar_joint4)
			elapsed_time = time.time() - start_time
			sleep_time = max(0, interval - elapsed_time)
			time.sleep(sleep_time)
		except ValueError as e:
			print(f"Skipping point ({x:.1f}, {y:.1f}): {e}")
timer = None
interval = 0.02
def time_profile(tar_joint2, tar_joint3, tar_joint4, travel_time):
	global interval
	cur_joint2, cur_joint3, cur_joint4 = read_present_position()   
	delta_joint2 = tar_joint2 - cur_joint2
	delta_joint3 = tar_joint3 - cur_joint3
	delta_joint4 = tar_joint4 - cur_joint4
	v2 = delta_joint2 / (travel_time / 1000)
	v3 = delta_joint3 / (travel_time / 1000)
	v4 = delta_joint4 / (travel_time / 1000)
	vp2 = v2 * interval
	vp3 = v3 * interval
	vp4 = v4 * interval
	max_cnt = int( (travel_time / 1000) / interval)
	print(f"{delta_joint2:.4f} {delta_joint3:.4f} {delta_joint4:.4f} ")
	print(f"{v2:.4f} {v3:.4f} {v4:.4f} ")
	print(f"{vp2:.4f} {vp3:.4f} {vp4:.4f} ")
	print(f"max cnt {max_cnt}")
	
	tarj2 = cur_joint2
	tarj3 = cur_joint3
	tarj4 = cur_joint4
	
	prev_time = time.time()
	
	for i in range(0, max_cnt):
		start_time = time.time()

		cur_time = time.time()
		delta_time = cur_time - prev_time
		prev_time = cur_time
		#print(f"dt {i} : {delta_time * 1000:.2f} ms")
		# Call the function and measure execution time
		tarj2 = tarj2 + vp2
		tarj3 = tarj3 + vp3
		tarj4 = tarj4 + vp4
		curj2, curj3, curj4 = read_present_position()
		#print(f"\033[94mcur: ({curj2:.2f}, {curj3:.2f}, {curj4:.2f})\033[0m")
		print(f"Tar: ({tarj2:.2f}, {tarj3:.2f}, {tarj4:.2f}) dt {i} : {delta_time * 1000:.2f} ms")
		relative_command(curj2, curj3, curj4, tarj2, tarj3, tarj4)

		# Calculate elapsed time
		elapsed_time = time.time() - start_time

		# Print elapsed time in milliseconds
		#print(f"calculation: {elapsed_time * 1000:.2f} ms")

		# Calculate sleep time to maintain the desired interval
		sleep_time = max(0, interval - elapsed_time)

		# Print the intended sleep time in milliseconds
		#print(f"Sleeping for: {sleep_time * 1000:.2f} ms")

		# Sleep to maintain the interval
		time.sleep(sleep_time)
	
	"""
	half_delta2 = (delta_joint2 / 2)
	half_delta3 = (delta_joint3 / 2)
	half_delta4 = (delta_joint4 / 2)
	
	
	
	if cnt <= 90:
		pos = half_delta2 * ( 1 - sin(toradian(90 + cnt))
	else:
		pos = half_delta + half_delta * sin(toradian(90 - cnt)
		
	"""
	


#time_profile(100.0, -100.0, 100.0, 7000)
#while True:
#	time.sleep(1)

#import time
gain2 = 0.0
gain3 = 0.0
gain4 = 0.0
tar_joint2 = 0.0
tar_joint3 = 0.0
tar_joint4 = 0.0

def put_val():
	"""
	global gain2
	global gain3
	global gain4
	gain2 = float(entry_motor1.get())
	gain3 = float(entry_motor2.get())
	gain4 = float(entry_motor3.get())
	"""
	tar_joint2 = float(entry_motor1.get())
	tar_joint3 = float(entry_motor2.get())
	tar_joint4 = float(entry_motor3.get())
	time_profile(tar_joint2, tar_joint3, tar_joint4, 7000)
	
	

def periodic_function():
	global tar_joint2
	global tar_joint3
	global tar_joint4
	global gain2
	global gain3
	global gain4
	tar_joint2 = tar_joint2 + gain2
	tar_joint3 = tar_joint3 + gain3
	tar_joint4 = tar_joint4 + gain4
	cur_joint2, cur_joint3, cur_joint4 = read_present_position()   
	#print(f"\033[94mcur: ({cur_joint2:.2f}, {cur_joint3:.2f}, {cur_joint4:.2f})\033[0m")
	#print(f"Tar: ({tar_joint2:.2f}, {tar_joint3:.2f}, {tar_joint4:.2f}) (dt: {delta_time:.2f} s)")

	relative_command(cur_joint2, cur_joint3, cur_joint4, tar_joint2, tar_joint3, tar_joint4)
	
def periodic_function2():
	global tar_joint2
	global tar_joint3
	global tar_joint4
	global gain2
	global gain3
	global gain4
	tar_joint2 = tar_joint2 + gain2
	tar_joint3 = tar_joint3 + gain3
	tar_joint4 = tar_joint4 + gain4
	cur_joint2, cur_joint3, cur_joint4 = read_present_position()   
	#print(f"\033[94mcur: ({cur_joint2:.2f}, {cur_joint3:.2f}, {cur_joint4:.2f})\033[0m")
	#print(f"Tar: ({tar_joint2:.2f}, {tar_joint3:.2f}, {tar_joint4:.2f}) (dt: {delta_time:.2f} s)")

	relative_command(cur_joint2, cur_joint3, cur_joint4, tar_joint2, tar_joint3, tar_joint4)
	
	ret2, ret3, ret4 = is_targets_reached()
	cur_joint2, cur_joint3, cur_joint4 = read_present_position()
	cur_speed2 = read_servo_speed(ID2)
	cur_speed3 = read_servo_speed(ID3)
	cur_speed4 = read_servo_speed(ID4)
	if (ret2 & ret3 & ret4):
		stop_timer()
	
def run_periodically():
	start_time = time.time()
	global interval
	global timer
	
	cur_time = time.time()
	delta_time = cur_time - prev_time
	prev_time = cur_time
	print(f"dt: {delta_time * 1000:.2f} ms")
	# Call the function and measure execution time
	periodic_function2()

	# Calculate elapsed time
	elapsed_time = time.time() - start_time

	# Print elapsed time in milliseconds
	print(f"calculation: {elapsed_time * 1000:.2f} ms")

	# Calculate sleep time to maintain the desired interval
	sleep_time = max(0, interval - elapsed_time)

	# Print the intended sleep time in milliseconds
	print(f"Sleeping for: {sleep_time * 1000:.2f} ms")

	# Sleep to maintain the interval
	#time.sleep(sleep_time)
	start_timer()


def start_timer():
	global timer
	global interval
	#if timer is None or not timer.is_alive():
	#print("Starting a new timer.")
	timer = threading.Timer(interval, run_periodically)
	timer.start()
	#else:
	#print("Timer is already running, not starting a new one.")

def stop_timer():
	global timer
	if timer is not None:
		timer.cancel()
		timer = None
		print("Timer stopped.")
	else:
		print("No timer is running to stop.")

    
def move():	
	target_x = float(entry_x_pos.get())
	target_y = float(entry_y_pos.get())
	target_c = float(entry_claw_angle.get())
	
	cur_joint2, cur_joint3, cur_joint4 = read_present_position()
	
	start_point = forward_kinematics(cur_joint2, cur_joint3, cur_joint4)
	end_point = (target_x, target_y, target_c)   # Ending position (x, y, c)
	steps = 180  # Number of steps in the trajectory
	
	print(f"fk (x, y, c) = ({start_point})")
	print(f"tar (x, y, c) = ({end_point})")
	
	
	trajectory = generate_trajectory(start_point, end_point, steps)
	
	start_time = time.time()
	plot_trajectory(trajectory)
	end_time = time.time()
	delta_time = end_time - start_time
	print(f"total time: {delta_time:.2f} s)")
	
def move2():	
	target_x = float(entry_x_pos.get())
	target_y = float(entry_y_pos.get())
	target_c = float(entry_claw_angle.get())
	
	cur_joint2, cur_joint3, cur_joint4 = read_present_position()
	tar_joint2, tar_joint3, tar_joint4 = inverse_kinematics(target_x, target_y, target_c)
	
	#start_point = forward_kinematics(cur_joint2, cur_joint3, cur_joint4)
	#end_point = (target_x, target_y, target_c)   # Ending position (x, y, c)
	#steps = 180  # Number of steps in the trajectory
	#print(f"fk (x, y, c) = ({start_point})")
	#print(f"tar (x, y, c) = ({end_point})")
	#trajectory = generate_trajectory(start_point, end_point, steps)
	
	start_time = time.time()
	#plot_trajectory(trajectory)
	relative_command(cur_joint2, cur_joint3, cur_joint4, tar_joint2, tar_joint3, tar_joint4)
	
	end_time = time.time()
	delta_time = end_time - start_time
	print(f"total time: {delta_time:.2f} s)")
	#time.sleep(7)
	#read_joint()
	
def move_xyc(target_x, target_y, target_c):	
	#target_x = float(entry_x_pos.get())
	#target_y = float(entry_y_pos.get())
	#target_c = float(entry_claw_angle.get())
	
	cur_joint2, cur_joint3, cur_joint4 = read_present_position()
	tar_joint2, tar_joint3, tar_joint4 = inverse_kinematics(target_x, target_y, target_c)
	
	#start_point = forward_kinematics(cur_joint2, cur_joint3, cur_joint4)
	#end_point = (target_x, target_y, target_c)   # Ending position (x, y, c)
	#steps = 180  # Number of steps in the trajectory
	#print(f"fk (x, y, c) = ({start_point})")
	#print(f"tar (x, y, c) = ({end_point})")
	#trajectory = generate_trajectory(start_point, end_point, steps)
	
	#start_time = time.time()
	#plot_trajectory(trajectory)
	relative_command(cur_joint2, cur_joint3, cur_joint4, tar_joint2, tar_joint3, tar_joint4)
	
	#end_time = time.time()
	#delta_time = end_time - start_time
	#print(f"total time: {delta_time:.2f} s)")
	#time.sleep(7)
	#read_joint()
	
def move_degree(tar_joint2, tar_joint3, tar_joint4):
	cur_joint2, cur_joint3, cur_joint4 = read_present_position()
	relative_command(cur_joint2, cur_joint3, cur_joint4, tar_joint2, tar_joint3, tar_joint4)
	
	
def take_data():
	target_x = float(entry_x_pos.get())
	target_y = float(entry_y_pos.get())
	target_c = float(entry_claw_angle.get())
	
	tar_joint2, tar_joint3, tar_joint4 = inverse_kinematics(target_x, target_y, target_c)
	cur_joint2, cur_joint3, cur_joint4 = read_present_position()
	

	delta2 = tar_joint2-cur_joint2
	delta3 = tar_joint3-cur_joint3
	delta4 = tar_joint4-cur_joint4
	print(f"delta (2,3,4) = {delta2:.2f}, {delta3:.2f}, {delta4:.2f} degree")
	
	
	duration_time = int(entry_accel_time.get())
	accel_time = int(duration_time/2)
	decel_time = int(duration_time/2)
	max_speed = int(entry_max_speed.get())
	
	if delta2 != 0:
		speed2 = max_speed
		speed3 = speed2 * (delta3 / delta2)
		speed4 = speed2 * (delta4 / delta2)
	elif delta3 != 0:
		speed2 = max_speed
		speed3 = max_speed
		speed4 = speed3 * (delta4 / delta3)
	elif delta4 != 0:
		speed2 = max_speed
		speed3 = max_speed
		speed4 = max_speed
		
	tar_speed2 = int(round(speed2))
	tar_speed3 = int(round(speed3))
	tar_speed4 = int(round(speed4))
	
	

	init_motor(accel_time, decel_time, tar_speed2, tar_speed3, tar_speed4)

	relative_command(cur_joint2, cur_joint3, cur_joint4, tar_joint2, tar_joint3, tar_joint4)
	with open("output.txt", "w") as file:
		while True:
			ret2, ret3, ret4 = is_targets_reached()
			cur_joint2, cur_joint3, cur_joint4 = read_present_position()
			cur_speed2 = read_servo_speed(ID2)
			cur_speed3 = read_servo_speed(ID3)
			cur_speed4 = read_servo_speed(ID4)
			ts = time.time()
			seconds = int(ts)  # Extract the whole seconds part
			milliseconds = int((ts - seconds) * 1000)  # Extract the milliseconds part
			file.write(f"target_angle (2,3,4) = {tar_joint2:.2f}, {tar_joint3:.2f}, {tar_joint4:.2f} degree\n")
			file.write(f"present_angle (2,3,4) = {cur_joint2:.2f}, {cur_joint3:.2f}, {cur_joint4:.2f} degree\n")
			file.write(f"desired_speed (2,3,4) = {speed2:.2f}, {speed3:.2f}, {speed4:.2f} r/min\n")
			file.write(f"desired speed rounded (2,3,4) = {int(speed2):.2f}, {int(speed3):.2f}, {int(speed4):.2f} r/min\n")
			file.write(f"present_speed (2,3,4) = {cur_speed2:.2f}, {cur_speed3:.2f}, {cur_speed4:.2f} r/min\n")
			file.write(f"is_target_reached (2,3,4) = {ret2}, {ret3}, {ret4}\n\n")
			file.write(f"time stamped = {seconds}s {milliseconds}ms\n\n")
			if (ret2 & ret3 & ret4):
				break

	

def read_kp():
	for id in [ID2, ID3, ID4] :
		kp = can_tx(id, READ_2_BYTE, INDEX_POSITION_KP, 0)
		print(f"current joint{id - 0x600} kp : {kp}")
	

def set_kp(kp2, kp3, kp4):
	print(f"set_kp joint 2,3,4 : ({kp2}, {kp3}, {kp4})")
	can_tx(ID2, SET_2_BYTE, INDEX_POSITION_KP, kp2)
	can_tx(ID3, SET_2_BYTE, INDEX_POSITION_KP, kp3)
	can_tx(ID4, SET_2_BYTE, INDEX_POSITION_KP, kp4)
	time.sleep(1)
	read_kp()


def send_kp():
	try:
		kp2 = int(entry_kp2.get())
		kp3 = int(entry_kp3.get())
		kp4 = int(entry_kp4.get())
		
		set_kp(kp2, kp3, kp4)
	except ValueError:
		messagebox.showerror("Invalid Input", "Please enter valid numerical values for kp2, kp3 kp4.")

def read_error():
    # Error descriptions
    error_descriptions = {
        0x0000: "No Error",
        0x0001: "Overvoltage",
        0x0002: "Overcurrent",
        0x0004: "Out of Tolerance",
        0x0008: "Missing Phase",
        0x0010: "Internal Reference Voltage Error",
        0x0020: "EEPROM Read/Write Error",
    }
    
    # Check errors for each servo ID
    for servo_id in [ID2, ID3, ID4]:
        error_code = can_tx(servo_id, READ_2_BYTE, INDEX_ERROR_CODE, 0)
        error_description = error_descriptions.get(error_code, "Unknown Error")
        print(f"Servo ID {servo_id:03X} error code ({error_code:02X}: {error_description})")

def fault_reset():
	error_descriptions = {
	0x00: "OK",
	0x01: "Transmit fail",
	}
    
	for servo_id in [ID2, ID3, ID4]:
		ret = can_tx(servo_id, SET_2_BYTE, INDEX_CONTROL_WORD, 0x80)
		ret = can_tx(servo_id, SET_2_BYTE, INDEX_CONTROL_WORD, 0x00)
		error_description = error_descriptions.get(ret, "Unknown Error")
		print(f"fault reset servo id {servo_id:03X} : {error_description}")


	
def on_closing():
	print("closing")
	shutdown_motors()
	stop_timer()
	root.destroy()
        #thread.stop()
        
def straight_line():
	try:
		duration_time = int(entry_accel_time.get())
		sleep = (duration_time / 1000)
		move_xyc(258,0,0)
		time.sleep(sleep)
		print(f"1")
		move_xyc(150,0,0)
		time.sleep(sleep)
		print(f"2")
		move_xyc(258,0,0)
		time.sleep(sleep)
		print(f"3")
		move_xyc(150,0,0)
		time.sleep(sleep)
		print(f"4")
		move_xyc(258,0,0)
		time.sleep(sleep)
		print(f"5")
		move_xyc(150,0,0)
		time.sleep(sleep)


	except ValueError:
		messagebox.showerror("Invalid Input", "Please enter valid numerical values duration time")
	
	
	
def rectangular():
	try:
		duration_time = int(entry_accel_time.get())
		sleep = (duration_time / 1000) + 0.1
		move_xyc(210,40,0)
		time.sleep(sleep)
		move_xyc(210,-40,0)
		time.sleep(sleep)
		move_xyc(130,-40,0)
		time.sleep(sleep)
		move_xyc(130,40,0)
		time.sleep(sleep)
		move_xyc(210,40,0)
		time.sleep(sleep)
		move_xyc(210,-40,0)
		time.sleep(sleep)
		move_xyc(130,-40,0)
		time.sleep(sleep)
		move_xyc(130,40,0)
		time.sleep(sleep)
		move_xyc(210,40,0)
		time.sleep(sleep)


	except ValueError:
		messagebox.showerror("Invalid Input", "Please enter valid numerical values duration time")
		
def home_position():
	try:
		duration_time = int(entry_accel_time.get())
		sleep = (duration_time / 1000)
		move_degree(0,0,0)
		time.sleep(sleep)
		#time.sleep(sleep)


	except ValueError:
		messagebox.showerror("Invalid Input", "Please enter valid numerical values duration time")
		
def shuttle_position():
	try:
		duration_time = int(entry_accel_time.get())
		sleep = (duration_time / 1000)
		move_xyc(166.82,-168,0)
		time.sleep(sleep)
		#time.sleep(sleep)


	except ValueError:
		messagebox.showerror("Invalid Input", "Please enter valid numerical values duration time")

def pre_past_shelf():
	try:
		duration_time = int(entry_accel_time.get())
		sleep = (duration_time / 1000)
		move_xyc(107,100,90)
		time.sleep(sleep)
		#time.sleep(sleep)


	except ValueError:
		messagebox.showerror("Invalid Input", "Please enter valid numerical values duration time")
		
def pickup_from_shelf():
	try:
		duration_time = int(entry_accel_time.get())
		sleep = (duration_time / 1000) + 0.2
		move_xyc(107,224,90)
		time.sleep(sleep)
		#time.sleep(sleep)


	except ValueError:
		messagebox.showerror("Invalid Input", "Please enter valid numerical values duration time")
		
def place_onto_shelf():
	try:
		duration_time = int(entry_accel_time.get())
		sleep = (duration_time / 1000) 
		move_xyc(107,197,90)
		time.sleep(sleep)
		#time.sleep(sleep)


	except ValueError:
		messagebox.showerror("Invalid Input", "Please enter valid numerical values duration time")

def dancing():
	try:
		while True:
			duration_time = int(entry_accel_time.get())
			sleep = (duration_time / 1000) 
			straight_line()
			move_xyc(140,0,-20)
			time.sleep(sleep)
			#move_xyc(140,0,0)
			#time.sleep(sleep)
			#move_xyc(140,0,45)
			#time.sleep(sleep)
			move_xyc(140,0,90)
			time.sleep(sleep)
			move_xyc(140,-168,0)
			time.sleep(sleep)
			home_position()
			shuttle_position()
			home_position()
			shuttle_position()
			home_position()
			rectangular()
			#
			pre_past_shelf()
			pickup_from_shelf()
			pre_past_shelf()
			pickup_from_shelf()
			pre_past_shelf()
			
			#
			move_xyc(150,0,0)
			time.sleep(sleep)
			move_xyc(258,0,90)
			time.sleep(sleep)
			move_xyc(258,0,-90)
			time.sleep(sleep)
			move_xyc(258,0,0)



	except ValueError:
		messagebox.showerror("Invalid Input", "Please enter valid numerical values duration time")
	
	
def signal_handler(sig, frame):
    on_closing()  # Panggil fungsi cleanup atau fungsi lain yang diinginkan
    exit(0)    # Keluar dari program dengan status 0

# Menangkap sinyal Ctrl+C (SIGINT)
signal.signal(signal.SIGINT, signal_handler)
        	
        	
#q3
root = tk.Tk()
root.title("Motor Control Panel")

take_data_button = tk.Button(root, text="take data", command=take_data)
take_data_button.grid(row=0, column=0, columnspan=3, pady=10)

# Input fields for x, y positions
tk.Label(root, text="x position in mm").grid(row=1, column=0, padx=5, pady=5)
entry_x_pos = tk.Entry(root)
entry_x_pos.insert(0, "258")
entry_x_pos.grid(row=1, column=1, padx=5, pady=5)

tk.Label(root, text="y position in mm").grid(row=2, column=0, padx=5, pady=5)
entry_y_pos = tk.Entry(root)
entry_y_pos.insert(0, "0")
entry_y_pos.grid(row=2, column=1, padx=5, pady=5)

tk.Label(root, text="claw position in degree").grid(row=3, column=0, padx=5, pady=5)
entry_claw_angle = tk.Entry(root)
entry_claw_angle.insert(0, "0")
entry_claw_angle.grid(row=3, column=1, padx=5, pady=5)

# Frame for buttons
frame = tk.Frame(root)
frame.grid(row=4, column=0, columnspan=3, pady=10)

init_motor_button = tk.Button(frame, text="Init Motor", command=initialize_motors)#set_origin)
init_motor_button.grid(row=0, column=0, padx=5, pady=10)

move_button = tk.Button(frame, text="Move", command=move)
move_button.grid(row=0, column=1, padx=5, pady=10)

move2_button = tk.Button(frame, text="move2", command=move2)
move2_button.grid(row=0, column=2, padx=5, pady=10)

shutdown_button = tk.Button(frame, text="shutdown", command=shutdown_motors)
shutdown_button.grid(row=0, column=3, padx=5, pady=10)



#shutdown_button = tk.Button(root, text="Shutdown Motors", command=shutdown_motors)
#shutdown_button.grid(row=5, column=0, columnspan=3, pady=10)

# Input fields for acceleration, deceleration, and maximum speed
tk.Label(root, text="Duration Time (ms):").grid(row=6, column=0, padx=5, pady=5)
entry_accel_time = tk.Entry(root)
entry_accel_time.insert(0, "4000")
entry_accel_time.grid(row=6, column=1, padx=5, pady=5)

#tk.Label(root, text="Deceleration Time (ms):").grid(row=7, column=0, padx=5, pady=5)
#entry_decel_time = tk.Entry(root)
##entry_decel_time.insert(0, "2000")
#entry_decel_time.grid(row=7, column=1, padx=5, pady=5)

tk.Label(root, text="Maximum Speed (rpm):").grid(row=8, column=0, padx=5, pady=5)
entry_max_speed = tk.Entry(root)
entry_max_speed.insert(0, "120")
entry_max_speed.grid(row=8, column=1, padx=5, pady=5)

# Checkbox for reversing motor direction
#reverse_var = tk.BooleanVar()
#reverse_checkbox = tk.Checkbutton(root, text="Reverse Motor Direction", variable=reverse_var)
#reverse_checkbox.grid(row=9, column=0, columnspan=3, pady=5)

# Input fields for positions
tk.Label(root, text="Enter position for Motor 1 (degrees):").grid(row=10, column=0, padx=5, pady=5)
entry_motor1 = tk.Entry(root)
entry_motor1.insert(0, "0")
entry_motor1.grid(row=10, column=1, padx=5, pady=5)

tk.Label(root, text="Enter position for Motor 2 (degrees):").grid(row=11, column=0, padx=5, pady=5)
entry_motor2 = tk.Entry(root)
entry_motor2.insert(0, "0")
entry_motor2.grid(row=11, column=1, padx=5, pady=5)

tk.Label(root, text="Enter position for Motor 3 (degrees):").grid(row=12, column=0, padx=5, pady=5)
entry_motor3 = tk.Entry(root)
entry_motor3.insert(0, "0")
entry_motor3.grid(row=12, column=1, padx=5, pady=5)

read_present_position_button = tk.Button(root, text="Read Present Position", command=read_joint)
read_present_position_button.grid(row=9, column=0, columnspan=3, pady=10)

#send_absolute_position_button = tk.Button(root, text="Send Absolute Position", command=send_angle_command)
#send_absolute_position_button.grid(row=14, column=0, columnspan=3, pady=10)

send_relative_position_button = tk.Button(root, text="Send Relative Position", command=send_relative_command)
send_relative_position_button.grid(row=14, column=0, columnspan=3, pady=10)

straight_line_button = tk.Button(root, text="strigt_line", command=straight_line)
straight_line_button.grid(row=15, column=0, columnspan=3, pady=10)

pickup_button = tk.Button(root, text="pickup shelf", command=pickup_from_shelf)
pickup_button.grid(row=16, column=0, columnspan=3, pady=10)

pre_shelf_button = tk.Button(root, text="pre past shelf", command=pre_past_shelf)
pre_shelf_button.grid(row=17, column=0, columnspan=3, pady=10)

dancing_button = tk.Button(root, text="dancing", command=dancing)
dancing_button.grid(row=18, column=0, columnspan=3, pady=10)

# Error handler
frame_error = tk.Frame(root)
frame_error.grid(row=21, column=0, columnspan=3, pady=10)

read_error_button = tk.Button(frame_error, text="read error", command=read_error)
read_error_button.grid(row=0, column=0, padx=5, pady=10)

fault_reset_button = tk.Button(frame_error, text="fault reset", command=fault_reset)
fault_reset_button.grid(row=0, column=1, padx=5, pady=10)
"""
# Input kp
tk.Label(root, text="Enter kp for Motor 2:").grid(row=16, column=0, padx=5, pady=5)
entry_kp2 = tk.Entry(root)
entry_kp2.insert(0, "6000")
entry_kp2.grid(row=16, column=1, padx=5, pady=5)

tk.Label(root, text="Enter kp for Motor 3:").grid(row=17, column=0, padx=5, pady=5)
entry_kp3 = tk.Entry(root)
entry_kp3.insert(0, "6000")
entry_kp3.grid(row=17, column=1, padx=5, pady=5)

tk.Label(root, text="Enter kp for Motor 4:").grid(row=18, column=0, padx=5, pady=5)
entry_kp4 = tk.Entry(root)
entry_kp4.insert(0, "6000")
entry_kp4.grid(row=18, column=1, padx=5, pady=5)

frame_kp = tk.Frame(root)
frame_kp.grid(row=19, column=0, columnspan=3, pady=10)

set_kp_button = tk.Button(frame_kp, text="set kp", command=send_kp)
set_kp_button.grid(row=0, column=0, padx=5, pady=10)

read_kp_button = tk.Button(frame_kp, text="read kp", command=read_kp)
read_kp_button.grid(row=0, column=1, padx=5, pady=10)
 """   
root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
bus.shutdown()