"""
author: teukuzikrifatahillah@gmail.com
about this file: we start using pusirobot motor
"""

import can
import threading
import time
import struct
import tkinter as tk
import math
import signal
import numpy as np

STEPPER_PPR = 4096
SERVO_PPR = 10000
STEPPER_RATIO = STEPPER_PPR / 360
SERVO_RATIO = SERVO_PPR / 360

def stepper_degrees_to_pulses(degrees):
    return int(degrees * STEPPER_RATIO)


def stepper_pulses_to_degrees(pulses):
    return float(pulses / STEPPER_RATIO)


def servo_degrees_to_pulses(degrees):
    return int(degrees * SERVO_RATIO)


def servo_pulses_to_degrees(pulses):
    return float(pulses / SERVO_RATIO)


# PUSIROBOT LIBRARY
SET_1_BYTE = 0x2F
SET_2_BYTE = 0x2B
SET_3_BYTE = 0x27
SET_4_BYTE = 0x23
SET_OK = 0x60
READ_REQ = 0x40
READ_1_BYTE = 0x4F
READ_2_BYTE = 0x4B
READ_3_BYTE = 0x47
READ_4_BYTE = 0x43
SET_ERROR = 0x80

# System Information (RO)
OD_STEPPER_DEVICE_NAME = 0x1008
OD_STEPPER_HARDWARE_VERSION = 0x1009
OD_STEPPER_SOFTWARE_VERSION = 0x100A

# Communication Configuration
# need to reset power to enable (cansend can0 000#82<CAN_ID>)
OD_STEPPER_NODE_ID = 0x2002
OD_STEPPER_BAUD_RATE = 0x2003
OD_STEPPER_GROUP_ID = 0x2006
OD_STEPPER_SYSTEM_CONTROL = 0x2007

# Motor Control Parameter
OD_STEPPER_ERROR_STATUS = 0x6000
OD_STEPPER_CONTROLLER_STATUS = 0x6001
OD_STEPPER_ROTATION_DIRECTION = 0x6002
OD_STEPPER_MAX_SPEED = 0x6003
OD_STEPPER_RELATIVE_DISPLACEMENT = 0x6004
OD_STEPPER_ABSOLUTE_DISPLACEMENT = 0x601C
OD_STEPPER_STOP_STEPPING = 0x6020
OD_STEPPER_OPERATION_MODE = 0x6005
OD_STEPPER_START_SPEED = 0x6006
INEDX_STOP_SPEED = 0x6007
OD_STEPPER_ACCEL_COEF = 0x6008
OD_STEPPER_DECEL_COEF = 0x6009
OD_STEPPER_MICROSTEPPING = 0x600A
OD_STEPPER_MAX_PHASE_CURRENT = 0x600B
OD_STEPPER_MOTOR_POSITION = 0x600C
OD_STEPPER_CALIBRATION_ZERO = 0x6034
OD_STEPPER_ENCODER_POSITION = 0x6035
OD_STEPPER_CURRENT_REDUCTION = 0x600D
OD_STEPPER_MOTOR_ENABLE = 0X600E
OD_STEPPER_STALL_SET = 0X601B
OD_STEPPER_STALL_PARAMETERS = 0X6017
OD_STEPPER_REAL_TIME_SPEED = 0X6030
OD_STEPPER_EXTERNAL_EMERGENCY_STOP = 0X600F

# CLOSE LOOP CONTROL
OD_STEPPER_ENCODER_RESOLUTION = 0X6021
OD_STEPPER_KP_PARAMETER = 0X6023
OD_STEPPER_KI_PARAMETER = 0X6024
OD_STEPPER_KD_PARAMETER = 0X6025
OD_STEPPER_PRE_FILTERING_PARAMETER = 0X6026
OD_STEPPER_POST_FILTERING_PARAMETER = 0X6027
OD_STEPPER_STALL_LENGTH = 0X6028
OD_STEPPER_TORQUE_RING_ENABLE = 0X6029
OD_STEPPER_AUTOSAVE_WHEN_POWEROFF = 0X602A

# SYNC POSITION MOTION MODE
OD_STEPPER_SP_MOTION = 0X601D
OD_STEPPER_PVT_MOTION = 0X6010
OD_STEPPER_PP_MOTION = 0X602D
OD_STEPPER_PP_MOTION_2 = 0X602E
OD_STEPPER_ANALOG_MODE = 0X602F

# BRAKE CONTROL
OD_STEPPER_BRAKE_CONTROL = 0X6016
OD_STEPPER_ANALOGUE_INPUT = 0X602B

# STEP NOTIFICATION
OD_STEPPER_STEP_NOTIFICATION = 0X602C

OD_STEPPER_POWER_LOSS_BEHAVIOR = 0X6031

# System Information (RO)
OD_SERVO_DEVICE_TYPE = 0x1000
OD_SERVO_ERROR_REGISTER = 0x1001
OD_SERVO_DEVICE_STATUS_REGISTER = 0x1002 #not available
OD_SERVO_PREDEFINED_ERROR_FIELD = 0x1003
OD_SERVO_COB_ID_SYNC_MSG = 0x1005
OD_SERVO_SYNC_PERIOD = 0x1006
OD_SERVO_SYNC_WINDOW_LENGTH = 0x1007
OD_SERVO_DEVICE_NAME = 0x1008 #not available
OD_SERVO_HARDWARE_VERSION = 0x1009 #not available
OD_SERVO_SOFTWARE_VERSION = 0x100A #not available
OD_SERVO_STORE_PARAMETERS = 0x1010
OD_SERVO_RESET_THE_FAULT_PARAMETERS = 0x1011
OD_SERVO_EMERGENCY_MSG_COB_ID = 0x1014
OD_SERVO_EMERGENCY_MSG_PROHIBITED_TIME = 0x1015 #not available
OD_SERVO_PRODUCER_HEARTH_BEAT = 0x1017
OD_SERVO_OBJECT_IDENTIFIER = 0x1018 #not available
OD_SERVO_SYNC_CNT_OVERFLOW = 0x1019 #not available
OD_SERVO_ERROR_BEHAVIOR = 0x1029 #not available
OD_SERVO_SDO_SERVER_PARAMETERS = 0x1200
OD_SERVO_RPDO_COMMUNICATION_0 = 0x1400
OD_SERVO_RPDO_COMMUNICATION_1 = 0x1401
OD_SERVO_RPDO_COMMUNICATION_2 = 0x1402
OD_SERVO_RPDO_COMMUNICATION_3 = 0x1403
OD_SERVO_RPDO_MAPPING_0 = 0x1600
OD_SERVO_RPDO_MAPPING_1 = 0x1601
OD_SERVO_RPDO_MAPPING_2 = 0x1602
OD_SERVO_RPDO_MAPPING_3 = 0x1603
OD_SERVO_TPDO_COMMUNICATION_0 = 0x1800
OD_SERVO_TPDO_COMMUNICATION_1 = 0x1801
# OD_SERVO_TPDO_COMMUNICATION_2 = 0x1802
# OD_SERVO_TPDO_COMMUNICATION_3 = 0x1803
OD_SERVO_TPDO_MAPPING_0 = 0x1A00
OD_SERVO_TPDO_MAPPING_1 = 0x1A01
# OD_SERVO_TPDO_MAPPING_2 = 0x1A02
# OD_SERVO_TPDO_MAPPING_3 = 0x1A03

# Device Protocol
OD_SERVO_DSP_ERROR_CODE = 0x603F #not available
OD_SERVO_CONTROL_WORD = 0x6040
OD_SERVO_STATUS_WORD = 0x6041
OD_SERVO_QUICK_STOP_CODE = 0x605A
OD_SERVO_HALT_CODE = 0x605D
OD_SERVO_MODE_OF_OPERATION = 0x6060
OD_SERVO_MODE_CODE_RESPONSE = 0X6061
OD_SERVO_POSITION_ACTUAL_VALUE = 0X6064
OD_SERVO_VELOCITY_ACTUAL_VALUE= 0X606C
OD_SERVO_TARGET_POSITION = 0x607A
OD_SERVO_HOME_OFFSET = 0X607C
OD_SERVO_TARGET_VELOCITY = 0X6081
OD_SERVO_ACCELERATION = 0X6083
OD_SERVO_DECELERATION = 0X6084
OD_SERVO_QUICK_STOP_DECELERATION = 0X6085
OD_SERVO_HOMING_METHOD = 0X6098
OD_SERVO_HOMING_SPEEDS = 0X6099
OD_SERVO_HOMING_ACCELERATION = 0X609A

ABORT_CODES = {
    0x05030000: "No alteration of trigger bits",
    0x05030001: "SDO protocol timeout",
    0x05040001: "Illegal or unknown Client/Server command word",
    0x05040002: "Invalid block size (only Transfer Block mode)",
    0x05040003: "Invalid serial number (only Transfer Block mode)",
    0x05040004: "CRC error (only Transfer Block mode)",
    0x05040005: "Out of memory",
    0x06010000: "Access is not supported for the Object",
    0x06010001: "Try to read write-only objects",
    0x06010002: "Try to write read-only objects",
    0x06020000: "Object does not exist in the Object Dictionary",
    0x06040041: "Object cannot be mapped to PDO",
    0x06040042: "The number and length of the mapped object exceeds the PDO length",
    0x06040043: "General parameters are not compatible",
    0x06040047: "General equipment is not compatible",
    0x06060000: "Hardware error causes the object access failure",
    0x06060010: "Data type does not match, and service parameter length does not match",
    0x06060011: "Data type does not match, the service parameter is too large",
    0x06060013: "Data type does not match, the service parameter is too small",
    0x06090011: "The subOD_STEPPER does not exist",
    0x06090030: "Beyond the range of the parameter values (when write access)",
    0x06090031: "Parameter value is written too large",
    0x06090032: "Parameter value is written too small",
    0x06090036: "The maximum value is less than the minimum value",
    0x08000000: "General error",
    0x08000020: "Data cannot be transferred or saved to applications",
    0x08000021: "Due to local control, data cannot be transferred or saved to applications",
    0x08000022: "Due to the current device status, data cannot be transferred or saved to applications",
    0x08000023: "The dynamic condition of Object dictionary generates error or Object Dictionary does not exist",
}

ID1 = 0x601
ID2 = 0x602
ID3 = 0x603
ID4 = 0x604

response_id_map = {
    ID1: 0x581,
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
                    if can_id in {ID1, ID2, ID3, ID4}:
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
        
bus = can.Bus(channel='can0', interface='socketcan')


def format_data(can_id, set_length, OD_STEPPER, data):
    # Convert ID and length to hexadecimal strings
    can_id_hex = format(can_id, '03X')  # Ensure ID is 3 characters long for proper formatting
    set_length_hex = format(set_length, '02X')  # Set length as 2 characters long

    # Convert OD_STEPPER to little-endian and then to hex
    OD_STEPPER_le = struct.pack('<H', OD_STEPPER)
    OD_STEPPER_hex = ''.join(f"{byte:02X}" for byte in OD_STEPPER_le)

    # Convert data to little-endian 4-byte format and then to hex
    if data < 0:
        # Ensure data is treated as an unsigned 32-bit integer if negative
        data = (data + 0x100000000) % 0x100000000  # Wrap around for 32-bit unsigned
    data_le = struct.pack('<I', data)
    data_hex = ''.join(f"{byte:02X}" for byte in data_le)

    # Format the message
    message = f"{can_id_hex}#{set_length_hex}{OD_STEPPER_hex}00{data_hex}"
    return message
    
def format_data_2(can_id, set_length, sdo_id, OD_STEPPER_id, data):
    # Convert ID and length to hexadecimal strings
    can_id_hex = format(can_id, '03X')  # Ensure ID is 3 characters long for proper formatting
    set_length_hex = format(set_length, '02X')  # Set length as 2 characters long

    # Convert OD_STEPPER to little-endian and then to hex
    sdo_id_le = struct.pack('<H', sdo_id)
    sdo_id_hex = ''.join(f"{byte:02X}" for byte in sdo_id_le)
    
    OD_STEPPER_id_hex = format(OD_STEPPER_id, '02X')  # Set length as 2 characters long

    # Convert data to little-endian 4-byte format and then to hex
    if data < 0:
        # Ensure data is treated as an unsigned 32-bit integer if negative
        data = (data + 0x100000000) % 0x100000000  # Wrap around for 32-bit unsigned
    data_le = struct.pack('<I', data)
    data_hex = ''.join(f"{byte:02X}" for byte in data_le)

    # Format the message
    message = f"{can_id_hex}#{set_length_hex}{sdo_id_hex}{OD_STEPPER_id_hex}{data_hex}"
    return message

def send_can_command(command):
    can_id, can_data = command.split('#')
    can_id = int(can_id, 16)
    can_data = bytes.fromhex(can_data)

    msg = can.Message(arbitration_id=can_id, data=can_data, is_extended_id=False)
    bus.send(msg)


def can_tx(request_id, set_length, OD_STEPPER, data):
    command = format_data(request_id, set_length, OD_STEPPER, data)
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
        if len(data_bytes) >= 4:
            abort_code = struct.unpack('<I', data_bytes[4:8])[0]
            abort_description = ABORT_CODES.get(abort_code, "Unknown abort code")
            print(f"Error: Abort Code {abort_code:08X} - {abort_description}")
        return 0x01
    else:
        if len(data_bytes) >= 4:
            last_4_bytes = data_bytes[-4:]
			# Unpack data as little-endian (assuming 4 bytes for 32-bit value)
            data_int = struct.unpack('<i', last_4_bytes)[0]
        else:
            data_int = 0
        return data_int
    
def can_tx_2(request_id, set_length, sdo_id, OD_STEPPER_id, data):
    command = format_data_2(request_id, set_length, sdo_id, OD_STEPPER_id, data)
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
        if len(data_bytes) >= 4:
            abort_code = struct.unpack('<I', data_bytes[4:8])[0]
            abort_description = ABORT_CODES.get(abort_code, "Unknown abort code")
            print(f"Error: Abort Code {abort_code:08X} - {abort_description}")
        return 0x01
    else:
        if len(data_bytes) >= 4:
            last_4_bytes = data_bytes[-4:]
			# Unpack data as little-endian (assuming 4 bytes for 32-bit value)
            data_int = struct.unpack('<i', last_4_bytes)[0]
        else:
            data_int = 0
        return data_int
    
def can_tx_rx(request_id, set_length, sdo_id, OD_STEPPER_id, data):
    write_ret = can_tx_2(request_id, set_length, sdo_id, OD_STEPPER_id, data)
    read_ret = can_tx_2(request_id, READ_REQ, sdo_id, OD_STEPPER_id, 0x00)
    return write_ret, read_ret

# Error Status Definitions
ERROR_BITS = {
	0: "TSD (Over temperature shutdown)",
	1: "AERR (Coil A error)",
	2: "BERR (Coil B error)",
	3: "AOC (A Overcurrent)",
	4: "BOC (B Overcurrent)",
	5: "UVLO (Low voltage fault)"
	}


def decode_error_status(error_status):
    """
    Decode and print the error status based on bit definitions.
    """
    print("Decoded Error Status:")
    for bit, description in ERROR_BITS.items():
        if error_status & (1 << bit):
            print(f"  - {description}")
    if error_status == 0:
        print("  - No errors detected.")


def error_status():
    for id in [ID2, ID3, ID4]:
        error_status = can_tx(id, READ_REQ, OD_STEPPER_ERROR_STATUS, 0)
        decode_error_status(error_status)
    
def save_settings():
    #save settings
    for id in [ID2, ID3, ID4]:
        can_tx_2(id, SET_1_BYTE, OD_STEPPER_SYSTEM_CONTROL, 0x00, 2)

def versatile():
    # ret = can_tx(ID3, READ_REQ, 0x6002, 0)
    # print(f"{ret}")
    send_can_command("603#2b2e600110000000")

    
def encoder_position():
    enc1 = can_tx(ID1, READ_REQ, OD_SERVO_POSITION_ACTUAL_VALUE, 0)
    enc2 = can_tx(ID2, READ_REQ, OD_STEPPER_ENCODER_POSITION, 0)
    enc3 = can_tx(ID3, READ_REQ, OD_STEPPER_ENCODER_POSITION, 0)
    enc4 = can_tx(ID4, READ_REQ, OD_STEPPER_ENCODER_POSITION, 0)
    print(f"enc: {enc1}, {enc2}, {enc3}, {enc4}")
    
    return enc1, enc2, enc3, enc4

# Forward kinematics function
def forward_kinematics(cur_deg1, cur_deg2, cur_deg3, cur_deg4):
    #x:258, y:0, c:0 = joint2:482.5, joint3:-187.5, joint4:262.5
    L2 = 137.0
    L3 = 121.0
    L4 = 56.82
    OFFSET_2 = -96.5
    OFFSET_3 = 134.0
    OFFSET_4 = -52.5
    theta2_rad = math.radians((cur_deg2 / 5) + OFFSET_2)
    theta3_rad = math.radians((cur_deg3 / 5) + OFFSET_3 - (cur_deg2 / 5))
    x2 = L2 * math.cos(theta2_rad)
    y2 = L2 * math.sin(theta2_rad)
    x3 = x2 + L3 * math.cos(theta2_rad + theta3_rad)
    y3 = y2 + L3 * math.sin(theta2_rad + theta3_rad)
    z = (cur_deg1/360)*90
    yaw = (cur_deg4 / 5) + OFFSET_4
    return x3, y3, z, yaw  
    
    
def read_present_position():
    m1_pulse = can_tx(ID1, READ_REQ, OD_SERVO_POSITION_ACTUAL_VALUE, 0)    
    m2_pulse = can_tx(ID2, READ_REQ, OD_STEPPER_MOTOR_POSITION, 0)
    m3_pulse = can_tx(ID3, READ_REQ, OD_STEPPER_MOTOR_POSITION, 0)
    m4_pulse = can_tx(ID4, READ_REQ, OD_STEPPER_MOTOR_POSITION, 0)
    
    m1_angle = servo_pulses_to_degrees(m1_pulse)
    m2_angle = stepper_pulses_to_degrees(m2_pulse)
    m3_angle = stepper_pulses_to_degrees(m3_pulse)
    m4_angle = stepper_pulses_to_degrees(-m4_pulse)
    
    
    cur_x, cur_y, cur_z, cur_yaw = forward_kinematics(m1_angle, m2_angle, m3_angle, m4_angle)
    
    print(f"cur coordinate : x:{cur_x:.1f} mm, y:{cur_y:.1f} mm, z:{cur_z:.1f} mm, yaw:{cur_yaw:.1f} degree")
    print(f"cur joint : m1:{m1_angle:.1f}, m2:{m2_angle:.1f}, m3:{m3_angle:.1f}, m4:{m4_angle:.1f} degree")
    
    
    return m1_angle, m2_angle, m3_angle, m4_angle


def int_to_little_endian(value):

    # Convert integer to 4-byte little-endian representation
    little_endian_bytes = value.to_bytes(4, byteorder='little', signed=True)
    
    # Convert bytes to a hexadecimal string and format
    little_endian_hex = ''.join(f'{byte:02X}' for byte in little_endian_bytes)
    
    return little_endian_hex

    


def relative_position():
    try:
        tar_joint_2 = float(entry_tar_joint_2.get())
        tar_joint_3 = float(entry_tar_joint_3.get())
        tar_joint_4 = float(entry_tar_joint_4.get())
        tar_joint_4 = tar_joint_4 * -1.0
        speed = int(entry_speed.get())
        
        
        
        pulse2 = stepper_degrees_to_pulses(tar_joint_2)
        pulse3 = stepper_degrees_to_pulses(tar_joint_3)
        pulse4 = stepper_degrees_to_pulses(tar_joint_4)
        
        
        
        if pulse2 >= 0:
            speed2 = speed
        else:
            speed2 = -speed
            
        if pulse3 >= 0:
            speed3 = speed
        else:
            speed3 = -speed
        
        if pulse4 >= 0:
            speed4 = speed
        else:
            speed4 = -speed
        
        pulse2_le = int_to_little_endian(abs(pulse2))
        pulse3_le = int_to_little_endian(abs(pulse3))
        pulse4_le = int_to_little_endian(abs(pulse4))
        speed2_le = int_to_little_endian(speed2)
        speed3_le = int_to_little_endian(speed3)
        speed4_le = int_to_little_endian(speed4)
    except ValueError:
        print("Please enter valid numbers for angles.")
        
    for id, speed_le in zip([ID2, ID3, ID4], [speed2_le, speed3_le, speed4_le]):
        send_can_command(f"{id:03X}#232e6003{speed_le}")
    time.sleep(0.5)

    for id, pulse_le in zip([ID2, ID3, ID4], [pulse2_le, pulse3_le, pulse4_le]):
        send_can_command(f"{id:03X}#232e6004{pulse_le}")
    time.sleep(0.5)

    for id in [ID2, ID3, ID4]:
        send_can_command(f"{id:03X}#2b2e600110000000")

    
def absolute_position():
    try:
        tar_joint_2 = float(entry_tar_joint_2.get())
        tar_joint_3 = float(entry_tar_joint_3.get())
        tar_joint_4 = float(entry_tar_joint_4.get())
        tar_joint_4 = tar_joint_4 * -1.0
        speed = int(entry_speed.get())
        
        pulse2 = stepper_degrees_to_pulses(tar_joint_2)
        pulse3 = stepper_degrees_to_pulses(tar_joint_3)
        pulse4 = stepper_degrees_to_pulses(tar_joint_4)
        
        pulse2_le = int_to_little_endian(pulse2)
        pulse3_le = int_to_little_endian(pulse3)
        pulse4_le = int_to_little_endian(pulse4)
        speed_le = int_to_little_endian(speed)
    except ValueError:
        print("Please enter valid numbers for angles.")
        
    for id in [ID2, ID3, ID4]:
        send_can_command(f"{id:03X}#232e6003{speed_le}")
    time.sleep(0.5)

    for id, pulse_le in zip([ID2, ID3, ID4], [pulse2_le, pulse3_le, pulse4_le]):
        send_can_command(f"{id:03X}#232e6004{pulse_le}")
    time.sleep(0.5)

    for id in [ID2, ID3, ID4]:
        send_can_command(f"{id:03X}#2b2e600150000000")

def stepper_accel_decel_calc(d_total, t_travel_ms):
    t_accel_ms = t_travel_ms / 2  # Accel and decel time (ms)
    d_accel = d_total / 2  # Distance during accel and decel (pulses)

    # Convert time from ms to seconds for calculations
    t_accel = t_accel_ms / 1000  # Accel and decel time (s)
    
    # Calculate acceleration and max speed
    accel = (2 * d_accel) / (t_accel ** 2)  # Acceleration (pps^2)
    # v_max = accel * t_accel  # Maximum speed (pps)
    int_accel = (int)(abs(accel))
    # print(int_accel)
    return int_accel

def servo_accel_decel_calc(d_total, t_travel_ms):
    t_accel_ms = t_travel_ms / 2  # Accel and decel time (ms)
    d_accel = d_total / 2  # Distance during accel and decel (pulses)

    # Convert time from ms to seconds for calculations
    t_accel = t_accel_ms / 1000  # Accel and decel time (s)
    
    # Calculate acceleration in pulses per second squared (pps²)
    accel_pps_squared = (2 * d_accel) / (t_accel ** 2)
    
    # Convert acceleration to revolutions per second squared (rps²)
    accel_rps_squared = (int)(abs((accel_pps_squared / SERVO_PPR) * 10))

    return accel_rps_squared

def stall_on():
	for id in [ID2, ID3, ID4]:
		send_can_command(f"{id:03X}#2f1b600001000000")
	print(f"stall (open-loop) activated")


def stall_off():
	for id in [ID2, ID3, ID4]:
		send_can_command(f"{id:03X}#2f1b600001000000")
	print(f"stall (open-loop) deactivated, be carefull")


def generate_trajectory(start, end, steps):
    x_points = np.linspace(start[0], end[0], steps)
    y_points = np.linspace(start[1], end[1], steps)
    z_points = np.linspace(start[2], end[2], steps)
    c_points = np.linspace(start[3], end[3], steps)
    trajectory = list(zip(x_points, y_points, z_points,c_points))
    return trajectory

def sp_mode_init():
    # Set working mode to position mode: 
    for id in [ID2, ID3, ID4]:
        _,ret = can_tx_rx(id, SET_1_BYTE, OD_STEPPER_OPERATION_MODE, 0x00, 0x00)
        print(f"{id:03X} operation mode is {ret}")
    
    for id in [ID2, ID3, ID4]:
        _,ret = can_tx_rx(id, SET_1_BYTE, OD_STEPPER_ACCEL_COEF, 0x00, 0x01)
        print(f"{id:03X} accel coef is {ret}")  
    
    for id in [ID2, ID3, ID4]:
        _,ret = can_tx_rx(id, SET_1_BYTE, OD_STEPPER_DECEL_COEF, 0x00, 0x01)
        print(f"{id:03X} decel coef is {ret}")  
    
    # read group id
    for id in [ID2, ID3, ID4]:
        _,ret = can_tx_rx(id, SET_1_BYTE, OD_STEPPER_GROUP_ID, 0x00, 0x05)
        print(f"{id:03X} group id is {ret}")  

def sp_mode_set_speed(speed_2, speed_3, speed_4):
    for id, speed in zip([ID2, ID3, ID4], [speed_2, speed_3, speed_4]):
        can_tx_2(id, SET_4_BYTE, OD_STEPPER_SP_MOTION, 0x01, speed)
        
def sp_mode_set_tar_pulse(tar_pulse_2, tar_pulse_3, tar_pulse_4):
    for id, pulse in zip([ID2, ID3, ID4], [tar_pulse_2, tar_pulse_3, -tar_pulse_4]):
        can_tx_2(id, SET_4_BYTE, OD_STEPPER_SP_MOTION, 0x02, pulse)

def sp_move(tar_joint_2, tar_joint_3, tar_joint_4, speed):
    pulse_2 = stepper_degrees_to_pulses(tar_joint_2)
    pulse_3 = stepper_degrees_to_pulses(tar_joint_3)
    pulse_4 = stepper_degrees_to_pulses(tar_joint_4)
    
    speed_2 = speed
    speed_3 = speed
    speed_4 = speed
    
    # sp_mode_set_speed(speed_2, speed_3, speed_4)
    sp_mode_set_tar_pulse(pulse_2, pulse_3, pulse_4)
        
    #send sync move
    send_can_command(f"000#0A05")
    # print(f"000#0A05")

def plot_trajectory(trajectory, speed):
    cnt = 0
    previous_time = 0
    interval = 4
    for (x, y, z, yaw) in trajectory:
        start_time = time.time()
        try:
            current_time = time.time()
            delta_time = current_time - previous_time
            previous_time = current_time
            
            #cur_joint_1, cur_joint_2, cur_joint_3, cur_joint_4 = read_present_position()       
            tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4 = inverse_kinematics(x, y, z, yaw)
            
            #print(f"Tar xyyaw {cnt}: {x:.2f}, {y:.2f}, {yaw:.2f} (dt: {delta_time:.2f} s)")
            #print(f"\033[94mcur: ({cur_joint2:.2f}, {cur_joint3:.2f}, {cur_joint4:.2f})\033[0m")
            print(f"Tar: ({tar_joint_2:.2f}, {tar_joint_3:.2f}, {tar_joint_4:.2f}) (dt: {delta_time:.2f} s)")
            sp_move(tar_joint_2, tar_joint_3, tar_joint_4, speed)
            #relative_command(cur_joint2, cur_joint3, cur_joint4, tar_joint2, tar_joint3, tar_joint4)
            elapsed_time = time.time() - start_time
            sleep_time = max(0, interval - elapsed_time)
            time.sleep(sleep_time)
        except ValueError as e:
            print(f"Skipping point ({x:.1f}, {y:.1f}): {e}")



def move():	
    try:
        tar_x = float(entry_tar_x.get())
        tar_y = float(entry_tar_y.get())
        tar_z = float(entry_tar_z.get())
        tar_yaw = float(entry_tar_yaw.get())
        speed = int(entry_speed.get())

    except ValueError:
        print("Please enter valid numbers for angles.")
        
    sp_mode_init()
    sp_mode_set_speed(speed, speed, speed)
    time.sleep(2)

    cur_joint_1, cur_joint_2, cur_joint_3, cur_joint_4 = read_present_position()
    start_point = forward_kinematics(cur_joint_1, cur_joint_2, cur_joint_3, cur_joint_4)
    end_point = (tar_x, tar_y, tar_z, tar_yaw)   # Ending position (x, y, c)
    steps = 5  # Number of steps in the trajectory

    print(f"fk (x, y, z, c) = ({start_point})")
    print(f"tar (x, y, z, c) = ({end_point})")


    trajectory = generate_trajectory(start_point, end_point, steps)

    start_time = time.time()
    plot_trajectory(trajectory, speed)
    end_time = time.time()
    delta_time = end_time - start_time
    print(f"total time: {delta_time:.2f} s)")

def sp_mode():
    # try:
    #     tar_joint_2 = float(entry_tar_joint_2.get())
    #     tar_joint_3 = float(entry_tar_joint_3.get())
    #     tar_joint_4 = float(entry_tar_joint_4.get())
    #     speed = int(entry_speed.get())

    # except ValueError:
    #     print("Please enter valid numbers for angles.")
    
    try:
        tar_x = float(entry_tar_x.get())
        tar_y = float(entry_tar_y.get())
        tar_yaw = float(entry_tar_yaw.get())
        speed = int(entry_speed.get())

    except ValueError:
        print("Please enter valid numbers for angles.")
    
    tar_joint_2, tar_joint_3, tar_joint_4 = inverse_kinematics(tar_x, tar_y, tar_yaw)
    sp_mode_init()     
    sp_move(tar_joint_2, tar_joint_3, tar_joint_4, speed)

    
def pvt_mode():
    try:
        tar_joint_2 = float(entry_tar_joint_2.get())
        tar_joint_3 = float(entry_tar_joint_3.get())
        tar_joint_4 = float(entry_tar_joint_4.get())
        tar_joint_4 = tar_joint_4 * -1.0
        speed = int(entry_speed.get())
        
        pulse2 = stepper_degrees_to_pulses(tar_joint_2)
        pulse3 = stepper_degrees_to_pulses(tar_joint_3)
        pulse4 = stepper_degrees_to_pulses(tar_joint_4)
        
        if pulse2 >= 0:
            speed2 = speed
        else:
            speed2 = -speed
            
        if pulse3 >= 0:
            speed3 = speed
        else:
            speed3 = -speed
        
        if pulse4 >= 0:
            speed4 = speed
        else:
            speed4 = -speed
        
        pulse2_le = int_to_little_endian(abs(pulse2))
        pulse3_le = int_to_little_endian(abs(pulse3))
        pulse4_le = int_to_little_endian(abs(pulse4))
        speed2_le = int_to_little_endian(speed2)
        speed3_le = int_to_little_endian(speed3)
        speed4_le = int_to_little_endian(speed4)
        time_le = int_to_little_endian(2000)
    except ValueError:
        print("Please enter valid numbers for angles.")
    
    # Set working mode to pvt mode: 
    for id in [ID2, ID3, ID4]:
        _,ret = can_tx_rx(id, SET_1_BYTE, OD_STEPPER_OPERATION_MODE, 0x00, 0x02)
        print(f"{id:03X} operation mode is {ret}")  

    # Set max pvt point 
    for id in [ID2, ID3, ID4]:
        _,ret = can_tx_rx(id, SET_2_BYTE, OD_STEPPER_PVT_MOTION, 0x03, 400)
        print(f"{id:03X} max pvt OD_STEPPER is {ret}")  
    
    # Set pvt mode 0,1,2
    for id in [ID2, ID3, ID4]:
        _,ret = can_tx_rx(id, SET_1_BYTE, OD_STEPPER_PVT_MOTION, 0x02, 0x00)
        print(f"{id:03X} pvt mode is mode {ret+1}")  
    
    #set pvt control operation to 2
    for id in [ID2, ID3, ID4]:
        _,ret = can_tx_rx(id, SET_1_BYTE, OD_STEPPER_PVT_MOTION, 0x01, 0x02)
        print(f"{id:03X} pvt control operation is {ret}")  

    #set position
    for id, pulse in zip([ID2, ID3, ID4], [pulse2, pulse3, pulse4]):
        _,ret = can_tx_rx(id, SET_4_BYTE, OD_STEPPER_PVT_MOTION, 0x11, pulse)
        print(f"{id:03X} pvt position is {ret}")  
    
    # Set speed
    for id, speed in zip([ID2, ID3, ID4], [speed2, speed3, speed4]):
        _,ret = can_tx_rx(id, SET_4_BYTE, OD_STEPPER_PVT_MOTION, 0x12, speed)
        print(f"{id:03X} pvt speed is {ret}")  
    
    #set time
    for id in [ID2, ID3, ID4]:
       _,ret = can_tx_rx(id, SET_4_BYTE, OD_STEPPER_PVT_MOTION, 0x13, 2000)
       print(f"{id:03X} pvt time is {ret}")  
    
    #set operation mode to 1
    for id in [ID2, ID3, ID4]:
        _,ret = can_tx_rx(id, SET_1_BYTE, OD_STEPPER_PVT_MOTION, 0x01, 0x01)
        print(f"{id:03X} pvt control operation is {ret}")  
    
    # read group id
    for id in [ID2, ID3, ID4]:
        _,ret = can_tx_rx(id, SET_1_BYTE, OD_STEPPER_GROUP_ID, 0x00, 0x05)
        print(f"{id:03X} group id is {ret}")  

    #send sync move
    send_can_command(f"000#0B05")
    print(f"000#0B05")  
    

        
def pp_mode_set_acceleration(accel_2, accel_3, accel_4):
    for id, accel in zip([ID2, ID3, ID4], [accel_2, accel_3, accel_4]):
        can_tx_2(id, SET_4_BYTE, OD_STEPPER_PP_MOTION, 0x01, accel)
        
def pp_mode_set_deceleration(decel_2, decel_3, decel_4):
    for id, decel in zip([ID2, ID3, ID4], [decel_2, decel_3, decel_4]):
        can_tx_2(id, SET_4_BYTE, OD_STEPPER_PP_MOTION, 0x02, decel)

def pp_mode_set_start_speed(start_speed):
    for id in [ID2, ID3, ID4]:
        can_tx_2(id, SET_4_BYTE, OD_STEPPER_PP_MOTION, 0x03, start_speed)

def pp_mode_set_stop_speed(stop_speed):
    for id in [ID2, ID3, ID4]:
        can_tx_2(id, SET_4_BYTE, OD_STEPPER_PP_MOTION, 0x04, stop_speed)
        
def pp_mode_read_status():
    status_2 = can_tx_2(ID2, READ_REQ, OD_STEPPER_PP_MOTION_2, 0x02, 0x00)
    status_3 = can_tx_2(ID3, READ_REQ, OD_STEPPER_PP_MOTION_2, 0x02, 0x00)
    status_4 = can_tx_2(ID4, READ_REQ, OD_STEPPER_PP_MOTION_2, 0x02, 0x00)
    return status_2, status_3, status_4

def get_bit(value, bit_position):
    return (value >> bit_position) & 1

def pp_mode_is_reached():
    status_2, status_3, status_4 = pp_mode_read_status()
    reach_2 = get_bit(status_2, 10)
    reach_3 = get_bit(status_3, 10)
    reach_4 = get_bit(status_4, 10)
        
    return reach_2, reach_3, reach_4
    
def pp_mode_set_max_speed(max_speed):
    for id in [ID2, ID3, ID4]:
        can_tx_2(id, SET_4_BYTE, OD_STEPPER_PP_MOTION_2, 0x03, max_speed)
        
def pp_mode_set_tar_pulse(tar_pulse_2, tar_pulse_3, tar_pulse_4):
    for id, pulse in zip([ID2, ID3, ID4], [tar_pulse_2, tar_pulse_3, -tar_pulse_4]):
        can_tx_2(id, SET_4_BYTE, OD_STEPPER_PP_MOTION_2, 0x04, pulse)
        
def pp_mode_start_absolute_motion():
    for id in [ID2, ID3, ID4]:
        send_can_command(f"{id:03X}#2b2e600150000000")

def pp_mode_start_realtive_motion():
    for id in [ID2, ID3, ID4]:
        send_can_command(f"{id:03X}#2b2e600110000000")
        

def init_motor_enable(enable):
    for id in [ID2, ID3, ID4]:
        can_tx_2(id, SET_1_BYTE, OD_STEPPER_MOTOR_ENABLE, 0x00, enable)

def init_torque_ring_enable(enable):
    for id in [ID2, ID3, ID4]:
        can_tx_2(id, SET_1_BYTE, OD_STEPPER_TORQUE_RING_ENABLE, 0x00, enable)
        
def init_set_max_current(max_current):
    for id in [ID2, ID3, ID4]:
        can_tx_2(id, SET_2_BYTE, OD_STEPPER_MAX_PHASE_CURRENT, 0x00, max_current) # the unit is in miliAmpere (mA) (0-6000)
        
def init_microstepping(sub_division):
    for id in [ID2, ID3, ID4]:
        can_tx_2(id, SET_2_BYTE, OD_STEPPER_MICROSTEPPING, 0x00, sub_division)

def init_operation_mode(mode):
    for id in [ID2, ID3, ID4]:
        can_tx_2(id, SET_1_BYTE, OD_STEPPER_OPERATION_MODE, 0x00, mode)
        
def emergency_stop_stepping():
    for id in [ID2, ID3, ID4]:
        can_tx_2(id, SET_1_BYTE, OD_STEPPER_STOP_STEPPING, 0x00, 0)

def init_change_baudrate(baudrate_option):
    for id in [ID2, ID3, ID4]:
        can_tx_2(id, SET_1_BYTE, OD_STEPPER_BAUD_RATE, 0x00, baudrate_option)

def reset_communication():
    for id in [0x02, 0x03, 0x04]:
        send_can_command(f"000#82{id:02X}")
        
# to do list:
# - bikin fungsi pengaman joint -> OK
# - bikin input form buat target xyz -> OK
# - buat fungsi move xyz -> OK
# - bikin program dancing -> OK
# - ganti baudrate jadi 1Mbps -> OK
# - nyoba sp mode burst target
# - bikin fungsi yg bisa baca data dari file semacam gcode
# - bikin unity

def is_reached():
    ret_2, ret_3, ret_4 = pp_mode_is_reached()
    print(f"{ret_2}, {ret_3}, {ret_4}")

        
def pp_mode_init():
    print(f"start pp mode initialization")
    
    init_motor_enable(1)
    init_torque_ring_enable(1)
    init_set_max_current(1500)
    init_microstepping(16)
    init_operation_mode(4)

    pp_mode_set_acceleration(8192, 8192, 8192)
    pp_mode_set_deceleration(8192, 8192, 8192)
    pp_mode_set_start_speed(150)
    pp_mode_set_stop_speed(150)
    
    print(f"the motors is in pp mode")

    
def check_limit(tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4):
    
    #limit2 = 0 to 178 degree
    #limit3 = -135 to 0 degree
    #limit4 = 0 to 196 degree
    
    if tar_joint_1 > (13004):
        tar_joint_1 = 13004
        print(f"tar_joint_1 greater than {tar_joint_1}")
    elif tar_joint_1 < 0:
        tar_joint_1 = 0
        print(f"tar_joint_1 lower than {tar_joint_1}")
    
    if tar_joint_2 > (178 * 5):
        tar_joint_2 = 178 * 5
        print(f"tar_joint_2 greater than {tar_joint_2}")
    elif tar_joint_2 < 0:
        tar_joint_2 = 0
        print(f"tar_joint_2 lower than {tar_joint_2}")
        
    if tar_joint_3 > (0 + tar_joint_2):
        tar_joint_3 = (0 + tar_joint_2)
        print(f"tar_joint_3 greater than {tar_joint_3}")
    elif tar_joint_3 < ((-135 * 5) + tar_joint_2):
        tar_joint_3 = (-135 * 5) + tar_joint_2
        print(f"tar_joint_3 lower than {tar_joint_3}")
        
    if tar_joint_4 > ((196 * 5)+tar_joint_3):
        tar_joint_4 = (196 * 5) + tar_joint_3
        print(f"tar_joint_4 greater than {tar_joint_4}")
    elif tar_joint_4 < (0 + tar_joint_3):
        tar_joint_4 = 0 + tar_joint_3
        print(f"tar_joint_4 lower than {tar_joint_4}")
    
    return tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4
    
def triangle_trajectory(cur_joint_2, cur_joint_3, cur_joint_4, tar_joint_2, tar_joint_3, tar_joint_4, travel_time, max_speed):
    
    tar_pulse_2 = stepper_degrees_to_pulses(tar_joint_2)
    tar_pulse_3 = stepper_degrees_to_pulses(tar_joint_3)
    tar_pulse_4 = stepper_degrees_to_pulses(tar_joint_4)
    
    delta_pulse_2 = stepper_degrees_to_pulses(tar_joint_2 - cur_joint_2)
    delta_pulse_3 = stepper_degrees_to_pulses(tar_joint_3 - cur_joint_3)
    delta_pulse_4 = stepper_degrees_to_pulses(tar_joint_4 - cur_joint_4)
    
    accel_decel_2 = stepper_accel_decel_calc(delta_pulse_2, travel_time)
    accel_decel_3 = stepper_accel_decel_calc(delta_pulse_3, travel_time)
    accel_decel_4 = stepper_accel_decel_calc(delta_pulse_4, travel_time)
    
    pp_mode_set_acceleration(accel_decel_2, accel_decel_3, accel_decel_4)
    pp_mode_set_deceleration(accel_decel_2, accel_decel_3, accel_decel_4)
    pp_mode_set_max_speed(max_speed)
    pp_mode_set_tar_pulse(tar_pulse_2, tar_pulse_3, tar_pulse_4)
    pp_mode_start_absolute_motion()
    
    
def pp_mode():
    
    cur_joint_1, cur_joint_2, cur_joint_3, cur_joint_4 = read_present_position()
    
    try:
        tar_joint_1 = float(entry_tar_joint_1.get())
        tar_joint_2 = float(entry_tar_joint_2.get())
        tar_joint_3 = float(entry_tar_joint_3.get())
        tar_joint_4 = float(entry_tar_joint_4.get())
        max_speed = int(entry_speed.get())
        travel_time = int(entry_time.get())
    except ValueError:
        print("Please enter valid numbers for angles.")
        
    tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4 = check_limit(tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4)
        
    triangle_trajectory(cur_joint_2, cur_joint_3, cur_joint_4, tar_joint_2, tar_joint_3, tar_joint_4, travel_time, max_speed)
    
def inverse_kinematics(x, y, z, yaw):
    # max area
    L2 = 137.0
    L3 = 121.0
    L4 = 56.82
    OFFSET_2 = -96.5
    OFFSET_3 = 134
    OFFSET_4 = -52.5
    #ration joint = 5:1

    distance = math.sqrt(x**2 + y**2)

    if distance > (L2 + L3):
        raise ValueError("out of boundary")

    # joint_3
    cos_theta3 = (x**2 + y**2 - L2**2 - L3**2) / (2 * L2 * L3)
    theta3 = math.acos(cos_theta3)  #angle in radian

    # joint_2
    k1 = L2 + L3 * cos_theta3
    k2 = L3 * math.sin(theta3)
    theta2 = math.atan2(y, x) - math.atan2(k2, k1)

    # rad to deg
    theta2 = math.degrees(theta2)
    theta3 = math.degrees(theta3)

    joint_1 = z * (360/90)
    joint_2 = (theta2-OFFSET_2)*5
    joint_3 = (theta3-OFFSET_3)*5 + joint_2
    joint_4 = (yaw-OFFSET_4)*5# + joint_3;
    
    if joint_1 > (13004):
        joint_1 = 13004
       # raise ValueError("out of joint_1 max limit")
    elif joint_1 < 0:
        joint_1 = 0
    if joint_2 > (178 * 5):
        joint_2 = 178 * 5
       # raise ValueError("out of joint_2 max limit")
    elif joint_2 < 0:
        joint_2 = 0
        #raise ValueError("out of joint_2 min limit")
    if joint_3 > (0 + joint_2):
        joint_3 = (0 + joint_2)
        #raise ValueError("out of joint_3 max limit")
    elif joint_3 < ((-135 * 5) + joint_2):
        joint_3 = (-135 * 5) + joint_2
        #raise ValueError("out of joint_2 min limit")
    if joint_4 > ((196 * 5) + joint_3):
        joint_4 = (196 * 5) + joint_3
        #raise ValueError("out of joint_4 max limit")
    elif joint_4 < (0 + joint_3):
        joint_4 = (0 + joint_3)
        #raise ValueError("out of joint_2 min limit")

    return joint_1, joint_2, joint_3, joint_4


def move_xyyaw(tar_x, tar_y, tar_yaw, travel_time, max_speed):
    tar_z = 0
    cur_joint_1, cur_joint_2, cur_joint_3, cur_joint_4 = read_present_position()
    tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4 = inverse_kinematics(tar_x, tar_y, tar_z, tar_yaw)
    tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4 = check_limit(tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4)
    print(f"tar joint = {tar_joint_2}, {tar_joint_3}, {tar_joint_4} degree")
    triangle_trajectory(cur_joint_2, cur_joint_3, cur_joint_4, tar_joint_2, tar_joint_3, tar_joint_4, travel_time, max_speed)
    
def move_degree(tar_joint_2, tar_joint_3, tar_joint_4, travel_time, max_speed):
    cur_joint_1, cur_joint_2, cur_joint_3, cur_joint_4 = read_present_position()
    tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4 = check_limit(tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4)
    print(f"tar joint = {tar_joint_2}, {tar_joint_3}, {tar_joint_4} degree")
    triangle_trajectory(cur_joint_2, cur_joint_3, cur_joint_4, tar_joint_2, tar_joint_3, tar_joint_4, travel_time, max_speed)
    
def move_coor():  
    try:
        tar_x = float(entry_tar_x.get())
        tar_y = float(entry_tar_y.get())
        tar_yaw = float(entry_tar_yaw.get())
        max_speed = int(entry_speed.get())
        travel_time = int(entry_time.get())
    except ValueError:
        print("Please enter valid numbers for angles.")
    
    move_xyyaw(tar_x, tar_y, tar_yaw, travel_time, max_speed)
    
        
    
    
    
def shutdown():
    emergency_stop_stepping()
    init_motor_enable(0)  
    init_torque_ring_enable(0)  
    init_set_max_current(0)
    
    # motor_1_shutdown()
    print(f"motor shutdown")
    
def calib_0():
    enc1, enc2, enc3, enc4 = encoder_position()
    for id, enc in zip([ID2, ID3, ID4], [enc2, enc3, enc4]):
        can_tx_2(id, SET_4_BYTE, OD_STEPPER_CALIBRATION_ZERO, 0x00, -enc)
    
    save_settings()

def straight_line():
    try:
        travel_time = int(entry_time.get())
        max_speed = int(entry_speed.get())
        sleep = (travel_time / 1000)
        
        for i in range(3):
            move_xyyaw(258,0,0, travel_time, max_speed)
            time.sleep(sleep)
            move_xyyaw(150,0,0, travel_time, max_speed)
            time.sleep(sleep)
    except ValueError:
        print("Please enter valid numbers for time and speed.")

	
	
def rectangular():
    try:
        travel_time = int(entry_time.get())
        max_speed = int(entry_speed.get())
        sleep = (travel_time / 1000) + 0.1
        
        for i in range(2):
            move_xyyaw(210, 40, 0, travel_time, max_speed)
            time.sleep(sleep)
            move_xyyaw(210, -40, 0, travel_time, max_speed)
            time.sleep(sleep)
            move_xyyaw(130, -40, 0, travel_time, max_speed)
            time.sleep(sleep)
            move_xyyaw(130, 40, 0, travel_time, max_speed)
            time.sleep(sleep)
        
        move_xyyaw(210, 40, 0, travel_time, max_speed)
        time.sleep(sleep)

    except ValueError:
        print("Please enter valid numbers for time and speed.")
        
def home_position():
    try:
        travel_time = int(entry_time.get())
        max_speed = int(entry_speed.get())
        sleep = (travel_time / 1000)
        
        move_degree(0,0,0, travel_time, max_speed)
        time.sleep(sleep)
        #time.sleep(sleep)


    except ValueError:
        print("Please enter valid numbers for time and speed.")
		
def shuttle_position():
    try:
        travel_time = int(entry_time.get())
        max_speed = int(entry_speed.get())
        sleep = (travel_time / 1000)
        
        move_xyyaw(166.82, -168, 0, travel_time, max_speed)
        time.sleep(sleep)

    except ValueError:
        print("Please enter valid numbers for time and speed.")

def pre_past_shelf():
    try:
        travel_time = int(entry_time.get())
        max_speed = int(entry_speed.get())
        sleep = (travel_time / 1000)
        
        move_xyyaw(107, 100, 90, travel_time, max_speed)
        time.sleep(sleep)
        #time.sleep(sleep)


    except ValueError:
        print("Please enter valid numbers for time and speed.")
		
def pickup_from_shelf():
    try:
        travel_time = int(entry_time.get())
        max_speed = int(entry_speed.get())
        sleep = (travel_time / 1000) + 0.2
        
        move_xyyaw(107, 224, 90, travel_time, max_speed)
        time.sleep(sleep)

    except ValueError:
        print("Please enter valid numbers for time and speed.")
		
def place_onto_shelf():
    try:
        travel_time = int(entry_time.get())
        max_speed = int(entry_speed.get())
        sleep = (travel_time / 1000)
        
        move_xyyaw(107, 197, 90, travel_time, max_speed)
        time.sleep(sleep)

    except ValueError:
        print("Please enter valid numbers for time and speed.")
            
def dancing():
    try:
        #while True:
        travel_time = int(entry_time.get())
        max_speed = int(entry_speed.get())
        sleep = (travel_time / 1000)
    
        straight_line()
        move_xyyaw(140, 0, -20, travel_time, max_speed)
        time.sleep(sleep)
        #move_xyc(140,0,0)
        #time.sleep(sleep)
        #move_xyc(140,0,45)
        #time.sleep(sleep)
        move_xyyaw(140, 0, 90, travel_time, max_speed)
        time.sleep(sleep)
        move_xyyaw(140, -168, 0, travel_time, max_speed)
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
        move_xyyaw(150, 0, 0, travel_time, max_speed)
        time.sleep(sleep)
        move_xyyaw(258, 0, 90, travel_time, max_speed)
        time.sleep(sleep)
        move_xyyaw(258, 0, -90, travel_time, max_speed)
        time.sleep(sleep)
        move_xyyaw(258, 0, 0, travel_time, max_speed)
            
    except ValueError:
        print("Please enter valid numbers for time and speed.")


# Mode of Operation Definition
MOTOR_1_OPERATION_MODE = {
	0: "not defined",
	1: "position mode",
	2: "pulse direction mode",
	3: "velocity mode",
	4: "torque mode (servo)",
	5: "not defined",
    6: "homing mode"
	}

def decode_opeation_mode(operation_mode):
    print("Decoded operation mode:")
    for bit, description in MOTOR_1_OPERATION_MODE.items():
        if operation_mode == bit:
            print(f"  - {description}")

def motor_1_read_operation_mode():
    operation_mode = can_tx_2(ID1, READ_REQ, OD_SERVO_MODE_OF_OPERATION, 0x00,  0x00)
    decode_opeation_mode(operation_mode)

def motor_1_shutdown():
    can_tx_2(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x06)

def motor_1_switch_on():
    can_tx_2(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x07)
    
def motor_1_set_operation_mode(operation_mode):
    can_tx_2(ID1, SET_1_BYTE, OD_SERVO_MODE_OF_OPERATION, 0x00,  operation_mode)
    
def motor_1_init():
    motor_1_switch_on()
    motor_1_set_operation_mode(1)
    motor_1_read_operation_mode()
    

def motor_1_set_acceleration(accel_1):
    can_tx_2(ID1, SET_2_BYTE, OD_SERVO_ACCELERATION, 0x00,  accel_1)
    
def motor_1_set_deceleration(decel_1):
    can_tx_2(ID1, SET_2_BYTE, OD_SERVO_DECELERATION, 0x00,  decel_1)
    
def motor_1_set_max_speed(max_speed):
    can_tx_2(ID1, SET_4_BYTE, OD_SERVO_TARGET_VELOCITY, 0x00,  max_speed)
    
def motor_1_set_tar_pulse(tar_pulse_1):
    can_tx_2(ID1, SET_4_BYTE, OD_SERVO_TARGET_POSITION, 0x00,  tar_pulse_1)

def motor_1_position_mode():
    try:
        tar_joint_1 = float(entry_tar_joint_1.get())
        tar_joint_2 = float(entry_tar_joint_2.get())
        tar_joint_3 = float(entry_tar_joint_3.get())
        tar_joint_4 = float(entry_tar_joint_4.get())
        max_speed = int(entry_speed.get())
        travel_time = int(entry_time.get())
        
        cur_joint_1, cur_joint_2, cur_joint_3, cur_joint_4 = read_present_position()
        tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4 = check_limit(tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4)
        tar_pulse_1 = servo_degrees_to_pulses(tar_joint_1)
    except ValueError:
        print("Please enter valid numbers for angles.")
    
    can_tx_2(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x0F)
    motor_1_set_max_speed(10)
    motor_1_set_acceleration(100)
    motor_1_set_deceleration(100)
    motor_1_set_tar_pulse(tar_pulse_1)
    can_tx_2(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x1F)
    
    #print(f"position mode")


def triangle_trajectory_with_motor_1(cur_joint_1, cur_joint_2, cur_joint_3, cur_joint_4, tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4, travel_time, max_speed):
    
    tar_pulse_1 = servo_degrees_to_pulses(tar_joint_1)
    tar_pulse_2 = stepper_degrees_to_pulses(tar_joint_2)
    tar_pulse_3 = stepper_degrees_to_pulses(tar_joint_3)
    tar_pulse_4 = stepper_degrees_to_pulses(tar_joint_4)
    
    delta_pulse_1 = servo_degrees_to_pulses(tar_joint_1 - cur_joint_1)
    delta_pulse_2 = stepper_degrees_to_pulses(tar_joint_2 - cur_joint_2)
    delta_pulse_3 = stepper_degrees_to_pulses(tar_joint_3 - cur_joint_3)
    delta_pulse_4 = stepper_degrees_to_pulses(tar_joint_4 - cur_joint_4)
    
    accel_decel_1 = servo_accel_decel_calc(delta_pulse_1, travel_time)
    accel_decel_2 = stepper_accel_decel_calc(delta_pulse_2, travel_time)
    accel_decel_3 = stepper_accel_decel_calc(delta_pulse_3, travel_time)
    accel_decel_4 = stepper_accel_decel_calc(delta_pulse_4, travel_time)
    
    pp_mode_set_acceleration(accel_decel_2, accel_decel_3, accel_decel_4)
    pp_mode_set_deceleration(accel_decel_2, accel_decel_3, accel_decel_4)
    pp_mode_set_max_speed(max_speed)
    pp_mode_set_tar_pulse(tar_pulse_2, tar_pulse_3, tar_pulse_4)
    
    can_tx_2(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x0F)
    motor_1_max_speed_ppr = (int)((max_speed / SERVO_PPR) * 10)
    motor_1_set_acceleration(accel_decel_1)
    motor_1_set_deceleration(accel_decel_1)
    motor_1_set_max_speed(motor_1_max_speed_ppr)
    motor_1_set_tar_pulse(tar_pulse_1)
    
    # pp_mode_start_absolute_motion()
    can_tx_2(ID1, SET_2_BYTE, OD_SERVO_CONTROL_WORD, 0x00,  0x1F)

def all_position_mode_init():
    motor_1_init()
    pp_mode_init()
 
def homing():
    
    #pp_mode_init()
    cur_joint_1, cur_joint_2, cur_joint_3, cur_joint_4 = read_present_position()
    
    try:
        tar_joint_1 = 0.0
        tar_joint_2 = 0.0
        tar_joint_3 = 0.0
        tar_joint_4 = 0.0
        max_speed = int(entry_speed.get())
        travel_time = int(entry_time.get())
    except ValueError:
        print("Please enter valid numbers for angles.")
    
    tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4 = check_limit(tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4)
        
    # triangle_trajectory(cur_joint_2, cur_joint_3, cur_joint_4, tar_joint_2, tar_joint_3, tar_joint_4, travel_time, max_speed)
    triangle_trajectory_with_motor_1(cur_joint_1, cur_joint_2, cur_joint_3, cur_joint_4, tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4, travel_time, max_speed)

 
def all_position_mode():
    cur_joint_1, cur_joint_2, cur_joint_3, cur_joint_4 = read_present_position()
    
    try:
        tar_joint_1 = float(entry_tar_joint_1.get())
        tar_joint_2 = float(entry_tar_joint_2.get())
        tar_joint_3 = float(entry_tar_joint_3.get())
        tar_joint_4 = float(entry_tar_joint_4.get())
        max_speed = int(entry_speed.get())
        travel_time = int(entry_time.get())
    except ValueError:
        print("Please enter valid numbers for angles.")
        
    tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4 = check_limit(tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4)
        
    triangle_trajectory_with_motor_1(cur_joint_1, cur_joint_2, cur_joint_3, cur_joint_4, tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4, travel_time, max_speed)
    

def on_closing():
    print("closing")
    shutdown()
    root.destroy()


def signal_handler(sig, frame):
    on_closing()
    exit(0)  # Keluar dari program dengan status 0


# Menangkap sinyal Ctrl+C (SIGINT)
signal.signal(signal.SIGINT, signal_handler)
        	
# q3
root = tk.Tk()
root.title("Motor Control Panel")

error_status_button = tk.Button(root, text="error status", command=error_status)
error_status_button.grid(row=0, column=0, columnspan=1, pady=10,sticky="ew", padx=5)

save_settings_button = tk.Button(root, text="save settings", command=save_settings)
save_settings_button.grid(row=0, column=1, columnspan=1, pady=10, sticky="ew", padx=5)

#versatile_button = tk.Button(root, text="versatile", command=versatile)
#versatile_button.grid(row=1, column=0, columnspan=3, pady=10)




tk.Label(root, text="Enter max speed:").grid(row=1, column=0, padx=5, pady=5, sticky="ew")
entry_speed = tk.Entry(root)
entry_speed.insert(0, "1000")
entry_speed.grid(row=1, column=1, padx=5, pady=5, sticky="ew")

tk.Label(root, text="Enter time:").grid(row=2, column=0, padx=5, pady=5, sticky="ew")
entry_time = tk.Entry(root)
entry_time.insert(0, "4000")
entry_time.grid(row=2, column=1, padx=5, pady=5, sticky="ew")


# pp_mode_init_button = tk.Button(root, text="pp mode init", command=pp_mode_init)
# pp_mode_init_button.grid(row=3, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

all_position_mode_init_button = tk.Button(root, text="all position init", command=all_position_mode_init)
all_position_mode_init_button.grid(row=3, column=0, columnspan=1, pady=10, padx=5, sticky="ew")



shutdown_button = tk.Button(root, text="shutdown", bg="red", fg="white", command=shutdown)
shutdown_button.grid(row=3, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

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
entry_tar_joint_4.insert(0, "262.5")
entry_tar_joint_4.grid(row=7, column=1, padx=5, pady=5, sticky="ew")

#absolute_position_button = tk.Button(root, text="absolute position", command=absolute_position)
#absolute_position_button.grid(row=7, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

#relative_position_button = tk.Button(root, text="relative position", command=relative_position)
#relative_position_button.grid(row=7, column=1, columnspan=1, pady=10, padx=5, sticky="ew")





pp_mode_button = tk.Button(root, text="all_position_mode", command=all_position_mode)
pp_mode_button.grid(row=11, column=0, columnspan=2, pady=10, padx=5, sticky="ew")



# stall_on_button = tk.Button(root, text="stall on", command=stall_on)
# stall_on_button.grid(row=12, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

# stall_off_button = tk.Button(root, text="stall off", command=stall_off)
# stall_off_button.grid(row=12, column=1, columnspan=1, pady=10, padx=5, sticky="ew")


# pvt_mode_button = tk.Button(root, text="pvt mode", command=pvt_mode)
# pvt_mode_button.grid(row=13, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

tk.Label(root, text="coor x (mm):").grid(row=14, column=0, padx=5, pady=5, sticky="ew")
entry_tar_x = tk.Entry(root)
entry_tar_x.insert(0, "258")
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

move_coor_button = tk.Button(root, text="move coor", command=move_coor)
move_coor_button.grid(row=18, column=0, columnspan=2, pady=10, padx=5, sticky="ew")

motor_position_button = tk.Button(root, text="motor position", bg="orange",fg="black", command=read_present_position)
motor_position_button.grid(row=19, column=0, columnspan=1, pady=10, padx=5, sticky="ew")



is_reached_button = tk.Button(root, text="is reached",bg="orange",fg="black", command=is_reached)
is_reached_button.grid(row=19, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

homing_button = tk.Button(root, text="homing", command=homing)
homing_button.grid(row=20, column=0, columnspan=2, pady=10, padx=5, sticky="ew")

# dancing_button = tk.Button(root, text="dancing", command=dancing)
# dancing_button.grid(row=22, column=0, columnspan=2, pady=10, padx=5, sticky="ew")

# encoder_position_button = tk.Button(root, text="encoder position", command=encoder_position)
# encoder_position_button.grid(row=21, column=0, columnspan=1, pady=10, padx=5, sticky="ew")

# calib_0_button = tk.Button(root, text="calib_0", command=calib_0)
# calib_0_button.grid(row=21, column=1, columnspan=1, pady=10, padx=5, sticky="ew")

# sp_mode_button = tk.Button(root, text="sp mode", command=sp_mode)
# sp_mode_button.grid(row=23, column=0, columnspan=2, pady=10, padx=5, sticky="ew")

# sp_mode_button = tk.Button(root, text="sp mode", command=sp_mode)
# sp_mode_button.grid(row=23, column=0, columnspan=2, pady=10, padx=5, sticky="ew")

motor_1_position_mode_button = tk.Button(root, text="position mode", command=motor_1_position_mode)
motor_1_position_mode_button.grid(row=24, column=0, columnspan=2, pady=10, padx=5, sticky="ew")

root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
bus.shutdown()


