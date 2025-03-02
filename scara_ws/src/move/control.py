import time
import os
import tkinter as tk
from tkinter import simpledialog, messagebox
import os
import time

# Conversion factor based on 360 degrees = 4000 pulses
current_d=[-96.5,134,-90]

DEGREES_TO_PULSES = 4000 / 360

def send_can_command(command):
    print(f"Sending CAN command: {command}")
    os.system(f'cansend can0 {command}')
    return []

def degrees_to_pulses(degrees):
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
    try:
        accel_time = int(entry_accel_time.get())
        decel_time = int(entry_decel_time.get())
        max_speed = int(entry_max_speed.get())

        accel_command_le = convert_to_little_endian(accel_time)
        decel_command_le = convert_to_little_endian(decel_time)
        speed_command_le = convert_to_little_endian(max_speed)

        if reverse_var.get():
            max_speed = max_speed
            speed_command_le = convert_to_little_endian_signed(max_speed)
        
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
            send_can_command(f"{node}#23816000{speed_command_le}")  # Set maximum speed (index 6081)
            time.sleep(0.5)
            send_can_command(f"{node}#2F60600001000000")  # Switch to position mode (index 6060)
            time.sleep(0.5)
        
        messagebox.showinfo("Initialize Motors", "Motors have been initialized and are ready for operation.")
    except ValueError:
        messagebox.showerror("Invalid Input", "Please enter valid numerical values for acceleration, deceleration, and speed.")

def shutdown_motors():
    for node in ['602', '603', '604']:
        send_can_command(f"{node}#2B40600000000000")  # Shutdown command
        time.sleep(0.5)
    messagebox.showinfo("Shutdown Motors", "Motors have been shut down properly.")

def send_position_command(is_relative):
    global current_d
    try:
        degrees1 = float(entry_motor1.get())
        degrees2 = float(entry_motor2.get())
        degrees3 = -1.0*float(entry_motor3.get())

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

        for node, pulse_command_le in zip(['602', '603', '604'], [pulse_command1_le, pulse_command2_le, pulse_command3_le]):
            send_can_command(f"{node}#2B406000{start_command}000000")  # Prepare for motion
            time.sleep(0.001)
            send_can_command(f"{node}#237A6000{pulse_command_le}")  # Set target position using index 607A
            time.sleep(0.001)
        for node in ['602', '603', '604']:
            send_can_command(f"{node}#2B406000{execute_command}000000")  # Execute motion
            time.sleep(0.001)
        
        messagebox.showinfo("Send Position Command", "Position commands sent to all motors.")
    except ValueError:
        messagebox.showerror("Invalid Input", "Please enter valid numbers for angles.")

    current_d= [degrees1-96.5, degrees2+134, degrees3-90]

def send_relative_position():
    send_position_command(is_relative=True)


def send_absolute_position():
    send_position_command(is_relative=False)

def send_position_command_xy(is_relative, degrees):
    global current_d
    degrees1= degrees[0]+96.5
    degrees2= degrees[1]-134
    degrees3= degrees[2]+90
    try:
        degrees1 = float(degrees1)
        degrees2 = float(degrees1)
        degrees3 = -1.0*float(degrees1)
        print(degrees1, degrees2, degrees2)
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

        for node, pulse_command_le in zip(['602', '603', '604'], [pulse_command1_le, pulse_command2_le, pulse_command3_le]):
            send_can_command(f"{node}#2B406000{start_command}000000")  # Prepare for motion
            time.sleep(0.001)
            send_can_command(f"{node}#237A6000{pulse_command_le}")  # Set target position using index 607A
            time.sleep(0.001)
        for node in ['602', '603', '604']:
            send_can_command(f"{node}#2B406000{execute_command}000000")  # Execute motion
            time.sleep(0.001)
        
        #messagebox.showinfo("Send Position Command", "Position commands sent to all motors.")
    except ValueError:
        messagebox.showerror("Invalid Input", "Please enter valid numbers for angles.")

    current_d = [degrees1-96.5, degrees2+134, degrees3-90]


def Home():
    send_position_command_xy(is_relative=False, degrees=[0,0,0])
    messagebox.showinfo("Homing", "Home done.")

def MoveL():
    global current_d
    x= float(entry_x_pos.get())
    y= float(entry_y_pos.get())
    target= [x, y, 0]
    print(target)
    current= FK(current_d[0], current_d[1], current_d[2])
    s= linear(target, current)
    for i in s:
        i = IK(i)
        send_position_command_xy(is_relative=False, degrees=i)
        time.sleep(0.5)

def Move():
    x= float(entry_x_pos.get())
    y= float(entry_y_pos.get())
    i = IK([x, y, 0])
    print(i)
    send_position_command_xy(is_relative=False, degrees=i)

def on_closing():
    if messagebox.askokcancel("Quit", "Do you want to quit and shut down all motors?"):
        shutdown_motors()
        root.destroy()


root = tk.Tk()
root.title("Motor Control Panel")

initialize_button = tk.Button(root, text="Initialize Motors", command=initialize_motors)
initialize_button.grid(row=0, column=0, columnspan=3, pady=10)

# Input fields for x, y positions
tk.Label(root, text="x position in mm").grid(row=1, column=0, padx=5, pady=5)
entry_x_pos = tk.Entry(root)
entry_x_pos.insert(0, "100")
entry_x_pos.grid(row=1, column=1, padx=5, pady=5)

tk.Label(root, text="y position in mm").grid(row=2, column=0, padx=5, pady=5)
entry_y_pos = tk.Entry(root)
entry_y_pos.insert(0, "100")
entry_y_pos.grid(row=2, column=1, padx=5, pady=5)

# Frame for buttons
frame = tk.Frame(root)
frame.grid(row=3, column=0, columnspan=3, pady=10)

Move_L = tk.Button(frame, text="Move_L", command=MoveL)
Move_L.grid(row=0, column=0, padx=5, pady=10)

Move = tk.Button(frame, text="Move", command=Move)
Move.grid(row=0, column=1, padx=5, pady=10)

Home = tk.Button(frame, text="Home", command=Home)
Home.grid(row=0, column=2, padx=5, pady=10)

shutdown_button = tk.Button(root, text="Shutdown Motors", command=shutdown_motors)
shutdown_button.grid(row=4, column=0, columnspan=3, pady=10)

# Input fields for acceleration, deceleration, and maximum speed
tk.Label(root, text="Acceleration Time (ms):").grid(row=5, column=0, padx=5, pady=5)
entry_accel_time = tk.Entry(root)
entry_accel_time.insert(0, "100")
entry_accel_time.grid(row=5, column=1, padx=5, pady=5)

tk.Label(root, text="Deceleration Time (ms):").grid(row=6, column=0, padx=5, pady=5)
entry_decel_time = tk.Entry(root)
entry_decel_time.insert(0, "100")
entry_decel_time.grid(row=6, column=1, padx=5, pady=5)

tk.Label(root, text="Maximum Speed (rpm):").grid(row=7, column=0, padx=5, pady=5)
entry_max_speed = tk.Entry(root)
entry_max_speed.insert(0, "60")
entry_max_speed.grid(row=7, column=1, padx=5, pady=5)

# Checkbox for reversing motor direction
reverse_var = tk.BooleanVar()
reverse_checkbox = tk.Checkbutton(root, text="Reverse Motor Direction", variable=reverse_var)
reverse_checkbox.grid(row=8, column=0, columnspan=3, pady=5)

# Input fields for positions
tk.Label(root, text="Enter position for Motor 1 (degrees):").grid(row=9, column=0, padx=5, pady=5)
entry_motor1 = tk.Entry(root)
entry_motor1.grid(row=9, column=1, padx=5, pady=5)

tk.Label(root, text="Enter position for Motor 2 (degrees):").grid(row=10, column=0, padx=5, pady=5)
entry_motor2 = tk.Entry(root)
entry_motor2.grid(row=10, column=1, padx=5, pady=5)

tk.Label(root, text="Enter position for Motor 3 (degrees):").grid(row=11, column=0, padx=5, pady=5)
entry_motor3 = tk.Entry(root)
entry_motor3.grid(row=11, column=1, padx=5, pady=5)

send_relative_position_button = tk.Button(root, text="Send Relative Position", command=send_relative_position)
send_relative_position_button.grid(row=12, column=0, columnspan=3, pady=10)

send_absolute_position_button = tk.Button(root, text="Send Absolute Position", command=send_absolute_position)
send_absolute_position_button.grid(row=13, column=0, columnspan=3, pady=10)

root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
