import tkinter as tk
import zmq

# Setup ZeroMQ Publisher
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")   # Port 5555 untuk publish data

def send_command(cmd, extra=None):
    msg = {"command": cmd}
    if extra:
        msg.update(extra)
    socket.send_json(msg)
    print("Sent:", msg)

# ===== GUI =====
root = tk.Tk()
root.title("Motor Control Panel")

motor_type = tk.StringVar(value="all")

def update_motor_selection():
    send_command("motor_selection", {"motor": motor_type.get()})

# --- Motor selection ---
radio_frame = tk.LabelFrame(root, text="Motor Selection", padx=10, pady=5)
radio_frame.grid(row=0, column=0, columnspan=2, pady=5, sticky="ew")

tk.Radiobutton(radio_frame, text="All motors", variable=motor_type, value="all",
               command=update_motor_selection).grid(row=0, column=0)
tk.Radiobutton(radio_frame, text="Stepper only", variable=motor_type, value="stepper_only",
               command=update_motor_selection).grid(row=0, column=1)
tk.Radiobutton(radio_frame, text="Servo only", variable=motor_type, value="servo_only",
               command=update_motor_selection).grid(row=0, column=2)

# --- Time input ---
tk.Label(root, text="Travel time (ms):").grid(row=1, column=0)
entry_time = tk.Entry(root)
entry_time.insert(0, "4000")
entry_time.grid(row=1, column=1)

# --- Joint inputs ---
entries_joint = []
for i in range(4):
    tk.Label(root, text=f"Joint {i+1} (deg):").grid(row=2+i, column=0)
    e = tk.Entry(root)
    e.insert(0, str(0))
    e.grid(row=2+i, column=1)
    entries_joint.append(e)

# --- Coor inputs ---
labels = ["X (mm):", "Y (mm):", "Z (mm):", "Yaw (deg):"]
entries_coor = []
for i, label in enumerate(labels):
    tk.Label(root, text=label).grid(row=6+i, column=0)
    e = tk.Entry(root)
    e.insert(0, str(0))
    e.grid(row=6+i, column=1)
    entries_coor.append(e)

# --- Button handlers ---
def send_pp_joint():
    data = [float(e.get()) for e in entries_joint]
    send_command("pp_joint", {"joints": data, "time": int(entry_time.get())})

def send_pp_coor():
    data = [float(e.get()) for e in entries_coor]
    send_command("pp_coor", {"coor": data, "time": int(entry_time.get())})

def send_pvt_joint():
    data = [float(e.get()) for e in entries_joint]
    send_command("pvt_joint", {"joints": data, "time": int(entry_time.get())})

def send_pvt_coor():
    data = [float(e.get()) for e in entries_coor]
    send_command("pvt_coor", {"coor": data, "time": int(entry_time.get())})

# --- Buttons ---
tk.Button(root, text="Wake Up", bg="purple", fg="white", 
          command=lambda: send_command("wake_up")).grid(row=10, column=0, sticky="ew")
tk.Button(root, text="Shutdown", bg="maroon", fg="white", 
          command=lambda: send_command("shutdown")).grid(row=10, column=1, sticky="ew")

tk.Button(root, text="PP Joint", command=send_pp_joint).grid(row=11, column=0, sticky="ew")
tk.Button(root, text="PP Coor", command=send_pp_coor).grid(row=11, column=1, sticky="ew")

tk.Button(root, text="PVT Joint", command=send_pvt_joint).grid(row=12, column=0, sticky="ew")
tk.Button(root, text="PVT Coor", command=send_pvt_coor).grid(row=12, column=1, sticky="ew")

tk.Button(root, text="Read Position", bg="orange", 
          command=lambda: send_command("read_position")).grid(row=13, column=0, sticky="ew")
tk.Button(root, text="Homing", bg="cyan", 
          command=lambda: send_command("homing")).grid(row=13, column=1, sticky="ew")

tk.Button(root, text="Stop", bg="red", fg="white", 
          command=lambda: send_command("stop")).grid(row=14, column=0, columnspan=2, sticky="ew")

root.mainloop()
