import os
import time
import tkinter as tk
import zmq

# === Endpoint ===
CMD_ENDPOINT = os.getenv("CMD_ENDPOINT", "tcp://127.0.0.1:5555")  # ubah kalau perlu

# === ZMQ PUB ===
ctx = zmq.Context.instance()
socket = ctx.socket(zmq.PUB)
socket.setsockopt(zmq.SNDHWM, 1000)
socket.bind(CMD_ENDPOINT)
time.sleep(0.1)  # slow-joiner guard

def send_command(cmd, extra=None):
    msg = {"command": cmd}
    if extra:
        msg.update(extra)
    socket.send_json(msg)
    print("Sent:", msg)
    lbl_last.config(text=f"last cmd: {cmd}")

# ===== GUI =====
root = tk.Tk()
root.title("Motor Control Panel (one-way)")

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
    e.insert(0, "0")
    e.grid(row=2+i, column=1)
    entries_joint.append(e)

# --- Coor inputs ---
labels = ["X (mm):", "Y (mm):", "Z (mm):", "Yaw (deg):"]
entries_coor = []
for i, label in enumerate(labels):
    tk.Label(root, text=label).grid(row=6+i, column=0)
    e = tk.Entry(root)
    e.insert(0, "0")
    e.grid(row=6+i, column=1)
    entries_coor.append(e)

# --- Button handlers ---
def send_pp_joint():
    data = [float(e.get() or 0) for e in entries_joint]
    send_command("pp_joint", {"joints": data, "time": int(entry_time.get() or 0)})

def send_pp_coor():
    data = [float(e.get() or 0) for e in entries_coor]
    send_command("pp_coor", {"coor": data, "time": int(entry_time.get() or 0)})

def send_pvt_joint():
    data = [float(e.get() or 0) for e in entries_joint]
    send_command("pvt_joint", {"joints": data, "time": int(entry_time.get() or 0)})

def send_pvt_coor():
    data = [float(e.get() or 0) for e in entries_coor]
    send_command("pvt_coor", {"coor": data, "time": int(entry_time.get() or 0)})

# --- Buttons (FIX: no stray indent here) ---
tk_btns = [
    ("Wake Up",       {"row":10, "col":0, "opt":{"bg":"purple", "fg":"white"}}, lambda: send_command("wake_up")),
    ("Shutdown",      {"row":10, "col":1, "opt":{"bg":"maroon", "fg":"white"}}, lambda: send_command("shutdown")),
    ("PP Joint",      {"row":11, "col":0},                                     send_pp_joint),
    ("PP Coor",       {"row":11, "col":1},                                     send_pp_coor),
    ("PVT Joint",     {"row":12, "col":0},                                     send_pvt_joint),
    ("PVT Coor",      {"row":12, "col":1},                                     send_pvt_coor),
    ("Read Position", {"row":13, "col":0, "opt":{"bg":"orange"}},              lambda: send_command("read_position")),
    ("Homing",        {"row":13, "col":1, "opt":{"bg":"cyan"}},                lambda: send_command("homing")),
    ("Stop",          {"row":14, "col":0, "span":2, "opt":{"bg":"red","fg":"white"}}, lambda: send_command("stop")),
]

for text, pos, fn in tk_btns:
    opt = pos.get("opt", {})
    span = pos.get("span", 1)
    btn = tk.Button(root, text=text, command=fn, **opt)
    btn.grid(row=pos["row"], column=pos["col"], columnspan=span, sticky="ew")

# status label
lbl_last = tk.Label(root, text=f"endpoint: {CMD_ENDPOINT}")
lbl_last.grid(row=18, column=0, columnspan=2, sticky="w")

root.mainloop()
