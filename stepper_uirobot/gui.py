"""
gui_zmq_oneway.py — Tkinter GUI (ONE-WAY only)
- Publishes JSON commands via ZeroMQ PUB
- No telemetry / no worker code here
- Same controls as before, with safer parsing & last-cmd indicator

Run:
    pip install pyzmq
    python gui_zmq_oneway.py

Endpoints:
    By default uses IPC:  CMD_ENDPOINT=ipc:///tmp/motor_cmd
    Override with env var, e.g. CMD_ENDPOINT=tcp://127.0.0.1:5555

Notes:
    • Start the receiver (worker) first to avoid losing the very first messages (slow joiner).
"""
import os
import time
import tkinter as tk
import zmq

CMD_ENDPOINT = os.getenv("CMD_ENDPOINT", "ipc:///tmp/motor_cmd")

# --- ZMQ PUB setup ---
ctx = zmq.Context.instance()
pub = ctx.socket(zmq.PUB)
pub.setsockopt(zmq.SNDHWM, 1000)
pub.bind(CMD_ENDPOINT)
# small delay so the socket is fully bound before first send
time.sleep(0.1)

# --- helpers ---
def send(cmd, **kw):
    msg = {"command": cmd}
    msg.update(kw)
    pub.send_json(msg)
    lbl_last.config(text=f"last cmd: {cmd}")
    print("Sent:", msg)

def f2(x, default=0.0):
    try:
        return float(x)
    except Exception:
        return float(default)

def i2(x, default=0):
    try:
        return int(float(x))
    except Exception:
        return int(default)

# --- GUI ---
root = tk.Tk()
root.title("Motor Control Panel (ZMQ one-way)")

# motor selection
motor_type = tk.StringVar(value="all")

def update_motor_selection():
    send("motor_selection", motor=motor_type.get())

radio = tk.LabelFrame(root, text="Motor Selection", padx=10, pady=5)
radio.grid(row=0, column=0, columnspan=2, pady=5, sticky="ew")
tk.Radiobutton(radio, text="All motors", variable=motor_type, value="all", command=update_motor_selection).grid(row=0, column=0)
tk.Radiobutton(radio, text="Stepper only", variable=motor_type, value="stepper_only", command=update_motor_selection).grid(row=0, column=1)
tk.Radiobutton(radio, text="Servo only", variable=motor_type, value="servo_only", command=update_motor_selection).grid(row=0, column=2)

# time input
tk.Label(root, text="Travel time (ms):").grid(row=1, column=0)
entry_time = tk.Entry(root)
entry_time.insert(0, "4000")
entry_time.grid(row=1, column=1)

# joints
entries_joint = []
for i in range(4):
    tk.Label(root, text=f"Joint {i+1} (deg):").grid(row=2+i, column=0)
    e = tk.Entry(root); e.insert(0, "0"); e.grid(row=2+i, column=1)
    entries_joint.append(e)

# coordinates
labels = ["X (mm):", "Y (mm):", "Z (mm):", "Yaw (deg):"]
entries_coor = []
for i, label in enumerate(labels):
    tk.Label(root, text=label).grid(row=6+i, column=0)
    e = tk.Entry(root); e.insert(0, "0"); e.grid(row=6+i, column=1)
    entries_coor.append(e)

# handlers

def send_pp_joint():
    data = [f2(e.get(), 0) for e in entries_joint]
    send("pp_joint", joints=data, time=i2(entry_time.get(), 0))

def send_pp_coor():
    data = [f2(e.get(), 0) for e in entries_coor]
    send("pp_coor", coor=data, time=i2(entry_time.get(), 0))

def send_pvt_joint():
    data = [f2(e.get(), 0) for e in entries_joint]
    send("pvt_joint", joints=data, time=i2(entry_time.get(), 0))

def send_pvt_coor():
    data = [f2(e.get(), 0) for e in entries_coor]
    send("pvt_coor", coor=data, time=i2(entry_time.get(), 0))

# buttons
 tk_btns = [
    ("Wake Up",    {"row":10, "col":0, "opt":{"bg":"purple", "fg":"white"}}, lambda: send("wake_up")),
    ("Shutdown",   {"row":10, "col":1, "opt":{"bg":"maroon", "fg":"white"}}, lambda: send("shutdown")),
    ("PP Joint",   {"row":11, "col":0}, send_pp_joint),
    ("PP Coor",    {"row":11, "col":1}, send_pp_coor),
    ("PVT Joint",  {"row":12, "col":0}, send_pvt_joint),
    ("PVT Coor",   {"row":12, "col":1}, send_pvt_coor),
    ("Read Position", {"row":13, "col":0, "opt": {"bg":"orange"}}, lambda: send("read_position")),
    ("Homing",        {"row":13, "col":1, "opt": {"bg":"cyan"}},   lambda: send("homing")),
    ("Stop",       {"row":14, "col":0, "span":2, "opt": {"bg":"red", "fg":"white"}}, lambda: send("stop")),
]

for text, pos, fn in tk_btns:
    opt = pos.get("opt", {})
    span = pos.get("span", 1)
    btn = tk.Button(root, text=text, command=fn, **opt)
    btn.grid(row=pos["row"], column=pos["col"], columnspan=span, sticky="ew")

# speed control
spd_var = tk.StringVar(value="200")
tk.Label(root, text="JV (pps):").grid(row=16, column=0, sticky='e')
tk.Entry(root, textvariable=spd_var).grid(row=16, column=1, sticky='w')
tk.Button(root, text="Set Speed", command=lambda: send("set_speed", pps=f2(spd_var.get(), 0))).grid(row=17, column=0, columnspan=2, sticky='ew')

# status label
lbl_last = tk.Label(root, text=f"endpoint: {CMD_ENDPOINT}")
lbl_last.grid(row=18, column=0, columnspan=2, sticky='w')

# window close
root.protocol("WM_DELETE_WINDOW", root.destroy)
root.mainloop()
