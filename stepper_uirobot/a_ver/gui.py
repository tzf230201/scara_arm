# import os
# import time
# import tkinter as tk
# import zmq

# # ===== Endpoint (samakan dengan receiver/main.py) =====
# CMD_ENDPOINT = os.getenv("CMD_ENDPOINT", "ipc:///tmp/motor_cmd")

# # ===== ZMQ PUB =====
# ctx = zmq.Context.instance()
# pub = ctx.socket(zmq.PUB)
# pub.setsockopt(zmq.SNDHWM, 1000)
# pub.bind(CMD_ENDPOINT)
# # slow-joiner guard: beri waktu socket siap sebelum kirim pertama
# time.sleep(0.1)

# # ---- helpers ----
# def send(cmd, **kw):
#     msg = {"command": cmd, **kw}
#     pub.send_json(msg)
#     print("Sent:", msg)
#     lbl_last.config(text=f"last cmd: {cmd}")

# def f2(x, default=0.0):
#     try:
#         return float(x)
#     except Exception:
#         return float(default)

# def i2(x, default=0):
#     try:
#         return int(float(x))
#     except Exception:
#         return int(default)

# # ===== GUI =====
# root = tk.Tk()
# root.title("Motor Control Panel (one-way)")

# # Motor selection (dipakai sebagai parameter di SETIAP command)
# motor_type = tk.StringVar(value="all")

# def current_motor():
#     return motor_type.get()

# radio = tk.LabelFrame(root, text="Motor Selection", padx=10, pady=5)
# radio.grid(row=0, column=0, columnspan=2, pady=5, sticky="ew")
# tk.Radiobutton(radio, text="All motors",   variable=motor_type, value="all").grid(row=0, column=0)
# TkStepper = tk.Radiobutton(radio, text="Stepper only", variable=motor_type, value="stepper_only")
# TkServo   = tk.Radiobutton(radio, text="Servo only",   variable=motor_type, value="servo_only")
# TkStepper.grid(row=0, column=1)
# TkServo.grid(row=0, column=2)

# # Travel time
# tk.Label(root, text="Travel time (ms):").grid(row=1, column=0)
# entry_time = tk.Entry(root)
# entry_time.insert(0, "4000")
# entry_time.grid(row=1, column=1)

# # Joint inputs
# entries_joint = []
# for i in range(4):
#     tk.Label(root, text=f"Joint {i+1} (deg):").grid(row=2+i, column=0)
#     e = tk.Entry(root)
#     e.insert(0, "0")
#     e.grid(row=2+i, column=1)
#     entries_joint.append(e)

# # Coordinate inputs
# labels = ["X (mm):", "Y (mm):", "Z (mm):", "Yaw (deg):"]
# entries_coor = []
# for i, label in enumerate(labels):
#     tk.Label(root, text=label).grid(row=6+i, column=0)
#     e = tk.Entry(root)
#     e.insert(0, "0")
#     e.grid(row=6+i, column=1)
#     entries_coor.append(e)

# # ---- Button handlers (motor=ikut setiap command) ----

# def send_pp_joint():
#     joints = [f2(e.get(), 0) for e in entries_joint]
#     send("pp_joint", joints=joints, time=i2(entry_time.get(), 0), motor=current_motor())

# def send_pp_coor():
#     coor = [f2(e.get(), 0) for e in entries_coor]
#     send("pp_coor", coor=coor, time=i2(entry_time.get(), 0), motor=current_motor())

# def send_pvt_joint():
#     joints = [f2(e.get(), 0) for e in entries_joint]
#     send("pvt_joint", joints=joints, time=i2(entry_time.get(), 0), motor=current_motor())

# def send_pvt_coor():
#     coor = [f2(e.get(), 0) for e in entries_coor]
#     send("pvt_coor", coor=coor, time=i2(entry_time.get(), 0), motor=current_motor())

# # ---- Buttons ----
# # Row 10
# tk.Button(root, text="Wake Up", bg="purple", fg="white",
#           command=lambda: send("wake_up", motor=current_motor())).grid(row=10, column=0, sticky="ew")
# tk.Button(root, text="Shutdown", bg="maroon", fg="white",
#           command=lambda: send("shutdown", motor=current_motor())).grid(row=10, column=1, sticky="ew")

# # Row 11
# tk.Button(root, text="PP Joint", command=send_pp_joint).grid(row=11, column=0, sticky="ew")
# tk.Button(root, text="PP Coor",  command=send_pp_coor ).grid(row=11, column=1, sticky="ew")

# # Row 12
# tk.Button(root, text="PVT Joint", command=send_pvt_joint).grid(row=12, column=0, sticky="ew")
# tk.Button(root, text="PVT Coor",  command=send_pvt_coor ).grid(row=12, column=1, sticky="ew")

# # Row 13
# tk.Button(root, text="Read Position", bg="orange",
#           command=lambda: send("read_position", motor=current_motor())).grid(row=13, column=0, sticky="ew")
# tk.Button(root, text="Dancing", bg="green", fg="white",
#           command=lambda: send("dancing", motor=current_motor())).grid(row=13, column=1, sticky="ew")

# # Row 14 — NEW: Dancing (left of Stop) & Stop
# tk.Button(root, text="Homing", bg="cyan",
#           command=lambda: send("homing", motor=current_motor())).grid(row=14, column=0, sticky="ew")
# tk.Button(root, text="Stop", bg="red", fg="white",
#           command=lambda: send("stop", motor=current_motor())).grid(row=14, column=1, sticky="ew")

# # Row 15 — NEW: Read Encoder & Set Origin
# tk.Button(root, text="Read Encoder",
#           command=lambda: send("read_encoder", motor=current_motor())).grid(row=15, column=0, sticky="ew")
# tk.Button(root, text="Set Origin",
#           command=lambda: send("set_origin", motor=current_motor())).grid(row=15, column=1, sticky="ew")

# # Status label
# lbl_last = tk.Label(root, text=f"endpoint: {CMD_ENDPOINT}")
# lbl_last.grid(row=18, column=0, columnspan=2, sticky='w')

# # Close
# root.protocol("WM_DELETE_WINDOW", root.destroy)
# root.mainloop()
