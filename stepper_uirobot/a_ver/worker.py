# """
# app_mp_min.py — Minimal prototype: Tk GUI + worker (print-only)
# - No ZMQ, no SharedMemory; just multiprocessing.Pipe for ultra-low overhead
# - Worker only prints whatever command it receives
# - GUI has the same buttons/inputs as your current design
# - Safe shutdown on window close

# Run:
#     python app_mp_min.py
# """
# from multiprocessing import Process, Pipe, Event, set_start_method
# import tkinter as tk
# import time, queue

# # ---------------- Worker ----------------

# def worker(cmd_r, stop_ev: Event):
#     print("[worker] started")
#     while not stop_ev.is_set():
#         if cmd_r.poll(0.5):  # wait up to 0.5s for a message
#             try:
#                 msg = cmd_r.recv()
#             except EOFError:
#                 break
#             print("[worker] command:", msg)
#     print("[worker] exiting")

# # ---------------- GUI ----------------

# def start_gui(cmd_w, stop_ev: Event):
#     root = tk.Tk()
#     root.title("Motor Control Panel (mp, print-only)")

#     def send(cmd, **kw):
#         try:
#             cmd_w.send({"command": cmd, **kw})
#             lbl_last.config(text=f"last cmd: {cmd}")
#         except (BrokenPipeError, OSError):
#             lbl_last.config(text="last cmd: pipe broken")

#     # --- Motor selection ---
#     motor_type = tk.StringVar(value="all")
#     def update_motor_selection():
#         send("motor_selection", motor=motor_type.get())

#     radio = tk.LabelFrame(root, text="Motor Selection", padx=10, pady=5)
#     radio.grid(row=0, column=0, columnspan=2, pady=5, sticky="ew")
#     tk.Radiobutton(radio, text="All motors",   variable=motor_type, value="all",         command=update_motor_selection).grid(row=0, column=0)
#     tk.Radiobutton(radio, text="Stepper only", variable=motor_type, value="stepper_only", command=update_motor_selection).grid(row=0, column=1)
#     tk.Radiobutton(radio, text="Servo only",   variable=motor_type, value="servo_only",   command=update_motor_selection).grid(row=0, column=2)

#     # --- Time input ---
#     tk.Label(root, text="Travel time (ms):").grid(row=1, column=0)
#     entry_time = tk.Entry(root); entry_time.insert(0, "4000"); entry_time.grid(row=1, column=1)

#     # --- Joint inputs (4) ---
#     entries_joint = []
#     for i in range(4):
#         tk.Label(root, text=f"Joint {i+1} (deg):").grid(row=2+i, column=0)
#         e = tk.Entry(root); e.insert(0, "0"); e.grid(row=2+i, column=1)
#         entries_joint.append(e)

#     # --- Coordinate inputs ---
#     labels = ["X (mm):", "Y (mm):", "Z (mm):", "Yaw (deg):"]
#     entries_coor = []
#     for i, label in enumerate(labels):
#         tk.Label(root, text=label).grid(row=6+i, column=0)
#         e = tk.Entry(root); e.insert(0, "0"); e.grid(row=6+i, column=1)
#         entries_coor.append(e)

#     # --- Button handlers ---
#     def send_pp_joint():
#         data = [float(e.get() or 0) for e in entries_joint]
#         send("pp_joint", joints=data, time=int(entry_time.get() or 0))

#     def send_pp_coor():
#         data = [float(e.get() or 0) for e in entries_coor]
#         send("pp_coor", coor=data, time=int(entry_time.get() or 0))

#     def send_pvt_joint():
#         data = [float(e.get() or 0) for e in entries_joint]
#         send("pvt_joint", joints=data, time=int(entry_time.get() or 0))

#     def send_pvt_coor():
#         data = [float(e.get() or 0) for e in entries_coor]
#         send("pvt_coor", coor=data, time=int(entry_time.get() or 0))

#     # --- Buttons ---
#     tk.Button(root, text="Wake Up", bg="purple", fg="white", command=lambda: send("wake_up")).grid(row=10, column=0, sticky="ew")
#     tk.Button(root, text="Shutdown", bg="maroon", fg="white", command=lambda: send("shutdown")).grid(row=10, column=1, sticky="ew")
#     tk.Button(root, text="PP Joint", command=send_pp_joint).grid(row=11, column=0, sticky="ew")
#     tk.Button(root, text="PP Coor",  command=send_pp_coor ).grid(row=11, column=1, sticky="ew")
#     tk.Button(root, text="PVT Joint", command=send_pvt_joint).grid(row=12, column=0, sticky="ew")
#     tk.Button(root, text="PVT Coor",  command=send_pvt_coor ).grid(row=12, column=1, sticky="ew")
#     tk.Button(root, text="Read Position", bg="orange", command=lambda: send("read_position")).grid(row=13, column=0, sticky="ew")
#     tk.Button(root, text="Homing",        bg="cyan",   command=lambda: send("homing")).grid(row=13, column=1, sticky="ew")
#     tk.Button(root, text="Stop", bg="red", fg="white", command=lambda: send("stop")).grid(row=14, column=0, columnspan=2, sticky="ew")

#     # status label
#     lbl_last = tk.Label(root, text="last cmd: --"); lbl_last.grid(row=15, column=0, columnspan=2, sticky="w")

#     def on_close():
#         stop_ev.set()
#         # small delay to let worker exit cleanly
#         root.after(100, root.destroy)

#     root.protocol("WM_DELETE_WINDOW", on_close)
#     root.mainloop()

# # ---------------- Bootstrap ----------------

# if __name__ == "__main__":
#     set_start_method('spawn')  # safest across OS
#     stop_ev = Event()
#     cmd_r, cmd_w = Pipe(duplex=False)  # recv end (worker), send end (GUI)

#     p = Process(target=worker, args=(cmd_r, stop_ev), daemon=True)
#     p.start()

#     try:
#         start_gui(cmd_w, stop_ev)
#     finally:
#         stop_ev.set()
#         try:
#             p.join(timeout=1)
#         except Exception:
#             pass
