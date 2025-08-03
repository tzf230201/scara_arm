import tkinter as tk
import threading
import time
import signal
import sys

# === Dummy functions (ganti dengan versi asli kamu) ===
def wake_up():
    print("[wake_up] Motor is activated.")

def shutdown():
    print("[shutdown] System shutting down.")

def routine():
    print("[routine] Periodic update...")
    root.after(500, routine)

# === Signal handler ===
def signal_handler():
    print("SIGINT received. Exiting...")
    try:
        root.quit()
        root.destroy()
    except:
        pass
    shutdown()
    sys.exit(0)

# === GUI function (runs in its own thread) ===
def gui_loop():
    wake_up()  # Opsional: wake up saat GUI dimulai
    root.after(500, routine)  # Loop fungsional
    root.mainloop()

# === GUI setup ===
root = tk.Tk()
root.title("Motor Control GUI")

motor_type = tk.StringVar(value="all")

# Radio buttons
tk.Label(root, text="Motor Mode:").grid(row=0, column=0, padx=5, pady=5)
tk.Radiobutton(root, text="All", variable=motor_type, value="all").grid(row=0, column=1)
tk.Radiobutton(root, text="Stepper", variable=motor_type, value="stepper_only").grid(row=0, column=2)
tk.Radiobutton(root, text="Servo", variable=motor_type, value="servo_only").grid(row=0, column=3)

# Buttons
tk.Button(root, text="Wake Up", command=wake_up).grid(row=1, column=0, columnspan=2, pady=10, padx=5, sticky="ew")
tk.Button(root, text="Shutdown", command=shutdown).grid(row=1, column=2, columnspan=2, pady=10, padx=5, sticky="ew")

# Tangani event close window
root.protocol("WM_DELETE_WINDOW", signal_handler)

# === Jalankan GUI di thread terpisah ===
gui_thread = threading.Thread(target=gui_loop)
gui_thread.daemon = True
gui_thread.start()

# === Logic utama tetap jalan ===
try:
    while True:
        print("[main thread] Logic utama berjalan...")
        time.sleep(1)

except KeyboardInterrupt:
    signal_handler()
