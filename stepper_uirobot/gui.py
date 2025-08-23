import zmq
import tkinter as tk

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")   # publish ke port 5555

def send_command(cmd):
    socket.send_string(cmd)
    print("Sent:", cmd)

root = tk.Tk()
tk.Button(root, text="Homing", command=lambda: send_command("homing")).pack()
tk.Button(root, text="Stop", command=lambda: send_command("stop")).pack()
root.mainloop()
