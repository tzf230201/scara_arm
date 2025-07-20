import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class TrajectoryPlayer:
    def __init__(self, master, trajectory, dt_ms=50):
        self.master = master
        self.trajectory = trajectory
        self.dt_ms = dt_ms
        self.index = 0
        self.playing = False

        # Setup Matplotlib figure
        self.fig = plt.figure(figsize=(6, 5))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=master)
        self.canvas.get_tk_widget().pack()

        # Setup buttons
        btn_frame = tk.Frame(master)
        btn_frame.pack()
        tk.Button(btn_frame, text="▶️ Play", command=self.play).pack(side=tk.LEFT)
        tk.Button(btn_frame, text="⏸️ Pause", command=self.pause).pack(side=tk.LEFT)
        tk.Button(btn_frame, text="🔁 Reset", command=self.reset).pack(side=tk.LEFT)

        # Setup slider
        self.slider = tk.Scale(master, from_=0, to=len(self.trajectory)-1, orient=tk.HORIZONTAL,
                              label="Waktu (index)", command=self.on_slider)
        self.slider.pack(fill=tk.X)

        self.plot_static_path()
        self.update_plot()

    def plot_static_path(self):
        data = np.array(self.trajectory)
        self.ax.plot(data[:, 0], data[:, 1], data[:, 2], color='lightgray', label='Full Trajectory')
        self.ax.set_xlim(0, 258)
        self.ax.set_ylim(-258, 258)
        self.ax.set_zlim(0, 300)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

    def update_plot(self):
        self.ax.cla()
        self.plot_static_path()

        if self.index < len(self.trajectory):
            point = self.trajectory[self.index]
            x, y, z, yaw_deg, t_ms = point
            self.ax.scatter(x, y, z, color='red', s=50)
            self.ax.quiver(x, y, z,
                           np.cos(np.radians(yaw_deg)),
                           np.sin(np.radians(yaw_deg)),
                           0,
                           length=20, color='blue')
            self.canvas.draw()

        # Update slider position (if not playing)
        if not self.playing:
            self.slider.set(self.index)

        if self.playing and self.index < len(self.trajectory) - 1:
            self.index += 1
            self.slider.set(self.index)
            self.master.after(self.dt_ms, self.update_plot)

    def play(self):
        if not self.playing:
            self.playing = True
            self.slider.config(state=tk.DISABLED)
            self.update_plot()

    def pause(self):
        self.playing = False
        self.slider.config(state=tk.NORMAL)

    def reset(self):
        self.playing = False
        self.index = 0
        self.slider.config(state=tk.NORMAL)
        self.slider.set(0)
        self.update_plot()

    def on_slider(self, val):
        if not self.playing:
            self.index = int(val)
            self.update_plot()


# ---- Contoh jalankan aplikasi ----
if __name__ == "__main__":
    import random

    # Buat contoh trajectory dummy (x, y, z, yaw, t_ms)
    trajectory = []
    t = 0
    for i in range(100):
        x = 107 + i
        y = 100 + 30 * np.sin(i / 10)
        z = 90 + 50 * np.cos(i / 20)
        yaw = (90 + i * 2) % 360
        trajectory.append([x, y, z, yaw, t])
        t += 50  # ms

    root = tk.Tk()
    root.title("Trajectory Player")
    player = TrajectoryPlayer(root, trajectory, dt_ms=50)
    root.mainloop()
