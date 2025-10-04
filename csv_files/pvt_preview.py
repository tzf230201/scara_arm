import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math

# ==========================================================
# Forward Kinematics
# ==========================================================
def forward_kinematics(cur_joints):
    cur_deg1, cur_deg2, cur_deg3, cur_deg4 = cur_joints
    cur_deg4 *= -1.0
    L2, L3 = 137.0, 121.0
    OFFSET_2, OFFSET_3, OFFSET_4 = -96.5, 134.0, -52.5

    theta2_rad = math.radians((cur_deg2 / 5) + OFFSET_2)
    theta3_rad = math.radians((cur_deg3 / 5) + OFFSET_3 - (cur_deg2 / 5))

    x2 = L2 * math.cos(theta2_rad)
    y2 = L2 * math.sin(theta2_rad)
    x3 = x2 + L3 * math.cos(theta2_rad + theta3_rad)
    y3 = y2 + L3 * math.sin(theta2_rad + theta3_rad)
    z = (cur_deg1 / 360) * 90
    yaw = (cur_deg4 / 5) + OFFSET_4
    return [x3, y3, z, yaw]

# ==========================================================
# Simulasi PVT (ambil hanya p1..p4)
# ==========================================================
def simulate_pvt(csv_file):
    df = pd.read_csv(csv_file)
    traj = []
    for _, row in df.iterrows():
        cur_joints = [row["p1"], row["p2"], row["p3"], row["p4"]]
        xyz_yaw = forward_kinematics(cur_joints)
        traj.append(xyz_yaw)
    return np.array(traj)

# ==========================================================
# Animasi Trajectory
# ==========================================================
def animate_traj(csv_file, max_frames=1000, trail_window=200, interval=30):
    traj = simulate_pvt(csv_file)
    xs, ys, zs, yaws = traj[:, 0], traj[:, 1], traj[:, 2], traj[:, 3]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_title("4-Motor PVT Trajectory")

    # Plot lintasan penuh abu-abu
    ax.plot(xs, ys, zs, color="gray", linestyle="-", linewidth=1)

    # Animasi garis biru
    line, = ax.plot([], [], [], "b-", lw=2)
    point, = ax.plot([], [], [], "ro")

    # Progress text
    progress_text = ax.text2D(0.05, 0.95, "Progress: 0%", transform=ax.transAxes)

    # pilih subset frame kalau datanya panjang
    if len(xs) > max_frames:
        frame_idx = np.linspace(0, len(xs)-1, max_frames, dtype=int)
    else:
        frame_idx = np.arange(len(xs))

    def init():
        ax.set_xlim(min(xs) - 50, max(xs) + 50)
        ax.set_ylim(min(ys) - 50, max(ys) + 50)
        ax.set_zlim(min(zs) - 10, max(zs) + 10)
        return line, point, progress_text

    def update(j):
        f = frame_idx[j]
        start = max(0, f-trail_window)
        line.set_data(xs[start:f], ys[start:f])
        line.set_3d_properties(zs[start:f])
        point.set_data(xs[f:f+1], ys[f:f+1])
        point.set_3d_properties(zs[f:f+1])

        progress = int((f / (len(xs)-1)) * 100)
        progress_text.set_text(f"Progress: {progress}%")

        return line, point, progress_text

    ani = FuncAnimation(fig, update, frames=len(frame_idx),
                        init_func=init, blit=False, interval=interval,
                        repeat=False)
    plt.show()

# ==========================================================
# Main
# ==========================================================
if __name__ == "__main__":
    animate_traj("resampled/motion_list.csv",
                 max_frames=1033,   # full dataset dipakai (1033 baris)
                 trail_window=100,  # panjang ekor
                 interval=30)       # ms antar frame
