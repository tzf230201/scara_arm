import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# =========================
# Step 1: Deteksi zona berbahaya
# =========================
def dangerous_angle_detection(csv_in, step_mm=2.0, angle_threshold=5):
    df = pd.read_csv(csv_in)
    pts = df[["X", "Y", "Z"]].values

    dists = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum_dist = np.insert(np.cumsum(dists), 0, 0.0)

    n = len(pts)
    angles = np.full(n, np.nan)
    flags = np.zeros(n, dtype=int)

    for i in range(n):
        left_idx = np.searchsorted(cum_dist, cum_dist[i] - step_mm, side="right") - 1
        right_idx = np.searchsorted(cum_dist, cum_dist[i] + step_mm, side="left")

        if 0 <= left_idx < i and right_idx > i and right_idx < n:
            v1 = pts[i] - pts[left_idx]
            v2 = pts[right_idx] - pts[i]
            cos_val = np.dot(v1, v2) / (np.linalg.norm(v1)*np.linalg.norm(v2))
            ang = np.degrees(np.arccos(np.clip(cos_val, -1, 1)))
            angles[i] = ang
            flags[i] = ang > angle_threshold

    df["angle_deg"] = angles
    df["dangerous"] = flags.astype(int)
    return df

def plot_trajectory(df):
    pts = df[["X", "Y", "Z"]].values
    flags = df["dangerous"].values.astype(bool)

    fig = plt.figure(figsize=(8,6))
    ax = fig.add_subplot(111, projection="3d")

    for safe, color in zip([~flags, flags], ["grey", "red"]):
        idx = np.where(safe[:-1] & safe[1:])[0]
        for i in idx:
            ax.plot(pts[i:i+2,0], pts[i:i+2,1], pts[i:i+2,2], color=color)

    ax.scatter(*pts[0], c="green", s=60, label="Start")
    ax.scatter(*pts[-1], c="orange", s=60, label="End")
    ax.set_title("Trajectory with Dangerous Zones")
    ax.legend()
    plt.show()

# =========================
# Profil trapezoid analitik
# =========================
def trapezoid_profile(D, v_start, v_end, vmax, t_acc_ms, dt_ms):
    dt = dt_ms / 1000.0
    t_acc = t_acc_ms / 1000.0
    t_dec = t_acc

    acc = (vmax - v_start) / t_acc if t_acc > 0 else float("inf")
    dec = (vmax - v_end) / t_dec if t_dec > 0 else float("inf")

    s_acc = (vmax**2 - v_start**2) / (2*acc) if acc > 0 else 0
    s_dec = (vmax**2 - v_end**2) / (2*dec) if dec > 0 else 0

    if s_acc + s_dec > D:  # segitiga asimetris
        vmax = math.sqrt((2*D*acc*dec + dec*v_start**2 + acc*v_end**2)/(acc+dec))
        s_acc = (vmax**2 - v_start**2) / (2*acc)
        s_dec = (vmax**2 - v_end**2) / (2*dec)
        s_cruise = 0
    else:
        s_cruise = D - (s_acc + s_dec)

    t_acc = (vmax - v_start) / acc if acc > 0 else 0
    t_dec = (vmax - v_end) / dec if dec > 0 else 0
    t_cruise = s_cruise / vmax if vmax > 0 else 0
    T = t_acc + t_cruise + t_dec

    times = np.arange(0, T+dt, dt)
    v_values = []
    for t in times:
        if t < t_acc:  # akselerasi
            v = v_start + acc*t
        elif t < t_acc + t_cruise:  # cruise
            v = vmax
        elif t <= T:  # deselerasi
            td = t - (t_acc + t_cruise)
            v = vmax - dec*td
        else:
            v = v_end
        v_values.append(max(v,0))
    return times, np.array(v_values)

# =========================
# Path length & profile builder
# =========================
def compute_path_length(points):
    seg_lengths = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))
    cum_lengths = np.insert(np.cumsum(seg_lengths), 0, 0)
    return seg_lengths, cum_lengths

def build_full_profile(points, seg_lengths, vmax, vsafe, t_acc_ms, dt_ms):
    times_all, v_all, s_all = [], [], []
    offset_s, v_current, t_offset, i = 0, 0, 0, 0

    while i < len(seg_lengths):
        # SAFE
        safe_len = 0
        while i < len(seg_lengths) and points[i,3] == 0 and points[i+1,3] == 0:
            safe_len += seg_lengths[i]; i += 1
        if safe_len > 0:
            v_end = vsafe if i < len(seg_lengths) else 0
            t, v = trapezoid_profile(safe_len, v_current, v_end, vmax, t_acc_ms, dt_ms)
            s = np.cumsum(v)*(dt_ms/1000.0)
            times_all.extend(t_offset + t); v_all.extend(v); s_all.extend(offset_s + s)
            v_current, t_offset, offset_s = v_end, t_offset+t[-1], offset_s+s[-1]

        # DANGER
        danger_len = 0
        while i < len(seg_lengths) and (points[i,3]==1 or points[i+1,3]==1):
            danger_len += seg_lengths[i]; i += 1
        if danger_len > 0:
            dt = dt_ms/1000.0
            steps = int(danger_len / (vsafe*dt)) + 1
            t = np.linspace(0, steps*dt, steps)
            v = np.ones_like(t)*vsafe
            s = np.cumsum(v)*dt
            times_all.extend(t_offset + t); v_all.extend(v); s_all.extend(offset_s + s)
            v_current, t_offset, offset_s = vsafe, t_offset+t[-1], offset_s+s[-1]

    return np.array(times_all), np.array(s_all), np.array(v_all)

# =========================
# Interpolasi posisi
# =========================
def interpolate_position(points, cum_lengths, s):
    idx = np.searchsorted(cum_lengths, s) - 1
    idx = max(0, min(idx, len(points)-2))
    s0, s1 = cum_lengths[idx], cum_lengths[idx+1]
    ratio = (s - s0) / (s1 - s0 + 1e-9)
    pos = (1-ratio)*points[idx, :3] + ratio*points[idx+1, :3]
    return pos, idx

# =========================
# Export profil trajectory ke CSV
# =========================
def export_profile_to_csv(filename_out, times, s_values, v_values, points, cum_lengths):
    rows = []
    for t, s, v in zip(times, s_values, v_values):
        pos, idx = interpolate_position(points, cum_lengths, s)
        danger = points[idx,3]
        rows.append([t, s, v, pos[0], pos[1], pos[2], danger])
    df_out = pd.DataFrame(rows, columns=["time_s","s_mm","v_mm_s","X","Y","Z","dangerous"])
    df_out.to_csv(filename_out, index=False)
    print(f"Trajectory profile disimpan ke {filename_out}")

# =========================
# Animasi
# =========================
def animate_trajectory(df, vmax=100, vsafe=10, t_acc_ms=2000, dt_ms=50, csv_out="trajectory_output.csv"):
    points = df[['X','Y','Z','dangerous']].values
    seg_lengths, cum_lengths = compute_path_length(points[:, :3])
    times, s_values, v_values = build_full_profile(points, seg_lengths, vmax, vsafe, t_acc_ms, dt_ms)

    # simpan hasil ke CSV
    export_profile_to_csv(csv_out, times, s_values, v_values, points, cum_lengths)

    fig = plt.figure(figsize=(12,6))
    ax1 = fig.add_subplot(121, projection='3d')
    ax2 = fig.add_subplot(122)

    for i in range(len(points)-1):
        color = 'gray' if points[i,3] == 0 else 'red'
        ax1.plot(points[i:i+2,0], points[i:i+2,1], points[i:i+2,2], color=color, linewidth=1)

    trail, = ax1.plot([], [], [], color='blue', linewidth=2)
    point, = ax1.plot([], [], [], 'o', color='blue', markersize=6)

    ax1.set_xlim(points[:,0].min(), points[:,0].max())
    ax1.set_ylim(points[:,1].min(), points[:,1].max())
    ax1.set_zlim(points[:,2].min(), points[:,2].max())
    ax1.set_title("Trajectory Animation")

    vel_line, = ax2.plot([], [], color='black', linewidth=1)
    marker, = ax2.plot([], [], 'ro')
    ax2.set_title("Velocity Profile")
    ax2.set_xlabel("Time [s]"); ax2.set_ylabel("Velocity [mm/s]")
    ax2.set_xlim(0, times[-1]); ax2.set_ylim(0, max(v_values)*1.1)

    def update(frame):
        s = s_values[frame]
        pos, idx = interpolate_position(points, cum_lengths, s)
        point.set_data([pos[0]], [pos[1]]); point.set_3d_properties([pos[2]])
        traversed = points[:idx+1, :3]
        trail.set_data(traversed[:,0], traversed[:,1]); trail.set_3d_properties(traversed[:,2])
        vel_line.set_data(times[:frame+1], v_values[:frame+1]); marker.set_data([times[frame]], [v_values[frame]])
        return point, trail, vel_line, marker

    ani = FuncAnimation(fig, update, frames=len(times), interval=dt_ms, blit=True, repeat=False)
    plt.show()

# =========================
# Main
# =========================
if __name__ == "__main__":
    # 1. baca file asli dan deteksi zona berbahaya
    input_csv = "pickup_shelf_20250922.csv"
    output_csv = "resampled/resampled_" + input_csv
    df = dangerous_angle_detection(input_csv, step_mm=2.0, angle_threshold=5)

    # 2. plot statis
    plot_trajectory(df)

    # 3. animasi + simpan profil hasil ke CSV
    animate_trajectory(df, vmax=100, vsafe=10, t_acc_ms=2000, dt_ms=50,
                       csv_out=output_csv)
