import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def visualize_xyz_trajectory(csv_file):
    df = pd.read_csv(csv_file)
    x, y, z = df["X"].values, df["Y"].values, df["Z"].values

    fig = plt.figure(figsize=(8,6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x, y, z, label="Trajectory", color="b")
    ax.scatter(x[0], y[0], z[0], color="g", s=50, label="Start")
    ax.scatter(x[-1], y[-1], z[-1], color="r", s=50, label="End")

    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    ax.set_zlabel("Z [mm]")
    ax.set_title("3D Trajectory Visualization")
    ax.legend()
    plt.show()


def compute_xyz_path_length(csv_file, x_col="X", y_col="Y", z_col="Z", velocity_mm_s=None):
    df = pd.read_csv(csv_file)
    x, y, z = df[x_col].to_numpy(float), df[y_col].to_numpy(float), df[z_col].to_numpy(float)

    if len(x) < 2:
        raise ValueError("Butuh minimal 2 titik untuk hitung lintasan")

    points = np.stack([x, y, z], axis=1)
    segments = np.linalg.norm(points[1:] - points[:-1], axis=1)
    total_distance_mm = float(np.sum(segments))

    total_time_s = total_distance_mm / velocity_mm_s if velocity_mm_s else None
    return {
        "samples": len(x),
        "start": (x[0], y[0], z[0]),
        "end": (x[-1], y[-1], z[-1]),
        "total_distance_mm": total_distance_mm,
        "total_time_s": total_time_s,
    }


def resample_xyz_by_velocity(csv_file, velocity_mm_s, dt_ms=50, x_col="X", y_col="Y", z_col="Z"):
    df = pd.read_csv(csv_file)
    x, y, z = df[x_col].to_numpy(float), df[y_col].to_numpy(float), df[z_col].to_numpy(float)
    pts = np.stack([x, y, z], axis=1)

    seg = np.linalg.norm(pts[1:] - pts[:-1], axis=1)
    cumdist = np.concatenate([[0], np.cumsum(seg)])
    total_dist = cumdist[-1]
    total_time_s = total_dist / velocity_mm_s

    times = np.arange(0, total_time_s, dt_ms / 1000.0)
    target_dists = times * velocity_mm_s

    new_pts = []
    for d in target_dists:
        idx = np.searchsorted(cumdist, d)
        if idx == 0:
            new_pts.append(pts[0])
        elif idx >= len(cumdist):
            new_pts.append(pts[-1])
        else:
            t = (d - cumdist[idx-1]) / (cumdist[idx] - cumdist[idx-1] + 1e-9)
            interp = pts[idx-1] + t * (pts[idx] - pts[idx-1])
            new_pts.append(interp)

    new_pts = np.array(new_pts)
    out_df = pd.DataFrame({
        "time_ms": (times * 1000).astype(int),
        "X": new_pts[:,0],
        "Y": new_pts[:,1],
        "Z": new_pts[:,2]
    })

    # pastikan baris terakhir = baris terakhir file asli
    out_df.loc[out_df.index[-1], ["X","Y","Z"]] = [x[-1], y[-1], z[-1]]

    print(f"Before resampling: {len(df)} rows, {df.shape[1]} columns")
    print(f"After resampling : {len(out_df)} rows, {out_df.shape[1]} columns")
    print(f"Total distance   : {total_dist:.3f} mm")
    print(f"Total time       : {total_time_s:.3f} s at {velocity_mm_s} mm/s")

    # --- otomatis bikin nama file output ---
    dirname, basename = os.path.split(csv_file)
    out_name = os.path.join(dirname, "resampled_" + basename)
    out_df.to_csv(out_name, index=False)
    print(f"Saved to {out_name}")

    return out_df, out_name


# ===============================
# Example usage
# ===============================
if __name__ == "__main__":
    input_csv = "DEMO_Shuttle.csv"
    velocity = 100   # mm/s
    dt = 50          # ms

    res = compute_xyz_path_length(input_csv, velocity_mm_s=velocity)
    print(f"Total jarak = {res['total_distance_mm']:.2f} mm")
    print(f"Waktu tempuh = {res['total_time_s']:.2f} s")

    out_df, out_file = resample_xyz_by_velocity(input_csv, velocity_mm_s=velocity, dt_ms=dt)

    visualize_xyz_trajectory(out_file)
