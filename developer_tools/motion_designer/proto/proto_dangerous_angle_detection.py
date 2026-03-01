import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# === Hitung sudut & flag dangerous ===
def dangerous_angle_detection(csv_in, csv_out="temp_1.csv", step_mm=2.0, angle_threshold=5):
    df = pd.read_csv(csv_in)
    pts = df[["X", "Y", "Z"]].values

    # Hitung cumulative distance (monoton)
    dists = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum_dist = np.insert(np.cumsum(dists), 0, 0.0)

    n = len(pts)
    angles = np.full(n, np.nan)
    flags = np.zeros(n, dtype=int)

    # Cari index kiri & kanan (pakai searchsorted untuk cepat)
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

    # Tambahkan ke dataframe & simpan
    df["angle_deg"] = angles
    df["dangerous"] = flags.astype(int)
    df.to_csv(csv_out, index=False)
    print(f"Hasil disimpan ke {csv_out}")
    return df

# === Visualisasi statis ===
def plot_trajectory(df):
    pts = df[["X", "Y", "Z"]].values
    flags = df["dangerous"].values.astype(bool)

    fig = plt.figure(figsize=(8,6))
    ax = fig.add_subplot(111, projection="3d")

    # Plot safe & dangerous dalam batch
    for safe, color in zip([~flags, flags], ["grey", "red"]):
        idx = np.where(safe[:-1] & safe[1:])[0]  # segmen konsisten
        for i in idx:
            ax.plot(pts[i:i+2,0], pts[i:i+2,1], pts[i:i+2,2], color=color)

    ax.scatter(*pts[0], c="green", s=60, label="Start")
    ax.scatter(*pts[-1], c="orange", s=60, label="End")
    ax.set_title("Trajectory with Dangerous Zones")
    ax.legend()
    plt.show()

# === Contoh pemakaian ===
if __name__ == "__main__":
    df = dangerous_angle_detection("pickup_shelf_20250922.csv", "temp_1.csv",
                            step_mm=2.0, angle_threshold=5)
    plot_trajectory(df)
