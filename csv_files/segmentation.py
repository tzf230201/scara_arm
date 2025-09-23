import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def _angle_deg(u, v):
    nu, nv = np.linalg.norm(u), np.linalg.norm(v)
    if nu < 1e-12 or nv < 1e-12:
        return 0.0
    cosang = np.dot(u, v) / (nu * nv)
    cosang = np.clip(cosang, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosang)))


def segment_by_angle(csv_file, angle_thresh=30.0, x_col="X", y_col="Y", z_col="Z"):
    """
    Pisahkan lintasan jadi segmen garis berdasarkan perubahan sudut.
    - Jika sudut > angle_thresh → mulai segmen baru
    - Warna tiap segmen: merah, biru, merah, ...
    """
    df = pd.read_csv(csv_file)
    P = df[[x_col, y_col, z_col]].to_numpy(dtype=float)
    n = len(P)

    # hitung vektor segmen
    V = P[1:] - P[:-1]

    # klasifikasi segmen index
    seg_id = np.zeros(len(V), dtype=int)
    cur_seg = 0
    for i in range(1, len(V)):
        ang = _angle_deg(V[i-1], V[i])
        if ang > angle_thresh:  # break garis
            cur_seg += 1
        seg_id[i] = cur_seg

    return df, seg_id


def visualize_segments(df, seg_id):
    P = df[["X","Y","Z"]].to_numpy()
    fig = plt.figure(figsize=(9,7))
    ax = fig.add_subplot(111, projection='3d')

    colors = ["b","r"]  # biru-merah bergantian
    for seg in np.unique(seg_id):
        idxs = np.where(seg_id == seg)[0]
        for i in idxs:
            ax.plot([P[i,0], P[i+1,0]],
                    [P[i,1], P[i+1,1]],
                    [P[i,2], P[i+1,2]],
                    color=colors[seg % 2])
    ax.scatter(P[0,0], P[0,1], P[0,2], color="g", s=50, label="Start")
    ax.scatter(P[-1,0], P[-1,1], P[-1,2], color="k", s=50, label="End")
    ax.set_xlabel("X [mm]"); ax.set_ylabel("Y [mm]"); ax.set_zlabel("Z [mm]")
    ax.set_title("Segments by angle change (blue/red alternating)")
    ax.legend()
    plt.show()

input_csv = "place_shelf_20250922.csv"
df, seg_id = segment_by_angle(input_csv, angle_thresh=1.0)
visualize_segments(df, seg_id)