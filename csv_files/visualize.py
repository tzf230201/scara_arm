import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def visualize_xyz_trajectory(csv_file):
    # --- 1) Load CSV ---
    df = pd.read_csv(csv_file)
    x = df["X"].values
    y = df["Y"].values
    z = df["Z"].values

    # --- 2) Plot 3D ---
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

visualize_xyz_trajectory("Shuttle_20250908.csv")

