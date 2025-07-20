import matplotlib.pyplot as plt
from kinematics_and_trajectory import (
    generate_trajectory_triangle,
    forward_kinematics,
    inverse_kinematics,
    convert_cartesian_traj_to_joint_traj
)

# ==== KONFIGURASI STEP ====
STEPPER_PPR = 4096
STEPPER_RATIO = STEPPER_PPR / 360

def stepper_degrees_to_pulses(degrees):
    return degrees * STEPPER_RATIO

# ==== DATA AWAL ====
cur_coor = [258, 0, 0, 0]  # [x, y, z, yaw] in mm and deg
list_tar_coor = [
    ([166.82, -168, 0, 0], 1000),
    ([258, 0, 0, 0], 1000),
    ([107, 100, 0, 90], 1000),
    ([107, 224, 0, 90], 1000),
]

# ==== HITUNG TRAJEKTORI CARTESIAN ====
t, x, y, z, yaw = generate_trajectory_triangle(cur_coor, list_tar_coor, dt=20)

# ==== KONVERSI CARTESIAN → JOINT ====
j1_traj, j2_traj, j3_traj, j4_traj = convert_cartesian_traj_to_joint_traj(x, y, z, yaw)

# ==== POSISI BASE JOINT UNTUK HITUNG POSISI RELATIF ====
base_joint_deg = inverse_kinematics(cur_coor)

def compute_relative_joint_trajectory(joint_list, base_value):
    return [val - base_value for val in joint_list]

j1_rel = compute_relative_joint_trajectory(j1_traj, base_joint_deg[0])
j2_rel = compute_relative_joint_trajectory(j2_traj, base_joint_deg[1])
j3_rel = compute_relative_joint_trajectory(j3_traj, base_joint_deg[2])
j4_rel = compute_relative_joint_trajectory(j4_traj, base_joint_deg[3])

# ==== KONVERSI RELATIVE JOINT DEG → PULSE ====
j1_rel_pulse = [stepper_degrees_to_pulses(val) for val in j1_rel]
j2_rel_pulse = [stepper_degrees_to_pulses(val) for val in j2_rel]
j3_rel_pulse = [stepper_degrees_to_pulses(val) for val in j3_rel]
j4_rel_pulse = [stepper_degrees_to_pulses(val) for val in j4_rel]

# ==== FUNGSI PLOT ====
def plot_xy_trajectory(x, y):
    plt.figure(figsize=(6, 6))
    plt.plot(x, y, marker='o', color='blue')
    plt.title("Trajectory in X-Y Plane")
    plt.xlabel("X (mm)")
    plt.ylabel("Y (mm)")
    plt.grid(True)
    plt.xlim(0, 516)
    plt.ylim(-258, 258)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

def plot_xyz_yaw_vs_time(t, x, y, z, yaw):
    plt.figure(figsize=(12, 8))
    titles = ["X", "Y", "Z", "Yaw"]
    data = [x, y, z, yaw]
    ylabels = ["X (mm)", "Y (mm)", "Z (mm)", "Yaw (deg)"]
    colors = ["red", "green", "orange", "purple"]

    for i in range(4):
        plt.subplot(2, 2, i + 1)
        plt.plot(t[:len(data[i])], data[i], color=colors[i])
        plt.title(f"{titles[i]} vs Time")
        plt.xlabel("Time (ms)")
        plt.ylabel(ylabels[i])
        plt.grid(True)

    plt.tight_layout()
    plt.show()

def plot_joint_trajectories(t, j1, j2, j3, j4):
    plt.figure(figsize=(12, 8))
    joints = [j1, j2, j3, j4]
    titles = ["Joint 1 (Z)", "Joint 2", "Joint 3", "Joint 4 (Yaw)"]
    colors = ["blue", "red", "green", "orange"]

    for i in range(4):
        plt.subplot(2, 2, i + 1)
        plt.plot(t[:len(joints[i])], joints[i], color=colors[i])
        plt.title(f"{titles[i]} vs Time")
        plt.xlabel("Time (ms)")
        plt.ylabel("Stepper Angle (deg)")
        plt.grid(True)

    plt.tight_layout()
    plt.show()

def plot_relative_joint_trajectories(t, j1, j2, j3, j4):
    plt.figure(figsize=(12, 8))
    joints = [j1, j2, j3, j4]
    titles = ["Relative Joint 1", "Relative Joint 2", "Relative Joint 3", "Relative Joint 4 (Yaw)"]
    colors = ["blue", "red", "green", "orange"]

    for i in range(4):
        plt.subplot(2, 2, i + 1)
        plt.plot(t[:len(joints[i])], joints[i], color=colors[i])
        plt.title(f"{titles[i]} vs Time")
        plt.xlabel("Time (ms)")
        plt.ylabel("Δ Stepper Angle (deg)")
        plt.grid(True)

    plt.tight_layout()
    plt.show()

def plot_relative_joint_pulses(t, j1_p, j2_p, j3_p, j4_p):
    plt.figure(figsize=(12, 8))
    joints = [j1_p, j2_p, j3_p, j4_p]
    titles = ["Rel Joint 1 (Z)", "Rel Joint 2", "Rel Joint 3", "Rel Joint 4 (Yaw)"]
    colors = ["blue", "red", "green", "orange"]

    for i in range(4):
        plt.subplot(2, 2, i + 1)
        plt.plot(t[:len(joints[i])], joints[i], color=colors[i])
        plt.title(f"{titles[i]} vs Time")
        plt.xlabel("Time (ms)")
        plt.ylabel("Pulse")
        plt.grid(True)

    plt.tight_layout()
    plt.show()

# ==== TAMPILKAN GRAFIK ====
plot_xy_trajectory(x, y)
plot_xyz_yaw_vs_time(t, x, y, z, yaw)
plot_joint_trajectories(t, j1_traj, j2_traj, j3_traj, j4_traj)
plot_relative_joint_trajectories(t, j1_rel, j2_rel, j3_rel, j4_rel)
plot_relative_joint_pulses(t, j1_rel_pulse, j2_rel_pulse, j3_rel_pulse, j4_rel_pulse)

# ==== PVT POINTS ====
def generate_pvt_points(j1_rel, j2_rel, j3_rel, j4_rel, dt_ms):
    pvt_points = []
    for i in range(1, len(j1_rel)):
        p = [stepper_degrees_to_pulses(j[i]) for j in [j1_rel, j2_rel, j3_rel, j4_rel]]
        v = [stepper_degrees_to_pulses((j[i] - j[i-1]) / (dt_ms / 1000)) for j in [j1_rel, j2_rel, j3_rel, j4_rel]]
        pvt_points.append([(int(p[0]), int(v[0])), (int(p[1]), int(v[1])), 
                           (int(p[2]), int(v[2])), (int(p[3]), int(v[3])), dt_ms])
    return pvt_points

# Cetak PVT
pvt_points = generate_pvt_points(j1_rel, j2_rel, j3_rel, j4_rel, dt_ms=20)

print("\n=== PVT Points ===")
for i, (j1, j2, j3, j4, t_ms) in enumerate(pvt_points):
    print(f"[{i}] J1: pos={j1[0]}, vel={j1[1]} | "
          f"J2: pos={j2[0]}, vel={j2[1]} | "
          f"J3: pos={j3[0]}, vel={j3[1]} | "
          f"J4: pos={j4[0]}, vel={j4[1]} | t={t_ms} ms")
