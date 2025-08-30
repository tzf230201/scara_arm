# import math
# import matplotlib.pyplot as plt
# from stepper import stepper_pulse_to_deg, stepper_deg_to_pulse



# def plot_yx(x_list,y_list):
#     plt.figure()
#     plt.plot(y_list, x_list, '-o', label='End Effector Path')
#     plt.xlabel('Y (mm)')
#     plt.ylabel('X (mm)')
#     plt.axis('equal')
#     plt.grid(True)
#     plt.legend()
#     plt.title("YX Trajectory from PT Path")
#     plt.show()

# def plot_xy_from_pt(pt1, pt2, pt3, pt4):
#     x_list, y_list = [], []
#     N = len(pt1)
#     for i in range(N):
#         # Pulse → degree
#         j1 = stepper_pulse_to_deg(pt1[i])
#         j2 = stepper_pulse_to_deg(pt2[i])
#         j3 = stepper_pulse_to_deg(pt3[i])
#         j4 = stepper_pulse_to_deg(pt4[i])
#         # FK: panggil dengan satu list
#         x, y, *_ = forward_kinematics([j1, j2, j3, j4])
#         x_list.append(x)
#         y_list.append(y)
#     plt.figure()
#     plt.plot(x_list, y_list, '-o', label='End Effector Path')
#     plt.xlabel('X (mm)')
#     plt.ylabel('Y (mm)')
#     plt.axis('equal')
#     plt.grid(True)
#     plt.legend()
#     plt.title("XY Trajectory from PT Path")
#     plt.show()
    
# def generate_multi_straight_pt_points(start_coor, list_tar_coor, pt_time_interval=50):
#     """
#     Generate PT trajectory (pulse) untuk 4 joint dengan triangle profile di Cartesian space.
#     - start_coor: [x0, y0, z0, yaw0]
#     - list_tar_coor: list of ([x, y, z, yaw], durasi_ms)
#     - pt_time_interval: waktu sampling ms
#     Returns: pt1_f, pt2_f, pt3_f, pt4_f
#     """
#     pt1_f, pt2_f, pt3_f, pt4_f = [], [], [], []
#     last_coor = start_coor

#     for tar_coor, traveltime in list_tar_coor:
#         # jumlah step & total waktu
#         n_step = max(int(round(traveltime / pt_time_interval)), 1)
#         T = n_step * pt_time_interval

#         for i in range(n_step):
#             t = i * pt_time_interval
#             tau = t / T  # normalisasi waktu [0,1)

#             # triangle profile: alpha = s(t)/s_total
#             if tau <= 0.5:
#                 alpha = 2 * tau**2
#             else:
#                 alpha = 1 - 2 * (1 - tau)**2

#             # interpolasi Cartesian
#             coor = [
#                 last_coor[j] + (tar_coor[j] - last_coor[j]) * alpha
#                 for j in range(4)
#             ]

#             # IK → pulse
#             joint = inverse_kinematics(coor)
#             pulses = [stepper_deg_to_pulse(j) for j in joint]

#             pt1_f.append(pulses[0])
#             pt2_f.append(pulses[1])
#             pt3_f.append(pulses[2])
#             pt4_f.append(pulses[3])

#         # beralih ke segmen berikutnya, pastikan posisinya pas di target
#         last_coor = tar_coor

#     # tambahkan titik akhir yang pasti sampai target terakhir
#     final_joint = inverse_kinematics(list_tar_coor[-1][0])
#     pulses = [stepper_deg_to_pulse(j) for j in final_joint]
#     pt1_f.append(pulses[0])
#     pt2_f.append(pulses[1])
#     pt3_f.append(pulses[2])
#     pt4_f.append(pulses[3])

#     return pt1_f, pt2_f, pt3_f, pt4_f

# import numpy as np

# def generate_trajectory_triangle(cur_coor, list_tar_coor, dt):
#     def triangle_profile(p0, p1, T, dt):
#         steps = int(T / dt)
#         half = steps // 2
#         a = 4 * (p1 - p0) / (T ** 2)

#         positions = []
#         for i in range(steps + 1):
#             t = i * dt
#             if i <= half:
#                 pos = p0 + 0.5 * a * t**2
#             else:
#                 t1 = t - T / 2
#                 vmax = a * (T / 2)
#                 pmid = p0 + 0.5 * a * (T / 2)**2
#                 pos = pmid + vmax * t1 - 0.5 * a * t1**2
#             positions.append(pos)
#         return positions

#     time_vals = []
#     total_time = 0
#     current = cur_coor
#     traj = [[] for _ in range(4)]

#     for target, T in list_tar_coor:
#         for j in range(4):  # x, y, z, yaw
#             interp = triangle_profile(current[j], target[j], T, dt)
#             traj[j].extend(interp)
#         time_vals.extend(np.arange(total_time, total_time + T + dt, dt))
#         total_time += T
#         current = target

#     return time_vals, traj[0], traj[1], traj[2], traj[3]

# def convert_cartesian_traj_to_joint_traj(x_list, y_list, z_list, yaw_list):
#     joint1_list, joint2_list, joint3_list, joint4_list = [], [], [], []

#     for i, (x, y, z, yaw) in enumerate(zip(x_list, y_list, z_list, yaw_list)):
#         joints = inverse_kinematics([x, y, z, yaw])
        
        
#         joint1_list.append(joints[0])
#         joint2_list.append(joints[1])
#         joint3_list.append(joints[2])
#         joint4_list.append(joints[3])

#     return joint1_list, joint2_list, joint3_list, joint4_list

# def generate_multi_straight_pvt_points(start_coor, list_tar_coor, dt):
#     """
#     - start_coor: [x0,y0,z0,yaw0]
#     - list_tar_coor: list of ([x,y,z,yaw], durasi_ms)
#     - dt: sampling interval per row (ms)

#     Return:
#       pvt1, pvt2, pvt3, pvt4
#     di mana setiap pvtX adalah list [positions, velocities, times]
#     """
#     # 1) Trajektori Cartesian + konversi ke joint (deg)
#     t_arr, x_arr, y_arr, z_arr, yaw_arr = generate_trajectory_triangle(start_coor, list_tar_coor, dt)
#     j1_arr, j2_arr, j3_arr, j4_arr = convert_cartesian_traj_to_joint_traj(x_arr, y_arr, z_arr, yaw_arr)

#     # 2) Degree → pulse
#     p1 = [stepper_deg_to_pulse(d) for d in j1_arr]
#     p2 = [stepper_deg_to_pulse(d) for d in j2_arr]
#     p3 = [stepper_deg_to_pulse(d) for d in j3_arr]
#     p4 = [stepper_deg_to_pulse(d) for d in j4_arr]

#     # 3) Hitung velocity (pulse/sec)
#     dt_s = dt / 1000.0
#     def compute_vel(p):
#         v = [int((p[i+1] - p[i]) / dt_s) for i in range(len(p)-1)]
#         v.append(0)
#         return v

#     v1 = compute_vel(p1)
#     v2 = compute_vel(p2)
#     v3 = compute_vel(p3)
#     v4 = compute_vel(p4)

#     # 4) Waktu per row (ms)
#     times = [dt] * len(p1)

#     # 5) Buat list PVT untuk tiap joint
#     pvt1 = [p1, v1, times]
#     pvt2 = [p2, v2, times]
#     pvt3 = [p3, v3, times]
#     pvt4 = [p4, v4, times]

#     return pvt1, pvt2, pvt3, pvt4