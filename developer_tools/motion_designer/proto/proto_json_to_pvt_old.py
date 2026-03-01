import math
import numpy as np

# =========================
# SERVO KINEMATICS
# =========================
def servo_check_limit(angle_1):
    if angle_1 > (3510 * 4):
        print(f"[angle_1 greater than {angle_1}]")
        angle_1 = (3510 * 4)
    elif angle_1 < -2:
        print(f"angle_1 lower than {angle_1}")
        angle_1 = -2
    return angle_1

def servo_inverse_kinematics(z):
    angle_1 = (360.0 / 90.0) * z
    angle_1 = servo_check_limit(angle_1)
    return angle_1

def servo_forward_kinematics(angle_1):
    z = (90.0 / 360.0) * angle_1
    return z

# =========================
# ARM KINEMATICS
# =========================
def arm_check_limit(angle_2, angle_3, angle_4):
    angle_2_upper_limit = 178 * 5
    angle_2_lower_limit = 0
    angle_3_upper_limit = 0 + angle_2
    angle_3_lower_limit = (-135 * 5) + angle_2
    angle_4_upper_limit = (196 * 5) + angle_3
    angle_4_lower_limit = 0 + angle_3
    angle_4 *= -1

    if angle_2 > angle_2_upper_limit:
        print(f"angle_2 greater than {angle_2}")
        angle_2 = angle_2_upper_limit
    elif angle_2 < angle_2_lower_limit:
        print(f"angle_2 lower than {angle_2}")
        angle_2 = angle_2_lower_limit

    if angle_3 > angle_3_upper_limit:
        print(f"angle_3 greater than {angle_3}")
        angle_3 = angle_3_upper_limit
    elif angle_3 < angle_3_lower_limit:
        print(f"angle_3 lower than {angle_3}")
        angle_3 = angle_3_lower_limit

    if angle_4 > angle_4_upper_limit:
        print(f"angle_4 greater than {angle_4}")
        angle_4 = angle_4_upper_limit
    elif angle_4 < angle_4_lower_limit:
        print(f"angle_4 lower than {angle_4}")
        angle_4 = angle_4_lower_limit

    angle_4 *= -1
    return angle_2, angle_3, angle_4

def arm_inverse_kinematics(x, y, yaw):
    # Panjang link (mm)
    L2 = 137.0
    L3 = 121.0
    # Offset (deg) dan rasio joint
    OFFSET_2 = -96.5
    OFFSET_3 = 134
    OFFSET_4 = -52.5
    RATIO = 5.0  # 5:1

    # Hitung jarak planar
    distance = math.hypot(x, y)
    max_reach = L2 + L3
    if distance > max_reach:
        print(f"Warning: target ({x:.1f},{y:.1f}) jarak {distance:.1f} > {max_reach:.1f}, akan di-clamp")
        scale = max_reach / distance
        x *= scale
        y *= scale

    # Hitung cos(theta3)
    cos_theta3 = (x**2 + y**2 - L2**2 - L3**2) / (2 * L2 * L3)
    cos_theta3 = max(-1.0, min(1.0, cos_theta3))
    theta3_rad = math.acos(cos_theta3)
    theta3 = math.degrees(theta3_rad)

    # Hitung theta2
    k1 = L2 + L3 * cos_theta3
    k2 = L3 * math.sin(theta3_rad)
    theta2_rad = math.atan2(y, x) - math.atan2(k2, k1)
    theta2 = math.degrees(theta2_rad)

    joint_2 = (theta2 - OFFSET_2) * RATIO
    joint_3 = (theta3 - OFFSET_3) * RATIO + joint_2
    joint_4 = -((yaw - OFFSET_4) * RATIO)

    angle_2, angle_3, angle_4 = arm_check_limit(joint_2, joint_3, joint_4)
    return angle_2, angle_3, angle_4

def arm_forward_kinematics(angle_2, angle_3, angle_4):
    angle_4 *= -1.0
    L2 = 137.0
    L3 = 121.0
    OFFSET_2 = -96.5
    OFFSET_3 = 134.0
    OFFSET_4 = -52.5

    theta2_rad = math.radians((angle_2 / 5) + OFFSET_2)
    theta3_rad = math.radians((angle_3 / 5) + OFFSET_3 - (angle_2 / 5))

    x2 = L2 * math.cos(theta2_rad)
    y2 = L2 * math.sin(theta2_rad)
    x3 = x2 + L3 * math.cos(theta2_rad + theta3_rad)
    y3 = y2 + L3 * math.sin(theta2_rad + theta3_rad)
    yaw = (angle_4 / 5) + OFFSET_4

    return x3, y3, yaw

# =========================
# TRAJECTORY GENERATOR
# =========================
def servo_generate_single_straight_pvt_points(start_z, tar_z, T, dt):
    def triangle_profile(p0, p1, T, dt):
        steps = int(T / dt)
        half = steps // 2
        a = 4 * (p1 - p0) / (T ** 2)
        positions = []
        for i in range(steps + 1):
            t = i * dt
            if i <= half:
                pos = p0 + 0.5 * a * t**2
            else:
                t1 = t - T / 2
                vmax = a * (T / 2)
                pmid = p0 + 0.5 * a * (T / 2)**2
                pos = pmid + vmax * t1 - 0.5 * a * t1**2
            positions.append(pos)
        return positions

    z_traj = triangle_profile(start_z, tar_z, T, dt)
    j1_arr = [servo_inverse_kinematics(z) for z in z_traj]

    dt_s = dt / 1000.0
    v1 = [(j1_arr[i+1] - j1_arr[i]) / dt_s for i in range(len(j1_arr)-1)]
    v1.append(0)

    times = [dt] * len(j1_arr)
    pvt1 = [[j1_arr[i], v1[i], times[i]] for i in range(len(j1_arr))]
    return pvt1

def arm_generate_single_straight_pvt_points(start_coor, tar_coor, T, dt):
    def triangle_profile(p0, p1, T, dt):
        steps = int(T / dt)
        half = steps // 2
        a = 4 * (p1 - p0) / (T ** 2)
        positions = []
        for i in range(steps + 1):
            t = i * dt
            if i <= half:
                pos = p0 + 0.5 * a * t**2
            else:
                t1 = t - T / 2
                vmax = a * (T / 2)
                pmid = p0 + 0.5 * a * (T / 2)**2
                pos = pmid + vmax * t1 - 0.5 * a * t1**2
            positions.append(pos)
        return positions

    sx, sy, syaw = start_coor
    tx, ty, tyaw = tar_coor

    x_traj = triangle_profile(sx, tx, T, dt)
    y_traj = triangle_profile(sy, ty, T, dt)
    yaw_traj = triangle_profile(syaw, tyaw, T, dt)

    j2_arr, j3_arr, j4_arr = [], [], []
    for x, y, yaw in zip(x_traj, y_traj, yaw_traj):
        a2, a3, a4 = arm_inverse_kinematics(x, y, yaw)
        j2_arr.append(a2)
        j3_arr.append(a3)
        j4_arr.append(a4)

    dt_s = dt / 1000.0
    def compute_vel(arr):
        v = [(arr[i+1] - arr[i]) / dt_s for i in range(len(arr)-1)]
        v.append(0)
        return v

    v2, v3, v4 = map(compute_vel, [j2_arr, j3_arr, j4_arr])
    times = [dt] * len(j2_arr)

    pvt2 = [[j2_arr[i], v2[i], times[i]] for i in range(len(j2_arr))]
    pvt3 = [[j3_arr[i], v3[i], times[i]] for i in range(len(j3_arr))]
    pvt4 = [[j4_arr[i], v4[i], times[i]] for i in range(len(j4_arr))]
    return pvt2, pvt3, pvt4


import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def run_trajectory_pipeline(input_csv, output_csv,
                            step_mm,
                            angle_threshold,
                            vmax,
                            vsafe,
                            t_acc_ms,
                            dt_ms,
                            enable_static,
                            enable_animation):
    # =========================
    # Step 1: Deteksi zona berbahaya (XYZ saja, C diabaikan)
    # =========================
    def dangerous_angle_detection(csv_in, step_mm, angle_threshold):
        df = pd.read_csv(csv_in)
        pts = df[["X", "Y", "Z"]].values  # hanya XYZ untuk analisis

        dists = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        cum_dist = np.insert(np.cumsum(dists), 0, 0.0)

        n = len(pts)
        angles = np.full(n, np.nan)
        flags = np.zeros(n, dtype=int)

        for i in range(n):
            l = np.searchsorted(cum_dist, cum_dist[i]-step_mm, side="right")-1
            r = np.searchsorted(cum_dist, cum_dist[i]+step_mm, side="left")
            if 0 <= l < i and r > i and r < n:
                v1, v2 = pts[i]-pts[l], pts[r]-pts[i]
                cosv = np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2))
                ang = np.degrees(np.arccos(np.clip(cosv,-1,1)))
                angles[i] = ang
                flags[i] = ang > angle_threshold

        df["angle_deg"], df["dangerous"] = angles, flags.astype(int)
        return df

    # =========================
    # Utility functions
    # =========================
    def compute_path_length(points_xyz):
        seg_lengths = np.sqrt(np.sum(np.diff(points_xyz, axis=0)**2, axis=1))
        return seg_lengths, np.insert(np.cumsum(seg_lengths), 0, 0)

    def trapezoid_profile(D, v_start, v_end, vmax, t_acc_ms, dt_ms):
        dt = dt_ms/1000.0
        t_acc = t_acc_ms/1000.0
        t_dec = t_acc
        acc = (vmax-v_start)/t_acc if t_acc>0 else float("inf")
        dec = (vmax-v_end)/t_dec if t_dec>0 else float("inf")
        s_acc = (vmax**2-v_start**2)/(2*acc) if acc>0 else 0
        s_dec = (vmax**2-v_end**2)/(2*dec) if dec>0 else 0
        if s_acc+s_dec > D:
            vmax = math.sqrt((2*D*acc*dec+dec*v_start**2+acc*v_end**2)/(acc+dec))
            s_acc = (vmax**2-v_start**2)/(2*acc)
            s_dec = (vmax**2-v_end**2)/(2*dec)
            s_cruise=0
        else:
            s_cruise = D-(s_acc+s_dec)
        t_acc = (vmax-v_start)/acc if acc>0 else 0
        t_dec = (vmax-v_end)/dec if dec>0 else 0
        t_cruise = s_cruise/vmax if vmax>0 else 0
        T = t_acc+t_cruise+t_dec
        times = np.arange(0,T+dt,dt)
        v_vals=[]
        for t in times:
            if t<t_acc: v=v_start+acc*t
            elif t<t_acc+t_cruise: v=vmax
            elif t<=T: v=vmax-dec*(t-(t_acc+t_cruise))
            else: v=v_end
            v_vals.append(max(v,0))
        return times, np.array(v_vals)

    def build_full_profile(points_xyzc, seg_lengths, vmax, vsafe, t_acc_ms, dt_ms):
        times_all,v_all,s_all=[],[],[]
        offset_s,v_cur,t_off,i=0,0,0,0
        while i < len(seg_lengths):
            safe_len=0
            while i<len(seg_lengths) and points_xyzc[i,4]==0 and points_xyzc[i+1,4]==0:
                safe_len+=seg_lengths[i]; i+=1
            if safe_len>0:
                v_end = vsafe if i<len(seg_lengths) else 0
                t,v=trapezoid_profile(safe_len,v_cur,v_end,vmax,t_acc_ms,dt_ms)
                s=np.cumsum(v)*(dt_ms/1000.0)
                times_all.extend(t_off+t); v_all.extend(v); s_all.extend(offset_s+s)
                v_cur,t_off,offset_s=v_end,t_off+t[-1],offset_s+s[-1]
            danger_len=0
            while i<len(seg_lengths) and (points_xyzc[i,4]==1 or points_xyzc[i+1,4]==1):
                danger_len+=seg_lengths[i]; i+=1
            if danger_len>0:
                dt=dt_ms/1000.0
                steps=int(danger_len/(vsafe*dt))+1
                t=np.linspace(0,steps*dt,steps)
                v=np.ones_like(t)*vsafe
                s=np.cumsum(v)*dt
                times_all.extend(t_off+t); v_all.extend(v); s_all.extend(offset_s+s)
                v_cur,t_off,offset_s=vsafe,t_off+t[-1],offset_s+s[-1]
        return np.array(times_all),np.array(s_all),np.array(v_all)

    def interpolate_position(points_xyz, cum_lengths, s):
        idx=np.searchsorted(cum_lengths,s)-1
        idx=max(0,min(idx,len(points_xyz)-2))
        s0,s1=cum_lengths[idx],cum_lengths[idx+1]
        r=(s-s0)/(s1-s0+1e-9)
        return (1-r)*points_xyz[idx,:3]+r*points_xyz[idx+1,:3], idx

    def export_profile_to_csv(filename_out,times,s_vals,v_vals,points_xyzc,cum_lengths):
        rows=[]
        for t,s,v in zip(times,s_vals,v_vals):
            pos,idx=interpolate_position(points_xyzc[:,:3], cum_lengths, s)
            danger=points_xyzc[idx,4]    # flag dangerous
            c_val = points_xyzc[idx,3]   # nilai C ikut diekspor
            rows.append([t,s,v,pos[0],pos[1],pos[2],c_val,danger])
        pd.DataFrame(rows,columns=["time_s","s_mm","v_mm_s","X","Y","Z","C","dangerous"]).to_csv(filename_out,index=False)
        print(f"CSV disimpan ke {filename_out}")

    # =========================
    # Plot statis
    # =========================
    def plot_trajectory(df):
        pts=df[["X","Y","Z"]].values; flags=df["dangerous"].values.astype(bool)
        fig=plt.figure(figsize=(8,6))
        ax=fig.add_subplot(111,projection="3d")
        for safe,color in zip([~flags,flags],["grey","red"]):
            idx=np.where(safe[:-1]&safe[1:])[0]
            for i in idx:
                ax.plot(pts[i:i+2,0],pts[i:i+2,1],pts[i:i+2,2],color=color)
        ax.scatter(*pts[0],c="green",s=60,label="Start")
        ax.scatter(*pts[-1],c="orange",s=60,label="End")
        ax.set_title("Trajectory with Dangerous Zones")
        ax.legend(); plt.show()

    # =========================
    # Animasi (versi 3D dengan subsampling)
    # =========================
    def animate_trajectory(times, s_vals, v_vals, points_xyzc, cum_lengths, dt_ms,
                           frame_step=2, max_frames=1000, trail_window=300):
        if len(times) == 0:
            raise ValueError("Timeline kosong.")

        # Subsample
        if max_frames is not None and len(times) > max_frames:
            frame_idx = np.linspace(0, len(times)-1, max_frames, dtype=int)
            interval_ms = int((times[frame_idx[1]]-times[frame_idx[0]])*1000)
        else:
            frame_idx = np.arange(0,len(times),frame_step,dtype=int)
            interval_ms = int(dt_ms*frame_step)

        # Precompute posisi
        idx = np.searchsorted(cum_lengths, s_vals, side="right") - 1
        idx = np.clip(idx, 0, len(cum_lengths)-2)
        s0, s1 = cum_lengths[idx], cum_lengths[idx+1]
        r = (s_vals-s0)/(s1-s0+1e-9)
        P0 = points_xyzc[idx,:3]; P1 = points_xyzc[idx+1,:3]
        pos = (1.0-r)[:,None]*P0 + r[:,None]*P1

        fig = plt.figure(figsize=(12,6))
        ax1 = fig.add_subplot(121,projection="3d")
        ax2 = fig.add_subplot(122)

        # lintasan statis
        for i in range(len(points_xyzc)-1):
            color = "gray" if points_xyzc[i,4]==0 else "red"
            ax1.plot(points_xyzc[i:i+2,0],points_xyzc[i:i+2,1],points_xyzc[i:i+2,2],color=color,linewidth=1)

        trail, = ax1.plot([],[],[],linewidth=2)
        point, = ax1.plot([],[],[],"o",markersize=6)

        ax1.set_xlim(points_xyzc[:,0].min(), points_xyzc[:,0].max())
        ax1.set_ylim(points_xyzc[:,1].min(), points_xyzc[:,1].max())
        ax1.set_zlim(points_xyzc[:,2].min(), points_xyzc[:,2].max())
        ax1.set_title("Trajectory Animation")

        ax2.plot(times, v_vals, linewidth=1)
        marker, = ax2.plot([],[],"ro")
        ax2.set_xlim(0,float(times[-1])); ax2.set_ylim(0,float(max(v_vals)*1.1))
        ax2.set_title("Velocity Profile"); ax2.set_xlabel("Time [s]"); ax2.set_ylabel("mm/s")

        def init():
            trail.set_data([],[]); trail.set_3d_properties([])
            point.set_data([],[]); point.set_3d_properties([])
            marker.set_data([],[])
            return trail, point, marker

        def update(j):
            f=int(frame_idx[j])
            px,py,pz = pos[f]
            point.set_data([px],[py]); point.set_3d_properties([pz])

            start = max(0,f-trail_window) if trail_window else 0
            trail.set_data(pos[start:f+1,0],pos[start:f+1,1])
            trail.set_3d_properties(pos[start:f+1,2])

            marker.set_data([times[f]],[v_vals[f]])
            return point, trail, marker

        ani = FuncAnimation(fig,update,init_func=init,
                            frames=len(frame_idx),interval=interval_ms,
                            blit=False,repeat=False)
        fig._ani = ani
        plt.show()

    # =========================
    # Pipeline
    # =========================
    df = dangerous_angle_detection(input_csv, step_mm, angle_threshold)
    # Ambil X,Y,Z,C + dangerous (5 kolom)
    points = df[["X","Y","Z","C","dangerous"]].values
    seg_lengths, cum_lengths = compute_path_length(points[:,:3])
    times, s_vals, v_vals = build_full_profile(points, seg_lengths, vmax, vsafe, t_acc_ms, dt_ms)

    export_profile_to_csv(output_csv, times, s_vals, v_vals, points, cum_lengths)

    if enable_static:
        plot_trajectory(df)
    if enable_animation:
        animate_trajectory(times,s_vals,v_vals,points,cum_lengths,dt_ms)


# =========================
# Contoh pemanggilan
# =========================
if __name__ == "__main__":
    run_trajectory_pipeline(
        input_csv="pickup_shelf_20250922.csv",
        output_csv="resampled/resampled_pickup_shelf_20250922.csv",
        step_mm=2.0,
        angle_threshold=5,
        vmax=100,
        vsafe=10,
        t_acc_ms=2000,
        dt_ms=50,
        enable_static=False,
        enable_animation=False
    )


import pandas as pd
import math

def convert_to_pvt(input_csv, output_csv):
    df = pd.read_csv(input_csv)

    # dt (detik) dan dt_ms
    dt_s = df['time_s'].iloc[1] - df['time_s'].iloc[0]
    dt_ms = dt_s * 1000

    # posisi joint
    p1, p2, p3, p4 = [], [], [], []
    for _, row in df.iterrows():
        z, x, y, c = row['Z'], row['X'], row['Y'], row['C']
        p1.append(servo_inverse_kinematics(z))
        a2, a3, a4 = arm_inverse_kinematics(x, y, c)
        p2.append(a2); p3.append(a3); p4.append(a4)

    # velocity
    v1 = [(p1[i+1] - p1[i]) / dt_s for i in range(len(p1)-1)] + [0]
    v2 = [(p2[i+1] - p2[i]) / dt_s for i in range(len(p2)-1)] + [0]
    v3 = [(p3[i+1] - p3[i]) / dt_s for i in range(len(p3)-1)] + [0]
    v4 = [(p4[i+1] - p4[i]) / dt_s for i in range(len(p4)-1)] + [0]

    # time stamps (akumulasi)
    times = [i*dt_ms for i in range(len(p1))]

    # export CSV gabungan
    data = {
        'p1': p1, 'v1': v1,
        'p2': p2, 'v2': v2,
        'p3': p3, 'v3': v3,
        'p4': p4, 'v4': v4,
        'dt': [dt_ms]*len(p1)
    }
    df_out = pd.DataFrame(data)
    df_out.to_csv(output_csv, index=False)

    # return PVT per joint
    pvt1 = [[p1[i], v1[i], times[i]] for i in range(len(p1))]
    pvt2 = [[p2[i], v2[i], times[i]] for i in range(len(p2))]
    pvt3 = [[p3[i], v3[i], times[i]] for i in range(len(p3))]
    pvt4 = [[p4[i], v4[i], times[i]] for i in range(len(p4))]

    return pvt1, pvt2, pvt3, pvt4

result = convert_to_pvt("resampled/resampled_pickup_shelf_20250922.csv", "output_pvt.csv")