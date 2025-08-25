import numpy as np
import math
import matplotlib.pyplot as plt




# Konstanta rasio stepper (gantilah dengan nilai yang sesuai)
STEPPER_RATIO = 4000 / 360  # Contoh: 4000 pulses per 360 degrees

def stepper_degrees_to_pulses(degrees):
    return int(degrees * STEPPER_RATIO)

def inverse_kinematics(tar_coor):
    x, y, z, yaw = tar_coor
    # max area
    L2 = 137.0
    L3 = 121.0
    L4 = 56.82
    OFFSET_2 = -96.5
    OFFSET_3 = 134
    OFFSET_4 = -52.5
    #ration joint = 5:1

    distance = math.sqrt(x**2 + y**2)

    if distance > (L2 + L3):
        raise ValueError("out of boundary")

    # joint_3
    cos_theta3 = (x**2 + y**2 - L2**2 - L3**2) / (2 * L2 * L3)
    theta3 = math.acos(cos_theta3)  #angle in radian

    # joint_2
    k1 = L2 + L3 * cos_theta3
    k2 = L3 * math.sin(theta3)
    theta2 = math.atan2(y, x) - math.atan2(k2, k1)

    # rad to deg
    theta2 = math.degrees(theta2)
    theta3 = math.degrees(theta3)

    joint_1 = z * (360/90)
    joint_2 = (theta2-OFFSET_2)*5
    joint_3 = (theta3-OFFSET_3)*5 + joint_2
    joint_4 = (yaw-OFFSET_4)*5# + joint_3;
    
    if joint_1 > (13004):
        joint_1 = 13004
       # raise ValueError("out of joint_1 max limit")
    elif joint_1 < 0:
        joint_1 = 0
    if joint_2 > (178 * 5):
        joint_2 = 178 * 5
       # raise ValueError("out of joint_2 max limit")
    elif joint_2 < 0:
        joint_2 = 0
        #raise ValueError("out of joint_2 min limit")
    if joint_3 > (0 + joint_2):
        joint_3 = (0 + joint_2)
        #raise ValueError("out of joint_3 max limit")
    elif joint_3 < ((-135 * 5) + joint_2):
        joint_3 = (-135 * 5) + joint_2
        #raise ValueError("out of joint_2 min limit")
    if joint_4 > ((196 * 5) + joint_3):
        joint_4 = (196 * 5) + joint_3
        #raise ValueError("out of joint_4 max limit")
    elif joint_4 < (0 + joint_3):
        joint_4 = (0 + joint_3)
        #raise ValueError("out of joint_2 min limit")
        
    joint_4 *= -1
        
    return [joint_1, joint_2, joint_3, joint_4]

def gen_circular(cur_pos, center_pos, end_angle, travel_time, direction="CCW", pvt_time_interval=50):
    dt = pvt_time_interval / 1000  # Konversi ke detik
    num_steps = int(travel_time / dt) + 1

    start_angle = math.atan2(cur_pos[1] - center_pos[1], cur_pos[0] - center_pos[0])
    end_angle = start_angle + math.radians(end_angle) if direction.upper() == "CCW" else start_angle - math.radians(end_angle)

    # Radius lingkaran
    radius = math.sqrt((cur_pos[0] - center_pos[0]) ** 2 + (cur_pos[1] - center_pos[1]) ** 2)

    # Sudut tiap titik
    theta_points = np.linspace(start_angle, end_angle, num_steps)

    # Hitung posisi x, y berdasarkan sudut
    x_points = center_pos[0] + radius * np.cos(theta_points)
    y_points = center_pos[1] + radius * np.sin(theta_points)
    
    # Nilai konstan untuk z dan yaw
    z_points = np.zeros_like(x_points)
    yaw_points = np.zeros_like(x_points)
    
    trajectory_points = [x_points, y_points, z_points, yaw_points]
    
    return trajectory_points

def generate_pvt_points(joint_pulses, pvt_time_interval=50):
    dt = pvt_time_interval / 1000  # Konversi ke detik
    
    # Hitung velocity menggunakan numpy gradient
    velocity_points = np.gradient(joint_pulses, dt)

    position_points = np.round(joint_pulses).astype(int)
    velocity_points = np.round(velocity_points).astype(int)
    position_points_size = len(position_points)
    print(position_points_size)
    time_points = np.full(position_points_size, pvt_time_interval)
    
    return position_points, velocity_points, time_points

def pvt_circular(cur_pos, center_pos, end_angle, travel_time, direction="CCW", pvt_time_interval=50):
    trajectory_points = gen_circular(cur_pos, center_pos, end_angle, travel_time, direction)
    x_points, y_points, z_points, yaw_points = trajectory_points
    joint1_values, joint2_values, joint3_values, joint4_values = [], [], [], []
    for tar_coor in zip(*trajectory_points):  # zip untuk mengambil titik per titik (x, y, z, yaw)
        try:
            joint1, joint2, joint3, joint4 = inverse_kinematics(tar_coor)

            joint1_values.append(joint1)
            joint2_values.append(joint2)
            joint3_values.append(joint3)
            joint4_values.append(joint4)
        except ValueError as e:
            print(f"Inverse kinematics error at {tar_coor}: {e}")
            joint1_values.append(None)
            joint2_values.append(None)
            joint3_values.append(None)
            joint4_values.append(None)
    
    # Normalisasi nilai joint agar mulai dari 0
    joint1_values = [val - joint1_values[0] for val in joint1_values]
    joint2_values = [val - joint2_values[0] for val in joint2_values]
    joint3_values = [val - joint3_values[0] for val in joint3_values]
    joint4_values = [val - joint4_values[0] for val in joint4_values]
    #convert to pulses
    joint1_pulses = [stepper_degrees_to_pulses(val) for val in joint1_values]
    joint2_pulses = [stepper_degrees_to_pulses(val) for val in joint2_values]
    joint3_pulses = [stepper_degrees_to_pulses(val) for val in joint3_values]
    joint4_pulses = [stepper_degrees_to_pulses(val) for val in joint4_values]
    #convert pvt
    joint1_p, joint1_v, joint1_t = generate_pvt_points(joint1_pulses)
    joint2_p, joint2_v, joint2_t = generate_pvt_points(joint2_pulses)
    joint3_p, joint3_v, joint3_t = generate_pvt_points(joint3_pulses)
    joint4_p, joint4_v, joint4_t = generate_pvt_points(joint4_pulses)
    
    joint1_p, joint1_v, joint1_t = generate_pvt_points(joint1_pulses)
    joint2_p, joint2_v, joint2_t = generate_pvt_points(joint2_pulses)
    joint3_p, joint3_v, joint3_t = generate_pvt_points(joint3_pulses)
    joint4_p, joint4_v, joint4_t = generate_pvt_points(joint4_pulses)


# Contoh penggunaan
cur_pos = (130, 0)  # (x, y) posisi awal
center_pos = (170, 0)  # Pusat lingkaran
end_angle = 180  # Gerakan setengah lingkaran
travel_time = 4  # dalam detik
direction = "CCW"  # Arah rotasi

trajectory_points = gen_circular(cur_pos, center_pos, end_angle, travel_time, direction)
x_points, y_points, z_points, yaw_points = trajectory_points


# Plot Trajectory (X-Y)
plt.figure(figsize=(6, 6))
plt.plot(x_points, y_points, label="Trajectory", marker='o')
plt.scatter(cur_pos[0], cur_pos[1], color='red', label="Start Position")
plt.scatter(center_pos[0], center_pos[1], color='green', label="Center Position")
plt.xlabel("X Posisi")
plt.ylabel("Y Posisi")
plt.legend()
plt.grid()
plt.title("Lintasan Circular pada Sumbu X-Y")
plt.axis("equal")  # Pastikan skala X dan Y sama agar lingkaran tidak terdistorsi
plt.show()
print("x_points size:", len(x_points))

joint1_values, joint2_values, joint3_values, joint4_values = [], [], [], []

for tar_coor in zip(*trajectory_points):  # zip untuk mengambil titik per titik (x, y, z, yaw)
    try:
        joint1, joint2, joint3, joint4 = inverse_kinematics(tar_coor)

        joint1_values.append(joint1)
        joint2_values.append(joint2)
        joint3_values.append(joint3)
        joint4_values.append(joint4)
    except ValueError as e:
        print(f"Inverse kinematics error at {tar_coor}: {e}")
        joint1_values.append(None)
        joint2_values.append(None)
        joint3_values.append(None)
        joint4_values.append(None)


        
plt.figure(figsize=(10, 8))

# Plot Joint 1
plt.subplot(4, 1, 1)
plt.plot(range(len(joint1_values)), joint1_values, marker='o', label="Joint 1")
plt.xlabel("Data Index")
plt.ylabel("Joint 1 Value")
plt.legend()
plt.grid()

# Plot Joint 2
plt.subplot(4, 1, 2)
plt.plot(range(len(joint2_values)), joint2_values, marker='o', label="Joint 2", color="orange")
plt.xlabel("Data Index")
plt.ylabel("Joint 2 Value")
plt.legend()
plt.grid()

# Plot Joint 3
plt.subplot(4, 1, 3)
plt.plot(range(len(joint3_values)), joint3_values, marker='o', label="Joint 3", color="green")
plt.xlabel("Data Index")
plt.ylabel("Joint 3 Value")
plt.legend()
plt.grid()

# Plot Joint 4
plt.subplot(4, 1, 4)
plt.plot(range(len(joint4_values)), joint4_values, marker='o', label="Joint 4", color="red")
plt.xlabel("Data Index")
plt.ylabel("Joint 4 Value")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()


# Normalisasi nilai joint agar mulai dari 0
joint1_values = [val - joint1_values[0] for val in joint1_values]
joint2_values = [val - joint2_values[0] for val in joint2_values]
joint3_values = [val - joint3_values[0] for val in joint3_values]
joint4_values = [val - joint4_values[0] for val in joint4_values]



# Konversi joint values ke pulses
joint1_pulses = [stepper_degrees_to_pulses(val) for val in joint1_values]
joint2_pulses = [stepper_degrees_to_pulses(val) for val in joint2_values]
joint3_pulses = [stepper_degrees_to_pulses(val) for val in joint3_values]
joint4_pulses = [stepper_degrees_to_pulses(val) for val in joint4_values]

# Plot grafik nilai joint values dan pulses
plt.figure(figsize=(10, 12))

# Plot Joint 1 - Degrees
plt.subplot(4, 2, 1)
plt.plot(range(len(joint1_values)), joint1_values, marker='o', label="Joint 1 (Degrees)")
plt.xlabel("Data Index")
plt.ylabel("Degrees")
plt.legend()
plt.grid()

# Plot Joint 1 - Pulses
plt.subplot(4, 2, 2)
plt.plot(range(len(joint1_pulses)), joint1_pulses, marker='o', label="Joint 1 (Pulses)", color="purple")
plt.xlabel("Data Index")
plt.ylabel("Pulses")
plt.legend()
plt.grid()

# Plot Joint 2 - Degrees
plt.subplot(4, 2, 3)
plt.plot(range(len(joint2_values)), joint2_values, marker='o', label="Joint 2 (Degrees)", color="orange")
plt.xlabel("Data Index")
plt.ylabel("Degrees")
plt.legend()
plt.grid()

# Plot Joint 2 - Pulses
plt.subplot(4, 2, 4)
plt.plot(range(len(joint2_pulses)), joint2_pulses, marker='o', label="Joint 2 (Pulses)", color="brown")
plt.xlabel("Data Index")
plt.ylabel("Pulses")
plt.legend()
plt.grid()

# Plot Joint 3 - Degrees
plt.subplot(4, 2, 5)
plt.plot(range(len(joint3_values)), joint3_values, marker='o', label="Joint 3 (Degrees)", color="green")
plt.xlabel("Data Index")
plt.ylabel("Degrees")
plt.legend()
plt.grid()

# Plot Joint 3 - Pulses
plt.subplot(4, 2, 6)
plt.plot(range(len(joint3_pulses)), joint3_pulses, marker='o', label="Joint 3 (Pulses)", color="blue")
plt.xlabel("Data Index")
plt.ylabel("Pulses")
plt.legend()
plt.grid()

# Plot Joint 4 - Degrees
plt.subplot(4, 2, 7)
plt.plot(range(len(joint4_values)), joint4_values, marker='o', label="Joint 4 (Degrees)", color="red")
plt.xlabel("Data Index")
plt.ylabel("Degrees")
plt.legend()
plt.grid()

# Plot Joint 4 - Pulses
plt.subplot(4, 2, 8)
plt.plot(range(len(joint4_pulses)), joint4_pulses, marker='o', label="Joint 4 (Pulses)", color="cyan")
plt.xlabel("Data Index")
plt.ylabel("Pulses")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()


# def generate_pvt_points(joint_pulses, pvt_time_interval=50):
#     """
#     Mengonversi list joint pulses menjadi PVT Points.
    
#     Parameters:
#     - joint_pulses: List of pulse values untuk satu joint.
#     - pvt_time_interval: Interval waktu antara tiap titik (ms).

#     Returns:
#     - List PVT points [(position, velocity, time)]
#     """
#     pvt_points = []
#     dt = pvt_time_interval / 1000  # Konversi ke detik

#     for i in range(len(joint_pulses)):
#         position = joint_pulses[i]
#         velocity = 0 if i == 0 else (joint_pulses[i] - joint_pulses[i-1]) / dt
#         time = i * dt
#         pvt_points.append((position, velocity, time))
    
#     return pvt_points



# Contoh penggunaan
joint1_p, joint1_v, joint1_t = generate_pvt_points(joint1_pulses)
joint2_p, joint2_v, joint2_t = generate_pvt_points(joint2_pulses)
joint3_p, joint3_v, joint3_t = generate_pvt_points(joint3_pulses)
joint4_p, joint4_v, joint4_t = generate_pvt_points(joint4_pulses)

# Print beberapa PVT points pertama untuk Joint 1
print("Joint 1 PVT Points (position, velocity, time):")
for p, v, t in zip(joint2_p, joint2_v, joint2_t):  # Print 5 titik pertama
    print(f"{p} | {v} | {t}")