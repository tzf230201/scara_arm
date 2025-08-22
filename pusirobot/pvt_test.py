import numpy as np
import matplotlib.pyplot as plt
import math

pvt_time_interval = 50  # 100ms
dt = pvt_time_interval / 1000  # 100ms = 0.1s
STEPPER_PPR = 4096
STEPPER_RATIO = STEPPER_PPR / 360

def generate_pvt_trajectory(cur_pulse, tar_pulse, time_travel):
    # Define the number of intervals (100ms steps) for one direction
    dt = pvt_time_interval / 1000  # 100ms = 0.1s
    num_steps = int(time_travel / dt) + 1
    time_points_one_way = np.linspace(0, time_travel, num_steps)

    # Generate sinusoidal acceleration-based motion profile for forward motion
    t_norm = np.linspace(0, np.pi, num_steps)  # Normalize to sinusoidal shape
    position_forward = cur_pulse + (tar_pulse - cur_pulse) * (0.5 * (1 - np.cos(t_norm)))  # Sinusoidal motion

    # Compute velocity for forward motion
    velocity_forward = np.gradient(position_forward, dt)

    # Generate return motion using the same profile
    position_backward = position_forward[::-1]  # Reverse the motion
    velocity_backward = -velocity_forward[::-1]  # Reverse the velocity

    # Concatenate forward and backward motion
    time_points = np.concatenate([time_points_one_way, time_travel + time_points_one_way[1:]])
    position_points = np.concatenate([position_forward, position_backward[1:]])
    velocity_points = np.concatenate([velocity_forward, velocity_backward[1:]])

    # Round values for better readability
    position_points = np.round(position_points, 2)
    velocity_points = np.round(velocity_points, 2)
    time_points = np.round(time_points, 2)

    return position_points, velocity_points, time_points

# def generate_pvt_trajectory_triangle(cur_pulse, tar_pulse, travel_time, pvt_time_interval=100):
#     # Menghitung interval waktu dt dalam detik
#     dt = pvt_time_interval / 1000  # 100 ms = 0.1 s

#     # Hitung total perpindahan dan waktu setengah siklus
#     total_pulse = tar_pulse - cur_pulse
#     half_time = travel_time / 2

#     # Hitung percepatan maksimum (a = 2 * s / t^2)
#     max_acc = 2 * total_pulse / (travel_time * half_time)

#     # Inisialisasi list untuk menyimpan data
#     position_points = []
#     velocity_points = []
#     time_points = []

#     # Inisialisasi variabel
#     time = 0.0
#     position = cur_pulse

#     # Fase akselerasi
#     while time < half_time:
#         velocity = max_acc * time
#         position += velocity * dt
        
#         position_points.append(position)
#         velocity_points.append(velocity)
#         time_points.append(time)
        
#         time += dt

#     # Fase deselerasi
#     while time < travel_time:
#         velocity = max_acc * (travel_time - time)
#         position += velocity * dt
        
#         position_points.append(position)
#         velocity_points.append(velocity)
#         time_points.append(time)
        
#         time += dt

#     # Pastikan posisi akhir tepat di target
#     position_points.append(tar_pulse)
#     velocity_points.append(0.0)
#     time_points.append(travel_time)

#     # return position_points, velocity_points, time_points

#     # Filter out small changes in position
#     filtered_position_points = []
#     filtered_velocity_points = []
#     filtered_time_points = []
#     last_position = cur_pulse

#     for pos, vel, time in zip(position_points, velocity_points, time_points):
#         if abs(pos - last_position) >= STEPPER_RATIO / 2:  # Half degree threshold
#             filtered_position_points.append(pos)
#             filtered_velocity_points.append(vel)
#             filtered_time_points.append(time)
#             last_position = pos

#     return filtered_position_points, filtered_velocity_points, filtered_time_points

def generate_pvt_trajectory_triangle(cur_pulse, tar_pulse, travel_time):
    dt = pvt_time_interval / 1000  # Konversi interval waktu ke detik
    total_pulse = tar_pulse - cur_pulse
    
    # Menghitung percepatan maksimum (a = 4 * s / t^2)
    max_acc = 4 * total_pulse / (travel_time ** 2)
    print(f"Max acceleration: {max_acc} pulse/s^2")
    half_time = travel_time / 2  # Waktu mencapai kecepatan maksimum

    # Variabel untuk menyimpan data
    position_points = []
    velocity_points = []
    time_points = []

    # Fase akselerasi
    for i in range(1, 201):
        curr_time = (i - 1) * dt
        curr_position = 0.5 * max_acc * curr_time ** 2
        curr_velocity = max_acc * curr_time
        
        position_wr = math.floor(curr_position + 0.5) + cur_pulse
        velocity_wr = math.floor(curr_velocity + 0.5)
        
        position_points.append(position_wr)
        velocity_points.append(velocity_wr)
        time_points.append(curr_time)
        
        if curr_time >= half_time:
            break

    last_position = curr_position
    last_velocity = curr_velocity

    # Fase deselerasi
    for i in range(1, 201):
        curr_time = i * dt
        curr_velocity = last_velocity - max_acc * curr_time
        curr_position = last_position + last_velocity * curr_time - 0.5 * max_acc * curr_time ** 2
        
        position_wr = math.floor(curr_position + 0.5) + cur_pulse
        velocity_wr = math.floor(curr_velocity + 0.5)
        
        position_points.append(position_wr)
        velocity_points.append(velocity_wr)
        time_points.append(time_points[-1] + dt)
        
        if curr_velocity <= 0:
            break
        
    
    position_points = np.round(position_points).astype(int)
    velocity_points = np.round(velocity_points).astype(int)
    position_points_size = len(position_points)
    print(position_points_size)
    # time_points = np.full(position_points_size, pvt_time_interval)

    return position_points, velocity_points, time_points

def calculate_position_from_velocity(velocity_points, dt, start_position=0):
    position_points = [start_position]
    for vel in velocity_points:
        new_position = position_points[-1] + vel * dt
        position_points.append(new_position)
    return position_points

# Example Usage
cur_pulse = 0
tar_pulse = 1024
time_travel = 4  # seconds

position_points, velocity_points, time_points = generate_pvt_trajectory_triangle(cur_pulse, tar_pulse, time_travel)
last_t = 0
# Print output using a loop
print("Position (pulse) | Velocity (pulse/s) | Time (s)")
print("-----------------------------------------------")
for pos, vel, t in zip(position_points, velocity_points, time_points):
    print(f"{int(pos)}     | {int(vel)}           | {int((100))}")
    last_t = t
    

# Plot Position vs Time
plt.figure(figsize=(8, 4))
plt.plot(time_points, position_points, marker='o', linestyle='-', label="Position")
plt.xlabel("Time (s)")
plt.ylabel("Position (pulse)")
plt.title("PVT Trajectory with Sinusoidal Acceleration")
plt.grid(True)
plt.legend()
plt.show()

# Plot Velocity vs Time
plt.figure(figsize=(8, 4))
plt.plot(time_points, velocity_points, marker='o', linestyle='-', color='r', label="Velocity")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (pulse/s)")
plt.title("Velocity Profile of PVT Trajectory")
plt.grid(True)
plt.legend()
plt.show()

calculated_position = calculate_position_from_velocity(velocity_points, dt, cur_pulse)

# Buat grafik perbandingan posisi asli dan posisi yang dihitung dari kecepatan
plt.figure(figsize=(10, 6))
plt.plot(time_points, position_points, label="Posisi Asli (PVT)", linestyle='--', marker='o')
plt.plot(time_points, calculated_position[:-1], label="Posisi dari Kecepatan", linestyle='-', marker='x')

plt.xlabel("Waktu (s)")
plt.ylabel("Posisi (Pulse)")
plt.title("Perbandingan Posisi Asli dan Posisi dari Kecepatan")
plt.legend()
plt.grid(True)
plt.show()

print("Position (pulse) | Velocity (pulse/s) | Time (s)")
print("-----------------------------------------------")
for pos, vel, t in zip(calculated_position, velocity_points, time_points):
    print(f"{int(pos)}     | {int(vel)}           | {int((100))}")
    last_t = t