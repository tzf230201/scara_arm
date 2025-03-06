import numpy as np
import matplotlib.pyplot as plt
import time
import math
import csv

def generate_trajectory(start, end, steps):
    x_points = np.linspace(start[0], end[0], steps)
    y_points = np.linspace(start[1], end[1], steps)
    c_points = np.linspace(start[2], end[2], steps)
    trajectory = list(zip(x_points, y_points, c_points))
    return trajectory

def transform_time_to_angle(current_time, start_time, travel_time):
    elapsed_time = current_time - start_time
    angle = ((elapsed_time / travel_time) * 180) - 90
    angle = min(max(angle, -90), 90)
    return angle

def sine_wave(current_time, start_time, travel_time, cur_pos, tar_pos):
    angle = transform_time_to_angle(current_time, start_time, travel_time)
    angle_radians = math.radians(angle)
    sin_value = math.sin(angle_radians)
    output = cur_pos + (sin_value + 1) / 2 * (tar_pos - cur_pos)
    return output

def inverse_kinematics(x, y, c):
    l2 = 137.0
    l3 = 121.0
    l4 = 56.82
    offset2 = -96.5
    offset3 = 134
    offset4 = -52.5
    
    distance = math.sqrt(x**2 + y**2)
    
    if distance > (l2 + l3):
        raise ValueError("out of boundary")
    
    cos_theta3 = (x**2 + y**2 - l2**2 - l3**2) / (2 * l2 * l3)
    cos_theta3 = max(min(cos_theta3, 1), -1)  # Clamp to the range [-1, 1]
    
    theta3 = math.acos(cos_theta3)
    
    k1 = l2 + l3 * cos_theta3
    k2 = l3 * math.sin(theta3)
    theta2 = math.atan2(y, x) - math.atan2(k2, k1)
    
    theta2 = math.degrees(theta2)
    theta3 = math.degrees(theta3)
    
    joint2 = (theta2 - offset2) * 5
    joint3 = (theta3 - offset3) * 5 + joint2
    joint4 = (c - offset4) * 5
    
    if joint2 > (178 * 5):
        joint2 = 178 * 5
        raise ValueError("out of joint2 max limit")
    elif joint2 < 0:
        joint2 = 0
        raise ValueError("out of joint2 min limit")
    if joint3 > (0 + joint2):
        joint3 = (0 + joint2)
        raise ValueError("out of joint3 max limit")
    elif joint3 < ((-135 * 5) + joint2):
        joint3 = (-135 * 5) + joint2
        raise ValueError("out of joint3 min limit")
    if joint4 > ((196 * 5) + joint3):
        joint4 = (196 * 5) + joint3
        raise ValueError("out of joint4 max limit")
    elif joint4 < (0 + joint3):
        joint4 = (0 + joint3)
        raise ValueError("out of joint4 min limit")

    return joint2, joint3, joint4

# Input coordinates: (cur_x, cur_y, cur_c) and (tar_x, tar_y, tar_c)
cur_x, cur_y, cur_c = 150, 0, 0
tar_x, tar_y, tar_c = 150,100, 90

# Define start time and travel time (in milliseconds)
start_time = time.time() * 1000  # Current time in milliseconds
travel_time = 4000  # Example: 5 seconds travel time
# Number of steps in the trajectory
steps = travel_time * 4

# Generate the trajectory
trajectory = generate_trajectory((cur_x, cur_y, cur_c), (tar_x, tar_y, tar_c), steps)


# Generate time values for the duration of the travel_time
time_values = np.linspace(start_time, start_time + travel_time, steps)

# Calculate sine wave values for x and y positions
x_over_time = [sine_wave(t, start_time, travel_time, cur_x, tar_x) for t in time_values]
y_over_time = [sine_wave(t, start_time, travel_time, cur_y, tar_y) for t in time_values]
c_over_time = [sine_wave(t, start_time, travel_time, cur_c, tar_c) for t in time_values]

trajectory_over_time = list(zip(x_over_time, y_over_time, c_over_time))

# Calculate joint angles for each point in the trajectory
joint2_values = []
joint3_values = []
joint4_values = []

for (x, y, c) in trajectory_over_time:
    try:
        joint2, joint3, joint4 = inverse_kinematics(x, y, c)
        joint2_values.append(joint2)
        joint3_values.append(joint3)
        joint4_values.append(joint4)
    except ValueError as e:
        print(f"Error calculating inverse kinematics for (x={x}, y={y}, c={c}): {e}")
        joint2_values.append(None)  # Use None to indicate out-of-bound values
        joint3_values.append(None)
        joint4_values.append(None)

# Plot the trajectory in one window
plt.figure(figsize=(8, 6))
plt.plot([point[0] for point in trajectory], [point[1] for point in trajectory], label='Trajectory')
plt.xlim(0, 258)
plt.ylim(-258, 258)
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Generated Trajectory')
plt.grid(True)
plt.legend()
plt.show()

# Plot x, y, and c over time in one window
plt.figure(figsize=(12, 8))

# Plot x and y over time
plt.subplot(3, 1, 1)
plt.plot(time_values - start_time, x_over_time, label='X over Time')
plt.xlabel('Time (ms)')
plt.ylabel('X Position')
plt.grid(True)
plt.legend()
plt.title('X and Y Position over Time')

plt.subplot(3, 1, 2)
plt.plot(time_values - start_time, y_over_time, label='Y over Time')
plt.xlabel('Time (ms)')
plt.ylabel('Y Position')
plt.grid(True)
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time_values - start_time, c_over_time, label='C over Time')
plt.xlabel('Time (ms)')
plt.ylabel('C Position')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()

# Plot joint angles over time
plt.figure(figsize=(12, 8))

# Plot joint2, joint3, and joint4 over time
plt.subplot(3, 1, 1)
plt.plot(time_values - start_time, joint2_values, label='Joint 2')
plt.xlabel('Time (ms)')
plt.ylabel('Joint 2 Angle')
plt.grid(True)
plt.legend()
plt.title('Joint Angles Over Time')

plt.subplot(3, 1, 2)
plt.plot(time_values - start_time, joint3_values, label='Joint 3')
plt.xlabel('Time (ms)')
plt.ylabel('Joint 3 Angle')
plt.grid(True)
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time_values - start_time, joint4_values, label='Joint 4')
plt.xlabel('Time (ms)')
plt.ylabel('Joint 4 Angle')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()

# Calculate displacement as if the joint position starts from zero
# Fungsi untuk menghitung diferensial posisi joint
def calculate_joint_displacement(joint_values):
    displacement = []
    for i in range(1, len(joint_values)):
        if joint_values[i] is not None and joint_values[i-1] is not None:
            delta_position = joint_values[i] - joint_values[i-1]
            displacement.append(delta_position)
        else:
            displacement.append(None)  # Jika ada nilai None, maka hasilnya None
    displacement.insert(0, 0)  # Tidak ada pergerakan pada titik awal, jadi displacement awal adalah 0
    return displacement

# Menghitung diferensial posisi untuk setiap joint
joint2_displacement = calculate_joint_displacement(joint2_values)
joint3_displacement = calculate_joint_displacement(joint3_values)
joint4_displacement = calculate_joint_displacement(joint4_values)

# Plot diferensial posisi joint seiring waktu
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(time_values - start_time, joint2_displacement, label='Joint 2 Displacement')
plt.xlabel('Time (ms)')
plt.ylabel('Joint 2 Displacement')
plt.grid(True)
plt.legend()
plt.title('Joint Displacement Over Time')

plt.subplot(3, 1, 2)
plt.plot(time_values - start_time, joint3_displacement, label='Joint 3 Displacement')
plt.xlabel('Time (ms)')
plt.ylabel('Joint 3 Displacement')
plt.grid(True)
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time_values - start_time, joint4_displacement, label='Joint 4 Displacement')
plt.xlabel('Time (ms)')
plt.ylabel('Joint 4 Displacement')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()

# Fungsi untuk menghitung integral dari diferensial posisi
def calculate_joint_position_from_displacement(displacement):
    position = [0]  # Posisi awal adalah 0
    for i in range(1, len(displacement)):
        if displacement[i] is not None:
            new_position = position[-1] + displacement[i]
            position.append(new_position)
        else:
            position.append(None)
    return position

joint2_integral = calculate_joint_position_from_displacement(joint2_displacement)
joint3_integral = calculate_joint_position_from_displacement(joint3_displacement)
joint4_integral = calculate_joint_position_from_displacement(joint4_displacement)

# Plot hasil integral
plt.figure(figsize=(12, 8))

# Plot joint2
plt.subplot(3, 1, 1)
plt.plot(joint2_values, label='Joint 2 Position')
plt.plot(joint2_integral, linestyle='--', label='Integral of Joint 2 Position')
plt.xlabel('Time Step')
plt.ylabel('Position')
plt.title('Joint 2 Position and Integral')
plt.legend()
plt.grid(True)

# Plot joint3
plt.subplot(3, 1, 2)
plt.plot(joint3_values, label='Joint 3 Position')
plt.plot(joint3_integral, linestyle='--', label='Integral of Joint 3 Position')
plt.xlabel('Time Step')
plt.ylabel('Position')
plt.title('Joint 3 Position and Integral')
plt.legend()
plt.grid(True)

# Plot joint4
plt.subplot(3, 1, 3)
plt.plot(joint4_values, label='Joint 4 Position')
plt.plot(joint4_integral, linestyle='--', label='Integral of Joint 4 Position')
plt.xlabel('Time Step')
plt.ylabel('Position')
plt.title('Joint 4 Position and Integral')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()



# Fungsi untuk menghitung diferensial posisi joint
def calculate_joint_displacement(joint_values):
    displacement = []
    for i in range(1, len(joint_values)):
        if joint_values[i] is not None and joint_values[i-1] is not None:
            delta_position = joint_values[i] - joint_values[i-1]
            displacement.append(delta_position)
        else:
            displacement.append(None)  # Jika ada nilai None, maka hasilnya None
    displacement.insert(0, 0)  # Tidak ada pergerakan pada titik awal, jadi displacement awal adalah 0
    return displacement

# Fungsi untuk menghitung kecepatan joint dalam RPM
def calculate_joint_speed(displacement, interval):
    speed_rpm = []
    for i in range(1, len(displacement)):
        if displacement[i] is not None and displacement[i-1] is not None:
            speed_dps = displacement[i] / interval
            speed_rpm.append((speed_dps * 60) / 360)
        else:
            speed_rpm.append(None)  # Jika ada nilai None, maka hasilnya None
    speed_rpm.insert(0, 0)  # Tidak ada kecepatan pada titik awal, jadi kecepatan awal adalah 0
    return speed_rpm

# Menghitung diferensial posisi untuk setiap joint
joint2_displacement = calculate_joint_displacement(joint2_values)
joint3_displacement = calculate_joint_displacement(joint3_values)
joint4_displacement = calculate_joint_displacement(joint4_values)

# Interval waktu antara setiap langkah
interval = (time_values[1] - time_values[0]) / 1000  # Konversi dari ms ke detik

# Menghitung kecepatan dalam RPM
joint2_speed_rpm = calculate_joint_speed(joint2_displacement, interval)
joint3_speed_rpm = calculate_joint_speed(joint3_displacement, interval)
joint4_speed_rpm = calculate_joint_speed(joint4_displacement, interval)

# Plot kecepatan joint dalam RPM
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(time_values - start_time, joint2_speed_rpm, label='Joint 2 Speed (RPM)')
plt.xlabel('Time (ms)')
plt.ylabel('Speed (RPM)')
plt.grid(True)
plt.legend()
plt.title('Joint 2 Speed Over Time')

plt.subplot(3, 1, 2)
plt.plot(time_values - start_time, joint3_speed_rpm, label='Joint 3 Speed (RPM)')
plt.xlabel('Time (ms)')
plt.ylabel('Speed (RPM)')
plt.grid(True)
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time_values - start_time, joint4_speed_rpm, label='Joint 4 Speed (RPM)')
plt.xlabel('Time (ms)')
plt.ylabel('Speed (RPM)')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
# Define the filename
filename = 'Downloads/joint_positions.csv'

# Open the file in write mode
with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file )
    
    # Write the header
    writer.writerow(['Time (ms)', 'Joint 2', 'Joint 3', 'Joint 4'])
    
    # Write the data
    for i, time_value in enumerate(time_values - start_time):
        # Write only if all joints have valid values
        if joint2_values[i] is not None and joint3_values[i] is not None and joint4_values[i] is not None:
            writer.writerow([time_value, joint2_values[i], joint3_values[i], joint4_values[i]])

print(f"Joint positions saved to {filename}")
