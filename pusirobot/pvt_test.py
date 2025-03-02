import numpy as np
import matplotlib.pyplot as plt

def generate_pvt_trajectory(cur_pulse, tar_pulse, time_travel):
    """
    Generates a Position-Velocity-Time (PVT) trajectory using sinusoidal acceleration/deceleration,
    with rounded values for better readability. The motion moves from cur_pulse to tar_pulse and back.

    Parameters:
    cur_pulse (int): Current pulse position.
    tar_pulse (int): Target pulse position.
    time_travel (float): Total travel time in seconds (for one way).

    Returns:
    time_points (numpy array): Time points at 100ms intervals.
    position_points (numpy array): Corresponding rounded position values.
    velocity_points (numpy array): Corresponding rounded velocity values.
    """
    # Define the number of intervals (100ms steps) for one direction
    dt = 0.1  # 100ms = 0.1s
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

    return time_points, position_points, velocity_points

# Example Usage
cur_pulse = 0
tar_pulse = 10000
time_travel = 5  # seconds

time_points, position_points, velocity_points = generate_pvt_trajectory(cur_pulse, tar_pulse, time_travel)
last_t = 0
# Print output using a loop
print("Position (pulse) | Velocity (pulse/s) | Time (s)")
print("-----------------------------------------------")
for t, pos, vel in zip(time_points, position_points, velocity_points):
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