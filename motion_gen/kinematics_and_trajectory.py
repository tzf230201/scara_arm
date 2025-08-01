import numpy as np
import math 

def check_limit(tar_joints, source_info=""):
    tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4 = tar_joints
    tar_joint_4 *= -1

    if tar_joint_1 > (3551 * 4):
        print(f"[{source_info}] tar_joint_1 greater than {tar_joint_1}")
        tar_joint_1 = (3551 * 4)
    elif tar_joint_1 < 0:
        print(f"[{source_info}] tar_joint_1 lower than {tar_joint_1}")
        tar_joint_1 = 0

    if tar_joint_2 > (178 * 5):
        print(f"[{source_info}] tar_joint_2 greater than {tar_joint_2}")
        tar_joint_2 = 178 * 5
    elif tar_joint_2 < 0:
        print(f"[{source_info}] tar_joint_2 lower than {tar_joint_2}")
        tar_joint_2 = 0

    if tar_joint_3 > (0 + tar_joint_2):
        print(f"[{source_info}] tar_joint_3 greater than {tar_joint_3}")
        tar_joint_3 = (0 + tar_joint_2)
    elif tar_joint_3 < ((-135 * 5) + tar_joint_2):
        print(f"[{source_info}] tar_joint_3 lower than {tar_joint_3}")
        tar_joint_3 = (-135 * 5) + tar_joint_2

    if tar_joint_4 > ((196 * 5) + tar_joint_3):
        print(f"[{source_info}] tar_joint_4 greater than {tar_joint_4}")
        tar_joint_4 = (196 * 5) + tar_joint_3
    elif tar_joint_4 < (0 + tar_joint_3):
        print(f"[{source_info}] tar_joint_4 lower than {tar_joint_4}")
        tar_joint_4 = 0 + tar_joint_3

    tar_joint_4 *= -1
    return [tar_joint_1, tar_joint_2, tar_joint_3, tar_joint_4]


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
        
    joint_4 *= -1
    
    joints = [joint_1, joint_2, joint_3, joint_4]
    joints = check_limit(joints)
    
    return joints


def forward_kinematics(cur_joints):
    cur_deg1, cur_deg2, cur_deg3, cur_deg4 = cur_joints
    cur_deg4 *= -1.0
    L2 = 137.0
    L3 = 121.0
    OFFSET_2 = -96.5
    OFFSET_3 = 134.0
    OFFSET_4 = -52.5

    theta2_rad = math.radians((cur_deg2 / 5) + OFFSET_2)
    theta3_rad = math.radians((cur_deg3 / 5) + OFFSET_3 - (cur_deg2 / 5))

    x2 = L2 * math.cos(theta2_rad)
    y2 = L2 * math.sin(theta2_rad)
    x3 = x2 + L3 * math.cos(theta2_rad + theta3_rad)
    y3 = y2 + L3 * math.sin(theta2_rad + theta3_rad)
    z = (cur_deg1 / 360) * 90
    yaw = (cur_deg4 / 5) + OFFSET_4

    return [x3, y3, z, yaw]


def triangle_profile(p0, p1, T, dt=20):
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


def generate_trajectory_triangle(cur_coor, list_tar_coor, dt=20):
    time_vals = []
    total_time = 0
    current = cur_coor
    traj = [[] for _ in range(4)]

    for target, T in list_tar_coor:
        for j in range(4):  # x, y, z, yaw
            interp = triangle_profile(current[j], target[j], T, dt)
            traj[j].extend(interp)
        time_vals.extend(np.arange(total_time, total_time + T + dt, dt))
        total_time += T
        current = target

    return time_vals, traj[0], traj[1], traj[2], traj[3]


def convert_cartesian_traj_to_joint_traj(x_list, y_list, z_list, yaw_list):
    joint1_list, joint2_list, joint3_list, joint4_list = [], [], [], []

    for i, (x, y, z, yaw) in enumerate(zip(x_list, y_list, z_list, yaw_list)):
        joints = inverse_kinematics([x, y, z, yaw])
        
        
        joint1_list.append(joints[0])
        joint2_list.append(joints[1])
        joint3_list.append(joints[2])
        joint4_list.append(joints[3])

    return joint1_list, joint2_list, joint3_list, joint4_list
