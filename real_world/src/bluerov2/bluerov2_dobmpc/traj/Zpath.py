#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#--------------------------------------
# Generate Z-shaped reference trajectory for NMPC in Tank Environment
# Tank dimensions: 18.15m x 2.5m x 1m (y=long, x=short)
# Z-shape parameters:
# - Start point: (-0.37743731689453125, -2.88159375, depth)
# - Z pattern: moves right, then diagonally down-left, then right again
# - Total width: 1.0m, Total height: 1.0m
# - Speed: Slow and smooth for precise control
#--------------------------------------

# Parameters
sample_time = 0.05   # Sample time (seconds)
duration = 180       # Total duration (seconds) - 3 minutes for complete Z
z_width = 1.0        # Width of Z pattern (meters)
z_height = 1.0       # Height of Z pattern (meters)

# Starting coordinates (provided by user)
start_x = -0.37743731689453125
start_y = -2.88159375
start_z = 0.23  # Fixed depth

# Define Z-shape waypoints
# Z字形包含3个线段：
# 1. 上横线：向右移动
# 2. 对角线：向右下移动
# 3. 下横线：向右移动

waypoints = [
    # 起始点
    [start_x, start_y, start_z],
    # 上横线结束点 (向右移动)
    [start_x + z_width, start_y, start_z],
    # 下横线起始点 (对角线结束点)
    [start_x, start_y - z_height, start_z],
    # 下横线结束点 (向右移动)
    [start_x + z_width, start_y - z_height, start_z],
]

# 为每个线段分配时间 (更快的速度)
segment_durations = [
    10,   # 上横线：10秒 (0.1 m/s)
    10,   # 对角线：10秒
    10    # 下横线：10秒
]

# Generate time sequence
t = np.arange(0, duration, sample_time)
t = np.append(t, duration)

# Calculate number of trajectory points
num_points = len(t)

# Create trajectory array (16 columns: x y z phi theta psi u v w p q r u1 u2 u3 u4)
traj = np.zeros((num_points, 16))

def interpolate_segment(start_point, end_point, t_start, t_end, current_time):
    """线性插值函数"""
    if current_time <= t_start:
        return start_point
    elif current_time >= t_end:
        return end_point
    else:
        # 线性插值
        alpha = (current_time - t_start) / (t_end - t_start)
        return start_point + alpha * (end_point - start_point)

def calculate_velocity(start_point, end_point, duration):
    """计算线段的速度"""
    if duration <= 0:
        return np.array([0.0, 0.0, 0.0])
    return (end_point - start_point) / duration

# Generate Z-shaped trajectory
for i, time in enumerate(t):
    current_pos = np.array([start_x, start_y, start_z])
    current_vel = np.array([0.0, 0.0, 0.0])
    
    # 确定当前在哪个线段
    if time <= segment_durations[0]:
        # 第一段：上横线
        start_point = np.array(waypoints[0])
        end_point = np.array(waypoints[1])
        current_pos = interpolate_segment(start_point, end_point, 0, segment_durations[0], time)
        current_vel = calculate_velocity(start_point, end_point, segment_durations[0])
        
    elif time <= segment_durations[0] + segment_durations[1]:
        # 第二段：对角线
        start_point = np.array(waypoints[1])
        end_point = np.array(waypoints[2])
        t_start = segment_durations[0]
        t_end = segment_durations[0] + segment_durations[1]
        current_pos = interpolate_segment(start_point, end_point, t_start, t_end, time)
        current_vel = calculate_velocity(start_point, end_point, segment_durations[1])
        
    else:
        # 第三段：下横线
        start_point = np.array(waypoints[2])
        end_point = np.array(waypoints[3])
        t_start = segment_durations[0] + segment_durations[1]
        t_end = sum(segment_durations)
        current_pos = interpolate_segment(start_point, end_point, t_start, t_end, time)
        current_vel = calculate_velocity(start_point, end_point, segment_durations[2])
    
    # Position
    traj[i, 0] = current_pos[0]  # x
    traj[i, 1] = current_pos[1]  # y
    traj[i, 2] = current_pos[2]  # z
    
    # Attitude (keep horizontal)
    traj[i, 3] = 0.0             # phi (roll)
    traj[i, 4] = 0.0             # theta (pitch)
    traj[i, 5] = 0.0             # psi (yaw)
    
    # Linear velocity
    traj[i, 6] = current_vel[0]  # u (x-direction velocity)
    traj[i, 7] = current_vel[1]  # v (y-direction velocity)
    traj[i, 8] = current_vel[2]  # w (z-direction velocity)
    
    # Angular velocity
    traj[i, 9] = 0.0             # p (roll rate)
    traj[i, 10] = 0.0            # q (pitch rate)
    traj[i, 11] = 0.0            # r (yaw rate)
    
    # Control inputs (set to 0, calculated by MPC)
    traj[i, 12] = 0.0            # u1
    traj[i, 13] = 0.0            # u2
    traj[i, 14] = 0.0            # u3
    traj[i, 15] = 0.0            # u4

# Add extra points for MPC prediction horizon
last_point = traj[-1, :]  # Get the last point
extra_points = np.tile(last_point, (20, 1))  # Repeat the last point 20 times
traj = np.vstack([traj, extra_points])  # Add to trajectory

# Save to file
output_filename = 'Zpath_trajectory.txt'
np.savetxt(output_filename, traj, fmt='%f')

# 打印轨迹信息
print(f"Z-shaped trajectory generated: {output_filename}")
print(f"Number of trajectory points: {traj.shape[0]}")
print(f"Start point: ({start_x:.6f}, {start_y:.6f}, {start_z:.6f})")
print(f"Z dimensions: {z_width}m x {z_height}m")
print(f"Total duration: {duration} seconds")
print(f"Sample time: {sample_time} seconds")
print()
print("Waypoints:")
for i, wp in enumerate(waypoints):
    print(f"  {i+1}: ({wp[0]:.6f}, {wp[1]:.6f}, {wp[2]:.6f})")
print()
print(f"X range: [{np.min(traj[:,0]):.3f}, {np.max(traj[:,0]):.3f}] m")
print(f"Y range: [{np.min(traj[:,1]):.3f}, {np.max(traj[:,1]):.3f}] m")
print(f"Z depth: {traj[0,2]:.3f} m (fixed)")

# Visualize trajectory
plt.figure(figsize=(15, 10))

# Tank boundary definition
tank_x_min, tank_x_max = -1.25, 1.25    # 2.5m width
tank_y_min, tank_y_max = -9.075, 9.075  # 18.15m length
tank_z_min, tank_z_max = -1.0, 0.0      # 1m depth

# 1. 3D plot - Overall trajectory
ax1 = plt.subplot(2, 3, 1, projection='3d')
ax1.plot(traj[:-20, 0], traj[:-20, 1], traj[:-20, 2], 'b-', linewidth=3, label='Z-shaped Trajectory')
ax1.scatter(traj[0, 0], traj[0, 1], traj[0, 2], color='green', s=100, label='Start Point')
ax1.scatter(traj[-21, 0], traj[-21, 1], traj[-21, 2], color='red', s=100, label='End Point')

# Mark waypoints
for i, wp in enumerate(waypoints):
    ax1.scatter(wp[0], wp[1], wp[2], color='orange', s=80, alpha=0.7)
    ax1.text(wp[0], wp[1], wp[2], f'  WP{i+1}', fontsize=8)

# Draw tank boundary
xx, yy = np.meshgrid([tank_x_min, tank_x_max], [tank_y_min, tank_y_max])
zz = np.full_like(xx, tank_z_min)
ax1.plot_surface(xx, yy, zz, alpha=0.3, color='gray')

ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)') 
ax1.set_zlabel('Z (m)')
ax1.set_title('3D Z-shaped Trajectory')
ax1.legend()

# 2. XY plane view (top view)
ax2 = plt.subplot(2, 3, 2)
ax2.plot(traj[:-20, 0], traj[:-20, 1], 'b-', linewidth=3, label='Z-shaped Trajectory')
ax2.scatter(traj[0, 0], traj[0, 1], color='green', s=100, label='Start Point')
ax2.scatter(traj[-21, 0], traj[-21, 1], color='red', s=100, label='End Point')

# Mark waypoints
for i, wp in enumerate(waypoints):
    ax2.scatter(wp[0], wp[1], color='orange', s=80, alpha=0.7)
    ax2.text(wp[0], wp[1], f'  WP{i+1}', fontsize=8)

# Draw direction arrows
for i in range(len(waypoints)-1):
    dx = waypoints[i+1][0] - waypoints[i][0]
    dy = waypoints[i+1][1] - waypoints[i][1]
    ax2.arrow(waypoints[i][0], waypoints[i][1], dx*0.8, dy*0.8, 
              head_width=0.02, head_length=0.02, fc='red', ec='red', alpha=0.7)

# Tank boundary
ax2.add_patch(plt.Rectangle((tank_x_min, tank_y_min), 
                           tank_x_max-tank_x_min, tank_y_max-tank_y_min, 
                           fill=False, edgecolor='gray', linewidth=2, label='Tank Boundary'))
ax2.set_xlabel('X (m)')
ax2.set_ylabel('Y (m)')
ax2.set_title('XY Plane View (Top View) - Z Pattern')
ax2.legend()
ax2.grid(True)
ax2.axis('equal')

# 3. X-time plot
ax3 = plt.subplot(2, 3, 3)
time_plot = t
ax3.plot(time_plot, traj[:len(time_plot), 0], 'r-', linewidth=2, label='X Position')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('X Position (m)')
ax3.set_title('X Position vs Time')
ax3.legend()
ax3.grid(True)

# 4. Y-time plot  
ax4 = plt.subplot(2, 3, 4)
ax4.plot(time_plot, traj[:len(time_plot), 1], 'g-', linewidth=2, label='Y Position')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Y Position (m)')
ax4.set_title('Y Position vs Time')
ax4.legend()
ax4.grid(True)

# 5. Z-time plot
ax5 = plt.subplot(2, 3, 5)
ax5.plot(time_plot, traj[:len(time_plot), 2], 'b-', linewidth=2, label='Z Position')
ax5.axhline(y=start_z, color='blue', linestyle='--', alpha=0.7, label=f'Fixed Depth: {start_z:.3f}')
ax5.set_xlabel('Time (s)')
ax5.set_ylabel('Z Position (m)')
ax5.set_title('Z Position vs Time')
ax5.legend()
ax5.grid(True)

# 6. Velocity plot
ax6 = plt.subplot(2, 3, 6)
ax6.plot(time_plot, traj[:len(time_plot), 6], 'r-', linewidth=2, label='X Velocity (u)')
ax6.plot(time_plot, traj[:len(time_plot), 7], 'g-', linewidth=2, label='Y Velocity (v)')
ax6.plot(time_plot, traj[:len(time_plot), 8], 'b-', linewidth=2, label='Z Velocity (w)')
ax6.set_xlabel('Time (s)')
ax6.set_ylabel('Velocity (m/s)')
ax6.set_title('Velocity vs Time')
ax6.legend()
ax6.grid(True)

plt.tight_layout()
plt.savefig('Zpath_trajectory_visualization.png', dpi=300, bbox_inches='tight')
plt.show()

print(f"\nTrajectory visualization saved: Zpath_trajectory_visualization.png")

# Verify trajectory quality
print(f"\n=== Trajectory Quality Verification ===")
# Check velocity continuity
velocities = np.sqrt(traj[:-20,6]**2 + traj[:-20,7]**2 + traj[:-20,8]**2)
max_velocity = np.max(velocities)
print(f"Maximum velocity: {max_velocity:.6f} m/s")

# Check trajectory bounds
print(f"X bounds: [{np.min(traj[:-20,0]):.6f}, {np.max(traj[:-20,0]):.6f}] m")
print(f"Y bounds: [{np.min(traj[:-20,1]):.6f}, {np.max(traj[:-20,1]):.6f}] m")
print(f"Z variation: {np.max(traj[:-20,2]) - np.min(traj[:-20,2]):.6f} m (should be 0)")

print(f"\nZ-pattern dimensions:")
print(f"Width: {np.max(traj[:-20,0]) - np.min(traj[:-20,0]):.6f} m")
print(f"Height: {np.max(traj[:-20,1]) - np.min(traj[:-20,1]):.6f} m")
