#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#--------------------------------------
# Generate Circular reference trajectory for NMPC in Tank Environment
# Tank dimensions: 18.15m x 2.5m x 1m
# Circle parameters:
# - Center: (0.5090687255859375, -0.8082335205078125, 0.2328541259765625)
# - Radius: 0.5m (50cm)
# - Motion: XY plane circular motion, Z fixed
# - Speed: Slow and smooth
#--------------------------------------

# Parameters
sample_time = 0.05   # Sample time (seconds)
duration = 120       # Total duration (seconds) - 2 minutes to complete one circle
radius = 0.3         # Radius (meters) - 50cm

# Circle center coordinates
center_x = -0.8264920043945313
center_y = -3.553818115234375
center_z = 0.2328541259765625



# Calculate angular velociy (rad/s) - 2π radians / 120 seconds = 0.0524 rad/s
angular_velocity = 2 * np.pi / duration

# Generate time sequence
t = np.arange(0, duration, sample_time)
t = np.append(t, duration)

# Calculate number of trajectory points
num_points = len(t)

# Create trajectory array (16 columns: x y z phi theta psi u v w p q r u1 u2 u3 u4)
traj = np.zeros((num_points, 16))

# Generate circular trajectory
for i, time in enumerate(t):
    # Current angle
    angle = angular_velocity * time
    
    # Position (circular parametric equations)
    traj[i, 0] = center_x + radius * np.cos(angle)           # x
    traj[i, 1] = center_y + radius * np.sin(angle)           # y
    traj[i, 2] = center_z                                     # z (fixed)
    
    # Attitude (keep horizontal)
    traj[i, 3] = 0.0                                          # phi (roll)
    traj[i, 4] = 0.0                                          # theta (pitch)
    traj[i, 5] = 0.0                                          # psi (yaw)
    
    # Linear velocity (derivative of position)
    traj[i, 6] = -radius * angular_velocity * np.sin(angle)   # u (x-direction velocity)
    traj[i, 7] = radius * angular_velocity * np.cos(angle)    # v (y-direction velocity)
    traj[i, 8] = 0.0                                          # w (z-direction velocity)
    
    # Angular velocity
    traj[i, 9] = 0.0                                          # p (roll rate)
    traj[i, 10] = 0.0                                         # q (pitch rate)
    traj[i, 11] = 0.0                                         # r (yaw rate)
    
    # Control inputs (set to 0, calculated by MPC)
    traj[i, 12] = 0.0                                         # u1
    traj[i, 13] = 0.0                                         # u2
    traj[i, 14] = 0.0                                         # u3
    traj[i, 15] = 0.0                                         # u4

# Add extra points for MPC prediction horizon (reference tank_dob.py)
last_point = traj[-1, :]  # Get the last point
extra_points = np.tile(last_point, (20, 1))  # Repeat the last point 20 times
traj = np.vstack([traj, extra_points])  # Add to trajectory

# Save to file
output_filename = 'circle_trajectory.txt'
np.savetxt(output_filename, traj, fmt='%f')

# 打印轨迹信息
print(f"Circular trajectory generated: {output_filename}")
print(f"Number of trajectory points: {traj.shape[0]}")
print(f"Center: ({center_x:.6f}, {center_y:.6f}, {center_z:.6f})")
print(f"Radius: {radius} m")
print(f"Total duration: {duration} seconds")
print(f"Sample time: {sample_time} seconds")
print(f"Angular velocity: {angular_velocity:.6f} rad/s")
print(f"Linear velocity: {radius * angular_velocity:.6f} m/s")
print()
print(f"X range: [{np.min(traj[:,0]):.3f}, {np.max(traj[:,0]):.3f}] m")
print(f"Y range: [{np.min(traj[:,1]):.3f}, {np.max(traj[:,1]):.3f}] m")
print(f"Z depth: {traj[0,2]:.3f} m (fixed)")
print()
print(f"Start point: ({traj[0,0]:.6f}, {traj[0,1]:.6f}, {traj[0,2]:.6f})")
print(f"End point: ({traj[-21,0]:.6f}, {traj[-21,1]:.6f}, {traj[-21,2]:.6f})")
print(f"Start and end points should be the same (circular closure)")

# Visualize trajectory
plt.figure(figsize=(15, 10))

# Tank boundary definition
tank_x_min, tank_x_max = -9.075, 9.075  # 18.15m length
tank_y_min, tank_y_max = -1.25, 1.25    # 2.5m width  
tank_z_min, tank_z_max = -1.0, 0.0      # 1m depth

# 1. 3D plot - Overall trajectory
ax1 = plt.subplot(2, 3, 1, projection='3d')
ax1.plot(traj[:-20, 0], traj[:-20, 1], traj[:-20, 2], 'b-', linewidth=2, label='Circular Trajectory')
ax1.scatter(traj[0, 0], traj[0, 1], traj[0, 2], color='green', s=100, label='Start/End Point')
ax1.scatter(center_x, center_y, center_z, color='red', s=100, label=f'Center({center_x:.3f}, {center_y:.3f})')

# Draw tank boundary
xx, yy = np.meshgrid([tank_x_min, tank_x_max], [tank_y_min, tank_y_max])
zz = np.full_like(xx, tank_z_min)
ax1.plot_surface(xx, yy, zz, alpha=0.3, color='gray')

ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)') 
ax1.set_zlabel('Z (m)')
ax1.set_title('3D Tank Circular Trajectory')
ax1.legend()
ax1.set_xlim(tank_x_min, tank_x_max)
ax1.set_ylim(tank_y_min, tank_y_max)
ax1.set_zlim(tank_z_min, tank_z_max)

# 2. XY plane view (top view)
ax2 = plt.subplot(2, 3, 2)
ax2.plot(traj[:-20, 0], traj[:-20, 1], 'b-', linewidth=2, label='Circular Trajectory')
ax2.scatter(traj[0, 0], traj[0, 1], color='green', s=100, label='Start/End Point')
ax2.scatter(center_x, center_y, color='red', s=100, label=f'Center({center_x:.3f}, {center_y:.3f})')

# Draw theoretical circle
theta = np.linspace(0, 2*np.pi, 100)
circle_x = center_x + radius * np.cos(theta)
circle_y = center_y + radius * np.sin(theta)
ax2.plot(circle_x, circle_y, 'r--', alpha=0.7, label='Theoretical Circle')

# Tank boundary
ax2.add_patch(plt.Rectangle((tank_x_min, tank_y_min), 
                           tank_x_max-tank_x_min, tank_y_max-tank_y_min, 
                           fill=False, edgecolor='gray', linewidth=2, label='Tank Boundary'))
ax2.set_xlabel('X (m)')
ax2.set_ylabel('Y (m)')
ax2.set_title('XY Plane View (Top View)')
ax2.legend()
ax2.grid(True)
ax2.axis('equal')
ax2.set_xlim(center_x-radius-1, center_x+radius+1)
ax2.set_ylim(center_y-radius-1, center_y+radius+1)

# 3. X-time plot
ax3 = plt.subplot(2, 3, 3)
time_plot = t
ax3.plot(time_plot, traj[:len(time_plot), 0], 'r-', linewidth=2, label='X Position')
ax3.axhline(y=center_x, color='red', linestyle='--', alpha=0.7, label=f'Center X: {center_x:.3f}')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('X Position (m)')
ax3.set_title('X Position vs Time')
ax3.legend()
ax3.grid(True)

# 4. Y-time plot  
ax4 = plt.subplot(2, 3, 4)
ax4.plot(time_plot, traj[:len(time_plot), 1], 'g-', linewidth=2, label='Y Position')
ax4.axhline(y=center_y, color='green', linestyle='--', alpha=0.7, label=f'Center Y: {center_y:.3f}')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Y Position (m)')
ax4.set_title('Y Position vs Time')
ax4.legend()
ax4.grid(True)

# 5. Z-time plot
ax5 = plt.subplot(2, 3, 5)
ax5.plot(time_plot, traj[:len(time_plot), 2], 'b-', linewidth=2, label='Z Position')
ax5.axhline(y=center_z, color='blue', linestyle='--', alpha=0.7, label=f'Fixed Depth: {center_z:.3f}')
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
plt.savefig('circle_trajectory_visualization.png', dpi=300, bbox_inches='tight')
plt.show()

print(f"\nTrajectory visualization saved: circle_trajectory_visualization.png")

# Verify trajectory quality
print(f"\n=== Trajectory Quality Verification ===")
# Check if start and end points are close
start_end_distance = np.sqrt((traj[0,0] - traj[-21,0])**2 + (traj[0,1] - traj[-21,1])**2)
print(f"Start to end distance: {start_end_distance:.6f} m (should be close to 0)")

# Check if radius is constant
radii = np.sqrt((traj[:-20,0] - center_x)**2 + (traj[:-20,1] - center_y)**2)
radius_error = np.max(np.abs(radii - radius))
print(f"Radius error: ±{radius_error:.6f} m")

# Check if Z is constant
z_variation = np.max(traj[:-20,2]) - np.min(traj[:-20,2])
print(f"Z variation: {z_variation:.6f} m (should be 0)")

# Check if velocity is reasonable
max_speed = np.max(np.sqrt(traj[:-20,6]**2 + traj[:-20,7]**2))
print(f"Maximum linear velocity: {max_speed:.6f} m/s")
print(f"Theoretical linear velocity: {radius * angular_velocity:.6f} m/s")
