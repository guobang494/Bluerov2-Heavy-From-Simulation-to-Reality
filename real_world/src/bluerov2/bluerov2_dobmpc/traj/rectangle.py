#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#--------------------------------------
# Generate Rectangle reference trajectory for NMPC in Tank Environment
# Tank dimensions: 18.15m x 2.5m x 1m
# Rectangle parameters:
# - Start Point (bottom-left): (-1.326492, -3.953818, 0.2328541259765625)
# - Width: 1.0m, Height: 0.8m
# - Motion: XY plane rectangular motion, Z fixed
# - Speed: Slow and smooth
# - Direction: Counter-clockwise starting from bottom-left corner
#--------------------------------------

# Parameters
sample_time = 0.05   # Sample time (seconds)
duration = 60       # Total duration (seconds) - 160 seconds to complete one rectangle
rect_width = 0.3     # Rectangle width (meters)
rect_height = 0.8    # Rectangle height (meters)
linear_speed = 0.15  # Linear speed (m/s)

# Rectangle starting point coordinates (bottom-left corner)
start_x = -0.43  # X coordinate of starting point (bottom-left corner)
start_y = -2.13  # Y coordinate of starting point (bottom-left corner)  
start_z = 0.2328541259765625  # Z coordinate (depth)

# Calculate rectangle corner points (starting from specified start point, counter-clockwise)
corners = np.array([
    [start_x, start_y],                           # Bottom-left (start point)
    [start_x + rect_width, start_y],              # Bottom-right
    [start_x + rect_width, start_y + rect_height], # Top-right
    [start_x, start_y + rect_height],             # Top-left
])

# Calculate center from corners for reference
center_x = start_x + rect_width/2
center_y = start_y + rect_height/2
center_z = start_z

# Calculate segment lengths and durations
segments = []
total_perimeter = 2 * (rect_width + rect_height)
segment_durations = []

for i in range(4):
    start = corners[i]
    end = corners[(i + 1) % 4]
    length = np.linalg.norm(end - start)
    duration_seg = length / linear_speed
    
    segments.append({
        'start': start,
        'end': end, 
        'length': length,
        'duration': duration_seg,
        'direction': (end - start) / length
    })
    segment_durations.append(duration_seg)

print(f"Rectangle trajectory segments:")
for i, seg in enumerate(segments):
    print(f"Segment {i+1}: {seg['length']:.3f}m, {seg['duration']:.1f}s")

# Adjust total duration to match segments
total_segment_duration = sum(segment_durations)
print(f"Natural segment duration: {total_segment_duration:.1f}s")
print(f"Requested duration: {duration}s")

# Generate time sequence
t = np.arange(0, duration, sample_time)
t = np.append(t, duration)

# Calculate number of trajectory points
num_points = len(t)

# Create trajectory array (16 columns: x y z phi theta psi u v w p q r u1 u2 u3 u4)
traj = np.zeros((num_points, 16))

# Generate rectangular trajectory
for i, time in enumerate(t):
    # Determine current position along the rectangle
    # Scale time to fit within one complete rectangle cycle
    cycle_time = (time % total_segment_duration)
    
    # Find which segment we're in
    current_segment = 0
    elapsed_time = 0
    
    for seg_idx, seg_duration in enumerate(segment_durations):
        if cycle_time <= elapsed_time + seg_duration:
            current_segment = seg_idx
            break
        elapsed_time += seg_duration
    
    # Calculate position within current segment
    segment_time = cycle_time - elapsed_time
    segment_progress = min(segment_time / segment_durations[current_segment], 1.0)
    
    # Get current segment info
    seg = segments[current_segment]
    
    # Calculate position (linear interpolation along segment)
    current_pos = seg['start'] + segment_progress * (seg['end'] - seg['start'])
    
    traj[i, 0] = current_pos[0]  # x
    traj[i, 1] = current_pos[1]  # y
    traj[i, 2] = start_z         # z (fixed)
    
    # Attitude (keep horizontal)
    traj[i, 3] = 0.0             # phi (roll)
    traj[i, 4] = 0.0             # theta (pitch)
    traj[i, 5] = 0.0             # psi (yaw)
    
    # Linear velocity (direction of current segment)
    if segment_progress < 1.0:  # Moving along segment
        traj[i, 6] = seg['direction'][0] * linear_speed  # u (x-direction velocity)
        traj[i, 7] = seg['direction'][1] * linear_speed  # v (y-direction velocity)
    else:  # At corner, velocity = 0
        traj[i, 6] = 0.0
        traj[i, 7] = 0.0
    
    traj[i, 8] = 0.0             # w (z-direction velocity)
    
    # Angular velocity
    traj[i, 9] = 0.0             # p (roll rate)
    traj[i, 10] = 0.0            # q (pitch rate)
    traj[i, 11] = 0.0            # r (yaw rate)
    
    # Control inputs (set to 0, calculated by MPC)
    traj[i, 12] = 0.0            # u1
    traj[i, 13] = 0.0            # u2
    traj[i, 14] = 0.0            # u3
    traj[i, 15] = 0.0            # u4

# Add extra points for MPC prediction horizon (reference tank_dob.py)
last_point = traj[-1, :]  # Get the last point
extra_points = np.tile(last_point, (20, 1))  # Repeat the last point 20 times
traj = np.vstack([traj, extra_points])  # Add to trajectory

# Save to file
output_filename = 'rectangle_trajectory.txt'
np.savetxt(output_filename, traj, fmt='%f')

# 打印轨迹信息
print(f"\nRectangle trajectory generated: {output_filename}")
print(f"Number of trajectory points: {traj.shape[0]}")
print(f"Start point (bottom-left): ({start_x:.6f}, {start_y:.6f}, {start_z:.6f})")
print(f"Center (calculated): ({center_x:.6f}, {center_y:.6f}, {center_z:.6f})")
print(f"Rectangle size: {rect_width}m x {rect_height}m")
print(f"Total duration: {duration} seconds")
print(f"Sample time: {sample_time} seconds")
print(f"Linear speed: {linear_speed:.6f} m/s")
print(f"Rectangle perimeter: {total_perimeter:.3f} m")
print()
print(f"X range: [{np.min(traj[:,0]):.3f}, {np.max(traj[:,0]):.3f}] m")
print(f"Y range: [{np.min(traj[:,1]):.3f}, {np.max(traj[:,1]):.3f}] m")
print(f"Z depth: {traj[0,2]:.3f} m (fixed)")
print()
print(f"Trajectory start: ({traj[0,0]:.6f}, {traj[0,1]:.6f}, {traj[0,2]:.6f})")
print(f"Trajectory end: ({traj[-21,0]:.6f}, {traj[-21,1]:.6f}, {traj[-21,2]:.6f})")

# Visualize trajectory
plt.figure(figsize=(15, 10))

# Tank boundary definition
tank_x_min, tank_x_max = -9.075, 9.075  # 18.15m length
tank_y_min, tank_y_max = -1.25, 1.25    # 2.5m width  
tank_z_min, tank_z_max = -1.0, 0.0      # 1m depth

# 1. 3D plot - Overall trajectory
ax1 = plt.subplot(2, 3, 1, projection='3d')
ax1.plot(traj[:-20, 0], traj[:-20, 1], traj[:-20, 2], 'b-', linewidth=2, label='Rectangle Trajectory')
ax1.scatter(traj[0, 0], traj[0, 1], traj[0, 2], color='green', s=100, label='Start Point')
ax1.scatter(center_x, center_y, center_z, color='red', s=100, label=f'Center({center_x:.3f}, {center_y:.3f})')

# Mark corners
for i, corner in enumerate(corners):
    ax1.scatter(corner[0], corner[1], start_z, color='orange', s=50, alpha=0.7)

# Draw tank boundary
xx, yy = np.meshgrid([tank_x_min, tank_x_max], [tank_y_min, tank_y_max])
zz = np.full_like(xx, tank_z_min)
ax1.plot_surface(xx, yy, zz, alpha=0.3, color='gray')

ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)') 
ax1.set_zlabel('Z (m)')
ax1.set_title('3D Tank Rectangle Trajectory')
ax1.legend()
ax1.set_xlim(tank_x_min, tank_x_max)
ax1.set_ylim(tank_y_min, tank_y_max)
ax1.set_zlim(tank_z_min, tank_z_max)

# 2. XY plane view (top view)
ax2 = plt.subplot(2, 3, 2)
ax2.plot(traj[:-20, 0], traj[:-20, 1], 'b-', linewidth=2, label='Rectangle Trajectory')
ax2.scatter(traj[0, 0], traj[0, 1], color='green', s=100, label='Start Point')
ax2.scatter(center_x, center_y, color='red', s=100, label=f'Center({center_x:.3f}, {center_y:.3f})')
ax2.scatter(start_x, start_y, color='blue', s=100, marker='s', label=f'Bottom-left({start_x:.3f}, {start_y:.3f})')

# Draw theoretical rectangle
rect_x = [corners[i][0] for i in range(4)] + [corners[0][0]]
rect_y = [corners[i][1] for i in range(4)] + [corners[0][1]]
ax2.plot(rect_x, rect_y, 'r--', alpha=0.7, linewidth=2, label='Theoretical Rectangle')

# Mark corners
for i, corner in enumerate(corners):
    ax2.scatter(corner[0], corner[1], color='orange', s=50, alpha=0.7)
    ax2.annotate(f'C{i+1}', (corner[0], corner[1]), xytext=(5, 5), 
                textcoords='offset points', fontsize=8)

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
ax2.set_xlim(center_x-rect_width/2-0.5, center_x+rect_width/2+0.5)
ax2.set_ylim(center_y-rect_height/2-0.5, center_y+rect_height/2+0.5)

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
plt.savefig('rectangle_trajectory_visualization.png', dpi=300, bbox_inches='tight')
plt.show()

print(f"\nTrajectory visualization saved: rectangle_trajectory_visualization.png")

# Verify trajectory quality
print(f"\n=== Trajectory Quality Verification ===")
# Check if start and end points are close
start_end_distance = np.sqrt((traj[0,0] - traj[-21,0])**2 + (traj[0,1] - traj[-21,1])**2)
print(f"Start to end distance: {start_end_distance:.6f} m (should be close to 0)")

# Check rectangle dimensions
actual_width = np.max(traj[:-20,0]) - np.min(traj[:-20,0])
actual_height = np.max(traj[:-20,1]) - np.min(traj[:-20,1])
print(f"Actual rectangle size: {actual_width:.3f}m x {actual_height:.3f}m")
print(f"Target rectangle size: {rect_width:.3f}m x {rect_height:.3f}m")

# Check if Z is constant
z_variation = np.max(traj[:-20,2]) - np.min(traj[:-20,2])
print(f"Z variation: {z_variation:.6f} m (should be 0)")

# Check if velocity is reasonable
max_speed = np.max(np.sqrt(traj[:-20,6]**2 + traj[:-20,7]**2))
print(f"Maximum linear velocity: {max_speed:.6f} m/s")
print(f"Target linear velocity: {linear_speed:.6f} m/s") 