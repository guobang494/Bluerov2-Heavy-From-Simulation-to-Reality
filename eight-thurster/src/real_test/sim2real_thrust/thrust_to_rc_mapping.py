#!/usr/bin/env python3
"""
Script to map thrust data from ROS bag to RC channel values and xyz/yaw control.
Author: Assistant
"""

import rosbag
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
import os
import pandas as pd

# BlueROV2 Thruster Allocation Matrix (TAM) - same as before
TAM = np.array([
    # X-direction force (surge)
    [0.7071067811847431, 0.7071067811847431, -0.7071067811919607, -0.7071067811919607, 0.0, 0.0, 0.0, 0.0],
    # Y-direction force (sway)
    [0.7071067811883519, -0.7071067811883519, 0.7071067811811348, -0.7071067811811348, 0.0, 0.0, 0.0, 0.0],
    # Z-direction force (heave)
    [0.0, 0.0, 0.0, 0.0, 1.0000000000000002, 1.0000000000000002, 1.0000000000000002, 1.0000000000000002],
    # Roll moment (around X-axis)
    [0.05126524163615552, -0.051265241636155506, 0.05126524163563227, -0.05126524163563226, -0.21805000000000005, -0.21805000000000005, 0.21805000000000005, 0.21805000000000005],
    # Pitch moment (around Y-axis)
    [-0.05126524163589388, -0.051265241635893875, 0.051265241636417144, 0.05126524163641714, -0.1199999999999745, 0.12000000000002554, -0.11999999999997452, 0.12000000000002554],
    # Yaw moment (around Z-axis)
    [0.166523646969496, -0.166523646969496, -0.17500892834341342, 0.17500892834341344, 0.0, 0.0, 0.0, 0.0]
])

# RC Channel mapping parameters (typical values for BlueROV2)
class RCMapping:
    def __init__(self):
        # PWM range (typical RC values)
        self.pwm_min = 1100  # microseconds
        self.pwm_max = 1900  # microseconds  
        self.pwm_center = 1500  # neutral position
        
        # Thrust range (estimated from BlueROV2 specs)
        # T200 thrusters: approximately ±50N at full power
        self.thrust_max = 50.0  # Newtons (positive)
        self.thrust_min = -50.0  # Newtons (negative)
        
        # Control input limits (from your previous data)
        self.control_limits = {
            'x_max': 104.65,    # N
            'y_max': 104.65,    # N  
            'z_max': 148.0,     # N
            'yaw_max': 25.27    # N·m
        }
        
    def thrust_to_pwm(self, thrust):
        """Convert thrust (N) to PWM value (microseconds)."""
        # Normalize thrust to [-1, 1] range
        if thrust >= 0:
            normalized = thrust / self.thrust_max
        else:
            normalized = thrust / (-self.thrust_min)
            
        # Clamp to [-1, 1]
        normalized = np.clip(normalized, -1, 1)
        
        # Convert to PWM range
        pwm = self.pwm_center + normalized * (self.pwm_max - self.pwm_center)
        return np.clip(pwm, self.pwm_min, self.pwm_max)
    
    def pwm_to_thrust(self, pwm):
        """Convert PWM value to thrust (N)."""
        # Normalize PWM to [-1, 1] range
        normalized = (pwm - self.pwm_center) / (self.pwm_max - self.pwm_center)
        normalized = np.clip(normalized, -1, 1)
        
        # Convert to thrust
        if normalized >= 0:
            thrust = normalized * self.thrust_max
        else:
            thrust = normalized * (-self.thrust_min)
            
        return thrust
    
    def body_force_to_rc_channels(self, x_force, y_force, z_force, yaw_moment):
        """Convert body forces to RC channel values (conceptual mapping)."""
        # Normalize forces to [-1, 1] based on control limits
        x_norm = np.clip(x_force / self.control_limits['x_max'], -1, 1)
        y_norm = np.clip(y_force / self.control_limits['y_max'], -1, 1)
        z_norm = np.clip(z_force / self.control_limits['z_max'], -1, 1)
        yaw_norm = np.clip(yaw_moment / self.control_limits['yaw_max'], -1, 1)
        
        # Convert to RC channel PWM values
        # Typical RC mapping: 
        # Channel 1: Roll (not used for ROV, set to center)
        # Channel 2: Pitch (not used for ROV, set to center) 
        # Channel 3: Throttle → Z (heave)
        # Channel 4: Rudder → Yaw
        # Channel 5: Forward/Backward → X (surge)
        # Channel 6: Left/Right → Y (sway)
        
        rc_channels = {
            'roll': self.pwm_center,  # Not used
            'pitch': self.pwm_center,  # Not used
            'throttle': self.pwm_center + z_norm * (self.pwm_max - self.pwm_center),  # Z
            'rudder': self.pwm_center + yaw_norm * (self.pwm_max - self.pwm_center),  # Yaw
            'forward': self.pwm_center + x_norm * (self.pwm_max - self.pwm_center),   # X
            'lateral': self.pwm_center + y_norm * (self.pwm_max - self.pwm_center)    # Y
        }
        
        return rc_channels

def extract_thrust_data(bag_file):
    """Extract thrust data from ROS bag file."""
    print(f"Processing ROS bag: {bag_file}")
    
    thrust_data = defaultdict(list)
    
    with rosbag.Bag(bag_file, 'r') as bag:
        print(f"Bag duration: {bag.get_end_time() - bag.get_start_time():.2f} seconds")
        
        for topic, msg, t in bag.read_messages():
            if '/bluerov2/thrusters/' in topic and '/thrust' in topic:
                thruster_id = int(topic.split('/')[3])
                if thruster_id < 8:
                    thrust_data[thruster_id].append({
                        'time': t.to_sec(),
                        'thrust': msg.data
                    })
    
    return thrust_data

def synchronize_data(thrust_data):
    """Synchronize thrust data across all thrusters."""
    all_times = []
    for thruster_id in range(8):
        if thruster_id in thrust_data:
            times = [d['time'] for d in thrust_data[thruster_id]]
            all_times.extend(times)
    
    if not all_times:
        raise ValueError("No thrust data found!")
    
    min_time = min(all_times)
    max_time = max(all_times)
    time_grid = np.linspace(min_time, max_time, 1000)
    
    synchronized_data = {}
    for thruster_id in range(8):
        if thruster_id in thrust_data:
            times = np.array([d['time'] for d in thrust_data[thruster_id]])
            thrusts = np.array([d['thrust'] for d in thrust_data[thruster_id]])
            synchronized_thrusts = np.interp(time_grid, times, thrusts)
            synchronized_data[thruster_id] = synchronized_thrusts
        else:
            synchronized_data[thruster_id] = np.zeros(len(time_grid))
    
    return time_grid, synchronized_data

def calculate_body_forces(time_grid, thrust_data):
    """Calculate body forces using TAM."""
    n_points = len(time_grid)
    
    thrust_matrix = np.zeros((8, n_points))
    for thruster_id in range(8):
        thrust_matrix[thruster_id, :] = thrust_data[thruster_id]
    
    body_forces = TAM @ thrust_matrix
    
    return {
        'time': time_grid,
        'x_force': body_forces[0, :],
        'y_force': body_forces[1, :],
        'z_force': body_forces[2, :],
        'roll_moment': body_forces[3, :],
        'pitch_moment': body_forces[4, :],
        'yaw_moment': body_forces[5, :]
    }

def thrust_to_rc_mapping(body_forces, thrust_data, time_grid, output_dir=None):
    """Map thrust data to RC channel values and create visualizations."""
    if output_dir is None:
        output_dir = os.path.dirname(os.path.abspath(__file__))
    
    rc_mapper = RCMapping()
    
    # Calculate RC channel values for each time step
    n_points = len(time_grid)
    rc_data = {
        'time': time_grid,
        'roll': np.full(n_points, rc_mapper.pwm_center),
        'pitch': np.full(n_points, rc_mapper.pwm_center),
        'throttle': np.zeros(n_points),
        'rudder': np.zeros(n_points),
        'forward': np.zeros(n_points),
        'lateral': np.zeros(n_points)
    }
    
    # Individual thruster PWM values
    thruster_pwm = {}
    for thruster_id in range(8):
        thruster_pwm[thruster_id] = np.array([
            rc_mapper.thrust_to_pwm(thrust) for thrust in thrust_data[thruster_id]
        ])
    
    # Convert body forces to RC channels
    for i in range(n_points):
        rc_channels = rc_mapper.body_force_to_rc_channels(
            body_forces['x_force'][i],
            body_forces['y_force'][i], 
            body_forces['z_force'][i],
            body_forces['yaw_moment'][i]
        )
        
        rc_data['throttle'][i] = rc_channels['throttle']
        rc_data['rudder'][i] = rc_channels['rudder']
        rc_data['forward'][i] = rc_channels['forward']
        rc_data['lateral'][i] = rc_channels['lateral']
    
    # Create plots
    create_rc_plots(body_forces, rc_data, thruster_pwm, output_dir)
    
    # Save data to CSV
    save_rc_data(body_forces, rc_data, thruster_pwm, output_dir)
    
    return rc_data, thruster_pwm

def create_rc_plots(body_forces, rc_data, thruster_pwm, output_dir):
    """Create RC mapping visualization plots."""
    time_rel = body_forces['time'] - body_forces['time'][0]
    
    # Create main figure with subplots
    fig, axes = plt.subplots(3, 2, figsize=(16, 12))
    fig.suptitle('Thrust to RC Channel Mapping - BlueROV2', fontsize=16, fontweight='bold')
    
    # Plot body forces
    axes[0, 0].plot(time_rel, body_forces['x_force'], 'r-', linewidth=2, label='X-Force')
    axes[0, 0].set_title('X-Direction Force (Surge)', fontweight='bold')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Force (N)')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()
    
    axes[0, 1].plot(time_rel, body_forces['y_force'], 'g-', linewidth=2, label='Y-Force')
    axes[0, 1].set_title('Y-Direction Force (Sway)', fontweight='bold')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Force (N)')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend()
    
    axes[1, 0].plot(time_rel, body_forces['z_force'], 'b-', linewidth=2, label='Z-Force')
    axes[1, 0].set_title('Z-Direction Force (Heave)', fontweight='bold')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Force (N)')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()
    
    axes[1, 1].plot(time_rel, body_forces['yaw_moment'], 'm-', linewidth=2, label='Yaw Moment')
    axes[1, 1].set_title('Yaw Moment', fontweight='bold')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Moment (N·m)')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()
    
    # Plot RC channels
    time_rc = rc_data['time'] - rc_data['time'][0]
    axes[2, 0].plot(time_rc, rc_data['forward'], 'r-', linewidth=2, label='Forward/Back (X)')
    axes[2, 0].plot(time_rc, rc_data['lateral'], 'g-', linewidth=2, label='Left/Right (Y)')
    axes[2, 0].set_title('RC Horizontal Channels', fontweight='bold')
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].set_ylabel('PWM (μs)')
    axes[2, 0].grid(True, alpha=0.3)
    axes[2, 0].legend()
    axes[2, 0].set_ylim(1000, 2000)
    
    axes[2, 1].plot(time_rc, rc_data['throttle'], 'b-', linewidth=2, label='Throttle (Z)')
    axes[2, 1].plot(time_rc, rc_data['rudder'], 'm-', linewidth=2, label='Rudder (Yaw)')
    axes[2, 1].set_title('RC Vertical/Yaw Channels', fontweight='bold')
    axes[2, 1].set_xlabel('Time (s)')
    axes[2, 1].set_ylabel('PWM (μs)')
    axes[2, 1].grid(True, alpha=0.3)
    axes[2, 1].legend()
    axes[2, 1].set_ylim(1000, 2000)
    
    plt.tight_layout()
    
    # Save plot
    plot_file = os.path.join(output_dir, 'thrust_to_rc_mapping.png')
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    print(f"RC mapping plot saved to: {plot_file}")
    plt.show()
    
    # Create individual thruster PWM plot
    fig2, ax = plt.subplots(1, 1, figsize=(14, 8))
    fig2.suptitle('Individual Thruster PWM Values', fontsize=16, fontweight='bold')
    
    colors = plt.cm.tab10(np.linspace(0, 1, 8))
    for thruster_id in range(8):
        ax.plot(time_rel, thruster_pwm[thruster_id], 
               color=colors[thruster_id], linewidth=2, 
               label=f'Thruster {thruster_id}', alpha=0.8)
    
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('PWM (μs)', fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.set_ylim(1000, 2000)
    
    plt.tight_layout()
    
    # Save thruster PWM plot
    pwm_plot_file = os.path.join(output_dir, 'thruster_pwm_values.png')
    plt.savefig(pwm_plot_file, dpi=300, bbox_inches='tight')
    print(f"Thruster PWM plot saved to: {pwm_plot_file}")
    plt.show()

def save_rc_data(body_forces, rc_data, thruster_pwm, output_dir):
    """Save RC mapping data to CSV files."""
    time_rel = body_forces['time'] - body_forces['time'][0]
    
    # Create main data DataFrame
    data_dict = {
        'Time(s)': time_rel,
        'X_Force(N)': body_forces['x_force'],
        'Y_Force(N)': body_forces['y_force'],
        'Z_Force(N)': body_forces['z_force'],
        'Yaw_Moment(Nm)': body_forces['yaw_moment'],
        'RC_Forward_PWM': rc_data['forward'],
        'RC_Lateral_PWM': rc_data['lateral'],
        'RC_Throttle_PWM': rc_data['throttle'],
        'RC_Rudder_PWM': rc_data['rudder']
    }
    
    # Add thruster PWM data
    for thruster_id in range(8):
        data_dict[f'Thruster_{thruster_id}_PWM'] = thruster_pwm[thruster_id]
    
    df = pd.DataFrame(data_dict)
    
    # Save to CSV
    csv_file = os.path.join(output_dir, 'thrust_to_rc_mapping_data.csv')
    df.to_csv(csv_file, index=False, float_format='%.4f')
    print(f"RC mapping data saved to: {csv_file}")
    
    # Print summary statistics
    print("\n" + "="*60)
    print("RC CHANNEL MAPPING SUMMARY")
    print("="*60)
    print(f"Forward/Back (X): {rc_data['forward'].min():.0f} - {rc_data['forward'].max():.0f} μs")
    print(f"Left/Right (Y):   {rc_data['lateral'].min():.0f} - {rc_data['lateral'].max():.0f} μs")
    print(f"Up/Down (Z):      {rc_data['throttle'].min():.0f} - {rc_data['throttle'].max():.0f} μs")
    print(f"Yaw:              {rc_data['rudder'].min():.0f} - {rc_data['rudder'].max():.0f} μs")
    print("="*60)

def main():
    """Main function."""
    bag_file = '/home/zeb/test-8/eight-thurster/src/bluerov2/bluerov2_dobmpc/rosbag/bluerov2_dobmpc_20250731_083732.bag'
    
    if not os.path.exists(bag_file):
        print(f"Error: ROS bag file not found: {bag_file}")
        return
    
    try:
        print("Step 1: Extracting thrust data from ROS bag...")
        thrust_data = extract_thrust_data(bag_file)
        
        print("Step 2: Synchronizing data across thrusters...")
        time_grid, synchronized_thrust_data = synchronize_data(thrust_data)
        
        print("Step 3: Calculating body forces using TAM...")
        body_forces = calculate_body_forces(time_grid, synchronized_thrust_data)
        
        print("Step 4: Mapping to RC channels...")
        output_dir = os.path.dirname(os.path.abspath(__file__))
        rc_data, thruster_pwm = thrust_to_rc_mapping(
            body_forces, synchronized_thrust_data, time_grid, output_dir)
        
        print("\nThrust to RC mapping completed successfully!")
        
    except Exception as e:
        print(f"Error during analysis: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main() 