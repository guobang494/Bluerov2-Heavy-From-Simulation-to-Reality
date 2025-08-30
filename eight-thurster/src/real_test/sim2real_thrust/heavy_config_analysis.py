#!/usr/bin/env python3
"""
Script to analyze individual thruster PWM vs thrust relationship 
considering BlueROV2 Heavy Configuration layout.
Author: Assistant
"""

import rosbag
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch
import pandas as pd
from collections import defaultdict
import os
import math

# BlueROV2 Heavy Configuration Thruster Layout
# Based on the actuators.xacro file
THRUSTER_POSITIONS = {
    # Horizontal thrusters (0-3) - angled at ±45° for vectored thrust
    0: {'pos': [0.1355, -0.1, -0.0725], 'angle': 45.0, 'type': 'horizontal'},    # Front-right
    1: {'pos': [0.1355, 0.1, -0.0725], 'angle': -45.0, 'type': 'horizontal'},    # Front-left  
    2: {'pos': [-0.1475, -0.1, -0.0725], 'angle': 135.0, 'type': 'horizontal'},  # Rear-right
    3: {'pos': [-0.1475, 0.1, -0.0725], 'angle': -135.0, 'type': 'horizontal'},  # Rear-left
    
    # Vertical thrusters (4-7) - pointing downward for heave control
    4: {'pos': [0.12, -0.21805, -0.005], 'angle': -90.0, 'type': 'vertical'},    # Front-right vertical
    5: {'pos': [-0.12, -0.21805, -0.005], 'angle': -90.0, 'type': 'vertical'},   # Rear-right vertical
    6: {'pos': [0.12, 0.21805, -0.005], 'angle': -90.0, 'type': 'vertical'},     # Front-left vertical
    7: {'pos': [-0.12, 0.21805, -0.005], 'angle': -90.0, 'type': 'vertical'}     # Rear-left vertical
}

# T200 Thruster Performance Characteristics (from Blue Robotics specs)
class T200ThrusterModel:
    def __init__(self):
        # PWM to thrust mapping (approximate from T200 datasheet)
        # These values are based on T200 performance curves
        self.pwm_thrust_map = {
            1100: -40.8,  # Full reverse
            1200: -28.0,
            1300: -16.8,
            1400: -7.2,
            1500: 0.0,    # Neutral
            1600: 7.2,
            1700: 16.8,
            1800: 28.0,
            1900: 40.8    # Full forward
        }
        
        # Convert to arrays for interpolation
        self.pwm_values = np.array(list(self.pwm_thrust_map.keys()))
        self.thrust_values = np.array(list(self.pwm_thrust_map.values()))
        
        # Maximum specifications
        self.max_thrust_forward = 40.8  # N
        self.max_thrust_reverse = -40.8  # N
        self.max_current = 25.0  # A
        self.max_power = 300.0   # W
        
    def pwm_to_thrust(self, pwm):
        """Convert PWM to thrust using T200 characteristics."""
        return np.interp(pwm, self.pwm_values, self.thrust_values)
    
    def thrust_to_pwm(self, thrust):
        """Convert thrust to PWM using T200 characteristics."""
        return np.interp(thrust, self.thrust_values, self.pwm_values)

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

def create_heavy_config_layout_plot(output_dir):
    """Create a visualization of BlueROV2 Heavy Configuration layout."""
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))
    
    # Draw vehicle outline (simplified)
    vehicle_length = 0.45
    vehicle_width = 0.3
    vehicle_rect = Rectangle((-vehicle_length/2, -vehicle_width/2), 
                           vehicle_length, vehicle_width, 
                           fill=False, edgecolor='black', linewidth=2)
    ax.add_patch(vehicle_rect)
    
    # Add coordinate system
    ax.arrow(0, 0, 0.1, 0, head_width=0.02, head_length=0.02, fc='red', ec='red')
    ax.text(0.12, 0.01, 'X (Forward)', fontsize=10, color='red')
    ax.arrow(0, 0, 0, 0.1, head_width=0.02, head_length=0.02, fc='green', ec='green')
    ax.text(0.01, 0.12, 'Y (Left)', fontsize=10, color='green')
    
    # Plot thrusters
    colors = {'horizontal': 'blue', 'vertical': 'orange'}
    
    for thruster_id, config in THRUSTER_POSITIONS.items():
        x, y, z = config['pos']
        thruster_type = config['type']
        angle = config['angle']
        
        # Draw thruster position
        circle = Circle((x, y), 0.02, color=colors[thruster_type], alpha=0.7)
        ax.add_patch(circle)
        
        # Add thruster ID label
        ax.text(x+0.03, y+0.03, f'T{thruster_id}', fontsize=10, fontweight='bold')
        
        # Draw thrust direction arrow
        if thruster_type == 'horizontal':
            # Convert angle to radians and draw arrow
            angle_rad = math.radians(angle)
            dx = 0.05 * math.cos(angle_rad)
            dy = 0.05 * math.sin(angle_rad)
            arrow = FancyArrowPatch((x, y), (x + dx, y + dy),
                                  arrowstyle='->', mutation_scale=15,
                                  color=colors[thruster_type])
            ax.add_patch(arrow)
        else:
            # Vertical thruster - show with different symbol
            ax.scatter(x, y, s=100, marker='s', color=colors[thruster_type], alpha=0.7)
    
    # Add legend
    horizontal_patch = plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=10, label='Horizontal Thrusters (0-3)')
    vertical_patch = plt.Line2D([0], [0], marker='s', color='w', markerfacecolor='orange', markersize=10, label='Vertical Thrusters (4-7)')
    ax.legend(handles=[horizontal_patch, vertical_patch], loc='upper right')
    
    ax.set_xlim(-0.3, 0.3)
    ax.set_ylim(-0.3, 0.3)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title('BlueROV2 Heavy Configuration - Thruster Layout', fontsize=14, fontweight='bold')
    
    # Save plot
    layout_file = os.path.join(output_dir, 'bluerov2_heavy_config_layout.png')
    plt.savefig(layout_file, dpi=300, bbox_inches='tight')
    print(f"Layout plot saved to: {layout_file}")
    plt.show()

def analyze_individual_thrusters(thrust_data, output_dir):
    """Analyze individual thruster performance with PWM mapping."""
    t200_model = T200ThrusterModel()
    
    # Create comprehensive analysis plots
    fig, axes = plt.subplots(4, 4, figsize=(20, 16))
    fig.suptitle('Individual Thruster Analysis - PWM vs Thrust (Heavy Config)', 
                 fontsize=16, fontweight='bold')
    
    # Statistics storage
    thruster_stats = []
    
    for thruster_id in range(8):
        row = thruster_id // 4
        col = thruster_id % 4
        ax = axes[row, col]
        
        if thruster_id in thrust_data:
            # Extract data
            times = np.array([d['time'] for d in thrust_data[thruster_id]])
            thrusts = np.array([d['thrust'] for d in thrust_data[thruster_id]])
            
            # Convert to relative time
            times_rel = times - times[0]
            
            # Convert thrust to PWM using T200 model
            pwm_values = np.array([t200_model.thrust_to_pwm(thrust) for thrust in thrusts])
            
            # Plot thrust vs time
            ax2 = ax.twinx()
            
            # Thrust plot (left y-axis)
            line1 = ax.plot(times_rel, thrusts, 'b-', linewidth=1.5, label='Thrust (N)', alpha=0.8)
            ax.set_ylabel('Thrust (N)', color='blue', fontsize=10)
            ax.tick_params(axis='y', labelcolor='blue')
            
            # PWM plot (right y-axis)  
            line2 = ax2.plot(times_rel, pwm_values, 'r-', linewidth=1.5, label='PWM (μs)', alpha=0.8)
            ax2.set_ylabel('PWM (μs)', color='red', fontsize=10)
            ax2.tick_params(axis='y', labelcolor='red')
            ax2.set_ylim(1100, 1900)
            
            # Statistics
            thrust_max = np.max(thrusts)
            thrust_min = np.min(thrusts)
            thrust_mean = np.mean(thrusts)
            thrust_std = np.std(thrusts)
            pwm_max = np.max(pwm_values)
            pwm_min = np.min(pwm_values)
            
            # Get thruster configuration
            config = THRUSTER_POSITIONS[thruster_id]
            thruster_type = config['type']
            angle = config['angle']
            
            # Store statistics
            thruster_stats.append({
                'Thruster_ID': thruster_id,
                'Type': thruster_type,
                'Angle_deg': angle,
                'Position_X_m': config['pos'][0],
                'Position_Y_m': config['pos'][1],
                'Position_Z_m': config['pos'][2],
                'Thrust_Max_N': thrust_max,
                'Thrust_Min_N': thrust_min,
                'Thrust_Mean_N': thrust_mean,
                'Thrust_Std_N': thrust_std,
                'PWM_Max_us': pwm_max,
                'PWM_Min_us': pwm_min,
                'Data_Points': len(thrusts)
            })
            
            # Add statistics text
            stats_text = f'Max: {thrust_max:.1f}N ({pwm_max:.0f}μs)\nMin: {thrust_min:.1f}N ({pwm_min:.0f}μs)\nMean: {thrust_mean:.1f}N\nStd: {thrust_std:.1f}N'
            ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, fontsize=8,
                   verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
            
        else:
            ax.text(0.5, 0.5, 'No Data', transform=ax.transAxes, fontsize=14,
                   horizontalalignment='center', verticalalignment='center')
            
            # Add placeholder stats
            thruster_stats.append({
                'Thruster_ID': thruster_id,
                'Type': THRUSTER_POSITIONS[thruster_id]['type'],
                'Angle_deg': THRUSTER_POSITIONS[thruster_id]['angle'],
                'Position_X_m': THRUSTER_POSITIONS[thruster_id]['pos'][0],
                'Position_Y_m': THRUSTER_POSITIONS[thruster_id]['pos'][1],
                'Position_Z_m': THRUSTER_POSITIONS[thruster_id]['pos'][2],
                'Thrust_Max_N': 0, 'Thrust_Min_N': 0, 'Thrust_Mean_N': 0, 'Thrust_Std_N': 0,
                'PWM_Max_us': 1500, 'PWM_Min_us': 1500, 'Data_Points': 0
            })
        
        # Set title with thruster info
        config = THRUSTER_POSITIONS[thruster_id]
        ax.set_title(f'Thruster {thruster_id} ({config["type"].title()})\nAngle: {config["angle"]}°', 
                    fontweight='bold', fontsize=10)
        ax.set_xlabel('Time (s)', fontsize=10)
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save individual analysis plot
    individual_file = os.path.join(output_dir, 'individual_thruster_pwm_thrust_analysis.png')
    plt.savefig(individual_file, dpi=300, bbox_inches='tight')
    print(f"Individual thruster analysis saved to: {individual_file}")
    plt.show()
    
    return thruster_stats

def create_pwm_thrust_relationship_plot(output_dir):
    """Create T200 PWM vs Thrust relationship plot."""
    t200_model = T200ThrusterModel()
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # Plot 1: PWM vs Thrust curve
    pwm_range = np.linspace(1100, 1900, 100)
    thrust_range = [t200_model.pwm_to_thrust(pwm) for pwm in pwm_range]
    
    ax1.plot(pwm_range, thrust_range, 'b-', linewidth=3, label='T200 Characteristic')
    ax1.scatter(t200_model.pwm_values, t200_model.thrust_values, 
               color='red', s=50, zorder=5, label='Datasheet Points')
    
    # Add neutral and max points
    ax1.axhline(y=0, color='gray', linestyle='--', alpha=0.7, label='Neutral')
    ax1.axvline(x=1500, color='gray', linestyle='--', alpha=0.7)
    
    ax1.set_xlabel('PWM Signal (μs)', fontsize=12)
    ax1.set_ylabel('Thrust (N)', fontsize=12)
    ax1.set_title('T200 Thruster: PWM vs Thrust Relationship', fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Add annotations
    ax1.annotate('Full Forward\n(40.8N @ 1900μs)', xy=(1900, 40.8), xytext=(1800, 35),
                arrowprops=dict(arrowstyle='->', color='red'), fontsize=10)
    ax1.annotate('Neutral\n(0N @ 1500μs)', xy=(1500, 0), xytext=(1600, -10),
                arrowprops=dict(arrowstyle='->', color='gray'), fontsize=10)
    ax1.annotate('Full Reverse\n(-40.8N @ 1100μs)', xy=(1100, -40.8), xytext=(1200, -35),
                arrowprops=dict(arrowstyle='->', color='red'), fontsize=10)
    
    # Plot 2: Thrust efficiency zones
    efficiency_zones = {
        'Dead Zone': (1480, 1520, 'lightgray'),
        'Low Efficiency': (1400, 1480, 'yellow'),
        'Medium Efficiency': (1300, 1400, 'orange'),  
        'High Efficiency': (1100, 1300, 'lightgreen'),
    }
    
    for zone_name, (pwm_start, pwm_end, color) in efficiency_zones.items():
        thrust_start = t200_model.pwm_to_thrust(pwm_start)
        thrust_end = t200_model.pwm_to_thrust(pwm_end)
        ax2.fill_between([pwm_start, pwm_end], [thrust_start, thrust_end], 
                        [thrust_start, thrust_end], alpha=0.3, color=color, label=zone_name)
    
    # Mirror for forward direction
    for zone_name, (pwm_start, pwm_end, color) in efficiency_zones.items():
        if zone_name != 'Dead Zone':
            pwm_start_fwd = 3000 - pwm_end  # Mirror around 1500
            pwm_end_fwd = 3000 - pwm_start
            thrust_start_fwd = t200_model.pwm_to_thrust(pwm_start_fwd)
            thrust_end_fwd = t200_model.pwm_to_thrust(pwm_end_fwd)
            ax2.fill_between([pwm_start_fwd, pwm_end_fwd], [thrust_start_fwd, thrust_end_fwd],
                            [thrust_start_fwd, thrust_end_fwd], alpha=0.3, color=color)
    
    ax2.plot(pwm_range, thrust_range, 'b-', linewidth=3)
    ax2.set_xlabel('PWM Signal (μs)', fontsize=12)
    ax2.set_ylabel('Thrust (N)', fontsize=12)
    ax2.set_title('T200 Thruster: Efficiency Zones', fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    plt.tight_layout()
    
    # Save relationship plot
    relationship_file = os.path.join(output_dir, 't200_pwm_thrust_relationship.png')
    plt.savefig(relationship_file, dpi=300, bbox_inches='tight')
    print(f"PWM-Thrust relationship plot saved to: {relationship_file}")
    plt.show()

def save_heavy_config_analysis(thruster_stats, output_dir):
    """Save detailed analysis to CSV."""
    df_stats = pd.DataFrame(thruster_stats)
    
    # Save to CSV
    stats_file = os.path.join(output_dir, 'heavy_config_thruster_analysis.csv')
    df_stats.to_csv(stats_file, index=False, float_format='%.4f')
    print(f"Heavy config analysis saved to: {stats_file}")
    
    # Print summary by thruster type
    print("\n" + "="*80)
    print("BLUEROV2 HEAVY CONFIGURATION ANALYSIS SUMMARY")
    print("="*80)
    
    # Horizontal thrusters summary
    horizontal_thrusters = df_stats[df_stats['Type'] == 'horizontal']
    print("\nHORIZONTAL THRUSTERS (0-3) - Vectored Thrust Control:")
    print(horizontal_thrusters[['Thruster_ID', 'Angle_deg', 'Thrust_Max_N', 'Thrust_Min_N', 'Thrust_Mean_N', 'PWM_Max_us', 'PWM_Min_us']].to_string(index=False))
    
    # Vertical thrusters summary
    vertical_thrusters = df_stats[df_stats['Type'] == 'vertical']
    print("\nVERTICAL THRUSTERS (4-7) - Heave Control:")
    print(vertical_thrusters[['Thruster_ID', 'Position_X_m', 'Position_Y_m', 'Thrust_Max_N', 'Thrust_Min_N', 'Thrust_Mean_N', 'PWM_Max_us', 'PWM_Min_us']].to_string(index=False))
    
    print("\nKEY INSIGHTS:")
    print(f"- Total horizontal thrust range: {horizontal_thrusters['Thrust_Max_N'].max():.1f}N to {horizontal_thrusters['Thrust_Min_N'].min():.1f}N")
    print(f"- Total vertical thrust range: {vertical_thrusters['Thrust_Max_N'].max():.1f}N to {vertical_thrusters['Thrust_Min_N'].min():.1f}N")
    print(f"- PWM range utilized: {df_stats['PWM_Min_us'].min():.0f} - {df_stats['PWM_Max_us'].max():.0f} μs")
    print("="*80)

def main():
    """Main function."""
    bag_file = '/home/zeb/test-8/eight-thurster/src/bluerov2/bluerov2_dobmpc/rosbag/bluerov2_dobmpc_20250731_083732.bag'
    
    if not os.path.exists(bag_file):
        print(f"Error: ROS bag file not found: {bag_file}")
        return
    
    output_dir = os.path.dirname(os.path.abspath(__file__))
    
    try:
        print("Step 1: Creating BlueROV2 Heavy Configuration layout visualization...")
        create_heavy_config_layout_plot(output_dir)
        
        print("Step 2: Creating T200 PWM-Thrust relationship plots...")
        create_pwm_thrust_relationship_plot(output_dir)
        
        print("Step 3: Extracting thrust data from ROS bag...")
        thrust_data = extract_thrust_data(bag_file)
        
        print("Step 4: Analyzing individual thruster performance...")
        thruster_stats = analyze_individual_thrusters(thrust_data, output_dir)
        
        print("Step 5: Saving detailed analysis...")
        save_heavy_config_analysis(thruster_stats, output_dir)
        
        print("\nHeavy Configuration analysis completed successfully!")
        
    except Exception as e:
        print(f"Error during analysis: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main() 