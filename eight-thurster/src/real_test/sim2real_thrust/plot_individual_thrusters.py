#!/usr/bin/env python3
"""
Script to plot individual thruster forces (0-7) from BlueROV2 ROS bag.
Author: Assistant
"""

import rosbag
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
import os

def extract_thrust_data(bag_file):
    """Extract thrust data from ROS bag file."""
    print(f"Processing ROS bag: {bag_file}")
    
    # Dictionary to store thrust data for each thruster
    thrust_data = defaultdict(list)
    
    # Open the bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        # Get bag info
        print(f"Bag duration: {bag.get_end_time() - bag.get_start_time():.2f} seconds")
        
        # Extract thrust data for all 8 thrusters
        for topic, msg, t in bag.read_messages():
            if '/bluerov2/thrusters/' in topic and '/thrust' in topic:
                # Extract thruster ID from topic name
                thruster_id = int(topic.split('/')[3])
                if thruster_id < 8:  # Only process thrusters 0-7
                    thrust_data[thruster_id].append({
                        'time': t.to_sec(),
                        'thrust': msg.data
                    })
    
    print(f"Extracted data for {len(thrust_data)} thrusters")
    for thruster_id in sorted(thrust_data.keys()):
        print(f"  Thruster {thruster_id}: {len(thrust_data[thruster_id])} data points")
    
    return thrust_data

def plot_individual_thrusters(thrust_data, output_dir=None):
    """Create plots for individual thruster forces."""
    if output_dir is None:
        output_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Create figure with 4x2 subplots for 8 thrusters
    fig, axes = plt.subplots(4, 2, figsize=(16, 20))
    fig.suptitle('Individual Thruster Forces (0-7) - BlueROV2', fontsize=16, fontweight='bold')
    
    # Colors for each thruster
    colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray']
    
    # Find global time range for consistent x-axis
    all_times = []
    for thruster_id in range(8):
        if thruster_id in thrust_data:
            times = [d['time'] for d in thrust_data[thruster_id]]
            all_times.extend(times)
    
    if not all_times:
        print("No thrust data found!")
        return
    
    min_time = min(all_times)
    max_time = max(all_times)
    
    # Plot each thruster
    for thruster_id in range(8):
        row = thruster_id // 2
        col = thruster_id % 2
        ax = axes[row, col]
        
        if thruster_id in thrust_data:
            # Extract time and thrust data
            times = np.array([d['time'] for d in thrust_data[thruster_id]])
            thrusts = np.array([d['thrust'] for d in thrust_data[thruster_id]])
            
            # Adjust time to start from 0
            times_rel = times - min_time
            
            # Plot thrust data
            ax.plot(times_rel, thrusts, color=colors[thruster_id], linewidth=1.5, alpha=0.8)
            ax.fill_between(times_rel, 0, thrusts, color=colors[thruster_id], alpha=0.3)
            
            # Statistics
            max_thrust = np.max(thrusts)
            min_thrust = np.min(thrusts)
            mean_thrust = np.mean(thrusts)
            rms_thrust = np.sqrt(np.mean(thrusts**2))
            
            # Add statistics text
            stats_text = f'Max: {max_thrust:.2f}N\nMin: {min_thrust:.2f}N\nMean: {mean_thrust:.2f}N\nRMS: {rms_thrust:.2f}N'
            ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, fontsize=9,
                   verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        else:
            # No data for this thruster
            ax.text(0.5, 0.5, 'No Data', transform=ax.transAxes, fontsize=14,
                   horizontalalignment='center', verticalalignment='center')
        
        # Set title and labels
        ax.set_title(f'Thruster {thruster_id}', fontweight='bold', fontsize=12)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Thrust (N)')
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0, max_time - min_time)
    
    plt.tight_layout()
    
    # Save plot
    plot_file = os.path.join(output_dir, 'individual_thrusters_0_7.png')
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    print(f"Individual thruster plot saved to: {plot_file}")
    
    # Show plot
    plt.show()

def plot_all_thrusters_overlay(thrust_data, output_dir=None):
    """Create overlay plot with all thrusters on same axes."""
    if output_dir is None:
        output_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Create figure
    fig, ax = plt.subplots(1, 1, figsize=(14, 8))
    fig.suptitle('All Thruster Forces (0-7) - Overlay View', fontsize=16, fontweight='bold')
    
    # Colors for each thruster
    colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray']
    
    # Find global time range
    all_times = []
    for thruster_id in range(8):
        if thruster_id in thrust_data:
            times = [d['time'] for d in thrust_data[thruster_id]]
            all_times.extend(times)
    
    if not all_times:
        print("No thrust data found!")
        return
    
    min_time = min(all_times)
    
    # Plot each thruster
    for thruster_id in range(8):
        if thruster_id in thrust_data:
            # Extract time and thrust data
            times = np.array([d['time'] for d in thrust_data[thruster_id]])
            thrusts = np.array([d['thrust'] for d in thrust_data[thruster_id]])
            
            # Adjust time to start from 0
            times_rel = times - min_time
            
            # Plot thrust data
            ax.plot(times_rel, thrusts, color=colors[thruster_id], 
                   linewidth=2, label=f'Thruster {thruster_id}', alpha=0.8)
    
    # Set labels and legend
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Thrust (N)', fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    plt.tight_layout()
    
    # Save plot
    plot_file = os.path.join(output_dir, 'all_thrusters_overlay.png')
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    print(f"Overlay plot saved to: {plot_file}")
    
    # Show plot
    plt.show()

def create_thruster_statistics_table(thrust_data, output_dir=None):
    """Create statistics table for all thrusters."""
    if output_dir is None:
        output_dir = os.path.dirname(os.path.abspath(__file__))
    
    stats_data = []
    
    for thruster_id in range(8):
        if thruster_id in thrust_data:
            thrusts = np.array([d['thrust'] for d in thrust_data[thruster_id]])
            
            stats_data.append({
                'Thruster': thruster_id,
                'Data Points': len(thrusts),
                'Max (N)': np.max(thrusts),
                'Min (N)': np.min(thrusts),
                'Mean (N)': np.mean(thrusts),
                'RMS (N)': np.sqrt(np.mean(thrusts**2)),
                'Std Dev (N)': np.std(thrusts)
            })
        else:
            stats_data.append({
                'Thruster': thruster_id,
                'Data Points': 0,
                'Max (N)': 0,
                'Min (N)': 0,
                'Mean (N)': 0,
                'RMS (N)': 0,
                'Std Dev (N)': 0
            })
    
    # Create DataFrame and save
    import pandas as pd
    df_stats = pd.DataFrame(stats_data)
    
    # Save to CSV
    stats_file = os.path.join(output_dir, 'individual_thruster_statistics.csv')
    df_stats.to_csv(stats_file, index=False, float_format='%.4f')
    print(f"Statistics saved to: {stats_file}")
    
    # Print table
    print("\n" + "="*80)
    print("INDIVIDUAL THRUSTER STATISTICS")
    print("="*80)
    print(df_stats.to_string(index=False, float_format='%.4f'))
    print("="*80)

def main():
    """Main function."""
    # ROS bag file path
    bag_file = '/home/zeb/test-8/eight-thurster/src/bluerov2/bluerov2_dobmpc/rosbag/bluerov2_dobmpc_20250731_083732.bag'
    
    # Check if bag file exists
    if not os.path.exists(bag_file):
        print(f"Error: ROS bag file not found: {bag_file}")
        return
    
    try:
        # Extract thrust data from bag
        print("Step 1: Extracting thrust data from ROS bag...")
        thrust_data = extract_thrust_data(bag_file)
        
        # Create output directory
        output_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Create individual thruster plots
        print("Step 2: Creating individual thruster plots...")
        plot_individual_thrusters(thrust_data, output_dir)
        
        # Create overlay plot
        print("Step 3: Creating overlay plot...")
        plot_all_thrusters_overlay(thrust_data, output_dir)
        
        # Create statistics table
        print("Step 4: Creating statistics table...")
        create_thruster_statistics_table(thrust_data, output_dir)
        
        print("\nThruster analysis completed successfully!")
        
    except Exception as e:
        print(f"Error during analysis: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main() 