#!/usr/bin/env python3
"""
Visualize constant velocity turn scenario with PID control performance.

This script analyzes the closed-loop PID controller behavior during turns,
showing velocity tracking, torque commands, and PID component contributions.

Usage:
    python3 plot_pid_velocity_control.py sim_out.csv
"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

def load_data(csv_path):
    """Load simulation CSV with sensor data."""
    try:
        df = pd.read_csv(csv_path)
        print(f"✓ Loaded {len(df)} samples from {csv_path}")
        return df
    except Exception as e:
        print(f"✗ Error loading CSV: {e}")
        sys.exit(1)

def plot_pid_performance(df, output_path='pid_velocity_control.png'):
    """
    Create comprehensive PID performance visualization.
    
    Plots:
    1. Velocity tracking (target vs actual)
    2. Torque command (PID output)
    3. Steering angle (disturbance input)
    4. Trajectory (X-Y path with velocity colormap)
    """
    fig = plt.figure(figsize=(16, 10))
    gs = GridSpec(3, 2, figure=fig, hspace=0.3, wspace=0.3)
    
    time = df['t_s'].values
    
    # ============================================================================
    # Plot 1: Velocity Tracking
    # ============================================================================
    ax1 = fig.add_subplot(gs[0, :])
    
    # Extract velocity (from GNSS or wheel speeds)
    if 'gnss_vn_mps' in df.columns and 'gnss_ve_mps' in df.columns:
        vn = df['gnss_vn_mps'].values
        ve = df['gnss_ve_mps'].values
        v_measured = np.sqrt(vn**2 + ve**2)
    else:
        v_measured = df['v_mps'].values  # Fallback to truth
    
    # Target velocity (constant 8.0 m/s for this scenario)
    v_target = 8.0 * np.ones_like(time)
    
    ax1.plot(time, v_target, 'k--', linewidth=2, label='Target (8.0 m/s)', alpha=0.7)
    ax1.plot(time, v_measured, 'b-', linewidth=1.5, label='Measured (GNSS)', alpha=0.8)
    ax1.fill_between(time, v_target - 0.3, v_target + 0.3, 
                      color='green', alpha=0.2, label='±0.3 m/s tolerance')
    
    ax1.set_xlabel('Time (s)', fontsize=11)
    ax1.set_ylabel('Velocity (m/s)', fontsize=11)
    ax1.set_title('PID Velocity Tracking Performance', fontsize=13, fontweight='bold')
    ax1.legend(loc='upper right', fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim([6, 10])
    
    # Calculate tracking error statistics
    error = v_measured - v_target
    rmse = np.sqrt(np.mean(error**2))
    max_error = np.max(np.abs(error))
    
    stats_text = f'RMSE: {rmse:.3f} m/s\nMax Error: {max_error:.3f} m/s'
    ax1.text(0.02, 0.98, stats_text, transform=ax1.transAxes,
             fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # ============================================================================
    # Plot 2: Torque Command (PID Output)
    # ============================================================================
    ax2 = fig.add_subplot(gs[1, 0])
    
    torque = df['motor_nm'].values
    
    ax2.plot(time, torque, 'r-', linewidth=1.5, label='Torque Command')
    ax2.axhline(2000, color='orange', linestyle='--', linewidth=1, label='Max Limit (2000 Nm)')
    ax2.axhline(-500, color='purple', linestyle='--', linewidth=1, label='Min Limit (-500 Nm)')
    ax2.fill_between(time, -500, 2000, color='gray', alpha=0.1)
    
    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.set_ylabel('Torque (Nm)', fontsize=11)
    ax2.set_title('PID Control Output (Motor Torque)', fontsize=12, fontweight='bold')
    ax2.legend(loc='upper right', fontsize=9)
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim([-1000, 2500])
    
    # Calculate saturation percentage
    saturated = np.sum((torque >= 2000) | (torque <= -500))
    sat_pct = 100 * saturated / len(torque)
    
    sat_text = f'Saturation: {sat_pct:.2f}% of time'
    ax2.text(0.02, 0.98, sat_text, transform=ax2.transAxes,
             fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='salmon', alpha=0.5))
    
    # ============================================================================
    # Plot 3: Steering Angle (Disturbance)
    # ============================================================================
    ax3 = fig.add_subplot(gs[1, 1])
    
    steer = df['steer_deg'].values
    
    ax3.plot(time, steer, 'g-', linewidth=1.5, label='Steering Command')
    ax3.axhline(0, color='k', linestyle='-', linewidth=0.5, alpha=0.5)
    ax3.fill_between(time, steer, 0, where=(steer > 0), color='green', alpha=0.3, label='Right turn')
    ax3.fill_between(time, steer, 0, where=(steer < 0), color='blue', alpha=0.3, label='Left turn')
    
    ax3.set_xlabel('Time (s)', fontsize=11)
    ax3.set_ylabel('Steering Angle (deg)', fontsize=11)
    ax3.set_title('Steering Input (Disturbance to PID)', fontsize=12, fontweight='bold')
    ax3.legend(loc='upper right', fontsize=9)
    ax3.grid(True, alpha=0.3)
    ax3.set_ylim([-20, 20])
    
    # ============================================================================
    # Plot 4: Trajectory with Velocity Colormap
    # ============================================================================
    ax4 = fig.add_subplot(gs[2, :])
    
    x = df['x_m'].values
    y = df['y_m'].values
    
    # Create scatter plot with velocity as color
    scatter = ax4.scatter(x, y, c=v_measured, cmap='RdYlGn', 
                          s=10, alpha=0.6, vmin=7.0, vmax=9.0)
    
    # Add start/end markers
    ax4.plot(x[0], y[0], 'go', markersize=12, label='Start', markeredgecolor='k', markeredgewidth=1)
    ax4.plot(x[-1], y[-1], 'rs', markersize=12, label='End', markeredgecolor='k', markeredgewidth=1)
    
    # Add turn labels
    for i, t in enumerate([10, 20, 30, 40, 50]):
        idx = np.argmin(np.abs(time - t))
        if idx < len(x):
            ax4.annotate(f't={t}s', (x[idx], y[idx]), 
                        fontsize=8, ha='center',
                        bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.5))
    
    ax4.set_xlabel('X Position (m)', fontsize=11)
    ax4.set_ylabel('Y Position (m)', fontsize=11)
    ax4.set_title('Vehicle Trajectory (Colored by Velocity)', fontsize=12, fontweight='bold')
    ax4.legend(loc='upper left', fontsize=9)
    ax4.grid(True, alpha=0.3)
    ax4.axis('equal')
    
    # Add colorbar
    cbar = plt.colorbar(scatter, ax=ax4, orientation='vertical', pad=0.02)
    cbar.set_label('Velocity (m/s)', fontsize=10)
    
    # ============================================================================
    # Overall title
    # ============================================================================
    fig.suptitle('Constant Velocity Turn Scenario - PID Controller Performance', 
                 fontsize=15, fontweight='bold', y=0.98)
    
    # Save
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f'✓ Saved plot to {output_path}')
    plt.close()
    
    return {
        'velocity_rmse': rmse,
        'velocity_max_error': max_error,
        'torque_saturation_pct': sat_pct
    }

def plot_pid_components(df, output_path='pid_components.png'):
    """
    Plot PID component breakdown (requires extended CSV logging).
    
    This is a placeholder for when you add PID diagnostics to CSV output.
    """
    # TODO: Add PID component logging to sim_app.cpp
    # Required columns: pid_p_term, pid_i_term, pid_d_term, pid_error
    
    if not all(col in df.columns for col in ['pid_p_term', 'pid_i_term', 'pid_d_term']):
        print("⚠ PID component data not found in CSV (requires extended logging)")
        return
    
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    time = df['t_s'].values
    
    # P term
    axes[0].plot(time, df['pid_p_term'], 'b-', linewidth=1.5)
    axes[0].set_ylabel('P Term (Nm)', fontsize=11)
    axes[0].set_title('Proportional Component', fontsize=12, fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    
    # I term
    axes[1].plot(time, df['pid_i_term'], 'g-', linewidth=1.5)
    axes[1].axhline(1000, color='r', linestyle='--', label='Integral Limit')
    axes[1].axhline(-1000, color='r', linestyle='--')
    axes[1].set_ylabel('I Term (Nm)', fontsize=11)
    axes[1].set_title('Integral Component (with Anti-windup)', fontsize=12, fontweight='bold')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    # D term
    axes[2].plot(time, df['pid_d_term'], 'r-', linewidth=1.5)
    axes[2].set_xlabel('Time (s)', fontsize=11)
    axes[2].set_ylabel('D Term (Nm)', fontsize=11)
    axes[2].set_title('Derivative Component', fontsize=12, fontweight='bold')
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f'✓ Saved PID components plot to {output_path}')
    plt.close()

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_pid_velocity_control.py <sim_out.csv>")
        sys.exit(1)
    
    csv_path = sys.argv[1]
    df = load_data(csv_path)
    
    print("\n" + "="*60)
    print("PID Velocity Control Analysis")
    print("="*60)
    
    # Main performance plot
    metrics = plot_pid_performance(df)
    
    print("\n📊 Performance Metrics:")
    print(f"  Velocity RMSE:        {metrics['velocity_rmse']:.3f} m/s")
    print(f"  Max Velocity Error:   {metrics['velocity_max_error']:.3f} m/s")
    print(f"  Torque Saturation:    {metrics['torque_saturation_pct']:.2f}%")
    
    # PID component breakdown (if available)
    plot_pid_components(df)
    
    print("\n✅ Analysis complete!")
    print("="*60 + "\n")

if __name__ == '__main__':
    main()