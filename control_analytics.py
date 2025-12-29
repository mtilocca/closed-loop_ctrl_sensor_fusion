#!/usr/bin/env python3
"""
Analyze PID controller performance from CSV log.

Usage:
    python3 analyze_pid_log.py pid_log.csv
"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

def load_pid_log(csv_path):
    """Load PID CSV log."""
    try:
        df = pd.read_csv(csv_path)
        print(f"✓ Loaded {len(df)} samples from {csv_path}")
        return df
    except Exception as e:
        print(f"✗ Error loading CSV: {e}")
        sys.exit(1)

def calculate_metrics(df):
    """Calculate PID performance metrics."""
    metrics = {}
    
    # Velocity tracking
    metrics['rmse_velocity'] = np.sqrt(np.mean(df['error_mps']**2))
    metrics['max_error'] = np.max(np.abs(df['error_mps']))
    metrics['settling_time'] = calculate_settling_time(df)
    
    # Control effort
    metrics['mean_torque'] = np.mean(df['torque_nm'])
    metrics['max_torque'] = np.max(df['torque_nm'])
    metrics['min_torque'] = np.min(df['torque_nm'])
    metrics['torque_std'] = np.std(df['torque_nm'])
    
    # Steady-state (last 20% of data)
    steady_idx = int(len(df) * 0.8)
    metrics['steady_error'] = np.mean(np.abs(df['error_mps'].iloc[steady_idx:]))
    metrics['steady_torque'] = np.mean(df['torque_nm'].iloc[steady_idx:])
    
    return metrics

def calculate_settling_time(df, tolerance=0.05):
    """Calculate 5% settling time."""
    target = df['target_velocity_mps'].iloc[0]
    threshold = target * tolerance
    
    # Find first time velocity enters and stays within tolerance
    within_tolerance = np.abs(df['error_mps']) < threshold
    
    for i in range(len(df) - 10):
        if all(within_tolerance.iloc[i:i+10]):
            return df['time_s'].iloc[i]
    
    return df['time_s'].iloc[-1]

def plot_pid_analysis(df, output_path='pid_analysis.png'):
    """Create comprehensive PID analysis plots."""
    fig = plt.figure(figsize=(16, 12))
    gs = GridSpec(4, 2, figure=fig, hspace=0.3, wspace=0.3)
    
    time = df['time_s'].values
    
    # ============================================================================
    # Plot 1: Velocity Tracking
    # ============================================================================
    ax1 = fig.add_subplot(gs[0, :])
    
    ax1.plot(time, df['target_velocity_mps'], 'k--', linewidth=2, 
             label='Target', alpha=0.7)
    ax1.plot(time, df['actual_velocity_mps'], 'b-', linewidth=1.5, 
             label='Actual', alpha=0.8)
    ax1.fill_between(time, 
                      df['target_velocity_mps'] - 0.1, 
                      df['target_velocity_mps'] + 0.1,
                      color='green', alpha=0.2, label='±0.1 m/s tolerance')
    
    ax1.set_xlabel('Time (s)', fontsize=11)
    ax1.set_ylabel('Velocity (m/s)', fontsize=11)
    ax1.set_title('PID Velocity Tracking Performance', fontsize=13, fontweight='bold')
    ax1.legend(loc='upper right', fontsize=10)
    ax1.grid(True, alpha=0.3)
    
    # ============================================================================
    # Plot 2: Tracking Error
    # ============================================================================
    ax2 = fig.add_subplot(gs[1, 0])
    
    ax2.plot(time, df['error_mps'], 'r-', linewidth=1.5)
    ax2.axhline(0, color='k', linestyle='-', linewidth=0.5)
    ax2.fill_between(time, -0.1, 0.1, color='green', alpha=0.2)
    
    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.set_ylabel('Error (m/s)', fontsize=11)
    ax2.set_title('Velocity Tracking Error', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    # ============================================================================
    # Plot 3: Torque Command
    # ============================================================================
    ax3 = fig.add_subplot(gs[1, 1])
    
    ax3.plot(time, df['torque_nm'], 'g-', linewidth=1.5)
    ax3.axhline(2000, color='r', linestyle='--', linewidth=1, label='Max limit')
    ax3.axhline(-500, color='r', linestyle='--', linewidth=1, label='Min limit')
    
    ax3.set_xlabel('Time (s)', fontsize=11)
    ax3.set_ylabel('Torque (Nm)', fontsize=11)
    ax3.set_title('PID Control Output (Torque Command)', fontsize=12, fontweight='bold')
    ax3.legend(loc='upper right', fontsize=9)
    ax3.grid(True, alpha=0.3)
    
    # ============================================================================
    # Plot 4: PID Component Breakdown
    # ============================================================================
    ax4 = fig.add_subplot(gs[2, :])
    
    ax4.plot(time, df['p_term_nm'], 'b-', linewidth=1.5, label='P term', alpha=0.7)
    ax4.plot(time, df['i_term_nm'], 'g-', linewidth=1.5, label='I term', alpha=0.7)
    ax4.plot(time, df['d_term_nm'], 'r-', linewidth=1.5, label='D term', alpha=0.7)
    ax4.plot(time, df['torque_nm'], 'k--', linewidth=2, label='Total', alpha=0.5)
    
    ax4.set_xlabel('Time (s)', fontsize=11)
    ax4.set_ylabel('Torque Contribution (Nm)', fontsize=11)
    ax4.set_title('PID Component Breakdown', fontsize=12, fontweight='bold')
    ax4.legend(loc='upper right', fontsize=10)
    ax4.grid(True, alpha=0.3)
    
    # ============================================================================
    # Plot 5: Integral Term & Anti-Windup
    # ============================================================================
    ax5 = fig.add_subplot(gs[3, 0])
    
    ax5.plot(time, df['integral'], 'purple', linewidth=1.5)
    ax5.axhline(1000, color='r', linestyle='--', linewidth=1, label='Integral limit')
    ax5.axhline(-1000, color='r', linestyle='--', linewidth=1)
    
    ax5.set_xlabel('Time (s)', fontsize=11)
    ax5.set_ylabel('Integral Value', fontsize=11)
    ax5.set_title('Integral Term (Anti-Windup Monitoring)', fontsize=12, fontweight='bold')
    ax5.legend(loc='upper right', fontsize=9)
    ax5.grid(True, alpha=0.3)
    
    # ============================================================================
    # Plot 6: Error vs Steering (Phase Plot)
    # ============================================================================
    ax6 = fig.add_subplot(gs[3, 1])
    
    scatter = ax6.scatter(df['steering_deg'], df['error_mps'], 
                          c=time, cmap='viridis', s=10, alpha=0.6)
    ax6.axhline(0, color='k', linestyle='-', linewidth=0.5)
    ax6.axvline(0, color='k', linestyle='-', linewidth=0.5)
    
    ax6.set_xlabel('Steering Angle (deg)', fontsize=11)
    ax6.set_ylabel('Velocity Error (m/s)', fontsize=11)
    ax6.set_title('Error vs Steering (Colored by Time)', fontsize=12, fontweight='bold')
    ax6.grid(True, alpha=0.3)
    
    cbar = plt.colorbar(scatter, ax=ax6)
    cbar.set_label('Time (s)', fontsize=9)
    
    # ============================================================================
    # Overall title
    # ============================================================================
    fig.suptitle('PID Controller Performance Analysis', 
                 fontsize=15, fontweight='bold', y=0.995)
    #show
    #plt.show()
    
    # Save
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f'✓ Saved analysis plot to {output_path}')
    plt.close()

def print_metrics(metrics):
    """Print performance metrics."""
    print("\n" + "="*60)
    print("PID PERFORMANCE METRICS")
    print("="*60)
    
    print("\n📊 Velocity Tracking:")
    print(f"  RMSE Error:           {metrics['rmse_velocity']:.4f} m/s")
    print(f"  Max Error:            {metrics['max_error']:.4f} m/s")
    print(f"  Settling Time (5%):   {metrics['settling_time']:.2f} s")
    print(f"  Steady-State Error:   {metrics['steady_error']:.4f} m/s")
    
    print("\n🎛️  Control Effort:")
    print(f"  Mean Torque:          {metrics['mean_torque']:.2f} Nm")
    print(f"  Steady-State Torque:  {metrics['steady_torque']:.2f} Nm")
    print(f"  Torque Range:         [{metrics['min_torque']:.1f}, {metrics['max_torque']:.1f}] Nm")
    print(f"  Torque Std Dev:       {metrics['torque_std']:.2f} Nm")
    
    print("\n✅ Performance Grade:")
    if metrics['rmse_velocity'] < 0.05:
        print("  Excellent (RMSE < 0.05 m/s)")
    elif metrics['rmse_velocity'] < 0.1:
        print("  Good (RMSE < 0.1 m/s)")
    elif metrics['rmse_velocity'] < 0.2:
        print("  Acceptable (RMSE < 0.2 m/s)")
    else:
        print("  Needs Tuning (RMSE > 0.2 m/s)")
    
    print("="*60 + "\n")

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_pid_log.py <pid_log.csv>")
        sys.exit(1)
    
    csv_path = sys.argv[1]
    df = load_pid_log(csv_path)
    
    # Calculate metrics
    metrics = calculate_metrics(df)
    
    # Print metrics
    print_metrics(metrics)
    
    # Create plots
    plot_pid_analysis(df)
    
    print("✅ Analysis complete!")

if __name__ == '__main__':
    main()