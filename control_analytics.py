#!/usr/bin/env python3
"""
Unified controller performance analysis for PID, MPC, and Auto-MPC.

Usage:
    python3 analyze_controller.py <csv_file> <controller_type>
    
    controller_type: 1=PID, 2=MPC, 3=Auto-MPC
    
Examples:
    python3 analyze_controller.py gentle_slalom_pid.csv 1
    python3 analyze_controller.py gentle_slalom_mpc.csv 2
    python3 analyze_controller.py gentle_slalom_auto_mpc.csv 3
"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

CONTROLLER_NAMES = {
    1: "PID",
    2: "MPC",
    3: "Auto-MPC"
}

def load_log(csv_path):
    """Load controller CSV log."""
    try:
        df = pd.read_csv(csv_path)
        print(f"✓ Loaded {len(df)} samples from {csv_path}")
        return df
    except Exception as e:
        print(f"✗ Error loading CSV: {e}")
        sys.exit(1)

def calculate_metrics(df, controller_type):
    """Calculate performance metrics for any controller."""
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
    
    # Brake usage (MPC and Auto-MPC)
    if controller_type in [2, 3]:
        metrics['brake_usage_pct'] = (df['brake_pct'] > 0).sum() / len(df) * 100
        metrics['mean_brake'] = df[df['brake_pct'] > 0]['brake_pct'].mean() if (df['brake_pct'] > 0).any() else 0
    else:
        metrics['brake_usage_pct'] = 0
        metrics['mean_brake'] = 0
    
    # Model confidence (MPC and Auto-MPC)
    if controller_type in [2, 3]:
        metrics['final_confidence'] = df['model_conf'].iloc[-1]
        metrics['est_mass'] = df['est_mass_kg'].iloc[-1]
        metrics['est_drag'] = df['est_drag'].iloc[-1]
    
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

def plot_analysis(df, controller_type, output_path):
    """Create comprehensive analysis plots for any controller."""
    controller_name = CONTROLLER_NAMES[controller_type]
    
    fig = plt.figure(figsize=(18, 14))
    gs = GridSpec(5, 2, figure=fig, hspace=0.3, wspace=0.3)
    
    time = df['time_s'].values
    
    # ============================================================================
    # Plot 1: Velocity Tracking
    # ============================================================================
    ax1 = fig.add_subplot(gs[0, :])
    
    ax1.plot(time, df['target_velocity_mps'], 'k--', linewidth=2.5, 
             label='Target', alpha=0.8)
    ax1.plot(time, df['actual_velocity_mps'], 'b-', linewidth=1.8, 
             label='Actual', alpha=0.9)
    ax1.fill_between(time, 
                      df['target_velocity_mps'] - 0.1, 
                      df['target_velocity_mps'] + 0.1,
                      color='green', alpha=0.15, label='±0.1 m/s tolerance')
    
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Velocity (m/s)', fontsize=12)
    ax1.set_title(f'{controller_name} Velocity Tracking Performance', 
                  fontsize=14, fontweight='bold')
    ax1.legend(loc='best', fontsize=11)
    ax1.grid(True, alpha=0.3)
    
    # ============================================================================
    # Plot 2: Tracking Error
    # ============================================================================
    ax2 = fig.add_subplot(gs[1, 0])
    
    ax2.plot(time, df['error_mps'], 'r-', linewidth=1.5, alpha=0.8)
    ax2.axhline(0, color='k', linestyle='-', linewidth=0.8)
    ax2.fill_between(time, -0.1, 0.1, color='green', alpha=0.15)
    
    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.set_ylabel('Error (m/s)', fontsize=11)
    ax2.set_title('Velocity Tracking Error', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    # ============================================================================
    # Plot 3: Torque & Brake Commands
    # ============================================================================
    ax3 = fig.add_subplot(gs[1, 1])
    
    ax3_twin = ax3.twinx()
    
    ln1 = ax3.plot(time, df['torque_nm'], 'g-', linewidth=1.5, label='Torque', alpha=0.8)
    ln2 = ax3_twin.plot(time, df['brake_pct'], 'r-', linewidth=1.5, label='Brake', alpha=0.8)
    
    ax3.set_xlabel('Time (s)', fontsize=11)
    ax3.set_ylabel('Torque (Nm)', fontsize=11, color='g')
    ax3_twin.set_ylabel('Brake (%)', fontsize=11, color='r')
    ax3.set_title('Control Commands', fontsize=12, fontweight='bold')
    
    lns = ln1 + ln2
    labs = [l.get_label() for l in lns]
    ax3.legend(lns, labs, loc='upper right', fontsize=10)
    ax3.grid(True, alpha=0.3)
    
    # ============================================================================
    # Plot 4: Control Components (PID terms or Model params)
    # ============================================================================
    ax4 = fig.add_subplot(gs[2, :])
    
    if controller_type == 1:  # PID
        ax4.plot(time, df['p_term_nm'], 'b-', linewidth=1.5, label='P term', alpha=0.7)
        ax4.plot(time, df['i_term_nm'], 'g-', linewidth=1.5, label='I term', alpha=0.7)
        ax4.plot(time, df['d_term_nm'], 'r-', linewidth=1.5, label='D term', alpha=0.7)
        ax4.plot(time, df['torque_nm'], 'k--', linewidth=2, label='Total', alpha=0.5)
        ax4.set_ylabel('Torque Contribution (Nm)', fontsize=11)
        ax4.set_title('PID Component Breakdown', fontsize=12, fontweight='bold')
    else:  # MPC or Auto-MPC
        ax4_twin = ax4.twinx()
        ln1 = ax4.plot(time, df['est_mass_kg'], 'b-', linewidth=2, label='Estimated Mass', alpha=0.8)
        ln2 = ax4_twin.plot(time, df['model_conf'], 'r-', linewidth=2, label='Model Confidence', alpha=0.8)
        ax4.set_ylabel('Estimated Mass (kg)', fontsize=11, color='b')
        ax4_twin.set_ylabel('Model Confidence (0-1)', fontsize=11, color='r')
        ax4.set_title(f'{controller_name} Model Learning', fontsize=12, fontweight='bold')
        
        lns = ln1 + ln2
        labs = [l.get_label() for l in lns]
        ax4.legend(lns, labs, loc='best', fontsize=10)
    
    ax4.set_xlabel('Time (s)', fontsize=11)
    ax4.legend(loc='best', fontsize=10)
    ax4.grid(True, alpha=0.3)
    
    # ============================================================================
    # Plot 5: Model Parameters (MPC/Auto-MPC) or Integral (PID)
    # ============================================================================
    ax5 = fig.add_subplot(gs[3, 0])
    
    if controller_type == 1:  # PID - show integral
        ax5.plot(time, df['integral'], 'purple', linewidth=1.5)
        ax5.set_ylabel('Integral Value', fontsize=11)
        ax5.set_title('Integral Term (Anti-Windup)', fontsize=12, fontweight='bold')
    else:  # MPC/Auto-MPC - show drag
        ax5.plot(time, df['est_drag'], 'orange', linewidth=2, alpha=0.8)
        ax5.set_ylabel('Estimated Drag Coeff', fontsize=11)
        ax5.set_title('Drag Coefficient Learning', fontsize=12, fontweight='bold')
    
    ax5.set_xlabel('Time (s)', fontsize=11)
    ax5.grid(True, alpha=0.3)
    
    # ============================================================================
    # Plot 6: Error vs Steering
    # ============================================================================
    ax6 = fig.add_subplot(gs[3, 1])
    
    scatter = ax6.scatter(df['steering_deg'], df['error_mps'], 
                          c=time, cmap='viridis', s=15, alpha=0.6)
    ax6.axhline(0, color='k', linestyle='-', linewidth=0.8)
    ax6.axvline(0, color='k', linestyle='-', linewidth=0.8)
    
    ax6.set_xlabel('Steering Angle (deg)', fontsize=11)
    ax6.set_ylabel('Velocity Error (m/s)', fontsize=11)
    ax6.set_title('Error vs Steering (Colored by Time)', fontsize=12, fontweight='bold')
    ax6.grid(True, alpha=0.3)
    
    cbar = plt.colorbar(scatter, ax=ax6)
    cbar.set_label('Time (s)', fontsize=10)
    
    # ============================================================================
    # Plot 7: Torque vs Velocity
    # ============================================================================
    ax7 = fig.add_subplot(gs[4, 0])
    
    scatter = ax7.scatter(df['actual_velocity_mps'], df['torque_nm'],
                          c=time, cmap='plasma', s=12, alpha=0.5)
    ax7.axvline(df['target_velocity_mps'].iloc[0], color='r', linestyle='--', 
                linewidth=2, label='Target Velocity')
    
    ax7.set_xlabel('Velocity (m/s)', fontsize=11)
    ax7.set_ylabel('Torque (Nm)', fontsize=11)
    ax7.set_title('Control Effort vs Velocity', fontsize=12, fontweight='bold')
    ax7.legend(loc='best', fontsize=10)
    ax7.grid(True, alpha=0.3)
    
    cbar = plt.colorbar(scatter, ax=ax7)
    cbar.set_label('Time (s)', fontsize=10)
    
    # ============================================================================
    # Plot 8: Brake Usage (if applicable)
    # ============================================================================
    ax8 = fig.add_subplot(gs[4, 1])
    
    if controller_type in [2, 3]:
        # Show brake usage over time
        ax8.fill_between(time, 0, df['brake_pct'], 
                         where=(df['brake_pct'] > 0),
                         color='red', alpha=0.5, label='Braking')
        ax8.set_xlabel('Time (s)', fontsize=11)
        ax8.set_ylabel('Brake Command (%)', fontsize=11)
        ax8.set_title('Brake Usage Profile', fontsize=12, fontweight='bold')
        ax8.legend(loc='best', fontsize=10)
        ax8.grid(True, alpha=0.3)
    else:
        # PID - show error histogram
        ax8.hist(df['error_mps'], bins=50, color='skyblue', edgecolor='black', alpha=0.7)
        ax8.axvline(0, color='r', linestyle='--', linewidth=2)
        ax8.set_xlabel('Error (m/s)', fontsize=11)
        ax8.set_ylabel('Frequency', fontsize=11)
        ax8.set_title('Error Distribution', fontsize=12, fontweight='bold')
        ax8.grid(True, alpha=0.3, axis='y')
    
    # Overall title
    fig.suptitle(f'{controller_name} Controller Performance Analysis', 
                 fontsize=16, fontweight='bold', y=0.995)
    
    # Save
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f'✓ Saved analysis plot to {output_path}')
    plt.close()

def print_metrics(metrics, controller_type):
    """Print performance metrics."""
    controller_name = CONTROLLER_NAMES[controller_type]
    
    print("\n" + "="*70)
    print(f"{controller_name} CONTROLLER PERFORMANCE METRICS")
    print("="*70)
    
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
    
    if controller_type in [2, 3]:
        print("\n🛑 Brake Usage:")
        print(f"  Brake Usage:          {metrics['brake_usage_pct']:.1f}% of time")
        print(f"  Mean Brake (active):  {metrics['mean_brake']:.1f}%")
        
        print("\n🧠 Model Learning:")
        print(f"  Final Confidence:     {metrics['final_confidence']:.2f}")
        print(f"  Estimated Mass:       {metrics['est_mass']:.1f} kg")
        print(f"  Estimated Drag:       {metrics['est_drag']:.3f}")
    
    print("\n✅ Performance Grade:")
    if metrics['rmse_velocity'] < 0.05:
        print("  ★★★★★ Excellent (RMSE < 0.05 m/s)")
    elif metrics['rmse_velocity'] < 0.1:
        print("  ★★★★☆ Very Good (RMSE < 0.1 m/s)")
    elif metrics['rmse_velocity'] < 0.2:
        print("  ★★★☆☆ Good (RMSE < 0.2 m/s)")
    elif metrics['rmse_velocity'] < 0.5:
        print("  ★★☆☆☆ Acceptable (RMSE < 0.5 m/s)")
    else:
        print("  ★☆☆☆☆ Needs Improvement (RMSE > 0.5 m/s)")
    
    print("="*70 + "\n")

def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    
    csv_path = sys.argv[1]
    controller_type = int(sys.argv[2])
    
    if controller_type not in [1, 2, 3]:
        print("Error: controller_type must be 1 (PID), 2 (MPC), or 3 (Auto-MPC)")
        sys.exit(1)
    
    # Load data
    df = load_log(csv_path)
    
    # Calculate metrics
    metrics = calculate_metrics(df, controller_type)
    
    # Print metrics
    print_metrics(metrics, controller_type)
    
    # Create plots
    output_name = csv_path.replace('.csv', '_analysis.png')
    plot_analysis(df, controller_type, output_name)
    
    print("✅ Analysis complete!")

if __name__ == '__main__':
    main()