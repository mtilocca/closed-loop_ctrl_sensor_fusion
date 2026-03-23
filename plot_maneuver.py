#!/usr/bin/env python3
"""
Maneuver diagnostic overlay: controller CSV vs vehicle dynamics sim CSV.

Aligns both logs on a shared time axis and produces a 5-panel figure that makes
command/response mismatches, runaway conditions, and gear-change timing immediately
visible.

Usage:
    python3 plot_maneuver.py <controller_csv> <sim_csv> [output_png]

    controller_csv  Output of the Go controller (e.g. full_cruise_maneuver_pid.csv)
    sim_csv         Vehicle dynamics simulator output
    output_png      Optional output path (default: <controller_csv stem>_overlay.png)

Examples:
    python3 plot_maneuver.py full_cruise_maneuver_pid.csv sim_dynamics.csv
    python3 plot_maneuver.py full_cruise_maneuver_pid.csv sim_dynamics.csv analysis.png
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec


# ---------------------------------------------------------------------------
# Colour palette (Okabe-Ito, colour-blind safe)
# ---------------------------------------------------------------------------
C = {
    "target":  "#4D4D4D",   # dark grey
    "ctrl_v":  "#0072B2",   # blue   — controller reported velocity
    "sim_v":   "#D55E00",   # vermillion — sim velocity
    "torque":  "#009E73",   # green
    "sim_mot": "#56B4E9",   # sky blue — sim motor
    "brake":   "#CC79A7",   # pink/purple
    "gear":    "#E69F00",   # orange
    "steer":   "#F0E442",   # yellow (on dark bg) / plotted on own axis
    "p":       "#0072B2",
    "i":       "#009E73",
    "d":       "#D55E00",
    "sat":     "#FF000033", # translucent red for saturation band
    "guard":   "#FFFF0033", # translucent yellow for gear-guard windows
}

MAX_TORQUE = 250_000.0  # XCMG XDE360 PID max — adjust if needed


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def load_csv(path: str, label: str) -> pd.DataFrame:
    try:
        df = pd.read_csv(path)
        print(f"  [{label}] {len(df)} rows — columns: {list(df.columns)[:6]}...")
        return df
    except Exception as exc:
        print(f"  ERROR loading {path}: {exc}")
        sys.exit(1)


def interp_sim_to_ctrl(ctrl: pd.DataFrame, sim: pd.DataFrame) -> pd.DataFrame:
    """Interpolate sim columns onto the controller time grid."""
    t_ctrl = ctrl["time_s"].values
    t_sim = sim["t_s"].values

    out = pd.DataFrame({"time_s": t_ctrl})
    for col in sim.columns:
        if col == "t_s":
            continue
        try:
            out[col] = np.interp(t_ctrl, t_sim, sim[col].values, left=np.nan, right=np.nan)
        except Exception:
            pass  # skip non-numeric columns silently
    return out


def gear_change_windows(ctrl: pd.DataFrame):
    """
    Return list of (t_start, t_end) for gear-change guard windows.
    Detected as gaps in the controller CSV (no rows logged while guard brakes).
    """
    t = ctrl["time_s"].values
    dt = np.diff(t)
    median_dt = np.median(dt)
    gaps = np.where(dt > median_dt * 5)[0]
    windows = [(t[i], t[i + 1]) for i in gaps]
    return windows


def gear_transition_times(ctrl: pd.DataFrame):
    """Return list of (t, from_gear, to_gear) for each gear change event."""
    g = ctrl["gear_position"].values
    t = ctrl["time_s"].values
    changes = []
    for i in range(1, len(g)):
        if g[i] != g[i - 1]:
            changes.append((t[i], int(g[i - 1]), int(g[i])))
    return changes


# ---------------------------------------------------------------------------
# Main plot
# ---------------------------------------------------------------------------

def plot_overlay(ctrl: pd.DataFrame, sim_aligned: pd.DataFrame, output_path: str):
    t = ctrl["time_s"].values

    guard_windows = gear_change_windows(ctrl)
    gear_transitions = gear_transition_times(ctrl)

    fig = plt.figure(figsize=(24, 18))
    gs = GridSpec(5, 1, figure=fig, hspace=0.55)

    axes = [fig.add_subplot(gs[i]) for i in range(5)]

    def shade_guards(ax):
        """Yellow bands for gear-guard braking windows; dashed lines at gear changes."""
        for t0, t1 in guard_windows:
            ax.axvspan(t0, t1, color=C["guard"], zorder=0, label="_nolegend_")
        for tc, fg, tg in gear_transitions:
            ax.axvline(tc, color=C["gear"], linestyle=":", linewidth=1.2, alpha=0.7)

    # -----------------------------------------------------------------------
    # Panel 1 — Velocity
    # -----------------------------------------------------------------------
    ax = axes[0]
    ax.plot(t, ctrl["target_velocity_mps"], linestyle="--", color=C["target"],
            linewidth=2.0, label="Target (ctrl)", alpha=0.9)
    ax.plot(t, ctrl["actual_velocity_mps"], linestyle="-", color=C["ctrl_v"],
            linewidth=1.5, label="Actual (ctrl / CAN)", alpha=0.95)
    if "v_mps" in sim_aligned.columns:
        ax.plot(t, sim_aligned["v_mps"], linestyle="-", color=C["sim_v"],
                linewidth=1.5, label="Velocity (sim)", alpha=0.8)

    # Mark braking events on velocity panel
    braking = ctrl["brake_pct"] > 0
    if braking.any():
        ax.fill_between(t, ax.get_ylim()[0] if hasattr(ax, '_ymin') else 0,
                        ctrl["actual_velocity_mps"],
                        where=braking, color=C["brake"], alpha=0.12,
                        label="Braking active")

    shade_guards(ax)
    ax.set_ylabel("Speed (m/s)")
    ax.set_title("Velocity — Controller vs Simulator", fontweight="bold")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.25)

    # -----------------------------------------------------------------------
    # Panel 2 — Torque
    # -----------------------------------------------------------------------
    ax = axes[1]
    ax.plot(t, ctrl["torque_nm"], color=C["torque"], linewidth=1.4,
            label="Torque cmd (ctrl)", alpha=0.9)
    if "motor_nm" in sim_aligned.columns:
        ax.plot(t, sim_aligned["motor_nm"], color=C["sim_mot"], linewidth=1.4,
                linestyle="--", label="Motor Nm (sim)", alpha=0.85)

    # Saturation band
    ax.axhspan(MAX_TORQUE * 0.99, MAX_TORQUE * 1.01, color=C["sat"],
               label=f"Saturation ({MAX_TORQUE/1000:.0f} kNm)")

    shade_guards(ax)
    ax.set_ylabel("Torque (Nm)")
    ax.set_title("Torque Command vs Simulator Motor Output", fontweight="bold")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.25)

    # -----------------------------------------------------------------------
    # Panel 3 — Brake
    # -----------------------------------------------------------------------
    ax = axes[2]
    ax.fill_between(t, 0, ctrl["brake_pct"], color=C["brake"], alpha=0.55,
                    label="Brake cmd (ctrl)")
    if "brake_pct" in sim_aligned.columns:
        ax.plot(t, sim_aligned["brake_pct"], color=C["sim_v"], linewidth=1.4,
                linestyle="--", label="Brake (sim)", alpha=0.85)

    shade_guards(ax)
    ax.set_ylabel("Brake (%)")
    ax.set_title("Brake Command vs Simulator Brake", fontweight="bold")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_ylim(-5, 110)
    ax.grid(True, alpha=0.25)

    # -----------------------------------------------------------------------
    # Panel 4 — Gear + Steer
    # -----------------------------------------------------------------------
    ax = axes[3]
    ax_steer = ax.twinx()

    ax.step(t, ctrl["gear_position"], color=C["gear"], linewidth=2.0,
            where="post", label="Gear (ctrl)")
    ax_steer.plot(t, ctrl["steering_deg"], color="#888888", linewidth=1.2,
                  linestyle="-", alpha=0.7, label="Steer (ctrl)")
    if "steer_deg" in sim_aligned.columns:
        ax_steer.plot(t, sim_aligned["steer_deg"], color=C["sim_v"], linewidth=1.2,
                      linestyle="--", alpha=0.7, label="Steer (sim)")

    shade_guards(ax)
    ax.set_ylabel("Gear (0=N 1=F 2=R)", color=C["gear"])
    ax.set_yticks([0, 1, 2])
    ax.tick_params(axis="y", colors=C["gear"])
    ax_steer.set_ylabel("Steering (deg)", color="#888888")
    ax.set_title("Gear Position + Steering Angle", fontweight="bold")

    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax_steer.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.25)

    # -----------------------------------------------------------------------
    # Panel 5 — PID internals
    # -----------------------------------------------------------------------
    ax = axes[4]
    ax_err = ax.twinx()

    if "p_term_nm" in ctrl.columns:
        ax.plot(t, ctrl["p_term_nm"], color=C["p"], linewidth=1.0, alpha=0.8, label="P")
        ax.plot(t, ctrl["i_term_nm"], color=C["i"], linewidth=1.0, alpha=0.8, label="I")
        ax.plot(t, ctrl["d_term_nm"], color=C["d"], linewidth=1.0, alpha=0.8, label="D")
    ax_err.plot(t, ctrl["error_mps"], color=C["target"], linewidth=1.6,
                linestyle="--", alpha=0.9, label="Error (m/s)")
    ax_err.axhline(0, color="black", linewidth=0.8, alpha=0.4)

    shade_guards(ax)
    ax.set_ylabel("PID term (Nm)")
    ax_err.set_ylabel("Velocity error (m/s)", color=C["target"])
    ax_err.tick_params(axis="y", colors=C["target"])
    ax.set_title("PID Internals (P/I/D terms) + Velocity Error", fontweight="bold")
    ax.set_xlabel("Time (s)")

    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax_err.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.25)

    # -----------------------------------------------------------------------
    # Legend patches for guard windows and gear change markers
    # -----------------------------------------------------------------------
    guard_patch = mpatches.Patch(color=C["guard"], label="Gear-guard braking window")
    gc_line = plt.Line2D([0], [0], color=C["gear"], linestyle=":", linewidth=1.2,
                         label="Gear change event")
    fig.legend(handles=[guard_patch, gc_line], loc="lower center",
               ncol=2, fontsize=9, framealpha=0.9)

    fig.suptitle("Maneuver Diagnostic Overlay — Controller vs Simulator",
                 fontsize=14, fontweight="bold", y=0.995)
    fig.subplots_adjust(left=0.07, right=0.93, top=0.97, bottom=0.07)

    fig.savefig(output_path, dpi=150)
    print(f"  Saved: {output_path}")
    plt.show()
    plt.close(fig)


# ---------------------------------------------------------------------------
# Diagnostics report
# ---------------------------------------------------------------------------

def print_diagnostics(ctrl: pd.DataFrame, sim_aligned: pd.DataFrame):
    print("\n" + "=" * 70)
    print("MANEUVER DIAGNOSTIC REPORT")
    print("=" * 70)

    t = ctrl["time_s"].values
    duration = t[-1] - t[0]
    print(f"\n  Duration:        {duration:.1f} s  ({len(ctrl)} controller samples)")

    # Gear changes
    transitions = gear_transition_times(ctrl)
    guard_windows = gear_change_windows(ctrl)
    print(f"\n  Gear changes:    {len(transitions)}")
    for tc, fg, tg in transitions:
        print(f"    t={tc:.2f}s  {fg} → {tg}")
    print(f"  Guard windows:   {len(guard_windows)}")
    for t0, t1 in guard_windows:
        print(f"    t=[{t0:.2f}, {t1:.2f}]s  ({t1-t0:.2f}s)")

    # Torque saturation
    sat_pct = (ctrl["torque_nm"] >= MAX_TORQUE * 0.99).sum() / len(ctrl) * 100
    print(f"\n  Torque saturation: {sat_pct:.1f}% of samples at max ({MAX_TORQUE/1000:.0f} kNm)")

    # Brake usage
    brake_pct_used = (ctrl["brake_pct"] > 0).sum() / len(ctrl) * 100
    print(f"  Brake usage:       {brake_pct_used:.1f}% of samples")

    # Velocity error by gear
    for gear in sorted(ctrl["gear_position"].unique()):
        mask = ctrl["gear_position"] == gear
        rmse = np.sqrt(np.mean(ctrl.loc[mask, "error_mps"] ** 2))
        max_e = np.max(np.abs(ctrl.loc[mask, "error_mps"]))
        label = {0: "Neutral", 1: "Forward", 2: "Reverse"}.get(gear, str(gear))
        print(f"\n  Gear={gear} ({label})  [{mask.sum()} samples]")
        print(f"    RMSE error:  {rmse:.3f} m/s")
        print(f"    Max |error|: {max_e:.3f} m/s")

    # Sim comparison (if available)
    if "v_mps" in sim_aligned.columns:
        valid = sim_aligned["v_mps"].notna()
        delta = ctrl.loc[valid, "actual_velocity_mps"].values - sim_aligned.loc[valid, "v_mps"].values
        print(f"\n  Ctrl vs sim velocity delta (where both available):")
        print(f"    Mean:  {np.mean(delta):.3f} m/s")
        print(f"    RMSE:  {np.sqrt(np.mean(delta**2)):.3f} m/s")
        print(f"    Max:   {np.max(np.abs(delta)):.3f} m/s")

    print("\n" + "=" * 70 + "\n")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    ctrl_path = sys.argv[1]
    sim_path = sys.argv[2]
    if len(sys.argv) >= 4:
        output_path = sys.argv[3]
    else:
        stem = Path(ctrl_path).stem
        output_path = str(Path(ctrl_path).parent / f"{stem}_overlay.png")

    print(f"\nLoading CSVs:")
    ctrl = load_csv(ctrl_path, "controller")
    sim = load_csv(sim_path, "sim")

    print("\nAligning sim to controller time grid...")
    sim_aligned = interp_sim_to_ctrl(ctrl, sim)

    print_diagnostics(ctrl, sim_aligned)
    plot_overlay(ctrl, sim_aligned, output_path)
    print("Done.")


if __name__ == "__main__":
    main()
