"""
Publication-Grade Visualizer for Automated Kinematics & Damped Motion
Automated Kinematics and Damped Harmonic Motion Analysis

Generates a comprehensive 4-panel analytical dashboard:
1. Position x(t): Raw data, Savitzky-Golay smooth, Harmonic fit, and Exponential envelope
2. Velocity v(t) and Acceleration a(t) time derivatives (with dual axes)
3. Phase-Space Portrait (v vs x - x_eq) showing inward energy dissipation spiral
4. Energy Conservation: Kinetic (T), Potential (V), Total (E), and Theoretical Decay Envelope
"""

import os
from typing import Dict, Any, Optional
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


class KinematicsVisualizer:
    def __init__(self, style: str = "seaborn-v0_8-whitegrid"):
        try:
            plt.style.use(style)
        except Exception:
            plt.style.use('default')

    def generate_dashboard(
        self,
        kinematics_data: Dict[str, Any],
        fit_results: Dict[str, Any],
        energy_data: Dict[str, Any],
        title_prefix: str = "Damped Harmonic Oscillator",
        save_path: Optional[str] = None
    ):
        """
        Renders the complete 4-panel analysis figure.
        """
        t = kinematics_data["time"]
        t_raw = kinematics_data["raw_time"]
        x_raw = kinematics_data["raw_dist"]
        x_smooth = kinematics_data["position"]
        v_smooth = kinematics_data["velocity"]
        a_smooth = kinematics_data["acceleration"]

        fitted_curve = fit_results["fitted_curve"]
        upper_env = fit_results["upper_envelope"]
        lower_env = fit_results["lower_envelope"]
        x_eq = fit_results["x_eq"]

        fig = plt.figure(figsize=(16, 10), dpi=120)
        gs = GridSpec(2, 2, figure=fig, hspace=0.32, wspace=0.25)

        # ----------------------------------------------------
        # Panel 1: Position Trajectory & Damped Envelope Fit
        # ----------------------------------------------------
        ax1 = fig.add_subplot(gs[0, 0])
        ax1.scatter(t_raw, x_raw, color="#888888", s=10, alpha=0.35, label="Raw HC-SR04 Samples")
        ax1.plot(t, x_smooth, color="#1f77b4", lw=1.8, label="Savitzky-Golay Smoothed")
        ax1.plot(t, fitted_curve, color="#d62728", ls="--", lw=1.5,
                 label=f"Fit: $x(t) = A_0 e^{{-\\gamma t}} \\cos(\\omega_d t + \\phi) + x_{{eq}}$")
        ax1.plot(t, upper_env, color="#2ca02c", ls=":", lw=1.6, label="Decay Envelope ($\\pm A_0 e^{-\\gamma t}$)")
        ax1.plot(t, lower_env, color="#2ca02c", ls=":", lw=1.6)
        ax1.axhline(x_eq, color="#7f7f7f", ls="-.", lw=1.0, label=f"Equilibrium $x_{{eq}} = {x_eq:.3f}$ m")

        ax1.set_title("1. Position Trajectory & Non-Linear Damped Fit", fontsize=12, fontweight="bold")
        ax1.set_xlabel("Time $t$ (seconds)", fontsize=10)
        ax1.set_ylabel("Displacement / Distance $x$ (m)", fontsize=10)
        ax1.legend(loc="upper right", framealpha=0.9, fontsize=8)
        ax1.grid(True, linestyle="--", alpha=0.6)

        # ----------------------------------------------------
        # Panel 2: Kinematic Derivatives (Velocity & Acceleration)
        # ----------------------------------------------------
        ax2_v = fig.add_subplot(gs[1, 0])
        ax2_a = ax2_v.twinx()

        line_v, = ax2_v.plot(t, v_smooth, color="#007acc", lw=1.8, label="Velocity $v(t) = \\dot{x}$ (m/s)")
        line_a, = ax2_a.plot(t, a_smooth, color="#ff7f0e", lw=1.5, ls="--", label="Acceleration $a(t) = \\ddot{x}$ (m/s²)")

        ax2_v.set_title("2. Real-Time Derivatives: Velocity & Acceleration", fontsize=12, fontweight="bold")
        ax2_v.set_xlabel("Time $t$ (seconds)", fontsize=10)
        ax2_v.set_ylabel("Velocity $v$ (m/s)", color="#007acc", fontsize=10)
        ax2_a.set_ylabel("Acceleration $a$ (m/s²)", color="#ff7f0e", fontsize=10)
        ax2_v.tick_params(axis='y', labelcolor="#007acc")
        ax2_a.tick_params(axis='y', labelcolor="#ff7f0e")

        lines = [line_v, line_a]
        labels = [l.get_label() for l in lines]
        ax2_v.legend(lines, labels, loc="upper right", framealpha=0.9, fontsize=8)
        ax2_v.grid(True, linestyle="--", alpha=0.6)

        # ----------------------------------------------------
        # Panel 3: Phase Space Trajectory (Inward Dissipation Spiral)
        # ----------------------------------------------------
        ax3 = fig.add_subplot(gs[0, 1])
        x_centered = (x_smooth - x_eq) * 100.0  # converted to cm for readability
        sc = ax3.scatter(
            x_centered,
            v_smooth,
            c=t,
            cmap="plasma",
            s=12,
            edgecolors="none",
            alpha=0.85
        )
        cbar = fig.colorbar(sc, ax=ax3, orientation="vertical", pad=0.02)
        cbar.set_label("Time Elapsed (s)", fontsize=9)

        # Arrow indicating initial trajectory direction
        if len(x_centered) > 20:
            ax3.annotate(
                "Initial release",
                xy=(x_centered[0], v_smooth[0]),
                xytext=(x_centered[0] + np.sign(x_centered[0])*2.0, v_smooth[0] + 0.05),
                arrowprops=dict(facecolor='black', shrink=0.05, width=1, headwidth=5),
                fontsize=8, fontweight="bold"
            )

        ax3.scatter([0], [0], color="red", marker="x", s=80, label="Equilibrium Attractor (0,0)")
        ax3.set_title("3. Phase-Space Portrait ($v$ vs $x - x_{eq}$)", fontsize=12, fontweight="bold")
        ax3.set_xlabel("Displacement $\\Delta x$ (cm)", fontsize=10)
        ax3.set_ylabel("Velocity $v$ (m/s)", fontsize=10)
        ax3.legend(loc="upper left", framealpha=0.9, fontsize=8)
        ax3.grid(True, linestyle="--", alpha=0.6)

        # ----------------------------------------------------
        # Panel 4: Energy Conservation & Dissipation
        # ----------------------------------------------------
        ax4 = fig.add_subplot(gs[1, 1])
        t_energy = t - t[0]
        K = energy_data["kinetic"] * 1000.0  # Joules to mJ for legible scale
        U = energy_data["potential"] * 1000.0
        E_tot = energy_data["total"] * 1000.0
        E_decay = energy_data["theoretical_decay"] * 1000.0

        ax4.plot(t_energy, K, color="#2ca02c", lw=1.2, alpha=0.75, label="Kinetic Energy $T = \\frac{1}{2}mv^2$ (mJ)")
        ax4.plot(t_energy, U, color="#1f77b4", lw=1.2, alpha=0.75, label="Potential Energy $V = \\frac{1}{2}k(\\Delta x)^2$ (mJ)")
        ax4.plot(t_energy, E_tot, color="#d62728", lw=2.0, label="Total Energy $E_{tot} = T + V$ (mJ)")
        ax4.plot(t_energy, E_decay, color="#111111", ls="--", lw=1.8, label="Theory: $E(t) = E_0 e^{-2\\gamma t}$")

        ax4.set_title("4. Mechanical Energy Conservation & Dissipation", fontsize=12, fontweight="bold")
        ax4.set_xlabel("Time Elapsed (seconds)", fontsize=10)
        ax4.set_ylabel("Energy (mJ)", fontsize=10)
        ax4.legend(loc="upper right", framealpha=0.9, fontsize=8)
        ax4.grid(True, linestyle="--", alpha=0.6)

        # ----------------------------------------------------
        # Super-title & Parameter Summary Banner
        # ----------------------------------------------------
        summary_text = (
            f"Fitted Parameters: "
            f"Period $T_d$ = {fit_results['damped_period_sec']:.3f} s  |  "
            f"Freq $f_d$ = {fit_results['damped_freq_hz']:.2f} Hz  |  "
            f"Decay $\\gamma$ = {fit_results['gamma']:.4f} $s^{{-1}}$  |  "
            f"Damping Ratio $\\zeta$ = {fit_results['damping_ratio_zeta']:.4f}  |  "
            f"Quality Factor $Q$ = {fit_results['quality_factor_Q']:.1f}  |  "
            f"Damping Coeff $c$ = {fit_results['damping_coeff_c']:.4f} N·s/m  |  "
            f"Fit $R^2$ = {fit_results['r_squared']:.4f}"
        )

        fig.suptitle(f"{title_prefix}\n{summary_text}", fontsize=13, fontweight="bold", y=0.98)

        if save_path:
            os.makedirs(os.path.dirname(os.path.abspath(save_path)), exist_ok=True)
            plt.savefig(save_path, dpi=300, bbox_inches="tight")
            print(f"[SUCCESS] High-resolution visualization saved to: {save_path}")

        return fig


def print_experiment_report(kinematics: Dict[str, Any], fit: Dict[str, Any], energy: Dict[str, Any]):
    """Prints a structured scientific terminal summary table."""
    print("\n" + "="*75)
    print("      AUTOMATED KINEMATICS & DAMPED HARMONIC MOTION REPORT")
    print("="*75)
    print(f"Data Samples:           {kinematics['stats']['valid_samples']} valid / {kinematics['stats']['total_raw_samples']} raw ({kinematics['stats']['rejection_rate_pct']:.1f}% rejected)")
    print(f"Duration:               {kinematics['time'][-1] - kinematics['time'][0]:.2f} seconds @ ~{kinematics['stats']['resampled_samples']/(kinematics['time'][-1] - kinematics['time'][0]):.1f} Hz")
    print(f"Goodness of Fit (R²):   {fit['r_squared']:.5f}")
    print("-"*75)
    print("PHYSICAL & DAMPING PARAMETERS:")
    print(f"  • Equilibrium Position (x_eq):    {fit['x_eq']:.4f} ± {fit['x_eq_err']:.4f} m")
    print(f"  • Initial Amplitude (A0):         {fit['A0']:.4f} ± {fit['A0_err']:.4f} m ({fit['A0']*100:.2f} cm)")
    print(f"  • Damped Frequency (f_d):         {fit['damped_freq_hz']:.3f} Hz  (Period T_d = {fit['damped_period_sec']:.3f} s)")
    print(f"  • Natural Frequency (f_0):        {fit['natural_freq_hz']:.3f} Hz (omega_0 = {fit['omega_0']:.3f} rad/s)")
    print(f"  • Decay Constant (gamma):         {fit['gamma']:.4f} ± {fit['gamma_err']:.4f} s⁻¹")
    print(f"  • Damping Ratio (zeta):           {fit['damping_ratio_zeta']:.5f}  ({'Underdamped' if fit['damping_ratio_zeta'] < 1.0 else 'Overdamped'})")
    print(f"  • Quality Factor (Q):             {fit['quality_factor_Q']:.2f}")
    print(f"  • Viscous Damping Coeff (c):      {fit['damping_coeff_c']:.5f} N·s/m (kg/s)")
    print(f"  • Dynamic Spring Constant (k):    {fit['k_effective']:.3f} N/m")
    print("-"*75)
    print("ENERGY CONSERVATION ANALYSIS:")
    print(f"  • Initial Stored Energy (E0):     {energy['E0']*1000.0:.3f} mJ")
    print(f"  • Final Mechanical Energy:        {energy['total'][-1]*1000.0:.3f} mJ")
    energy_lost_pct = ((energy['E0'] - energy['total'][-1]) / energy['E0']) * 100.0 if energy['E0'] > 0 else 0
    print(f"  • Total Mechanical Energy Lost:   {energy_lost_pct:.1f}% over interval")
    print("="*75 + "\n")
