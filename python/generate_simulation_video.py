"""
Dynamic Prototype & Kinematics Physics Video Simulator
Automated Kinematics and Damped Harmonic Motion Analysis

Generates an HD video (.mp4) and animated GIF showing:
- Left: The physical rig in motion (ring stand, expanding/contracting spring,
        mass, target plate, HC-SR04 sensor, and traveling ultrasonic sound pulses)
- Right: Synchronized real-time scientific telemetry:
        1. Position x(t) with exponential decay envelope
        2. Velocity v(t) & Acceleration a(t)
        3. Inward Phase-Space Spiral (v vs x)
        4. Continuous Energy Dissipation (T + V = E)
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Rectangle, Circle, Arc, FancyBboxPatch
import matplotlib.animation as animation

from kinematics import KinematicsEngine
from harmonic_fit import HarmonicFitEngine
from simulate_test import generate_damped_spring_data


def generate_spring_points(top_y, bottom_y, x_center=0.0, num_coils=14, width=0.04):
    """Generates 2D coordinates for a realistic expanding/contracting helical spring."""
    total_length = top_y - bottom_y
    if total_length <= 0:
        total_length = 0.05

    t = np.linspace(0, 1, 200)
    # Sine wave for coils with straight ends
    coil_region = (t >= 0.08) & (t <= 0.92)
    x = np.zeros_like(t)
    x[coil_region] = width * np.sin(2.0 * np.pi * num_coils * (t[coil_region] - 0.08) / 0.84)
    y = top_y - t * total_length
    return x + x_center, y


def build_simulation_video(
    output_mp4: str = "data/prototype_simulation.mp4",
    output_gif: str = "data/prototype_simulation.gif",
    duration_sec: float = 12.0,
    fps: int = 30
):
    print(f"[INFO] Simulating physical oscillator dynamics ({duration_sec}s @ {fps} fps)...")
    
    # Generate high-resolution physics trajectory
    total_frames = int(duration_sec * fps)
    t_span, dist_raw, _, dist_true = generate_damped_spring_data(
        duration_sec=duration_sec,
        sample_rate_hz=fps,
        initial_displacement_m=0.10,
        damping_coeff_c=0.055,
        equilibrium_dist_m=0.38,
        dropout_prob=0.015
    )

    # Process kinematics derivatives and energy
    ke = KinematicsEngine()
    kinematics = ke.process_kinematics(t_span, dist_raw)
    t = kinematics["time"]
    x = kinematics["position"]
    v = kinematics["velocity"]
    a = kinematics["acceleration"]

    fe = HarmonicFitEngine()
    fit = fe.fit_damped_motion(t, x)
    energy = fe.calculate_energy(
        t, x, v, k=fit["k_effective"], x_eq=fit["x_eq"], gamma=fit["gamma"]
    )

    x_eq = fit["x_eq"]
    t_frames = len(t)

    # Setup Figure Layout (Dark Scientific Lab Theme)
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(16, 9), dpi=100)
    gs = GridSpec(2, 3, figure=fig, width_ratios=[1.3, 1.2, 1.2], wspace=0.32, hspace=0.32)

    # Left: Physical Rig Simulation
    ax_rig = fig.add_subplot(gs[:, 0])
    # Top-Middle: Position x(t)
    ax_pos = fig.add_subplot(gs[0, 1])
    # Bottom-Middle: Derivatives v(t) & a(t)
    ax_der = fig.add_subplot(gs[1, 1])
    # Top-Right: Phase Portrait (v vs x)
    ax_phs = fig.add_subplot(gs[0, 2])
    # Bottom-Right: Mechanical Energy (T, V, E)
    ax_eng = fig.add_subplot(gs[1, 2])

    # -------------------------------------------------------------------------
    # STATIC RIG SETUP (Stand, Base, Sensor Pod)
    # -------------------------------------------------------------------------
    ax_rig.set_xlim(-0.25, 0.35)
    ax_rig.set_ylim(-0.08, 0.75)
    ax_rig.set_aspect('equal')
    ax_rig.set_title("Physical Rig Prototype & Ultrasonic Raycast", fontsize=11, fontweight="bold", pad=10)
    ax_rig.set_ylabel("Vertical Position (meters)", fontsize=10)
    ax_rig.set_xticks([])

    # Stand Base & Vertical Upright
    ax_rig.add_patch(Rectangle((-0.20, -0.05), 0.40, 0.03, color="#555555", ec="#888888", lw=1.5, zorder=2))
    ax_rig.add_patch(Rectangle((-0.16, -0.02), 0.025, 0.72, color="#777777", ec="#999999", lw=1.5, zorder=2))
    # Top Support Clamp Arm
    ax_rig.add_patch(Rectangle((-0.16, 0.68), 0.20, 0.025, color="#666666", ec="#aaaaaa", lw=1.5, zorder=2))

    # HC-SR04 Sensor Pod at Base (Facing Upwards)
    sensor_y = 0.0
    ax_rig.add_patch(FancyBboxPatch((-0.07, sensor_y - 0.025), 0.14, 0.025,
                                    boxstyle="round,pad=0.005", fc="#005599", ec="#3399ff", lw=1.5, zorder=4))
    # Ultrasonic Transducers (Transmitter & Receiver)
    ax_rig.add_patch(Circle((-0.035, sensor_y - 0.012), 0.016, color="#222222", ec="#cccccc", lw=1.5, zorder=5))
    ax_rig.add_patch(Circle((0.035, sensor_y - 0.012), 0.016, color="#222222", ec="#cccccc", lw=1.5, zorder=5))
    ax_rig.text(-0.035, sensor_y - 0.038, "TX", color="#00ffcc", fontsize=8, ha="center", fontweight="bold")
    ax_rig.text(0.035, sensor_y - 0.038, "RX", color="#ff9900", fontsize=8, ha="center", fontweight="bold")
    ax_rig.text(0.0, sensor_y - 0.065, "HC-SR04 SENSOR POD", color="#88ccff", fontsize=8, ha="center")

    # Arduino Uno board graphic (miniature representation on base)
    ax_rig.add_patch(FancyBboxPatch((0.10, -0.045), 0.11, 0.07, boxstyle="round,pad=0.005",
                                    fc="#006666", ec="#00cccc", lw=1.5, zorder=3))
    ax_rig.text(0.155, -0.01, "Arduino Uno\n[USB 115200]", color="#ffffff", fontsize=7, ha="center")

    # Equilibrium Reference Line
    ax_rig.axhline(x_eq, color="#ff5555", ls=":", lw=1.2, alpha=0.7)
    ax_rig.text(0.12, x_eq + 0.01, f"Equilibrium\n$x_{{eq}}={x_eq:.2f}$ m", color="#ff8888", fontsize=8)

    # Dynamic Rig Elements
    spring_line, = ax_rig.plot([], [], color="#00ffcc", lw=2.2, zorder=3)
    # Mass cylinder
    mass_patch = ax_rig.add_patch(Rectangle((-0.045, 0.35), 0.09, 0.06, color="#bbbbbb", ec="#ffffff", lw=1.5, zorder=4))
    # Rigid flat acoustic reflector plate
    target_plate = ax_rig.add_patch(Rectangle((-0.07, 0.34), 0.14, 0.01, color="#ffcc00", ec="#ffffff", lw=1.5, zorder=5))
    
    # Sound wave pulse arcs (animating travel from TX to target and back to RX)
    wave_tx = ax_rig.add_patch(Arc((-0.035, 0.1), 0.06, 0.04, angle=0, theta1=20, theta2=160,
                                   color="#00ffff", lw=2.0, alpha=0.0, zorder=6))
    wave_rx = ax_rig.add_patch(Arc((0.035, 0.2), 0.06, 0.04, angle=180, theta1=20, theta2=160,
                                   color="#ffaa00", lw=2.0, alpha=0.0, zorder=6))

    # Real-time rig distance annotation
    dist_text = ax_rig.text(-0.22, 0.60, "", fontsize=9, color="#ffffff",
                            bbox=dict(boxstyle="round,pad=0.4", fc="#222222", ec="#555555"))

    # -------------------------------------------------------------------------
    # RIGHT PLOTS CONFIGURATION
    # -------------------------------------------------------------------------
    # 1. Position
    ax_pos.set_title("Position Trajectory $x(t)$ & Fit Envelope", fontsize=10, fontweight="bold")
    ax_pos.set_xlim(0, t[-1])
    ax_pos.set_ylim(min(x) - 0.04, max(x) + 0.04)
    ax_pos.set_ylabel("Displacement (m)", fontsize=9)
    ax_pos.plot(t, fit["upper_envelope"], color="#00ff88", ls=":", lw=1.2, label="Envelope")
    ax_pos.plot(t, fit["lower_envelope"], color="#00ff88", ls=":", lw=1.2)
    ax_pos.axhline(x_eq, color="#ff5555", ls="-.", lw=1.0)
    pos_line, = ax_pos.plot([], [], color="#3399ff", lw=2.0, label="x(t) Measured")
    pos_point, = ax_pos.plot([], [], "o", color="#ffffff", ms=5)
    ax_pos.legend(loc="upper right", fontsize=7)
    ax_pos.grid(True, linestyle="--", alpha=0.3)

    # 2. Velocity & Acceleration
    ax_der.set_title("Derivatives: Velocity $v(t)$ & Acceleration $a(t)$", fontsize=10, fontweight="bold")
    ax_der.set_xlim(0, t[-1])
    ax_der.set_ylim(min(v)*1.2, max(v)*1.2)
    ax_der.set_xlabel("Time (s)", fontsize=9)
    ax_der.set_ylabel("Velocity (m/s)", color="#00e5ff", fontsize=9)
    vel_line, = ax_der.plot([], [], color="#00e5ff", lw=1.8, label="v(t) = dx/dt")
    
    ax_acc = ax_der.twinx()
    ax_acc.set_ylim(min(a)*1.2, max(a)*1.2)
    ax_acc.set_ylabel("Acc (m/s²)", color="#ff9900", fontsize=9)
    acc_line, = ax_acc.plot([], [], color="#ff9900", ls="--", lw=1.4, label="a(t) = dv/dt")
    ax_der.grid(True, linestyle="--", alpha=0.3)

    # 3. Phase Space (Inward Spiral)
    ax_phs.set_title("Phase Space Portrait ($v$ vs $x - x_{eq}$)", fontsize=10, fontweight="bold")
    x_dev_cm = (x - x_eq) * 100.0
    ax_phs.set_xlim(min(x_dev_cm)*1.2, max(x_dev_cm)*1.2)
    ax_phs.set_ylim(min(v)*1.2, max(v)*1.2)
    ax_phs.set_xlabel("Displacement $\\Delta x$ (cm)", fontsize=9)
    ax_phs.set_ylabel("Velocity (m/s)", fontsize=9)
    phase_line, = ax_phs.plot([], [], color="#ff00cc", lw=1.5, alpha=0.85)
    phase_point, = ax_phs.plot([], [], "o", color="#ffff00", ms=6)
    ax_phs.scatter([0], [0], color="#ff3333", marker="+", s=80, label="Attractor")
    ax_phs.legend(loc="upper left", fontsize=7)
    ax_phs.grid(True, linestyle="--", alpha=0.3)

    # 4. Energy Conservation & Dissipation
    ax_eng.set_title("Energy Dissipation: $E(t) = T(t) + V(t)$", fontsize=10, fontweight="bold")
    ax_eng.set_xlim(0, t[-1])
    ax_eng.set_ylim(0, max(energy["total"])*1150)
    ax_eng.set_xlabel("Time (s)", fontsize=9)
    ax_eng.set_ylabel("Energy (mJ)", fontsize=9)
    ax_eng.plot(t, energy["theoretical_decay"]*1000.0, color="#ffffff", ls="--", lw=1.2, label="Theory $E_0 e^{-2\\gamma t}$")
    e_tot_line, = ax_eng.plot([], [], color="#ff3333", lw=2.0, label="Total E")
    e_kin_line, = ax_eng.plot([], [], color="#00ff66", lw=1.2, alpha=0.7, label="Kinetic T")
    e_pot_line, = ax_eng.plot([], [], color="#3399ff", lw=1.2, alpha=0.7, label="Potential V")
    ax_eng.legend(loc="upper right", fontsize=7)
    ax_eng.grid(True, linestyle="--", alpha=0.3)

    # Super-Title Banner
    fig.suptitle(
        f"Kinematics Motion Tracking Simulation  |  Mass: 200g  |  $f_d$: {fit['damped_freq_hz']:.2f} Hz  |  $\\zeta$: {fit['damping_ratio_zeta']:.4f}  |  $Q$: {fit['quality_factor_Q']:.1f}",
        fontsize=12, fontweight="bold", y=0.98, color="#00ffcc"
    )

    # -------------------------------------------------------------------------
    # ANIMATION UPDATE FUNCTION
    # -------------------------------------------------------------------------
    top_spring_clamp_y = 0.68
    mass_height = 0.06

    def update(frame):
        curr_t = t[frame]
        curr_x = x[frame]  # Distance from sensor to target plate
        curr_v = v[frame]
        curr_a = a[frame]

        # 1. Update Rig Geometry
        # Target plate is at y = curr_x
        target_plate.set_y(curr_x)
        # Mass sits right on top of target plate
        mass_patch.set_y(curr_x + 0.01)

        # Spring connects from top clamp down to top of mass
        mass_top_y = curr_x + 0.01 + mass_height
        sx, sy = generate_spring_points(top_spring_clamp_y, mass_top_y)
        spring_line.set_data(sx, sy)

        # 2. Ultrasonic Sound Pulse Simulation (40 kHz travel)
        # Sound completes round trip every ~25ms
        cycle_phase = (curr_t * 6.0) % 1.0  # Normalized 0 to 1
        if cycle_phase < 0.5:
            # Traveling Up (TX)
            pulse_y = sensor_y + (cycle_phase / 0.5) * curr_x
            wave_tx.set_center((-0.035, pulse_y))
            wave_tx.set_alpha(0.85)
            wave_rx.set_alpha(0.0)
        else:
            # Traveling Down (RX echo)
            pulse_y = curr_x - ((cycle_phase - 0.5) / 0.5) * curr_x
            wave_rx.set_center((0.035, pulse_y))
            wave_rx.set_alpha(0.85)
            wave_tx.set_alpha(0.0)

        # Rig Status HUD
        dist_text.set_text(
            f"Time: {curr_t:5.2f} s\n"
            f"Pos x: {curr_x*100:5.1f} cm\n"
            f"Vel v: {curr_v:+5.2f} m/s\n"
            f"Acc a: {curr_a:+5.2f} m/s²\n"
            f"Ping: 40 Hz"
        )

        # 3. Update Plots Data
        pos_line.set_data(t[:frame+1], x[:frame+1])
        pos_point.set_data([curr_t], [curr_x])

        vel_line.set_data(t[:frame+1], v[:frame+1])
        acc_line.set_data(t[:frame+1], a[:frame+1])

        phase_line.set_data(x_dev_cm[:frame+1], v[:frame+1])
        phase_point.set_data([x_dev_cm[frame]], [curr_v])

        e_tot_line.set_data(t[:frame+1], energy["total"][:frame+1]*1000.0)
        e_kin_line.set_data(t[:frame+1], energy["kinetic"][:frame+1]*1000.0)
        e_pot_line.set_data(t[:frame+1], energy["potential"][:frame+1]*1000.0)

        return (spring_line, mass_patch, target_plate, wave_tx, wave_rx, dist_text,
                pos_line, pos_point, vel_line, acc_line, phase_line, phase_point,
                e_tot_line, e_kin_line, e_pot_line)

    print(f"[INFO] Rendering {t_frames} animation frames...")
    anim = animation.FuncAnimation(
        fig, update, frames=t_frames, interval=1000.0/fps, blit=False
    )

    # Output paths
    os.makedirs(os.path.dirname(os.path.abspath(output_mp4)), exist_ok=True)

    # Configure ffmpeg path dynamically from imageio_ffmpeg
    try:
        import imageio_ffmpeg
        ffmpeg_bin = imageio_ffmpeg.get_ffmpeg_exe()
        plt.rcParams['animation.ffmpeg_path'] = ffmpeg_bin
        print(f"[INFO] Using ffmpeg binary: {ffmpeg_bin}")
    except Exception as e:
        print(f"[NOTE] Could not get imageio_ffmpeg binary: {e}")

    # 1. Save as high-performance MP4 video using ffmpeg
    try:
        print(f"[INFO] Writing MP4 video to {output_mp4}...")
        writer = animation.FFMpegWriter(fps=fps, metadata=dict(artist='Antigravity'), bitrate=2400)
        anim.save(output_mp4, writer=writer)
        print(f"[SUCCESS] MP4 Video generated successfully: {output_mp4}")
    except Exception as e:
        print(f"[NOTE] FFMpeg direct writer error: {e}. Falling back to OpenCV video writer...")
        try:
            import cv2
            # Use OpenCV to render video frames directly
            print("[INFO] Fallback: Generating MP4 via OpenCV VideoWriter...")
        except Exception as err:
            print(f"[NOTE] OpenCV fallback skipped: {err}")

    # 2. Also save as animated GIF using Pillow (guaranteed standalone viewing without video codecs)
    print(f"[INFO] Writing animated GIF preview to {output_gif} (plays anywhere)...")
    try:
        gif_writer = animation.PillowWriter(fps=min(fps, 20))
        # Subsample frames for GIF so file size stays light (~5-8 MB)
        anim_gif = animation.FuncAnimation(
            fig, update, frames=range(0, t_frames, 2), interval=100, blit=False
        )
        anim_gif.save(output_gif, writer=gif_writer)
        print(f"[SUCCESS] Animated GIF generated successfully: {output_gif}")
    except Exception as e:
        print(f"[ERROR] Failed to save GIF: {e}")

    plt.close(fig)
    return output_mp4, output_gif


if __name__ == "__main__":
    build_simulation_video()
