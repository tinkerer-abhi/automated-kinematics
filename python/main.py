"""
Main Execution CLI for Automated Kinematics Tracking System
Supports Live Data Acquisition, Offline File Analysis, and Simulation Modes.
"""

import os
import sys
import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from config import PhysicsParams, AcquisitionParams, FilterParams, DEFAULT_PHYSICS, DEFAULT_ACQUISITION, DEFAULT_FILTER
from serial_logger import SerialKinematicsLogger
from kinematics import KinematicsEngine
from harmonic_fit import HarmonicFitEngine
from visualizer import KinematicsVisualizer, print_experiment_report
from simulate_test import generate_damped_spring_data, save_synthetic_csv


def run_analysis_pipeline(
    raw_time: np.ndarray,
    raw_dist: np.ndarray,
    physics: PhysicsParams,
    filter_params: FilterParams,
    title: str = "Harmonic Motion Kinematics",
    save_plot_path: str = "data/kinematics_dashboard.png",
    show_gui: bool = True
):
    """Executes the full signal processing, kinematics differentiation, fitting, and visualization."""
    ke = KinematicsEngine(filter_params)
    kinematics = ke.process_kinematics(raw_time, raw_dist)

    fe = HarmonicFitEngine(physics)
    fit = fe.fit_damped_motion(kinematics["time"], kinematics["position"])

    energy = fe.calculate_energy(
        kinematics["time"],
        kinematics["position"],
        kinematics["velocity"],
        k=fit["k_effective"],
        x_eq=fit["x_eq"],
        gamma=fit["gamma"]
    )

    # Print summary table in terminal
    print_experiment_report(kinematics, fit, energy)

    # Generate multi-panel visualizer dashboard
    viz = KinematicsVisualizer()
    fig = viz.generate_dashboard(
        kinematics,
        fit,
        energy,
        title_prefix=title,
        save_path=save_plot_path
    )

    if show_gui:
        print("[INFO] Displaying interactive plot window (close window to finish)...")
        plt.show()

    return kinematics, fit, energy


def main():
    parser = argparse.ArgumentParser(
        description="Automated Kinematics & Damped Harmonic Motion Analysis Suite"
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help="Launch the interactive Desktop Graphical User Interface (no code editing needed!)"
    )
    parser.add_argument(
        "--mode",
        choices=["live", "analyze", "sim"],
        default=None,
        help="Execution mode: 'live' (stream from Arduino), 'analyze' (process existing CSV), 'sim' (synthetic simulation)"
    )
    parser.add_argument(
        "--file",
        type=str,
        default=None,
        help="Path to CSV file for 'analyze' mode"
    )
    parser.add_argument(
        "--port",
        type=str,
        default="AUTO",
        help="Arduino serial port (e.g., COM3, /dev/ttyUSB0, or 'AUTO')"
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200)"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=18.0,
        help="Recording duration in seconds (default: 18.0)"
    )
    parser.add_argument(
        "--mass",
        type=float,
        default=None,
        help="Oscillator mass in kg (e.g. 0.200 for 200g, or enter via prompt)"
    )
    parser.add_argument(
        "--type",
        choices=["spring", "pendulum", "freefall"],
        default="spring",
        help="Type of physical oscillator setup (default: spring)"
    )
    parser.add_argument(
        "--out-plot",
        type=str,
        default="data/kinematics_dashboard.png",
        help="Output filepath to save the analysis dashboard figure"
    )
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="Disable interactive graphical popup (useful in automated headless runs)"
    )

    args = parser.parse_args()

    # If --gui or launched with zero args, launch Desktop GUI control panel!
    if args.gui or (args.mode is None and not sys.stdin.isatty()):
        from gui_app import launch_gui
        launch_gui()
        return

    # If mode is not specified in an interactive terminal, ask the user
    if args.mode is None:
        print("\n=======================================================")
        print("     AUTOMATED KINEMATICS EXPERIMENT LAUNCHER          ")
        print("=======================================================")
        print("Select Execution Mode:")
        print("  [1] Live Hardware Run (Stream from Arduino Uno)")
        print("  [2] Analyze Existing CSV File")
        print("  [3] Synthetic Simulation Test (No hardware)")
        print("  [4] Open Desktop GUI Control Center")
        choice = input("Enter choice [1-4, default 4]: ").strip()
        if choice == "1":
            args.mode = "live"
        elif choice == "2":
            args.mode = "analyze"
        elif choice == "3":
            args.mode = "sim"
        else:
            from gui_app import launch_gui
            launch_gui()
            return

    # Interactive prompt for mass if not supplied via command line
    if args.mass is None:
        try:
            m_input = input("\nEnter oscillating mass in grams (e.g., 200 for 200g) [default 200]: ").strip()
            args.mass = float(m_input) / 1000.0 if m_input else 0.200
        except Exception:
            args.mass = 0.200
        print(f"[INFO] Using mass m = {args.mass*1000.0:.1f} g ({args.mass:.4f} kg)")

    # Configure physics
    physics = PhysicsParams(
        mass=args.mass,
        oscillator_type=args.type
    )
    filters = FilterParams()

    if args.mode == "sim":
        print("\n=======================================================")
        print("          MODE: SYNTHETIC SYSTEM SIMULATION            ")
        print("=======================================================")
        sim_csv = os.path.join("data", "simulated_oscillator_data.csv")
        t_raw, dist_raw, _, _ = generate_damped_spring_data(
            duration_sec=args.duration,
            sample_rate_hz=40.0,
            physics=physics,
            initial_displacement_m=0.10,
            damping_coeff_c=0.060,
            equilibrium_dist_m=0.45,
            dropout_prob=0.03
        )
        save_synthetic_csv(sim_csv, t_raw, dist_raw)
        run_analysis_pipeline(
            t_raw,
            dist_raw,
            physics=physics,
            filter_params=filters,
            title="Damped Harmonic Motion - Simulation Verification",
            save_plot_path=args.out_plot,
            show_gui=not args.no_gui
        )

    elif args.mode == "live":
        print("\n=======================================================")
        print("          MODE: LIVE HARDWARE ACQUISITION              ")
        print("=======================================================")
        acq_params = AcquisitionParams(
            port=args.port,
            baud_rate=args.baud,
            acquisition_duration_sec=args.duration
        )
        logger = SerialKinematicsLogger(acq_params)
        if not logger.connect():
            sys.exit(1)

        try:
            csv_path = logger.record_to_csv(duration_sec=args.duration)
        finally:
            logger.close()

        # Load the recorded CSV and process
        df = pd.read_csv(csv_path, comment="#")
        t_raw = df["time_sec"].values
        dist_raw = df["distance_m"].values

        run_analysis_pipeline(
            t_raw,
            dist_raw,
            physics=physics,
            filter_params=filters,
            title="Live Hardware Experiment: Damped Harmonic Motion",
            save_plot_path=args.out_plot,
            show_gui=not args.no_gui
        )

    elif args.mode == "analyze":
        print("\n=======================================================")
        print("          MODE: OFFLINE CSV DATA ANALYSIS              ")
        print("=======================================================")
        if not args.file or not os.path.exists(args.file):
            print(f"[ERROR] Specified data file does not exist: {args.file}")
            sys.exit(1)

        print(f"[INFO] Reading data from: {args.file}")
        df = pd.read_csv(args.file, comment="#")
        t_raw = df["time_sec"].values
        dist_raw = df["distance_m"].values

        run_analysis_pipeline(
            t_raw,
            dist_raw,
            physics=physics,
            filter_params=filters,
            title=f"Kinematics Analysis: {os.path.basename(args.file)}",
            save_plot_path=args.out_plot,
            show_gui=not args.no_gui
        )


if __name__ == "__main__":
    main()
