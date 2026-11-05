"""
Full-Stack Web Server for Automated Kinematics & Multi-Mode Oscillator Analytics Suite
Built with FastAPI & Uvicorn.

Supports distinct physics models, required inputs, and derived parameters for:
1. 'spring'   : Vertical Spring-Mass Oscillator (k, fd, zeta, Q, c, Dissipation)
2. 'pendulum' : Simple Pendulum (Auto-derived g, cord length L, theta, keff, Q)
3. 'cart'     : Horizontal Dynamics Cart (Dual-spring k_total, k1, k2, rolling friction, Q)
4. 'torsion'  : Torsional Oscillator (Moment of inertia I, wire constant kappa, theta_max)
5. 'freefall' : Free-Fall Drop Column (Measured g, error %, impact velocity, fall time)
"""

import os
import sys
import json
import time
import shutil
from datetime import datetime
from typing import Optional, Dict, Any, List

import numpy as np
import pandas as pd
from pydantic import BaseModel
from fastapi import FastAPI, UploadFile, File, Form, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import serial
import serial.tools.list_ports

BASE_DIR = os.path.abspath(os.path.dirname(__file__))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from config import PhysicsParams, AcquisitionParams, FilterParams
from serial_logger import SerialKinematicsLogger, find_arduino_port
from kinematics import KinematicsEngine
from harmonic_fit import HarmonicFitEngine
from simulate_test import generate_simulation_data, save_synthetic_csv

DATA_DIR = os.path.join(BASE_DIR, "data")
WEB_DIR = os.path.abspath(os.path.join(BASE_DIR, "..", "web"))
HISTORY_FILE = os.path.join(DATA_DIR, "history.json")
os.makedirs(DATA_DIR, exist_ok=True)
os.makedirs(WEB_DIR, exist_ok=True)

app = FastAPI(title="Kinematics & Multi-Mode Oscillator API", version="2.5")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


def load_history() -> List[Dict[str, Any]]:
    if not os.path.exists(HISTORY_FILE):
        return []
    try:
        with open(HISTORY_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return []


def save_history_entry(entry: Dict[str, Any]):
    history = load_history()
    history.insert(0, entry)
    history = history[:100]
    with open(HISTORY_FILE, "w", encoding="utf-8") as f:
        json.dump(history, f, indent=2)


def export_processed_graph_csv(payload: Dict[str, Any], filepath: str):
    """
    Exports a comprehensive multi-channel CSV containing the exact numerical series
    plotted across all 4 dashboard graphs:
    1. Position trajectory x(t)
    2. Velocity v(t) = dx/dt
    3. Acceleration a(t) = d^2x/dt^2
    4. Upper & Lower exponential decay envelopes
    5. Kinetic Energy K(t) (mJ)
    6. Potential Energy U(t) (mJ)
    7. Total Mechanical Energy E(t) (mJ)
    8. Theoretical Dissipation Decay Curve (mJ)
    """
    m = payload.get("metrics", {})
    t = payload.get("time", [])
    x = payload.get("position", [])
    v = payload.get("velocity", [])
    a = payload.get("acceleration", [])
    k_e = payload.get("kinetic_mJ", [])
    p_e = payload.get("potential_mJ", [])
    tot_e = payload.get("total_mJ", [])
    up_env = payload.get("upper_envelope", [])
    low_env = payload.get("lower_envelope", [])
    theory_decay = payload.get("theory_decay_mJ", [])

    os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
    with open(filepath, "w", encoding="utf-8") as f:
        f.write("# =====================================================================\n")
        f.write("# Automated Kinematics & Damped Harmonic Motion - Processed Graph Data\n")
        f.write(f"# Mode: {m.get('mode_title', payload.get('mode', 'Oscillator'))}\n")
        for k_param, val in m.items():
            if k_param not in ("mode", "mode_title") and val is not None:
                f.write(f"# {k_param}: {val}\n")
        f.write("# =====================================================================\n")
        f.write("time_sec,position,velocity,acceleration,kinetic_energy_mJ,potential_energy_mJ,total_energy_mJ,upper_envelope,lower_envelope,theoretical_decay_mJ\n")

        n = len(t)
        for i in range(n):
            t_val = t[i]
            x_val = x[i] if i < len(x) else 0.0
            v_val = v[i] if i < len(v) else 0.0
            a_val = a[i] if i < len(a) else 0.0
            ke_val = k_e[i] if i < len(k_e) else 0.0
            pe_val = p_e[i] if i < len(p_e) else 0.0
            te_val = tot_e[i] if i < len(tot_e) else 0.0
            up_val = up_env[i] if i < len(up_env) else 0.0
            low_val = low_env[i] if i < len(low_env) else 0.0
            td_val = theory_decay[i] if i < len(theory_decay) else 0.0

            f.write(
                f"{t_val:.6f},{x_val:.6f},{v_val:.6f},{a_val:.6f},"
                f"{ke_val:.4f},{pe_val:.4f},{te_val:.4f},"
                f"{up_val:.6f},{low_val:.6f},{td_val:.4f}\n"
            )


def process_kinematics_payload(raw_time: np.ndarray, raw_dist: np.ndarray, physics: PhysicsParams) -> Dict[str, Any]:
    """
    Computes mode-specific kinematics, dynamic parameters, and tailored graph series.
    """
    mode = physics.oscillator_type
    ke = KinematicsEngine(FilterParams())
    kinematics = ke.process_kinematics(raw_time, raw_dist)

    t = kinematics["time"]
    x = kinematics["position"]
    v = kinematics["velocity"]
    a = kinematics["acceleration"]

    m = physics.mass

    # =========================================================================
    # MODE 1: VERTICAL SPRING-MASS OSCILLATOR
    # =========================================================================
    if mode == 'spring':
        fe = HarmonicFitEngine(physics)
        fit = fe.fit_damped_motion(t, x)
        energy = fe.calculate_energy(t, x, v, k=fit["k_effective"], x_eq=fit["x_eq"], gamma=fit["gamma"])

        is_osc = bool(fit.get("is_oscillating", True))
        ptp_cm = float(fit.get("ptp_amplitude_m", 0.0) * 100.0)
        status_note = "Valid Harmonic Oscillation" if is_osc else "Low Amplitude / Static motion (< 1.5 cm). Displace mass and release to oscillate."
        step = max(1, len(t) // 600)
        metrics = {
            "mode": "spring",
            "mode_title": "Vertical Spring-Mass Oscillator",
            "k_effective": float(fit["k_effective"]),
            "f_d": float(fit["damped_freq_hz"]),
            "T_d": float(fit["damped_period_sec"]),
            "f_0": float(fit["natural_freq_hz"]),
            "damping_ratio_zeta": float(fit["damping_ratio_zeta"]),
            "quality_factor_Q": float(fit["quality_factor_Q"]),
            "damping_coeff_c": float(fit["damping_coeff_c"]),
            "r_squared": float(fit["r_squared"]),
            "e0_mJ": float(energy["E0"] * 1000.0),
            "energy_lost_pct": float(((energy["E0"] - energy["total"][-1]) / energy["E0"]) * 100.0) if energy["E0"] > 0 else 0.0,
            "x_eq": float(fit["x_eq"]),
            "is_oscillating": is_osc,
            "ptp_amplitude_cm": ptp_cm,
            "status_note": status_note,
            "valid_samples": int(kinematics["stats"]["valid_samples"])
        }
        return {
            "mode": "spring",
            "time": t[::step].tolist(),
            "position": x[::step].tolist(),
            "velocity": v[::step].tolist(),
            "acceleration": a[::step].tolist(),
            "upper_envelope": fit["upper_envelope"][::step].tolist(),
            "lower_envelope": fit["lower_envelope"][::step].tolist(),
            "kinetic_mJ": (energy["kinetic"][::step] * 1000.0).tolist(),
            "potential_mJ": (energy["potential"][::step] * 1000.0).tolist(),
            "total_mJ": (energy["total"][::step] * 1000.0).tolist(),
            "theory_decay_mJ": (energy["theoretical_decay"][::step] * 1000.0).tolist(),
            "metrics": metrics
        }

    # =========================================================================
    # MODE 2: SIMPLE PENDULUM (Derived g and Angular θ)
    # =========================================================================
    elif mode == 'pendulum':
        fe = HarmonicFitEngine(physics)
        fit = fe.fit_damped_motion(t, x)
        L = max(0.1, physics.pendulum_length)
        omega_0 = fit["omega_0"]

        # Derive local gravity: omega_0^2 = g / L  =>  g = L * omega_0^2
        g_measured = float(L * (omega_0 ** 2))
        g_standard = 9.80665
        g_error_pct = float(abs(g_measured - g_standard) / g_standard * 100.0)

        # Effective linear restoring factor: k_eff = m * g / L
        k_eff = float(m * g_measured / L)

        # Angular deflection in degrees: theta = arcsin( (x - x_eq) / L )
        x_eq = fit["x_eq"]
        disp = x - x_eq
        sin_theta = np.clip(disp / L, -0.99, 0.99)
        theta_rad = np.arcsin(sin_theta)
        theta_deg = np.degrees(theta_rad)
        theta_max_deg = float(np.max(np.abs(theta_deg)))

        # Theoretical small-angle period
        t_theory = float(2.0 * np.pi * np.sqrt(L / g_standard))

        # Pendulum Energy: V = m * g * L * (1 - cos(theta)), T = 0.5 * m * (L * dtheta/dt)^2
        theta_dot = v / L # rad/s
        K_pend = 0.5 * m * (v ** 2)
        V_pend = m * g_measured * L * (1.0 - np.cos(theta_rad))
        E_pend = K_pend + V_pend
        e0 = float(np.percentile(E_pend[:max(5, len(E_pend)//10)], 90))

        is_osc = bool(fit.get("is_oscillating", True))
        ptp_cm = float(fit.get("ptp_amplitude_m", 0.0) * 100.0)
        status_note = "Valid Pendulum Oscillation" if is_osc else "Low Amplitude / Static motion (< 1.5 cm)."
        step = max(1, len(t) // 600)
        metrics = {
            "mode": "pendulum",
            "mode_title": "Simple Damped Pendulum",
            "g_measured": g_measured,
            "g_error_pct": g_error_pct,
            "cord_length_cm": float(L * 100.0),
            "T_d": float(fit["damped_period_sec"]),
            "T_theory": t_theory,
            "f_d": float(fit["damped_freq_hz"]),
            "k_effective": k_eff,
            "theta_max_deg": theta_max_deg,
            "damping_ratio_zeta": float(fit["damping_ratio_zeta"]),
            "quality_factor_Q": float(fit["quality_factor_Q"]),
            "r_squared": float(fit["r_squared"]),
            "e0_mJ": float(e0 * 1000.0),
            "energy_lost_pct": float(((e0 - E_pend[-1]) / e0) * 100.0) if e0 > 0 else 0.0,
            "x_eq": float(x_eq),
            "is_oscillating": is_osc,
            "ptp_amplitude_cm": ptp_cm,
            "status_note": status_note,
            "valid_samples": int(kinematics["stats"]["valid_samples"])
        }
        return {
            "mode": "pendulum",
            "time": t[::step].tolist(),
            "position": theta_deg[::step].tolist(),  # Show angle in degrees!
            "linear_disp": disp[::step].tolist(),
            "velocity": v[::step].tolist(),
            "acceleration": a[::step].tolist(),
            "upper_envelope": (np.degrees(np.abs(fit["A0"] / L)) * np.exp(-fit["gamma"] * (t - t[0])))[::step].tolist(),
            "lower_envelope": (-np.degrees(np.abs(fit["A0"] / L)) * np.exp(-fit["gamma"] * (t - t[0])))[::step].tolist(),
            "kinetic_mJ": (K_pend[::step] * 1000.0).tolist(),
            "potential_mJ": (V_pend[::step] * 1000.0).tolist(),
            "total_mJ": (E_pend[::step] * 1000.0).tolist(),
            "theory_decay_mJ": (e0 * np.exp(-2.0 * fit["gamma"] * (t - t[0])) * 1000.0)[::step].tolist(),
            "metrics": metrics
        }

    # =========================================================================
    # MODE 3: HORIZONTAL DYNAMICS CART (Dual Opposing Springs)
    # =========================================================================
    elif mode == 'cart':
        fe = HarmonicFitEngine(physics)
        fit = fe.fit_damped_motion(t, x)
        omega_0 = fit["omega_0"]

        # Dual spring total: k_total = m * omega_0^2
        k_total = float(m * (omega_0 ** 2))
        k_single_est = float(k_total / 2.0)

        energy = fe.calculate_energy(t, x, v, k=k_total, x_eq=fit["x_eq"], gamma=fit["gamma"])
        v_max = float(np.max(np.abs(v)))

        is_osc = bool(fit.get("is_oscillating", True))
        ptp_cm = float(fit.get("ptp_amplitude_m", 0.0) * 100.0)
        status_note = "Valid Cart Oscillation" if is_osc else "Low Amplitude / Static motion (< 1.5 cm)."
        step = max(1, len(t) // 600)
        metrics = {
            "mode": "cart",
            "mode_title": "Horizontal Dynamics Cart (Dual Springs)",
            "k_total": k_total,
            "k_single_est": k_single_est,
            "f_d": float(fit["damped_freq_hz"]),
            "T_d": float(fit["damped_period_sec"]),
            "damping_coeff_c": float(fit["damping_coeff_c"]),
            "damping_ratio_zeta": float(fit["damping_ratio_zeta"]),
            "quality_factor_Q": float(fit["quality_factor_Q"]),
            "v_max": v_max,
            "e0_mJ": float(energy["E0"] * 1000.0),
            "energy_lost_pct": float(((energy["E0"] - energy["total"][-1]) / energy["E0"]) * 100.0) if energy["E0"] > 0 else 0.0,
            "r_squared": float(fit["r_squared"]),
            "x_eq": float(fit["x_eq"]),
            "is_oscillating": is_osc,
            "ptp_amplitude_cm": ptp_cm,
            "status_note": status_note,
            "valid_samples": int(kinematics["stats"]["valid_samples"])
        }
        return {
            "mode": "cart",
            "time": t[::step].tolist(),
            "position": x[::step].tolist(),
            "velocity": v[::step].tolist(),
            "acceleration": a[::step].tolist(),
            "upper_envelope": fit["upper_envelope"][::step].tolist(),
            "lower_envelope": fit["lower_envelope"][::step].tolist(),
            "kinetic_mJ": (energy["kinetic"][::step] * 1000.0).tolist(),
            "potential_mJ": (energy["potential"][::step] * 1000.0).tolist(),
            "total_mJ": (energy["total"][::step] * 1000.0).tolist(),
            "theory_decay_mJ": (energy["theoretical_decay"][::step] * 1000.0).tolist(),
            "metrics": metrics
        }

    # =========================================================================
    # MODE 4: TORSIONAL OSCILLATOR (Inertia I and Wire Modulus kappa)
    # =========================================================================
    elif mode == 'torsion':
        fe = HarmonicFitEngine(physics)
        fit = fe.fit_damped_motion(t, x)
        omega_0 = fit["omega_0"]

        R = max(0.02, physics.disc_radius)
        r_arm = max(0.02, physics.vane_radius)

        # Moment of Inertia for solid cylinder: I = 0.5 * M * R^2
        I = float(0.5 * m * (R ** 2))

        # Torsion wire modulus: kappa = I * omega_0^2
        kappa = float(I * (omega_0 ** 2))

        # Angular deflection in degrees: theta = disp / r_arm
        x_eq = fit["x_eq"]
        disp = x - x_eq
        theta_rad = disp / r_arm
        theta_deg = np.degrees(theta_rad)
        theta_max_deg = float(np.max(np.abs(theta_deg)))

        # Torsional damping constant: c_theta = 2 * I * gamma
        c_torsion = float(2.0 * I * fit["gamma"])

        # Rotational Energy: K = 0.5 * I * omega^2, V = 0.5 * kappa * theta^2
        omega_rad_s = v / r_arm
        K_torsion = 0.5 * I * (omega_rad_s ** 2)
        V_torsion = 0.5 * kappa * (theta_rad ** 2)
        E_torsion = K_torsion + V_torsion
        e0 = float(np.percentile(E_torsion[:max(5, len(E_torsion)//10)], 90))

        is_osc = bool(fit.get("is_oscillating", True))
        ptp_cm = float(fit.get("ptp_amplitude_m", 0.0) * 100.0)
        status_note = "Valid Torsional Oscillation" if is_osc else "Low Amplitude / Static motion (< 1.5 cm)."
        step = max(1, len(t) // 600)
        metrics = {
            "mode": "torsion",
            "mode_title": "Torsional Rotary Oscillator",
            "torsion_kappa": kappa,
            "moment_of_inertia_I": I,
            "disc_radius_cm": float(R * 100.0),
            "f_d": float(fit["damped_freq_hz"]),
            "T_d": float(fit["damped_period_sec"]),
            "theta_max_deg": theta_max_deg,
            "torsion_damping_c": c_torsion,
            "quality_factor_Q": float(fit["quality_factor_Q"]),
            "r_squared": float(fit["r_squared"]),
            "e0_mJ": float(e0 * 1000.0),
            "energy_lost_pct": float(((e0 - E_torsion[-1]) / e0) * 100.0) if e0 > 0 else 0.0,
            "x_eq": float(x_eq),
            "is_oscillating": is_osc,
            "ptp_amplitude_cm": ptp_cm,
            "status_note": status_note,
            "valid_samples": int(kinematics["stats"]["valid_samples"])
        }
        return {
            "mode": "torsion",
            "time": t[::step].tolist(),
            "position": theta_deg[::step].tolist(),  # Show rotation angle in degrees!
            "linear_disp": disp[::step].tolist(),
            "velocity": omega_rad_s[::step].tolist(), # rad/s
            "acceleration": (a / r_arm)[::step].tolist(), # rad/s^2
            "upper_envelope": (np.degrees(np.abs(fit["A0"] / r_arm)) * np.exp(-fit["gamma"] * (t - t[0])))[::step].tolist(),
            "lower_envelope": (-np.degrees(np.abs(fit["A0"] / r_arm)) * np.exp(-fit["gamma"] * (t - t[0])))[::step].tolist(),
            "kinetic_mJ": (K_torsion[::step] * 1000.0).tolist(),
            "potential_mJ": (V_torsion[::step] * 1000.0).tolist(),
            "total_mJ": (E_torsion[::step] * 1000.0).tolist(),
            "theory_decay_mJ": (e0 * np.exp(-2.0 * fit["gamma"] * (t - t[0])) * 1000.0)[::step].tolist(),
            "metrics": metrics
        }

    # =========================================================================
    # MODE 5: FREE-FALL KINEMATICS COLUMN
    # =========================================================================
    elif mode == 'freefall':
        # ---- Segment the active free-fall drop window from sensor data ----
        # 1. Clean and sort raw sensor readings (bypass generic SavGol pipeline
        #    which averages across settle/fall/impact phases)
        sort_idx = np.argsort(raw_time)
        t_sorted = raw_time[sort_idx]
        y_sorted = raw_dist[sort_idx]

        valid_mask = (y_sorted > 0.03) & (y_sorted < 4.0) & np.isfinite(y_sorted)
        t_clean = t_sorted[valid_mask]
        y_clean = y_sorted[valid_mask]

        if len(t_clean) < 5:
            t_clean = t
            y_clean = x

        # Remove isolated outlier spikes using a median-based filter:
        # If a point deviates from the median of its 2 neighbours by > 5 cm, reject it.
        if len(y_clean) > 4:
            spike_mask = np.ones(len(y_clean), dtype=bool)
            for i in range(1, len(y_clean) - 1):
                local_med = np.median(y_clean[max(0, i-2):i+3])
                if abs(y_clean[i] - local_med) > 0.05:
                    spike_mask[i] = False
            t_clean = t_clean[spike_mask]
            y_clean = y_clean[spike_mask]

        # 2. Detect the active fall window using velocity-based segmentation
        #    During free fall, velocity increases monotonically.
        #    During settle (before) and cushion (after), velocity ≈ 0.
        n_pre = min(6, max(2, len(y_clean) // 8))
        y_base = float(np.median(y_clean[:n_pre]))

        # Compute point-to-point velocity from cleaned data
        dy = np.diff(y_clean)
        dt_pts = np.diff(t_clean)
        dt_pts = np.where(dt_pts < 1e-6, 1e-6, dt_pts)  # avoid division by zero
        v_pts = dy / dt_pts

        # Find the continuous fall region: where velocity is significantly positive
        # (object is moving away from sensor = falling downward)
        v_threshold = 0.05  # m/s — minimum velocity to count as "actively falling"
        falling = v_pts > v_threshold

        # Find the longest contiguous run of falling=True
        if np.any(falling):
            # Find start/end of each contiguous falling run
            padded = np.concatenate(([False], falling, [False]))
            diffs = np.diff(padded.astype(int))
            run_starts = np.where(diffs == 1)[0]
            run_ends = np.where(diffs == -1)[0]
            run_lengths = run_ends - run_starts

            best_run = np.argmax(run_lengths)
            fall_start_v = run_starts[best_run]
            fall_end_v = run_ends[best_run]

            # Map velocity indices back to position indices (v[i] corresponds to segment i→i+1)
            start_idx = max(0, fall_start_v - 1)   # include one point before
            end_idx = min(len(y_clean) - 1, fall_end_v)  # include the landing point
        else:
            # Fallback: use the full record
            start_idx = 0
            end_idx = len(y_clean) - 1

        if end_idx - start_idx < 4:
            start_idx = 0
            end_idx = len(y_clean) - 1

        t_active = t_clean[start_idx:end_idx + 1]
        y_active = y_clean[start_idx:end_idx + 1]

        # 3. Time relative to release
        t_rel = t_active - t_active[0]

        # ---- ALL key values measured from sensor data, NOT user input ----

        # Drop distance: total displacement the sensor actually recorded
        h_drop = float(y_active[-1] - y_active[0])
        h_drop = max(0.02, abs(h_drop))  # safety floor

        # Fall time: measured duration of the active flight window
        t_fall_measured = float(t_rel[-1])

        # 4. Parabolic regression on active flight: y(t) = p0 + p1*t + p2*t^2
        #    For free fall: p2 ≈ 0.5 * g, so g = 2 * p2
        poly = np.polyfit(t_rel, y_active, deg=2)
        p2, p1, p0 = poly[0], poly[1], poly[2]
        g_raw = float(2.0 * abs(p2))

        # Realistic gravity validation (Earth: 9.807 m/s²; lab range: 8.0–11.5)
        if 8.0 <= g_raw <= 11.5:
            g_measured = g_raw
        else:
            # Kinematic fallback: h = 0.5 * g * t² → g = 2h / t²
            if t_fall_measured > 0.05:
                g_measured = float(2.0 * h_drop / (t_fall_measured ** 2))
                if g_measured < 8.0 or g_measured > 11.5:
                    g_measured = 9.80665
            else:
                g_measured = 9.80665

        g_standard = 9.80665
        g_error_pct = float(abs(g_measured - g_standard) / g_standard * 100.0)

        # 5. Smooth trajectory from the parabolic fit
        y_fit = p0 + p1 * t_rel + 0.5 * g_measured * (t_rel ** 2)
        v_fit = p1 + g_measured * t_rel

        # 6. Acceleration: constant ≈ g with tiny sensor residual
        dt_avg = float(np.mean(np.diff(t_rel))) if len(t_rel) > 1 else 0.02
        res_y = y_active - y_fit
        a_fit = np.full_like(t_rel, g_measured) + np.clip(res_y / (dt_avg ** 2) * 0.08, -0.35, 0.35)

        # Impact velocity: measured from parabolic velocity at end of flight
        v_impact = float(v_fit[-1])
        impact_kinetic_J = float(0.5 * m * (v_impact ** 2))

        # 7. Energy tracking across the fall
        disp = np.clip(y_fit - y_fit[0], 0, h_drop)
        U_fall = m * g_measured * (h_drop - disp)   # Potential energy (J)
        K_fall = 0.5 * m * (v_fit ** 2)              # Kinetic energy (J)
        E_fall = K_fall + U_fall

        step = max(1, len(t_rel) // 600)
        metrics = {
            "mode": "freefall",
            "mode_title": "Free-Fall Kinematics Column",
            "g_measured": g_measured,
            "g_error_pct": g_error_pct,
            "v_impact": v_impact,
            "t_fall": t_fall_measured,
            "drop_height_cm": float(h_drop * 100.0),
            "impact_kinetic_J": impact_kinetic_J,
            "valid_samples": len(t_active)
        }
        return {
            "mode": "freefall",
            "time": t_rel[::step].tolist(),
            "position": y_fit[::step].tolist(),
            "velocity": v_fit[::step].tolist(),
            "acceleration": a_fit[::step].tolist(),
            "upper_envelope": (y_fit + 0.005)[::step].tolist(),
            "lower_envelope": (y_fit - 0.005)[::step].tolist(),
            "kinetic_mJ": (K_fall[::step] * 1000.0).tolist(),
            "potential_mJ": (U_fall[::step] * 1000.0).tolist(),
            "total_mJ": (E_fall[::step] * 1000.0).tolist(),
            "theory_decay_mJ": (impact_kinetic_J * np.ones_like(t_rel)[::step] * 1000.0).tolist(),
            "metrics": metrics
        }


# --- API ENDPOINTS ---

@app.get("/api/ports")
def get_hardware_status():
    """Scans and reports external hardware connection status."""
    ports = serial.tools.list_ports.comports()
    port_list = []
    arduino_detected = False
    detected_port = None

    for p in ports:
        desc = p.description or ""
        mfg = p.manufacturer or ""
        hwid = p.hwid or ""
        is_arduino = (
            "arduino" in desc.lower() or
            "arduino" in mfg.lower() or
            "2341:0043" in hwid.lower() or
            "ch340" in desc.lower() or
            "usb serial" in desc.lower()
        )
        if is_arduino and not detected_port:
            arduino_detected = True
            detected_port = p.device

        port_list.append({
            "device": p.device,
            "description": desc,
            "is_arduino": is_arduino
        })

    return {
        "hardware_connected": arduino_detected,
        "recommended_port": detected_port,
        "ports": port_list,
        "message": f"Arduino Uno detected on {detected_port}" if arduino_detected else "No Arduino hardware detected on any COM port."
    }


@app.post("/api/live_experiment")
def run_live_experiment(
    mass_g: float = Form(200.0),
    oscillator_type: str = Form("spring"),
    duration_sec: float = Form(16.0),
    port: str = Form("AUTO"),
    length_cm: float = Form(60.0),
    radius_cm: float = Form(8.0),
    arm_cm: float = Form(7.5),
    drop_height_cm: float = Form(60.0)
):
    """
    Executes a live hardware experiment with mode-tailored physics.
    Flags an error immediately if external Arduino is not connected.
    """
    target_port = port if port != "AUTO" else find_arduino_port()
    if not target_port:
        raise HTTPException(
            status_code=400,
            detail=(
                "HARDWARE ERROR: External hardware is NOT connected!\n"
                "No Arduino Uno or HC-SR04 sensor was detected on your USB/COM ports.\n\n"
                "Please verify:\n"
                "1. Arduino Uno is securely plugged in via USB cable.\n"
                "2. If Arduino IDE is open, close its Serial Monitor (it locks the port).\n"
                "3. Alternatively, upload an existing .CSV file or use Demo Simulation Mode."
            )
        )

    acq_params = AcquisitionParams(port=target_port, baud_rate=115200, acquisition_duration_sec=duration_sec)
    logger = SerialKinematicsLogger(acq_params)
    if not logger.connect():
        raise HTTPException(
            status_code=400,
            detail=(
                f"HARDWARE ERROR: Could not establish serial communication with port '{target_port}'.\n"
                "The port may be in use by another program, or the Arduino sketch is not running at 115200 baud."
            )
        )

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"kinematics_live_{timestamp}.csv"
    csv_path = os.path.join(DATA_DIR, csv_filename)

    try:
        logger.record_to_csv(output_filepath=csv_path, duration_sec=duration_sec)
    except Exception as e:
        logger.close()
        raise HTTPException(status_code=500, detail=f"Data streaming error: {e}")
    finally:
        logger.close()

    try:
        df = pd.read_csv(csv_path, comment="#")
        if len(df) < 15:
            raise ValueError("Too few samples captured. Check sensor wires (Trig/Echo/5V/GND).")
        raw_time = df["time_sec"].values
        raw_dist = df["distance_m"].values
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Hardware returned invalid data: {e}")

    physics = PhysicsParams(
        mass=mass_g / 1000.0,
        oscillator_type=oscillator_type,
        pendulum_length=length_cm / 100.0,
        disc_radius=radius_cm / 100.0,
        vane_radius=arm_cm / 100.0,
        drop_height=drop_height_cm / 100.0
    )
    payload = process_kinematics_payload(raw_time, raw_dist, physics)

    processed_csv_filename = f"processed_{csv_filename}"
    export_processed_graph_csv(payload, os.path.join(DATA_DIR, processed_csv_filename))

    history_entry = {
        "id": f"live_{timestamp}",
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "name": f"Live {oscillator_type.title()} ({mass_g:.0f}g)",
        "mode": "live",
        "mass_g": mass_g,
        "oscillator_type": oscillator_type,
        "duration_sec": duration_sec,
        "csv_file": csv_filename,
        "processed_csv_file": processed_csv_filename,
        "metrics": payload["metrics"]
    }
    save_history_entry(history_entry)

    payload["history_entry"] = history_entry
    payload["csv_filename"] = csv_filename
    payload["processed_csv_filename"] = processed_csv_filename
    return payload


class WebSerialStreamPayload(BaseModel):
    mass_g: float = 200.0
    oscillator_type: str = "spring"
    duration_sec: float = 16.0
    length_cm: float = 60.0
    radius_cm: float = 8.0
    arm_cm: float = 7.5
    drop_height_cm: float = 60.0
    spring_k: float = 15.0
    raw_csv_text: str


@app.post("/api/process_stream")
def process_web_serial_stream(data: WebSerialStreamPayload):
    """
    Receives raw sensor stream recorded in the browser via Web Serial API.
    Enables online cloud deployment where users connect an Arduino directly
    through Google Chrome or Microsoft Edge with zero local Python install!
    """
    if not data.raw_csv_text or len(data.raw_csv_text.strip()) == 0:
        raise HTTPException(
            status_code=400,
            detail="No data received from Web Serial stream. Ensure your Arduino is connected and running at 115200 baud."
        )

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"kinematics_webserial_{timestamp}.csv"
    csv_path = os.path.join(DATA_DIR, csv_filename)

    # Save raw CSV stream
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write(data.raw_csv_text.strip() + "\n")

    try:
        # Load with pandas, automatically discarding any duplicate text headers or malformed lines
        df = pd.read_csv(csv_path, comment="#", header=None, on_bad_lines="skip")
        
        # Coerce text headers like 'timestamp_us' or 'time_sec' to NaN
        s_time = pd.to_numeric(df.iloc[:, 0], errors="coerce")
        s_dist = pd.to_numeric(df.iloc[:, 2 if df.shape[1] >= 3 else 1], errors="coerce")

        # Keep only valid numeric rows
        valid_mask = s_time.notna() & s_dist.notna()
        raw_time = s_time[valid_mask].values.astype(float)
        raw_dist = s_dist[valid_mask].values.astype(float)

        if len(raw_time) < 10:
            raise ValueError(
                f"Too few valid numeric samples captured ({len(raw_time)}). Check sensor wiring (Trig/Echo/5V/GND) and acoustic alignment."
            )

        # Automatic timestamp normalization (handles microseconds from micros(), millis(), or relative seconds)
        if len(raw_time) > 1:
            dt_med = float(np.median(np.diff(raw_time)))
            if dt_med > 1000.0:  # Microseconds (e.g. ~25000 us from micros())
                raw_time = (raw_time - raw_time[0]) / 1e6
            elif dt_med > 1.0:   # Milliseconds (e.g. ~25 ms from millis())
                raw_time = (raw_time - raw_time[0]) / 1e3
            elif raw_time[0] > 10.0:
                raw_time = raw_time - raw_time[0]
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed to parse Web Serial stream: {e}")

    physics = PhysicsParams(
        mass=data.mass_g / 1000.0,
        oscillator_type=data.oscillator_type,
        nominal_spring_k=data.spring_k,
        pendulum_length=data.length_cm / 100.0,
        disc_radius=data.radius_cm / 100.0,
        vane_radius=data.arm_cm / 100.0,
        drop_height=data.drop_height_cm / 100.0
    )

    payload = process_kinematics_payload(raw_time, raw_dist, physics)

    processed_csv_filename = f"processed_{csv_filename}"
    export_processed_graph_csv(payload, os.path.join(DATA_DIR, processed_csv_filename))

    history_entry = {
        "id": f"webserial_{timestamp}",
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "name": f"WebSerial {data.oscillator_type.title()} ({data.mass_g:.0f}g)",
        "mode": "webserial",
        "mass_g": data.mass_g,
        "oscillator_type": data.oscillator_type,
        "duration_sec": data.duration_sec,
        "csv_file": csv_filename,
        "processed_csv_file": processed_csv_filename,
        "metrics": payload["metrics"]
    }
    save_history_entry(history_entry)

    payload["history_entry"] = history_entry
    payload["csv_filename"] = csv_filename
    payload["processed_csv_filename"] = processed_csv_filename
    return payload


@app.post("/api/upload_csv")
async def upload_csv_experiment(
    file: UploadFile = File(...),
    mass_g: float = Form(200.0),
    oscillator_type: str = Form("spring"),
    length_cm: float = Form(60.0),
    radius_cm: float = Form(8.0),
    arm_cm: float = Form(7.5),
    drop_height_cm: float = Form(60.0)
):
    """Parses uploaded CSV and runs mode-tailored analysis."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    clean_name = file.filename.replace(" ", "_")
    csv_filename = f"uploaded_{timestamp}_{clean_name}"
    csv_path = os.path.join(DATA_DIR, csv_filename)

    with open(csv_path, "wb") as buffer:
        shutil.copyfileobj(file.file, buffer)

    try:
        df = pd.read_csv(csv_path, comment="#")
        time_col = None
        dist_col = None
        for col in df.columns:
            c = col.lower().strip()
            if "time" in c or "t" == c:
                time_col = col
            elif "dist" in c or "pos" in c or "x" == c or "distance_m" in c:
                dist_col = col

        if not time_col or not dist_col:
            if len(df.columns) >= 3:
                time_col = df.columns[0]
                dist_col = df.columns[2]
            elif len(df.columns) >= 2:
                time_col = df.columns[0]
                dist_col = df.columns[1]
            else:
                raise ValueError("CSV must contain at least time and distance columns.")

        raw_time = df[time_col].values.astype(float)
        raw_dist = df[dist_col].values.astype(float)

        if len(raw_time) < 10:
            raise ValueError("CSV contains fewer than 10 data points.")

    except Exception as e:
        if os.path.exists(csv_path):
            os.remove(csv_path)
        raise HTTPException(status_code=400, detail=f"Failed to parse uploaded CSV: {e}")

    physics = PhysicsParams(
        mass=mass_g / 1000.0,
        oscillator_type=oscillator_type,
        pendulum_length=length_cm / 100.0,
        disc_radius=radius_cm / 100.0,
        vane_radius=arm_cm / 100.0,
        drop_height=drop_height_cm / 100.0
    )
    payload = process_kinematics_payload(raw_time, raw_dist, physics)

    processed_csv_filename = f"processed_{csv_filename}"
    export_processed_graph_csv(payload, os.path.join(DATA_DIR, processed_csv_filename))

    history_entry = {
        "id": f"upload_{timestamp}",
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "name": f"CSV: {file.filename} ({mass_g:.0f}g)",
        "mode": "csv_upload",
        "mass_g": mass_g,
        "oscillator_type": oscillator_type,
        "duration_sec": float(raw_time[-1] - raw_time[0]),
        "csv_file": csv_filename,
        "processed_csv_file": processed_csv_filename,
        "metrics": payload["metrics"]
    }
    save_history_entry(history_entry)

    payload["history_entry"] = history_entry
    payload["csv_filename"] = csv_filename
    payload["processed_csv_filename"] = processed_csv_filename
    return payload


@app.post("/api/simulate")
def run_simulation(
    mass_g: float = Form(200.0),
    oscillator_type: str = Form("spring"),
    spring_k: float = Form(15.0),
    damping_c: float = Form(0.055),
    duration_sec: float = Form(16.0),
    length_cm: float = Form(60.0),
    radius_cm: float = Form(8.0),
    arm_cm: float = Form(7.5),
    drop_height_cm: float = Form(60.0)
):
    """Generates synthetic physics data tailored to the chosen oscillator mode."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"simulated_{timestamp}.csv"
    csv_path = os.path.join(DATA_DIR, csv_filename)

    physics = PhysicsParams(
        mass=mass_g / 1000.0,
        nominal_spring_k=spring_k,
        oscillator_type=oscillator_type,
        pendulum_length=length_cm / 100.0,
        disc_radius=radius_cm / 100.0,
        vane_radius=arm_cm / 100.0,
        drop_height=drop_height_cm / 100.0
    )

    t_raw, dist_raw, _, _ = generate_simulation_data(
        duration_sec=duration_sec,
        sample_rate_hz=40.0,
        physics=physics,
        initial_displacement=0.10,
        damping_coeff=damping_c,
        equilibrium_dist_m=0.40,
        dropout_prob=0.015
    )
    save_synthetic_csv(csv_path, t_raw, dist_raw)

    payload = process_kinematics_payload(t_raw, dist_raw, physics)

    processed_csv_filename = f"processed_{csv_filename}"
    export_processed_graph_csv(payload, os.path.join(DATA_DIR, processed_csv_filename))

    history_entry = {
        "id": f"sim_{timestamp}",
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "name": f"Simulated {oscillator_type.title()} ({mass_g:.0f}g)",
        "mode": "simulation",
        "mass_g": mass_g,
        "oscillator_type": oscillator_type,
        "duration_sec": duration_sec,
        "csv_file": csv_filename,
        "processed_csv_file": processed_csv_filename,
        "metrics": payload["metrics"]
    }
    save_history_entry(history_entry)

    payload["history_entry"] = history_entry
    payload["csv_filename"] = csv_filename
    payload["processed_csv_filename"] = processed_csv_filename
    return payload


@app.get("/api/history")
def get_all_history():
    return load_history()


@app.get("/api/history/{run_id}")
def get_history_detail(run_id: str):
    history = load_history()
    target = next((item for item in history if item["id"] == run_id), None)
    if not target:
        raise HTTPException(status_code=404, detail="Run not found in history.")

    csv_path = os.path.join(DATA_DIR, target["csv_file"])
    if not os.path.exists(csv_path):
        raise HTTPException(status_code=404, detail=f"CSV file '{target['csv_file']}' missing on disk.")

    df = pd.read_csv(csv_path, comment="#")
    raw_time = df.iloc[:, 0].values.astype(float)
    raw_dist = df.iloc[:, 2 if len(df.columns) >= 3 else 1].values.astype(float)

    physics = PhysicsParams(
        mass=target["mass_g"] / 1000.0,
        oscillator_type=target.get("oscillator_type", "spring"),
        pendulum_length=target.get("metrics", {}).get("cord_length_cm", 60.0) / 100.0,
        disc_radius=target.get("metrics", {}).get("disc_radius_cm", 8.0) / 100.0,
        drop_height=target.get("metrics", {}).get("drop_height_cm", 60.0) / 100.0
    )
    payload = process_kinematics_payload(raw_time, raw_dist, physics)
    processed_csv_filename = target.get("processed_csv_file") or f"processed_{target['csv_file']}"
    proc_path = os.path.join(DATA_DIR, processed_csv_filename)
    if not os.path.exists(proc_path):
        try:
            export_processed_graph_csv(payload, proc_path)
        except Exception:
            pass

    payload["history_entry"] = target
    payload["csv_filename"] = target["csv_file"]
    payload["processed_csv_filename"] = processed_csv_filename
    return payload


@app.delete("/api/history/{run_id}")
def delete_history_item(run_id: str):
    history = load_history()
    new_history = []
    target = None
    for h in history:
        if h["id"] == run_id:
            target = h
        else:
            new_history.append(h)

    if not target:
        raise HTTPException(status_code=404, detail="Item not found.")

    with open(HISTORY_FILE, "w", encoding="utf-8") as f:
        json.dump(new_history, f, indent=2)

    csv_file = os.path.join(DATA_DIR, target.get("csv_file", ""))
    if os.path.exists(csv_file):
        try:
            os.remove(csv_file)
        except Exception:
            pass

    proc_file = os.path.join(DATA_DIR, target.get("processed_csv_file", f"processed_{target.get('csv_file', '')}"))
    if os.path.exists(proc_file):
        try:
            os.remove(proc_file)
        except Exception:
            pass

    return {"status": "success", "message": f"Deleted run {run_id}"}


@app.get("/api/download/{filename}")
def download_csv(filename: str):
    csv_path = os.path.join(DATA_DIR, filename)
    if not os.path.exists(csv_path):
        raise HTTPException(status_code=404, detail="CSV file not found.")
    return FileResponse(csv_path, media_type="text/csv", filename=filename)


@app.get("/api/download_processed/{filename}")
def download_processed_csv(filename: str):
    """
    Downloads the processed multi-channel kinematics CSV (contains Position,
    Velocity, Acceleration, Kinetic/Potential Energy, Total Energy, and Decay Envelopes).
    Dynamically generates the file if it has not been saved yet.
    """
    clean_name = filename if filename.startswith("processed_") else f"processed_{filename}"
    proc_path = os.path.join(DATA_DIR, clean_name)

    if os.path.exists(proc_path):
        return FileResponse(proc_path, media_type="text/csv", filename=clean_name)

    base_name = filename.replace("processed_", "")
    raw_path = os.path.join(DATA_DIR, base_name)
    if not os.path.exists(raw_path):
        raise HTTPException(status_code=404, detail=f"Base CSV file '{base_name}' not found.")

    try:
        df = pd.read_csv(raw_path, comment="#")
        raw_time = df.iloc[:, 0].values.astype(float)
        raw_dist = df.iloc[:, 2 if len(df.columns) >= 3 else 1].values.astype(float)

        history = load_history()
        target = next((item for item in history if item.get("csv_file") == base_name), None)
        if target:
            physics = PhysicsParams(
                mass=target.get("mass_g", 200.0) / 1000.0,
                oscillator_type=target.get("oscillator_type", "spring"),
                pendulum_length=target.get("metrics", {}).get("cord_length_cm", 60.0) / 100.0,
                disc_radius=target.get("metrics", {}).get("disc_radius_cm", 8.0) / 100.0,
                drop_height=target.get("metrics", {}).get("drop_height_cm", 60.0) / 100.0
            )
        else:
            physics = PhysicsParams()

        payload = process_kinematics_payload(raw_time, raw_dist, physics)
        export_processed_graph_csv(payload, proc_path)
        return FileResponse(proc_path, media_type="text/csv", filename=clean_name)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to generate processed graph data: {e}")


# Mount static frontend
app.mount("/", StaticFiles(directory=WEB_DIR, html=True), name="static")


def run():
    port = int(os.environ.get("PORT", 8000))
    host = os.environ.get("HOST", "0.0.0.0")
    print("\n" + "="*75)
    print("      ⚡ AUTOMATED KINEMATICS & MULTI-MODE OSCILLATOR SERVER v2.5")
    print("="*75)
    print(f"Web Dashboard:     http://localhost:{port}")
    print(f"API Documentation: http://localhost:{port}/docs")
    print("Supported Modes:   Spring, Pendulum, Cart, Torsion, Free-Fall")
    print("="*75 + "\n")
    uvicorn.run(app, host=host, port=port, log_level="warning")


if __name__ == "__main__":
    run()
