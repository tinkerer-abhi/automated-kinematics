"""
Synthetic Sensor Data Generator & Physics Engine for All Oscillator Modes
Automated Kinematics and Damped Harmonic Motion Analysis

Supports:
1. 'spring'   : Vertical Spring-Mass Oscillator
2. 'pendulum' : Simple Pendulum (Angular small-angle arc)
3. 'cart'     : Horizontal Dynamics Cart (Opposing springs k1 + k2)
4. 'torsion'  : Torsional Oscillator (Rotating disc on wire)
5. 'freefall' : Free-Fall Drop Column (g acceleration)
"""

import os
from typing import Tuple
import numpy as np
from scipy.integrate import solve_ivp

from config import PhysicsParams, DEFAULT_PHYSICS


def generate_simulation_data(
    duration_sec: float = 16.0,
    sample_rate_hz: float = 40.0,
    physics: PhysicsParams = DEFAULT_PHYSICS,
    initial_displacement: float = 0.10,
    damping_coeff: float = 0.055,
    equilibrium_dist_m: float = 0.40,
    dropout_prob: float = 0.02
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Generates synthetic HC-SR04 ultrasonic distance data for any of the 5 oscillator types.
    Returns: (t_raw, dist_raw, t_true, dist_true)
    """
    mode = physics.oscillator_type
    m = physics.mass

    t_eval = np.linspace(0, duration_sec, int(duration_sec * sample_rate_hz))

    if mode == 'spring':
        k = physics.nominal_spring_k
        c = damping_coeff
        def ode(t, y):
            x, v = y
            return [v, -(k / m) * x - (c / m) * v]
        sol = solve_ivp(ode, [0, duration_sec], [initial_displacement, 0.0], t_eval=t_eval, rtol=1e-8)
        displacement_true = sol.y[0]
        distance_true = equilibrium_dist_m + displacement_true

    elif mode == 'pendulum':
        L = max(0.1, physics.pendulum_length)
        g = physics.gravity
        c_pend = damping_coeff
        # d^2 theta / dt^2 + (c/m)*dtheta/dt + (g/L)*sin(theta) = 0
        # initial_displacement in degrees or radians:
        theta_0 = initial_displacement if initial_displacement < 0.5 else 0.18 # ~10 degrees
        def ode_pend(t, y):
            theta, omega = y
            return [omega, -(g / L) * np.sin(theta) - (c_pend / (m * L)) * omega]
        sol = solve_ivp(ode_pend, [0, duration_sec], [theta_0, 0.0], t_eval=t_eval, rtol=1e-8)
        # Measured linear arc chord distance from horizontal sensor:
        theta_arr = sol.y[0]
        # Linear chord s = L * sin(theta)
        displacement_true = L * np.sin(theta_arr)
        distance_true = equilibrium_dist_m + displacement_true

    elif mode == 'cart':
        # Dual springs: k_total = k1 + k2
        k_total = physics.nominal_spring_k * 2.0
        c_cart = damping_coeff
        def ode_cart(t, y):
            x, v = y
            return [v, -(k_total / m) * x - (c_cart / m) * v]
        sol = solve_ivp(ode_cart, [0, duration_sec], [initial_displacement, 0.0], t_eval=t_eval, rtol=1e-8)
        displacement_true = sol.y[0]
        distance_true = equilibrium_dist_m + displacement_true

    elif mode == 'torsion':
        R = max(0.02, physics.disc_radius)
        r_arm = max(0.02, physics.vane_radius)
        I = 0.5 * m * (R ** 2)
        kappa = physics.nominal_spring_k * (r_arm ** 2) # Torsion constant (N*m/rad)
        c_torsion = damping_coeff * 0.01
        theta_0 = initial_displacement / r_arm # radians
        def ode_torsion(t, y):
            theta, omega = y
            return [omega, -(kappa / I) * theta - (c_torsion / I) * omega]
        sol = solve_ivp(ode_torsion, [0, duration_sec], [theta_0, 0.0], t_eval=t_eval, rtol=1e-8)
        theta_arr = sol.y[0]
        displacement_true = r_arm * theta_arr
        distance_true = equilibrium_dist_m + displacement_true

    elif mode == 'freefall':
        # Single free-fall drop: settle at top → release → fall under gravity → rest at bottom
        g = physics.gravity
        h_max = max(0.2, physics.drop_height)
        t_hit = np.sqrt(2.0 * h_max / g)  # theoretical fall time

        t_settle = 0.12   # stationary period before release (120 ms)
        t_cushion = 0.18  # rest at bottom after impact

        distance_true = np.zeros_like(t_eval)
        for i, tv in enumerate(t_eval):
            if tv < t_settle:
                # Stationary at top
                distance_true[i] = equilibrium_dist_m
            elif tv <= t_settle + t_hit:
                # Active free fall: d = d0 + 0.5 * g * (t - t_release)^2
                t_rel = tv - t_settle
                distance_true[i] = equilibrium_dist_m + 0.5 * g * (t_rel ** 2)
            else:
                # Resting on cushion at bottom
                distance_true[i] = equilibrium_dist_m + h_max

        displacement_true = distance_true - equilibrium_dist_m
        sol = type('obj', (object,), {'t': t_eval, 'y': [displacement_true]})()

    t_true = sol.t

    # Add realistic timing jitter (HC-SR04 pulse timing varies slightly ~1.2ms)
    dt_jitter = np.random.normal(0, 0.0012, size=len(t_true))
    t_raw = np.maximum(0, t_true + dt_jitter)

    # Additive Gaussian distance noise (~1.0 mm)
    dist_noise = np.random.normal(0, 0.0010, size=len(distance_true))

    # Discrete ultrasonic quantization (~0.003 m)
    dist_quantized = np.round((distance_true + dist_noise) / 0.003) * 0.003

    # Add random sensor dropouts (3% echo miss)
    dist_raw = dist_quantized.copy()
    num_samples = len(dist_raw)
    dropout_indices = np.random.choice(num_samples, size=int(num_samples * dropout_prob), replace=False)

    for idx in dropout_indices:
        if np.random.rand() > 0.5:
            dist_raw[idx] = -1.0  # Ultrasonic timeout flag
        else:
            dist_raw[idx] = np.random.uniform(0.9, 1.8)

    return t_raw, dist_raw, t_true, distance_true


# Alias for backward compatibility
generate_damped_spring_data = generate_simulation_data


def save_synthetic_csv(
    filepath: str,
    t_raw: np.ndarray,
    dist_raw: np.ndarray,
    sensor_id: int = 1
):
    """Saves synthetic stream formatted identically to the Arduino CSV output."""
    os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
    with open(filepath, "w", encoding="utf-8") as f:
        f.write("# Automated Kinematics Tracking System - Synthetic Simulation Run\n")
        f.write("time_sec,sensor_id,distance_m,raw_pulse_us\n")
        for t, d in zip(t_raw, dist_raw):
            pulse_us = int((2.0 * max(0.0, d) / 343.2) * 1e6) if d > 0 else 0
            f.write(f"{t:.6f},{sensor_id},{d:.5f},{pulse_us}\n")
