"""
Damped Harmonic Motion Fitting & Energy Conservation Engine
Automated Kinematics and Damped Harmonic Motion Analysis

Implements:
1. Underdamped oscillator model fitting: x(t) = A0 * exp(-gamma * t) * cos(omega_d * t + phi) + x_eq
2. Exact calculation of damping ratio (zeta), damping coefficient (c), quality factor (Q),
   natural frequency (omega_0), and spring constant (k).
3. Logarithmic decrement analysis from peak extrema.
4. Continuous mechanical energy tracking:
   - Kinetic Energy: K(t) = 0.5 * m * v(t)^2
   - Potential Energy: U(t) = 0.5 * k * (x(t) - x_eq)^2
   - Total Energy: E(t) = K(t) + U(t)
   - Verification of dissipation rate: dE/dt = -c * v^2
"""

from typing import Dict, Any, Tuple
import numpy as np
from scipy import optimize, signal

from config import PhysicsParams, DEFAULT_PHYSICS


def damped_oscillator_model(t: np.ndarray, A0: float, gamma: float, omega_d: float, phi: float, x_eq: float) -> np.ndarray:
    """Analytical solution for an underdamped harmonic oscillator."""
    return A0 * np.exp(-gamma * t) * np.cos(omega_d * t + phi) + x_eq


class HarmonicFitEngine:
    def __init__(self, physics_params: PhysicsParams = DEFAULT_PHYSICS):
        self.physics = physics_params

    def estimate_initial_priors(self, t: np.ndarray, x: np.ndarray) -> Tuple[float, float, list, float, float]:
        """
        Derives robust, physically-bounded candidate priors for non-linear curve fitting.
        Combines:
        1. Detrended + Hann-windowed FFT (rejects low-frequency DC spectral leakage).
        2. Time-domain peak extrema detection (find_peaks on smoothed trajectory).
        3. Theoretical nominal frequency prior based on rig configuration.
        """
        x_eq_guess = float(np.median(x))
        x_centered = x - x_eq_guess
        ptp = float(np.max(x) - np.min(x))

        # Amplitude guess from 95th percentile to reject spurious ultrasonic spikes
        A0_guess = float(np.percentile(np.abs(x_centered), 95) * 1.15)
        A0_guess = max(0.005, min(A0_guess, max(0.01, ptp / 2.0)))

        dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.025
        n = len(t)
        candidate_omegas = []

        # --- Method 1: Detrended Hann-Windowed FFT ---
        if n >= 16:
            try:
                x_detrend = signal.detrend(x_centered)
                window = signal.windows.hann(n)
                fft_vals = np.abs(np.fft.rfft(x_detrend * window))
                fft_freqs = np.fft.rfftfreq(n, d=dt)

                # Restrict to realistic physical oscillator frequencies (0.25 Hz to 7.5 Hz)
                phys_mask = (fft_freqs >= 0.25) & (fft_freqs <= 7.5)
                if np.any(phys_mask):
                    peak_phys_idx = np.where(phys_mask)[0][np.argmax(fft_vals[phys_mask])]
                    f_fft = float(fft_freqs[peak_phys_idx])
                    if f_fft > 0:
                        candidate_omegas.append(float(2.0 * np.pi * f_fft))
            except Exception:
                pass

        # --- Method 2: Time-Domain Peak Extrema Detection ---
        if n >= 20 and dt > 0:
            try:
                # Smooth slightly with Savitzky-Golay to avoid ultrasonic jitter
                w_len = max(5, int(0.08 / dt))
                if w_len % 2 == 0:
                    w_len += 1
                if w_len <= n:
                    x_smooth = signal.savgol_filter(x_centered, w_len, min(2, w_len - 1))
                else:
                    x_smooth = x_centered

                min_dist = max(2, int(0.18 / dt))  # At least 0.18s between peaks (~5.5 Hz max)
                prom = max(0.003, ptp * 0.10)
                peaks_pos, _ = signal.find_peaks(x_smooth, distance=min_dist, prominence=prom)
                peaks_neg, _ = signal.find_peaks(-x_smooth, distance=min_dist, prominence=prom)

                if len(peaks_pos) >= 2:
                    p_pos = float(np.median(np.diff(t[peaks_pos])))
                    if 0.12 <= p_pos <= 4.0:
                        candidate_omegas.append(float(2.0 * np.pi / p_pos))

                if len(peaks_neg) >= 2:
                    p_neg = float(np.median(np.diff(t[peaks_neg])))
                    if 0.12 <= p_neg <= 4.0:
                        candidate_omegas.append(float(2.0 * np.pi / p_neg))
            except Exception:
                pass

        # --- Method 3: Theoretical Prior based on Rig Mode ---
        m = max(0.01, self.physics.mass)
        if self.physics.oscillator_type == 'spring' and self.physics.nominal_spring_k > 0:
            candidate_omegas.append(float(np.sqrt(self.physics.nominal_spring_k / m)))
        elif self.physics.oscillator_type == 'cart' and self.physics.nominal_spring_k > 0:
            candidate_omegas.append(float(np.sqrt(2.0 * self.physics.nominal_spring_k / m)))
        elif self.physics.oscillator_type == 'pendulum' and self.physics.pendulum_length > 0:
            candidate_omegas.append(float(np.sqrt(self.physics.gravity / max(0.1, self.physics.pendulum_length))))
        elif self.physics.oscillator_type == 'torsion':
            R = max(0.02, self.physics.disc_radius)
            I = 0.5 * m * (R ** 2)
            # Typical wire kappa ~0.05 - 0.25 N*m/rad
            candidate_omegas.append(float(np.sqrt(0.12 / I)))

        # Fallback if no valid candidates found
        if not candidate_omegas:
            candidate_omegas = [float(2.0 * np.pi * 1.35)]

        # Estimate gamma from decay envelope
        t_span = float(t[-1] - t[0]) if len(t) > 1 else 1.0
        gamma_guess = 0.05
        if t_span > 2.0:
            w_size = max(5, int(n * 0.18))
            head_amp = float(np.std(x_centered[:w_size]) * np.sqrt(2))
            tail_amp = float(np.std(x_centered[-w_size:]) * np.sqrt(2))
            if head_amp > tail_amp > 1e-4:
                gamma_guess = float(-np.log(max(1e-4, tail_amp) / head_amp) / (t_span * 0.8))
                gamma_guess = max(0.001, min(gamma_guess, 4.0))

        phi_guess = 0.0

        return A0_guess, gamma_guess, candidate_omegas, phi_guess, x_eq_guess

    def fit_damped_motion(self, t: np.ndarray, x: np.ndarray) -> Dict[str, Any]:
        """
        Fits the underdamped motion model to position data using Multi-Start Non-Linear Least Squares.
        Tests all physical frequency candidates to avoid local minima or boundary traps.
        """
        t0 = t[0]
        t_rel = t - t0
        ptp = float(np.max(x) - np.min(x))

        A0_g, gamma_g, omegas, phi_g, x_eq_g = self.estimate_initial_priors(t_rel, x)

        # Realistic physical parameter bounds:
        # omega_d between 0.3 rad/s (T = 20.9 s) and 50 rad/s (~8 Hz)
        bounds = (
            [-np.inf, 0.0, 0.30, -2.0 * np.pi, -np.inf],
            [np.inf, 25.0, 50.0, 2.0 * np.pi, np.inf]
        )

        best_popt = None
        best_pcov = None
        best_ss_res = np.inf
        ss_tot = float(np.sum((x - np.mean(x)) ** 2))

        # Evaluate unique candidate frequencies to find global minimum
        unique_omegas = sorted(list({round(w, 3): w for w in omegas}.values()))
        for omega_cand in unique_omegas:
            p0 = [A0_g, gamma_g, omega_cand, phi_g, x_eq_g]
            try:
                popt, pcov = optimize.curve_fit(
                    damped_oscillator_model,
                    t_rel,
                    x,
                    p0=p0,
                    bounds=bounds,
                    maxfev=6000
                )
                fit_eval = damped_oscillator_model(t_rel, *popt)
                ss_res = float(np.sum((x - fit_eval) ** 2))
                if ss_res < best_ss_res:
                    best_ss_res = ss_res
                    best_popt = popt
                    best_pcov = pcov
            except Exception:
                continue

        if best_popt is not None:
            popt = best_popt
            try:
                perr = np.sqrt(np.diag(best_pcov)) if best_pcov is not None else np.zeros(5)
            except Exception:
                perr = np.zeros(5)
        else:
            popt = [A0_g, gamma_g, unique_omegas[0], phi_g, x_eq_g]
            perr = np.zeros(5)

        A0, gamma, omega_d, phi, x_eq = popt
        A0_err, gamma_err, omega_d_err, phi_err, x_eq_err = perr

        # Calculate physical oscillator parameters
        omega_0 = float(np.sqrt(omega_d**2 + gamma**2))
        natural_freq_hz = float(omega_0 / (2.0 * np.pi))
        damped_freq_hz = float(omega_d / (2.0 * np.pi))
        damped_period_sec = float((2.0 * np.pi) / omega_d) if omega_d > 0 else np.nan

        # Damping ratio: zeta = gamma / omega_0
        damping_ratio_zeta = float(gamma / omega_0) if omega_0 > 0 else 0.0

        # Quality factor: Q = 1 / (2 * zeta) = omega_0 / (2 * gamma)
        quality_factor_Q = float(omega_0 / (2.0 * gamma)) if gamma > 1e-6 else 999.0

        # Viscous damping coefficient c (N*s/m): c = 2 * m * gamma
        damping_coeff_c = float(2.0 * self.physics.mass * gamma)

        # Mode-tailored dynamic stiffness:
        m = self.physics.mass
        if self.physics.oscillator_type == 'spring':
            k_effective = float(m * (omega_0 ** 2))
        elif self.physics.oscillator_type == 'cart':
            k_effective = float(m * (omega_0 ** 2))
        elif self.physics.oscillator_type == 'pendulum':
            L = max(0.05, self.physics.pendulum_length)
            k_effective = float(m * self.physics.gravity / L)
        elif self.physics.oscillator_type == 'torsion':
            R = max(0.02, self.physics.disc_radius)
            I = 0.5 * m * (R ** 2)
            k_effective = float(I * (omega_0 ** 2))
        else:
            k_effective = float(m * (omega_0 ** 2))

        # Fitted curve & envelope
        fitted_curve = damped_oscillator_model(t_rel, *popt)
        upper_envelope = x_eq + np.abs(A0) * np.exp(-gamma * t_rel)
        lower_envelope = x_eq - np.abs(A0) * np.exp(-gamma * t_rel)

        # Goodness of fit (R^2)
        ss_res = float(np.sum((x - fitted_curve) ** 2))
        r_squared = float(1.0 - (ss_res / ss_tot)) if ss_tot > 1e-9 else 0.0
        r_squared = max(0.0, min(1.0, r_squared))

        # Amplitude check flag
        is_oscillating = ptp >= 0.015 and r_squared >= 0.15

        return {
            "popt": popt,
            "perr": perr,
            "A0": float(A0),
            "A0_err": float(A0_err),
            "gamma": float(gamma),
            "gamma_err": float(gamma_err),
            "omega_d": float(omega_d),
            "omega_d_err": float(omega_d_err),
            "phi": float(phi),
            "x_eq": float(x_eq),
            "x_eq_err": float(x_eq_err),
            "omega_0": omega_0,
            "natural_freq_hz": natural_freq_hz,
            "damped_freq_hz": damped_freq_hz,
            "damped_period_sec": damped_period_sec,
            "damping_ratio_zeta": damping_ratio_zeta,
            "quality_factor_Q": quality_factor_Q,
            "damping_coeff_c": damping_coeff_c,
            "k_effective": k_effective,
            "r_squared": r_squared,
            "is_oscillating": is_oscillating,
            "ptp_amplitude_m": ptp,
            "fitted_curve": fitted_curve,
            "upper_envelope": upper_envelope,
            "lower_envelope": lower_envelope,
            "t_rel": t_rel
        }

    def compute_logarithmic_decrement(self, t: np.ndarray, x: np.ndarray, x_eq: float) -> Dict[str, Any]:
        """
        Calculates the logarithmic decrement (delta) directly from observed local peaks:
        delta = (1 / n) * ln(x_0 / x_n)
        zeta = delta / sqrt(4 * pi^2 + delta^2)
        """
        x_dev = x - x_eq
        dt = t[1] - t[0]
        # Min peak distance ~0.3 sec to avoid noise peaks
        min_distance = max(1, int(0.3 / dt))

        peak_indices, _ = signal.find_peaks(x_dev, distance=min_distance, prominence=0.005)

        if len(peak_indices) < 2:
            return {"log_decrement": np.nan, "zeta_from_peaks": np.nan, "num_peaks": len(peak_indices)}

        peak_times = t[peak_indices]
        peak_amps = x_dev[peak_indices]

        # Filter strictly positive peaks
        pos_mask = peak_amps > 1e-4
        peak_times = peak_times[pos_mask]
        peak_amps = peak_amps[pos_mask]

        if len(peak_amps) < 2:
            return {"log_decrement": np.nan, "zeta_from_peaks": np.nan, "num_peaks": len(peak_amps)}

        # Fit linear slope to ln(peak_amp) vs peak_times: slope = -gamma
        ln_amps = np.log(peak_amps)
        slope, intercept = np.polyfit(peak_times, ln_amps, deg=1)
        fitted_gamma = -slope

        # Successive peak decrement
        decrements = np.diff(ln_amps) * -1.0
        mean_delta = float(np.mean(decrements))

        # Damping ratio from delta
        zeta_from_delta = float(mean_delta / np.sqrt(4.0 * np.pi**2 + mean_delta**2))

        return {
            "peak_times": peak_times,
            "peak_amps": peak_amps,
            "log_decrement": mean_delta,
            "gamma_from_peaks": float(fitted_gamma),
            "zeta_from_peaks": zeta_from_delta,
            "num_peaks": len(peak_amps)
        }

    def calculate_energy(
        self,
        t: np.ndarray,
        x: np.ndarray,
        v: np.ndarray,
        k: float,
        x_eq: float,
        gamma: float
    ) -> Dict[str, np.ndarray]:
        """
        Computes instantaneous mechanical energy components:
        - Kinetic Energy: T = 0.5 * m * v^2
        - Potential Energy: V = 0.5 * k * (x - x_eq)^2
        - Total Energy: E_tot = T + V
        - Theoretical Exponential Dissipation Envelope: E_theory(t) = E_0 * exp(-2 * gamma * t)
        """
        m = self.physics.mass
        kinetic_energy = 0.5 * m * (v**2)

        displacement = x - x_eq
        if self.physics.oscillator_type == 'spring' or self.physics.oscillator_type == 'cart':
            potential_energy = 0.5 * k * (displacement**2)
        elif self.physics.oscillator_type == 'pendulum':
            # Small angle pendulum: V = m * g * L * (1 - cos(theta)) ≈ 0.5 * (m*g/L) * s^2
            k_pendulum = (m * self.physics.gravity) / max(0.05, self.physics.pendulum_length)
            potential_energy = 0.5 * k_pendulum * (displacement**2)
        elif self.physics.oscillator_type == 'torsion':
            # Torsional disc: linear displacement x = r * theta -> theta = x / r
            r_arm = max(0.01, self.physics.vane_radius)
            theta_rad = displacement / r_arm
            # k is effective linear stiffness: k_linear = kappa / r^2 -> kappa = k * r^2
            kappa = k * (r_arm**2)
            potential_energy = 0.5 * kappa * (theta_rad**2)
        else:
            potential_energy = np.zeros_like(displacement)

        total_energy = kinetic_energy + potential_energy

        # Initial smoothed total energy
        # To avoid noise at t=0, average over first 10% of cycle
        e0 = float(np.percentile(total_energy[:max(5, len(total_energy)//10)], 90))
        t_rel = t - t[0]
        theoretical_energy_decay = e0 * np.exp(-2.0 * gamma * t_rel)

        # Damping dissipation rate: dE/dt = -c * v^2
        c = 2.0 * m * gamma
        instantaneous_dissipation_power = -c * (v**2)

        return {
            "kinetic": kinetic_energy,
            "potential": potential_energy,
            "total": total_energy,
            "theoretical_decay": theoretical_energy_decay,
            "dissipation_power": instantaneous_dissipation_power,
            "E0": e0
        }
