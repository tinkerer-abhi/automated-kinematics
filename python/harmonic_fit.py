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

    def estimate_initial_priors(self, t: np.ndarray, x: np.ndarray) -> Tuple[float, float, float, float, float]:
        """Derives robust initial guesses for the non-linear curve fitting."""
        x_eq_guess = float(np.median(x))
        x_centered = x - x_eq_guess

        # Amplitude guess
        A0_guess = float((np.max(x) - np.min(x)) / 2.0)

        # Estimate frequency using FFT
        dt = float(t[1] - t[0])
        n = len(t)
        fft_vals = np.abs(np.fft.rfft(x_centered))
        fft_freqs = np.fft.rfftfreq(n, d=dt)

        # Ignore DC component
        if len(fft_vals) > 2:
            peak_idx = np.argmax(fft_vals[1:]) + 1
            freq_hz = float(fft_freqs[peak_idx])
            omega_d_guess = float(2.0 * np.pi * freq_hz)
        else:
            omega_d_guess = 6.28  # ~1 Hz fallback

        # Estimate gamma from initial vs ending envelope
        # A_end ~ A0 * exp(-gamma * T)
        t_span = float(t[-1] - t[0])
        gamma_guess = 0.1
        if t_span > 2.0:
            window_size = max(5, int(len(x) * 0.15))
            head_amp = np.std(x_centered[:window_size]) * np.sqrt(2)
            tail_amp = np.std(x_centered[-window_size:]) * np.sqrt(2)
            if head_amp > tail_amp > 1e-4:
                gamma_guess = float(-np.log(tail_amp / head_amp) / t_span)
                gamma_guess = max(0.001, min(gamma_guess, 5.0))

        phi_guess = 0.0

        return A0_guess, gamma_guess, omega_d_guess, phi_guess, x_eq_guess

    def fit_damped_motion(self, t: np.ndarray, x: np.ndarray) -> Dict[str, Any]:
        """
        Fits the underdamped motion model to position data using Non-Linear Least Squares.
        Extracts key physical damping properties and uncertainties.
        """
        # Zero-center time for numerical stability during fitting
        t0 = t[0]
        t_rel = t - t0

        p0 = self.estimate_initial_priors(t_rel, x)

        bounds = (
            [-np.inf, 0.0, 0.05, -2.0 * np.pi, -np.inf],   # Lower bounds (gamma >= 0, omega_d > 0)
            [np.inf, 50.0, 100.0, 2.0 * np.pi, np.inf]     # Upper bounds
        )

        try:
            popt, pcov = optimize.curve_fit(
                damped_oscillator_model,
                t_rel,
                x,
                p0=p0,
                bounds=bounds,
                maxfev=10000
            )
            perr = np.sqrt(np.diag(pcov))
        except Exception as e:
            print(f"[WARNING] Non-linear fit failed: {e}. Falling back to initial estimates.")
            popt = p0
            perr = np.zeros(5)

        A0, gamma, omega_d, phi, x_eq = popt
        A0_err, gamma_err, omega_d_err, phi_err, x_eq_err = perr

        # Calculate physical oscillator parameters
        # Undamped natural angular frequency: omega_0 = sqrt(omega_d^2 + gamma^2)
        omega_0 = float(np.sqrt(omega_d**2 + gamma**2))
        natural_freq_hz = omega_0 / (2.0 * np.pi)
        damped_freq_hz = omega_d / (2.0 * np.pi)
        damped_period_sec = (2.0 * np.pi) / omega_d if omega_d > 0 else np.nan

        # Damping ratio: zeta = gamma / omega_0
        damping_ratio_zeta = float(gamma / omega_0) if omega_0 > 0 else np.nan

        # Quality factor: Q = 1 / (2 * zeta) = omega_0 / (2 * gamma)
        quality_factor_Q = float(omega_0 / (2.0 * gamma)) if gamma > 0 else np.inf

        # Viscous damping coefficient c (N*s/m): c = 2 * m * gamma
        damping_coeff_c = float(2.0 * self.physics.mass * gamma)

        # Dynamic Spring Constant: k = m * omega_0^2
        if self.physics.oscillator_type == 'spring':
            k_effective = float(self.physics.mass * (omega_0**2))
        else:
            # For a pendulum: omega_0^2 = g / L -> effective restoring factor k_eff = m * g / L
            k_effective = float(self.physics.mass * self.physics.gravity / self.physics.pendulum_length)

        # Fitted curve & envelope
        fitted_curve = damped_oscillator_model(t_rel, *popt)
        upper_envelope = x_eq + np.abs(A0) * np.exp(-gamma * t_rel)
        lower_envelope = x_eq - np.abs(A0) * np.exp(-gamma * t_rel)

        # Goodness of fit (R^2)
        ss_res = np.sum((x - fitted_curve)**2)
        ss_tot = np.sum((x - np.mean(x))**2)
        r_squared = float(1.0 - (ss_res / ss_tot)) if ss_tot > 0 else 0.0

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
