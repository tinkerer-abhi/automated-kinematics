"""
High-Precision Kinematics Processing & Derivative Engine
Automated Kinematics and Damped Harmonic Motion Analysis

Solves the core challenge of ultrasonic sensor tracking:
- Eliminates echo dropouts and multipath reflections via physical outlier rejection
- Uniformly resamples timing jitter
- Uses Savitzky-Golay polynomial differentiation to extract smooth velocity (dx/dt)
  and acceleration (d^2x/dt^2) WITHOUT noise amplification.
"""

from typing import Tuple, Dict, Any
import numpy as np
from scipy import signal, interpolate

from config import FilterParams, DEFAULT_FILTER


class KinematicsEngine:
    def __init__(self, filter_params: FilterParams = DEFAULT_FILTER):
        self.params = filter_params

    def clean_and_resample(
        self,
        raw_time: np.ndarray,
        raw_dist: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, Dict[str, Any]]:
        """
        Cleans raw ultrasonic data:
        1. Filters out-of-range sensor readings (< min or > max or negative timeout).
        2. Rejects velocity discontinuities (impossible jumps exceeding physical kinematics).
        3. Interpolates missing frames onto an exact uniform time grid.
        """
        # Ensure numpy float arrays and sort monotonically by time
        time_arr = np.asarray(raw_time, dtype=np.float64)
        dist_arr = np.asarray(raw_dist, dtype=np.float64)

        sort_idx = np.argsort(time_arr)
        time_arr = time_arr[sort_idx]
        dist_arr = dist_arr[sort_idx]

        # 1. Boundary mask
        valid_mask = (
            (dist_arr >= self.params.min_valid_distance) &
            (dist_arr <= self.params.max_valid_distance) &
            np.isfinite(dist_arr)
        )

        # 2. Outlier rejection via consecutive physical velocity threshold
        valid_indices = np.where(valid_mask)[0]
        cleaned_indices = []

        if len(valid_indices) > 2:
            cleaned_indices.append(valid_indices[0])
            for i in range(1, len(valid_indices)):
                prev_idx = cleaned_indices[-1]
                curr_idx = valid_indices[i]

                dt = time_arr[curr_idx] - time_arr[prev_idx]
                if dt <= 1e-5:
                    continue  # Skip identical timestamps

                dx = abs(dist_arr[curr_idx] - dist_arr[prev_idx])
                apparent_v = dx / dt

                if apparent_v <= self.params.max_physically_possible_velocity:
                    cleaned_indices.append(curr_idx)

        cleaned_indices = np.array(cleaned_indices, dtype=int)
        
        if len(cleaned_indices) < 10:
            raise ValueError(
                f"Too few valid ultrasonic data points ({len(cleaned_indices)}) after outlier filtering. "
                "Check sensor alignment, target plate reflectivity, or range boundaries."
            )

        t_clean = time_arr[cleaned_indices]
        x_clean = dist_arr[cleaned_indices]

        # 3. Resample onto a uniform time grid
        t_start = t_clean[0]
        t_end = t_clean[-1]
        dt_resample = 1.0 / self.params.resample_rate_hz
        num_uniform_points = int(np.floor((t_end - t_start) / dt_resample)) + 1
        t_uniform = np.linspace(t_start, t_end, num_uniform_points)

        # Cubic spline or linear interpolation for uniform grid
        if len(t_clean) > 4:
            spline = interpolate.CubicSpline(t_clean, x_clean, extrapolate=False)
            x_uniform = spline(t_uniform)
            # Fallback for any NaN edges
            nan_mask = np.isnan(x_uniform)
            if np.any(nan_mask):
                x_uniform[nan_mask] = np.interp(t_uniform[nan_mask], t_clean, x_clean)
        else:
            x_uniform = np.interp(t_uniform, t_clean, x_clean)

        stats = {
            "total_raw_samples": len(raw_time),
            "valid_samples": len(cleaned_indices),
            "dropped_samples": len(raw_time) - len(cleaned_indices),
            "rejection_rate_pct": ((len(raw_time) - len(cleaned_indices)) / len(raw_time)) * 100.0,
            "resampled_dt": dt_resample,
            "resampled_samples": len(t_uniform)
        }

        return t_uniform, x_uniform, stats

    def compute_derivatives(
        self,
        t_uniform: np.ndarray,
        x_uniform: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Computes smoothed position, velocity, and acceleration derivatives
        using the Savitzky-Golay digital filter.
        
        Returns:
            x_smooth: Filtered position (m)
            velocity: 1st time derivative dx/dt (m/s)
            acceleration: 2nd time derivative d^2x/dt^2 (m/s^2)
        """
        window_len = self.params.savgol_window_length
        polyorder = self.params.savgol_polyorder

        # Window length must be odd and strictly less than data length
        if len(x_uniform) <= window_len:
            window_len = len(x_uniform) if len(x_uniform) % 2 != 0 else len(x_uniform) - 1
            if window_len <= polyorder:
                polyorder = max(1, window_len - 1)

        dt = float(t_uniform[1] - t_uniform[0])

        # Savitzky-Golay direct derivative calculation:
        # Fits local polynomial and analytically evaluates 0th, 1st, and 2nd derivatives
        x_smooth = signal.savgol_filter(
            x_uniform, window_length=window_len, polyorder=polyorder, deriv=0
        )
        velocity = signal.savgol_filter(
            x_uniform, window_length=window_len, polyorder=polyorder, deriv=1, delta=dt
        )
        acceleration = signal.savgol_filter(
            x_uniform, window_length=window_len, polyorder=polyorder, deriv=2, delta=dt
        )

        return x_smooth, velocity, acceleration

    def process_kinematics(
        self,
        raw_time: np.ndarray,
        raw_dist: np.ndarray
    ) -> Dict[str, Any]:
        """
        Full end-to-end pipeline:
        Raw Sensor Stream -> Clean Resampled Data -> Filtered x(t), v(t), a(t)
        """
        t_grid, x_grid, stats = self.clean_and_resample(raw_time, raw_dist)
        x_filt, v_filt, a_filt = self.compute_derivatives(t_grid, x_grid)

        return {
            "time": t_grid,
            "raw_time": raw_time,
            "raw_dist": raw_dist,
            "position_uniform": x_grid,
            "position": x_filt,
            "velocity": v_filt,
            "acceleration": a_filt,
            "stats": stats
        }
