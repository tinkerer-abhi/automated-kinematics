"""
Experiment & Signal Processing Configuration
Automated Kinematics and Damped Harmonic Motion Analysis
"""

import dataclasses
from typing import Literal

@dataclasses.dataclass
class PhysicsParams:
    # Mass of the oscillating body (kg)
    mass: float = 0.200  # 200 g standard hooked mass
    
    # Nominal spring constant (N/m) - used as initial guess or prior
    nominal_spring_k: float = 15.0
    
    # Length of pendulum string from pivot to center of mass (m)
    pendulum_length: float = 0.60

    # Torsional disc parameters
    disc_radius: float = 0.08  # 8 cm radius
    vane_radius: float = 0.075 # 7.5 cm arm

    # Free fall drop height (m)
    drop_height: float = 0.60
    
    # Gravitational acceleration (m/s^2)
    gravity: float = 9.80665
    
    # Experiment type: 'spring', 'pendulum', 'cart', 'torsion', or 'freefall'
    oscillator_type: Literal['spring', 'pendulum', 'cart', 'torsion', 'freefall'] = 'spring'

@dataclasses.dataclass
class AcquisitionParams:
    # Serial configuration
    port: str = "AUTO"  # 'AUTO' will scan for Arduino Uno VID/PID or serial description
    baud_rate: int = 115200
    timeout_sec: float = 1.0
    
    # Target duration for data acquisition (seconds)
    acquisition_duration_sec: float = 20.0

@dataclasses.dataclass
class FilterParams:
    # Physical sensor boundaries (meters)
    min_valid_distance: float = 0.03   # 3 cm minimum HC-SR04 boundary
    max_valid_distance: float = 2.50   # 2.5 m maximum lab boundary
    
    # Outlier threshold: maximum physical velocity delta permitted between consecutive frames
    max_physically_possible_velocity: float = 4.5  # m/s
    
    # Savitzky-Golay filter parameters for numerical differentiation
    # Window length must be an odd integer, polyorder typically 2, 3, or 4
    savgol_window_length: int = 15
    savgol_polyorder: int = 3
    
    # Desired uniform resampling frequency (Hz) for derivative accuracy
    resample_rate_hz: float = 40.0

# Default configuration instances
DEFAULT_PHYSICS = PhysicsParams()
DEFAULT_ACQUISITION = AcquisitionParams()
DEFAULT_FILTER = FilterParams()
