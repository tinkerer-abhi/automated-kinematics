# Automated Kinematics & Damped Harmonic Motion Tracking System

A laboratory-grade, automated motion tracking and signal processing suite that replaces manual stopwatches with high-frequency ultrasonic sensing (**Arduino Uno R3 + HC-SR04**) and rigorous numerical kinematics analysis in **Python**.

---

## 📑 Table of Contents
- [System Architecture](#system-architecture)
- [Directory Structure](#directory-structure)
- [Quick Start: Python & Conda Setup](#quick-start-python--conda-setup)
- [Arduino Firmware Flashing](#arduino-firmware-flashing)
- [Running the System](#running-the-system)
  - [1. Synthetic Simulation Mode (Hardware-Free Verification)](#1-synthetic-simulation-mode-hardware-free-verification)
  - [2. Live Data Acquisition from Arduino](#2-live-data-acquisition-from-arduino)
  - [3. Offline Analysis of Existing Data](#3-offline-analysis-of-existing-data)
- [Scientific Capabilities & Analytics](#scientific-capabilities--analytics)
- [Hardware Wiring & Rig Alignment](#hardware-wiring--rig-alignment)
- [Mathematical Derivations](#mathematical-derivations)

---

## System Architecture

```
[ Physical Rig ]                 [ Arduino Uno R3 ]                    [ Python Kinematics Engine ]
Spring/Pendulum  === Sound ===>  HC-SR04 Polling                       1. Outlier Rejection & Spline Resampling
Oscillator with                  • Non-blocking micros()               2. Savitzky-Golay Numerical Derivatives (v, a)
Target Reflector                 • 115200 Baud CSV Stream              3. Non-Linear Levenberg-Marquardt Fit
                                         |                             4. Energy Conservation Tracking: E(t) = T + V
                                         v                             5. Publication 4-Panel Visualization
                                   [ USB Cable ]
```

---

## Directory Structure

```
automated-kinematics/
├── firmware/
│   └── hc_sr04_tracker/
│       └── hc_sr04_tracker.ino     # High-rate Arduino Uno firmware (115200 baud, micros timestamp, timeout protection)
├── python/
│   ├── config.py                   # Centralized configuration (mass, spring k, filter parameters, COM port)
│   ├── serial_logger.py            # Serial acquisition module with auto-port detection and CSV writer
│   ├── kinematics.py               # Outlier rejection, uniform resampling & Savitzky-Golay derivative engine
│   ├── harmonic_fit.py             # Damped harmonic ODE fitting (zeta, Q, c, k) & continuous energy calculator
│   ├── visualizer.py               # 4-panel publication dashboard + terminal experiment report
│   ├── simulate_test.py            # Physics-accurate sensor simulator with noise & echo dropouts
│   ├── main.py                     # Unified CLI entrypoint (Sim, Live, Analyze)
│   └── requirements.txt            # Python dependencies
├── docs/
│   ├── WIRING_AND_SETUP.md         # Pinout table, circuit diagram & physical target plate mounting rules
│   └── THEORY_DERIVATION.md        # Mathematical derivations (DHO ODE, Savitzky-Golay, energy dissipation)
└── README.md
```

---

## Quick Start: Python & Conda Setup

### Step 1: Open Terminal in VS Code
Open VS Code in the project root:
```bash
cd "C:\Users\abhiy\.gemini\antigravity\scratch\automated-kinematics\python"
```

### Step 2: Create / Activate Environment
If you are using **Conda**:
```bash
conda create -n kinematics python=3.11 -y
conda activate kinematics
pip install -r requirements.txt
```

Or using native **pip** directly:
```bash
pip install -r requirements.txt
```

Dependencies installed: `pyserial`, `numpy`, `scipy`, `matplotlib`, `pandas`.

---

## Arduino Firmware Flashing

1. Connect your **Arduino Uno R3** to your computer via USB.
2. Open the **Arduino IDE**.
3. Open `firmware/hc_sr04_tracker/hc_sr04_tracker.ino`.
4. Select **Tools > Board > Arduino Uno**.
5. Select **Tools > Port** (e.g. `COM3` or `COM4`).
6. Click **Upload** ($\rightarrow$).
7. (Optional) Open the Serial Monitor at **115200 baud** to confirm you see the CSV stream:
   ```csv
   # Automated Kinematics Tracking System v1.0
   timestamp_us,sensor_id,distance_m,raw_pulse_us
   1482040,1,0.34215,1994
   1507080,1,0.34580,2015
   ```

---

## Running the System

### 1. Synthetic Simulation Mode (Hardware-Free Verification)
Test the entire filtering, differentiation, curve fitting, and energy conservation pipeline immediately without any physical hardware attached:

```bash
python main.py --mode sim --duration 18.0 --mass 0.200
```

This generates synthetic damped oscillator data with realistic ultrasonic noise, quantization steps, and echo dropouts, then outputs:
- Real-time terminal physics table
- An interactive 4-panel analysis plot
- Saved high-res dashboard at `data/kinematics_dashboard.png`

---

### 2. Live Data Acquisition from Arduino
To record real-time oscillations from your physical rig:

```bash
python main.py --mode live --duration 20.0 --mass 0.200 --type spring
```

The script will:
1. Auto-discover the Arduino Uno COM port.
2. Stream and timestamp microsecond echo pulses.
3. Automatically save the raw data run to `data/kinematics_run_<timestamp>.csv`.
4. Clean and differentiate velocity and acceleration.
5. Fit the damped harmonic model and render the analytics dashboard.

---

### 3. Offline Analysis of Existing Data
If you've already recorded a run and want to analyze or replot it with different masses:

```bash
python main.py --mode analyze --file data/kinematics_run_20261026_143000.csv --mass 0.250
```

---

## Scientific Capabilities & Analytics

The suite automatically derives:

| Metric | Symbol | Derivation / Formula |
| :--- | :--- | :--- |
| **Equilibrium Position** | $x_{\text{eq}}$ | Fitted baseline offset |
| **Initial Amplitude** | $A_0$ | Peak displacement at release |
| **Damped Angular Frequency** | $\omega_d$ | $2\pi / T_d$ from non-linear regression |
| **Undamped Natural Frequency** | $\omega_0$ | $\sqrt{\omega_d^2 + \gamma^2}$ |
| **Decay Constant** | $\gamma$ | Exponential envelope decay rate |
| **Damping Ratio** | $\zeta$ | $\gamma / \omega_0$ (Identifies underdamped $\zeta < 1$) |
| **Quality Factor** | $Q$ | $\frac{1}{2\zeta} = \frac{\omega_0}{2\gamma}$ |
| **Viscous Damping Coefficient** | $c$ | $2 m \gamma$ ($\text{N}\cdot\text{s/m}$) |
| **Effective Dynamic Stiffness** | $k$ | $m \omega_0^2$ ($\text{N/m}$) |
| **Kinetic Energy** | $T(t)$ | $\frac{1}{2} m v(t)^2$ |
| **Potential Energy** | $V(t)$ | $\frac{1}{2} k (x(t) - x_{\text{eq}})^2$ |
| **Total Mechanical Energy** | $E(t)$ | $T(t) + V(t)$, verified against $E_0 e^{-2\gamma t}$ |
| **Rate of Dissipation** | $\frac{dE}{dt}$ | Verified against $-c v(t)^2 \le 0$ |

---

## Detailed Documentation

- **Hardware & Wiring**: See [docs/WIRING_AND_SETUP.md](file:///C:/Users/abhiy/.gemini/antigravity/scratch/automated-kinematics/docs/WIRING_AND_SETUP.md) for pinouts, target plate construction, and beam geometry.
- **Physics Derivations**: See [docs/THEORY_DERIVATION.md](file:///C:/Users/abhiy/.gemini/antigravity/scratch/automated-kinematics/docs/THEORY_DERIVATION.md) for complete mathematical proofs, noise amplification analysis, and Savitzky-Golay filtering theory.
