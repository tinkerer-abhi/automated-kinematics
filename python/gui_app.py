"""
Graphical User Interface (GUI) Control Center
Automated Kinematics & Damped Harmonic Motion Analysis

Enables zero-code experimentation:
- Adjust mass (grams), pendulum length, and duration directly in input boxes
- 1-click Arduino COM port detection and Tare/Zero calibration
- 1-click Live Run, CSV Analysis, or Simulation
- Automatically derives dynamic spring constant (k) from observed oscillation frequency!
"""

import os
import sys
import threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from config import PhysicsParams, AcquisitionParams, FilterParams
from serial_logger import SerialKinematicsLogger, find_arduino_port
from kinematics import KinematicsEngine
from harmonic_fit import HarmonicFitEngine
from visualizer import KinematicsVisualizer
from simulate_test import generate_damped_spring_data, save_synthetic_csv


class KinematicsApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Automated Kinematics & Damped Motion Suite")
        self.geometry("680x740")
        self.minsize(620, 680)

        # Apply clean modern styling
        self.configure(bg="#0f172a") # Slate 900
        self.style = ttk.Style(self)
        self.style.theme_use('clam')
        self._configure_styles()

        self.logger_instance = None
        self._build_ui()
        self.refresh_ports()

    def _configure_styles(self):
        self.style.configure(".", background="#0f172a", foreground="#f8fafc", font=("Segoe UI", 10))
        self.style.configure("TLabel", background="#0f172a", foreground="#cbd5e1", font=("Segoe UI", 10))
        self.style.configure("Header.TLabel", font=("Segoe UI", 14, "bold"), foreground="#38bdf8")
        self.style.configure("SubHeader.TLabel", font=("Segoe UI", 9), foreground="#94a3b8")
        self.style.configure("Card.TLabelframe", background="#1e293b", bordercolor="#334155", relief="solid", borderwidth=1)
        self.style.configure("Card.TLabelframe.Label", background="#1e293b", foreground="#38bdf8", font=("Segoe UI", 10, "bold"))
        self.style.configure("TEntry", fieldbackground="#0f172a", foreground="#f8fafc", bordercolor="#475569")
        self.style.configure("TCombobox", fieldbackground="#0f172a", foreground="#f8fafc", bordercolor="#475569")
        
        # Primary Action Button (Emerald Green)
        self.style.configure("Primary.TButton", font=("Segoe UI", 10, "bold"), background="#059669", foreground="#ffffff")
        self.style.map("Primary.TButton", background=[("active", "#10b981")])
        
        # Secondary Action Button (Sky Blue)
        self.style.configure("Secondary.TButton", font=("Segoe UI", 10), background="#0284c7", foreground="#ffffff")
        self.style.map("Secondary.TButton", background=[("active", "#38bdf8")])

        # Amber Action Button
        self.style.configure("Amber.TButton", font=("Segoe UI", 10), background="#d97706", foreground="#ffffff")
        self.style.map("Amber.TButton", background=[("active", "#f59e0b")])

    def _build_ui(self):
        main_frame = ttk.Frame(self, padding="16 12 16 12")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 1. Header Banner
        header = ttk.Label(main_frame, text="⚡ Automated Kinematics Lab Suite", style="Header.TLabel")
        header.pack(anchor="w", pady=(0, 2))
        sub = ttk.Label(main_frame, text="Live Ultrasonic Tracking, Savitzky-Golay Derivatives & Energy Dissipation", style="SubHeader.TLabel")
        sub.pack(anchor="w", pady=(0, 12))

        # 2. Experiment Setup Card
        setup_card = ttk.LabelFrame(main_frame, text=" ⚙️ Physical Experiment Parameters ", style="Card.TLabelframe", padding="12")
        setup_card.pack(fill=tk.X, pady=(0, 10))

        # Row 1: Oscillator Type
        r1 = ttk.Frame(setup_card)
        r1.pack(fill=tk.X, pady=4)
        ttk.Label(r1, text="Oscillator Setup:", width=18).pack(side=tk.LEFT)
        self.osc_type_var = tk.StringVar(value="spring")
        types_cb = ttk.Combobox(r1, textvariable=self.osc_type_var, state="readonly",
                                values=["spring (Vertical Extension)", "pendulum (Simple Bob)", "cart (Horizontal Track)", "freefall (Drop Tube)"], width=28)
        types_cb.pack(side=tk.LEFT, padx=4)
        types_cb.bind("<<ComboboxSelected>>", self._on_type_changed)

        # Row 2: Mass (grams)
        r2 = ttk.Frame(setup_card)
        r2.pack(fill=tk.X, pady=4)
        ttk.Label(r2, text="Oscillating Mass (g):", width=18).pack(side=tk.LEFT)
        self.mass_var = tk.StringVar(value="200.0")
        mass_entry = ttk.Entry(r2, textvariable=self.mass_var, width=12)
        mass_entry.pack(side=tk.LEFT, padx=4)
        ttk.Label(r2, text="grams (e.g., 50g - 500g)", style="SubHeader.TLabel").pack(side=tk.LEFT, padx=4)

        # Row 3: Pendulum Length (cm) - Optional
        self.row_len = ttk.Frame(setup_card)
        ttk.Label(self.row_len, text="Pendulum Length (cm):", width=18).pack(side=tk.LEFT)
        self.length_var = tk.StringVar(value="60.0")
        len_entry = ttk.Entry(self.row_len, textvariable=self.length_var, width=12)
        len_entry.pack(side=tk.LEFT, padx=4)
        ttk.Label(self.row_len, text="cm (pivot to mass center)", style="SubHeader.TLabel").pack(side=tk.LEFT, padx=4)

        # Row 4: Spring Note (Self-Calculation Notice)
        r4 = ttk.Frame(setup_card)
        r4.pack(fill=tk.X, pady=4)
        self.notice_lbl = ttk.Label(r4, text="💡 Tip: Spring constant (k) is AUTO-CALCULATED from motion frequency! No need to measure k beforehand.", foreground="#38bdf8", wraplength=580)
        self.notice_lbl.pack(side=tk.LEFT)

        # 3. Hardware & Serial Connection Card
        hw_card = ttk.LabelFrame(main_frame, text=" 🔌 Hardware & Arduino Serial Connection ", style="Card.TLabelframe", padding="12")
        hw_card.pack(fill=tk.X, pady=(0, 10))

        hw_row = ttk.Frame(hw_card)
        hw_row.pack(fill=tk.X, pady=4)
        ttk.Label(hw_row, text="Arduino COM Port:", width=18).pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value="AUTO")
        self.port_cb = ttk.Combobox(hw_row, textvariable=self.port_var, width=20)
        self.port_cb.pack(side=tk.LEFT, padx=4)

        ttk.Button(hw_row, text="🔍 Scan", width=8, command=self.refresh_ports).pack(side=tk.LEFT, padx=4)
        ttk.Button(hw_row, text="🎯 Zero / Tare", style="Amber.TButton", width=14, command=self.send_tare).pack(side=tk.LEFT, padx=6)

        hw_row2 = ttk.Frame(hw_card)
        hw_row2.pack(fill=tk.X, pady=4)
        ttk.Label(hw_row2, text="Run Duration (sec):", width=18).pack(side=tk.LEFT)
        self.dur_var = tk.StringVar(value="16.0")
        dur_entry = ttk.Entry(hw_row2, textvariable=self.dur_var, width=10)
        dur_entry.pack(side=tk.LEFT, padx=4)
        ttk.Label(hw_row2, text="seconds @ ~40 Hz", style="SubHeader.TLabel").pack(side=tk.LEFT, padx=4)

        # 4. Action Buttons Card
        btn_card = ttk.Frame(main_frame)
        btn_card.pack(fill=tk.X, pady=(4, 10))

        self.btn_live = ttk.Button(btn_card, text="🚀 START LIVE EXPERIMENT", style="Primary.TButton", command=self.on_start_live)
        self.btn_live.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4), ipady=6)

        self.btn_file = ttk.Button(btn_card, text="📂 Analyze CSV File", style="Secondary.TButton", command=self.on_load_csv)
        self.btn_file.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4, ipady=6)

        self.btn_sim = ttk.Button(btn_card, text="🧪 Run Simulation Test", command=self.on_run_sim)
        self.btn_sim.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 0), ipady=6)

        # 5. Live Telemetry & Results Card
        res_card = ttk.LabelFrame(main_frame, text=" 📊 Results & Derived Physics Quantities ", style="Card.TLabelframe", padding="12")
        res_card.pack(fill=tk.BOTH, expand=True, pady=(0, 4))

        self.status_lbl = ttk.Label(res_card, text="Status: Ready. Adjust parameters above and click Start.", foreground="#10b981", font=("Segoe UI", 10, "bold"))
        self.status_lbl.pack(anchor="w", pady=(0, 6))

        # Results Text Box
        self.res_text = tk.Text(res_card, height=10, bg="#0b0f19", fg="#f1f5f9", insertbackground="#38bdf8",
                                font=("Consolas", 9), relief="flat", borderwidth=0, padx=8, pady=8)
        self.res_text.pack(fill=tk.BOTH, expand=True)
        self.res_text.insert(tk.END, "Derived quantities will appear here automatically upon experiment completion:\n"
                                     " - Natural Frequency (f0) & Damped Period (Td)\n"
                                     " - Dynamic Spring Constant (k = m*w0^2)\n"
                                     " - Damping Ratio (zeta) & Quality Factor (Q)\n"
                                     " - Viscous Damping Coefficient (c = 2*m*gamma)\n"
                                     " - Total Mechanical Energy Dissipation\n")
        self.res_text.configure(state="disabled")

    def _on_type_changed(self, event=None):
        val = self.osc_type_var.get().split()[0].lower()
        if val == "pendulum":
            self.row_len.pack(fill=tk.X, pady=4)
            self.notice_lbl.config(text="💡 Tip: Restoring factor for pendulum is k_eff = (m*g)/L.")
        else:
            self.row_len.pack_forget()
            self.notice_lbl.config(text="💡 Tip: Spring constant (k) is AUTO-CALCULATED from motion frequency! No need to measure k beforehand.")

    def refresh_ports(self):
        import serial.tools.list_ports
        ports = [p.device for p in serial.tools.list_ports.comports()]
        port_list = ["AUTO"] + ports
        self.port_cb['values'] = port_list
        if ports:
            auto_port = find_arduino_port()
            self.port_var.set(auto_port or ports[0])
        else:
            self.port_var.set("AUTO")

    def send_tare(self):
        target_port = self.port_var.get()
        if target_port == "AUTO":
            target_port = find_arduino_port()
        if not target_port:
            messagebox.showwarning("No Port", "Please plug in Arduino Uno or select a COM port first.")
            return

        def _do_tare():
            try:
                import serial
                s = serial.Serial(target_port, 115200, timeout=1.0)
                import time; time.sleep(1.8)
                s.write(b"TARE\n")
                s.flush()
                time.sleep(0.5)
                s.close()
                self.set_status("Equilibrium Tare Sent! Offset calibrated.", "#10b981")
            except Exception as e:
                self.set_status(f"Tare failed: {e}", "#ef4444")

        threading.Thread(target=_do_tare, daemon=True).start()

    def set_status(self, text, color="#cbd5e1"):
        self.status_lbl.config(text=f"Status: {text}", foreground=color)

    def _get_physics(self):
        try:
            mass_kg = float(self.mass_var.get()) / 1000.0 # grams to kg
            if mass_kg <= 0: raise ValueError
        except Exception:
            messagebox.showerror("Invalid Mass", "Please enter a positive numeric mass in grams (e.g. 200).")
            return None

        osc_type = self.osc_type_var.get().split()[0].lower()
        try:
            L_m = float(self.length_var.get()) / 100.0 # cm to m
        except Exception:
            L_m = 0.60

        return PhysicsParams(mass=mass_kg, oscillator_type=osc_type, pendulum_length=L_m)

    def on_run_sim(self):
        physics = self._get_physics()
        if not physics: return
        self.set_status("Running synthetic physics simulation...", "#38bdf8")

        try:
            dur = float(self.dur_var.get())
        except Exception:
            dur = 16.0

        def _worker():
            t_raw, dist_raw, _, _ = generate_damped_spring_data(
                duration_sec=dur, sample_rate_hz=40.0, physics=physics,
                initial_displacement_m=0.10, damping_coeff_c=0.060, equilibrium_dist_m=0.40, dropout_prob=0.02
            )
            self._process_and_show(t_raw, dist_raw, physics, title="Synthetic Simulation Run")

        threading.Thread(target=_worker, daemon=True).start()

    def on_load_csv(self):
        physics = self._get_physics()
        if not physics: return

        filepath = filedialog.askopenfilename(
            title="Select Recorded Kinematics CSV",
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")],
            initialdir=os.path.join(os.path.dirname(__file__), "data")
        )
        if not filepath: return

        try:
            df = pd.read_csv(filepath, comment="#")
            t_raw = df["time_sec"].values
            dist_raw = df["distance_m"].values
            self.set_status(f"Loaded {os.path.basename(filepath)}. Processing...", "#38bdf8")
            threading.Thread(
                target=lambda: self._process_and_show(t_raw, dist_raw, physics, title=f"Analysis: {os.path.basename(filepath)}"),
                daemon=True
            ).start()
        except Exception as e:
            messagebox.showerror("Error Reading CSV", f"Could not parse file: {e}")

    def on_start_live(self):
        physics = self._get_physics()
        if not physics: return

        target_port = self.port_var.get()
        try:
            dur = float(self.dur_var.get())
        except Exception:
            dur = 16.0

        self.btn_live.config(state="disabled")
        self.set_status(f"Recording live motion ({dur}s)... Keep mass oscillating!", "#fbbf24")

        def _live_worker():
            try:
                acq = AcquisitionParams(port=target_port, baud_rate=115200, acquisition_duration_sec=dur)
                logger = SerialKinematicsLogger(acq)
                if not logger.connect():
                    self.set_status("Failed to connect to Arduino. Check USB connection.", "#ef4444")
                    self.btn_live.config(state="normal")
                    return

                try:
                    csv_path = logger.record_to_csv(duration_sec=dur)
                finally:
                    logger.close()

                df = pd.read_csv(csv_path, comment="#")
                t_raw = df["time_sec"].values
                dist_raw = df["distance_m"].values
                self._process_and_show(t_raw, dist_raw, physics, title=f"Live Hardware Run: {physics.oscillator_type.title()}")
            except Exception as e:
                self.set_status(f"Error during live run: {e}", "#ef4444")
            finally:
                self.btn_live.config(state="normal")

        threading.Thread(target=_live_worker, daemon=True).start()

    def _process_and_show(self, t_raw, dist_raw, physics, title):
        try:
            ke = KinematicsEngine(FilterParams())
            kinematics = ke.process_kinematics(t_raw, dist_raw)

            fe = HarmonicFitEngine(physics)
            fit = fe.fit_damped_motion(kinematics["time"], kinematics["position"])
            energy = fe.calculate_energy(
                kinematics["time"], kinematics["position"], kinematics["velocity"],
                k=fit["k_effective"], x_eq=fit["x_eq"], gamma=fit["gamma"]
            )

            # Update Text Results Box
            self.res_text.configure(state="normal")
            self.res_text.delete("1.0", tk.END)
            summary = (
                f"=== DERIVED EXPERIMENT RESULTS ({physics.oscillator_type.upper()}) ===\n"
                f"• Mass Entered:           {physics.mass*1000.0:.1f} g ({physics.mass:.3f} kg)\n"
                f"• Damped Frequency (fd):  {fit['damped_freq_hz']:.3f} Hz  (Period Td = {fit['damped_period_sec']:.3f} s)\n"
                f"• Natural Frequency (f0): {fit['natural_freq_hz']:.3f} Hz (omega_0 = {fit['omega_0']:.3f} rad/s)\n"
                f"• DYNAMIC SPRING CONST (k): {fit['k_effective']:.3f} N/m  <-- AUTO-DERIVED!\n"
                f"• Damping Ratio (zeta):   {fit['damping_ratio_zeta']:.5f} ({'Underdamped' if fit['damping_ratio_zeta'] < 1 else 'Overdamped'})\n"
                f"• Viscous Damping (c):    {fit['damping_coeff_c']:.5f} N·s/m\n"
                f"• Quality Factor (Q):     {fit['quality_factor_Q']:.2f}\n"
                f"• Goodness of Fit (R²):   {fit['r_squared']:.5f}\n"
                f"• Initial Stored Energy:  {energy['E0']*1000.0:.2f} mJ\n"
                f"• Energy Lost:            {((energy['E0'] - energy['total'][-1])/energy['E0'])*100:.1f}% over run\n"
            )
            self.res_text.insert(tk.END, summary)
            self.res_text.configure(state="disabled")

            self.set_status("Complete! Displaying interactive kinematics dashboard...", "#10b981")

            # Pop up the 4-panel dashboard
            viz = KinematicsVisualizer()
            plot_file = os.path.join(os.path.dirname(__file__), "data", "kinematics_dashboard.png")
            fig = viz.generate_dashboard(kinematics, fit, energy, title_prefix=title, save_path=plot_file)
            plt.show()

        except Exception as e:
            self.set_status(f"Processing error: {e}", "#ef4444")


def launch_gui():
    app = KinematicsApp()
    app.mainloop()


if __name__ == "__main__":
    launch_gui()
