"""
Serial Data Acquisition and Real-Time Logger for Arduino Uno + HC-SR04
Automated Kinematics Tracking System
"""

import sys
import time
import os
import csv
from datetime import datetime
from typing import Optional, List, Tuple, Generator
import serial
import serial.tools.list_ports

from config import AcquisitionParams, DEFAULT_ACQUISITION


def find_arduino_port() -> Optional[str]:
    """
    Scans available COM ports to automatically discover an Arduino Uno
    or compatible USB-to-Serial converter (CH340, CP2102, FTDI).
    """
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("[WARNING] No active serial ports found on this system.")
        return None

    print("\nScanning serial ports:")
    arduino_port = None
    for p in ports:
        desc = p.description or ""
        mfg = p.manufacturer or ""
        hwid = p.hwid or ""
        print(f"  - {p.device}: {desc} (Mfg: {mfg}, HWID: {hwid})")

        # Typical Uno R3: VID 0x2341, PID 0x0043 / 0x0001
        is_arduino = (
            "arduino" in desc.lower()
            or "arduino" in mfg.lower()
            or "2341:0043" in hwid.lower()
            or "ch340" in desc.lower()
            or "usb serial" in desc.lower()
        )
        if is_arduino and arduino_port is None:
            arduino_port = p.device

    if arduino_port:
        print(f"[INFO] Auto-selected Arduino port: {arduino_port}")
    else:
        # Fallback to the first available port if only one is present
        arduino_port = ports[0].device
        print(f"[INFO] No specific Arduino match detected; defaulting to: {arduino_port}")

    return arduino_port


class SerialKinematicsLogger:
    """
    Manages serial connection, high-frequency streaming, and CSV logging.
    """
    def __init__(self, params: Optional[AcquisitionParams] = None):
        self.params = params or DEFAULT_ACQUISITION
        self.serial_conn: Optional[serial.Serial] = None
        self.active = False

    def connect(self, port: Optional[str] = None) -> bool:
        target_port = port or self.params.port
        if target_port == "AUTO":
            target_port = find_arduino_port()
            if not target_port:
                raise ConnectionError("No available serial COM port found. Please plug in the Arduino Uno.")

        print(f"[INFO] Connecting to {target_port} at {self.params.baud_rate} baud...")
        try:
            self.serial_conn = serial.Serial(
                port=target_port,
                baudrate=self.params.baud_rate,
                timeout=self.params.timeout_sec
            )
            # Give Arduino time to auto-reset on DTR toggle
            time.sleep(2.0)
            self.serial_conn.reset_input_buffer()
            self.active = True
            print(f"[SUCCESS] Connected to {target_port}.")
            return True
        except serial.SerialException as e:
            print(f"[ERROR] Failed to connect to serial port {target_port}: {e}")
            return False

    def send_command(self, cmd: str):
        """Sends an interactive command to Arduino ('START', 'STOP', 'TARE', 'TEMP:23.5')."""
        if self.serial_conn and self.serial_conn.is_open:
            full_cmd = f"{cmd.strip()}\n"
            self.serial_conn.write(full_cmd.encode('ascii'))
            self.serial_conn.flush()
            print(f"[CMD SENT] -> {cmd}")

    def stream_data(
        self,
        duration_sec: Optional[float] = None
    ) -> Generator[Tuple[float, int, float, int], None, None]:
        """
        Yields parsed data tuples: (time_sec, sensor_id, distance_m, raw_pulse_us)
        Automatically handles relative time zeroing.
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            raise RuntimeError("Serial connection is not open.")

        max_dur = duration_sec or self.params.acquisition_duration_sec
        start_wall_time = time.time()
        initial_micros: Optional[int] = None

        print(f"[STREAM] Recording data for up to {max_dur:.1f} seconds. Press Ctrl+C to stop.")

        try:
            while True:
                if (time.time() - start_wall_time) >= max_dur:
                    print("[STREAM] Acquisition duration reached.")
                    break

                line_raw = self.serial_conn.readline()
                if not line_raw:
                    continue

                line = line_raw.decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                # Echo comments or status
                if line.startswith("#"):
                    print(f"  Arduino: {line}")
                    continue

                # Expected format: timestamp_us,sensor_id,distance_m,raw_pulse_us
                parts = line.split(',')
                if len(parts) >= 4:
                    try:
                        timestamp_us = int(parts[0])
                        sensor_id = int(parts[1])
                        distance_m = float(parts[2])
                        raw_pulse_us = int(parts[3])

                        if initial_micros is None:
                            initial_micros = timestamp_us

                        # Compute relative elapsed time in seconds with microsecond precision
                        rel_time_sec = (timestamp_us - initial_micros) * 1e-6

                        yield (rel_time_sec, sensor_id, distance_m, raw_pulse_us)

                    except ValueError:
                        # Malformed packet from startup noise; safely skip
                        continue
        except KeyboardInterrupt:
            print("\n[STREAM] User interrupted recording.")

    def record_to_csv(
        self,
        output_filepath: Optional[str] = None,
        duration_sec: Optional[float] = None
    ) -> str:
        """
        Streams data from Arduino and writes directly to a timestamped CSV file.
        Returns the absolute path of the created file.
        """
        if not output_filepath:
            os.makedirs("data", exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_filepath = os.path.join("data", f"kinematics_run_{timestamp}.csv")

        print(f"[INFO] Logging data directly to: {output_filepath}")

        count = 0
        with open(output_filepath, mode='w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(["time_sec", "sensor_id", "distance_m", "raw_pulse_us"])

            for entry in self.stream_data(duration_sec=duration_sec):
                writer.writerow([f"{entry[0]:.6f}", entry[1], f"{entry[2]:.6f}", entry[3]])
                count += 1
                if count % 20 == 0:
                    print(f"\r  Samples captured: {count:5d} | Time: {entry[0]:6.2f}s | Dist: {entry[2]:.4f}m", end="", flush=True)

        print(f"\n[COMPLETE] Logged {count} samples to {output_filepath}")
        return output_filepath

    def close(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("[INFO] Serial connection closed.")


if __name__ == "__main__":
    logger = SerialKinematicsLogger()
    if logger.connect():
        try:
            logger.record_to_csv(duration_sec=15.0)
        finally:
            logger.close()
