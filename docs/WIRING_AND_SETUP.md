# Hardware Wiring, Electronics & Physical Rig Setup Guide

This guide details the physical assembly, electrical pinouts, and alignment geometry for the **Automated Kinematics and Damped Harmonic Motion Tracking System**.

---

## 1. Electronics & Wiring Schematic

The system uses an **Arduino Uno R3** communicating with one (or two) **HC-SR04 Ultrasonic Sensors**.

### Pinout Table (Single Axis - Standard Vertical Spring / 1D Pendulum)

| HC-SR04 Pin | Arduino Uno Pin | Wire Type | Notes |
| :--- | :--- | :--- | :--- |
| **VCC** | **5V** | Male-to-Female | Requires stable 5.0V regulated power |
| **GND** | **GND** | Male-to-Female | Common ground reference |
| **TRIG** | **Digital Pin 9** | Male-to-Female | 10 µs trigger pulse output from Uno |
| **ECHO** | **Digital Pin 10** | Male-to-Female | 5V TTL pulse input to Uno (microsecond round-trip) |

### Optional Dual-Axis Configuration (2D Pendulum / X-Y Tracking)
If measuring 2D planar motion (e.g., elliptical pendulum trajectories), wire a second HC-SR04:
- `TRIG_2` $\rightarrow$ **Digital Pin 7**
- `ECHO_2` $\rightarrow$ **Digital Pin 8**
- Set `ENABLE_DUAL_SENSOR = true` in `firmware/hc_sr04_tracker/hc_sr04_tracker.ino`.

### Wiring Diagram (ASCII Schematic)

```
        +-----------------------------------+
        |          ARDUINO UNO R3           |
        |                                   |
        |   5V    GND    Pin 9     Pin 10   |
        +----+-----+-------+---------+------+
             |     |       |         |
             |     |       |         |
      (Red)  |     | (Blk) | (Yel)   | (Grn)
             |     |       |         |
        +----+-----+-------+---------+------+
        |   VCC   GND    TRIG       ECHO    |
        |                                   |
        |        [ T ]       [ R ]          |
        |          HC-SR04 SENSOR           |
        +-----------------------------------+
```

---

## 2. Physical Rig & Mechanical Alignment

```
               [ Ring Stand Support Arm ]
                         |
                         |  (Clamp)
                         O
                        / \
                       /   \   <--- Extension Spring (k ≈ 10-25 N/m)
                      /     \
                         |
                   +-----------+
                   | Mass Set  | (100g - 300g)
                   +-----------+
                         |
               +-------------------+  <--- Flat Target Plate (Cardboard / Plastic disk)
               |   [===========]   |       (Ensures strong normal acoustic reflection)
               +-------------------+
                         |
                         |  Oscillation path (15 - 50 cm)
                         v
                    (Cone ≈ 15°)
                    /         \
                   /           \
                 [ T ]       [ R ]
             +-----------------------+
             |    HC-SR04 Mounted    |
             |    Facing Upwards     |
             +-----------------------+
                         |
             [ Heavy Ring Stand Base ]
```

### Essential Assembly Rules for High Precision:

1. **Acoustic Target Plate (Crucial!)**:
   - A standard cylindrical mass hook has a narrow, curved metallic surface that scatters ultrasonic sound waves in all directions, causing frequent echo misses.
   - **Fix**: Glue or tape a lightweight flat disk (e.g., a $6 \times 6\text{ cm}$ piece of sturdy cardboard, thin balsa wood, or a 3D-printed plastic wafer) to the bottom of the mass hook.
   - The ultrasonic wave strikes this perpendicular surface and reflects directly back into the receiver cone with high signal-to-noise ratio.

2. **HC-SR04 Beam Dispersion Cone (~15°)**:
   - The ultrasonic transmitter disperses sound in a cone of approximately $15^\circ$.
   - Ensure ring stand rods, clamps, and adjacent bench equipment are positioned outside this cone so the sensor does not detect secondary stationary echoes.

3. **Dead-Zone Constraint**:
   - HC-SR04 sensors have a physical blind spot below $2\text{ cm}$ (transducer ringing prevents immediate listening).
   - Adjust the ring stand height so that at maximum downward oscillation, the target plate remains at least **$10\text{ cm}$ away from the sensor**. An equilibrium distance of $30\text{ cm}$ to $45\text{ cm}$ is ideal.

4. **Sensor Orientation**:
   - For vertical spring oscillators: Mount the HC-SR04 on the base facing directly upward towards the descending mass plate.
   - For simple pendulums: Mount the HC-SR04 horizontally at the equilibrium height facing the side of the mass at its lowest point.

---

## 3. Calibration & Temperature Compensation

The speed of sound in air varies with temperature:
$$v_{\text{sound}} = 331.3 \times \sqrt{1 + \frac{T}{273.15}} \approx 331.3 + 0.606 \times T \,(\text{m/s})$$

At $20^\circ\text{C}$, $v \approx 343.2\text{ m/s}$. At $28^\circ\text{C}$, $v \approx 348.3\text{ m/s}$ (a $1.5\%$ shift that directly scales all distance derivatives!).

- You can calibrate temperature on the fly by sending `TEMP:24.5` through the serial monitor.
- Baseline zeroing: Sending `TARE` sets the current distance as $x=0$, allowing the Arduino to stream displacement relative to equilibrium.
