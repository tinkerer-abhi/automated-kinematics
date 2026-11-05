# Universal Hardware Rig & Modular Sensor Mount Design Guide

This guide details the mechanical architecture and mounting fixtures needed to make the **HC-SR04 Arduino Kinematics Tracker** universally compatible across multiple physical oscillation experiments:
1. **Vertical Spring-Mass Oscillator**
2. **Simple & Compound Pendulums**
3. **Horizontal Cart / Dynamics Track Oscillator**
4. **Torsional Oscillator (Rotary Disc)**
5. **Free-Fall Kinematics Drop Column**

---

## 1. The Core Universal "Sensor Pod" Design

Instead of hot-gluing or taping the HC-SR04 directly to a stand, build a standardized **Modular Sensor Pod**.

### Bill of Materials (Sensor Pod)
- **HC-SR04 Bracket**: 3D-printed clip or small acrylic plate with two 16mm cutouts for the transducers.
- **Adjustable Swivel Mount**: Standard 1/4"-20 mini camera ball-head clamp (or a laboratory boss-head clamp with swivel arm).
- **Acoustic Baffle Hood**: A short 15mm-deep cylindrical cardboard/foam shroud around the receiver to suppress side-reflections from lab ring stands.
- **Detachable 4-Wire Ribbon Cable**: 1-meter ribbon cable (5V, GND, Trig, Echo) terminating in a standardized 4-pin Dupont header.

```
       +-----------------------------------------------+
       |             MODULAR SENSOR POD                |
       |                                               |
       |     [ Transmit ]            [ Receive ]       |
       |     ( 40 kHz )              ( 40 kHz )        |
       |        +--+                    +--+           |
       |        |  |  [Foam Baffle]     |  |           |
       |        +--+                    +--+           |
       |                                               |
       |       [ 1/4"-20 Threaded Mounting Nut ]       |
       +-----------------------+-----------------------+
                               |
                   [ Ball-and-Socket Swivel ]
                   (360° Pan / 180° Tilt Lock)
                               |
                   [ 1/2" Lab Rod Swivel Clamp ]
                               |
                     [ Main Lab Ring Stand ]
```

---

## 2. Universal Target Plate Standard (The "Acoustic Retro-Reflector")

The primary cause of hardware failure in acoustic tracking is ultrasonic wave scattering off curved mass surfaces.

### Universal Plate Specs:
- **Geometry**: $60\text{ mm} \times 60\text{ mm}$ square or $\varnothing 65\text{ mm}$ round disk.
- **Material**: 2mm corrugated plastic (Coroplast), thin balsa wood, or 3D-printed PLA.
  - *Weight constraint*: $< 5\text{ grams}$ so it represents $< 2.5\%$ of a 200g test mass.
- **Attachment Interface**:
  - Top surface: Center hole for hook ring + small neodymium magnet ($\varnothing 8\text{ mm} \times 1.5\text{ mm}$) to snap onto slotted steel masses.
  - Bottom surface: Flat, smooth acoustic reflective plane.

---

## 3. Modular Rig Configurations

### Configuration A: Vertical Spring-Mass Oscillator
- **Movement Axis**: Vertical ($Z$-axis).
- **Sensor Orientation**: Upward-facing, clamped to the heavy ring stand base.
- **Sensor-to-Equilibrium Distance**: $35 - 45\text{ cm}$.
- **Mass Target Plate**: Glued/magnetized beneath the bottom-most hooked mass.
- **Why this works**: The mass moves directly along the acoustic boresight. Keeping equilibrium at $40\text{ cm}$ ensures even large $15\text{ cm}$ oscillations never enter the $<5\text{ cm}$ deadband.

```
          [ Top Ring Stand Clamp ]
                     |
                 [ Spring ]
                     |
                [ Mass Set ]
                     |
            [ Target Plate (Disk) ]
                     |
                     |  Δz (Oscillation)
                     v
           [ Sensor Pod (Upward) ]
                     |
             [ Heavy Cast Base ]
```

---

### Configuration B: Simple & Compound Pendulum
- **Movement Axis**: Circular arc in $X-Z$ plane.
- **Sensor Orientation**: Clamped horizontally at the height of the pendulum's lowest point (equilibrium), aimed horizontally along the tangential arc of motion.
- **Target Plate**: Lightweight vertical vane attached to the pendulum bob facing the sensor.
- **Kinematic Relation**:
  - For small angles ($\theta < 15^\circ$), arc displacement $s \approx x$.
  - Distance measured by sensor: $d(t) = d_{\text{eq}} - x(t)$.
  - Restoring factor: $k_{\text{eff}} = \frac{mg}{L}$.

```
                 [ Pivot Clamp ]
                       | \
       String/Rod (L)  |  \
                       |   \
                       |    \
        [ Sensor Pod ] |   [ Bob + Target Vane ]
       (Horizontal) -> |
```

---

### Configuration C: Horizontal Dynamics Track / Air Track Glider
- **Movement Axis**: 1D Horizontal Linear Track.
- **Sensor Orientation**: Mounted to the track end-stop, pointing horizontally down the track center axis.
- **Target Plate**: Mounted vertically on the trailing face of the glider / low-friction cart.
- **Advantages**: Eliminates gravitational sag and allows testing pure two-spring harmonic motion ($k_{\text{eff}} = k_1 + k_2$) without changing equilibrium height!

```
    [ Sensor Pod ]
      (End Stop) ======[ Spring 1 ]======[ Cart + Vane ]======[ Spring 2 ]======[ End Stop ]
   ========================================================================================
                                [ Low-Friction Track ]
```

---

### Configuration D: Torsional Oscillator (Rotary Disc)
- **Movement Axis**: Angular rotation $\theta(t)$ around vertical torsion wire.
- **Sensor Orientation**: Horizontal, aimed perpendicularly at a tangential eccentric paddle ($r = 10\text{ cm}$) attached to the rotating disc.
- **Kinematic Relation**:
  - Linear displacement $x(t) = r \cdot \theta(t)$.
  - Restoring torque: $\tau = -\kappa \theta \implies k_{\text{eff}} = \frac{\kappa}{r^2}$.
  - Inertia: $I = \frac{1}{2} M R^2$.

```
                  [ Torsion Wire ]
                         |
           +-------------O-------------+
           |       Rotating Disc       |
           +---------------------------+
                         \
                          [ Radial Arm (r) ]
                                  |
                           [ Vertical Vane ]
                                  ^
                                  | (Ultrasound)
                          [ Sensor Pod ]
```

---

### Configuration E: Free-Fall Drop Column
- **Movement Axis**: Vertical 1D downward trajectory under gravity.
- **Sensor Orientation**: Mounted at the top inside a vertical PVC or clear acrylic tube ($\varnothing 100\text{ mm}$ minimum to avoid acoustic sidewall reflections), pointing downward.
- **Release Mechanism**: 5V electromagnet controlled by an Arduino digital pin.
- **Measurement**:
  - $x(t) = \frac{1}{2} g t^2$.
  - Savitzky-Golay acceleration derivative directly measures local gravitational constant $g$ ($\approx 9.81\text{ m/s}^2$) with no stopwatch human error!

---

## 4. Hardware Calibration Checklist Before Every Run

1. **Plumb-Line Alignment**: Hang a weighted string from the pivot/spring to confirm the mass center aligns with the ultrasonic transducer center.
2. **Clear Cone Test**: Wave your hand in the $15^\circ$ beam corridor to confirm no bench clamps, cables, or walls create secondary echoes.
3. **Tare Baseline Check**: Send the `TARE` command over serial once the rig is settled at rest to zero the equilibrium reference.
4. **Room Temperature Check**: Measure ambient temperature and issue `TEMP:<degC>` so speed of sound scales accurately.
