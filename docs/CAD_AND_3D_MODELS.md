# 3D CAD Models & Interactive Viewer Suite

This directory contains production-ready 3D CAD models (`.obj` and `.stl` formats) for the **Automated Kinematics and Damped Harmonic Motion** tracking system across all 5 oscillator configurations, plus the standalone Universal Sensor Pod.

---

## 1. 3D Model Catalog

| Model Name | Files (.OBJ / .STL) | Description | Key Components Included |
| :--- | :--- | :--- | :--- |
| **Universal Sensor Pod** | [`universal_sensor_pod.obj`](file:///C:/Users/abhiy/.gemini/antigravity/scratch/automated-kinematics/cad/universal_sensor_pod.obj)<br>[`universal_sensor_pod.stl`](file:///C:/Users/abhiy/.gemini/antigravity/scratch/automated-kinematics/cad/universal_sensor_pod.stl) | Modular bracket for the HC-SR04 transducer | 45x20mm PCB, 16mm TX/RX transducer cans, crystal oscillator, 4-pin header, ball-and-socket swivel joint, 1/4"-20 rod clamp adapter. |
| **Vertical Spring Oscillator** | [`oscillator_vertical_spring.obj`](file:///C:/Users/abhiy/.gemini/antigravity/scratch/automated-kinematics/cad/oscillator_vertical_spring.obj)<br>[`oscillator_vertical_spring.stl`](file:///C:/Users/abhiy/.gemini/antigravity/scratch/automated-kinematics/cad/oscillator_vertical_spring.stl) | 1D Vertical Spring-Mass Rig | Cast-iron base, 650mm upright rod, top bosshead crossbar clamp, 14-coil helical spring, slotted brass weight set, 60x60mm retro-reflector plate, upward-aimed sensor pod, Arduino Uno. |
| **Simple Pendulum** | [`oscillator_simple_pendulum.obj`](file:///C:/Users/abhiy/.gemini/antigravity/scratch/automated-kinematics/cad/oscillator_simple_pendulum.obj)<br>[`oscillator_simple_pendulum.stl`](file:///C:/Users/abhiy/.gemini/antigravity/scratch/automated-kinematics/cad/oscillator_simple_pendulum.stl) | Circular Arc Pendulum Rig | Ring stand, suspension arm clamp, nylon cord, 35mm spherical brass bob, vertical acoustic reflector vane, horizontally-mounted sensor pod aimed at equilibrium. |
| **Horizontal Dynamics Cart** | [`oscillator_horizontal_cart.obj`](file:///C:/Users/abhiy/.gemini/antigravity/scratch/automated-kinematics/cad/oscillator_horizontal_cart.obj)<br>[`oscillator_horizontal_cart.stl`](file:///C:/Users/abhiy/.gemini/antigravity/scratch/automated-kinematics/cad/oscillator_horizontal_cart.stl) | Linear Dynamics Track Rig | 600mm low-friction aluminum track rail, end-stop bumpers, dual opposing springs ($k_1 + k_2$), wheeled glider cart, vertical reflector plate, axial sensor pod. |
| **Torsional Rotary Oscillator** | [`oscillator_torsion_disc.obj`](file:///C:/Users/abhiy/.gemini/antigravity/scratch/automated-kinematics/cad/oscillator_torsion_disc.obj)<br>[`oscillator_torsion_disc.stl`](file:///C:/Users/abhiy/.gemini/antigravity/scratch/automated-kinematics/cad/oscillator_torsion_disc.stl) | Angular $\theta$-Axis Rig | Vertical high-tensile torsion wire, 160mm heavy brass inertia disc, radial arm ($r = 75\text{ mm}$) with vertical target vane, tangential sensor pod. |
| **Free-Fall Kinematics Column** | [`oscillator_freefall_column.obj`](file:///C:/Users/abhiy/.gemini/antigravity/scratch/automated-kinematics/cad/oscillator_freefall_column.obj)<br>[`oscillator_freefall_column.stl`](file:///C:/Users/abhiy/.gemini/antigravity/scratch/automated-kinematics/cad/oscillator_freefall_column.stl) | Vertical Acceleration $g$ Column | Soft foam landing pad, upright guide rod, top electromagnet release trigger, steel falling mass with top target disk, downward-aimed top sensor pod. |

---

## 2. Interactive 3D WebGL Viewer

You can open the complete interactive 3D model viewer directly in your web browser:
🌐 **[interactive_3d_viewer.html](file:///C:/Users/abhiy/.gemini/antigravity/scratch/automated-kinematics/cad/interactive_3d_viewer.html)**

### Features:
- **360° Free Orbit Rotation**: Click and drag with Left Mouse Button to inspect from any angle.
- **Pan & Zoom**: Right click + drag to pan, Scroll wheel to zoom in/out.
- **Model Switching**: Tabs to jump between Spring, Pendulum, Cart, Torsion, Free-Fall, and Sensor Pod.
- **Real-Time Physics Animation**: Watch the mechanical oscillations animate in 3D.
- **Ultrasonic Raycast Simulation**: Visualize the $40\text{ kHz}$ acoustic beam cone and echo reflections traveling between the HC-SR04 and the target plate.
- **3D Component Labels**: Inspect callout labels for each part.

---

## 3. How to Open the CAD Files

### Option A: Windows 3D Viewer (Built into Windows 10/11)
Simply double-click any `.obj` or `.stl` file in Windows Explorer. Windows 3D Viewer will open it with full lighting and texture controls.

### Option B: CAD / 3D Modeling Software
- **Blender**: `File > Import > Wavefront (.obj)` or `Stl (.stl)`.
- **Autodesk Fusion 360**: `Insert > Insert Mesh`.
- **Tinkercad**: Click `Import` in the top right and drag any `.stl` file.
- **FreeCAD**: `File > Import`.

### Option C: 3D Printing & Slicing
To 3D-print the **Universal Sensor Pod Bracket** or **Target Plates**:
1. Open your slicer (UltiMaker Cura, PrusaSlicer, Bambu Studio, or Creality Print).
2. Load [`universal_sensor_pod.stl`](file:///C:/Users/abhiy/.gemini/antigravity/scratch/automated-kinematics/cad/universal_sensor_pod.stl).
3. Recommended print settings:
   - Infill: 20% - 30% (Gyroid or Grid)
   - Layer height: 0.20 mm
   - Material: PLA or PETG
   - Supports: Touching buildplate only
