"""
3D CAD Model Generator for Kinematics Oscillator Tracking Systems
Generates standard Wavefront .OBJ and .STL 3D models for all 5 oscillator configurations.
Compatible with Windows 3D Viewer, Blender, Fusion 360, Tinkercad, and 3D printing slicers.
"""

import os
import math

class MeshBuilder:
    def __init__(self, name="model"):
        self.name = name
        self.vertices = []
        self.normals = []
        self.faces = [] # 1-based vertex indices list of tuples

    def add_vertex(self, x, y, z):
        self.vertices.append((x, y, z))
        return len(self.vertices)

    def add_face(self, v_indices):
        self.faces.append(v_indices)

    def add_box(self, cx, cy, cz, sx, sy, sz):
        """Adds an axis-aligned 3D box centered at (cx, cy, cz) with dimensions (sx, sy, sz)."""
        hx, hy, hz = sx/2.0, sy/2.0, sz/2.0
        v = [
            self.add_vertex(cx - hx, cy - hy, cz - hz), # 0
            self.add_vertex(cx + hx, cy - hy, cz - hz), # 1
            self.add_vertex(cx + hx, cy + hy, cz - hz), # 2
            self.add_vertex(cx - hx, cy + hy, cz - hz), # 3
            self.add_vertex(cx - hx, cy - hy, cz + hz), # 4
            self.add_vertex(cx + hx, cy - hy, cz + hz), # 5
            self.add_vertex(cx + hx, cy + hy, cz + hz), # 6
            self.add_vertex(cx - hx, cy + hy, cz + hz), # 7
        ]
        # 6 faces (two triangles each)
        faces = [
            (v[0], v[2], v[1]), (v[0], v[3], v[2]), # -Z
            (v[4], v[5], v[6]), (v[4], v[6], v[7]), # +Z
            (v[0], v[1], v[5]), (v[0], v[5], v[4]), # -Y
            (v[3], v[6], v[2]), (v[3], v[7], v[6]), # +Y
            (v[0], v[4], v[7]), (v[0], v[7], v[3]), # -X
            (v[1], v[2], v[6]), (v[1], v[6], v[5]), # +X
        ]
        for f in faces:
            self.add_face(f)

    def add_cylinder(self, p1, p2, radius, segments=16):
        """Adds a cylinder from 3D point p1 to p2 with given radius."""
        x1, y1, z1 = p1
        x2, y2, z2 = p2
        dx, dy, dz = x2 - x1, y2 - y1, z2 - z1
        length = math.sqrt(dx*dx + dy*dy + dz*dz)
        if length < 1e-6:
            return

        # Basis vectors
        dir_v = (dx/length, dy/length, dz/length)
        # Find perpendicular vector safely
        if abs(dir_v[2]) > 0.9:
            up = (1.0, 0.0, 0.0)
        else:
            up = (0.0, 0.0, 1.0)
        
        # u = dir x up
        ux = dir_v[1]*up[2] - dir_v[2]*up[1]
        uy = dir_v[2]*up[0] - dir_v[0]*up[2]
        uz = dir_v[0]*up[1] - dir_v[1]*up[0]
        u_len = math.sqrt(ux*ux + uy*uy + uz*uz)
        if u_len < 1e-6:
            up = (0.0, 1.0, 0.0)
            ux = dir_v[1]*up[2] - dir_v[2]*up[1]
            uy = dir_v[2]*up[0] - dir_v[0]*up[2]
            uz = dir_v[0]*up[1] - dir_v[1]*up[0]
            u_len = math.sqrt(ux*ux + uy*uy + uz*uz)
        u = (ux/u_len, uy/u_len, uz/u_len)

        # v = dir x u
        vx = dir_v[1]*u[2] - dir_v[2]*u[1]
        vy = dir_v[2]*u[0] - dir_v[0]*u[2]
        vz = dir_v[0]*u[1] - dir_v[1]*u[0]
        v = (vx, vy, vz)

        bottom_ring = []
        top_ring = []
        for i in range(segments):
            angle = 2.0 * math.pi * i / segments
            cos_a = math.cos(angle) * radius
            sin_a = math.sin(angle) * radius

            bx = x1 + cos_a * u[0] + sin_a * v[0]
            by = y1 + cos_a * u[1] + sin_a * v[1]
            bz = z1 + cos_a * u[2] + sin_a * v[2]
            bottom_ring.append(self.add_vertex(bx, by, bz))

            tx = x2 + cos_a * u[0] + sin_a * v[0]
            ty = y2 + cos_a * u[1] + sin_a * v[1]
            tz = z2 + cos_a * u[2] + sin_a * v[2]
            top_ring.append(self.add_vertex(tx, ty, tz))

        # Side quads (as 2 triangles)
        for i in range(segments):
            next_i = (i + 1) % segments
            b1, b2 = bottom_ring[i], bottom_ring[next_i]
            t1, t2 = top_ring[i], top_ring[next_i]
            self.add_face((b1, t1, t2))
            self.add_face((b1, t2, b2))

        # Bottom cap center
        bc = self.add_vertex(x1, y1, z1)
        for i in range(segments):
            next_i = (i + 1) % segments
            self.add_face((bc, bottom_ring[next_i], bottom_ring[i]))

        # Top cap center
        tc = self.add_vertex(x2, y2, z2)
        for i in range(segments):
            next_i = (i + 1) % segments
            self.add_face((tc, top_ring[i], top_ring[next_i]))

    def add_helix_spring(self, p_top, p_bottom, radius=0.025, wire_radius=0.003, coils=12, segments_per_coil=12):
        """Builds a helical coil spring between p_top and p_bottom."""
        total_steps = coils * segments_per_coil
        x1, y1, z1 = p_top
        x2, y2, z2 = p_bottom
        dx, dy, dz = x2 - x1, y2 - y1, z2 - z1

        prev_pt = None
        for step in range(total_steps + 1):
            fraction = step / float(total_steps)
            angle = 2.0 * math.pi * coils * fraction

            cx = x1 + dx * fraction + radius * math.cos(angle)
            cy = y1 + dy * fraction
            cz = z1 + dz * fraction + radius * math.sin(angle)
            curr_pt = (cx, cy, cz)

            if prev_pt is not None:
                self.add_cylinder(prev_pt, curr_pt, wire_radius, segments=8)
            prev_pt = curr_pt

    def add_sphere(self, cx, cy, cz, radius, rings=12, sectors=16):
        """Adds a UV sphere."""
        vert_grid = []
        for r in range(rings + 1):
            phi = math.pi * r / float(rings)
            row = []
            for s in range(sectors):
                theta = 2.0 * math.pi * s / float(sectors)
                x = cx + radius * math.sin(phi) * math.cos(theta)
                y = cy + radius * math.cos(phi)
                z = cz + radius * math.sin(phi) * math.sin(theta)
                row.append(self.add_vertex(x, y, z))
            vert_grid.append(row)

        for r in range(rings):
            for s in range(sectors):
                next_s = (s + 1) % sectors
                v1 = vert_grid[r][s]
                v2 = vert_grid[r+1][s]
                v3 = vert_grid[r+1][next_s]
                v4 = vert_grid[r][next_s]
                self.add_face((v1, v2, v3))
                self.add_face((v1, v3, v4))

    def export_obj(self, filepath):
        """Exports mesh to Wavefront .OBJ file."""
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        with open(filepath, "w") as f:
            f.write(f"# 3D Model: {self.name}\n")
            f.write(f"# Automated Kinematics & Damped Motion Tracking Suite\n")
            for vx, vy, vz in self.vertices:
                f.write(f"v {vx:.5f} {vy:.5f} {vz:.5f}\n")
            for face in self.faces:
                f.write("f " + " ".join(str(idx) for idx in face) + "\n")
        print(f"[SUCCESS] Exported OBJ ({len(self.vertices)} verts, {len(self.faces)} faces) -> {filepath}")

    def export_stl(self, filepath):
        """Exports mesh to standard ASCII STL format for 3D printing and CAD."""
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        with open(filepath, "w") as f:
            f.write(f"solid {self.name}\n")
            for face in self.faces:
                if len(face) == 3:
                    v1 = self.vertices[face[0]-1]
                    v2 = self.vertices[face[1]-1]
                    v3 = self.vertices[face[2]-1]
                    
                    # Normal vector: (v2 - v1) x (v3 - v1)
                    ax, ay, az = v2[0]-v1[0], v2[1]-v1[1], v2[2]-v1[2]
                    bx, by, bz = v3[0]-v1[0], v3[1]-v1[1], v3[2]-v1[2]
                    nx = ay*bz - az*by
                    ny = az*bx - ax*bz
                    nz = ax*by - ay*bx
                    nlen = math.sqrt(nx*nx + ny*ny + nz*nz) or 1.0

                    f.write(f"  facet normal {nx/nlen:.4e} {ny/nlen:.4e} {nz/nlen:.4e}\n")
                    f.write("    outer loop\n")
                    f.write(f"      vertex {v1[0]:.4e} {v1[1]:.4e} {v1[2]:.4e}\n")
                    f.write(f"      vertex {v2[0]:.4e} {v2[1]:.4e} {v2[2]:.4e}\n")
                    f.write(f"      vertex {v3[0]:.4e} {v3[1]:.4e} {v3[2]:.4e}\n")
                    f.write("    endloop\n")
                    f.write("  endfacet\n")
            f.write(f"endsolid {self.name}\n")
        print(f"[SUCCESS] Exported STL -> {filepath}")


# =============================================================================
# 1. UNIVERSAL SENSOR POD MODEL
# =============================================================================
def build_sensor_pod():
    m = MeshBuilder("Universal_HCSR04_Sensor_Pod")
    # PCB Base (45 x 20 x 1.6 mm)
    m.add_box(0, 0, 0, 0.045, 0.020, 0.002)

    # Transducer Cylinders (TX and RX: 16mm diameter x 12mm depth)
    # Transmitter on left (-13mm), Receiver on right (+13mm)
    m.add_cylinder((-0.013, 0, 0.001), (-0.013, 0, 0.013), 0.008, segments=16)
    m.add_cylinder((0.013, 0, 0.001), (0.013, 0, 0.013), 0.008, segments=16)

    # Crystal oscillator (metal can)
    m.add_box(0, 0.005, 0.003, 0.010, 0.004, 0.003)

    # 4-pin header block on back
    m.add_box(0, -0.008, -0.004, 0.012, 0.003, 0.006)

    # Swivel ball-and-socket mount underneath
    m.add_sphere(0, 0, -0.015, 0.009, rings=10, sectors=12) # Ball joint
    m.add_cylinder((0, 0, -0.015), (0, 0, -0.035), 0.005, segments=12) # Stem
    m.add_box(0, 0, -0.040, 0.025, 0.025, 0.010) # 1/4"-20 rod clamp adapter
    return m


# =============================================================================
# 2. VERTICAL SPRING OSCILLATOR
# =============================================================================
def build_vertical_spring_system():
    m = MeshBuilder("Vertical_Spring_Oscillator_System")
    # Cast Iron Base (220 x 140 x 25 mm)
    m.add_box(0, 0.012, 0, 0.22, 0.025, 0.14)

    # Vertical Stainless Steel Rod (12mm dia x 650mm height)
    m.add_cylinder((-0.08, 0.025, 0), (-0.08, 0.65, 0), 0.006, segments=16)

    # Top Crossbar Clamp (Bosshead)
    m.add_box(-0.08, 0.62, 0, 0.03, 0.03, 0.03)
    # Horizontal Crossbar
    m.add_cylinder((-0.08, 0.62, 0), (0.05, 0.62, 0), 0.005, segments=12)
    # Suspension hook at (0.05, 0.62, 0)
    m.add_cylinder((0.05, 0.62, 0), (0.05, 0.60, 0), 0.002, segments=8)

    # Helical Extension Spring (from 0.60 down to 0.44 = 16cm stretched length)
    m.add_helix_spring((0.05, 0.60, 0), (0.05, 0.44, 0), radius=0.015, wire_radius=0.0018, coils=14)

    # Hooked Mass Set (200g slotted weights: 35mm dia x 40mm height)
    m.add_cylinder((0.05, 0.44, 0), (0.05, 0.38, 0), 0.018, segments=16)
    m.add_cylinder((0.05, 0.44, 0), (0.05, 0.45, 0), 0.002, segments=8) # Top hook

    # Acoustic Retro-Reflector Target Plate (60 x 60 x 2 mm disk)
    m.add_box(0.05, 0.375, 0, 0.065, 0.003, 0.065)

    # HC-SR04 Sensor Pod on Base (Upward Looking at x=0.05, y=0.05, z=0)
    m.add_box(0.05, 0.035, 0, 0.045, 0.004, 0.020)
    m.add_cylinder((0.037, 0.035, 0), (0.037, 0.050, 0), 0.008, segments=12) # TX
    m.add_cylinder((0.063, 0.035, 0), (0.063, 0.050, 0), 0.008, segments=12) # RX

    # Miniature Arduino Uno representation on base
    m.add_box(-0.02, 0.030, 0.04, 0.068, 0.012, 0.053)

    return m


# =============================================================================
# 3. SIMPLE PENDULUM SYSTEM
# =============================================================================
def build_pendulum_system():
    m = MeshBuilder("Simple_Pendulum_Tracking_System")
    # Cast Iron Base
    m.add_box(0, 0.012, 0, 0.25, 0.025, 0.16)

    # Vertical Rod
    m.add_cylinder((-0.10, 0.025, 0), (-0.10, 0.70, 0), 0.006, segments=16)

    # Pivot Clamp Arm
    m.add_cylinder((-0.10, 0.68, 0), (0.06, 0.68, 0), 0.005, segments=12)

    # Suspension Line (Fishing Line/Cord) swinging at ~12 deg angle
    pivot_pt = (0.06, 0.68, 0)
    # Deflected bob position
    bob_pt = (0.15, 0.22, 0)
    m.add_cylinder(pivot_pt, bob_pt, 0.001, segments=6)

    # Spherical Pendulum Bob (35mm dia)
    m.add_sphere(bob_pt[0], bob_pt[1], bob_pt[2], 0.018, rings=12, sectors=16)

    # Lightweight Acoustic Target Vane on Bob (facing horizontally towards sensor at x=0)
    m.add_box(bob_pt[0] - 0.020, bob_pt[1], bob_pt[2], 0.002, 0.055, 0.055)

    # Horizontally Aimed Sensor Pod on Stand Rod (Aiming at Bob Equilibrium)
    sensor_x = -0.02
    sensor_y = 0.22
    m.add_cylinder((-0.10, sensor_y, 0), (sensor_x - 0.02, sensor_y, 0), 0.004, segments=8) # Mount arm
    m.add_box(sensor_x - 0.01, sensor_y, 0, 0.004, 0.020, 0.045)
    # TX & RX pointing along +X towards the bob
    m.add_cylinder((sensor_x - 0.01, sensor_y, -0.013), (sensor_x + 0.005, sensor_y, -0.013), 0.008, segments=12)
    m.add_cylinder((sensor_x - 0.01, sensor_y, 0.013), (sensor_x + 0.005, sensor_y, 0.013), 0.008, segments=12)

    return m


# =============================================================================
# 4. HORIZONTAL DYNAMICS TRACK / CART SYSTEM
# =============================================================================
def build_horizontal_cart_system():
    m = MeshBuilder("Horizontal_Dynamics_Track_System")
    # Low-Friction Track Rail (600mm long x 80mm wide x 20mm high)
    m.add_box(0.20, 0.01, 0, 0.60, 0.02, 0.08)

    # Track Leveling Feet
    m.add_box(-0.08, -0.005, 0, 0.03, 0.01, 0.10)
    m.add_box(0.48, -0.005, 0, 0.03, 0.01, 0.10)

    # Track End-Stop Bumper at left (-0.09) and right (+0.49)
    m.add_box(-0.09, 0.04, 0, 0.015, 0.06, 0.08)
    m.add_box(0.49, 0.04, 0, 0.015, 0.06, 0.08)

    # Dynamics Cart (140mm length x 70mm width x 40mm height)
    cart_x = 0.22 # Cart position
    m.add_box(cart_x, 0.045, 0, 0.14, 0.035, 0.065)

    # Cart Wheels (4 wheels)
    m.add_cylinder((cart_x - 0.045, 0.020, -0.038), (cart_x - 0.045, 0.020, 0.038), 0.012, segments=12)
    m.add_cylinder((cart_x + 0.045, 0.020, -0.038), (cart_x + 0.045, 0.020, 0.038), 0.012, segments=12)

    # Opposing Springs (Spring 1: Endstop to Cart; Spring 2: Cart to Endstop)
    m.add_helix_spring((-0.08, 0.045, 0), (cart_x - 0.07, 0.045, 0), radius=0.010, wire_radius=0.0012, coils=16)
    m.add_helix_spring((cart_x + 0.07, 0.045, 0), (0.48, 0.045, 0), radius=0.010, wire_radius=0.0012, coils=16)

    # Vertical Acoustic Target Plate on Cart Facing Sensor (-X face)
    m.add_box(cart_x - 0.072, 0.060, 0, 0.003, 0.055, 0.055)

    # HC-SR04 Sensor Pod Clamped to Left End-Stop (Aiming axially down the track)
    m.add_box(-0.11, 0.060, 0, 0.004, 0.020, 0.045)
    m.add_cylinder((-0.11, 0.060, -0.013), (-0.095, 0.060, -0.013), 0.008, segments=12)
    m.add_cylinder((-0.11, 0.060, 0.013), (-0.095, 0.060, 0.013), 0.008, segments=12)

    return m


# =============================================================================
# 5. TORSIONAL OSCILLATOR SYSTEM
# =============================================================================
def build_torsion_system():
    m = MeshBuilder("Torsional_Rotary_Oscillator_System")
    # Base
    m.add_box(0, 0.012, 0, 0.22, 0.025, 0.14)
    # Upright Column
    m.add_cylinder((-0.09, 0.025, 0), (-0.09, 0.55, 0), 0.006, segments=16)
    # Top Clamp
    m.add_cylinder((-0.09, 0.52, 0), (0.05, 0.52, 0), 0.005, segments=12)

    # High-tensile steel torsion wire (from 0.52 down to 0.25)
    m.add_cylinder((0.05, 0.52, 0), (0.05, 0.25, 0), 0.0012, segments=8)

    # Heavy Brass Inertia Disc (160mm dia x 15mm thick)
    m.add_cylinder((0.05, 0.25, 0), (0.05, 0.235, 0), 0.080, segments=24)

    # Tangential Target Vane Arm on Rim (at radius r = 75mm)
    vane_x = 0.05 + 0.075 * math.cos(0.3)
    vane_z = 0.075 * math.sin(0.3)
    m.add_cylinder((0.05, 0.24, 0), (vane_x, 0.24, vane_z), 0.002, segments=6)
    # Flat Acoustic Target Vane
    m.add_box(vane_x, 0.26, vane_z, 0.050, 0.050, 0.002)

    # Sensor Pod Pointed at Tangential Vane
    sensor_z = vane_z + 0.18
    m.add_cylinder((-0.09, 0.26, sensor_z), (vane_x, 0.26, sensor_z), 0.004, segments=8)
    m.add_box(vane_x, 0.26, sensor_z + 0.01, 0.045, 0.020, 0.004)
    m.add_cylinder((vane_x - 0.013, 0.26, sensor_z + 0.01), (vane_x - 0.013, 0.26, sensor_z - 0.005), 0.008, segments=12)
    m.add_cylinder((vane_x + 0.013, 0.26, sensor_z + 0.01), (vane_x + 0.013, 0.26, sensor_z - 0.005), 0.008, segments=12)

    return m


# =============================================================================
# 6. FREE-FALL KINEMATICS DROP COLUMN
# =============================================================================
def build_freefall_system():
    m = MeshBuilder("Freefall_Kinematics_Drop_Column")
    # Base with soft foam landing pad
    m.add_box(0, 0.012, 0, 0.24, 0.025, 0.16)
    m.add_box(0, 0.035, 0, 0.12, 0.020, 0.12) # Cushion pad

    # Vertical Guide Column (100mm Clear Tube / Rails, 700mm height)
    m.add_cylinder((-0.07, 0.025, 0), (-0.07, 0.72, 0), 0.006, segments=16)

    # Top Cap with Electromagnet Release (at y=0.70)
    m.add_box(0, 0.71, 0, 0.09, 0.025, 0.06)
    m.add_cylinder((0, 0.70, 0), (0, 0.68, 0), 0.015, segments=16) # Electromagnet coil

    # Downward-Looking HC-SR04 Sensor Pod at top (next to electromagnet)
    m.add_box(0, 0.67, 0.035, 0.045, 0.004, 0.020)
    m.add_cylinder((-0.013, 0.67, 0.035), (-0.013, 0.655, 0.035), 0.008, segments=12)
    m.add_cylinder((0.013, 0.67, 0.035), (0.013, 0.655, 0.035), 0.008, segments=12)

    # Falling Target Plate & Mass (frozen at mid-drop y=0.40)
    m.add_cylinder((0, 0.40, 0.035), (0, 0.42, 0.035), 0.015, segments=16) # Steel mass
    m.add_box(0, 0.425, 0.035, 0.065, 0.003, 0.065) # Acoustic target plate on top!

    return m


def generate_all_models():
    output_dir = os.path.dirname(os.path.abspath(__file__))

    models = [
        ("universal_sensor_pod", build_sensor_pod()),
        ("oscillator_vertical_spring", build_vertical_spring_system()),
        ("oscillator_simple_pendulum", build_pendulum_system()),
        ("oscillator_horizontal_cart", build_horizontal_cart_system()),
        ("oscillator_torsion_disc", build_torsion_system()),
        ("oscillator_freefall_column", build_freefall_system())
    ]

    for filename, mesh in models:
        obj_path = os.path.join(output_dir, f"{filename}.obj")
        stl_path = os.path.join(output_dir, f"{filename}.stl")
        mesh.export_obj(obj_path)
        mesh.export_stl(stl_path)

    print("\n[COMPLETE] All 3D CAD models generated successfully!")


if __name__ == "__main__":
    generate_all_models()
