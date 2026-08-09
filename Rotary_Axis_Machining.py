"""
Brian Rotary CAM - surface-normal v0.11

Purpose
-------
Convert a radially machinable STL into a simple Mach3 X/A/Z serpentine toolpath.

IMPORTANT:
- This is a geometry/toolpath proof of concept, not production CAM.
- It assumes the part rotates about the X axis.
- Z=0 is the ROTARY CENTRELINE.
- The outside of the cylindrical stock is at Z = stock radius.
- Surface Z values are therefore positive radii measured from the centreline.
- Simple radial ballnose compensation is included (surface radius + R).
- True surface-normal/taper compensation is not included yet.
- Undercuts/internal radial surfaces are intentionally ignored.
- Always inspect the output in Mach3 and test above the work before cutting.

Dependencies:
    pip install numpy trimesh

SciPy is NOT required in v0.3.

The STL does NOT need to be watertight if the radial surface itself is complete.
"""

import math
from pathlib import Path

import numpy as np
import trimesh

import tkinter as tk
from tkinter import filedialog, simpledialog, ttk, messagebox


# ============================================================
# USER VARIABLES
# ============================================================

# File selection
# When True, the script opens a normal file chooser at startup.
USE_FILE_DIALOG = True

# Used only if USE_FILE_DIALOG = False.
STL_FILE = "Brian;'s Handle.stl"

# If left as None, the G-code file is automatically named from the STL,
# e.g. "My Handle.stl" -> "My Handle_rotary.tap"
OUTPUT_FILE = None

# Geometry
STOCK_DIAMETER_MM = None        # Optional fallback if dialogs are disabled
AUTO_STOCK_ALLOWANCE_MM = 5.0      # If stock diameter left blank: STL max diameter + 5 mm
X_STEP_MM = 1.0                 # Sampling distance along handle
A_STEP_DEG = 1.0                # Rotary increment; try 0.5 later for a finer finish
GCODE_Z_TOLERANCE_MM = 0.01      # Simplification tolerance
GCODE_A_TOLERANCE_DEG = 0.02     # Preserve meaningful normal-compensation A shifts

# Cutter defaults shown in the startup dialog
DEFAULT_BALL_RADIUS_MM = 1.5
DEFAULT_CUTTING_EDGE_LENGTH_MM = 35.0
DEFAULT_MIN_CUTTER_Z_MM = 5.0

# Rotary centre in STL coordinates.
# Brian's model is already essentially centred on Y=0, Z=0.
ROTARY_CENTRE_Y = 0.0
ROTARY_CENTRE_Z = 0.0

# Start angle in STL coordinates.
A_START_DEG = 0.0

# Rotary coordinate convention
#
# The cutter approaches the work along +Z toward the X-axis.
# Therefore machine A=0 corresponds to the +Z radial direction in the STL,
# not +Y.  A positive machine rotation about +X (right-hand rule) means
# the STL sampling angle moves in the opposite direction.
#
# STL sampling angle convention used internally:
#   0 deg   = +Y
#   90 deg  = +Z
#
# For a conventional A axis:
#   sample_angle = A_ZERO_RAY_DEG - A_DIRECTION * machine_A
A_ZERO_RAY_DEG = 90.0
A_DIRECTION = 1.0       # Change to -1.0 if Brian's A axis rotates oppositely

# G-code setup
SAFE_CLEARANCE_MM = 5.0         # Clearance above cylindrical stock OD
CUT_FEED = 800.0                # Trial only - set appropriately for Brian's machine
RAPID_FEED = None               # G0 used; value not required

# First meridian / entry channel.
# The first angular line is progressively opened in depth-limited passes.
# Only this one narrow channel is "roughed"; subsequent A passes go to the surface.
ENTRY_DEPTH_PER_PASS_MM = 2.0

# Safety / diagnostics
MAX_ALLOWED_ADJACENT_RADIAL_CHANGE_MM = 1.0
ABORT_IF_RADIAL_CHANGE_EXCEEDED = False

# Small distance inside each open STL end so slicing is reliable.
END_INSET_MM = 0.10


# ============================================================
# FILE SELECTION
# ============================================================

def choose_stl_file():
    """Open a standard desktop file dialog and return the selected STL path."""
    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)

    filename = filedialog.askopenfilename(
        title="Choose STL for rotary machining",
        filetypes=[
            ("STL files", "*.stl"),
            ("All files", "*.*"),
        ],
    )

    root.destroy()

    if not filename:
        raise SystemExit("No STL selected.")

    return Path(filename)


def get_cutter_parameters():
    """
    Ask for the two cutter values most likely to change.

    R = ballnose radius.
    L = usable cutting-edge length.

    L is currently used as a safety/checking value only.
    """
    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)

    radius = simpledialog.askfloat(
        "Rotary CAM - Cutter",
        "Ballnose radius R (mm):",
        initialvalue=DEFAULT_BALL_RADIUS_MM,
        minvalue=0.0,
        parent=root,
    )
    if radius is None:
        root.destroy()
        raise SystemExit("Cancelled.")

    cutting_length = simpledialog.askfloat(
        "Rotary CAM - Cutter",
        "Cutting-edge length L (mm):",
        initialvalue=DEFAULT_CUTTING_EDGE_LENGTH_MM,
        minvalue=0.1,
        parent=root,
    )
    if cutting_length is None:
        root.destroy()
        raise SystemExit("Cancelled.")

    min_cutter_z = simpledialog.askfloat(
        "Rotary CAM - Cutter",
        "Minimum cutter-centre Z (mm):",
        initialvalue=DEFAULT_MIN_CUTTER_Z_MM,
        minvalue=0.0,
        parent=root,
    )
    if min_cutter_z is None:
        root.destroy()
        raise SystemExit("Cancelled.")

    stock_text = simpledialog.askstring(
        "Rotary CAM - Stock",
        "Stock diameter (mm):\n"
        "Leave blank for STL max diameter + "
        f"{AUTO_STOCK_ALLOWANCE_MM:.1f} mm",
        initialvalue="",
        parent=root,
    )
    if stock_text is None:
        root.destroy()
        raise SystemExit("Cancelled.")

    stock_text = stock_text.strip()
    if stock_text == "":
        stock_diameter = None
    else:
        try:
            stock_diameter = float(stock_text)
        except ValueError:
            root.destroy()
            raise ValueError("Stock diameter must be a number or left blank.")

        if stock_diameter <= 0:
            root.destroy()
            raise ValueError("Stock diameter must be greater than zero.")

    root.destroy()
    return (
        float(radius),
        float(cutting_length),
        float(min_cutter_z),
        stock_diameter,
    )


def choose_output_path(stl_path):
    """
    Return the output G-code path.
    If OUTPUT_FILE is None, save beside the STL using an automatic name.
    """
    if OUTPUT_FILE:
        return Path(OUTPUT_FILE)

    return stl_path.with_name(stl_path.stem + "_rotary.tap")


# ============================================================
# GEOMETRY FUNCTIONS
# ============================================================

def cross2(a, b):
    """2D cross product, vectorised for (...,2) arrays."""
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def section_segments_yz(mesh, x, centre_y, centre_z):
    """
    Intersect every STL triangle directly with the plane X = constant.

    This deliberately avoids trimesh.section(), because that path-building
    routine pulls in SciPy.  For our rotary CAM purpose we only need the raw
    2D line segments produced where triangles cross the section plane.

    Returns arrays of segment start/end points in local Y/Z coordinates.
    """
    triangles = np.asarray(mesh.triangles, dtype=float)
    eps = 1e-10

    starts = []
    ends = []

    for tri in triangles:
        d = tri[:, 0] - x

        # Entire triangle is clearly on one side of the slicing plane.
        if np.all(d > eps) or np.all(d < -eps):
            continue

        points = []

        # Check the three triangle edges.
        for i0, i1 in ((0, 1), (1, 2), (2, 0)):
            p0 = tri[i0]
            p1 = tri[i1]
            d0 = p0[0] - x
            d1 = p1[0] - x

            # Vertex lies on plane.
            if abs(d0) <= eps:
                points.append(p0[1:3])

            # Proper crossing of the plane.
            if (d0 < -eps and d1 > eps) or (d0 > eps and d1 < -eps):
                t = (x - p0[0]) / (p1[0] - p0[0])
                p = p0 + t * (p1 - p0)
                points.append(p[1:3])

        if not points:
            continue

        # Remove duplicate intersection points caused by vertices on the plane.
        unique = []
        for pt in points:
            pt = np.asarray(pt, dtype=float)
            if not any(np.linalg.norm(pt - q) < 1e-8 for q in unique):
                unique.append(pt)

        if len(unique) >= 2:
            a = unique[0] - np.array([centre_y, centre_z], dtype=float)
            b = unique[1] - np.array([centre_y, centre_z], dtype=float)

            # Ignore zero-length numerical fragments.
            if np.linalg.norm(b - a) > 1e-9:
                starts.append(a)
                ends.append(b)

    if not starts:
        return None, None

    return np.asarray(starts), np.asarray(ends)


def radii_for_section(mesh, x, angles_deg, centre_y, centre_z):
    """
    For one X position, cast radial rays from the rotary axis outwards
    and return the OUTERMOST surface radius for every requested angle.

    This is the cylindrical equivalent of reading one column of a height map.
    """
    a, b = section_segments_yz(mesh, x, centre_y, centre_z)

    if a is None:
        return np.full(len(angles_deg), np.nan)

    seg = b - a
    result = np.full(len(angles_deg), np.nan, dtype=float)

    for i, angle_deg in enumerate(angles_deg):
        angle = math.radians(float(angle_deg))

        # 0 degrees points along +Y; +90 degrees points along +Z.
        direction = np.array([math.cos(angle), math.sin(angle)], dtype=float)
        dirs = np.broadcast_to(direction, seg.shape)

        denominator = cross2(dirs, seg)
        nonparallel = np.abs(denominator) > 1e-12

        t = np.full(len(a), np.nan)
        u = np.full(len(a), np.nan)

        # Ray:       p = t * direction
        # Segment:   p = a + u * seg
        t[nonparallel] = (
            cross2(a[nonparallel], seg[nonparallel])
            / denominator[nonparallel]
        )

        u[nonparallel] = (
            cross2(a[nonparallel], dirs[nonparallel])
            / denominator[nonparallel]
        )

        valid = (
            nonparallel
            & (t >= 0.0)
            & (u >= -1e-9)
            & (u <= 1.0 + 1e-9)
        )

        intersections = t[valid]

        if len(intersections):
            # Outermost hit is the machinable radial envelope.
            result[i] = np.max(intersections)

    return result


def build_radius_map(mesh, x_positions, angles_deg):
    """Create radius_map[x_index, angle_index]."""
    radius_map = np.empty((len(x_positions), len(angles_deg)), dtype=float)

    print("Sampling STL surface...")

    for i, x in enumerate(x_positions):
        radius_map[i, :] = radii_for_section(
            mesh,
            x,
            angles_deg,
            ROTARY_CENTRE_Y,
            ROTARY_CENTRE_Z,
        )

        if i % 25 == 0 or i == len(x_positions) - 1:
            print(f"  X section {i + 1} / {len(x_positions)}")

    return radius_map


# ============================================================
# G-CODE FUNCTIONS
# ============================================================


def compute_surface_normal_toolpath(
    x_surface,
    machine_angles_deg,
    sample_angles_deg,
    radius_map,
    ball_radius,
    min_cutter_z,
    fallback_mask=None,
):
    """
    Convert sampled surface radii into true ball-centre coordinates.

    Surface parameterisation:
        P(x, theta) = [x, r*cos(theta), r*sin(theta)]

    The local outward surface normal is derived from central differences of the
    cylindrical radius map.  The ball centre is then:
        C = P + R * n

    C is converted back to the machine's X / A / Z coordinates.

    Missing/clipped samples can be marked in fallback_mask; those points use the
    proven simple radial compensation instead.
    """
    r = np.asarray(radius_map, dtype=float)
    x = np.asarray(x_surface, dtype=float)
    theta = np.radians(np.asarray(sample_angles_deg, dtype=float))

    nx_count, na_count = r.shape

    # dr/dx
    dr_dx = np.empty_like(r)
    if nx_count == 1:
        dr_dx[:] = 0.0
    else:
        dr_dx[0, :] = (r[1, :] - r[0, :]) / max(x[1] - x[0], 1e-12)
        dr_dx[-1, :] = (r[-1, :] - r[-2, :]) / max(x[-1] - x[-2], 1e-12)
        if nx_count > 2:
            dx_span = (x[2:] - x[:-2])[:, None]
            dr_dx[1:-1, :] = (r[2:, :] - r[:-2, :]) / np.maximum(dx_span, 1e-12)

    # dr/dtheta with wrap-around.
    if na_count == 1:
        dr_dtheta = np.zeros_like(r)
    else:
        # All A steps are uniform in this application.
        dtheta = math.radians(abs(float(machine_angles_deg[1] - machine_angles_deg[0])))
        dtheta = max(dtheta, 1e-12)
        # sample theta runs opposite machine A when A_DIRECTION=+1.
        theta_sign = -float(A_DIRECTION)
        dr_dtheta = (
            np.roll(r, -1, axis=1) - np.roll(r, 1, axis=1)
        ) / (2.0 * dtheta * theta_sign)

    # Mild derivative smoothing only; do NOT smooth the actual surface radius.
    dr_dx = (
        dr_dx
        + np.roll(dr_dx, 1, axis=1)
        + np.roll(dr_dx, -1, axis=1)
    ) / 3.0
    dr_dtheta = (
        dr_dtheta
        + np.roll(dr_dtheta, 1, axis=0)
        + np.roll(dr_dtheta, -1, axis=0)
    ) / 3.0

    th = theta[None, :]
    cos_t = np.cos(th)
    sin_t = np.sin(th)

    # P_x
    px_x = np.ones_like(r)
    px_y = dr_dx * cos_t
    px_z = dr_dx * sin_t

    # P_theta
    pt_x = np.zeros_like(r)
    pt_y = dr_dtheta * cos_t - r * sin_t
    pt_z = dr_dtheta * sin_t + r * cos_t

    # Outward normal = P_theta x P_x.
    n_x = pt_y * px_z - pt_z * px_y
    n_y = pt_z * px_x - pt_x * px_z
    n_z = pt_x * px_y - pt_y * px_x

    norm = np.sqrt(n_x*n_x + n_y*n_y + n_z*n_z)
    norm = np.maximum(norm, 1e-12)
    n_x /= norm
    n_y /= norm
    n_z /= norm

    # Surface coordinates.
    surf_x = x[:, None] * np.ones((1, na_count))
    surf_y = r * cos_t
    surf_z = r * sin_t

    # True ball centre.
    cx = surf_x + ball_radius * n_x
    cy = surf_y + ball_radius * n_y
    cz = surf_z + ball_radius * n_z

    tool_radius = np.sqrt(cy*cy + cz*cz)
    tool_theta = np.arctan2(cz, cy)

    # Convert compensated angular position back to machine A, but preserve
    # continuity near the nominal pass angle instead of wrapping 0/360.
    surface_theta = th
    delta_theta = np.arctan2(
        np.sin(tool_theta - surface_theta),
        np.cos(tool_theta - surface_theta),
    )
    tool_a = (
        np.asarray(machine_angles_deg, dtype=float)[None, :]
        - A_DIRECTION * np.degrees(delta_theta)
    )

    # Simple radial fallback for clipped / unreliable locations.
    if fallback_mask is not None:
        fallback_mask = np.asarray(fallback_mask, dtype=bool)
        radial_z = r + ball_radius
        nominal_a = np.asarray(machine_angles_deg, dtype=float)[None, :]
        cx = np.where(fallback_mask, surf_x, cx)
        tool_a = np.where(fallback_mask, nominal_a, tool_a)
        tool_radius = np.where(fallback_mask, radial_z, tool_radius)

    tool_radius = np.maximum(tool_radius, min_cutter_z)

    return cx, tool_a, tool_radius


def write_toolpath_pass(f, x_profile, a_profile, z_profile, reverse=False):
    """
    Write one X/A/Z pass with conservative 0.01 mm simplification.

    A point is skipped only while BOTH Z and A remain effectively unchanged.
    This preserves the small A corrections introduced by true normal
    compensation.
    """
    if reverse:
        indexes = list(range(len(x_profile) - 1, -1, -1))
    else:
        indexes = list(range(len(x_profile)))

    if not indexes:
        return

    i0 = indexes[0]
    f.write(
        f"G1 X{x_profile[i0]:.3f} A{a_profile[i0]:.4f} "
        f"Z{z_profile[i0]:.3f} F{CUT_FEED:.1f}\n"
    )

    ref_z = float(z_profile[i0])
    ref_a = float(a_profile[i0])
    last_skipped = None

    for pos in range(1, len(indexes)):
        i = indexes[pos]
        z = float(z_profile[i])
        a = float(a_profile[i])
        is_last = (pos == len(indexes) - 1)

        same_z = abs(z - ref_z) <= GCODE_Z_TOLERANCE_MM
        same_a = abs(a - ref_a) <= GCODE_A_TOLERANCE_DEG

        if same_z and same_a and not is_last:
            last_skipped = i
            continue

        if last_skipped is not None:
            k = last_skipped
            f.write(
                f"G1 X{x_profile[k]:.3f} A{a_profile[k]:.4f} "
                f"Z{z_profile[k]:.3f}\n"
            )
            last_skipped = None

        f.write(
            f"G1 X{x_profile[i]:.3f} A{a_profile[i]:.4f} "
            f"Z{z_profile[i]:.3f}\n"
        )
        ref_z = z
        ref_a = a


def generate_gcode(
    output_path,
    x_relative,
    machine_angles_deg,
    sample_angles_deg,
    radius_map,
    fallback_mask,
    stock_radius,
    ball_radius,
    cutting_edge_length,
    min_cutter_z,
):
    """
    Generate progressive entry plus continuous serpentine rotary machining
    using true surface-normal ballnose compensation.
    """

    x_map, a_map, z_map = compute_surface_normal_toolpath(
        x_relative,
        machine_angles_deg,
        sample_angles_deg,
        radius_map,
        ball_radius,
        min_cutter_z,
        fallback_mask=fallback_mask,
    )

    # Keep the tool centre outside the original cylindrical stock during entry.
    stock_tool_radius = stock_radius + ball_radius

    first_x = x_map[:, 0]
    first_a = a_map[:, 0]
    first_z = z_map[:, 0]
    minimum_entry_radius = float(np.min(first_z))

    entry_levels = []
    level = stock_tool_radius - ENTRY_DEPTH_PER_PASS_MM
    while level > minimum_entry_radius:
        entry_levels.append(level)
        level -= ENTRY_DEPTH_PER_PASS_MM
    entry_levels.append(minimum_entry_radius)

    with open(output_path, "w", newline="\n") as f:
        f.write("(Brian Rotary CAM surface-normal v0.11)\n")
        f.write("(WARNING: inspect in Mach3 and air-cut before machining)\n")
        f.write("(True ballnose compensation derived from cylindrical surface normal)\n")
        f.write("(X = longitudinal axis, A = rotary axis, Z0 = rotary centreline)\n")
        f.write(f"(Stock diameter: {stock_radius * 2.0:.3f} mm)\n")
        f.write(f"(Ballnose radius R: {ball_radius:.3f} mm)\n")
        f.write(f"(Cutting-edge length L: {cutting_edge_length:.3f} mm)\n")
        f.write(f"(Minimum cutter-centre Z: {min_cutter_z:.3f} mm)\n")
        f.write("(Undercuts/internal radial surfaces ignored; outer envelope used)\n")
        f.write(f"(X step: {X_STEP_MM:.3f} mm, A step: {A_STEP_DEG:.4f} deg)\n")
        f.write(f"(G-code tolerance: Z {GCODE_Z_TOLERANCE_MM:.3f} mm, A {GCODE_A_TOLERANCE_DEG:.3f} deg)\n")
        f.write("\nG21\nG90\nG94\n")
        f.write("S20000\n")
        f.write("M03\n")

        safe_z = stock_tool_radius + SAFE_CLEARANCE_MM
        f.write(f"G0 Z{safe_z:.3f}\n")
        f.write(f"G0 A{first_a[0]:.4f}\n")
        f.write(f"G0 X{first_x[0]:.3f}\n")
        f.write(f"G0 Z{stock_tool_radius:.3f}\n\n")

        f.write("(Progressive first-meridian entry channel)\n")
        reverse = False

        for level in entry_levels:
            limited_z = np.maximum(first_z, level)
            write_toolpath_pass(
                f, first_x, first_a, limited_z, reverse=reverse
            )
            reverse = not reverse

        f.write("\n(Continuous surface-normal rotary passes)\n")

        for j in range(1, len(machine_angles_deg)):
            write_toolpath_pass(
                f,
                x_map[:, j],
                a_map[:, j],
                z_map[:, j],
                reverse=reverse,
            )
            reverse = not reverse

        f.write(f"\nG0 Z{safe_z:.3f}\n")
        f.write("M30\n")

    return x_map, a_map, z_map


def estimate_gcode_time(output_path, default_feed):
    """
    Rough machining-time estimate based on G1 X/Z travel only.

    Rotary A movement is intentionally ignored: in this application it is
    normally a very small increment between long X traverses and contributes
    little to the overall elapsed time.

    Returns (minutes, g1_moves, xz_distance_mm).
    """
    current_x = None
    current_z = None
    current_feed = float(default_feed)

    total_distance = 0.0
    total_minutes = 0.0
    g1_moves = 0

    with open(output_path, "r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = raw.split("(", 1)[0].strip().upper()
            if not line or not line.startswith("G1"):
                continue

            new_x = current_x
            new_z = current_z
            feed = current_feed

            for word in line.split():
                if len(word) < 2:
                    continue
                letter = word[0]
                try:
                    value = float(word[1:])
                except ValueError:
                    continue

                if letter == "X":
                    new_x = value
                elif letter == "Z":
                    new_z = value
                elif letter == "F":
                    feed = value

            current_feed = feed

            if current_x is not None and current_z is not None:
                dx = 0.0 if new_x is None else new_x - current_x
                dz = 0.0 if new_z is None else new_z - current_z
                distance = math.hypot(dx, dz)

                if distance > 0.0 and feed > 0.0:
                    total_distance += distance
                    total_minutes += distance / feed

            current_x = new_x
            current_z = new_z
            g1_moves += 1

    return total_minutes, g1_moves, total_distance


def format_minutes(minutes):
    if minutes < 1.0:
        return "< 1 min"
    hours = int(minutes // 60)
    mins = int(round(minutes - hours * 60))

    if mins == 60:
        hours += 1
        mins = 0

    if hours:
        return f"{hours} h {mins:02d} min"
    return f"{mins} min"


def process_job(
    stl_path,
    ball_radius,
    cutting_edge_length,
    min_cutter_z,
    requested_stock_diameter,
    x_step,
    a_step,
    entry_step,
    cut_feed,
    status_callback=None,
):
    """
    Run the proven v0.9 machining pipeline using values supplied by the UI.
    Returns a dictionary used by the status panel.
    """
    global X_STEP_MM, A_STEP_DEG, ENTRY_DEPTH_PER_PASS_MM, CUT_FEED

    X_STEP_MM = float(x_step)
    A_STEP_DEG = float(a_step)
    ENTRY_DEPTH_PER_PASS_MM = float(entry_step)
    CUT_FEED = float(cut_feed)

    stl_path = Path(stl_path)

    if not stl_path.exists():
        raise FileNotFoundError(f"Cannot find STL file: {stl_path}")

    if status_callback:
        status_callback("Loading STL...")

    mesh = trimesh.load_mesh(stl_path, process=False)

    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError("The file did not load as a single triangular mesh.")

    bounds = mesh.bounds
    x_min = float(bounds[0, 0])
    x_max = float(bounds[1, 0])

    # Keep the ball centre away from closed end faces by at least one ball radius.
    effective_end_inset = max(END_INSET_MM, ball_radius)
    first_x = x_min + effective_end_inset
    last_x = x_max - effective_end_inset

    x_positions = np.arange(first_x, last_x, X_STEP_MM)

    if len(x_positions) == 0 or x_positions[-1] < last_x - 1e-9:
        x_positions = np.append(x_positions, last_x)

    x_relative = x_positions - first_x

    machine_angles_deg = np.arange(
        A_START_DEG,
        A_START_DEG + 360.0,
        A_STEP_DEG,
        dtype=float,
    )

    sample_angles_deg = (
        A_ZERO_RAY_DEG - A_DIRECTION * machine_angles_deg
    ) % 360.0

    if status_callback:
        status_callback(
            f"Sampling STL: {len(x_positions):,} X sections × "
            f"{len(machine_angles_deg):,} rotary positions..."
        )

    radius_map = build_radius_map(mesh, x_positions, sample_angles_deg)

    missing_mask = np.isnan(radius_map)
    missing = int(missing_mask.sum())

    if missing:
        fallback_surface_radius = max(0.0, min_cutter_z - ball_radius)
        radius_map = np.where(missing_mask, fallback_surface_radius, radius_map)

    # Expand the fallback region by one neighbour in X and A so finite
    # differences do not use an artificial clipped value to form a normal.
    fallback_mask = missing_mask.copy()
    if missing:
        fallback_mask |= np.roll(missing_mask, 1, axis=0)
        fallback_mask |= np.roll(missing_mask, -1, axis=0)
        fallback_mask |= np.roll(missing_mask, 1, axis=1)
        fallback_mask |= np.roll(missing_mask, -1, axis=1)

    min_radius = float(np.min(radius_map))
    max_radius = float(np.max(radius_map))
    model_max_diameter = max_radius * 2.0

    if requested_stock_diameter is None:
        stock_diameter = model_max_diameter + AUTO_STOCK_ALLOWANCE_MM
        stock_auto = True
    else:
        stock_diameter = float(requested_stock_diameter)
        stock_auto = False

    stock_radius = stock_diameter / 2.0

    if max_radius > stock_radius + 1e-6:
        raise RuntimeError(
            f"Stock too small: model diameter is {model_max_diameter:.3f} mm "
            f"but stock diameter is only {stock_diameter:.3f} mm."
        )

    wrapped = np.roll(radius_map, -1, axis=1)
    radial_changes = np.abs(wrapped - radius_map)
    max_adjacent_change = float(np.max(radial_changes))

    maximum_cut_depth = stock_radius - min_radius
    cutting_length_warning = maximum_cut_depth > cutting_edge_length
    adjacent_warning = (
        max_adjacent_change > MAX_ALLOWED_ADJACENT_RADIAL_CHANGE_MM
    )

    output_path = choose_output_path(stl_path)

    if status_callback:
        status_callback("Generating G-code...")

    x_tool_map, a_tool_map, z_tool_map = generate_gcode(
        output_path,
        x_relative,
        machine_angles_deg,
        sample_angles_deg,
        radius_map,
        fallback_mask,
        stock_radius,
        ball_radius,
        cutting_edge_length,
        min_cutter_z,
    )

    if status_callback:
        status_callback("Estimating machining time...")

    estimated_minutes, g1_moves, xz_distance = estimate_gcode_time(
        output_path, CUT_FEED
    )

    raw_surface_points = len(x_positions) * len(machine_angles_deg)
    reduction = 0.0
    if raw_surface_points > 0:
        reduction = max(
            0.0,
            100.0 * (1.0 - min(g1_moves, raw_surface_points) / raw_surface_points)
        )

    return {
        "output_path": output_path,
        "length_mm": x_max - x_min,
        "model_diameter": model_max_diameter,
        "stock_diameter": stock_diameter,
        "stock_auto": stock_auto,
        "x_sections": len(x_positions),
        "angular_passes": len(machine_angles_deg),
        "surface_samples": radius_map.size,
        "missing_samples": missing,
        "max_adjacent_change": max_adjacent_change,
        "maximum_cut_depth": maximum_cut_depth,
        "g1_moves": g1_moves,
        "raw_surface_points": raw_surface_points,
        "reduction_pct": reduction,
        "estimated_minutes": estimated_minutes,
        "xz_distance_mm": xz_distance,
        "cutting_length_warning": cutting_length_warning,
        "adjacent_warning": adjacent_warning,
        "normal_fallback_points": int(fallback_mask.sum()),
        "max_x_compensation": float(np.max(np.abs(x_tool_map - x_relative[:, None]))),
        "max_a_compensation": float(
            np.max(
                np.abs(
                    a_tool_map - np.asarray(machine_angles_deg, dtype=float)[None, :]
                )
            )
        ),
        "end_inset": effective_end_inset,
    }


class RotaryCamUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Brian Rotary CAM v0.11")
        self.root.resizable(False, False)

        self.stl_var = tk.StringVar()
        self.r_var = tk.StringVar(value=f"{DEFAULT_BALL_RADIUS_MM:.2f}")
        self.l_var = tk.StringVar(value=f"{DEFAULT_CUTTING_EDGE_LENGTH_MM:.2f}")
        self.stock_var = tk.StringVar(value="")
        self.a_step_var = tk.StringVar(value="1.00")
        self.x_step_var = tk.StringVar(value="1.00")
        self.entry_step_var = tk.StringVar(value="2.00")
        self.min_z_var = tk.StringVar(value=f"{DEFAULT_MIN_CUTTER_Z_MM:.2f}")
        self.feed_var = tk.StringVar(value="800")
        self.status_var = tk.StringVar(value="Choose an STL file to begin.")

        outer = ttk.Frame(root, padding=14)
        outer.grid(row=0, column=0, sticky="nsew")

        file_frame = ttk.LabelFrame(outer, text="STL file", padding=10)
        file_frame.grid(row=0, column=0, sticky="ew")
        file_frame.columnconfigure(0, weight=1)

        self.file_entry = ttk.Entry(
            file_frame, textvariable=self.stl_var, width=54
        )
        self.file_entry.grid(row=0, column=0, padx=(0, 8), sticky="ew")

        ttk.Button(
            file_frame, text="Choose…", command=self.choose_file
        ).grid(row=0, column=1)

        params = ttk.Frame(outer)
        params.grid(row=1, column=0, pady=(12, 0), sticky="ew")

        cutter = ttk.LabelFrame(params, text="Cutter / stock", padding=10)
        cutter.grid(row=0, column=0, padx=(0, 8), sticky="nsew")

        machining = ttk.LabelFrame(params, text="Machining", padding=10)
        machining.grid(row=0, column=1, sticky="nsew")

        self.add_field(cutter, 0, "Ballnose radius R", self.r_var, "mm")
        self.add_field(cutter, 1, "Cutting-edge length L", self.l_var, "mm")
        self.add_field(cutter, 2, "Stock diameter", self.stock_var, "mm")
        ttk.Label(
            cutter,
            text=f"Leave stock blank = STL max + {AUTO_STOCK_ALLOWANCE_MM:.0f} mm",
        ).grid(row=3, column=0, columnspan=3, sticky="w", pady=(2, 0))

        self.add_field(machining, 0, "A rotation step", self.a_step_var, "deg")
        self.add_field(machining, 1, "X step", self.x_step_var, "mm")
        self.add_field(machining, 2, "Entry Z step/pass", self.entry_step_var, "mm")
        self.add_field(machining, 3, "Minimum cutter Z", self.min_z_var, "mm")
        self.add_field(machining, 4, "Cut feed", self.feed_var, "mm/min")

        ttk.Label(
            machining,
            text=f"G-code tolerance: {GCODE_Z_TOLERANCE_MM:.2f} mm",
        ).grid(row=5, column=0, columnspan=3, sticky="w", pady=(4, 0))

        self.generate_button = ttk.Button(
            outer, text="Generate G-code", command=self.generate
        )
        self.generate_button.grid(row=2, column=0, pady=(12, 0), sticky="ew")

        status_frame = ttk.LabelFrame(outer, text="Status", padding=10)
        status_frame.grid(row=3, column=0, pady=(12, 0), sticky="ew")

        self.status_label = ttk.Label(
            status_frame,
            textvariable=self.status_var,
            justify="left",
            anchor="w",
            width=68,
        )
        self.status_label.grid(row=0, column=0, sticky="w")

    def add_field(self, parent, row, label, variable, unit):
        ttk.Label(parent, text=label).grid(
            row=row, column=0, sticky="w", padx=(0, 8), pady=3
        )
        ttk.Entry(parent, textvariable=variable, width=10).grid(
            row=row, column=1, sticky="e", pady=3
        )
        ttk.Label(parent, text=unit).grid(
            row=row, column=2, sticky="w", padx=(5, 0), pady=3
        )

    def choose_file(self):
        filename = filedialog.askopenfilename(
            title="Choose STL for rotary machining",
            filetypes=[("STL files", "*.stl"), ("All files", "*.*")],
        )
        if filename:
            self.stl_var.set(filename)
            self.status_var.set("STL selected. Check settings and generate.")

    def update_status(self, text):
        self.status_var.set(text)
        self.root.update_idletasks()
        self.root.update()

    def parse_positive(self, value, name, allow_zero=False):
        try:
            number = float(value)
        except ValueError:
            raise ValueError(f"{name} must be a number.")

        if allow_zero:
            if number < 0:
                raise ValueError(f"{name} cannot be negative.")
        elif number <= 0:
            raise ValueError(f"{name} must be greater than zero.")

        return number

    def generate(self):
        stl_path = self.stl_var.get().strip()
        if not stl_path:
            messagebox.showerror("Rotary CAM", "Choose an STL file first.")
            return

        try:
            ball_radius = self.parse_positive(
                self.r_var.get(), "Ballnose radius", allow_zero=True
            )
            cutting_length = self.parse_positive(
                self.l_var.get(), "Cutting-edge length"
            )
            a_step = self.parse_positive(self.a_step_var.get(), "A step")
            x_step = self.parse_positive(self.x_step_var.get(), "X step")
            entry_step = self.parse_positive(
                self.entry_step_var.get(), "Entry Z step"
            )
            min_z = self.parse_positive(
                self.min_z_var.get(), "Minimum cutter Z", allow_zero=True
            )
            feed = self.parse_positive(self.feed_var.get(), "Cut feed")

            stock_text = self.stock_var.get().strip()
            stock_diameter = (
                None
                if stock_text == ""
                else self.parse_positive(stock_text, "Stock diameter")
            )

            self.generate_button.state(["disabled"])
            self.update_status("Starting...")

            result = process_job(
                stl_path=stl_path,
                ball_radius=ball_radius,
                cutting_edge_length=cutting_length,
                min_cutter_z=min_z,
                requested_stock_diameter=stock_diameter,
                x_step=x_step,
                a_step=a_step,
                entry_step=entry_step,
                cut_feed=feed,
                status_callback=self.update_status,
            )

            stock_note = "auto" if result["stock_auto"] else "entered"
            warnings = []
            if result["missing_samples"]:
                warnings.append(
                    f"{result['missing_samples']:,} missed rays clipped to minimum Z"
                )
            if result["cutting_length_warning"]:
                warnings.append("cut depth exceeds entered cutting-edge length")
            if result["adjacent_warning"]:
                warnings.append(
                    f"adjacent A change exceeds "
                    f"{MAX_ALLOWED_ADJACENT_RADIAL_CHANGE_MM:.2f} mm"
                )

            warning_text = (
                "\nWarnings: " + "; ".join(warnings)
                if warnings
                else "\nWarnings: none"
            )

            self.status_var.set(
                f"Model length:             {result['length_mm']:.2f} mm\n"
                f"Model max diameter:       {result['model_diameter']:.2f} mm\n"
                f"Stock diameter:           {result['stock_diameter']:.2f} mm ({stock_note})\n"
                f"X sections / A passes:    {result['x_sections']:,} / "
                f"{result['angular_passes']:,}\n"
                f"Surface samples:          {result['surface_samples']:,}\n"
                f"Missed rays clipped:      {result['missing_samples']:,}\n"
                f"Maximum adjacent A change:{result['max_adjacent_change']:8.3f} mm\n"
                f"Normal fallback points:   {result['normal_fallback_points']:,}\n"
                f"Max normal X correction:  {result['max_x_compensation']:.3f} mm\n"
                f"Max normal A correction:  {result['max_a_compensation']:.3f} deg\n"
                f"End inset used:            {result['end_inset']:.3f} mm\n"
                f"G-code cutting moves:     {result['g1_moves']:,}\n"
                f"Approx. X/Z travel:       {result['xz_distance_mm']/1000:.2f} m\n"
                f"Estimated cut time:       {format_minutes(result['estimated_minutes'])}\n"
                f"                          (X/Z travel only; A indexing ignored)\n"
                f"{warning_text}\n\n"
                f"Saved:\n{result['output_path']}"
            )

        except Exception as exc:
            self.status_var.set(f"Error:\n{exc}")
            messagebox.showerror("Rotary CAM", str(exc))
        finally:
            self.generate_button.state(["!disabled"])


def main():
    root = tk.Tk()
    RotaryCamUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
