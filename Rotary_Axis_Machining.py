"""
Brian Rotary CAM v1.0 - Indexed XZA

Purpose
-------
Convert a radially machinable STL into TWO simple Mach3 X/A/Z G-code files:
    <name>_roughing.tap
    <name>_finishing.tap

Design goals for Brian's first practical version:
- A is indexed BETWEEN passes and remains stationary during each X/Z cutting pass.
- Roughing and finishing are separate files so tool changes are completely manual.
- A-step can be calculated automatically from cutter diameter and stock diameter.
- Roughing removes stock in depth-limited layers and leaves a user-set allowance.
- Finishing follows the sampled radial envelope with simple radial tip-radius offset.
- Undercuts/internal radial surfaces are intentionally ignored.

IMPORTANT
---------
This is simple rotary CAM for radially machinable parts, not general production CAM.
Always inspect the output in Mach3 and air-cut above the work before machining.

Dependencies:
    pip install numpy trimesh
"""

import math
from pathlib import Path

import numpy as np
import trimesh

import tkinter as tk
from tkinter import filedialog, ttk, messagebox


# ============================================================
# DEFAULTS
# ============================================================

AUTO_STOCK_ALLOWANCE_MM = 5.0
X_STEP_MM_DEFAULT = 1.0

# Automatic circumferential pass spacing as a fraction of cutter diameter.
# e.g. 0.50 = passes approximately half a cutter diameter apart around stock OD.
ROUGH_SPACING_FRACTION_DEFAULT = 0.50
FINISH_SPACING_FRACTION_DEFAULT = 0.20

ROUGH_CUTTER_DIAMETER_DEFAULT = 6.0
ROUGH_TIP_RADIUS_DEFAULT = 3.0
ROUGH_DEPTH_PER_PASS_DEFAULT = 2.5
ROUGH_ALLOWANCE_DEFAULT = 1.0
ROUGH_FEED_DEFAULT = 900.0
ROUGH_MIN_CUTTER_Z_DEFAULT = 5.0

FINISH_CUTTER_DIAMETER_DEFAULT = 3.0
FINISH_TIP_RADIUS_DEFAULT = 1.5
FINISH_FEED_DEFAULT = 700.0
FINISH_MIN_CUTTER_Z_DEFAULT = 5.0

SAFE_CLEARANCE_MM = 5.0
SPINDLE_SPEED = 20000

# Rotary centre in STL coordinates.
ROTARY_CENTRE_Y = 0.0
ROTARY_CENTRE_Z = 0.0

# Machine A=0 corresponds to +Z ray in STL.
A_START_DEG = 0.0
A_ZERO_RAY_DEG = 90.0
A_DIRECTION = 1.0  # set -1.0 if Brian's rotary runs opposite direction

# Small inset from mesh end faces. Increased automatically to at least tip radius.
END_INSET_MM = 0.10

# G-code simplification. A is fixed within a pass, so only Z matters here.
GCODE_Z_TOLERANCE_MM = 0.01


# ============================================================
# GEOMETRY
# ============================================================

def cross2(a, b):
    """2D cross product, vectorised for (...,2) arrays."""
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def section_segments_yz(mesh, x, centre_y, centre_z):
    """Intersect STL triangles with X=constant and return Y/Z line segments."""
    triangles = np.asarray(mesh.triangles, dtype=float)
    eps = 1e-10
    starts = []
    ends = []

    for tri in triangles:
        d = tri[:, 0] - x
        if np.all(d > eps) or np.all(d < -eps):
            continue

        points = []
        for i0, i1 in ((0, 1), (1, 2), (2, 0)):
            p0 = tri[i0]
            p1 = tri[i1]
            d0 = p0[0] - x
            d1 = p1[0] - x

            if abs(d0) <= eps:
                points.append(p0[1:3])

            if (d0 < -eps and d1 > eps) or (d0 > eps and d1 < -eps):
                t = (x - p0[0]) / (p1[0] - p0[0])
                p = p0 + t * (p1 - p0)
                points.append(p[1:3])

        if not points:
            continue

        unique = []
        for pt in points:
            pt = np.asarray(pt, dtype=float)
            if not any(np.linalg.norm(pt - q) < 1e-8 for q in unique):
                unique.append(pt)

        if len(unique) >= 2:
            a = unique[0] - np.array([centre_y, centre_z], dtype=float)
            b = unique[1] - np.array([centre_y, centre_z], dtype=float)
            if np.linalg.norm(b - a) > 1e-9:
                starts.append(a)
                ends.append(b)

    if not starts:
        return None, None
    return np.asarray(starts), np.asarray(ends)


def radii_for_section(mesh, x, angles_deg, centre_y, centre_z):
    """Return outermost radial STL intersection for each angle at one X section."""
    a, b = section_segments_yz(mesh, x, centre_y, centre_z)
    if a is None:
        return np.full(len(angles_deg), np.nan)

    seg = b - a
    result = np.full(len(angles_deg), np.nan, dtype=float)

    for i, angle_deg in enumerate(angles_deg):
        angle = math.radians(float(angle_deg))
        direction = np.array([math.cos(angle), math.sin(angle)], dtype=float)
        dirs = np.broadcast_to(direction, seg.shape)

        denominator = cross2(dirs, seg)
        nonparallel = np.abs(denominator) > 1e-12

        t = np.full(len(a), np.nan)
        u = np.full(len(a), np.nan)

        t[nonparallel] = cross2(a[nonparallel], seg[nonparallel]) / denominator[nonparallel]
        u[nonparallel] = cross2(a[nonparallel], dirs[nonparallel]) / denominator[nonparallel]

        valid = (
            nonparallel
            & (t >= 0.0)
            & (u >= -1e-9)
            & (u <= 1.0 + 1e-9)
        )
        intersections = t[valid]
        if len(intersections):
            result[i] = np.max(intersections)

    return result


def build_radius_map(mesh, x_positions, sample_angles_deg, status_callback=None):
    radius_map = np.empty((len(x_positions), len(sample_angles_deg)), dtype=float)
    for i, x in enumerate(x_positions):
        radius_map[i, :] = radii_for_section(
            mesh, x, sample_angles_deg, ROTARY_CENTRE_Y, ROTARY_CENTRE_Z
        )
        if status_callback and (i % 20 == 0 or i == len(x_positions) - 1):
            status_callback(f"Sampling STL section {i + 1:,} / {len(x_positions):,}...")
    return radius_map


# ============================================================
# PASS SPACING / A INDEXING
# ============================================================

def automatic_a_index(stock_diameter, cutter_diameter, spacing_fraction):
    """
    Choose an integer number of equal divisions around 360 degrees.

    Target circumferential spacing = cutter diameter * spacing fraction.
    We round UP the pass count so the actual spacing never exceeds the target.
    Returns: (a_step_deg, pass_count, actual_spacing_mm)
    """
    if stock_diameter <= 0 or cutter_diameter <= 0 or spacing_fraction <= 0:
        raise ValueError("Stock diameter, cutter diameter and spacing fraction must be positive.")

    circumference = math.pi * stock_diameter
    target_spacing = cutter_diameter * spacing_fraction
    pass_count = max(1, int(math.ceil(circumference / target_spacing)))
    a_step = 360.0 / pass_count
    actual_spacing = circumference / pass_count
    return a_step, pass_count, actual_spacing


def resolve_a_index(stock_diameter, cutter_diameter, spacing_fraction, manual_step=None):
    if manual_step is not None:
        if manual_step <= 0 or manual_step > 360:
            raise ValueError("Manual A step must be greater than 0 and no more than 360 degrees.")
        pass_count = max(1, int(math.ceil(360.0 / manual_step)))
        a_step = 360.0 / pass_count  # force exact equal divisions / clean closure
        spacing = math.pi * stock_diameter / pass_count
        return a_step, pass_count, spacing, True

    a_step, pass_count, spacing = automatic_a_index(
        stock_diameter, cutter_diameter, spacing_fraction
    )
    return a_step, pass_count, spacing, False


def machine_angles_for_count(pass_count):
    return np.arange(pass_count, dtype=float) * (360.0 / pass_count)


def sample_angles_for_machine_angles(machine_angles_deg):
    return (A_ZERO_RAY_DEG - A_DIRECTION * machine_angles_deg) % 360.0


# ============================================================
# G-CODE WRITING
# ============================================================

def write_xz_pass(f, x_profile, z_profile, a_angle, reverse=False, feed=700.0):
    """Write one fixed-A X/Z cutting pass, simplifying only nearly unchanged Z."""
    indexes = list(range(len(x_profile) - 1, -1, -1)) if reverse else list(range(len(x_profile)))
    if not indexes:
        return

    # A is intentionally fixed for the entire cutting traverse.
    f.write(f"G0 A{a_angle:.5f}\n")

    i0 = indexes[0]
    f.write(f"G1 X{x_profile[i0]:.4f} Z{z_profile[i0]:.4f} F{feed:.1f}\n")

    ref_z = float(z_profile[i0])
    last_skipped = None

    for pos in range(1, len(indexes)):
        i = indexes[pos]
        z = float(z_profile[i])
        is_last = pos == len(indexes) - 1

        if abs(z - ref_z) <= GCODE_Z_TOLERANCE_MM and not is_last:
            last_skipped = i
            continue

        if last_skipped is not None:
            k = last_skipped
            f.write(f"G1 X{x_profile[k]:.4f} Z{z_profile[k]:.4f}\n")
            last_skipped = None

        f.write(f"G1 X{x_profile[i]:.4f} Z{z_profile[i]:.4f}\n")
        ref_z = z


def write_header(f, operation, stock_diameter, cutter_diameter, tip_radius,
                 a_step, pass_count, spacing, x_step, feed, extra_lines):
    f.write(f"(Brian Rotary CAM v1.0 - Indexed XZA - {operation})\n")
    f.write("(WARNING: inspect in Mach3 and air-cut before machining)\n")
    f.write("(A indexes between passes and remains stationary during each X/Z cut)\n")
    f.write("(X = longitudinal axis, A = rotary axis, Z0 = rotary centreline)\n")
    f.write(f"(Stock diameter: {stock_diameter:.3f} mm)\n")
    f.write(f"(Cutter diameter: {cutter_diameter:.3f} mm)\n")
    f.write(f"(Tip/ball radius: {tip_radius:.3f} mm)\n")
    f.write(f"(A divisions: {pass_count:d}, A step: {a_step:.5f} deg)\n")
    f.write(f"(Approx spacing at stock OD: {spacing:.3f} mm)\n")
    f.write(f"(X step: {x_step:.3f} mm)\n")
    f.write(f"(Cut feed: {feed:.1f} mm/min)\n")
    f.write("(Estimated cut time: __TIME__)\n")
    f.write("(Undercuts/internal radial surfaces ignored; outer envelope used)\n")
    for line in extra_lines:
        f.write(f"({line})\n")
    f.write("\nG21\nG90\nG94\n")
    f.write(f"S{SPINDLE_SPEED}\n")
    f.write("M03\n")


def generate_roughing_gcode(output_path, x_relative, machine_angles_deg, radius_map,
                             stock_radius, stock_diameter, cutter_diameter, tip_radius,
                             depth_per_pass, allowance, min_cutter_z, a_step,
                             pass_count, spacing, feed, x_step):
    # Simple radial cutter-centre target. Roughing stays allowance outside finish surface.
    final_z = np.maximum(radius_map + tip_radius + allowance, min_cutter_z)
    stock_tool_radius = stock_radius + tip_radius
    safe_z = stock_tool_radius + SAFE_CLEARANCE_MM

    minimum_target = float(np.min(final_z))
    levels = []
    level = stock_tool_radius - depth_per_pass
    while level > minimum_target + 1e-9:
        levels.append(level)
        level -= depth_per_pass
    levels.append(minimum_target)

    with open(output_path, "w", newline="\n") as f:
        write_header(
            f, "ROUGHING", stock_diameter, cutter_diameter, tip_radius,
            a_step, pass_count, spacing, x_step, feed,
            [
                f"Roughing allowance: {allowance:.3f} mm",
                f"Maximum radial depth/pass: {depth_per_pass:.3f} mm",
                f"Depth layers: {len(levels):d}",
                f"Minimum cutter-centre Z: {min_cutter_z:.3f} mm",
            ],
        )
        f.write(f"G0 Z{safe_z:.4f}\n")
        f.write(f"G0 X{x_relative[0]:.4f}\n\n")

        reverse = False
        for layer_index, level in enumerate(levels, start=1):
            f.write(f"(Roughing layer {layer_index} of {len(levels)} - limiter Z {level:.4f})\n")
            for j, a_angle in enumerate(machine_angles_deg):
                z_profile = np.maximum(final_z[:, j], level)
                # Retract before every index; simple and deliberately conservative.
                f.write(f"G0 Z{safe_z:.4f}\n")
                start_x = x_relative[-1] if reverse else x_relative[0]
                f.write(f"G0 X{start_x:.4f}\n")
                f.write(f"G0 A{a_angle:.5f}\n")
                f.write(f"G0 Z{z_profile[-1 if reverse else 0]:.4f}\n")
                write_xz_pass(f, x_relative, z_profile, a_angle, reverse=reverse, feed=feed)
                reverse = not reverse
            f.write("\n")

        f.write(f"G0 Z{safe_z:.4f}\n")
        f.write("M05\nM30\n")


def generate_finishing_gcode(output_path, x_relative, machine_angles_deg, radius_map,
                              stock_radius, stock_diameter, cutter_diameter, tip_radius,
                              min_cutter_z, a_step, pass_count, spacing, feed, x_step):
    finish_z = np.maximum(radius_map + tip_radius, min_cutter_z)
    stock_tool_radius = stock_radius + tip_radius
    safe_z = stock_tool_radius + SAFE_CLEARANCE_MM

    with open(output_path, "w", newline="\n") as f:
        write_header(
            f, "FINISHING", stock_diameter, cutter_diameter, tip_radius,
            a_step, pass_count, spacing, x_step, feed,
            [
                "Simple radial tip-radius compensation",
                f"Minimum cutter-centre Z: {min_cutter_z:.3f} mm",
            ],
        )
        f.write(f"G0 Z{safe_z:.4f}\n")
        f.write(f"G0 X{x_relative[0]:.4f}\n\n")
        f.write("(Indexed finishing passes)\n")

        reverse = False
        for j, a_angle in enumerate(machine_angles_deg):
            z_profile = finish_z[:, j]
            f.write(f"G0 Z{safe_z:.4f}\n")
            start_x = x_relative[-1] if reverse else x_relative[0]
            f.write(f"G0 X{start_x:.4f}\n")
            f.write(f"G0 A{a_angle:.5f}\n")
            f.write(f"G0 Z{z_profile[-1 if reverse else 0]:.4f}\n")
            write_xz_pass(f, x_relative, z_profile, a_angle, reverse=reverse, feed=feed)
            reverse = not reverse

        f.write(f"\nG0 Z{safe_z:.4f}\n")
        f.write("M05\nM30\n")


def estimate_gcode_time(output_path, default_feed):
    """Estimate G1 X/Z time. A indexing and G0 time are intentionally ignored."""
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
                if distance > 0 and feed > 0:
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
    return f"{hours} h {mins:02d} min" if hours else f"{mins} min"


def update_time_header(path, minutes):
    text = Path(path).read_text(encoding="utf-8")
    text = text.replace("(Estimated cut time: __TIME__)",
                        f"(Estimated cut time: {format_minutes(minutes)})", 1)
    Path(path).write_text(text, encoding="utf-8", newline="\n")


# ============================================================
# JOB PROCESSING
# ============================================================

def parse_optional_float(text):
    text = text.strip()
    if not text:
        return None
    return float(text)


def process_job(stl_path, params, status_callback=None):
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

    max_tip_radius = max(params["rough_tip_radius"], params["finish_tip_radius"])
    effective_end_inset = max(END_INSET_MM, max_tip_radius)
    first_x = x_min + effective_end_inset
    last_x = x_max - effective_end_inset
    if last_x <= first_x:
        raise RuntimeError("Model is too short for the entered cutter tip radii/end inset.")

    x_step = params["x_step"]
    x_positions = np.arange(first_x, last_x, x_step)
    if len(x_positions) == 0 or x_positions[-1] < last_x - 1e-9:
        x_positions = np.append(x_positions, last_x)
    x_relative = x_positions - first_x

    # First sample fairly densely only to determine model maximum diameter if stock is auto.
    # 1 degree is conservative and keeps stock auto-detection independent of tool spacing.
    probe_machine_angles = np.arange(0.0, 360.0, 1.0, dtype=float)
    probe_sample_angles = sample_angles_for_machine_angles(probe_machine_angles)
    if status_callback:
        status_callback("Sampling STL to determine model diameter...")
    probe_map = build_radius_map(mesh, x_positions, probe_sample_angles, status_callback)
    probe_missing = np.isnan(probe_map)
    valid_probe = probe_map[~probe_missing]
    if valid_probe.size == 0:
        raise RuntimeError("No radial STL intersections were found. Check model orientation/centreline.")

    model_max_radius = float(np.max(valid_probe))
    model_max_diameter = model_max_radius * 2.0
    requested_stock = params["stock_diameter"]
    stock_diameter = (model_max_diameter + AUTO_STOCK_ALLOWANCE_MM
                      if requested_stock is None else requested_stock)
    stock_auto = requested_stock is None
    stock_radius = stock_diameter / 2.0
    if model_max_radius > stock_radius + 1e-6:
        raise RuntimeError(
            f"Stock too small: model diameter is {model_max_diameter:.3f} mm "
            f"but stock diameter is only {stock_diameter:.3f} mm."
        )

    rough_a_step, rough_passes, rough_spacing, rough_manual = resolve_a_index(
        stock_diameter,
        params["rough_cutter_diameter"],
        params["rough_spacing_fraction"],
        params["rough_manual_a"],
    )
    finish_a_step, finish_passes, finish_spacing, finish_manual = resolve_a_index(
        stock_diameter,
        params["finish_cutter_diameter"],
        params["finish_spacing_fraction"],
        params["finish_manual_a"],
    )

    rough_machine_angles = machine_angles_for_count(rough_passes)
    finish_machine_angles = machine_angles_for_count(finish_passes)

    if status_callback:
        status_callback(f"Sampling roughing surface: {rough_passes:,} A divisions...")
    rough_map = build_radius_map(
        mesh, x_positions, sample_angles_for_machine_angles(rough_machine_angles), status_callback
    )
    if status_callback:
        status_callback(f"Sampling finishing surface: {finish_passes:,} A divisions...")
    finish_map = build_radius_map(
        mesh, x_positions, sample_angles_for_machine_angles(finish_machine_angles), status_callback
    )

    # Missing rays are clipped conservatively to entered minimum cutter-centre Z minus tip radius.
    rough_missing = np.isnan(rough_map)
    finish_missing = np.isnan(finish_map)
    rough_surface_fallback = max(0.0, params["rough_min_z"] - params["rough_tip_radius"])
    finish_surface_fallback = max(0.0, params["finish_min_z"] - params["finish_tip_radius"])
    rough_map = np.where(rough_missing, rough_surface_fallback, rough_map)
    finish_map = np.where(finish_missing, finish_surface_fallback, finish_map)

    rough_path = stl_path.with_name(stl_path.stem + "_roughing.tap")
    finish_path = stl_path.with_name(stl_path.stem + "_finishing.tap")

    if status_callback:
        status_callback("Writing roughing G-code...")
    generate_roughing_gcode(
        rough_path, x_relative, rough_machine_angles, rough_map,
        stock_radius, stock_diameter,
        params["rough_cutter_diameter"], params["rough_tip_radius"],
        params["rough_depth_per_pass"], params["rough_allowance"], params["rough_min_z"],
        rough_a_step, rough_passes, rough_spacing, params["rough_feed"], x_step,
    )

    if status_callback:
        status_callback("Writing finishing G-code...")
    generate_finishing_gcode(
        finish_path, x_relative, finish_machine_angles, finish_map,
        stock_radius, stock_diameter,
        params["finish_cutter_diameter"], params["finish_tip_radius"], params["finish_min_z"],
        finish_a_step, finish_passes, finish_spacing, params["finish_feed"], x_step,
    )

    rough_minutes, rough_moves, rough_distance = estimate_gcode_time(rough_path, params["rough_feed"])
    finish_minutes, finish_moves, finish_distance = estimate_gcode_time(finish_path, params["finish_feed"])
    update_time_header(rough_path, rough_minutes)
    update_time_header(finish_path, finish_minutes)

    return {
        "model_length": x_max - x_min,
        "model_diameter": model_max_diameter,
        "stock_diameter": stock_diameter,
        "stock_auto": stock_auto,
        "x_sections": len(x_positions),
        "rough_path": rough_path,
        "finish_path": finish_path,
        "rough_a_step": rough_a_step,
        "rough_passes": rough_passes,
        "rough_spacing": rough_spacing,
        "rough_manual": rough_manual,
        "finish_a_step": finish_a_step,
        "finish_passes": finish_passes,
        "finish_spacing": finish_spacing,
        "finish_manual": finish_manual,
        "rough_missing": int(rough_missing.sum()),
        "finish_missing": int(finish_missing.sum()),
        "rough_minutes": rough_minutes,
        "finish_minutes": finish_minutes,
        "rough_moves": rough_moves,
        "finish_moves": finish_moves,
        "rough_distance": rough_distance,
        "finish_distance": finish_distance,
        "end_inset": effective_end_inset,
    }


# ============================================================
# UI
# ============================================================

class RotaryCamUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Brian Rotary CAM v1.0 - Indexed XZA")
        self.root.resizable(False, False)

        self.stl_var = tk.StringVar()
        self.stock_var = tk.StringVar(value="")
        self.x_step_var = tk.StringVar(value=f"{X_STEP_MM_DEFAULT:.2f}")

        self.rough_diam_var = tk.StringVar(value=f"{ROUGH_CUTTER_DIAMETER_DEFAULT:.2f}")
        self.rough_tip_var = tk.StringVar(value=f"{ROUGH_TIP_RADIUS_DEFAULT:.2f}")
        self.rough_depth_var = tk.StringVar(value=f"{ROUGH_DEPTH_PER_PASS_DEFAULT:.2f}")
        self.rough_allow_var = tk.StringVar(value=f"{ROUGH_ALLOWANCE_DEFAULT:.2f}")
        self.rough_spacing_var = tk.StringVar(value=f"{ROUGH_SPACING_FRACTION_DEFAULT * 100:.0f}")
        self.rough_manual_a_var = tk.StringVar(value="")
        self.rough_min_z_var = tk.StringVar(value=f"{ROUGH_MIN_CUTTER_Z_DEFAULT:.2f}")
        self.rough_feed_var = tk.StringVar(value=f"{ROUGH_FEED_DEFAULT:.0f}")

        self.finish_diam_var = tk.StringVar(value=f"{FINISH_CUTTER_DIAMETER_DEFAULT:.2f}")
        self.finish_tip_var = tk.StringVar(value=f"{FINISH_TIP_RADIUS_DEFAULT:.2f}")
        self.finish_spacing_var = tk.StringVar(value=f"{FINISH_SPACING_FRACTION_DEFAULT * 100:.0f}")
        self.finish_manual_a_var = tk.StringVar(value="")
        self.finish_min_z_var = tk.StringVar(value=f"{FINISH_MIN_CUTTER_Z_DEFAULT:.2f}")
        self.finish_feed_var = tk.StringVar(value=f"{FINISH_FEED_DEFAULT:.0f}")

        self.status_var = tk.StringVar(value="Choose an STL file. One run creates separate roughing and finishing files.")

        outer = ttk.Frame(root, padding=12)
        outer.grid(row=0, column=0, sticky="nsew")

        file_frame = ttk.LabelFrame(outer, text="STL / stock", padding=10)
        file_frame.grid(row=0, column=0, columnspan=2, sticky="ew")
        file_frame.columnconfigure(0, weight=1)

        ttk.Entry(file_frame, textvariable=self.stl_var, width=62).grid(row=0, column=0, padx=(0, 8), sticky="ew")
        ttk.Button(file_frame, text="Choose…", command=self.choose_file).grid(row=0, column=1)

        self.add_field(file_frame, 1, "Stock diameter", self.stock_var, "mm", start_col=0)
        ttk.Label(file_frame, text=f"blank = model max + {AUTO_STOCK_ALLOWANCE_MM:.0f} mm").grid(row=1, column=3, sticky="w", padx=(8, 0))
        self.add_field(file_frame, 2, "X sampling step", self.x_step_var, "mm", start_col=0)

        rough = ttk.LabelFrame(outer, text="ROUGHING file", padding=10)
        rough.grid(row=1, column=0, padx=(0, 6), pady=(10, 0), sticky="nsew")
        finish = ttk.LabelFrame(outer, text="FINISHING file", padding=10)
        finish.grid(row=1, column=1, padx=(6, 0), pady=(10, 0), sticky="nsew")

        self.add_field(rough, 0, "Cutter diameter", self.rough_diam_var, "mm")
        self.add_field(rough, 1, "Tip / ball radius", self.rough_tip_var, "mm")
        self.add_field(rough, 2, "Depth per layer", self.rough_depth_var, "mm")
        self.add_field(rough, 3, "Leave allowance", self.rough_allow_var, "mm")
        self.add_field(rough, 4, "Auto spacing", self.rough_spacing_var, "% cutter dia")
        self.add_field(rough, 5, "Manual A step", self.rough_manual_a_var, "deg (blank=auto)")
        self.add_field(rough, 6, "Minimum cutter Z", self.rough_min_z_var, "mm")
        self.add_field(rough, 7, "Cut feed", self.rough_feed_var, "mm/min")

        self.add_field(finish, 0, "Cutter diameter", self.finish_diam_var, "mm")
        self.add_field(finish, 1, "Tip / ball radius", self.finish_tip_var, "mm")
        self.add_field(finish, 2, "Auto spacing", self.finish_spacing_var, "% cutter dia")
        self.add_field(finish, 3, "Manual A step", self.finish_manual_a_var, "deg (blank=auto)")
        self.add_field(finish, 4, "Minimum cutter Z", self.finish_min_z_var, "mm")
        self.add_field(finish, 5, "Cut feed", self.finish_feed_var, "mm/min")
        ttk.Label(
            finish,
            text="A stays fixed throughout every X/Z pass.\nManual A step is optional; blank uses cutter/stock size.",
            justify="left",
        ).grid(row=6, column=0, columnspan=3, sticky="w", pady=(8, 0))

        self.generate_button = ttk.Button(outer, text="Generate Roughing + Finishing Files", command=self.generate)
        self.generate_button.grid(row=2, column=0, columnspan=2, pady=(12, 0), sticky="ew")

        status_frame = ttk.LabelFrame(outer, text="Status", padding=10)
        status_frame.grid(row=3, column=0, columnspan=2, pady=(10, 0), sticky="ew")
        ttk.Label(status_frame, textvariable=self.status_var, justify="left", anchor="w", width=94).grid(row=0, column=0, sticky="w")

    def add_field(self, parent, row, label, variable, unit, start_col=0):
        ttk.Label(parent, text=label).grid(row=row, column=start_col, sticky="w", padx=(0, 7), pady=2)
        ttk.Entry(parent, textvariable=variable, width=10).grid(row=row, column=start_col + 1, sticky="e", pady=2)
        ttk.Label(parent, text=unit).grid(row=row, column=start_col + 2, sticky="w", padx=(5, 0), pady=2)

    def choose_file(self):
        filename = filedialog.askopenfilename(
            title="Choose STL for rotary machining",
            filetypes=[("STL files", "*.stl"), ("All files", "*.*")],
        )
        if filename:
            self.stl_var.set(filename)
            self.status_var.set("STL selected. Check roughing/finishing settings and generate both files.")

    def update_status(self, text):
        self.status_var.set(text)
        self.root.update_idletasks()
        self.root.update()

    @staticmethod
    def positive(text, name, allow_zero=False):
        try:
            value = float(text)
        except ValueError:
            raise ValueError(f"{name} must be a number.")
        if allow_zero:
            if value < 0:
                raise ValueError(f"{name} cannot be negative.")
        elif value <= 0:
            raise ValueError(f"{name} must be greater than zero.")
        return value

    def generate(self):
        stl_path = self.stl_var.get().strip()
        if not stl_path:
            messagebox.showerror("Rotary CAM", "Choose an STL file first.")
            return

        try:
            stock_text = self.stock_var.get().strip()
            stock_diameter = None if stock_text == "" else self.positive(stock_text, "Stock diameter")

            rough_manual = parse_optional_float(self.rough_manual_a_var.get())
            finish_manual = parse_optional_float(self.finish_manual_a_var.get())

            params = {
                "stock_diameter": stock_diameter,
                "x_step": self.positive(self.x_step_var.get(), "X sampling step"),

                "rough_cutter_diameter": self.positive(self.rough_diam_var.get(), "Roughing cutter diameter"),
                "rough_tip_radius": self.positive(self.rough_tip_var.get(), "Roughing tip radius", allow_zero=True),
                "rough_depth_per_pass": self.positive(self.rough_depth_var.get(), "Roughing depth per layer"),
                "rough_allowance": self.positive(self.rough_allow_var.get(), "Roughing allowance", allow_zero=True),
                "rough_spacing_fraction": self.positive(self.rough_spacing_var.get(), "Roughing auto spacing") / 100.0,
                "rough_manual_a": rough_manual,
                "rough_min_z": self.positive(self.rough_min_z_var.get(), "Roughing minimum cutter Z", allow_zero=True),
                "rough_feed": self.positive(self.rough_feed_var.get(), "Roughing feed"),

                "finish_cutter_diameter": self.positive(self.finish_diam_var.get(), "Finishing cutter diameter"),
                "finish_tip_radius": self.positive(self.finish_tip_var.get(), "Finishing tip radius", allow_zero=True),
                "finish_spacing_fraction": self.positive(self.finish_spacing_var.get(), "Finishing auto spacing") / 100.0,
                "finish_manual_a": finish_manual,
                "finish_min_z": self.positive(self.finish_min_z_var.get(), "Finishing minimum cutter Z", allow_zero=True),
                "finish_feed": self.positive(self.finish_feed_var.get(), "Finishing feed"),
            }

            self.generate_button.state(["disabled"])
            result = process_job(stl_path, params, self.update_status)

            stock_note = "auto" if result["stock_auto"] else "entered"
            rough_mode = "manual override" if result["rough_manual"] else "automatic"
            finish_mode = "manual override" if result["finish_manual"] else "automatic"

            self.status_var.set(
                f"Model length:                 {result['model_length']:.2f} mm\n"
                f"Model max diameter:           {result['model_diameter']:.2f} mm\n"
                f"Stock diameter:               {result['stock_diameter']:.2f} mm ({stock_note})\n"
                f"X sections:                   {result['x_sections']:,}\n\n"
                f"ROUGHING A:                    {result['rough_passes']:,} divisions @ {result['rough_a_step']:.5f}° ({rough_mode})\n"
                f"Rough spacing at stock OD:    {result['rough_spacing']:.3f} mm\n"
                f"Rough estimated cut time:     {format_minutes(result['rough_minutes'])}\n"
                f"Rough missed rays clipped:    {result['rough_missing']:,}\n"
                f"Saved: {result['rough_path']}\n\n"
                f"FINISHING A:                   {result['finish_passes']:,} divisions @ {result['finish_a_step']:.5f}° ({finish_mode})\n"
                f"Finish spacing at stock OD:   {result['finish_spacing']:.3f} mm\n"
                f"Finish estimated cut time:    {format_minutes(result['finish_minutes'])}\n"
                f"Finish missed rays clipped:   {result['finish_missing']:,}\n"
                f"Saved: {result['finish_path']}\n\n"
                f"Times are X/Z cutting travel only; A indexing and G0 moves are ignored."
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
