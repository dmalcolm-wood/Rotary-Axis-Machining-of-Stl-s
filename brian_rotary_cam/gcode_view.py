"""Read Brian Rotary CAM G-code and de-rotate it into STL coordinates."""

import math
from pathlib import Path

import numpy as np

from .cam_core import (
    A_DIRECTION,
    A_ZERO_RAY_DEG,
    ROTARY_CENTRE_Y,
    ROTARY_CENTRE_Z,
)


def gcode_segments_for_view(path, x_origin=0.0):
    """Return chronological rapid and cutting segments in workpiece coordinates.

    A machine move is transformed back around the X rotary axis. Each returned
    segment is a two-row NumPy array, suitable for PyVista line construction.
    """
    current = {"X": None, "Z": None, "A": 0.0}
    rapid_segments = []
    cut_segments = []

    def world_point(state):
        angle = math.radians(A_ZERO_RAY_DEG - A_DIRECTION * state["A"])
        radius = state["Z"]
        return np.array(
            [
                x_origin + state["X"],
                ROTARY_CENTRE_Y + radius * math.cos(angle),
                ROTARY_CENTRE_Z + radius * math.sin(angle),
            ],
            dtype=float,
        )

    with Path(path).open("r", encoding="utf-8", errors="ignore") as handle:
        for raw in handle:
            line = raw.split("(", 1)[0].strip().upper()
            if not line:
                continue
            words = line.split()
            motion = next(
                (word for word in words if word in ("G0", "G00", "G1", "G01")),
                None,
            )
            if motion is None:
                continue

            previous = dict(current)
            for word in words:
                if len(word) < 2 or word[0] not in current:
                    continue
                try:
                    current[word[0]] = float(word[1:])
                except ValueError:
                    pass

            if previous["X"] is None or previous["Z"] is None:
                continue
            if current["X"] is None or current["Z"] is None:
                continue

            start = world_point(previous)
            end = world_point(current)
            if np.linalg.norm(end - start) <= 1e-9:
                continue
            target = rapid_segments if motion in ("G0", "G00") else cut_segments
            target.append(np.vstack((start, end)))

    return rapid_segments, cut_segments


def segments_to_polydata(pyvista_module, segments):
    """Combine independent two-point segments into one efficient PolyData."""
    if not segments:
        return None
    points = np.asarray(segments, dtype=float).reshape((-1, 3))
    segment_count = len(segments)
    lines = np.empty((segment_count, 3), dtype=np.int64)
    lines[:, 0] = 2
    lines[:, 1] = np.arange(0, segment_count * 2, 2)
    lines[:, 2] = lines[:, 1] + 1
    poly = pyvista_module.PolyData(points)
    poly.lines = lines.ravel()
    return poly
