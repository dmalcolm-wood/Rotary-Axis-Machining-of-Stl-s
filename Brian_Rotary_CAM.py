#!/usr/bin/env python3
"""Brian's single launcher for the modular PyVista edition."""

import sys


def main():
    try:
        from brian_rotary_cam.interface import run
    except ImportError as exc:
        missing = getattr(exc, "name", "a required package")
        print(
            "Brian Rotary CAM could not start because "
            f"{missing!r} is not installed.\n\n"
            "Install the application dependencies with:\n"
            "  python -m pip install numpy trimesh pyvista pyvistaqt PySide6",
            file=sys.stderr,
        )
        return 1
    return int(run())


if __name__ == "__main__":
    raise SystemExit(main())
