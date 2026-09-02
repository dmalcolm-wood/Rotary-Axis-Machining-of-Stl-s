"""Embedded PyVista presentation for STL and indexed rotary toolpaths."""

from pathlib import Path

import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor
from PySide6 import QtCore, QtWidgets

from .gcode_view import gcode_segments_for_view, segments_to_polydata


class RotaryViewer(QtWidgets.QWidget):
    """PyVista viewer using the colour scheme of Helpers/Vizualizer.py."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.mesh = None
        self.paths = {"rough": ([], []), "finish": ([], [])}
        self.actors = {}

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.plotter = QtInteractor(self)
        layout.addWidget(self.plotter.interactor)

        self.plotter.set_background("black")
        self.plotter.show_grid(color="white")
        self.plotter.add_axes(color="white")
        self.plotter.enable_trackball_style()

    def load_stl(self, path):
        mesh = pv.read(str(Path(path)))
        if mesh.n_points == 0:
            raise ValueError("The STL contains no points.")
        self.mesh = mesh
        self.paths = {"rough": ([], []), "finish": ([], [])}
        self.rebuild()
        self.fit_view()

    def load_toolpaths(self, rough_path, finish_path, x_origin, linear_axis="X"):
        self.paths["rough"] = gcode_segments_for_view(
            rough_path, x_origin, linear_axis
        )
        self.paths["finish"] = gcode_segments_for_view(
            finish_path, x_origin, linear_axis
        )
        self.rebuild()
        self.fit_view()

    def rebuild(self):
        self.plotter.clear()
        self.actors.clear()
        self.plotter.set_background("black")
        self.plotter.show_grid(color="white")
        self.plotter.add_axes(color="white")

        if self.mesh is not None:
            self.actors["stl"] = self.plotter.add_mesh(
                self.mesh,
                color="lightgray",
                opacity=0.30,
                smooth_shading=True,
                show_edges=False,
                name="Design STL",
            )

        self._add_operation("rough", line_color="orange")
        self._add_operation("finish", line_color="red")
        self.plotter.render()

    def _add_operation(self, key, line_color):
        rapids, cuts = self.paths[key]
        rapid_poly = segments_to_polydata(pv, rapids)
        cut_poly = segments_to_polydata(pv, cuts)

        if rapid_poly is not None:
            self.actors[f"{key}_rapid"] = self.plotter.add_mesh(
                rapid_poly,
                color="gray",
                line_width=1,
                opacity=0.30,
                name=f"{key.title()} rapid moves",
            )

        if cut_poly is not None:
            self.actors[f"{key}_line"] = self.plotter.add_mesh(
                cut_poly,
                color=line_color,
                line_width=2,
                opacity=0.55,
                name=f"{key.title()} cutting path",
            )
            points = pv.PolyData(cut_poly.points)
            points["Z depth"] = np.asarray(cut_poly.points[:, 2], dtype=float)
            self.actors[f"{key}_points"] = self.plotter.add_mesh(
                points,
                scalars="Z depth",
                cmap="coolwarm",
                point_size=3,
                opacity=0.90,
                render_points_as_spheres=False,
                show_scalar_bar=False,
                name=f"{key.title()} depth points",
            )

    def set_stl_visible(self, visible):
        self._set_actor_visible("stl", visible)

    def set_operation_visible(self, key, visible):
        self._set_actor_visible(f"{key}_line", visible)
        self._set_actor_visible(f"{key}_points", visible)

    def set_rapids_visible(self, visible):
        self._set_actor_visible("rough_rapid", visible)
        self._set_actor_visible("finish_rapid", visible)

    def _set_actor_visible(self, key, visible):
        actor = self.actors.get(key)
        if actor is not None:
            actor.SetVisibility(bool(visible))
            self.plotter.render()

    def fit_view(self):
        self.plotter.view_isometric()
        self.plotter.reset_camera()
        self.plotter.render()
