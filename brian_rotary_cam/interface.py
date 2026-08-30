"""Simple PySide6 operator interface for Brian Rotary CAM."""

from pathlib import Path

from PySide6 import QtCore, QtGui, QtWidgets

from .cam_core import (
    AUTO_STOCK_ALLOWANCE_MM,
    FINISH_CUTTER_DIAMETER_DEFAULT,
    FINISH_FEED_DEFAULT,
    FINISH_MIN_CUTTER_Z_DEFAULT,
    FINISH_SPACING_FRACTION_DEFAULT,
    FINISH_TIP_RADIUS_DEFAULT,
    ROUGH_ALLOWANCE_DEFAULT,
    ROUGH_CUTTER_DIAMETER_DEFAULT,
    ROUGH_DEPTH_PER_PASS_DEFAULT,
    ROUGH_FEED_DEFAULT,
    ROUGH_MIN_CUTTER_Z_DEFAULT,
    ROUGH_SPACING_FRACTION_DEFAULT,
    ROUGH_TIP_RADIUS_DEFAULT,
    X_STEP_MM_DEFAULT,
    format_minutes,
    parse_optional_float,
    process_job,
)
from .pyvista_viewer import RotaryViewer


class RotaryCamWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Brian Rotary CAM v1.3 - Indexed XZA + PyVista")
        self.resize(1500, 850)
        self.setMinimumSize(1200, 700)

        self.fields = {}
        self._build_ui()
        self._connect_view_switches()
        self.status_label.setText(
            "Choose an STL file. One run creates separate roughing and finishing files."
        )

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(7)

        upper = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        root.addWidget(upper, 1)

        controls = QtWidgets.QWidget()
        controls.setMaximumWidth(660)
        controls_layout = QtWidgets.QVBoxLayout(controls)
        controls_layout.setContentsMargins(0, 0, 5, 0)
        controls_layout.setSpacing(6)
        upper.addWidget(controls)

        file_box = QtWidgets.QGroupBox("STL / stock")
        file_layout = QtWidgets.QGridLayout(file_box)
        file_layout.setContentsMargins(7, 7, 7, 7)
        file_layout.setVerticalSpacing(4)
        self.stl_path = QtWidgets.QLineEdit()
        choose = QtWidgets.QPushButton("Choose…")
        choose.clicked.connect(self.choose_stl)
        file_layout.addWidget(self.stl_path, 0, 0, 1, 3)
        file_layout.addWidget(choose, 0, 3)
        self._add_field(file_layout, 1, "stock", "Stock diameter", "", "mm")
        file_layout.addWidget(
            QtWidgets.QLabel(f"blank = model max + {AUTO_STOCK_ALLOWANCE_MM:.0f} mm"),
            1,
            3,
        )
        self._add_field(
            file_layout, 2, "x_step", "X sampling step", f"{X_STEP_MM_DEFAULT:.2f}", "mm"
        )
        controls_layout.addWidget(file_box)

        operations = QtWidgets.QHBoxLayout()
        operations.setSpacing(6)
        operations.addWidget(self._roughing_box())
        operations.addWidget(self._finishing_box())
        controls_layout.addLayout(operations)

        self.generate_button = QtWidgets.QPushButton(
            "Generate Roughing + Finishing Files"
        )
        self.generate_button.setMinimumHeight(34)
        self.generate_button.clicked.connect(self.generate)
        controls_layout.addWidget(self.generate_button)
        controls_layout.addStretch(1)

        viewer_box = QtWidgets.QGroupBox("STL and toolpath visualizer")
        viewer_layout = QtWidgets.QVBoxLayout(viewer_box)
        viewer_layout.setContentsMargins(6, 6, 6, 6)
        switches = QtWidgets.QHBoxLayout()
        self.show_stl = QtWidgets.QCheckBox("STL")
        self.show_rough = QtWidgets.QCheckBox("Roughing path")
        self.show_finish = QtWidgets.QCheckBox("Finishing path")
        self.show_rapids = QtWidgets.QCheckBox("Rapid moves")
        self.show_stl.setChecked(True)
        self.show_rough.setChecked(True)
        self.show_finish.setChecked(True)
        self.show_rapids.setChecked(False)
        for control in (
            self.show_stl,
            self.show_rough,
            self.show_finish,
            self.show_rapids,
        ):
            switches.addWidget(control)
        switches.addStretch(1)
        fit = QtWidgets.QPushButton("Fit view")
        fit.clicked.connect(lambda: self.viewer.fit_view())
        switches.addWidget(fit)
        viewer_layout.addLayout(switches)
        self.viewer = RotaryViewer()
        viewer_layout.addWidget(self.viewer, 1)
        upper.addWidget(viewer_box)
        upper.setSizes([600, 900])
        upper.setStretchFactor(0, 0)
        upper.setStretchFactor(1, 1)

        lower = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        lower.setMinimumHeight(210)
        lower.setMaximumHeight(285)
        root.addWidget(lower, 0)

        status_box = QtWidgets.QGroupBox("Status")
        status_layout = QtWidgets.QVBoxLayout(status_box)
        self.status_label = QtWidgets.QLabel()
        self.status_label.setAlignment(
            QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignTop
        )
        self.status_label.setWordWrap(True)
        self.status_label.setTextInteractionFlags(
            QtCore.Qt.TextInteractionFlag.TextSelectableByMouse
        )
        status_scroll = QtWidgets.QScrollArea()
        status_scroll.setWidgetResizable(True)
        status_scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        status_scroll.setWidget(self.status_label)
        status_layout.addWidget(status_scroll)
        lower.addWidget(status_box)

        text_box = QtWidgets.QGroupBox("Generated G-code (text)")
        text_layout = QtWidgets.QVBoxLayout(text_box)
        text_layout.setContentsMargins(5, 5, 5, 5)
        self.gcode_tabs = QtWidgets.QTabWidget()
        self.rough_text = self._gcode_text_widget()
        self.finish_text = self._gcode_text_widget()
        self.gcode_tabs.addTab(self.rough_text, "Roughing G-code")
        self.gcode_tabs.addTab(self.finish_text, "Finishing G-code")
        text_layout.addWidget(self.gcode_tabs)
        lower.addWidget(text_box)
        lower.setSizes([470, 1030])
        lower.setStretchFactor(0, 0)
        lower.setStretchFactor(1, 1)

    def _roughing_box(self):
        box = QtWidgets.QGroupBox("ROUGHING file")
        layout = QtWidgets.QGridLayout(box)
        layout.setContentsMargins(7, 7, 7, 7)
        layout.setVerticalSpacing(3)
        rows = (
            ("rough_diam", "Cutter diameter", ROUGH_CUTTER_DIAMETER_DEFAULT, "mm"),
            ("rough_tip", "Tip / ball radius", ROUGH_TIP_RADIUS_DEFAULT, "mm"),
            ("rough_depth", "Depth per layer", ROUGH_DEPTH_PER_PASS_DEFAULT, "mm"),
            ("rough_allow", "Leave allowance", ROUGH_ALLOWANCE_DEFAULT, "mm"),
            ("rough_spacing", "Auto spacing", ROUGH_SPACING_FRACTION_DEFAULT * 100, "% dia"),
            ("rough_manual", "Manual A step", "", "deg"),
            ("rough_min_z", "Minimum cutter Z", ROUGH_MIN_CUTTER_Z_DEFAULT, "mm"),
            ("rough_feed", "Cut feed", ROUGH_FEED_DEFAULT, "mm/min"),
        )
        for row, (key, label, value, unit) in enumerate(rows):
            self._add_field(layout, row, key, label, value, unit)
        return box

    def _finishing_box(self):
        box = QtWidgets.QGroupBox("FINISHING file")
        layout = QtWidgets.QGridLayout(box)
        layout.setContentsMargins(7, 7, 7, 7)
        layout.setVerticalSpacing(3)
        rows = (
            ("finish_diam", "Cutter diameter", FINISH_CUTTER_DIAMETER_DEFAULT, "mm"),
            ("finish_tip", "Tip / ball radius", FINISH_TIP_RADIUS_DEFAULT, "mm"),
            ("finish_spacing", "Auto spacing", FINISH_SPACING_FRACTION_DEFAULT * 100, "% dia"),
            ("finish_manual", "Manual A step", "", "deg"),
            ("finish_min_z", "Minimum cutter Z", FINISH_MIN_CUTTER_Z_DEFAULT, "mm"),
            ("finish_feed", "Cut feed", FINISH_FEED_DEFAULT, "mm/min"),
        )
        for row, (key, label, value, unit) in enumerate(rows):
            self._add_field(layout, row, key, label, value, unit)
        note = QtWidgets.QLabel(
            "A stays fixed during each X/Z pass.\nBlank A step uses cutter/stock size."
        )
        note.setWordWrap(True)
        layout.addWidget(note, len(rows), 0, 1, 3)
        return box

    def _add_field(self, layout, row, key, label, value, unit):
        layout.addWidget(QtWidgets.QLabel(label), row, 0)
        edit = QtWidgets.QLineEdit("" if value == "" else f"{float(value):g}")
        edit.setMaximumWidth(76)
        edit.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight)
        self.fields[key] = edit
        layout.addWidget(edit, row, 1)
        layout.addWidget(QtWidgets.QLabel(unit), row, 2)

    @staticmethod
    def _gcode_text_widget():
        text = QtWidgets.QPlainTextEdit()
        text.setReadOnly(True)
        text.setLineWrapMode(QtWidgets.QPlainTextEdit.LineWrapMode.NoWrap)
        text.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont))
        return text

    def _connect_view_switches(self):
        self.show_stl.toggled.connect(self.viewer.set_stl_visible)
        self.show_rough.toggled.connect(
            lambda visible: self.viewer.set_operation_visible("rough", visible)
        )
        self.show_finish.toggled.connect(
            lambda visible: self.viewer.set_operation_visible("finish", visible)
        )
        self.show_rapids.toggled.connect(self.viewer.set_rapids_visible)

    def choose_stl(self):
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Choose STL for rotary machining",
            "",
            "STL files (*.stl);;All files (*)",
        )
        if not filename:
            return
        try:
            self.stl_path.setText(filename)
            self.update_status("Loading STL into the visualizer…")
            self.viewer.load_stl(filename)
            self.rough_text.clear()
            self.finish_text.clear()
            self._apply_view_visibility()
            self.update_status(
                "STL loaded. Check the roughing and finishing settings, then generate both files."
            )
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Rotary CAM", str(exc))
            self.update_status(f"Error loading STL:\n{exc}")

    def update_status(self, text):
        self.status_label.setText(str(text))
        QtWidgets.QApplication.processEvents()

    @staticmethod
    def _positive(text, name, allow_zero=False):
        try:
            value = float(text)
        except ValueError as exc:
            raise ValueError(f"{name} must be a number.") from exc
        if allow_zero:
            if value < 0:
                raise ValueError(f"{name} cannot be negative.")
        elif value <= 0:
            raise ValueError(f"{name} must be greater than zero.")
        return value

    def _parameters(self):
        value = lambda key: self.fields[key].text().strip()
        stock_text = value("stock")
        return {
            "stock_diameter": None
            if not stock_text
            else self._positive(stock_text, "Stock diameter"),
            "x_step": self._positive(value("x_step"), "X sampling step"),
            "rough_cutter_diameter": self._positive(value("rough_diam"), "Roughing cutter diameter"),
            "rough_tip_radius": self._positive(value("rough_tip"), "Roughing tip radius", True),
            "rough_depth_per_pass": self._positive(value("rough_depth"), "Roughing depth per layer"),
            "rough_allowance": self._positive(value("rough_allow"), "Roughing allowance", True),
            "rough_spacing_fraction": self._positive(value("rough_spacing"), "Roughing auto spacing") / 100.0,
            "rough_manual_a": parse_optional_float(value("rough_manual")),
            "rough_min_z": self._positive(value("rough_min_z"), "Roughing minimum cutter Z", True),
            "rough_feed": self._positive(value("rough_feed"), "Roughing feed"),
            "finish_cutter_diameter": self._positive(value("finish_diam"), "Finishing cutter diameter"),
            "finish_tip_radius": self._positive(value("finish_tip"), "Finishing tip radius", True),
            "finish_spacing_fraction": self._positive(value("finish_spacing"), "Finishing auto spacing") / 100.0,
            "finish_manual_a": parse_optional_float(value("finish_manual")),
            "finish_min_z": self._positive(value("finish_min_z"), "Finishing minimum cutter Z", True),
            "finish_feed": self._positive(value("finish_feed"), "Finishing feed"),
        }

    def generate(self):
        stl_path = self.stl_path.text().strip()
        if not stl_path:
            QtWidgets.QMessageBox.warning(self, "Rotary CAM", "Choose an STL file first.")
            return
        self.generate_button.setEnabled(False)
        try:
            result = process_job(stl_path, self._parameters(), self.update_status)
            self.viewer.load_toolpaths(
                result["rough_path"], result["finish_path"], result["x_origin"]
            )
            self._apply_view_visibility()
            self.rough_text.setPlainText(
                Path(result["rough_path"]).read_text(encoding="utf-8", errors="replace")
            )
            self.finish_text.setPlainText(
                Path(result["finish_path"]).read_text(encoding="utf-8", errors="replace")
            )
            self.status_label.setText(self._result_text(result))
        except Exception as exc:
            self.update_status(f"Error:\n{exc}")
            QtWidgets.QMessageBox.critical(self, "Rotary CAM", str(exc))
        finally:
            self.generate_button.setEnabled(True)

    def _apply_view_visibility(self):
        self.viewer.set_stl_visible(self.show_stl.isChecked())
        self.viewer.set_operation_visible("rough", self.show_rough.isChecked())
        self.viewer.set_operation_visible("finish", self.show_finish.isChecked())
        self.viewer.set_rapids_visible(self.show_rapids.isChecked())

    @staticmethod
    def _result_text(result):
        stock_note = "auto" if result["stock_auto"] else "entered"
        rough_mode = "manual" if result["rough_manual"] else "automatic"
        finish_mode = "manual" if result["finish_manual"] else "automatic"
        return (
            f"Model: {result['model_length']:.2f} mm long × {result['model_diameter']:.2f} mm max diameter\n"
            f"Stock: {result['stock_diameter']:.2f} mm ({stock_note}); X sections: {result['x_sections']:,}\n\n"
            f"ROUGHING: {result['rough_passes']:,} divisions @ {result['rough_a_step']:.5f}° ({rough_mode})\n"
            f"Spacing: {result['rough_spacing']:.3f} mm; estimated cut time: {format_minutes(result['rough_minutes'])}\n"
            f"Missed rays clipped: {result['rough_missing']:,}\nSaved: {result['rough_path']}\n\n"
            f"FINISHING: {result['finish_passes']:,} divisions @ {result['finish_a_step']:.5f}° ({finish_mode})\n"
            f"Spacing: {result['finish_spacing']:.3f} mm; estimated cut time: {format_minutes(result['finish_minutes'])}\n"
            f"Missed rays clipped: {result['finish_missing']:,}\nSaved: {result['finish_path']}\n\n"
            "Times include X/Z cutting travel only; inspect in Mach3 and air-cut before machining."
        )


def run():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    app.setApplicationName("Brian Rotary CAM")
    window = RotaryCamWindow()
    window.show()
    return app.exec()
