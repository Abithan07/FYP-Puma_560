#!/usr/bin/env python3
"""
PUMA-560 Trajectory GUI Launcher
PyQt5-based control panel for run_traj.sh
"""

import sys
import os
import subprocess
import shlex
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QDoubleSpinBox, QSpinBox, QComboBox,
    QPushButton, QGroupBox, QFrame, QSizePolicy, QScrollArea,
    QTextEdit, QSplitter, QSlider
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QPropertyAnimation, QEasingCurve, QRect
from PyQt5.QtGui import QFont, QColor, QPalette, QPixmap, QPainter, QLinearGradient, QFontDatabase


# ══════════════════════════════════════════════════════════════
#  CONFIGURATION  —  edit paths here
# ══════════════════════════════════════════════════════════════

SCRIPT_PATH = "/data/ros2/ros2_ws2/arm_bot/src/scripts/Demo_controller/automater.sh"
BASE_DIR    = "/data/ros2/ros2_ws2/arm_bot/src/scripts/Demo_controller/demo_trajectories"

# Joint angle limits (degrees)
# Each joint can have ONE or TWO valid ranges.
# Format: list of (min, max) tuples — one tuple = single range, two tuples = dual range.
JOINT_LIMITS = {
    "Q1 (Base)":      [(-170, -20), (20, 170)],   # dual range — skip near-zero
    "Q2 (Shoulder)":  [(-90, 180)],                # single range
    "Q3 (Elbow)":     [(-45, 225)],                # single range
}

# Default start angles (degrees) — must match TrajConfig.q_start_deg in the Python generator
Q_START_DEG = [0.0, 45.0, 135.0]

T_TOTAL_MIN  = 5.0
T_TOTAL_MAX  = 60.0
T_TOTAL_DEF  = 18.0

NUM_PATHS_MIN = 1
NUM_PATHS_MAX = 20


# ══════════════════════════════════════════════════════════════
#  COLOURS & STYLE
# ══════════════════════════════════════════════════════════════

STYLE = """
QMainWindow, QWidget {
    background-color: #0e1117;
    color: #e0e6f0;
    font-family: 'JetBrains Mono', 'Fira Mono', 'Courier New', monospace;
}

QGroupBox {
    border: 1px solid #2a3040;
    border-radius: 6px;
    margin-top: 14px;
    padding: 12px 8px 8px 8px;
    font-size: 11px;
    font-weight: bold;
    color: #5b8af5;
    letter-spacing: 1.5px;
    text-transform: uppercase;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 6px;
    background-color: #0e1117;
}

QLabel {
    color: #9ab0cc;
    font-size: 12px;
}
QLabel#header {
    color: #e0e6f0;
    font-size: 22px;
    font-weight: bold;
    letter-spacing: 2px;
}
QLabel#subheader {
    color: #5b8af5;
    font-size: 11px;
    letter-spacing: 3px;
}
QLabel#start_val {
    color: #3a6fd8;
    font-size: 11px;
}
QLabel#limit_note {
    color: #4a5a70;
    font-size: 10px;
    font-style: italic;
}

QDoubleSpinBox, QSpinBox {
    background-color: #151c28;
    border: 1px solid #2a3a55;
    border-radius: 4px;
    color: #e0e6f0;
    padding: 4px 8px;
    font-size: 13px;
    min-height: 28px;
    min-width: 90px;
}
QDoubleSpinBox:focus, QSpinBox:focus {
    border: 1px solid #5b8af5;
}
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button,
QSpinBox::up-button, QSpinBox::down-button {
    background-color: #1e2a3a;
    border: none;
    width: 18px;
}
QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover,
QSpinBox::up-button:hover, QSpinBox::down-button:hover {
    background-color: #2a3a55;
}

QComboBox {
    background-color: #151c28;
    border: 1px solid #2a3a55;
    border-radius: 4px;
    color: #e0e6f0;
    padding: 5px 10px;
    font-size: 13px;
    min-height: 28px;
    min-width: 200px;
}
QComboBox:focus { border: 1px solid #5b8af5; }
QComboBox::drop-down {
    border: none;
    width: 24px;
}
QComboBox QAbstractItemView {
    background-color: #151c28;
    border: 1px solid #2a3a55;
    color: #e0e6f0;
    selection-background-color: #1e3a6e;
}

QPushButton#run_btn {
    background-color: #1a3d8f;
    color: #e0f0ff;
    border: none;
    border-radius: 6px;
    font-size: 14px;
    font-weight: bold;
    letter-spacing: 2px;
    padding: 12px 0;
    min-height: 44px;
}
QPushButton#run_btn:hover {
    background-color: #2550b8;
}
QPushButton#run_btn:pressed {
    background-color: #0f2860;
}
QPushButton#run_btn:disabled {
    background-color: #1a2230;
    color: #3a4a60;
}

QPushButton#clear_btn {
    background-color: #1a2230;
    color: #6a7a90;
    border: 1px solid #2a3a55;
    border-radius: 6px;
    font-size: 11px;
    letter-spacing: 1px;
    padding: 6px 16px;
}
QPushButton#clear_btn:hover {
    background-color: #222f42;
    color: #9ab0cc;
}

QTextEdit {
    background-color: #080d14;
    border: 1px solid #1a2535;
    border-radius: 4px;
    color: #7aad6e;
    font-family: 'JetBrains Mono', 'Fira Mono', 'Courier New', monospace;
    font-size: 11px;
    padding: 6px;
}

QFrame#divider {
    background-color: #1a2535;
    max-height: 1px;
}

QFrame#range_badge {
    background-color: #0f1a2a;
    border: 1px solid #1e3050;
    border-radius: 3px;
    padding: 2px 6px;
}

QScrollBar:vertical {
    background: #0e1117;
    width: 8px;
    margin: 0;
}
QScrollBar::handle:vertical {
    background: #2a3a55;
    border-radius: 4px;
    min-height: 20px;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }

QSplitter::handle {
    background-color: #1a2535;
    width: 2px;
}
"""


# ══════════════════════════════════════════════════════════════
#  RANGE VALIDATOR  —  checks a value is in any valid range
# ══════════════════════════════════════════════════════════════

def in_valid_range(value: float, ranges: list) -> bool:
    return any(lo <= value <= hi for lo, hi in ranges)


def range_label(ranges: list) -> str:
    parts = [f"[{lo}°, {hi}°]" for lo, hi in ranges]
    return "  or  ".join(parts)


# ══════════════════════════════════════════════════════════════
#  JOINT ANGLE SPINBOX  —  validates against allowed ranges
# ══════════════════════════════════════════════════════════════

class JointSpinBox(QDoubleSpinBox):
    def __init__(self, ranges: list, parent=None):
        super().__init__(parent)
        self._ranges = ranges
        # Use the widest span for the spinbox min/max
        lo = min(r[0] for r in ranges)
        hi = max(r[1] for r in ranges)
        self.setRange(lo, hi)
        self.setDecimals(1)
        self.setSingleStep(1.0)
        self.setSuffix(" °")
        self._apply_style(self.value())
        self.valueChanged.connect(self._on_value_changed)

    def _on_value_changed(self, val):
        self._apply_style(val)

    def _apply_style(self, val):
        if in_valid_range(val, self._ranges):
            self.setStyleSheet("")
        else:
            self.setStyleSheet(
                "QDoubleSpinBox { border: 1px solid #c0392b; color: #e74c3c; }"
            )

    def is_valid(self) -> bool:
        return in_valid_range(self.value(), self._ranges)


# ══════════════════════════════════════════════════════════════
#  SHELL RUNNER THREAD
# ══════════════════════════════════════════════════════════════

class ShellThread(QThread):
    output   = pyqtSignal(str)
    finished = pyqtSignal(int)   # exit code

    def __init__(self, cmd: list):
        super().__init__()
        self._cmd = cmd

    def run(self):
        try:
            proc = subprocess.Popen(
                self._cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            for line in proc.stdout:
                self.output.emit(line.rstrip())
            proc.wait()
            self.finished.emit(proc.returncode)
        except Exception as e:
            self.output.emit(f"[ERROR] {e}")
            self.finished.emit(1)


# ══════════════════════════════════════════════════════════════
#  JOINT ROW WIDGET
# ══════════════════════════════════════════════════════════════

class JointRow(QWidget):
    def __init__(self, label: str, ranges: list, start_deg: float, parent=None):
        super().__init__(parent)
        self._ranges = ranges

        row = QHBoxLayout(self)
        row.setContentsMargins(0, 4, 0, 4)
        row.setSpacing(12)

        # Joint label
        lbl = QLabel(label)
        lbl.setFixedWidth(110)
        row.addWidget(lbl)

        # Spinbox
        self.spin = JointSpinBox(ranges)
        self.spin.setValue(start_deg)
        row.addWidget(self.spin)

        # Start angle badge
        start_lbl = QLabel(f"start: {start_deg:+.0f}°")
        start_lbl.setObjectName("start_val")
        start_lbl.setFixedWidth(80)
        row.addWidget(start_lbl)

        # Allowed range hint
        hint = QLabel(range_label(ranges))
        hint.setObjectName("limit_note")
        row.addWidget(hint)

        row.addStretch()

    def value(self) -> float:
        return self.spin.value()

    def is_valid(self) -> bool:
        return self.spin.is_valid()


# ══════════════════════════════════════════════════════════════
#  MAIN WINDOW
# ══════════════════════════════════════════════════════════════

class TrajectoryGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PUMA-560  ·  Trajectory Launcher")
        self.setMinimumSize(860, 700)
        self._thread = None
        self._build_ui()

    # ── UI construction ──────────────────────────────────────

    def _build_ui(self):
        self.setStyleSheet(STYLE)

        central = QWidget()
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(24, 20, 24, 20)
        root.setSpacing(16)

        # Header
        root.addLayout(self._make_header())

        divider = QFrame(); divider.setObjectName("divider"); divider.setFrameShape(QFrame.HLine)
        root.addWidget(divider)

        # Splitter: form (left) | console (right)
        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)

        form_scroll = QScrollArea()
        form_scroll.setWidgetResizable(True)
        form_scroll.setFrameShape(QFrame.NoFrame)
        form_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        form_container = QWidget()
        form_layout = QVBoxLayout(form_container)
        form_layout.setContentsMargins(0, 0, 12, 0)
        form_layout.setSpacing(14)

        # ── Curve type ──────────────────────────────────────
        form_layout.addWidget(self._make_curve_group())

        # ── End point ───────────────────────────────────────
        form_layout.addWidget(self._make_endpoint_group())

        # ── Mid point (hidden for single-S) ─────────────────
        self._mid_group = self._make_midpoint_group()
        form_layout.addWidget(self._mid_group)

        # ── Timing ──────────────────────────────────────────
        form_layout.addWidget(self._make_timing_group())

        # ── Run controls ────────────────────────────────────
        form_layout.addLayout(self._make_run_controls())

        form_layout.addStretch()
        form_scroll.setWidget(form_container)
        splitter.addWidget(form_scroll)

        # ── Console ─────────────────────────────────────────
        console_frame = QWidget()
        console_layout = QVBoxLayout(console_frame)
        console_layout.setContentsMargins(8, 0, 0, 0)
        console_layout.setSpacing(6)

        console_hdr = QHBoxLayout()
        console_lbl = QLabel("CONSOLE OUTPUT")
        console_lbl.setObjectName("subheader")
        console_hdr.addWidget(console_lbl)
        console_hdr.addStretch()
        clear_btn = QPushButton("CLEAR")
        clear_btn.setObjectName("clear_btn")
        clear_btn.setFixedHeight(26)
        clear_btn.clicked.connect(self._clear_console)
        console_hdr.addWidget(clear_btn)
        console_layout.addLayout(console_hdr)

        self._console = QTextEdit()
        self._console.setReadOnly(True)
        self._console.setPlaceholderText("Output will appear here…")
        console_layout.addWidget(self._console)

        splitter.addWidget(console_frame)
        splitter.setSizes([480, 340])

        root.addWidget(splitter, stretch=1)

        # Initial visibility
        self._on_curve_changed(self._curve_combo.currentIndex())

    def _make_header(self):
        hdr = QVBoxLayout()
        hdr.setSpacing(2)
        title = QLabel("PUMA-560")
        title.setObjectName("header")
        sub   = QLabel("TRAJECTORY  LAUNCHER")
        sub.setObjectName("subheader")
        hdr.addWidget(title)
        hdr.addWidget(sub)
        return hdr

    def _make_curve_group(self):
        grp = QGroupBox("Trajectory Profile")
        lay = QHBoxLayout(grp)
        lay.setSpacing(16)

        lbl = QLabel("Curve type:")
        lbl.setFixedWidth(90)
        lay.addWidget(lbl)

        self._curve_combo = QComboBox()
        self._curve_combo.addItem("Single S-Curve  (start → end)",        "single")
        self._curve_combo.addItem("Double S-Curve  (start → mid → end)",  "double")
        self._curve_combo.currentIndexChanged.connect(self._on_curve_changed)
        lay.addWidget(self._curve_combo)
        lay.addStretch()
        return grp

    def _make_endpoint_group(self):
        grp = QGroupBox("End Point  (q-end)")
        lay = QVBoxLayout(grp)
        lay.setSpacing(4)
        self._end_rows = []
        names = list(JOINT_LIMITS.keys())
        for i, (name, ranges) in enumerate(JOINT_LIMITS.items()):
            row = JointRow(name, ranges, Q_START_DEG[i])
            lay.addWidget(row)
            self._end_rows.append(row)
        return grp

    def _make_midpoint_group(self):
        grp = QGroupBox("Mid Point  (q-mid)  —  waypoint")
        lay = QVBoxLayout(grp)
        lay.setSpacing(4)
        self._mid_rows = []
        for i, (name, ranges) in enumerate(JOINT_LIMITS.items()):
            row = JointRow(name, ranges, Q_START_DEG[i])
            lay.addWidget(row)
            self._mid_rows.append(row)
        return grp

    def _make_timing_group(self):
        grp = QGroupBox("Timing")
        grid = QGridLayout(grp)
        grid.setSpacing(10)
        grid.setColumnStretch(2, 1)

        # t-total
        grid.addWidget(QLabel("Duration  (t-total):"), 0, 0)
        self._t_spin = QDoubleSpinBox()
        self._t_spin.setRange(T_TOTAL_MIN, T_TOTAL_MAX)
        self._t_spin.setValue(T_TOTAL_DEF)
        self._t_spin.setDecimals(1)
        self._t_spin.setSingleStep(0.5)
        self._t_spin.setSuffix("  s")
        grid.addWidget(self._t_spin, 0, 1)
        grid.addWidget(QLabel(f"range: {T_TOTAL_MIN} – {T_TOTAL_MAX} s"), 0, 2)

        # num-paths
        grid.addWidget(QLabel("Num paths:"), 1, 0)
        self._np_spin = QSpinBox()
        self._np_spin.setRange(NUM_PATHS_MIN, NUM_PATHS_MAX)
        self._np_spin.setValue(1)
        grid.addWidget(self._np_spin, 1, 1)
        grid.addWidget(QLabel(f"range: {NUM_PATHS_MIN} – {NUM_PATHS_MAX}"), 1, 2)

        return grp

    def _make_run_controls(self):
        lay = QVBoxLayout()
        lay.setSpacing(8)

        # Command preview
        self._cmd_label = QLabel()
        self._cmd_label.setObjectName("limit_note")
        self._cmd_label.setWordWrap(True)
        lay.addWidget(self._cmd_label)

        self._run_btn = QPushButton("▶   RUN TRAJECTORY")
        self._run_btn.setObjectName("run_btn")
        self._run_btn.clicked.connect(self._on_run)
        lay.addWidget(self._run_btn)

        # Wire up live preview
        for row in self._end_rows + self._mid_rows:
            row.spin.valueChanged.connect(self._update_preview)
        self._t_spin.valueChanged.connect(self._update_preview)
        self._np_spin.valueChanged.connect(self._update_preview)
        self._curve_combo.currentIndexChanged.connect(self._update_preview)

        self._update_preview()
        return lay

    # ── slots ────────────────────────────────────────────────

    def _on_curve_changed(self, index):
        is_double = (self._curve_combo.itemData(index) == "double")
        self._mid_group.setVisible(is_double)
        self._update_preview()

    def _build_command(self) -> list:
        """Return the shell command as a list of strings."""
        is_double = (self._curve_combo.currentData() == "double")

        q_end = [r.value() for r in self._end_rows]
        q_end_str = f"[{q_end[0]:.1f},{q_end[1]:.1f},{q_end[2]:.1f}]"

        cmd = [
            "bash", SCRIPT_PATH,
            "--q-end",     q_end_str,
            "--t-total",   str(self._t_spin.value()),
            "--num-paths", str(self._np_spin.value()),
            "--base-dir",  BASE_DIR,
        ]

        if is_double:
            q_mid = [r.value() for r in self._mid_rows]
            q_mid_str = f"[{q_mid[0]:.1f},{q_mid[1]:.1f},{q_mid[2]:.1f}]"
            cmd += ["--q-mid", q_mid_str]

        return cmd

    def _update_preview(self):
        cmd = self._build_command()
        preview = " ".join(shlex.quote(c) for c in cmd)
        self._cmd_label.setText(f"$ {preview}")

    def _validate(self) -> bool:
        is_double = (self._curve_combo.currentData() == "double")
        rows = self._end_rows + (self._mid_rows if is_double else [])
        bad = [r for r in rows if not r.is_valid()]
        if bad:
            self._log("⚠  One or more joint angles are outside valid ranges. Please correct (highlighted in red).",
                      color="#e74c3c")
            return False
        return True

    def _on_run(self):
        if not self._validate():
            return
        if self._thread and self._thread.isRunning():
            self._log("⚠  Already running — please wait.", color="#f39c12")
            return

        cmd = self._build_command()
        self._log(f"\n$ {' '.join(shlex.quote(c) for c in cmd)}", color="#5b8af5")
        self._run_btn.setEnabled(False)
        self._run_btn.setText("⏳   RUNNING…")

        self._thread = ShellThread(cmd)
        self._thread.output.connect(self._log)
        self._thread.finished.connect(self._on_finished)
        self._thread.start()

    def _on_finished(self, code: int):
        if code == 0:
            self._log("\n✔  Done — trajectory generated and simulation launched.", color="#2ecc71")
        else:
            self._log(f"\n✘  Process exited with code {code}.", color="#e74c3c")
        self._run_btn.setEnabled(True)
        self._run_btn.setText("▶   RUN TRAJECTORY")

    def _log(self, text: str, color: str = "#7aad6e"):
        cursor = self._console.textCursor()
        cursor.movePosition(cursor.End)
        self._console.setTextCursor(cursor)
        self._console.setTextColor(QColor(color))
        self._console.insertPlainText(text + "\n")
        self._console.ensureCursorVisible()

    def _clear_console(self):
        self._console.clear()


# ══════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════

def main():
    app = QApplication(sys.argv)
    app.setApplicationName("PUMA-560 Trajectory Launcher")

    win = TrajectoryGUI()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()