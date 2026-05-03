#!/usr/bin/env python3
"""
PUMA-560 Trajectory GUI Launcher  —  Light Theme
PyQt5 control panel for run_traj.sh + Gazebo orchestration.

Flow:
  1. RUN pressed  →  SimMonitorThread starts gazebo_launch.sh (single process, piped)
  2. Simulation output streams into the "Simulation" console tab in the GUI
  3. All 3 controller-ready lines detected  →  trajectory generation runs in background
     (automater.sh handles both generation AND execution internally)
  4. STOP kills everything and returns to idle
"""

import sys
import os
import re
import subprocess
import shlex
import signal
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QDoubleSpinBox, QSpinBox, QComboBox,
    QPushButton, QGroupBox, QFrame, QScrollArea, QTextEdit,
    QSplitter, QTabWidget
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPalette


# ══════════════════════════════════════════════════════════════
#  CONFIGURATION
# ══════════════════════════════════════════════════════════════

TRAJ_GEN_SCRIPT   = "/home/priyankan/Desktop/FYP-Puma_560/arm_bot/src/scripts/Demo_controller/automater.sh"
SIMULATION_SCRIPT = "/home/priyankan/Desktop/FYP-Puma_560/arm_bot/src/scripts/Demo_controller/gazebo_launch.sh"
BASE_DIR          = "/home/priyankan/Desktop/FYP-Puma_560/arm_bot/src/scripts/Demo_controller/demo_trajectories"

READY_LINES = [
    "Configured and activated joint_3_controller",
    "Configured and activated joint_2_controller",
    "Configured and activated joint_1_controller",
]

# Pre-compiled regex to strip ANSI/VT100 escape sequences from ROS2 output.
# ROS2 spawner nodes emit colour codes like \x1b[92m, \x1b[1m, \x1b[0m which
# corrupt plain-text matching unless removed first.  The actual lines look like:
#   [spawner-8] ... [92mConfigured and activated [1mjoint_1_controller[0m
import re as _re
_ANSI_ESC = _re.compile(r'\x1b\[[0-9;]*[A-Za-z]')

def strip_ansi(text: str) -> str:
    return _ANSI_ESC.sub('', text)


JOINT_LIMITS = {
    "Q1 (Base)":     [(-170, -20), (20, 170)],
    "Q2 (Shoulder)": [(-90, 180)],
    "Q3 (Elbow)":    [(-45, 225)],
}

Q_START_DEG   = [0.0, 45.0, 135.0]
T_TOTAL_MIN   = 5.0
T_TOTAL_MAX   = 60.0
T_TOTAL_DEF   = 18.0
NUM_PATHS_MIN = 1
NUM_PATHS_MAX = 20


# ══════════════════════════════════════════════════════════════
#  LIGHT STYLE SHEET
# ══════════════════════════════════════════════════════════════

STYLE = """
/* ── Base ── */
QMainWindow, QWidget {
    background-color: #f4f6fb;
    color: #1a2236;
    font-family: 'Segoe UI', 'Inter', 'Helvetica Neue', Arial, sans-serif;
    font-size: 12px;
}

/* ── Group boxes ── */
QGroupBox {
    background-color: #ffffff;
    border: 1px solid #d0d8e8;
    border-radius: 8px;
    margin-top: 16px;
    padding: 14px 10px 10px 10px;
    font-size: 10px;
    font-weight: bold;
    color: #3a6fd8;
    letter-spacing: 1.5px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 6px;
    background-color: #f4f6fb;
}

/* ── Labels ── */
QLabel                { color: #3a4a60; font-size: 12px; }
QLabel#header         { color: #1a2236; font-size: 22px; font-weight: bold; letter-spacing: 1px; }
QLabel#subheader      { color: #3a6fd8; font-size: 11px; letter-spacing: 3px; font-weight: bold; }
QLabel#start_val      { color: #7090c0; font-size: 11px; }
QLabel#limit_note     { color: #a0aec0; font-size: 10px; font-style: italic; }

/* ── Spin boxes ── */
QDoubleSpinBox, QSpinBox {
    background-color: #f8faff;
    border: 1.5px solid #c8d4e8;
    border-radius: 5px;
    color: #1a2236;
    padding: 4px 8px;
    font-size: 13px;
    min-height: 28px;
    min-width: 90px;
}
QDoubleSpinBox:focus, QSpinBox:focus {
    border: 1.5px solid #3a6fd8;
    background-color: #ffffff;
}
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button,
QSpinBox::up-button,       QSpinBox::down-button {
    background-color: #e8eef8;
    border: none;
    width: 18px;
}
QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover,
QSpinBox::up-button:hover,       QSpinBox::down-button:hover {
    background-color: #c8d4e8;
}

/* ── Combo box ── */
QComboBox {
    background-color: #f8faff;
    border: 1.5px solid #c8d4e8;
    border-radius: 5px;
    color: #1a2236;
    padding: 5px 10px;
    font-size: 12px;
    min-height: 28px;
    min-width: 220px;
}
QComboBox:focus { border: 1.5px solid #3a6fd8; background-color: #ffffff; }
QComboBox::drop-down { border: none; width: 24px; }
QComboBox QAbstractItemView {
    background-color: #ffffff;
    border: 1px solid #c8d4e8;
    color: #1a2236;
    selection-background-color: #dce8ff;
    selection-color: #1a2236;
}

/* ── RUN button ── */
QPushButton#run_btn {
    background-color: #2c5fcc;
    color: #ffffff;
    border: none;
    border-radius: 7px;
    font-size: 14px;
    font-weight: bold;
    letter-spacing: 1.5px;
    padding: 12px 0;
    min-height: 44px;
}
QPushButton#run_btn:hover    { background-color: #3a6fd8; }
QPushButton#run_btn:pressed  { background-color: #1a4ab0; }
QPushButton#run_btn:disabled { background-color: #c8d4e8; color: #90a0b8; }

/* ── STOP button ── */
QPushButton#stop_btn {
    background-color: #cc2c2c;
    color: #ffffff;
    border: none;
    border-radius: 7px;
    font-size: 14px;
    font-weight: bold;
    letter-spacing: 1.5px;
    padding: 12px 0;
    min-height: 44px;
}
QPushButton#stop_btn:hover   { background-color: #d84040; }
QPushButton#stop_btn:pressed { background-color: #a01818; }

/* ── Clear button ── */
QPushButton#clear_btn {
    background-color: #eef2fa;
    color: #6070a0;
    border: 1px solid #c8d4e8;
    border-radius: 5px;
    font-size: 11px;
    padding: 5px 14px;
}
QPushButton#clear_btn:hover { background-color: #dce8ff; color: #2c5fcc; }

/* ── Console / text area ── */
QTextEdit {
    background-color: #f8faff;
    border: 1px solid #d0d8e8;
    border-radius: 5px;
    color: #1a2236;
    font-family: 'JetBrains Mono', 'Fira Mono', 'Courier New', monospace;
    font-size: 11px;
    padding: 6px;
}

/* ── Tabs ── */
QTabWidget::pane {
    border: 1px solid #d0d8e8;
    border-radius: 0 6px 6px 6px;
    background-color: #ffffff;
}
QTabBar::tab {
    background-color: #e8eef8;
    color: #5070a0;
    border: 1px solid #d0d8e8;
    border-bottom: none;
    border-radius: 5px 5px 0 0;
    padding: 5px 16px;
    font-size: 11px;
    font-weight: bold;
    letter-spacing: 0.5px;
    margin-right: 2px;
}
QTabBar::tab:selected {
    background-color: #ffffff;
    color: #2c5fcc;
    border-bottom: 2px solid #ffffff;
}
QTabBar::tab:hover:!selected { background-color: #dce8ff; }

/* ── Divider ── */
QFrame#divider { background-color: #d0d8e8; max-height: 1px; }

/* ── Scroll bars ── */
QScrollBar:vertical { background: #f4f6fb; width: 8px; margin: 0; }
QScrollBar::handle:vertical { background: #c8d4e8; border-radius: 4px; min-height: 20px; }
QScrollBar::handle:vertical:hover { background: #a0b4d0; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }

/* ── Splitter ── */
QSplitter::handle { background-color: #d0d8e8; width: 2px; }
"""


# ══════════════════════════════════════════════════════════════
#  HELPERS
# ══════════════════════════════════════════════════════════════

def in_valid_range(value: float, ranges: list) -> bool:
    return any(lo <= value <= hi for lo, hi in ranges)

def range_label(ranges: list) -> str:
    return "  or  ".join(f"[{lo}°, {hi}°]" for lo, hi in ranges)

def graceful_stop(proc, sigint_timeout: float = 5.0):
    """
    Send SIGINT (Ctrl+C) to the process group, wait up to sigint_timeout
    seconds for it to exit cleanly, then escalate to SIGTERM, and finally
    SIGKILL if the process still hasn't stopped.

    This mirrors what a user pressing Ctrl+C in a terminal would do and
    allows ROS2/Gazebo nodes to shut down cleanly via their signal handlers.
    """
    if proc is None:
        return

    try:
        pgid = os.getpgid(proc.pid)
    except OSError:
        return  # process already gone

    # ── 1. SIGINT — the polite Ctrl+C ────────────────────────
    try:
        os.killpg(pgid, signal.SIGINT)
    except OSError:
        return  # already gone

    try:
        proc.wait(timeout=sigint_timeout)
        return   # exited cleanly after SIGINT
    except subprocess.TimeoutExpired:
        pass

    # ── 2. SIGTERM — standard termination request ─────────────
    try:
        os.killpg(pgid, signal.SIGTERM)
    except OSError:
        return

    try:
        proc.wait(timeout=3.0)
        return
    except subprocess.TimeoutExpired:
        pass

    # ── 3. SIGKILL — force kill as last resort ────────────────
    try:
        os.killpg(pgid, signal.SIGKILL)
    except OSError:
        pass


# ══════════════════════════════════════════════════════════════
#  JOINT WIDGETS
# ══════════════════════════════════════════════════════════════

class JointSpinBox(QDoubleSpinBox):
    def __init__(self, ranges, parent=None):
        super().__init__(parent)
        self._ranges = ranges
        self.setRange(min(r[0] for r in ranges), max(r[1] for r in ranges))
        self.setDecimals(1)
        self.setSingleStep(1.0)
        self.setSuffix(" °")
        self._refresh(self.value())
        self.valueChanged.connect(self._refresh)

    def _refresh(self, val):
        ok = in_valid_range(val, self._ranges)
        self.setStyleSheet("" if ok else
            "QDoubleSpinBox { border: 1.5px solid #cc2c2c; color: #cc2c2c; "
            "background-color: #fff5f5; }")

    def is_valid(self):
        return in_valid_range(self.value(), self._ranges)


class JointRow(QWidget):
    def __init__(self, label, ranges, start_deg, parent=None):
        super().__init__(parent)
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 4, 0, 4)
        row.setSpacing(12)

        lbl = QLabel(label); lbl.setFixedWidth(115)
        row.addWidget(lbl)

        self.spin = JointSpinBox(ranges)
        self.spin.setValue(start_deg)
        row.addWidget(self.spin)

        s = QLabel(f"start: {start_deg:+.0f}°")
        s.setObjectName("start_val"); s.setFixedWidth(82)
        row.addWidget(s)

        h = QLabel(range_label(ranges)); h.setObjectName("limit_note")
        row.addWidget(h)
        row.addStretch()

    def value(self):    return self.spin.value()
    def is_valid(self): return self.spin.is_valid()


# ══════════════════════════════════════════════════════════════
#  SIMULATION MONITOR THREAD
#  Runs ONE simulation process with a pipe.
#  Output is forwarded to the GUI via signals.
# ══════════════════════════════════════════════════════════════

class SimMonitorThread(QThread):
    line_received     = pyqtSignal(str)
    controllers_ready = pyqtSignal()
    process_ended     = pyqtSignal(int)

    def __init__(self, cmd: list):
        super().__init__()
        self._cmd  = cmd
        self._proc = None

    def run(self):
        try:
            self._proc = subprocess.Popen(
                self._cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            seen          = set()
            ready_emitted = False
            for raw in self._proc.stdout:
                line = strip_ansi(raw.rstrip())   # remove ANSI codes before match & display
                self.line_received.emit(line)
                if not ready_emitted:
                    for marker in READY_LINES:
                        if marker in line:
                            seen.add(marker)
                    if len(seen) == len(READY_LINES):
                        ready_emitted = True
                        self.controllers_ready.emit()
            self._proc.wait()
            self.process_ended.emit(self._proc.returncode)
        except Exception as e:
            self.line_received.emit(f"[SIM ERROR] {e}")
            self.process_ended.emit(1)

    def stop(self):
        if self._proc:
            # Use a longer SIGINT timeout — Gazebo can take several seconds
            # to shut down its nodes cleanly after receiving Ctrl+C.
            graceful_stop(self._proc, sigint_timeout=8.0)
            self._proc = None


# ══════════════════════════════════════════════════════════════
#  GENERIC BACKGROUND PROCESS THREAD  (trajectory generation + execution)
# ══════════════════════════════════════════════════════════════

class ProcThread(QThread):
    line_received = pyqtSignal(str)
    process_ended = pyqtSignal(int)

    def __init__(self, cmd: list):
        super().__init__()
        self._cmd  = cmd
        self._proc = None

    def run(self):
        try:
            self._proc = subprocess.Popen(
                self._cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            for raw in self._proc.stdout:
                self.line_received.emit(raw.rstrip())
            self._proc.wait()
            self.process_ended.emit(self._proc.returncode)
        except Exception as e:
            self.line_received.emit(f"[ERROR] {e}")
            self.process_ended.emit(1)

    def stop(self):
        if self._proc:
            graceful_stop(self._proc, sigint_timeout=4.0)
            self._proc = None


# ══════════════════════════════════════════════════════════════
#  MAIN WINDOW
# ══════════════════════════════════════════════════════════════

class TrajectoryGUI(QMainWindow):

    ST_IDLE     = "idle"
    ST_SIM      = "sim_starting"
    ST_WAITING  = "waiting_controllers"
    ST_RUNNING  = "running"       # automater.sh is generating + executing the trajectory
    ST_FINISHED = "finished"      # automater.sh exited; sim still alive, awaiting STOP

    # (label, text-colour, border/bg accent, background)
    _STATE_THEME = {
        ST_IDLE:     ("IDLE",                                         "#5070a0", "#c8d4e8", "#f0f4fa"),
        ST_SIM:      ("SIMULATION STARTING…",                         "#b05010", "#f0c060", "#fff8e8"),
        ST_WAITING:  ("WAITING FOR CONTROLLERS…",                     "#b05010", "#f0c060", "#fff8e8"),
        ST_RUNNING:  ("TRAJECTORY RUNNING",                           "#1a7a40", "#60c080", "#edfff4"),
        ST_FINISHED: ("EXECUTION COMPLETE  —  PRESS STOP TO RESET",   "#1a5c30", "#50b070", "#edfff4"),
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle("PUMA-560  ·  Trajectory Launcher")
        self.setMinimumSize(980, 760)

        self._state      = self.ST_IDLE
        self._sim_thread = None   # SimMonitorThread — the ONE simulation process
        self._gen_thread = None   # ProcThread for automater.sh (generation + execution)
        self._gen_cmd    = None

        self._build_ui()

    # ─────────────────────────────────────────────────────────
    #  UI
    # ─────────────────────────────────────────────────────────

    def _build_ui(self):
        self.setStyleSheet(STYLE)
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(24, 20, 24, 20)
        root.setSpacing(14)

        root.addLayout(self._make_header())
        div = QFrame(); div.setObjectName("divider"); div.setFrameShape(QFrame.HLine)
        root.addWidget(div)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)

        # ── Left: scrollable form ────────────────────────────
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        form_w = QWidget()
        form   = QVBoxLayout(form_w)
        form.setContentsMargins(0, 0, 12, 0)
        form.setSpacing(14)

        form.addWidget(self._make_curve_group())
        form.addWidget(self._make_endpoint_group())
        self._mid_group = self._make_midpoint_group()
        form.addWidget(self._mid_group)
        form.addWidget(self._make_timing_group())
        form.addWidget(self._make_status_group())
        form.addLayout(self._make_action_buttons())
        form.addStretch()

        scroll.setWidget(form_w)
        splitter.addWidget(scroll)

        # ── Right: tabbed console ────────────────────────────
        right_w   = QWidget()
        right_lay = QVBoxLayout(right_w)
        right_lay.setContentsMargins(8, 0, 0, 0)
        right_lay.setSpacing(6)

        self._tabs = QTabWidget()

        # Tab 1 — Simulation output
        sim_tab = QWidget()
        sim_lay = QVBoxLayout(sim_tab)
        sim_lay.setContentsMargins(4, 4, 4, 4)
        self._sim_console = QTextEdit()
        self._sim_console.setReadOnly(True)
        self._sim_console.setPlaceholderText("Gazebo simulation output will appear here…")
        sim_lay.addWidget(self._sim_console)
        self._tabs.addTab(sim_tab, "SIMULATION")

        # Tab 2 — Generation / execution log
        gen_tab = QWidget()
        gen_lay = QVBoxLayout(gen_tab)
        gen_lay.setContentsMargins(4, 4, 4, 4)
        self._gen_console = QTextEdit()
        self._gen_console.setReadOnly(True)
        self._gen_console.setPlaceholderText("Trajectory generation & execution output will appear here…")
        gen_lay.addWidget(self._gen_console)
        self._tabs.addTab(gen_tab, "TRAJECTORY")

        right_lay.addWidget(self._tabs, stretch=1)

        # Clear button row below tabs
        cb_row = QHBoxLayout()
        cb_row.addStretch()
        clear_btn = QPushButton("CLEAR TAB")
        clear_btn.setObjectName("clear_btn")
        clear_btn.setFixedHeight(26)
        clear_btn.clicked.connect(self._clear_current_tab)
        cb_row.addWidget(clear_btn)
        right_lay.addLayout(cb_row)

        splitter.addWidget(right_w)
        splitter.setSizes([520, 420])
        root.addWidget(splitter, stretch=1)

        self._on_curve_changed(self._curve_combo.currentIndex())
        self._apply_state(self.ST_IDLE)

    def _make_header(self):
        lay = QHBoxLayout()
        txt = QVBoxLayout(); txt.setSpacing(2)
        txt.addWidget(self._lbl("PUMA-560", "header"))
        txt.addWidget(self._lbl("TRAJECTORY  LAUNCHER", "subheader"))
        lay.addLayout(txt)
        lay.addStretch()
        return lay

    def _make_curve_group(self):
        grp = QGroupBox("TRAJECTORY PROFILE")
        lay = QHBoxLayout(grp); lay.setSpacing(16)
        lay.addWidget(self._lbl("Curve type:", fixed_w=90))
        self._curve_combo = QComboBox()
        self._curve_combo.addItem("Single S-Curve  (start → end)",       "single")
        self._curve_combo.addItem("Double S-Curve  (start → mid → end)", "double")
        self._curve_combo.currentIndexChanged.connect(self._on_curve_changed)
        lay.addWidget(self._curve_combo)
        lay.addStretch()
        return grp

    def _make_endpoint_group(self):
        grp = QGroupBox("END POINT  (q-end)")
        lay = QVBoxLayout(grp); lay.setSpacing(4)
        self._end_rows = []
        for i, (name, ranges) in enumerate(JOINT_LIMITS.items()):
            r = JointRow(name, ranges, Q_START_DEG[i])
            lay.addWidget(r); self._end_rows.append(r)
        return grp

    def _make_midpoint_group(self):
        grp = QGroupBox("MID POINT  (q-mid)  —  waypoint")
        lay = QVBoxLayout(grp); lay.setSpacing(4)
        self._mid_rows = []
        for i, (name, ranges) in enumerate(JOINT_LIMITS.items()):
            r = JointRow(name, ranges, Q_START_DEG[i])
            lay.addWidget(r); self._mid_rows.append(r)
        return grp

    def _make_timing_group(self):
        grp  = QGroupBox("TIMING")
        grid = QGridLayout(grp); grid.setSpacing(10); grid.setColumnStretch(2, 1)

        grid.addWidget(self._lbl("Duration  (t-total):"), 0, 0)
        self._t_spin = QDoubleSpinBox()
        self._t_spin.setRange(T_TOTAL_MIN, T_TOTAL_MAX)
        self._t_spin.setValue(T_TOTAL_DEF)
        self._t_spin.setDecimals(1); self._t_spin.setSingleStep(0.5)
        self._t_spin.setSuffix("  s")
        grid.addWidget(self._t_spin, 0, 1)
        grid.addWidget(self._lbl(f"range: {T_TOTAL_MIN} – {T_TOTAL_MAX} s", "limit_note"), 0, 2)

        grid.addWidget(self._lbl("Num paths:"), 1, 0)
        self._np_spin = QSpinBox()
        self._np_spin.setRange(NUM_PATHS_MIN, NUM_PATHS_MAX); self._np_spin.setValue(1)
        grid.addWidget(self._np_spin, 1, 1)
        grid.addWidget(self._lbl(f"range: {NUM_PATHS_MIN} – {NUM_PATHS_MAX}", "limit_note"), 1, 2)
        return grp

    def _make_status_group(self):
        grp = QGroupBox("STATUS")
        lay = QVBoxLayout(grp); lay.setSpacing(10)

        # Status badge
        self._status_lbl = QLabel("IDLE")
        self._status_lbl.setAlignment(Qt.AlignCenter)
        self._status_lbl.setMinimumHeight(32)
        lay.addWidget(self._status_lbl)

        # Controller checklist
        self._ctrl_widgets = {}
        for marker in READY_LINES:
            row = QHBoxLayout()
            dot = QLabel("○"); dot.setFixedWidth(20)
            dot.setStyleSheet("color: #b0bece; font-size: 15px;")
            name = marker.replace("Configured and activated ", "")
            txt  = QLabel(name)
            txt.setStyleSheet("color: #a0aec0; font-size: 11px;")
            row.addWidget(dot); row.addWidget(txt); row.addStretch()
            lay.addLayout(row)
            self._ctrl_widgets[marker] = (dot, txt)

        # Command preview
        self._cmd_preview = QLabel()
        self._cmd_preview.setObjectName("limit_note")
        self._cmd_preview.setWordWrap(True)
        lay.addWidget(self._cmd_preview)

        for r in self._end_rows + self._mid_rows:
            r.spin.valueChanged.connect(self._update_preview)
        self._t_spin.valueChanged.connect(self._update_preview)
        self._np_spin.valueChanged.connect(self._update_preview)
        self._curve_combo.currentIndexChanged.connect(self._update_preview)
        self._update_preview()
        return grp

    def _make_action_buttons(self):
        lay = QVBoxLayout(); lay.setSpacing(6)

        # ST_IDLE — initial run
        self._run_btn = QPushButton("▶   RUN TRAJECTORY")
        self._run_btn.setObjectName("run_btn")
        self._run_btn.clicked.connect(self._on_run)

        # ST_SIM / ST_WAITING / ST_RUNNING — abort everything
        self._stop_btn = QPushButton("■   STOP EXECUTION")
        self._stop_btn.setObjectName("stop_btn")
        self._stop_btn.clicked.connect(self._on_stop)
        self._stop_btn.setVisible(False)

        lay.addWidget(self._run_btn)
        lay.addWidget(self._stop_btn)
        return lay

    # ─────────────────────────────────────────────────────────
    #  STATE MACHINE
    # ─────────────────────────────────────────────────────────

    def _apply_state(self, state: str):
        self._state = state
        text, fg, accent, bg = self._STATE_THEME.get(
            state, ("IDLE", "#5070a0", "#c8d4e8", "#f0f4fa")
        )
        self._status_lbl.setText(text)
        self._status_lbl.setStyleSheet(
            f"QLabel {{ color: {fg}; background-color: {bg}; "
            f"border: 1.5px solid {accent}; border-radius: 6px; "
            f"font-size: 12px; font-weight: bold; letter-spacing: 1px; padding: 5px 12px; }}"
        )

        is_idle = (state == self.ST_IDLE)

        self._run_btn.setVisible(is_idle)
        self._stop_btn.setVisible(not is_idle)

        # Form is editable only when idle
        for w in self._end_rows + self._mid_rows:
            w.setEnabled(is_idle)
        self._curve_combo.setEnabled(is_idle)
        self._t_spin.setEnabled(is_idle)
        self._np_spin.setEnabled(is_idle)

    # ─────────────────────────────────────────────────────────
    #  FORM SLOTS
    # ─────────────────────────────────────────────────────────

    def _on_curve_changed(self, index):
        self._mid_group.setVisible(self._curve_combo.itemData(index) == "double")
        self._update_preview()

    def _update_preview(self):
        self._cmd_preview.setText(
            "$ " + " ".join(shlex.quote(c) for c in self._build_gen_command())
        )

    def _build_gen_command(self) -> list:
        is_double = (self._curve_combo.currentData() == "double")
        qe = [r.value() for r in self._end_rows]
        cmd = [
            "bash", TRAJ_GEN_SCRIPT,
            "--q-end",     f"[{qe[0]:.1f},{qe[1]:.1f},{qe[2]:.1f}]",
            "--t-total",   str(self._t_spin.value()),
            "--num-paths", str(self._np_spin.value()),
            "--base-dir",  BASE_DIR,
        ]
        if is_double:
            qm = [r.value() for r in self._mid_rows]
            cmd += ["--q-mid", f"[{qm[0]:.1f},{qm[1]:.1f},{qm[2]:.1f}]"]
        return cmd

    def _validate(self) -> bool:
        is_double = (self._curve_combo.currentData() == "double")
        rows = self._end_rows + (self._mid_rows if is_double else [])
        if any(not r.is_valid() for r in rows):
            self._log_gen("⚠  One or more joint angles are outside valid ranges.", "#cc2c2c")
            return False
        return True

    # ─────────────────────────────────────────────────────────
    #  RUN / STOP
    # ─────────────────────────────────────────────────────────

    def _on_run(self):
        if not self._validate():
            return

        self._gen_cmd = self._build_gen_command()

        # ── Fresh start — launch simulation first ─────────────────────────
        self._reset_checklist()
        self._sim_console.clear()
        self._gen_console.clear()

        self._log_sim("── STEP 1: Starting Gazebo simulation ──────────────────")
        self._sim_thread = SimMonitorThread(["bash", SIMULATION_SCRIPT])
        self._sim_thread.line_received.connect(self._on_sim_line)
        self._sim_thread.controllers_ready.connect(self._on_controllers_ready)
        self._sim_thread.process_ended.connect(self._on_sim_proc_ended)
        self._sim_thread.start()

        self._tabs.setCurrentIndex(0)
        self._apply_state(self.ST_SIM)
        self._log_sim("Waiting for Gazebo controllers to initialise…", "#b05010")

    def _on_stop(self):
        self._log_sim("── STOP requested: sending Ctrl+C (SIGINT) to simulation ─", "#cc2c2c")
        self._log_gen("── STOP requested: sending Ctrl+C (SIGINT) to simulation ─", "#cc2c2c")
        self._hard_reset()

    # ─────────────────────────────────────────────────────────
    #  SIMULATION MONITORING
    # ─────────────────────────────────────────────────────────

    def _on_sim_line(self, line: str):
        self._log_sim(line, "#3a5a80")
        for marker in READY_LINES:
            if marker in line:
                self._mark_controller(marker)
        # Transition out of ST_SIM as soon as first output arrives
        if self._state == self.ST_SIM:
            self._apply_state(self.ST_WAITING)

    def _on_controllers_ready(self):
        self._log_sim("── All controllers ready ─────────────────────────────────", "#1a7a40")
        self._apply_state(self.ST_RUNNING)
        self._tabs.setCurrentIndex(1)   # switch to trajectory tab

        self._log_gen("── STEP 2: Generating & executing trajectory ────────────")
        self._gen_thread = ProcThread(self._gen_cmd)
        self._gen_thread.line_received.connect(lambda l: self._log_gen(l, "#3a5a80"))
        self._gen_thread.process_ended.connect(self._on_gen_done)
        self._gen_thread.start()

    def _on_sim_proc_ended(self, code: int):
        if self._state != self.ST_IDLE:
            self._log_sim(f"[SIM] Process exited (code {code}).", "#b05010")

    # ─────────────────────────────────────────────────────────
    #  TRAJECTORY GENERATION + EXECUTION DONE
    # ─────────────────────────────────────────────────────────

    def _on_gen_done(self, code: int):
        if code != 0:
            self._log_gen(f"[GEN] automater.sh failed (exit {code}).", "#cc2c2c")
        else:
            self._log_gen("\n✔  Trajectory generation & execution complete.", "#1a7a40")
        self._log_gen("Simulation is still running. Press  ■ STOP  to terminate.", "#5070a0")
        self._gen_thread = None
        self._apply_state(self.ST_FINISHED)

    # ─────────────────────────────────────────────────────────
    #  HARD RESET
    # ─────────────────────────────────────────────────────────

    def _hard_reset(self):
        # ── Stop trajectory generator (automater.sh) if still running ──
        if self._gen_thread and self._gen_thread.isRunning():
            self._log_gen("[STOP] Sending Ctrl+C to automater.sh…", "#b05010")
            self._gen_thread.stop()
            self._gen_thread.wait(2000)
        self._gen_thread = None

        # ── Stop simulation (SIGINT → SIGTERM → SIGKILL, longer timeout) ─
        if self._sim_thread and self._sim_thread.isRunning():
            self._log_sim("[STOP] Sending Ctrl+C to Gazebo simulation…", "#b05010")
            self._sim_thread.stop()      # graceful_stop with 8 s SIGINT window
            self._sim_thread.wait(3000)  # wait for the QThread itself to join
        self._sim_thread = None

        self._reset_checklist()
        self._apply_state(self.ST_IDLE)
        self._log_sim("[STOP] Simulation terminated.", "#5070a0")
        self._log_gen("Ready for next trajectory.", "#5070a0")

    # ─────────────────────────────────────────────────────────
    #  CHECKLIST
    # ─────────────────────────────────────────────────────────

    def _mark_controller(self, marker: str):
        if marker in self._ctrl_widgets:
            dot, txt = self._ctrl_widgets[marker]
            dot.setText("●"); dot.setStyleSheet("color: #1a7a40; font-size: 15px;")
            txt.setStyleSheet("color: #1a7a40; font-size: 11px; font-weight: bold;")

    def _reset_checklist(self):
        for dot, txt in self._ctrl_widgets.values():
            dot.setText("○"); dot.setStyleSheet("color: #b0bece; font-size: 15px;")
            txt.setStyleSheet("color: #a0aec0; font-size: 11px;")

    # ─────────────────────────────────────────────────────────
    #  LOGGING
    # ─────────────────────────────────────────────────────────

    def _log_sim(self, text: str, color: str = "#2a3a50"):
        self._sim_console.setTextColor(QColor(color))
        self._sim_console.append(text)
        self._sim_console.ensureCursorVisible()

    def _log_gen(self, text: str, color: str = "#2a3a50"):
        self._gen_console.setTextColor(QColor(color))
        self._gen_console.append(text)
        self._gen_console.ensureCursorVisible()

    def _clear_current_tab(self):
        idx = self._tabs.currentIndex()
        (self._sim_console if idx == 0 else self._gen_console).clear()

    # ─────────────────────────────────────────────────────────
    #  UTILITIES
    # ─────────────────────────────────────────────────────────

    @staticmethod
    def _lbl(text: str, obj_name: str = "", fixed_w: int = 0) -> QLabel:
        l = QLabel(text)
        if obj_name: l.setObjectName(obj_name)
        if fixed_w:  l.setFixedWidth(fixed_w)
        return l

    def closeEvent(self, event):
        self._hard_reset()
        event.accept()


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