"""
Beam Stabilizer GUI
====================
Feedback-controlled liquid surface stabilization.

Left panel  — Thorlabs MLJ250 motorized lab jack control.
Right panel — Newport CONEX-PSD9 beam position sensor.

Feedback law: stage_delta = gain * ||beam - setpoint||  (proportional on 2D magnitude)
"""

import sys
import os
import json
import math
import time
import queue
from datetime import datetime
from collections import deque

import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QGroupBox, QLabel, QLineEdit, QPushButton, QDoubleSpinBox, QSpinBox,
    QComboBox, QFrame, QFormLayout, QSizePolicy, QMessageBox,
)
from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QFont

import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

# ── Local import ──────────────────────────────────────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from conex_psd9 import ConexPSD

# ── Thorlabs Kinesis (graceful fallback if SDK not installed) ─────────────────
KINESIS_PATH = "C:\\Program Files\\Thorlabs\\Kinesis\\"
_kinesis_ok = False
try:
    import clr  # pythonnet
    if hasattr(os, "add_dll_directory"):
        os.add_dll_directory(KINESIS_PATH)
    clr.AddReference(KINESIS_PATH + "Thorlabs.MotionControl.DeviceManagerCLI.dll")
    clr.AddReference(KINESIS_PATH + "Thorlabs.MotionControl.GenericMotorCLI.dll")
    clr.AddReference(KINESIS_PATH + "Thorlabs.MotionControl.IntegratedStepperMotorsCLI.dll")
    from Thorlabs.MotionControl.DeviceManagerCLI import DeviceManagerCLI
    from Thorlabs.MotionControl.IntegratedStepperMotorsCLI import LabJack
    from System import Decimal as NetDecimal
    _kinesis_ok = True
except Exception as _ke:
    print(f"[WARN] Thorlabs Kinesis not available: {_ke}")

# ── Constants ─────────────────────────────────────────────────────────────────
STAGE_MIN_MM   = 0.0
STAGE_MAX_MM   = 50.0
STAGE_POLL_S   = 0.25      # stage position poll interval (seconds)
FEEDBACK_HZ    = 0.10       # feedback loop rate
PSD_POLL_HZ    = 10.0      # beam sensor poll rate
PSD_AVG_N      = 144       # rolling-window length for the feedback position estimate
                           # (PSD_AVG_N / PSD_POLL_HZ = 14.4 s averaging window)
TRAIL_LEN      = 100       # number of beam positions shown as trail


# =============================================================================
# PSD Worker Thread
# =============================================================================
class PSDWorker(QThread):
    position_updated  = pyqtSignal(float, float, float)  # x_mm, y_mm, laser_pct (live, every poll)
    averaged_position = pyqtSignal(float, float, int)    # mean_x_mm, mean_y_mm, sample_count
    connected         = pyqtSignal()
    disconnected      = pyqtSignal()
    error_occurred    = pyqtSignal(str)

    def __init__(self, port: str, address: int = 1):
        super().__init__()
        self.port      = port
        self.address   = address
        self._stop     = False
        self._interval = 1.0 / PSD_POLL_HZ
        self.device: ConexPSD | None = None

        # Rolling window for the feedback position estimate. Owned solely by this
        # thread — only the scalar mean ever crosses the thread boundary (signal).
        self._avg_x: deque = deque(maxlen=PSD_AVG_N)
        self._avg_y: deque = deque(maxlen=PSD_AVG_N)

    def stop(self):
        self._stop = True

    def run(self):
        try:
            self.device = ConexPSD(self.port, self.address)
        except Exception as exc:
            self.error_occurred.emit(f"PSD connection failed: {exc}")
            return

        self.connected.emit()

        while not self._stop:
            t0 = time.monotonic()
            try:
                x, y, lp = self.device.get_position()
                self.position_updated.emit(x, y, lp)   # live readout + plot (unchanged path)

                # Feed the rolling window and publish the current mean. Each poll
                # is one 10 Hz sample, so no decimation is needed.
                self._avg_x.append(x)
                self._avg_y.append(y)
                n = len(self._avg_x)
                self.averaged_position.emit(sum(self._avg_x) / n,
                                            sum(self._avg_y) / n, n)
            except Exception as exc:
                self.error_occurred.emit(str(exc))
                break
            dt = time.monotonic() - t0
            rem = self._interval - dt
            if rem > 0:
                time.sleep(rem)

        if self.device:
            self.device.close()
        self.disconnected.emit()


# =============================================================================
# Stage Worker Thread
# =============================================================================
class StageWorker(QThread):
    connected        = pyqtSignal(dict)   # {'max_velocity', 'acceleration', 'homing_velocity'}
    position_updated = pyqtSignal(float)
    homed            = pyqtSignal()
    move_done        = pyqtSignal(float)
    feedback_status  = pyqtSignal(bool)
    error_occurred   = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._cmd_queue = queue.Queue()
        self._stop      = False
        self.device     = None

        # Shared state written atomically (Python GIL makes float write safe)
        self._pos             = 0.0
        self._feedback_active = False
        self._gain            = 0.0
        self._deadband        = 0.0
        self._sp_x            = 0.0
        self._sp_y            = 0.0
        self._psd_x           = 0.0
        self._psd_y           = 0.0
        self._psd_ready       = False   # True once the averaging window is full

    # ── Thread-safe command API (called from the GUI thread) ──────────────────
    def cmd_connect(self, serial_no: str):
        self._cmd_queue.put(("connect", serial_no))

    def cmd_home(self):
        self._cmd_queue.put(("home", None))

    def cmd_jog(self, delta_mm: float):
        self._cmd_queue.put(("jog", delta_mm))

    def cmd_move_to(self, target_mm: float):
        self._cmd_queue.put(("move_to", target_mm))

    def cmd_start_feedback(self, gain: float, deadband: float, sp_x: float, sp_y: float):
        self._cmd_queue.put(("start_fb", (gain, deadband, sp_x, sp_y)))

    def cmd_stop_feedback(self):
        self._cmd_queue.put(("stop_fb", None))

    def update_psd(self, x: float, y: float, ready: bool = False):
        """Forward the latest *averaged* PSD position. Called from the GUI thread.

        ``ready`` is False until the rolling window has filled (PSD_AVG_N
        samples); the feedback tick refuses to actuate until then.
        """
        self._psd_x = x
        self._psd_y = y
        self._psd_ready = ready

    def stop(self):
        self.device.StopPolling()
        self.device.Disconnect()
        self._stop = True

    # ── Run loop ──────────────────────────────────────────────────────────────
    def run(self):
        last_poll     = 0.0
        last_feedback = 0.0
        fb_interval   = 1.0 / FEEDBACK_HZ

        while not self._stop:
            now = time.monotonic()

            # Process one pending command per loop iteration
            try:
                cmd, arg = self._cmd_queue.get_nowait()
                self._execute(cmd, arg)
            except queue.Empty:
                pass

            # Position polling
            if self.device and (now - last_poll) >= STAGE_POLL_S:
                try:
                    self._pos = float(str(self.device.Position))
                    self.position_updated.emit(self._pos)
                except Exception as exc:
                    self.error_occurred.emit(f"Position poll error: {exc}")
                last_poll = now

            # Feedback tick
            if self._feedback_active and self.device:
                if (now - last_feedback) >= fb_interval:
                    self._feedback_tick()
                    last_feedback = now

            time.sleep(0.2)  # 5 Hz loop resolution

    def _execute(self, cmd: str, arg):
        dispatch = {
            "connect":  lambda a: self._do_connect(a),
            "home":     lambda a: self._do_home(),
            "jog":      lambda a: self._do_move_to(
                            max(STAGE_MIN_MM, min(STAGE_MAX_MM, self._pos + a))),
            "move_to":  lambda a: self._do_move_to(
                            max(STAGE_MIN_MM, min(STAGE_MAX_MM, a))),
            "start_fb": lambda a: self._start_fb(*a),
            "stop_fb":  lambda a: self._stop_fb(),
        }
        handler = dispatch.get(cmd)
        if handler:
            handler(arg)

    def _do_connect(self, serial_no: str):
        if not _kinesis_ok:
            self.error_occurred.emit("Thorlabs Kinesis SDK is not installed.")
            return
        try:
            DeviceManagerCLI.BuildDeviceList()
            self.device = LabJack.CreateLabJack(serial_no)
            self.device.Connect(serial_no)
            time.sleep(0.5)
            if not self.device.IsSettingsInitialized():
                self.device.WaitForSettingsInitialized(10000)
            self.device.LoadMotorConfiguration(serial_no)
            self.device.StartPolling(250)
            time.sleep(0.5)
            self.device.EnableDevice()
            time.sleep(0.5)

            vel  = self.device.GetVelocityParams()
            home = self.device.GetHomingParams()
            self._pos = float(str(self.device.Position))
            self.connected.emit({
                "max_velocity":    float(str(vel.MaxVelocity)),
                "acceleration":    float(str(vel.Acceleration)),
                "homing_velocity": float(str(home.Velocity)),
                "position":        self._pos,
                "is_homed":        _read_homed(self.device),
            })
        except Exception as exc:
            self.device = None
            self.error_occurred.emit(f"Stage connect failed: {exc}")

    def _do_home(self):
        if not self.device:
            return
        try:
            self.device.Home(60000)   # blocking, 60 s timeout
            self._pos = 0.0
            self.homed.emit()
        except Exception as exc:
            self.error_occurred.emit(f"Homing failed: {exc}")

    def _do_move_to(self, target_mm: float):
        if not self.device:
            return
        try:
            self.device.MoveTo(NetDecimal(target_mm), 30000)   # blocking, 30 s timeout
            self._pos = float(str(self.device.Position))
            self.move_done.emit(self._pos)
        except Exception as exc:
            self.error_occurred.emit(f"Move failed: {exc}")

    def _start_fb(self, gain: float, deadband: float, sp_x: float, sp_y: float):
        self._gain     = gain
        self._deadband = deadband
        self._sp_x     = sp_x
        self._sp_y     = sp_y
        self._feedback_active = True
        self.feedback_status.emit(True)

    def _stop_fb(self):
        self._feedback_active = False
        self.feedback_status.emit(False)

    def _feedback_tick(self):
        # Runs once per feedback interval whenever feedback is enabled, including
        # the warm-up phase before the averaging window fills. Actuation is gated
        # on the averaging window being full and the error exceeding the deadband.
        if not self._psd_ready:
            return                                 # warm-up: never actuate

        # Error is computed on the *averaged* (sliding-window) estimate, matching
        # the feedback law.
        error = math.sqrt((self._psd_x - self._sp_x) ** 2 +
                          (self._psd_y - self._sp_y) ** 2)
        if error <= self._deadband:
            return                                 # inside deadband: hold

        # Skip if the stage is still moving toward a previous target.
        try:
            if bool(self.device.Status.IsMoving):
                return
        except Exception:
            pass

        direction = math.copysign(1, self._psd_x - self._sp_x)
        delta  = self._gain * error * direction
        target = max(STAGE_MIN_MM, min(STAGE_MAX_MM, self._pos + delta))
        try:
            self.device.MoveTo(NetDecimal(target), 0)   # non-blocking
        except Exception:
            pass


# =============================================================================
# Beam Position Plot
# =============================================================================
class BeamPlotCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self._fig = Figure(figsize=(4, 4), facecolor="#1e1e1e")
        super().__init__(self._fig)
        self.setParent(parent)
        self._trail_x: deque = deque(maxlen=TRAIL_LEN)
        self._trail_y: deque = deque(maxlen=TRAIL_LEN)
        self._sp_x = 0.0
        self._sp_y = 0.0
        self._ax = self._fig.add_subplot(111)
        self._init_axes()

    def _init_axes(self):
        ax = self._ax
        ax.set_facecolor("#2b2b2b")
        ax.set_xlim(-5, 5)
        ax.set_ylim(-5, 5)
        ax.set_xlabel("X (mm)", color="#aaaaaa", fontsize=9)
        ax.set_ylabel("Y (mm)", color="#aaaaaa", fontsize=9)
        ax.tick_params(colors="#aaaaaa", labelsize=8)
        for spine in ax.spines.values():
            spine.set_color("#555555")
        ax.grid(True, color="#3a3a3a", linewidth=0.5)
        ax.axhline(0, color="#555555", linewidth=0.8, linestyle="--")
        ax.axvline(0, color="#555555", linewidth=0.8, linestyle="--")

        self._trail_line, = ax.plot([], [], "o", color="#4fc3f7",
                                    markersize=2, alpha=0.35, linestyle="none")
        self._cur_dot,    = ax.plot([], [], "o", color="#ef5350", markersize=8)
        self._sp_marker,  = ax.plot([0], [0], "+", color="#a5d6a7",
                                    markersize=14, markeredgewidth=2,
                                    label="Setpoint")
        ax.legend(fontsize=7, facecolor="#2b2b2b", labelcolor="#aaaaaa",
                  loc="upper right")
        self._fig.tight_layout(pad=1.2)
        self.draw()

    def update_position(self, x: float, y: float):
        self._trail_x.append(x)
        self._trail_y.append(y)
        self._trail_line.set_data(list(self._trail_x), list(self._trail_y))
        self._cur_dot.set_data([x], [y])

        # Keep axes centred on the larger of (data extent, 2 mm)
        extent = max(2.0,
                     abs(x) * 1.4, abs(y) * 1.4,
                     abs(self._sp_x) * 1.4, abs(self._sp_y) * 1.4)
        self._ax.set_xlim(-extent, extent)
        self._ax.set_ylim(-extent, extent)
        self.draw_idle()

    def set_setpoint(self, sp_x: float, sp_y: float):
        self._sp_x, self._sp_y = sp_x, sp_y
        self._sp_marker.set_data([sp_x], [sp_y])
        self.draw_idle()


# =============================================================================
# Live feedback trace (pop-up window shown while feedback is running)
# =============================================================================
class FeedbackPlotWindow(QWidget):
    """Live time-series of beam X (raw + sliding window) and stage position.

    Created when feedback is enabled and closed when it ends. Beam X (small mm
    values near the setpoint) shares the left axis; stage position (0–50 mm) uses
    a twin right axis so both stay readable. Data handlers just append; a QTimer
    redraws at a fixed rate, decoupled from the incoming sample rates."""

    def __init__(self, parent=None, meta=None):
        super().__init__(parent)
        # Run parameters (gain/deadband/setpoint), written to the saved header.
        self._meta    = meta or {}
        self._started = datetime.now()
        self.setWindowTitle("Feedback Live Trace — Beam X & Stage")
        self.resize(760, 480)

        self.fig    = Figure(figsize=(7.4, 4.6))
        self.canvas = FigureCanvas(self.fig)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.addWidget(self.canvas)

        self.ax  = self.fig.add_subplot(111)
        self.ax2 = self.ax.twinx()
        self.ax.set_xlabel("Logging time (s)")
        self.ax.set_ylabel("Relative height (µm)")
        self.ax2.set_ylabel("Stage position (mm)")
        self.ax.grid(True, alpha=0.3)

        (self._raw_line,)   = self.ax.plot([],  [], color="#90caf9", lw=1.0,
                                           label="Raw X")
        (self._win_line,)   = self.ax.plot([],  [], color="#1e88e5", lw=1.8,
                                           label="Sliding-window X")
        (self._stage_line,) = self.ax2.plot([], [], color="#e53935", lw=1.5,
                                            label="Stage position")

        lines = [self._raw_line, self._win_line, self._stage_line]
        self.ax.legend(lines, [ln.get_label() for ln in lines],
                       loc="upper left", fontsize=8)

        # Independent (time, value) series — each stream arrives at its own rate.
        self._t_raw,   self._v_raw   = [], []
        self._t_win,   self._v_win   = [], []
        self._t_stage, self._v_stage = [], []
        self._t0 = time.monotonic()

        self.fig.tight_layout()
        self.canvas.draw_idle()

        # Redraw at ~5 Hz regardless of sample arrival, to keep the UI light.
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._redraw)
        self._timer.start(200)

    # ── Data intake (called from the GUI thread) ──────────────────────────────
    def add_raw(self, x: float):
        self._t_raw.append(time.monotonic() - self._t0)
        self._v_raw.append(x/4.2)

    def add_window(self, x: float):
        self._t_win.append(time.monotonic() - self._t0)
        self._v_win.append(x/4.2)

    def add_stage(self, pos: float):
        self._t_stage.append(time.monotonic() - self._t0)
        self._v_stage.append(pos)

    # ── Rendering ─────────────────────────────────────────────────────────────
    def _redraw(self):
        self._raw_line.set_data(self._t_raw, self._v_raw)
        self._win_line.set_data(self._t_win, self._v_win)
        self._stage_line.set_data(self._t_stage, self._v_stage)
        if self._v_raw or self._v_win:
            self.ax.relim()
            self.ax.autoscale_view()
        if self._v_stage:
            self.ax2.relim()
            self.ax2.autoscale_view()
        self.canvas.draw_idle()

    # ── Persistence ───────────────────────────────────────────────────────────
    def save(self, directory):
        """Write the three plotted (time, value) series to a padded wide CSV.

        The streams arrive at independent rates, so each keeps its own
        timestamps and shorter streams are blank-padded. Returns the written
        path, or None if there is no data to save."""
        self._v_raw = self._v_raw[::5]
        self._v_win = self._v_win[::5]

        n = max(len(self._v_raw), len(self._v_win), len(self._v_stage))
        if n == 0:
            return None

        def cell(seq, i, fmt):
            return fmt.format(seq[i]) if i < len(seq) else ""

        fname = "feedback_trace_" + self._started.strftime("%Y%m%d_%H%M%S") + ".csv"
        path  = os.path.join(directory, fname)
        with open(path, "w") as f:
            f.write("# Feedback live-trace data\n")
            f.write(f"# started:     {self._started.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"# gain:        {self._meta.get('gain', 0.0):.6f} mm/mm\n")
            f.write(f"# deadband:    {self._meta.get('deadband', 0.0):.6f} mm\n")
            f.write(f"# setpoint_x:  {self._meta.get('sp_x', 0.0):.6f} mm\n")
            f.write(f"# setpoint_y:  {self._meta.get('sp_y', 0.0):.6f} mm\n")
            f.write(f"# feedback_hz: {FEEDBACK_HZ}\n")
            f.write("t_raw_s,raw_x_mm,t_win_s,win_x_mm,t_stage_s,stage_mm\n")
            for i in range(n):
                row = [
                    cell(self._t_raw,   i, "{:.3f}"), cell(self._v_raw,   i, "{:.6f}"),
                    cell(self._t_win,   i, "{:.3f}"), cell(self._v_win,   i, "{:.6f}"),
                    cell(self._t_stage, i, "{:.3f}"), cell(self._v_stage, i, "{:.4f}"),
                ]
                f.write(",".join(row) + "\n")
        return path

    def shutdown(self):
        """Stop the redraw timer and close the window."""
        self._timer.stop()
        self.close()


# =============================================================================
# Persistent stage state  (last position / reference height, keyed by serial)
# =============================================================================
STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "stage_state.json")

def _load_stage_state() -> dict:
    """Return the full state dict, or {} if missing/corrupt."""
    try:
        with open(STATE_FILE, "r") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}

def _save_stage_state(serial: str, **fields):
    """Merge ``fields`` into this serial's entry and write atomically.

    Uses temp-file + os.replace so a crash mid-write can never corrupt the
    existing state file.
    """
    if not serial:
        return
    data = _load_stage_state()
    entry = data.get(serial, {})
    entry.update(fields)
    entry["timestamp"] = datetime.now().isoformat(timespec="seconds")
    data[serial] = entry
    tmp = STATE_FILE + ".tmp"
    try:
        with open(tmp, "w") as f:
            json.dump(data, f, indent=2)
        os.replace(tmp, STATE_FILE)
    except OSError as exc:
        print(f"[WARN] Could not save stage state: {exc}")

def _read_homed(device) -> bool:
    """Read the controller's homed status. Conservative: any failure → False.

    Mirrors the existing ``device.Status.IsMoving`` usage in the feedback loop.
    The exact property name may vary by Kinesis version; the False fallback
    keeps motion locked until an explicit home if we can't read it.
    """
    try:
        return bool(device.Status.IsHomed)
    except Exception:
        return False


# =============================================================================
# UI helpers
# =============================================================================
def _dot(color: str = "#e53935") -> QLabel:
    lbl = QLabel("●")
    lbl.setStyleSheet(f"color: {color}; font-size: 14px;")
    return lbl

def _set_dot(lbl: QLabel, connected: bool):
    color = "#43a047" if connected else "#e53935"
    lbl.setStyleSheet(f"color: {color}; font-size: 14px;")

def _heading(text: str) -> QLabel:
    lbl = QLabel(text)
    f = lbl.font()
    f.setBold(True)
    f.setPointSize(11)
    lbl.setFont(f)
    return lbl

def _separator() -> QFrame:
    line = QFrame()
    line.setFrameShape(QFrame.HLine)
    line.setStyleSheet("color: #444444;")
    return line

# =============================================================================
# Main Window
# =============================================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Beam Stabilizer (version 1.1)")
        self.resize(1150, 760)

        self._psd_worker:   PSDWorker   | None = None
        self._stage_worker: StageWorker | None = None
        self._fb_plot:      FeedbackPlotWindow | None = None

        self._stage_connected = False
        self._stage_homed     = False
        self._psd_connected   = False
        self._feedback_active = False

        self._latest_psd     = (0.0, 0.0)   # latest raw (per-poll) PSD sample
        self._latest_psd_avg = (0.0, 0.0)   # latest sliding-window mean position
        self._setpoint    = (0.0, 0.0)
        self._ref_height  = 8.950 # Gold height in mm.
        self._last_stage_pos = 0.0

        self._build_ui()
        self._apply_style()
        self._update_ui_state()

    # ──────────────────────────────────────────────────────────────────────────
    # UI Construction
    # ──────────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        root_w = QWidget()
        self.setCentralWidget(root_w)
        root = QHBoxLayout(root_w)
        root.setSpacing(16)
        root.setContentsMargins(14, 14, 14, 14)

        root.addWidget(self._build_stage_panel(), stretch=1)

        div = QFrame()
        div.setFrameShape(QFrame.VLine)
        div.setStyleSheet("color: #484848;")
        root.addWidget(div)

        root.addWidget(self._build_psd_panel(), stretch=1)

        self.statusBar().showMessage("Ready.")

    # ── Stage panel (left) ────────────────────────────────────────────────────
    def _build_stage_panel(self) -> QWidget:
        panel = QWidget()
        vbox  = QVBoxLayout(panel)
        vbox.setSpacing(8)
        vbox.addWidget(_heading("MLJ250  Stage Control"))

        # Connection
        grp = QGroupBox("Connection")
        form = QFormLayout(grp)
        self.stage_serial_edit = QLineEdit()
        self.stage_serial_edit.setText("49535734")
        self.stage_connect_btn = QPushButton("Connect")
        self.stage_connect_btn.clicked.connect(self._on_stage_connect)
        self._stage_dot = _dot()
        self._stage_status_lbl = QLabel("Disconnected")
        row = QHBoxLayout()
        row.addWidget(self._stage_dot)
        row.addWidget(self._stage_status_lbl)
        row.addStretch()
        form.addRow("Serial No.:", self.stage_serial_edit)
        form.addRow("", self.stage_connect_btn)
        form.addRow("Status:", row)
        vbox.addWidget(grp)

        # Device info
        grp2 = QGroupBox("Device Info")
        form2 = QFormLayout(grp2)
        self.stage_pos_lbl  = QLabel("—")
        self.stage_vel_lbl  = QLabel("—")
        self.stage_acc_lbl  = QLabel("—")
        self.stage_hvel_lbl = QLabel("—")
        form2.addRow("Position:",       self.stage_pos_lbl)
        form2.addRow("Max Velocity:",   self.stage_vel_lbl)
        form2.addRow("Acceleration:",   self.stage_acc_lbl)
        form2.addRow("Home Velocity:",  self.stage_hvel_lbl)
        vbox.addWidget(grp2)

        # Reference height
        grp3 = QGroupBox("Reference Height")
        hl = QHBoxLayout(grp3)
        self.ref_height_lbl = QLabel("8.950 mm")
        self.move_ref_btn = QPushButton("Move to Reference")
        self.move_ref_btn.clicked.connect(self._on_move_to_ref)
        self.set_ref_btn = QPushButton("Set Current as Reference")
        self.set_ref_btn.clicked.connect(self._on_set_ref_height)
        hl.addWidget(QLabel("Stored:"))
        hl.addWidget(self.ref_height_lbl)
        hl.addStretch()
        hl.addWidget(self.move_ref_btn)
        hl.addStretch()
        hl.addWidget(self.set_ref_btn)
        vbox.addWidget(grp3)

        # Home
        self.home_btn = QPushButton("Home Stage")
        self.home_btn.clicked.connect(self._on_home)
        vbox.addWidget(self.home_btn)

        # Homed status indicator
        homed_row = QHBoxLayout()
        self._homed_dot = _dot()
        self._homed_status_lbl = QLabel("Not homed")
        homed_row.addWidget(QLabel("Homed:"))
        homed_row.addWidget(self._homed_dot)
        homed_row.addWidget(self._homed_status_lbl)
        homed_row.addStretch()
        vbox.addLayout(homed_row)

        vbox.addWidget(_separator())

        # Jog
        grp4 = QGroupBox("Jog Controls")
        vb4 = QVBoxLayout(grp4)
        jog_row = QHBoxLayout()
        jog_row.addWidget(QLabel("Jog size:"))
        self.jog_size_spin = QDoubleSpinBox()
        self.jog_size_spin.setRange(0.001, 1.0)
        self.jog_size_spin.setSingleStep(0.002)
        self.jog_size_spin.setValue(0.020)
        self.jog_size_spin.setDecimals(3)
        self.jog_size_spin.setSuffix("  mm")
        jog_row.addWidget(self.jog_size_spin)
        jog_row.addStretch()
        vb4.addLayout(jog_row)
        btn_row = QHBoxLayout()
        self.jog_up_btn   = QPushButton("▲   Jog Up")
        self.jog_down_btn = QPushButton("▼   Jog Down")
        self.jog_up_btn.clicked.connect(self._on_jog_up)
        self.jog_down_btn.clicked.connect(self._on_jog_down)
        btn_row.addWidget(self.jog_up_btn)
        btn_row.addWidget(self.jog_down_btn)
        vb4.addLayout(btn_row)
        vbox.addWidget(grp4)

        # Move to position
        grp5 = QGroupBox("Move to Position  (0 – 50 mm)")
        hl5 = QHBoxLayout(grp5)
        hl5.addWidget(QLabel("Target:"))
        self.move_target_spin = QDoubleSpinBox()
        self.move_target_spin.setRange(STAGE_MIN_MM, STAGE_MAX_MM)
        self.move_target_spin.setValue(0.0)
        self.move_target_spin.setDecimals(3)
        self.move_target_spin.setSuffix("  mm")
        self.move_btn = QPushButton("Move")
        self.move_btn.clicked.connect(self._on_move_to)
        hl5.addWidget(self.move_target_spin)
        hl5.addWidget(self.move_btn)
        vbox.addWidget(grp5)

        vbox.addWidget(_separator())

        # Feedback
        grp6 = QGroupBox("Feedback Loop")
        vb6 = QVBoxLayout(grp6)
        fb_form = QFormLayout()
        self.gain_spin = QDoubleSpinBox()
        self.gain_spin.setRange(-9.99, 9.99)
        self.gain_spin.setSingleStep(0.01)
        self.gain_spin.setValue(0.2)
        self.gain_spin.setDecimals(2)
        self.gain_spin.setSuffix("  mm/mm")
        self.deadband_spin = QDoubleSpinBox()
        self.deadband_spin.setRange(0.0, 1.0)
        self.deadband_spin.setSingleStep(0.01)
        self.deadband_spin.setValue(0.02)
        self.deadband_spin.setDecimals(3)
        self.deadband_spin.setSuffix("  mm")
        fb_form.addRow("Gain:", self.gain_spin)
        fb_form.addRow("Deadband:", self.deadband_spin)
        vb6.addLayout(fb_form)
        fb_stat_row = QHBoxLayout()
        self._fb_dot = _dot()
        self._fb_status_lbl = QLabel("Inactive")
        fb_stat_row.addWidget(self._fb_dot)
        fb_stat_row.addWidget(self._fb_status_lbl)
        fb_stat_row.addStretch()
        vb6.addLayout(fb_stat_row)
        self.feedback_btn = QPushButton("Enable Feedback")
        self.feedback_btn.clicked.connect(self._on_toggle_feedback)
        vb6.addWidget(self.feedback_btn)
        vbox.addWidget(grp6)

        vbox.addStretch()

        # Register controls for lock management.
        #  - connected group: parameter entry, safe on an un-homed axis.
        #  - homed group: anything that commands motion or captures a position
        #    that only means something once the stage has a valid datum.
        # (home_btn is handled separately so it stays reachable when un-homed.)
        self._stage_connected_controls = [
            self.jog_size_spin, self.move_target_spin,
            self.gain_spin, self.deadband_spin,
        ]
        self._stage_homed_controls = [
            self.jog_up_btn, self.jog_down_btn, self.move_btn, 
            self.set_ref_btn, self.move_ref_btn,
        ]

        return panel

    # ── PSD panel (right) ─────────────────────────────────────────────────────
    def _build_psd_panel(self) -> QWidget:
        panel = QWidget()
        vbox  = QVBoxLayout(panel)
        vbox.setSpacing(8)
        vbox.addWidget(_heading("CONEX-PSD9  Beam Sensor"))

        # Connection
        grp = QGroupBox("Connection")
        form = QFormLayout(grp)
        port_row = QHBoxLayout()
        self.psd_port_combo = QComboBox()
        self.psd_port_combo.setEditable(True)
        self._refresh_ports()
        self.psd_refresh_btn = QPushButton("↺")
        self.psd_refresh_btn.setFixedWidth(30)
        self.psd_refresh_btn.clicked.connect(self._refresh_ports)
        port_row.addWidget(self.psd_port_combo, stretch=1)
        port_row.addWidget(self.psd_refresh_btn)
        self.psd_addr_spin = QSpinBox()
        self.psd_addr_spin.setRange(1, 31)
        self.psd_addr_spin.setValue(1)
        self.psd_connect_btn = QPushButton("Connect")
        self.psd_connect_btn.clicked.connect(self._on_psd_connect)
        self._psd_dot = _dot()
        self._psd_status_lbl = QLabel("Disconnected")
        psd_stat_row = QHBoxLayout()
        psd_stat_row.addWidget(self._psd_dot)
        psd_stat_row.addWidget(self._psd_status_lbl)
        psd_stat_row.addStretch()
        form.addRow("Port:", port_row)
        form.addRow("Address:", self.psd_addr_spin)
        form.addRow("", self.psd_connect_btn)
        form.addRow("Status:", psd_stat_row)
        vbox.addWidget(grp)

        # Beam plot
        grp2 = QGroupBox("Beam Position (live)")
        vb2 = QVBoxLayout(grp2)
        self.beam_canvas = BeamPlotCanvas()
        self.beam_canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        vb2.addWidget(self.beam_canvas)
        vbox.addWidget(grp2, stretch=1)

        # Readouts
        grp3 = QGroupBox("Readouts")
        form3 = QFormLayout(grp3)
        self.psd_x_lbl   = QLabel("—")
        self.psd_y_lbl   = QLabel("—")
        self.psd_mag_lbl = QLabel("—")
        self.psd_pwr_lbl = QLabel("—")
        self.sp_x_lbl    = QLabel("0.0000 mm")
        self.sp_y_lbl    = QLabel("0.0000 mm")
        form3.addRow("X position:",  self.psd_x_lbl)
        form3.addRow("Y position:",  self.psd_y_lbl)
        form3.addRow("Magnitude:",   self.psd_mag_lbl)
        form3.addRow("Laser power:", self.psd_pwr_lbl)
        form3.addRow("Setpoint X:",  self.sp_x_lbl)
        form3.addRow("Setpoint Y:",  self.sp_y_lbl)
        vbox.addWidget(grp3)

        # Setpoint button
        self.set_sp_btn = QPushButton("Set Current Position as Setpoint")
        self.set_sp_btn.clicked.connect(self._on_set_setpoint)
        vbox.addWidget(self.set_sp_btn)

        # Register PSD-side controls for lock management.
        # Port selection / refresh / address must be usable *before* connecting,
        # so they are enabled while DISCONNECTED. The setpoint button needs a
        # live beam reading, so it is enabled only while CONNECTED.
        self._psd_predisconnect_controls = [
            self.psd_port_combo, self.psd_addr_spin, self.psd_refresh_btn,
        ]
        self._psd_controls = [
            self.set_sp_btn,
        ]

        return panel

    # ──────────────────────────────────────────────────────────────────────────
    # Stylesheet
    # ──────────────────────────────────────────────────────────────────────────
    def _apply_style(self):
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #2b2b2b;
                color: #dcdcdc;
                font-size: 10pt;
            }
            QGroupBox {
                border: 1px solid #4a4a4a;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 6px;
                font-weight: bold;
                color: #aaaaaa;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 4px;
            }
            QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox {
                background-color: #3c3f41;
                border: 1px solid #555555;
                border-radius: 3px;
                padding: 3px 5px;
                color: #dcdcdc;
            }
            QPushButton {
                background-color: #3c3f41;
                border: 1px solid #555555;
                border-radius: 3px;
                padding: 5px 12px;
                color: #dcdcdc;
            }
            QPushButton:hover   { background-color: #4e5254; }
            QPushButton:pressed { background-color: #252525; }
            QPushButton:disabled { color: #666666; background-color: #2d2d2d; border-color: #3a3a3a; }
            QLabel { color: #dcdcdc; }
            QStatusBar { color: #aaaaaa; font-size: 9pt; }
        """)
        # Colour specific buttons
        self.stage_connect_btn.setStyleSheet(
            "background-color: #1565c0; color: white; padding: 5px 12px;")
        self.psd_connect_btn.setStyleSheet(
            "background-color: #1565c0; color: white; padding: 5px 12px;")
        self.home_btn.setStyleSheet(
            "background-color: #e65100; color: white; padding: 5px 12px;")
        self.feedback_btn.setStyleSheet(
            "background-color: #2e7d32; color: white; padding: 6px 12px; font-weight: bold;")

    # ──────────────────────────────────────────────────────────────────────────
    # Centralised UI-state manager
    # ──────────────────────────────────────────────────────────────────────────
    def _update_ui_state(self):
        locked = self._feedback_active

        # Connect buttons: always reachable unless feedback is running
        self.stage_connect_btn.setEnabled(not locked)
        self.psd_connect_btn.setEnabled(not locked)

        # Home button: reachable whenever connected (this is how you get homed)
        self.home_btn.setEnabled(self._stage_connected and not locked)

        # Parameter-entry controls: need stage connected AND not locked
        for w in self._stage_connected_controls:
            w.setEnabled(self._stage_connected and not locked)

        # Motion controls: additionally require the stage to be homed
        for w in self._stage_homed_controls:
            w.setEnabled(self._stage_connected and self._stage_homed and not locked)

        # PSD port/refresh/address: usable only while DISCONNECTED (and unlocked),
        # i.e. when you are choosing what to connect to.
        for w in self._psd_predisconnect_controls:
            w.setEnabled(not self._psd_connected and not locked)

        # PSD controls (setpoint): need PSD connected AND not locked
        for w in self._psd_controls:
            w.setEnabled(self._psd_connected and not locked)

        # Feedback button
        if locked:
            self.feedback_btn.setText("Disable Feedback")
            self.feedback_btn.setStyleSheet(
                "background-color: #b71c1c; color: white; padding: 6px 12px; font-weight: bold;")
            self.feedback_btn.setEnabled(True)
        else:
            self.feedback_btn.setText("Enable Feedback")
            self.feedback_btn.setStyleSheet(
                "background-color: #2e7d32; color: white; padding: 6px 12px; font-weight: bold;")
            both_connected = self._stage_connected and self._psd_connected
            self.feedback_btn.setEnabled(both_connected and self._stage_homed)

    def _set_homed(self, homed: bool):
        """Update the homed flag, its indicator dot/label, and dependent UI."""
        self._stage_homed = homed
        _set_dot(self._homed_dot, homed)
        self._homed_status_lbl.setText("Homed" if homed else "Not homed")
        self._update_ui_state()

    # ──────────────────────────────────────────────────────────────────────────
    # Port helpers
    # ──────────────────────────────────────────────────────────────────────────
    def _refresh_ports(self):
        self.psd_port_combo.clear()
        for p in serial.tools.list_ports.comports():
            self.psd_port_combo.addItem(p.device)

    # ──────────────────────────────────────────────────────────────────────────
    # Stage slots
    # ──────────────────────────────────────────────────────────────────────────
    def _on_stage_connect(self):
        serial_no = self.stage_serial_edit.text().strip()
        if not serial_no:
            QMessageBox.warning(self, "Input required",
                                "Enter the MLJ250 serial number.")
            return
        self.stage_connect_btn.setEnabled(False)
        self._stage_status_lbl.setText("Connecting…")

        if self._stage_worker:
            self._stage_worker.stop()
            self._stage_worker.wait(3000)

        w = StageWorker()
        w.connected.connect(self._on_stage_connected)
        w.position_updated.connect(self._on_stage_position)
        w.homed.connect(self._on_stage_homed)
        w.move_done.connect(self._on_move_done)
        w.error_occurred.connect(self._on_stage_error)
        w.feedback_status.connect(self._on_feedback_status)
        self._stage_worker = w
        w.start()
        w.cmd_connect(serial_no)

    @pyqtSlot(dict)
    def _on_stage_connected(self, info: dict):
        self._stage_connected = True
        _set_dot(self._stage_dot, True)
        self._stage_status_lbl.setText("Connected")
        self.stage_connect_btn.setText("Disconnect")
        self.stage_connect_btn.setEnabled(True)
        self.stage_connect_btn.clicked.disconnect()
        self.stage_connect_btn.clicked.connect(self._on_stage_disconnect)

        self.stage_pos_lbl.setText(f"{info['position']:.4f} mm")
        self.stage_vel_lbl.setText(f"{info['max_velocity']:.3f} mm/s")
        self.stage_acc_lbl.setText(f"{info['acceleration']:.3f} mm/s²")
        self.stage_hvel_lbl.setText(f"{info['homing_velocity']:.3f} mm/s")
        self._last_stage_pos = info["position"]

        # Homed status comes from the controller (authoritative), not the file.
        self._set_homed(info.get("is_homed", False))   # also calls _update_ui_state

        # Restore remembered software state (reference height) and compare the
        # freshly-read position against what we last saved.
        saved = _load_stage_state().get(self.stage_serial_edit.text().strip(), {})
        ref = saved.get("reference_height_mm")
        if ref is not None:
            self._ref_height = ref
            self.ref_height_lbl.setText(f"{ref:.4f} mm")
        last = saved.get("last_position_mm")
        if (last is not None and not self._stage_homed
                and abs(last - info["position"]) > 0.01):
            self.statusBar().showMessage(
                f"Stage reads {info['position']:.4f} mm but last session ended at "
                f"{last:.4f} mm — home recommended before moving.", 8000)

    def _on_stage_disconnect(self):
        if self._stage_worker:
            self._stage_worker.stop()
            self._stage_worker.wait(3000)
            self._stage_worker = None
        self._stage_connected = False
        _set_dot(self._stage_dot, False)
        self._stage_status_lbl.setText("Disconnected")
        self.stage_connect_btn.setText("Connect")
        self.stage_connect_btn.clicked.disconnect()
        self.stage_connect_btn.clicked.connect(self._on_stage_connect)
        for lbl in (self.stage_pos_lbl, self.stage_vel_lbl,
                    self.stage_acc_lbl, self.stage_hvel_lbl):
            lbl.setText("—")
        self._set_homed(False)   # also calls _update_ui_state

    @pyqtSlot(float)
    def _on_stage_position(self, pos: float):
        self._last_stage_pos = pos
        self.stage_pos_lbl.setText(f"{pos:.4f} mm")
        if self._fb_plot is not None:
            self._fb_plot.add_stage(pos)

    @pyqtSlot()
    def _on_stage_homed(self):
        self.stage_pos_lbl.setText("0.0000 mm")
        self._last_stage_pos = 0.0
        self._set_homed(True)   # unlocks motion controls, updates indicator
        _save_stage_state(self.stage_serial_edit.text().strip(),
                          last_position_mm=0.0, homed=True)
        self.statusBar().showMessage("Stage homed successfully.", 4000)

    @pyqtSlot(float)
    def _on_move_done(self, pos: float):
        self._last_stage_pos = pos
        self.stage_pos_lbl.setText(f"{pos:.4f} mm")
        _save_stage_state(self.stage_serial_edit.text().strip(),
                          last_position_mm=pos)
        self.statusBar().showMessage(f"Move complete → {pos:.4f} mm", 3000)

    @pyqtSlot(str)
    def _on_stage_error(self, msg: str):
        QMessageBox.critical(self, "Stage Error", msg)
        self.stage_connect_btn.setEnabled(True)

    def _on_home(self):
        if self._stage_worker:
            self._stage_worker.cmd_home()
            self.statusBar().showMessage("Homing stage… (up to 60 s)")

    def _on_jog_up(self):
        if self._stage_worker:
            self._stage_worker.cmd_jog(+self.jog_size_spin.value())

    def _on_jog_down(self):
        if self._stage_worker:
            self._stage_worker.cmd_jog(-self.jog_size_spin.value())

    def _on_move_to(self):
        if self._stage_worker:
            self._stage_worker.cmd_move_to(self.move_target_spin.value())

    def _on_set_ref_height(self):
        pos = self._last_stage_pos
        self._ref_height = pos
        self.ref_height_lbl.setText(f"{pos:.4f} mm")
        _save_stage_state(self.stage_serial_edit.text().strip(),
                          reference_height_mm=pos)
        self.statusBar().showMessage(f"Reference height stored: {pos:.4f} mm", 3000)

    def _on_move_to_ref(self):
        if self._stage_worker:
            self._stage_worker.cmd_move_to(self._ref_height)

    # ── Feedback ──────────────────────────────────────────────────────────────
    def _on_toggle_feedback(self):
        if not self._feedback_active:
            self._start_feedback()
        else:
            if self._stage_worker:
                self._stage_worker.cmd_stop_feedback()

    def _start_feedback(self):
        if not (self._stage_connected and self._psd_connected):
            QMessageBox.warning(self, "Not ready",
                                "Connect both the stage and the PSD sensor first.")
            return
        gain     = self.gain_spin.value()
        deadband = self.deadband_spin.value()
        sp_x, sp_y = self._setpoint
        if self._stage_worker:
            self._stage_worker.cmd_start_feedback(gain, deadband, sp_x, sp_y)

    @pyqtSlot(bool)
    def _on_feedback_status(self, active: bool):
        self._feedback_active = active
        _set_dot(self._fb_dot, active)
        self._fb_status_lbl.setText("Active" if active else "Inactive")
        self._update_ui_state()

        # Live trace window: pops up when feedback starts, saves and closes when
        # it ends.
        if active:
            if self._fb_plot is None:
                meta = {
                    "gain":     self.gain_spin.value(),
                    "deadband": self.deadband_spin.value(),
                    "sp_x":     self._setpoint[0],
                    "sp_y":     self._setpoint[1],
                }
                self._fb_plot = FeedbackPlotWindow(self, meta=meta)
            self._fb_plot.show()
            self._fb_plot.raise_()
        elif self._fb_plot is not None:
            try:
                self._fb_plot.save(os.path.dirname(os.path.abspath(__file__)))
            except Exception as exc:
                QMessageBox.warning(self, "Save failed",
                                    f"Could not save feedback trace:\n{exc}")
            self._fb_plot.shutdown()
            self._fb_plot = None

    # ──────────────────────────────────────────────────────────────────────────
    # PSD slots
    # ──────────────────────────────────────────────────────────────────────────
    def _on_psd_connect(self):
        port = self.psd_port_combo.currentText().strip()
        addr = self.psd_addr_spin.value()
        if not port:
            QMessageBox.warning(self, "Input required", "Select a COM port.")
            return
        self.psd_connect_btn.setEnabled(False)
        self._psd_status_lbl.setText("Connecting…")

        if self._psd_worker:
            self._psd_worker.stop()
            self._psd_worker.wait(3000)

        w = PSDWorker(port, addr)
        w.position_updated.connect(self._on_psd_update)
        w.averaged_position.connect(self._on_psd_averaged)
        w.connected.connect(self._on_psd_connected)
        w.disconnected.connect(self._on_psd_disconnected)
        w.error_occurred.connect(self._on_psd_error)
        self._psd_worker = w
        w.start()

    @pyqtSlot()
    def _on_psd_connected(self):
        self._psd_connected = True
        _set_dot(self._psd_dot, True)
        self._psd_status_lbl.setText("Connected")
        self.psd_connect_btn.setText("Disconnect")
        self.psd_connect_btn.setEnabled(True)
        self.psd_connect_btn.clicked.disconnect()
        self.psd_connect_btn.clicked.connect(self._on_psd_disconnect)
        self._update_ui_state()

    def _on_psd_disconnect(self):
        if self._psd_worker:
            self._psd_worker.stop()
        # _on_psd_disconnected handles the rest via signal

    @pyqtSlot()
    def _on_psd_disconnected(self):
        # If feedback was active, stop it safely
        if self._feedback_active and self._stage_worker:
            self._stage_worker.cmd_stop_feedback()

        self._psd_connected = False
        _set_dot(self._psd_dot, False)
        self._psd_status_lbl.setText("Disconnected")
        self.psd_connect_btn.setText("Connect")
        self.psd_connect_btn.clicked.disconnect()
        self.psd_connect_btn.clicked.connect(self._on_psd_connect)
        self.psd_connect_btn.setEnabled(True)
        for lbl in (self.psd_x_lbl, self.psd_y_lbl,
                    self.psd_mag_lbl, self.psd_pwr_lbl):
            lbl.setText("—")
        self._update_ui_state()

    @pyqtSlot(float, float, float)
    def _on_psd_update(self, x: float, y: float, lp: float):
        self._latest_psd = (x, y)
        if self._fb_plot is not None:
            self._fb_plot.add_raw(x - self._setpoint[0])
        sp_x, sp_y = self._setpoint
        mag = math.sqrt((x - sp_x) ** 2 + (y - sp_y) ** 2)

        self.psd_x_lbl.setText(f"{x:.4f} mm")
        self.psd_y_lbl.setText(f"{y:.4f} mm")
        self.psd_mag_lbl.setText(f"{mag:.4f} mm")
        self.psd_pwr_lbl.setText(f"{lp:.1f} %")

        self.beam_canvas.update_position(x, y)

    @pyqtSlot(float, float, int)
    def _on_psd_averaged(self, mean_x: float, mean_y: float, count: int):
        """Feed the feedback loop with the rolling-mean position.

        The plot and live readouts above keep using the raw per-poll value; only
        the stage feedback consumes this averaged estimate. ``count`` reaches
        PSD_AVG_N once the window is full, which arms the feedback tick.
        """
        self._latest_psd_avg = (mean_x, mean_y)
        if self._stage_worker:
            self._stage_worker.update_psd(mean_x, mean_y, count >= PSD_AVG_N)
        if self._fb_plot is not None:
            self._fb_plot.add_window(mean_x - self._setpoint[0])

    @pyqtSlot(str)
    def _on_psd_error(self, msg: str):
        QMessageBox.critical(self, "PSD Sensor Error", msg)
        self.psd_connect_btn.setEnabled(True)

    def _on_set_setpoint(self):
        # Use the latest sliding-window mean (the same estimator the feedback
        # error is computed against) rather than the noisy instantaneous sample.
        x, y = self._latest_psd_avg
        self._setpoint = (x, y)
        self.sp_x_lbl.setText(f"{x:.4f} mm")
        self.sp_y_lbl.setText(f"{y:.4f} mm")
        self.beam_canvas.set_setpoint(x, y)
        self.statusBar().showMessage(
            f"Setpoint set to ({x:.4f},  {y:.4f}) mm  (window mean)", 4000)

    # ──────────────────────────────────────────────────────────────────────────
    # Cleanup
    # ──────────────────────────────────────────────────────────────────────────
    def closeEvent(self, event):
        # Persist the latest known position on a clean exit. Feedback moves are
        # non-blocking and emit no move_done, so this is where that drift is
        # captured; _last_stage_pos is kept current by the poll signal.
        if self._stage_connected:
            _save_stage_state(self.stage_serial_edit.text().strip(),
                              last_position_mm=self._last_stage_pos,
                              homed=self._stage_homed)
        if self._fb_plot is not None:
            self._fb_plot.shutdown()
            self._fb_plot = None
        if self._psd_worker:
            self._psd_worker.stop()
            self._psd_worker.wait(5000)
        if self._stage_worker:
            self._stage_worker.stop()
            self._stage_worker.wait(5000)
        event.accept()


# =============================================================================
def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
