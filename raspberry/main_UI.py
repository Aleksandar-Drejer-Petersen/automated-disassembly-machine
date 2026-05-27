"""
main_UI.py — Full visual control panel for the automated disassembly machine.

Runs main.py's workflow in a background thread while showing:
  • Live laser distance chart  (intercepted from L: telemetry stream)
  • Live force chart           (intercepted from F: telemetry stream)
  • Vision image viewer        (auto-refreshes whenever a new image is saved)
  • Operation log              (all print() output, colour-coded)
  • Status panel               (phase, op counter, position)
  • Manual control buttons     (homing, grab/place, cam positions, single ops)

Run with:
    python main_UI.py
    python main_UI.py --no-execute     # detect + plan only, skip execution
"""

import sys
import os
import threading
import queue
import time
import argparse
from collections import deque

# ── Tkinter ───────────────────────────────────────────────────────────────────
import tkinter as tk
from tkinter import scrolledtext

# ── Matplotlib embedded in Tk ─────────────────────────────────────────────────
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ── PIL for image display (optional but recommended) ──────────────────────────
try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False

# Ensure the raspberry/ folder is on sys.path so imports of main, config, etc. work
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

# ─────────────────────────────────────────────────────────────────────────────
# Shared state — all cross-thread data lives here
# ─────────────────────────────────────────────────────────────────────────────
_ui_queue   = queue.Queue()          # (event_type, payload) → main thread
_state_lock = threading.Lock()

_machine_state = {
    "phase":    "IDLE",
    "op_index": 0,
    "op_total": 0,
    "op_ok":    0,
    "op_fail":  0,
    "pos_x":    0.0,
    "pos_y":    0.0,
    "pos_z":    0.0,
}

_ser_proxy    = None     # set once serial opens
_main_running = False    # True while workflow thread is alive


def _post_state(**kwargs):
    with _state_lock:
        _machine_state.update(kwargs)
    _ui_queue.put(("state", dict(_machine_state)))


# ─────────────────────────────────────────────────────────────────────────────
# Serial proxy — wraps pyserial, intercepts L:/F: telemetry for live charts
# ─────────────────────────────────────────────────────────────────────────────
class _SerialProxy:
    """Wraps a real serial.Serial instance.
    Reads pass through transparently; L: and F: lines are also posted to the
    UI queue so the charts update in real time without touching serial_comm.py.
    """

    def __init__(self, real_ser):
        self._ser = real_ser

    # serial_comm.py always calls readline() to receive data
    def readline(self):
        raw = self._ser.readline()
        if raw:
            try:
                line = raw.decode(errors="replace").strip()
            except Exception:
                line = ""
            if line:
                self._dispatch(line)
        return raw

    def _dispatch(self, line):
        # Laser telemetry: L:<ms>:<mm>
        if line.startswith("L:") and line.count(":") == 2:
            try:
                parts = line.split(":")
                _ui_queue.put(("laser", (int(parts[1]), float(parts[2]))))
            except ValueError:
                pass
            return

        # Force telemetry: F:<ms>:<N>:<z_mm>
        if line.startswith("F:") and line.count(":") == 3:
            try:
                parts = line.split(":")
                _ui_queue.put(("force", (int(parts[1]), float(parts[2]))))
            except ValueError:
                pass
            return

        # All other lines: post for state-machine parsing
        # (the print is already captured by stdout redirect → log)
        _ui_queue.put(("ard_msg", line))

    # Pass-through for write and every other attribute
    def write(self, data):
        return self._ser.write(data)

    def read(self, n=1):
        return self._ser.read(n)

    def __getattr__(self, name):
        return getattr(self._ser, name)


# ─────────────────────────────────────────────────────────────────────────────
# Monkey-patch serial_comm.open_serial to inject the proxy
# ─────────────────────────────────────────────────────────────────────────────
def _patch_serial_comm():
    import serial_comm as sc
    _orig = sc.open_serial

    def _patched():
        global _ser_proxy
        real = _orig()
        _ser_proxy = _SerialProxy(real)
        _ui_queue.put(("log", "[UI] Serial opened — proxy active.\n"))
        return _ser_proxy

    sc.open_serial = _patched


# ─────────────────────────────────────────────────────────────────────────────
# Stdout redirect — every print() goes to both terminal and the UI log
# ─────────────────────────────────────────────────────────────────────────────
class _LogRedirect:
    def __init__(self, original, q):
        self._orig = original
        self._q    = q

    def write(self, text):
        if text:
            self._q.put(("log", text))
        self._orig.write(text)

    def flush(self):
        self._orig.flush()

    def isatty(self):
        return False


# ─────────────────────────────────────────────────────────────────────────────
# Workflow thread — runs main.main() without blocking the UI
# ─────────────────────────────────────────────────────────────────────────────
def _run_main_thread(args_ns):
    global _main_running
    _main_running = True
    _post_state(phase="RUNNING")
    try:
        import main as main_mod
        # Temporarily replace sys.argv so main()'s argparse sees the right flags
        orig_argv = sys.argv[:]
        sys.argv  = [sys.argv[0]]
        if args_ns.no_execute:
            sys.argv.append("--no-execute")
        main_mod.main()
        sys.argv = orig_argv
    except SystemExit:
        pass
    except Exception as exc:
        _ui_queue.put(("log", f"\n[UI ERROR] {type(exc).__name__}: {exc}\n"))
    finally:
        _main_running = False
        _post_state(phase="DONE")
        _ui_queue.put(("workflow_done", None))


# ─────────────────────────────────────────────────────────────────────────────
# Manual command helper — safe to call from any thread
# ─────────────────────────────────────────────────────────────────────────────
def _manual_send(cmd):
    """Send an arbitrary command to the Arduino via the open serial proxy."""
    if _ser_proxy is None:
        _ui_queue.put(("log", "[UI] No serial connection — start the main script first.\n"))
        return
    try:
        import serial_comm as sc
        sc.send_command(_ser_proxy, cmd)
    except Exception as exc:
        _ui_queue.put(("log", f"[UI] Send error: {exc}\n"))


# ─────────────────────────────────────────────────────────────────────────────
# UI class
# ─────────────────────────────────────────────────────────────────────────────
class MachineUI:
    # Rolling chart buffer length (samples shown)
    CHART_LEN = 400

    # ── Dark theme colours ────────────────────────────────────────────────────
    BG      = "#12141a"   # window background
    PANEL   = "#1c1f2b"   # side panels / frames
    ACCENT  = "#1e2a45"   # section headers
    LINE    = "#2a3350"   # dividers
    GREEN   = "#00d4aa"
    RED     = "#e94560"
    ORANGE  = "#f5a623"
    BLUE    = "#4fc3f7"
    PINK    = "#ef9a9a"
    TEXT    = "#dde1ec"
    DIM     = "#6b7199"
    BTN     = "#1e3a5f"
    BTN_HOV = "#265073"
    LOG_BG  = "#0d0f14"
    LOG_FG  = "#b0c4de"

    def __init__(self, root, args_ns):
        self.root    = root
        self.args_ns = args_ns

        # Rolling chart data (main-thread only — filled from _poll_queue)
        self._laser_t  = deque(maxlen=self.CHART_LEN)
        self._laser_mm = deque(maxlen=self.CHART_LEN)
        self._force_t  = deque(maxlen=self.CHART_LEN)
        self._force_N  = deque(maxlen=self.CHART_LEN)

        # Image viewer state
        self._last_img_path = None
        self._img_photoref  = None   # must be kept alive (garbage-collection guard)

        root.title("Automated Disassembly Machine — Control Panel")
        root.configure(bg=self.BG)
        root.minsize(1280, 780)

        self._build_ui()
        self._poll_queue()
        self._poll_images()

    # ──────────────────────────────────────────────────────────────────────────
    # Layout builders
    # ──────────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        root = self.root

        # ── Top header bar ────────────────────────────────────────────────────
        hdr = tk.Frame(root, bg=self.ACCENT, height=44)
        hdr.pack(fill=tk.X, side=tk.TOP)
        hdr.pack_propagate(False)

        tk.Label(
            hdr,
            text="  ⚙  AUTOMATED DISASSEMBLY MACHINE — CONTROL PANEL",
            bg=self.ACCENT, fg=self.GREEN,
            font=("Consolas", 13, "bold"),
        ).pack(side=tk.LEFT, pady=10)

        self._status_lbl = tk.Label(
            hdr, text="● IDLE",
            bg=self.ACCENT, fg=self.DIM,
            font=("Consolas", 11, "bold"),
        )
        self._status_lbl.pack(side=tk.RIGHT, padx=20)

        # ── Main body: left controls  |  right content ────────────────────────
        body = tk.Frame(root, bg=self.BG)
        body.pack(fill=tk.BOTH, expand=True)

        # Left control panel (fixed width)
        left = tk.Frame(body, bg=self.PANEL, width=230)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(6, 3), pady=6)
        left.pack_propagate(False)

        # Right area (expands)
        right = tk.Frame(body, bg=self.BG)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(3, 6), pady=6)

        self._build_left_panel(left)
        self._build_right_area(right)

        # ── Bottom log ────────────────────────────────────────────────────────
        log_outer = tk.Frame(root, bg=self.PANEL, bd=0)
        log_outer.pack(fill=tk.X, side=tk.BOTTOM, padx=6, pady=(0, 6))

        log_hdr = tk.Frame(log_outer, bg=self.ACCENT, height=24)
        log_hdr.pack(fill=tk.X)
        log_hdr.pack_propagate(False)

        tk.Label(
            log_hdr, text="  OPERATION LOG",
            bg=self.ACCENT, fg=self.GREEN,
            font=("Consolas", 9, "bold"),
        ).pack(side=tk.LEFT, pady=3)

        tk.Button(
            log_hdr, text="CLEAR  ✕",
            command=self._clear_log,
            bg=self.BTN, fg=self.TEXT,
            relief=tk.FLAT, font=("Consolas", 8),
            cursor="hand2", padx=6,
        ).pack(side=tk.RIGHT, padx=4, pady=2)

        self._log = scrolledtext.ScrolledText(
            log_outer, height=9,
            bg=self.LOG_BG, fg=self.LOG_FG,
            font=("Consolas", 8),
            state=tk.DISABLED, wrap=tk.WORD,
            insertbackground=self.TEXT,
            selectbackground=self.ACCENT,
        )
        self._log.pack(fill=tk.X)
        # Tag colours for log highlighting
        self._log.tag_config("ok",    foreground=self.GREEN)
        self._log.tag_config("err",   foreground=self.RED)
        self._log.tag_config("warn",  foreground=self.ORANGE)
        self._log.tag_config("cmd",   foreground="#aaaaff")
        self._log.tag_config("ard",   foreground="#88bbdd")
        self._log.tag_config("ui",    foreground=self.DIM)

    # ── Left control panel ────────────────────────────────────────────────────
    def _build_left_panel(self, parent):

        def section(text):
            tk.Frame(parent, bg=self.LINE, height=1).pack(fill=tk.X, padx=4, pady=(10, 0))
            tk.Label(
                parent, text=f"  {text}",
                bg=self.PANEL, fg=self.DIM,
                font=("Consolas", 8, "bold"),
            ).pack(anchor=tk.W, pady=(2, 3))

        def big_btn(text, cmd, color=None, state=tk.NORMAL):
            b = tk.Button(
                parent, text=text, command=cmd,
                bg=color or self.BTN, fg=self.TEXT,
                activebackground=self.BTN_HOV, activeforeground=self.TEXT,
                relief=tk.FLAT, font=("Consolas", 9, "bold"),
                cursor="hand2", bd=0, pady=5, state=state,
            )
            b.pack(fill=tk.X, padx=6, pady=2)
            return b

        def row_btns(pairs, color=None):
            row = tk.Frame(parent, bg=self.PANEL)
            row.pack(fill=tk.X, padx=6, pady=2)
            btns = []
            for text, cmd in pairs:
                b = tk.Button(
                    row, text=text, command=cmd,
                    bg=color or self.BTN, fg=self.TEXT,
                    activebackground=self.BTN_HOV,
                    relief=tk.FLAT, font=("Consolas", 9),
                    cursor="hand2", bd=0, pady=4,
                )
                b.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
                btns.append(b)
            return btns

        # ── Workflow ──────────────────────────────────────────────────────────
        section("WORKFLOW")
        self._run_btn  = big_btn("▶  RUN MAIN SCRIPT", self._start_main, color="#1a5c3a")
        self._stop_btn = big_btn("■  REQUEST STOP",    self._request_stop,
                                  color="#5c1a1a", state=tk.DISABLED)

        # ── Status display ────────────────────────────────────────────────────
        section("STATUS")
        self._phase_var = tk.StringVar(value="IDLE")
        self._op_var    = tk.StringVar(value="Op  —  /  —")
        self._result_var = tk.StringVar(value="✓ 0   ✗ 0")

        tk.Label(parent, textvariable=self._phase_var,
                 bg=self.PANEL, fg=self.GREEN,
                 font=("Consolas", 10, "bold")).pack(anchor=tk.W, padx=10, pady=1)
        tk.Label(parent, textvariable=self._op_var,
                 bg=self.PANEL, fg=self.TEXT,
                 font=("Consolas", 9)).pack(anchor=tk.W, padx=10, pady=1)
        tk.Label(parent, textvariable=self._result_var,
                 bg=self.PANEL, fg=self.TEXT,
                 font=("Consolas", 9)).pack(anchor=tk.W, padx=10, pady=1)

        # ── Position readout ──────────────────────────────────────────────────
        section("POSITION")
        self._px_var = tk.StringVar(value="X:    0.000 mm")
        self._py_var = tk.StringVar(value="Y:    0.000 mm")
        self._pz_var = tk.StringVar(value="Z:    0.000 mm")
        for var, col in [(self._px_var, self.ORANGE),
                          (self._py_var, self.ORANGE),
                          (self._pz_var, self.ORANGE)]:
            tk.Label(parent, textvariable=var,
                     bg=self.PANEL, fg=col,
                     font=("Consolas", 9, "bold")).pack(anchor=tk.W, padx=10, pady=1)

        # ── Homing ────────────────────────────────────────────────────────────
        section("HOMING")
        big_btn("HOME ALL  (h)", lambda: _manual_send("h"), color="#1a3d5c")
        row_btns([("HOME X", lambda: _manual_send("h x")),
                  ("HOME Y", lambda: _manual_send("h y"))])
        big_btn("HOME Z  (h z)", lambda: _manual_send("h z"))

        # ── Camera positions ──────────────────────────────────────────────────
        section("CAMERA SWEEP")
        row_btns([("CAM 1", lambda: _manual_send("cam1")),
                  ("CAM 2", lambda: _manual_send("cam2"))])

        # ── Grab / Place ──────────────────────────────────────────────────────
        section("GRAB BIT")
        row_btns([("BIT 1", lambda: _manual_send("grab bit1")),
                  ("BIT 2", lambda: _manual_send("grab bit2"))])
        row_btns([("BIT 3", lambda: _manual_send("grab bit3")),
                  ("BIT 4", lambda: _manual_send("grab bit4"))])

        section("PLACE BIT")
        row_btns([("BIT 1", lambda: _manual_send("place bit1")),
                  ("BIT 2", lambda: _manual_send("place bit2"))])
        row_btns([("BIT 3", lambda: _manual_send("place bit3")),
                  ("BIT 4", lambda: _manual_send("place bit4"))])

        # ── Single operations ─────────────────────────────────────────────────
        section("SINGLE OPERATION")
        row_btns([("UNSCREW", lambda: _manual_send("UNSCREW")),
                  ("PRESS",   lambda: _manual_send("PRESS"))])

        # ── Manual command entry ──────────────────────────────────────────────
        section("MANUAL COMMAND")
        self._cmd_var = tk.StringVar()
        entry_row = tk.Frame(parent, bg=self.PANEL)
        entry_row.pack(fill=tk.X, padx=6, pady=2)

        cmd_entry = tk.Entry(
            entry_row, textvariable=self._cmd_var,
            bg=self.LOG_BG, fg=self.TEXT,
            insertbackground=self.TEXT,
            font=("Consolas", 9), relief=tk.FLAT,
        )
        cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))
        cmd_entry.bind("<Return>", self._send_manual_cmd)

        tk.Button(
            entry_row, text="SEND",
            command=self._send_manual_cmd,
            bg=self.BTN, fg=self.TEXT,
            relief=tk.FLAT, font=("Consolas", 9),
            cursor="hand2", padx=6,
        ).pack(side=tk.LEFT)

    # ── Right area: camera + charts ───────────────────────────────────────────
    def _build_right_area(self, parent):
        # Top half: camera view (left) + charts (right)
        top = tk.Frame(parent, bg=self.BG)
        top.pack(fill=tk.BOTH, expand=True)

        self._build_camera_pane(top)
        self._build_charts_pane(top)

    def _build_camera_pane(self, parent):
        cam_outer = tk.LabelFrame(
            parent, text="  CAMERA / VISION  ",
            bg=self.PANEL, fg=self.GREEN,
            font=("Consolas", 9, "bold"),
            bd=1, relief=tk.GROOVE,
            labelanchor=tk.NW,
        )
        cam_outer.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 3))

        # Image display label — fills the pane
        self._cam_label = tk.Label(
            cam_outer,
            text="No image yet.\nImages appear automatically\nwhen the main script captures them.",
            bg=self.LOG_BG, fg=self.DIM,
            font=("Consolas", 9),
        )
        self._cam_label.pack(fill=tk.BOTH, expand=True)

        # Footer: shows filename + timestamp
        self._img_footer = tk.Label(
            cam_outer, text="",
            bg=self.PANEL, fg=self.DIM,
            font=("Consolas", 7), anchor=tk.W,
        )
        self._img_footer.pack(fill=tk.X, padx=4)

    def _build_charts_pane(self, parent):
        charts = tk.Frame(parent, bg=self.BG, width=430)
        charts.pack(side=tk.LEFT, fill=tk.Y, padx=(3, 0))
        charts.pack_propagate(False)

        # ── Laser chart ───────────────────────────────────────────────────────
        laser_frame = tk.LabelFrame(
            charts, text="  LASER DISTANCE (mm)  ",
            bg=self.PANEL, fg=self.GREEN,
            font=("Consolas", 9, "bold"),
            bd=1, relief=tk.GROOVE, labelanchor=tk.NW,
        )
        laser_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 3))

        self._laser_fig = Figure(figsize=(4.2, 2.4), facecolor=self.LOG_BG)
        self._laser_ax  = self._laser_fig.add_subplot(111)
        self._style_ax(self._laser_ax, ylabel="mm")
        self._laser_line, = self._laser_ax.plot([], [], color=self.BLUE, lw=1.3)
        self._laser_fig.tight_layout(pad=0.7)

        self._laser_canvas = FigureCanvasTkAgg(self._laser_fig, master=laser_frame)
        self._laser_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self._laser_readout = tk.Label(
            laser_frame, text="  — . — —  mm",
            bg=self.PANEL, fg=self.BLUE,
            font=("Consolas", 13, "bold"), anchor=tk.W,
        )
        self._laser_readout.pack(fill=tk.X, padx=8)

        # ── Force chart ───────────────────────────────────────────────────────
        force_frame = tk.LabelFrame(
            charts, text="  FORCE SENSOR (N)  ",
            bg=self.PANEL, fg=self.GREEN,
            font=("Consolas", 9, "bold"),
            bd=1, relief=tk.GROOVE, labelanchor=tk.NW,
        )
        force_frame.pack(fill=tk.BOTH, expand=True, pady=(3, 0))

        self._force_fig = Figure(figsize=(4.2, 2.4), facecolor=self.LOG_BG)
        self._force_ax  = self._force_fig.add_subplot(111)
        self._style_ax(self._force_ax, ylabel="N")
        # Max-force safety line at 5 N
        self._force_ax.axhline(5.0, color=self.RED, lw=0.9, ls="--", alpha=0.55,
                                label="limit 5 N")
        self._force_line, = self._force_ax.plot([], [], color=self.PINK, lw=1.3)
        self._force_fig.tight_layout(pad=0.7)

        self._force_canvas = FigureCanvasTkAgg(self._force_fig, master=force_frame)
        self._force_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self._force_readout = tk.Label(
            force_frame, text="  — . — —  N",
            bg=self.PANEL, fg=self.PINK,
            font=("Consolas", 13, "bold"), anchor=tk.W,
        )
        self._force_readout.pack(fill=tk.X, padx=8)

    def _style_ax(self, ax, ylabel=""):
        ax.set_facecolor(self.LOG_BG)
        ax.tick_params(colors=self.DIM, labelsize=7)
        ax.set_ylabel(ylabel, color=self.DIM, fontsize=7)
        ax.set_xlabel("time (s)", color=self.DIM, fontsize=7)
        for spine in ax.spines.values():
            spine.set_color(self.LINE)

    # ──────────────────────────────────────────────────────────────────────────
    # Button callbacks
    # ──────────────────────────────────────────────────────────────────────────
    def _start_main(self):
        if _main_running:
            return
        # Clear chart buffers for a fresh run
        self._laser_t.clear();  self._laser_mm.clear()
        self._force_t.clear();  self._force_N.clear()
        self._laser_ax.cla();   self._style_ax(self._laser_ax, "mm")
        self._force_ax.cla();   self._style_ax(self._force_ax, "N")
        self._force_ax.axhline(5.0, color=self.RED, lw=0.9, ls="--", alpha=0.55)
        self._laser_line, = self._laser_ax.plot([], [], color=self.BLUE, lw=1.3)
        self._force_line, = self._force_ax.plot([], [], color=self.PINK, lw=1.3)
        self._laser_canvas.draw()
        self._force_canvas.draw()

        self._run_btn.config(state=tk.DISABLED)
        self._stop_btn.config(state=tk.NORMAL)
        self._status_lbl.config(text="● RUNNING", fg=self.ORANGE)

        t = threading.Thread(
            target=_run_main_thread, args=(self.args_ns,), daemon=True
        )
        t.start()

    def _request_stop(self):
        """Best-effort stop — logs a message; the workflow finishes its current op."""
        self._append_log(
            "[UI] Stop requested. The workflow will exit after the current operation.\n",
            tag="warn",
        )

    def _send_manual_cmd(self, _event=None):
        cmd = self._cmd_var.get().strip()
        if cmd:
            self._cmd_var.set("")
            threading.Thread(target=_manual_send, args=(cmd,), daemon=True).start()

    def _clear_log(self):
        self._log.config(state=tk.NORMAL)
        self._log.delete("1.0", tk.END)
        self._log.config(state=tk.DISABLED)

    # ──────────────────────────────────────────────────────────────────────────
    # Queue polling — runs every ~80 ms in the main (Tk) thread
    # ──────────────────────────────────────────────────────────────────────────
    def _poll_queue(self):
        # Process up to 60 events per tick so we never block the UI
        for _ in range(60):
            try:
                evt, payload = _ui_queue.get_nowait()
            except queue.Empty:
                break

            if evt == "log":
                self._route_log(payload)

            elif evt == "ard_msg":
                self._parse_ard_msg(payload)

            elif evt == "laser":
                t_ms, mm = payload
                self._laser_t.append(t_ms)
                self._laser_mm.append(mm)
                self._laser_readout.config(text=f"  {mm:6.2f}  mm")

            elif evt == "force":
                t_ms, fN = payload
                self._force_t.append(t_ms)
                self._force_N.append(fN)
                self._force_readout.config(text=f"  {fN:6.2f}  N")

            elif evt == "state":
                self._refresh_state(payload)

            elif evt == "workflow_done":
                self._run_btn.config(state=tk.NORMAL)
                self._stop_btn.config(state=tk.DISABLED)
                self._status_lbl.config(text="● DONE", fg=self.GREEN)
                self._phase_var.set("DONE")

        self._update_charts()
        self.root.after(80, self._poll_queue)

    # ──────────────────────────────────────────────────────────────────────────
    # Log routing — picks a colour tag based on content
    # ──────────────────────────────────────────────────────────────────────────
    def _route_log(self, text):
        tu = text.upper()
        if any(k in tu for k in ("✓", "SUCCESS", "COMPLETE", "HOMED", "GRAB: COMPLETE", "PLACE: COMPLETE")):
            tag = "ok"
        elif any(k in tu for k in ("✗", "ERROR", "FAILED", "TIMED OUT")):
            tag = "err"
        elif any(k in tu for k in ("WARNING", "SKIP", "RETRY", "LASER RECOVERY")):
            tag = "warn"
        elif "[PI → ARD]" in text:
            tag = "cmd"
        elif "[ARD → PI]" in text:
            tag = "ard"
        elif "[UI]" in text:
            tag = "ui"
        else:
            tag = None
        self._append_log(text, tag=tag)

    def _append_log(self, text, tag=None):
        self._log.config(state=tk.NORMAL)
        if tag:
            self._log.insert(tk.END, text, tag)
        else:
            self._log.insert(tk.END, text)
        self._log.see(tk.END)
        self._log.config(state=tk.DISABLED)

    # ──────────────────────────────────────────────────────────────────────────
    # Arduino message parsing — updates status label and phase indicator
    # ──────────────────────────────────────────────────────────────────────────
    def _parse_ard_msg(self, line):
        lu = line.upper()

        if "SYSTEM IS HOMED" in lu:
            self._phase_var.set("HOMED")
            self._status_lbl.config(text="● HOMED", fg=self.GREEN)
            _post_state(pos_x=0.0, pos_y=0.0, pos_z=0.0)

        elif "SYSTEM IS AT POSITION" in lu:
            self._phase_var.set("AT POSITION")
            self._status_lbl.config(text="● MOVING", fg=self.ORANGE)

        elif "UNSCREW: CONTACT" in lu or "UNSCREW: SOCKET" in lu:
            self._phase_var.set("UNSCREWING ↓")
            self._status_lbl.config(text="● UNSCREWING", fg=self.ORANGE)

        elif "UNSCREW: ACTIVE" in lu:
            self._phase_var.set("UNSCREWING ↺")
            self._status_lbl.config(text="● UNSCREWING ↺", fg=self.ORANGE)

        elif "UNSCREW: COMPLETE" in lu:
            self._phase_var.set("UNSCREW ✓")
            self._status_lbl.config(text="● UNSCREW OK", fg=self.GREEN)

        elif "UNSCREW ERROR" in lu or "CONTROL RELEASED" in lu:
            self._phase_var.set("UNSCREW ✗")
            self._status_lbl.config(text="● UNSCREW ERR", fg=self.RED)

        elif "PRESS: CONTACT" in lu:
            self._phase_var.set("PRESSING ↓")
            self._status_lbl.config(text="● PRESSING", fg=self.ORANGE)

        elif "PRESS: COMPLETE" in lu:
            self._phase_var.set("PRESS ✓")
            self._status_lbl.config(text="● PRESS OK", fg=self.GREEN)

        elif "PRESS: FAILED" in lu or "PRESS ERROR" in lu:
            self._phase_var.set("PRESS ✗")
            self._status_lbl.config(text="● PRESS ERR", fg=self.RED)

        elif "PRESS: FORCE LIMIT" in lu:
            self._phase_var.set("FORCE LIMIT!")
            self._status_lbl.config(text="● FORCE LIMIT", fg=self.RED)

        elif "GRAB: COMPLETE" in lu:
            self._phase_var.set("BIT GRABBED")
            self._status_lbl.config(text="● BIT GRABBED", fg=self.GREEN)

        elif "GRAB ERROR" in lu:
            self._phase_var.set("GRAB ✗")
            self._status_lbl.config(text="● GRAB ERR", fg=self.RED)

        elif "PLACE: COMPLETE" in lu:
            self._phase_var.set("BIT PLACED")
            self._status_lbl.config(text="● BIT PLACED", fg=self.GREEN)

    # ──────────────────────────────────────────────────────────────────────────
    # State display refresh
    # ──────────────────────────────────────────────────────────────────────────
    def _refresh_state(self, state):
        phase = state.get("phase", "IDLE")
        if phase not in ("DONE", "IDLE"):
            pass  # phase label is updated more granularly by _parse_ard_msg
        else:
            self._phase_var.set(phase)

        n_ok   = state.get("op_ok",   0)
        n_fail = state.get("op_fail", 0)
        total  = state.get("op_total", 0)
        idx    = state.get("op_index", 0)

        self._result_var.set(f"✓ {n_ok}   ✗ {n_fail}")
        self._op_var.set(f"Op  {idx}  /  {total}" if total else "Op  —  /  —")
        self._px_var.set(f"X:  {state.get('pos_x', 0.0):8.3f} mm")
        self._py_var.set(f"Y:  {state.get('pos_y', 0.0):8.3f} mm")
        self._pz_var.set(f"Z:  {state.get('pos_z', 0.0):8.3f} mm")

    # ──────────────────────────────────────────────────────────────────────────
    # Live chart update — called every poll tick
    # ──────────────────────────────────────────────────────────────────────────
    def _update_charts(self):
        if len(self._laser_t) >= 2:
            t0 = self._laser_t[0]
            xs = [(t - t0) / 1000.0 for t in self._laser_t]
            self._laser_line.set_data(xs, list(self._laser_mm))
            self._laser_ax.relim()
            self._laser_ax.autoscale_view()
            self._laser_canvas.draw_idle()

        if len(self._force_t) >= 2:
            t0 = self._force_t[0]
            xs = [(t - t0) / 1000.0 for t in self._force_t]
            self._force_line.set_data(xs, list(self._force_N))
            self._force_ax.relim()
            self._force_ax.autoscale_view()
            self._force_canvas.draw_idle()

    # ──────────────────────────────────────────────────────────────────────────
    # Vision image watcher — checks for new images every 600 ms
    # ──────────────────────────────────────────────────────────────────────────
    def _poll_images(self):
        if PIL_AVAILABLE:
            try:
                from config import VISION_CURRENT_RUN_DIR, MAIN_ANALYSIS_DIR
                watch_dirs = [VISION_CURRENT_RUN_DIR, MAIN_ANALYSIS_DIR]
            except ImportError:
                watch_dirs = []

            candidates = []
            for d in watch_dirs:
                if os.path.isdir(d):
                    for fname in os.listdir(d):
                        if fname.lower().endswith((".jpg", ".jpeg", ".png")):
                            fp = os.path.join(d, fname)
                            try:
                                candidates.append((os.path.getmtime(fp), fp))
                            except OSError:
                                pass

            if candidates:
                candidates.sort(reverse=True)
                newest = candidates[0][1]
                if newest != self._last_img_path:
                    self._last_img_path = newest
                    self._display_image(newest)

        self.root.after(600, self._poll_images)

    def _display_image(self, path):
        if not PIL_AVAILABLE:
            return
        try:
            img = Image.open(path)
            # Fit inside the label widget without stretching
            w = max(self._cam_label.winfo_width(),  200)
            h = max(self._cam_label.winfo_height(), 200)
            img.thumbnail((w, h), Image.LANCZOS)
            photo = ImageTk.PhotoImage(img)
            self._cam_label.config(image=photo, text="")
            self._img_photoref = photo       # must stay referenced
            self._img_footer.config(
                text=f"  {os.path.basename(path)}"
            )
        except Exception as exc:
            self._img_footer.config(text=f"  (image error: {exc})")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Disassembly machine UI")
    parser.add_argument(
        "--no-execute", action="store_true",
        help="Run detection and planning only — skip execution",
    )
    args = parser.parse_args()

    # Patch serial_comm before any import of main.py
    _patch_serial_comm()

    # Redirect stdout so all print() output goes to both terminal and UI log
    orig_stdout = sys.stdout
    sys.stdout  = _LogRedirect(orig_stdout, _ui_queue)

    root = tk.Tk()
    app  = MachineUI(root, args)

    def _on_close():
        sys.stdout = orig_stdout
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", _on_close)

    try:
        root.mainloop()
    finally:
        sys.stdout = orig_stdout


if __name__ == "__main__":
    main()
