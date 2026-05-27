"""
Unscrew analyser.

Homes Z, starts LASER STREAM, sends UNSCREW, records every laser sample and all
key event timestamps through the full sequence (probe → pitch measure → active
unscrew → exit → retract). Plots laser reading and cumulative change over time
with vertical markers for each phase transition.

Usage:
    python unscrew_analyser.py
    python unscrew_analyser.py --port /dev/ttyACM0

Requirements: pip install pyserial matplotlib numpy
"""

import argparse
import os
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import serial

try:
    from config import SERIAL_PORT, BAUD_RATE, UNSCREW_PHOTOS_DIR
except ImportError:
    SERIAL_PORT       = "COM3"
    BAUD_RATE         = 115200
    UNSCREW_PHOTOS_DIR = "unscrew_photos"

os.makedirs(UNSCREW_PHOTOS_DIR, exist_ok=True)

# ── CLI ───────────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument("--port",    default=SERIAL_PORT)
parser.add_argument("--timeout", type=float, default=120.0,
                    help="Max seconds to wait for UNSCREW to complete")
args = parser.parse_args()


# ── helpers ───────────────────────────────────────────────────────────────────
def send(ser, cmd):
    ser.write((cmd.strip() + "\n").encode())
    print(f"  → {cmd.strip()}")


def wait_for(ser, keyword, timeout=30.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ser.in_waiting:
            try:
                line = ser.readline().decode(errors="replace").strip()
            except Exception:
                continue
            if line:
                print(f"  ← {line}")
                if keyword.upper() in line.upper():
                    return True
        else:
            time.sleep(0.01)
    return False


def wait_for_idle(ser, timeout=60.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        time.sleep(0.5)
        send(ser, "STATUS")
        if wait_for(ser, "STATUS IDLE", timeout=2.0):
            return True
    return False


# ── connect ───────────────────────────────────────────────────────────────────
print(f"Connecting to {args.port} @ {BAUD_RATE} …")
try:
    ser = serial.Serial(args.port, BAUD_RATE, timeout=0.01)
except serial.SerialException as e:
    sys.exit(f"Could not open port: {e}")
time.sleep(2)
ser.reset_input_buffer()
print("Connected.\n")

# ── home all axes ─────────────────────────────────────────────────────────────
print("Homing all axes …")
send(ser, "H")
if not wait_for(ser, "HOME: all axes homed", timeout=120.0):
    ser.close()
    sys.exit("Homing timed out.")
print("Homed.\n")

# ── move to SCREW1 ────────────────────────────────────────────────────────────
print("Moving to SCREW1 …")
send(ser, "SCREW1")
wait_for(ser, "SYSTEM IS AT SCREW1", timeout=60.0)
print("At SCREW1.\n")

input("Press Enter to begin the unscrew sequence …\n")

# ── enable laser stream ───────────────────────────────────────────────────────
send(ser, "LASER STREAM")
time.sleep(0.1)
ser.reset_input_buffer()

# ── send unscrew ──────────────────────────────────────────────────────────────
print("Sending UNSCREW ANALYSE …\n")
send(ser, "UNSCREW ANALYSE")

# ── collect ───────────────────────────────────────────────────────────────────
# Each entry: (arduino_ms, label, full_line)
EVENT_KEYWORDS = [
    ("UNSCREW: contact detected",          "contact"),
    ("UNSCREW: socket engaged",            "socket engaged"),
    ("UNSCREW: pitch measurement started", "pitch start"),
    ("UNSCREW: measured pitch",            "measured pitch"),
    ("UNSCREW: selected pitch",            "selected pitch"),
    ("UNSCREW: active unscrewing started", "active"),
    ("UNSCREW: screw exit detected",       "exit"),
    ("UNSCREW: done",                      "done"),
    ("UNSCREW ERROR",                      "ERROR"),
]

ard_ms            = []
laser_mm          = []
events            = []    # list of (ard_ms_value, short_label, full_line)
last_ard_ms       = 0
done              = False
error_msg         = None
probe_baseline_mm = None   # unscrewProbeStartDistance from Arduino
deadline          = time.time() + args.timeout

while time.time() < deadline and not done:
    if not ser.in_waiting:
        time.sleep(0.001)
        continue
    try:
        line = ser.readline().decode(errors="replace").strip()
    except Exception:
        continue
    if not line:
        continue

    if line.startswith("L:"):
        parts = line.split(":")
        if len(parts) == 3:
            try:
                ms  = int(parts[1])
                val = float(parts[2])
                ard_ms.append(ms)
                laser_mm.append(val)
                last_ard_ms = ms
            except ValueError:
                pass
    else:
        print(f"  ← {line}")
        if "UNSCREW: started" in line and "probe_baseline=" in line:
            try:
                probe_baseline_mm = float(line.split("probe_baseline=")[1].split()[0])
                print(f"  [analyser] probe_baseline={probe_baseline_mm:.3f} mm")
            except (IndexError, ValueError):
                pass
        for keyword, label in EVENT_KEYWORDS:
            if keyword.upper() in line.upper():
                # for measured/selected pitch, append the value to the label
                if "measured pitch" in keyword or "selected pitch" in keyword:
                    try:
                        val_str = line.split("=")[1].strip().split()[0]
                        label = f"{label} {val_str} mm"
                    except (IndexError, ValueError):
                        pass
                elif "ERROR" in keyword:
                    error_msg = line
                # use embedded Arduino timestamp if present, else fall back to last laser sample
                ts = last_ard_ms
                try:
                    ts = int(line.split(" at ")[1].split(" ms")[0])
                except (IndexError, ValueError):
                    pass
                events.append((ts, label, line))
                if "done" in keyword.lower() or "ERROR" in keyword.upper():
                    done = True
                break

# ── stop streaming & close ────────────────────────────────────────────────────
send(ser, "LASER STOP")
ser.close()

if len(laser_mm) < 10:
    sys.exit("Too few samples collected — check wiring and rerun.")

duration_s = (ard_ms[-1] - ard_ms[0]) / 1000.0
print(f"\nCollected {len(laser_mm)} samples over "
      f"{duration_s:.2f} s  "
      f"({len(laser_mm) / duration_s:.0f} Hz avg)\n")

if error_msg:
    print(f"Ended with error: {error_msg}\n")

# ── process ───────────────────────────────────────────────────────────────────
t_sec    = np.array([(ms - ard_ms[0]) / 1000.0 for ms in ard_ms])
laser    = np.array(laser_mm)
# Baseline for cumulative drop: use Arduino's probe_baseline if available, else first sample.
# The Arduino sets unscrewProbeStartDistance from a live laser reading slightly after
# recording starts, so using laser[0] can mis-align the contact threshold.
_cum_baseline = probe_baseline_mm if probe_baseline_mm is not None else laser[0]
cum_drop = _cum_baseline - laser   # positive = probe compressed, negative = spring released

event_times  = [(ms - ard_ms[0]) / 1000.0 for ms, _, _ in events]
event_labels = [lbl                        for _,  lbl, _ in events]

print("Events:")
for t, lbl in zip(event_times, event_labels):
    print(f"  t = {t:6.2f} s  →  {lbl}")

CONTACT_DROP_MM    = 2.5
UNSCREW_EXIT_MM    = 0.5

# ── plot ──────────────────────────────────────────────────────────────────────
PHASE_COLORS = {
    "contact":        "green",
    "socket engaged": "limegreen",
    "pitch start":    "goldenrod",
    "measured pitch": "darkorange",
    "selected pitch": "peru",
    "active":         "royalblue",
    "exit":           "darkorchid",
    "done":           "seagreen",
    "ERROR":          "red",
}

def event_color(lbl):
    for key, color in PHASE_COLORS.items():
        if lbl.startswith(key):
            return color
    return "black"

measured_pitch_lbl = next((lbl for lbl in event_labels if lbl.startswith("measured pitch")), None)
selected_pitch_lbl = next((lbl for lbl in event_labels if lbl.startswith("selected pitch")), None)
pitch_str = ""
if measured_pitch_lbl:
    pitch_str += f"  |  {measured_pitch_lbl}"
if selected_pitch_lbl:
    pitch_str += f"  |  {selected_pitch_lbl}"

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
fig.suptitle(
    f"Unscrew analyser — {len(laser_mm)} samples  |  {duration_s:.1f} s total"
    + (f"  |  ERROR" if error_msg else "  |  success")
    + pitch_str,
    fontsize=12
)

ax1.plot(t_sec, laser, lw=0.8, color="steelblue")
if probe_baseline_mm is not None:
    ax1.axhline(probe_baseline_mm, color="orange", lw=1.0, ls="--", alpha=0.7,
                label=f"probe baseline ({probe_baseline_mm:.2f} mm)")
    ax1.axhline(probe_baseline_mm - CONTACT_DROP_MM, color="green", lw=1.0, ls="--", alpha=0.7,
                label=f"contact threshold ({probe_baseline_mm - CONTACT_DROP_MM:.2f} mm)")
    ax1.legend(loc="lower left", fontsize=8)
ax1.set_ylabel("Laser distance (mm)")
ax1.set_title("Raw laser reading vs time")
ax1.grid(True, alpha=0.3)

ax2.fill_between(t_sec, 0, cum_drop, color="tomato", alpha=0.5)
ax2.axhline(0,               color="gray",   lw=0.8, ls=":")
ax2.axhline(CONTACT_DROP_MM, color="orange", lw=1.5, ls="--",
            label=f"contact threshold ({CONTACT_DROP_MM} mm)")
ax2.set_ylabel("Cumulative change from start (mm)\n+ = compressed  − = released")
ax2.set_xlabel("Time (s)")
ax2.set_title("Cumulative laser change  (positive = probe compressed, negative = screw pulled away)")
ax2.grid(True, alpha=0.3)

# draw event lines on both axes
for t, lbl in zip(event_times, event_labels):
    color = event_color(lbl)
    ax1.axvline(t, color=color, lw=1.5, ls="--")
    ax2.axvline(t, color=color, lw=1.5, ls="--")

# draw exit threshold relative to active-start laser value (matches Arduino logic)
active_events = [(t, lbl) for t, lbl in zip(event_times, event_labels) if lbl == "active"]
if active_events:
    t_active = active_events[0][0]
    idx_active = np.searchsorted(t_sec, t_active)
    cum_at_active = cum_drop[min(idx_active, len(cum_drop) - 1)]
    exit_line_y = cum_at_active - UNSCREW_EXIT_MM
    ax2.axhline(exit_line_y, color="purple", lw=1.5, ls="--",
                label=f"exit threshold (active start − {UNSCREW_EXIT_MM} mm)")

ax2.legend(loc="lower left", fontsize=9)

plt.tight_layout()

# draw event labels staggered so they don't overlap
# pitch values are in the title so skip them here
SKIP_LABELS = {"measured pitch", "selected pitch"}
y_min, y_max = ax1.get_ylim()
y_range = y_max - y_min
placed = []  # list of (t, y) already placed
for t, lbl in zip(event_times, event_labels):
    color = event_color(lbl)
    if any(lbl.startswith(s) for s in SKIP_LABELS):
        continue
    nearby = sum(1 for pt, _ in placed if abs(pt - t) < 3.0)
    y = y_min + 0.5 + nearby * y_range * 0.18
    placed.append((t, y))
    ax1.text(t + 0.1, y, lbl, color=color, fontsize=7, rotation=90, va="bottom")

out = os.path.join(UNSCREW_PHOTOS_DIR, "unscrew_analysis.png")
plt.savefig(out, dpi=150)
print(f"\nPlot saved → {out}")
plt.show()
