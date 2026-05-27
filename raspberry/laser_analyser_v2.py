"""
Laser drop analyser v2 — rolling average contact detection.

Same data collection as v1, but instead of checking the instantaneous laser
reading against the start value, it compares a rolling average of the last
~1 mm of travel against the rolling average captured at probe start.

This shows whether a smoothed contact check would be reliable despite the
raw signal noise, and gives you a concrete window size to implement in the
Arduino.

Usage:
    python laser_analyser_v2.py
    python laser_analyser_v2.py --port COM3 --mm 60 --window-mm 1.0

Requirements:  pip install pyserial matplotlib numpy
Before running: position X/Y over the screw, Z will be homed by the script.
"""

import argparse
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import serial

try:
    from config import SERIAL_PORT, BAUD_RATE
except ImportError:
    SERIAL_PORT = "COM3"
    BAUD_RATE   = 115200

PROBE_SPEED_MM_S = 50.0   # 5000 steps/s ÷ 400 steps/rev × 4 mm/rev

# ── CLI ───────────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument("--port",       default=SERIAL_PORT)
parser.add_argument("--mm",         type=float, default=75.0)
parser.add_argument("--window-mm",  type=float, default=1.0,
                    help="Rolling average window in mm of Z travel (default 1.0)")
parser.add_argument("--timeout",    type=float, default=30.0)
args = parser.parse_args()

window_ms = (args.window_mm / PROBE_SPEED_MM_S) * 1000.0
print(f"Window: {args.window_mm} mm  =  {window_ms:.0f} ms at {PROBE_SPEED_MM_S} mm/s\n")


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


# ── connect ───────────────────────────────────────────────────────────────────
print(f"Connecting to {args.port} @ {BAUD_RATE} …")
try:
    ser = serial.Serial(args.port, BAUD_RATE, timeout=0.01)
except serial.SerialException as e:
    sys.exit(f"Could not open port: {e}")
time.sleep(2)
ser.reset_input_buffer()
print("Connected.\n")

# ── home Z ────────────────────────────────────────────────────────────────────
print("Homing Z …")
send(ser, "HOME Z")
if not wait_for(ser, "Axis Z safe", timeout=30.0):
    ser.close()
    sys.exit("Z homing timed out.")
print("Z homed.\n")

input(f"Press Enter to move Z down {args.mm:.0f} mm into the screw …\n")

# ── stream + move ─────────────────────────────────────────────────────────────
send(ser, "LASER STREAM")
time.sleep(0.1)
ser.reset_input_buffer()
send(ser, f"z {args.mm:.1f}")
print("\nCollecting …\n")

ard_ms   = []
laser_mm = []
deadline = time.time() + args.timeout

while time.time() < deadline:
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
                ard_ms.append(int(parts[1]))
                laser_mm.append(float(parts[2]))
            except ValueError:
                pass
    elif line.startswith("DATA"):
        print(f"  ← {line}")
        print("\nMotion complete.")
        break
    else:
        print(f"  ← {line}")

send(ser, "LASER STOP")
ser.close()

if len(laser_mm) < 20:
    sys.exit("Too few samples.")

print(f"\nCollected {len(laser_mm)} samples over "
      f"{(ard_ms[-1] - ard_ms[0]) / 1000:.2f} s  "
      f"({len(laser_mm) / ((ard_ms[-1] - ard_ms[0]) / 1000):.0f} Hz avg)\n")

# ── rolling average ───────────────────────────────────────────────────────────
t_ms   = np.array(ard_ms, dtype=float)
t_ms  -= t_ms[0]
laser  = np.array(laser_mm)

def rolling_avg(values, timestamps_ms, window_ms):
    result = np.empty_like(values)
    for i in range(len(values)):
        mask = (timestamps_ms[i] - timestamps_ms >= 0) & \
               (timestamps_ms[i] - timestamps_ms <= window_ms)
        result[i] = values[mask].mean()
    return result

print(f"Computing {window_ms:.0f} ms rolling average …")
smoothed = rolling_avg(laser, t_ms, window_ms)

t_sec = t_ms / 1000.0

# drops on the smoothed signal
smooth_delta = np.diff(smoothed)
smooth_drop  = np.maximum(-smooth_delta, 0.0)

# simulate contact detection: drop from smoothed start value
smooth_start = smoothed[0]
smooth_diff_from_start = smooth_start - smoothed   # positive when lower than start

CONTACT_DROP_MM = 2.0
contact_idx = next(
    (i for i, d in enumerate(smooth_diff_from_start) if d >= CONTACT_DROP_MM),
    None
)

# ── stats ─────────────────────────────────────────────────────────────────────
print(f"Raw   range : {laser.min():.2f} – {laser.max():.2f} mm  "
      f"(peak-to-peak {laser.max()-laser.min():.2f} mm)")
print(f"Smooth range: {smoothed.min():.2f} – {smoothed.max():.2f} mm  "
      f"(peak-to-peak {smoothed.max()-smoothed.min():.2f} mm)")
print(f"Max smoothed step-drop : {smooth_drop.max():.3f} mm")
if contact_idx is not None:
    print(f"Simulated contact detection at t={t_sec[contact_idx]:.3f} s  "
          f"(Z ≈ {t_sec[contact_idx] * PROBE_SPEED_MM_S:.1f} mm)")
else:
    print("Simulated contact: NOT triggered during this sweep.")

# ── plot ──────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
fig.suptitle(
    f"Laser analyser v2 — {args.window_mm} mm rolling avg  |  "
    f"{len(laser_mm)} samples  |  "
    f"raw p-p {laser.max()-laser.min():.1f} mm  →  "
    f"smooth p-p {smoothed.max()-smoothed.min():.1f} mm",
    fontsize=12
)

# panel 1: raw
axes[0].plot(t_sec, laser, lw=0.5, color="steelblue", alpha=0.7, label="raw")
axes[0].set_ylabel("Laser (mm)")
axes[0].set_title("Raw laser reading")
axes[0].grid(True, alpha=0.3)
axes[0].legend(loc="upper right")

# panel 2: smoothed + contact marker
axes[1].plot(t_sec, smoothed, lw=1.2, color="steelblue", label=f"{args.window_mm} mm rolling avg")
axes[1].axhline(smooth_start - CONTACT_DROP_MM, color="orange", lw=1.2, ls="--",
                label=f"contact threshold (start − {CONTACT_DROP_MM} mm)")
if contact_idx is not None:
    axes[1].axvline(t_sec[contact_idx], color="red", lw=1.5, ls="--",
                    label=f"contact detected at t={t_sec[contact_idx]:.3f} s")
axes[1].set_ylabel("Laser (mm)")
axes[1].set_title("Smoothed signal + simulated contact detection")
axes[1].grid(True, alpha=0.3)
axes[1].legend(loc="upper right")

# panel 3: per-step drops on the smoothed signal
axes[2].fill_between(t_sec[:-1], 0, smooth_drop, color="tomato", alpha=0.6,
                     label="smoothed drop per step")
axes[2].axhline(CONTACT_DROP_MM, color="orange", lw=1.5, ls="--",
                label=f"CONTACT_DROP_MM = {CONTACT_DROP_MM}")
if contact_idx is not None:
    axes[2].axvline(t_sec[contact_idx], color="red", lw=1.5, ls="--")
axes[2].set_ylabel("Drop per step (mm)")
axes[2].set_xlabel("Time (s)")
axes[2].set_title("Per-step drops on smoothed signal")
axes[2].legend(loc="upper right")
axes[2].grid(True, alpha=0.3)

plt.tight_layout()
out = "laser_analysis_v2.png"
plt.savefig(out, dpi=150)
print(f"\nPlot saved → {out}")
plt.show()
