"""
Press analyser.

Homes Z, starts LASER STREAM + FORCE STREAM, sends PRESS ANALYSE, records every
laser and force sample through the full sequence (descend → contact → press →
retract). Plots:
  1. Laser distance over time  (with contact marker and rigid-surface reference)
  2. Force over time           (with max-force threshold line)
  3. Surface movement          (actual drop − expected rigid drop from contact)

Usage:
    python press_analyser.py
    python press_analyser.py --port /dev/ttyACM0

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
    PRESS_PHOTOS_DIR = UNSCREW_PHOTOS_DIR.replace("unscrew", "press") if "unscrew" in UNSCREW_PHOTOS_DIR else UNSCREW_PHOTOS_DIR
except ImportError:
    SERIAL_PORT     = "COM3"
    BAUD_RATE       = 115200
    PRESS_PHOTOS_DIR = "press_photos"

os.makedirs(PRESS_PHOTOS_DIR, exist_ok=True)

# ── CLI ───────────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument("--port",      default=SERIAL_PORT)
parser.add_argument("--timeout",   type=float, default=120.0,
                    help="Max seconds to wait for PRESS to complete")
parser.add_argument("--max-force", type=float, default=5.0,
                    help="Force threshold drawn on the plot (N)")
args = parser.parse_args()

# ── Serial helpers ────────────────────────────────────────────────────────────
def send(ser, cmd):
    ser.write((cmd + "\n").encode())
    print(f"[PI → ARD] {cmd}")

def readline_timeout(ser, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if ser.in_waiting:
            try:
                return ser.readline().decode(errors="replace").strip()
            except Exception:
                pass
        time.sleep(0.005)
    return None

# ── Open serial ───────────────────────────────────────────────────────────────
print(f"Opening {args.port} @ {BAUD_RATE} …")
ser = serial.Serial(args.port, BAUD_RATE, timeout=0.05)
time.sleep(2)
ser.reset_input_buffer()

# ── Tare force sensor ─────────────────────────────────────────────────────────
print("Taring force sensor (20 samples) …")
send(ser, "T")
deadline = time.time() + 10.0
while time.time() < deadline:
    line = readline_timeout(ser, 0.5)
    if line:
        print(f"[ARD] {line}")
        if "Tared" in line:
            break

# ── Home Z ────────────────────────────────────────────────────────────────────
print("\nHoming Z …")
send(ser, "H Z")
deadline = time.time() + 60.0
while time.time() < deadline:
    line = readline_timeout(ser, 0.5)
    if line:
        print(f"[ARD] {line}")
        if "Axis Z safe" in line or "all axes homed" in line.lower():
            break

# ── Start streams & press ──────────────────────────────────────────────────────
send(ser, "LASER STREAM")
time.sleep(0.2)

input("\nPress Enter to begin the press sequence …\n")

print("Sending PRESS ANALYSE …\n")
send(ser, "PRESS ANALYSE")

# ── Record ────────────────────────────────────────────────────────────────────
# Laser samples: (timestamp_ms, distance_mm)
laser_t_ms  = []
laser_mm    = []

# Force samples: (timestamp_ms, force_N, z_mm)
force_t_ms  = []
force_N     = []
force_z_mm  = []

# Named events: (timestamp_ms, label)
events = []

DONE_MARKERS = ("PRESS: done", "PRESS ERROR")

probe_baseline_mm  = None
contact_distance   = None
contact_t_ms       = None
contact_z_mm_val   = None

t0_ms = None
started = False

print("Recording … (waiting for 'PRESS: done')\n")
deadline = time.time() + args.timeout

while time.time() < deadline:
    line = readline_timeout(ser, 0.2)
    if not line:
        continue
    print(f"[ARD] {line}")

    if "PRESS: started" in line and "probe_baseline=" in line:
        started = True
        try:
            probe_baseline_mm = float(line.split("probe_baseline=")[1].split()[0])
        except Exception:
            pass

    # Laser stream: L:<ms>:<mm>
    if line.startswith("L:"):
        parts = line.split(":")
        if len(parts) == 3:
            try:
                t_ms = int(parts[1])
                mm   = float(parts[2])
                if t0_ms is None and started:
                    t0_ms = t_ms
                if t0_ms is not None:
                    laser_t_ms.append(t_ms - t0_ms)
                    laser_mm.append(mm)
            except ValueError:
                pass
        continue

    # Force stream: F:<ms>:<N>:<z_mm>
    if line.startswith("F:"):
        parts = line.split(":")
        if len(parts) == 4:
            try:
                t_ms  = int(parts[1])
                fN    = float(parts[2])
                zMm   = float(parts[3])
                if t0_ms is None and started:
                    t0_ms = t_ms
                if t0_ms is not None:
                    force_t_ms.append(t_ms - t0_ms)
                    force_N.append(fN)
                    force_z_mm.append(zMm)
            except ValueError:
                pass
        continue

    # Event lines
    if "PRESS: contact detected" in line:
        try:
            contact_distance = float(line.split("probe_baseline=")[0].split("mm")[0].split()[-1]) if "probe_baseline" in line else None
        except Exception:
            pass
        try:
            # parse Z= value
            z_part = line.split("Z=")[1].split()[0].rstrip("mm")
            contact_z_mm_val = float(z_part)
        except Exception:
            pass
        if t0_ms is not None:
            contact_t_ms = laser_t_ms[-1] if laser_t_ms else None
        events.append((contact_t_ms, "contact"))

    if "PRESS: force limit reached" in line:
        ts = laser_t_ms[-1] if laser_t_ms else None
        events.append((ts, "force limit"))

    if "PRESS: depth limit reached" in line:
        ts = laser_t_ms[-1] if laser_t_ms else None
        events.append((ts, "depth limit"))

    if "PRESS: Z retracted" in line:
        ts = laser_t_ms[-1] if laser_t_ms else None
        events.append((ts, "retracted"))

    if any(m in line for m in DONE_MARKERS):
        break

ser.close()
print("\nSerial closed.")

if not laser_mm:
    print("No laser data recorded. Exiting.")
    sys.exit(1)

# ── Convert to numpy and seconds ──────────────────────────────────────────────
laser_t  = np.array(laser_t_ms)  / 1000.0
laser_mm = np.array(laser_mm)

force_t  = np.array(force_t_ms)  / 1000.0  if force_t_ms  else np.array([])
force_N  = np.array(force_N)                if force_N     else np.array([])
force_z  = np.array(force_z_mm)            if force_z_mm  else np.array([])

duration_s = laser_t[-1] if len(laser_t) else 0.0

# ── Find contact point in laser data ─────────────────────────────────────────
# Use the first reported contact distance or detect from laser drop
if probe_baseline_mm is None and laser_mm.size:
    probe_baseline_mm = laser_mm[0]

contact_idx = None
if contact_t_ms is not None:
    contact_t_s = contact_t_ms / 1000.0
    diffs = np.abs(laser_t - contact_t_s)
    contact_idx = int(np.argmin(diffs))
    contact_distance = laser_mm[contact_idx]

# ── Compute surface movement from laser data ──────────────────────────────────
# After contact: surface_move = (actual_laser_drop_from_contact) - (expected_drop_if_rigid)
# expected_drop_if_rigid = Z_descent_below_contact = Z_now - Z_at_contact (in mm going down)
# We approximate Z descent using force sample Z values interpolated to laser timestamps.
surface_move_t  = np.array([])
surface_move_mm = np.array([])

if contact_idx is not None and len(force_z) > 1:
    post_laser_t  = laser_t[contact_idx:]
    post_laser_mm = laser_mm[contact_idx:]
    z_at_contact  = float(np.interp(laser_t[contact_idx], force_t, force_z)) if force_t.size else contact_z_mm_val or 0.0

    # Interpolate Z to laser timestamps after contact
    z_interp = np.interp(post_laser_t, force_t, force_z)
    z_depth  = (z_interp - z_at_contact)   # positive = Z gone down (Z_DOWN_SIGN=1 means bigger value = deeper)

    actual_drop  = contact_distance - post_laser_mm    # positive = laser reading decreased
    surface_move_mm = actual_drop - z_depth            # positive = surface moved away (button pressed)
    surface_move_t  = post_laser_t

# ── Plot ──────────────────────────────────────────────────────────────────────
EVENT_COLORS = {
    "contact":     ("tab:green",  "--"),
    "force limit": ("tab:red",    "--"),
    "depth limit": ("tab:orange", "--"),
    "retracted":   ("tab:blue",   ":"),
}

fig, axes_plot = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
fig.suptitle(
    f"Press analyser — {len(laser_mm)} laser samples  |  {len(force_N)} force samples  |  {duration_s:.1f} s",
    fontsize=12
)

# ── Subplot 1: Laser distance ─────────────────────────────────────────────────
ax1 = axes_plot[0]
ax1.plot(laser_t, laser_mm, color="steelblue", lw=1.2, label="Laser distance")
if probe_baseline_mm is not None:
    ax1.axhline(probe_baseline_mm, color="grey", lw=0.8, ls=":", label=f"Baseline {probe_baseline_mm:.2f} mm")
if contact_idx is not None:
    ax1.axhline(contact_distance, color="tab:green", lw=0.8, ls="--",
                label=f"Contact {contact_distance:.2f} mm")
ax1.set_ylabel("Distance (mm)")
ax1.legend(fontsize=8, loc="upper right")
ax1.grid(True, alpha=0.3)

# ── Subplot 2: Force ─────────────────────────────────────────────────────────
ax2 = axes_plot[1]
if force_t.size:
    ax2.plot(force_t, force_N, color="tab:red", lw=1.2, label="Force")
    ax2.axhline(args.max_force, color="darkred", lw=0.8, ls="--",
                label=f"Max force {args.max_force:.1f} N")
else:
    ax2.text(0.5, 0.5, "No force data", ha="center", va="center",
             transform=ax2.transAxes, fontsize=11, color="grey")
ax2.set_ylabel("Force (N)")
ax2.legend(fontsize=8, loc="upper right")
ax2.grid(True, alpha=0.3)

# ── Subplot 3: Surface movement ───────────────────────────────────────────────
ax3 = axes_plot[2]
if surface_move_mm.size:
    ax3.plot(surface_move_t, surface_move_mm, color="tab:purple", lw=1.2,
             label="Surface movement")
    ax3.axhline(0, color="grey", lw=0.6, ls=":")
else:
    ax3.text(0.5, 0.5, "No contact data for surface movement", ha="center", va="center",
             transform=ax3.transAxes, fontsize=11, color="grey")
ax3.set_ylabel("Surface movement (mm)")
ax3.set_xlabel("Time (s)")
ax3.legend(fontsize=8, loc="upper right")
ax3.grid(True, alpha=0.3)

# ── Event markers on all subplots ─────────────────────────────────────────────
for t_ms, label in events:
    if t_ms is None:
        continue
    t_s   = t_ms / 1000.0
    color, ls = EVENT_COLORS.get(label, ("black", "--"))
    for ax in axes_plot:
        ax.axvline(t_s, color=color, lw=1.0, ls=ls, alpha=0.7)
    axes_plot[0].text(t_s, axes_plot[0].get_ylim()[0], f" {label}",
                      color=color, fontsize=7, va="bottom", rotation=90)

plt.tight_layout()
out = os.path.join(PRESS_PHOTOS_DIR, "press_analysis.png")
plt.savefig(out, dpi=150)
print(f"\nPlot saved → {out}")
plt.show()
