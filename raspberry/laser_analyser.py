"""
Laser drop analyser.

Homes Z, then moves it down into the screw so the spring-loaded probe
compresses and the laser reading changes. Streams every raw laser sample
(~200 Hz) and plots the result so you can see the signal shape and whether
any noise spikes are large enough to false-trigger contact detection.

Usage (run from the raspberry/ folder):
    python laser_analyser.py                   # defaults from config.py, 60 mm sweep
    python laser_analyser.py --port COM3
    python laser_analyser.py --port COM3 --mm 40

Requirements:  pip install pyserial matplotlib numpy

Before running:
    - Position X/Y over the screw you want to test against.
    - Z can be anywhere — the script homes it first.
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


# ── CLI ───────────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument("--port",    default=SERIAL_PORT)
parser.add_argument("--mm",      type=float, default=75.0,
                    help="Max Z travel (mm) — script stops early if contact detected")
parser.add_argument("--timeout", type=float, default=30.0,
                    help="Max seconds to wait for the Z move to complete")
args = parser.parse_args()


# ── helpers ───────────────────────────────────────────────────────────────────
def send(ser, cmd):
    ser.write((cmd.strip() + "\n").encode())
    print(f"  → {cmd.strip()}")


def wait_for(ser, keyword, timeout=30.0, also_print=True):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ser.in_waiting:
            try:
                line = ser.readline().decode(errors="replace").strip()
            except Exception:
                continue
            if line and also_print:
                print(f"  ← {line}")
            if line and keyword.upper() in line.upper():
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

# ── home Z only ───────────────────────────────────────────────────────────────
print("Homing Z …")
send(ser, "HOME Z")
if not wait_for(ser, "Axis Z safe", timeout=30.0):
    ser.close()
    sys.exit("Z homing timed out — check limit switch.")
print("Z homed.\n")

# ── enable laser stream ───────────────────────────────────────────────────────
send(ser, "LASER STREAM")
time.sleep(0.1)
ser.reset_input_buffer()

# ── move Z down into the screw ────────────────────────────────────────────────
print(f"Moving Z down {args.mm:.0f} mm — probe will contact the screw …\n")
send(ser, f"z {args.mm:.1f}")

# ── collect ───────────────────────────────────────────────────────────────────
CONTACT_DROP_MM = 2.0

ard_ms       = []
laser_mm     = []
contact_sent = False
stop_ms      = None   # Arduino timestamp when stop was sent
deadline     = time.time() + args.timeout

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

        # check cumulative drop and stop if contact threshold reached
        if not contact_sent and len(laser_mm) >= 2:
            if (laser_mm[0] - laser_mm[-1]) >= CONTACT_DROP_MM:
                stop_ms = ard_ms[-1]
                send(ser, "S")
                contact_sent = True
                print(f"\nContact detected — stopping.")

    elif line.startswith("DATA"):
        print(f"  ← {line}")
        print("\nMotion complete.")
        break

    else:
        print(f"  ← {line}")

# ── stop streaming ────────────────────────────────────────────────────────────
send(ser, "LASER STOP")
ser.close()

if len(laser_mm) < 10:
    sys.exit("Too few samples collected — check wiring and rerun.")

print(f"\nCollected {len(laser_mm)} samples over "
      f"{(ard_ms[-1] - ard_ms[0]) / 1000:.2f} s  "
      f"({len(laser_mm) / ((ard_ms[-1] - ard_ms[0]) / 1000):.0f} Hz avg)\n")

# ── process ───────────────────────────────────────────────────────────────────
# Estimate Z using AccelStepper kinematics — avoids reading Z from Arduino
# during motion which caused jamming.
# 5000 steps/s ÷ (400 steps/rev ÷ 4 mm/rev) = 50 mm/s
# 3500 steps/s² ÷ (400 steps/rev ÷ 4 mm/rev) = 35 mm/s²
PROBE_MAX_SPEED_MM_S = 50.0
PROBE_ACCEL_MM_S2    = 35.0
T_ACCEL = PROBE_MAX_SPEED_MM_S / PROBE_ACCEL_MM_S2   # time to reach full speed
D_ACCEL = 0.5 * PROBE_ACCEL_MM_S2 * T_ACCEL ** 2    # distance covered during accel

def estimate_z(t):
    if t <= T_ACCEL:
        return 0.5 * PROBE_ACCEL_MM_S2 * t ** 2
    return D_ACCEL + PROBE_MAX_SPEED_MM_S * (t - T_ACCEL)

t_sec  = np.array([(ms - ard_ms[0]) / 1000.0 for ms in ard_ms])
z_stop = estimate_z((stop_ms - ard_ms[0]) / 1000.0) if stop_ms is not None else None
z_mm   = np.array([
    estimate_z(t) if (stop_ms is None or ms <= stop_ms) else z_stop
    for t, ms in zip(t_sec, ard_ms)
])
laser = np.array(laser_mm)
cum_drop = laser[0] - laser     # mirrors Arduino: unscrewProbeStartDistance - latestLaserDistanceMm

contact_idx = next((i for i, d in enumerate(cum_drop) if d >= CONTACT_DROP_MM), None)

print(f"Laser range      : {laser.min():.3f} – {laser.max():.3f} mm")
print(f"Total drop at end: {cum_drop[-1]:.3f} mm")
if contact_idx is not None:
    print(f"Contact threshold crossed at Z = {z_mm[contact_idx]:.2f} mm  "
          f"(drop = {cum_drop[contact_idx]:.3f} mm)")
else:
    print("Contact threshold never crossed in this sweep.")

# ── plot ──────────────────────────────────────────────────────────────────────
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
contact_label = f"contact at Z = {z_mm[contact_idx]:.1f} mm" if contact_idx is not None else "no contact"
fig.suptitle(
    f"Laser analyser — {len(laser_mm)} samples  |  "
    f"total drop = {cum_drop[-1]:.2f} mm  |  {contact_label}",
    fontsize=12
)

ax1.plot(z_mm, laser, lw=0.8, color="steelblue")
ax1.set_ylabel("Laser distance (mm)")
ax1.set_title("Raw laser reading vs Z position")
ax1.grid(True, alpha=0.3)

ax2.fill_between(z_mm, 0, cum_drop, color="tomato", alpha=0.6,
                 label="cumulative drop from start (mm)")
ax2.axhline(CONTACT_DROP_MM, color="orange", lw=1.5, ls="--",
            label=f"CONTACT_DROP_MM = {CONTACT_DROP_MM}")
if contact_idx is not None:
    ax2.axvline(z_mm[contact_idx], color="red", lw=1.5, ls="--",
                label=f"contact detected at Z = {z_mm[contact_idx]:.1f} mm")
ax2.set_ylabel("Cumulative drop from start (mm)")
ax2.set_xlabel("Z position (mm)")
ax2.set_title("Cumulative laser drop — crosses orange line = contact detected")
ax2.legend(loc="upper left")
ax2.grid(True, alpha=0.3)

plt.tight_layout()
out = "laser_analysis.png"
plt.savefig(out, dpi=150)
print(f"\nPlot saved → {out}")
plt.show()
