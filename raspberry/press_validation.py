"""
press_validation.py

Runs 20 press cycles to validate press operation repeatability.

Workflow:
  1. Home machine
  2. Go to CAM1, find and iteratively centre on the red square
  3. Compute press target = square_x + 20 mm (same Y), corrected for camera offset
  4. Move axle to press target
  5. For each of 20 cycles:
       a. Prompt operator to press Enter
       b. Run PRESS ANALYSE — capture laser + force data
       c. Save analysis plot to output directory
       d. Append key metrics row to CSV
       e. Home Z
       f. If not last cycle: prompt to replace item

Usage:
    python press_validation.py
    python press_validation.py --cycles 5    # shorter run
"""

import argparse
import csv
import os
import time
import datetime
import numpy as np

from config import (
    CAM1_X, CAM1_Y,
    MM_PER_PIXEL_TXT_PATH,
    CAMERA_OFFSET_X_MM, CAMERA_OFFSET_Y_MM,
    VISION_CURRENT_RUN_DIR,
)
from camera import capture_image, load_mm_per_pixel
from vision import find_red_square_offset, check_press_success
from serial_comm import open_serial, send_command, wait_for_message
from main_analyser import _run_press, _plot_press

# ── Settings ──────────────────────────────────────────────────────────────────
PRESS_X_OFFSET_FROM_SQUARE_MM = 20.0   # press target is this far in +X from square
CENTER_TOLERANCE_MM = 0.5
MAX_CENTER_ITERS    = 4


def _centre_on_square(ser, approx_x, approx_y, mm_per_pixel_x, mm_per_pixel_y):
    """Iteratively centre camera on the red square. Returns (final_x, final_y)."""
    machine_x, machine_y = approx_x, approx_y
    for i in range(MAX_CENTER_ITERS):
        tmp = os.path.join(VISION_CURRENT_RUN_DIR, f"pressval_centre_iter{i}.jpg")
        capture_image(tmp)
        dx, dy = find_red_square_offset(tmp, mm_per_pixel_x, mm_per_pixel_y)
        print(f"  Centre iter {i+1}: dx={dx:+.3f} mm  dy={dy:+.3f} mm")
        if abs(dx) <= CENTER_TOLERANCE_MM and abs(dy) <= CENTER_TOLERANCE_MM:
            print("  Centred.")
            break
        send_command(ser, f"x {dx:.3f}")
        wait_for_message(ser, "SYSTEM IS AT X", timeout=60)
        send_command(ser, f"y {dy:.3f}")
        wait_for_message(ser, "SYSTEM IS AT Y", timeout=60)
        machine_x += dx
        machine_y += dy
    else:
        print(f"  WARNING: not fully centred after {MAX_CENTER_ITERS} iters.")
    return machine_x, machine_y


def _extract_csv_row(cycle, ok, msg, data, press_x, press_y,
                     vision_ok=None, vision_ratio=None):
    """Pull summary scalars out of _run_press data dict for one CSV row."""
    laser_t_ms = data["laser_t_ms"]
    force_N    = data["force_N"]
    contact_z  = data["contact_z_mm_val"]
    contact_d  = data["contact_distance"]

    duration_s = (laser_t_ms[-1] / 1000.0) if laser_t_ms else 0.0
    max_force  = float(np.max(force_N)) if force_N else float("nan")
    max_surf   = data.get("max_item_disp_mm", float("nan"))
    max_drop   = data.get("max_actual_drop_mm", float("nan"))

    return {
        "cycle":              cycle,
        "timestamp":          datetime.datetime.now().isoformat(timespec="seconds"),
        "ok":                 1 if ok else 0,
        "result_msg":         msg,
        "vision_ok":          ("" if vision_ok is None else (1 if vision_ok else 0)),
        "vision_ratio":       (f"{vision_ratio:.3f}" if vision_ratio is not None else ""),
        "press_x_mm":          f"{press_x:.3f}",
        "press_y_mm":          f"{press_y:.3f}",
        "probe_baseline_mm":  data["probe_baseline_mm"] if data["probe_baseline_mm"] is not None else "",
        "contact_distance_mm": f"{contact_d:.3f}" if contact_d is not None else "",
        "contact_z_mm":        f"{contact_z:.3f}" if contact_z is not None else "",
        "max_force_N":        f"{max_force:.3f}" if not np.isnan(max_force) else "",
        "max_item_disp_mm":   f"{max_surf:.3f}" if not np.isnan(max_surf) else "",
        "max_actual_drop_mm": f"{max_drop:.3f}" if not np.isnan(max_drop) else "",
        "duration_s":         f"{duration_s:.2f}",
        "n_laser_samples":    len(laser_t_ms),
        "n_force_samples":    len(force_N),
    }


CSV_FIELDS = [
    "cycle", "timestamp", "ok", "result_msg",
    "vision_ok", "vision_ratio",
    "press_x_mm", "press_y_mm",
    "probe_baseline_mm", "contact_distance_mm", "contact_z_mm",
    "max_force_N", "max_item_disp_mm", "max_actual_drop_mm",
    "duration_s", "n_laser_samples", "n_force_samples",
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cycles", type=int, default=20,
                        help="Number of press cycles (default 20)")
    args = parser.parse_args()
    n_cycles = args.cycles

    # ── Output directory ──────────────────────────────────────────────────────
    run_ts  = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(
        os.path.dirname(__file__), "press_validation_runs", run_ts
    )
    os.makedirs(out_dir, exist_ok=True)
    os.makedirs(VISION_CURRENT_RUN_DIR, exist_ok=True)
    csv_path = os.path.join(out_dir, "press_validation.csv")
    print(f"Output directory : {out_dir}")
    print(f"CSV              : {csv_path}")

    # ── Load scale ────────────────────────────────────────────────────────────
    mm_per_pixel_x, mm_per_pixel_y = load_mm_per_pixel(MM_PER_PIXEL_TXT_PATH)

    ser = open_serial()
    try:
        # ── Home ──────────────────────────────────────────────────────────────
        print("\n=== Homing machine ===")
        send_command(ser, "h")
        wait_for_message(ser, "SYSTEM IS HOMED", timeout=120)

        # ── Go to CAM1 ────────────────────────────────────────────────────────
        print(f"\n=== Moving to CAM1 (X={CAM1_X}, Y={CAM1_Y}) ===")
        send_command(ser, "cam1")
        wait_for_message(ser, "SYSTEM IS AT CAM1", timeout=60)

        # ── Find and centre on red square ─────────────────────────────────────
        print("\n=== Centering on red square ===")
        rs_x, rs_y = _centre_on_square(
            ser, CAM1_X, CAM1_Y, mm_per_pixel_x, mm_per_pixel_y
        )
        print(f"Red square machine position: X={rs_x:.3f}  Y={rs_y:.3f}")

        # ── Compute press target ───────────────────────────────────────────────
        # Press target in camera coords = square + (+20, 0) mm
        # Apply camera-to-bit offset so the axle sits over the button
        press_x = rs_x + PRESS_X_OFFSET_FROM_SQUARE_MM - CAMERA_OFFSET_X_MM
        press_y = rs_y - CAMERA_OFFSET_Y_MM
        print(f"Press target (bit coords): X={press_x:.3f}  Y={press_y:.3f}")

        # ── Move to press target ───────────────────────────────────────────────
        print("\n=== Moving to press target ===")
        send_command(ser, f"to {press_x:.3f} {press_y:.3f} 0.000")
        wait_for_message(ser, "SYSTEM IS AT POSITION", timeout=60)

        # ── Open CSV ──────────────────────────────────────────────────────────
        csv_file = open(csv_path, "w", newline="")
        writer   = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        csv_file.flush()

        succeeded = 0

        # ── Main cycle loop ───────────────────────────────────────────────────
        for cycle in range(1, n_cycles + 1):
            print(f"\n{'='*55}")
            print(f"  CYCLE {cycle}/{n_cycles}")
            print(f"{'='*55}")

            input(f"  Place item and press Enter to start cycle {cycle}/{n_cycles}...")

            # Enable laser stream, run press, stop stream
            from main_analyser import _raw_write
            _raw_write(ser, "LASER STREAM")
            time.sleep(0.2)
            ser.reset_input_buffer()

            ok, msg, data = _run_press(ser, timeout=120)

            _raw_write(ser, "LASER STOP")
            time.sleep(0.15)
            ser.reset_input_buffer()

            # Save analysis plot
            op = {"index": cycle, "type": "press", "subtype": "press",
                  "abs_x": press_x, "abs_y": press_y}
            try:
                _plot_press(op, data, ok, out_dir)
            except Exception as exc:
                print(f"  WARNING: plot failed: {exc}")

            # Home Z
            send_command(ser, "h z")
            wait_for_message(ser, "Axis Z safe. Position = 0.", timeout=30)

            # Camera verification: move camera above press target and check hole darkness
            verify_x = press_x + CAMERA_OFFSET_X_MM
            verify_y = press_y + CAMERA_OFFSET_Y_MM
            ser.reset_input_buffer()
            send_command(ser, f"to {verify_x:.3f} {verify_y:.3f} 0.000")
            wait_for_message(ser, "SYSTEM IS AT POSITION", timeout=60)

            verify_img   = os.path.join(out_dir, f"cycle_{cycle:02d}_verify.jpg")
            verify_debug = os.path.join(out_dir, f"cycle_{cycle:02d}_verify_debug.jpg")
            capture_image(verify_img)
            try:
                vision_ok, vision_ratio, _ = check_press_success(verify_img, verify_debug)
            except Exception as exc:
                print(f"  WARNING: vision check failed: {exc}")
                vision_ok, vision_ratio = None, None

            # Combined result after both sensor and vision checks
            sensor_str = "SUCCESS" if ok else "FAILED"
            vision_str = ("VISION OK" if vision_ok else
                          "VISION FAILED" if vision_ok is not None else
                          "VISION ERROR")
            if ok:
                succeeded += 1
            print(f"  Cycle {cycle}: {sensor_str} (sensors) / {vision_str}  —  {msg}")

            # Update CSV row with vision result and re-write it
            row = _extract_csv_row(cycle, ok, msg, data, press_x, press_y,
                                   vision_ok=vision_ok, vision_ratio=vision_ratio)
            writer.writerow(row)
            csv_file.flush()

            # Move back to press target so the axle is positioned for the next cycle
            print(f"  Returning to press target (X={press_x:.3f} Y={press_y:.3f})...")
            send_command(ser, f"to {press_x:.3f} {press_y:.3f} 0.000")
            wait_for_message(ser, "SYSTEM IS AT POSITION", timeout=60)

            if cycle < n_cycles:
                print(f"  Remove item if needed. Next cycle: {cycle+1}/{n_cycles}.")

        csv_file.close()

        # ── Summary ───────────────────────────────────────────────────────────
        print(f"\n{'='*55}")
        print(f"  VALIDATION COMPLETE: {succeeded}/{n_cycles} cycles succeeded")
        print(f"  CSV  → {csv_path}")
        print(f"  Plots→ {out_dir}")
        print(f"{'='*55}")

        # Home everything
        print("\n=== Final home ===")
        send_command(ser, "h")
        wait_for_message(ser, "SYSTEM IS HOMED", timeout=120)

    finally:
        try:
            csv_file.close()
        except Exception:
            pass
        ser.close()
        print("Serial closed.")


if __name__ == "__main__":
    main()
