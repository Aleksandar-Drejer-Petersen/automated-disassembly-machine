"""
vision_validation.py

Validates vision accuracy for screw detection.

Ground truth for SCREW1 (lower-right screw) measured via the POSITION command
on 2026-05-20:  X=75.763 mm, Y=85.563 mm
Update SCREW1_X / SCREW1_Y if the board is repositioned and POSITION is re-run.

Workflow:
  1. Machine homes, then runs N full cycles:
       CAM1  →  find red square  →  move to square  →  centre  →  detect screws
  2. Each cycle records the detected abs position of the screw closest to ground
     truth and computes the error.
  3. Results saved as a CSV (raw data + statistics block).

Usage:
    python vision_validation.py
    python vision_validation.py --iterations 20 --output results.csv
"""

import argparse
import csv
import math
import os
import sys
from datetime import datetime

from config import (
    MM_PER_PIXEL_TXT_PATH,
    CAMERA_OFFSET_X_MM, CAMERA_OFFSET_Y_MM,
    VISION_CURRENT_RUN_DIR,
    CAM1_X, CAM1_Y,
)
from camera import capture_image, load_mm_per_pixel
from vision import find_red_square_offset, find_all_red_squares, detect_screws
from serial_comm import open_serial, send_command, wait_for_message

os.makedirs(VISION_CURRENT_RUN_DIR, exist_ok=True)

# ── Ground truth (from POSITION command run, 2026-05-20) ─────────────────────
SCREW1_X = 75.763   # mm
SCREW1_Y = 85.563   # mm

CENTER_TOLERANCE_MM = 0.5
MAX_CENTER_ITERS    = 4


# ── Vision helpers ────────────────────────────────────────────────────────────
def _centre_on_square(ser, approx_x, approx_y, mm_per_pixel_x, mm_per_pixel_y, label):
    machine_x, machine_y = approx_x, approx_y
    for i in range(MAX_CENTER_ITERS):
        img = os.path.join(VISION_CURRENT_RUN_DIR, f"val_{label}_c{i}.jpg")
        capture_image(img)
        dx, dy = find_red_square_offset(img, mm_per_pixel_x, mm_per_pixel_y)
        print(f"    centre iter {i+1}: dx={dx:+.3f}  dy={dy:+.3f}")
        if abs(dx) <= CENTER_TOLERANCE_MM and abs(dy) <= CENTER_TOLERANCE_MM:
            print("    centred.")
            break
        send_command(ser, f"x {dx:.3f}")
        wait_for_message(ser, "SYSTEM IS AT X", timeout=60)
        send_command(ser, f"y {dy:.3f}")
        wait_for_message(ser, "SYSTEM IS AT Y", timeout=60)
        machine_x += dx
        machine_y += dy
    else:
        print(f"    WARNING: not fully centred after {MAX_CENTER_ITERS} iters.")
    return machine_x, machine_y


# ── Stats helpers ─────────────────────────────────────────────────────────────
def _mean(vals):
    return sum(vals) / len(vals) if vals else 0.0

def _rms(vals):
    return math.sqrt(sum(v ** 2 for v in vals) / len(vals)) if vals else 0.0

def _std(vals):
    if not vals:
        return 0.0
    m = _mean(vals)
    return math.sqrt(sum((v - m) ** 2 for v in vals) / len(vals))


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Vision accuracy validation — runs the full detection pipeline N times."
    )
    parser.add_argument("--iterations", type=int, default=20,
                        help="Detection cycles to run (default: 20)")
    parser.add_argument("--output", default=None,
                        help="CSV output path (default: vision_validation_<ts>.csv)")
    args = parser.parse_args()

    ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = args.output or f"vision_validation_{ts}.csv"

    print()
    print("=" * 60)
    print("  Vision Accuracy Validation")
    print("=" * 60)
    print(f"  Ground truth : X={SCREW1_X:.3f}  Y={SCREW1_Y:.3f}  (SCREW1)")
    print(f"  Iterations   : {args.iterations}")
    print(f"  Output CSV   : {csv_path}")
    print()
    input("  Press Enter to home and begin …\n")

    mm_per_pixel_x, mm_per_pixel_y = load_mm_per_pixel(MM_PER_PIXEL_TXT_PATH)
    ser = open_serial()

    records = []

    try:
        print("=== Homing machine ===")
        send_command(ser, "h")
        wait_for_message(ser, "SYSTEM IS HOMED", timeout=120)

        for trial in range(1, args.iterations + 1):
            print(f"\n{'─' * 55}")
            print(f"  Trial {trial} / {args.iterations}")
            print(f"{'─' * 55}")

            # ── Go to CAM1 ────────────────────────────────────────────────
            send_command(ser, "cam1")
            wait_for_message(ser, "SYSTEM IS AT CAM1", timeout=60)

            # ── Find all red squares ──────────────────────────────────────
            sweep_img = os.path.join(VISION_CURRENT_RUN_DIR, f"val_t{trial:02d}_sweep.jpg")
            capture_image(sweep_img)

            found = find_all_red_squares(sweep_img, mm_per_pixel_x, mm_per_pixel_y)
            abs_squares = [(CAM1_X + dx, CAM1_Y + dy) for dx, dy in found]

            if not abs_squares:
                print(f"  [trial {trial}] No red squares found — skipping.")
                records.append(_skip_record(trial, "no red square found"))
                continue

            sq = min(abs_squares, key=lambda s: math.hypot(s[0] - SCREW1_X, s[1] - SCREW1_Y))
            print(f"  Red square at approx  X={sq[0]:.2f}  Y={sq[1]:.2f}")

            send_command(ser, f"to {sq[0]:.3f} {sq[1]:.3f} 0.000")
            wait_for_message(ser, "SYSTEM IS AT POSITION", timeout=60)

            # ── Centre on square ──────────────────────────────────────────
            rs_x, rs_y = _centre_on_square(
                ser, sq[0], sq[1], mm_per_pixel_x, mm_per_pixel_y, f"t{trial:02d}"
            )
            print(f"  Square centred at  X={rs_x:.3f}  Y={rs_y:.3f}")

            # ── Detect screws ─────────────────────────────────────────────
            det_img   = os.path.join(VISION_CURRENT_RUN_DIR, f"val_t{trial:02d}_detect.jpg")
            debug_img = os.path.join(VISION_CURRENT_RUN_DIR, f"val_t{trial:02d}_debug.jpg")
            capture_image(det_img)

            screw_results, _ = detect_screws(det_img, debug_img, mm_per_pixel_x, mm_per_pixel_y)

            if not screw_results:
                print(f"  [trial {trial}] No screws detected — skipping.")
                records.append(_skip_record(trial, "no screws detected"))
                continue

            for s in screw_results:
                s["abs_x"] = rs_x + s["machine_dx"] - CAMERA_OFFSET_X_MM
                s["abs_y"] = rs_y + s["machine_dy"] - CAMERA_OFFSET_Y_MM

            best = min(screw_results,
                       key=lambda s: math.hypot(s["abs_x"] - SCREW1_X, s["abs_y"] - SCREW1_Y))

            det_x    = best["abs_x"]
            det_y    = best["abs_y"]
            err_x    = det_x - SCREW1_X
            err_y    = det_y - SCREW1_Y
            combined = math.hypot(err_x, err_y)

            print(f"  Detected  X={det_x:.3f}  Y={det_y:.3f}  "
                  f"err_x={err_x:+.3f}  err_y={err_y:+.3f}  "
                  f"|err|={combined:.3f} mm  ({best['screw_type']})")

            records.append({
                "trial":          trial,
                "detected_x":     round(det_x,    4),
                "detected_y":     round(det_y,    4),
                "err_x":          round(err_x,    4),
                "err_y":          round(err_y,    4),
                "combined_error": round(combined, 4),
                "note":           "",
            })

    finally:
        ser.close()
        print("\nSerial closed.")

    # ── Statistics ────────────────────────────────────────────────────────────
    valid = [r for r in records if r["err_x"] is not None]
    n     = len(valid)

    if n == 0:
        print("No valid trials — cannot compute statistics.")
        sys.exit(1)

    ex = [r["err_x"]          for r in valid]
    ey = [r["err_y"]          for r in valid]
    ev = [r["combined_error"] for r in valid]

    stats = {
        "mean_signed_err_x": _mean(ex),
        "mean_signed_err_y": _mean(ey),
        "mean_abs_err_x":    _mean([abs(v) for v in ex]),
        "mean_abs_err_y":    _mean([abs(v) for v in ey]),
        "max_abs_err_x":     max(abs(v) for v in ex),
        "max_abs_err_y":     max(abs(v) for v in ey),
        "rms_err_x":         _rms(ex),
        "rms_err_y":         _rms(ey),
        "std_err_x":         _std(ex),
        "std_err_y":         _std(ey),
        "max_combined":      max(ev),
        "rms_combined":      _rms(ev),
    }
    worst = max(valid, key=lambda r: r["combined_error"])

    # ── Write CSV ─────────────────────────────────────────────────────────────
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)

        w.writerow(["Vision Accuracy Validation"])
        w.writerow(["Timestamp",      ts])
        w.writerow(["Ground truth X", f"{SCREW1_X:.4f} mm"])
        w.writerow(["Ground truth Y", f"{SCREW1_Y:.4f} mm"])
        w.writerow(["Valid trials",   f"{n} / {args.iterations}"])
        w.writerow([])

        w.writerow(["trial", "detected_x", "detected_y",
                    "err_x", "err_y", "combined_error_mm", "note"])
        for r in records:
            w.writerow([
                r["trial"],
                r["detected_x"]     if r["detected_x"]     is not None else "–",
                r["detected_y"]     if r["detected_y"]     is not None else "–",
                r["err_x"]          if r["err_x"]          is not None else "–",
                r["err_y"]          if r["err_y"]          is not None else "–",
                r["combined_error"] if r["combined_error"]  is not None else "–",
                r["note"],
            ])

        w.writerow([])
        w.writerow(["STATISTICS"])
        w.writerow([])
        w.writerow(["metric",              "X (mm)",                          "Y (mm)"])
        w.writerow(["Mean signed error",   f"{stats['mean_signed_err_x']:+.4f}", f"{stats['mean_signed_err_y']:+.4f}"])
        w.writerow(["Mean absolute error", f"{stats['mean_abs_err_x']:.4f}",     f"{stats['mean_abs_err_y']:.4f}"])
        w.writerow(["Max absolute error",  f"{stats['max_abs_err_x']:.4f}",      f"{stats['max_abs_err_y']:.4f}"])
        w.writerow(["RMS error",           f"{stats['rms_err_x']:.4f}",          f"{stats['rms_err_y']:.4f}"])
        w.writerow(["Std deviation",       f"{stats['std_err_x']:.4f}",          f"{stats['std_err_y']:.4f}"])
        w.writerow([])
        w.writerow(["metric",                    "value (mm)"])
        w.writerow(["Max combined vector error",  f"{stats['max_combined']:.4f}"])
        w.writerow(["RMS combined vector error",  f"{stats['rms_combined']:.4f}"])
        w.writerow([f"Worst trial: #{worst['trial']}",
                    f"err_x={worst['err_x']:+.4f}",
                    f"err_y={worst['err_y']:+.4f}",
                    f"|err|={worst['combined_error']:.4f}"])

    # ── Console summary ───────────────────────────────────────────────────────
    print(f"\n{'=' * 55}")
    print(f"  RESULTS  ({n}/{args.iterations} valid trials)")
    print(f"{'=' * 55}")
    print(f"  Ground truth:  X={SCREW1_X:.3f}  Y={SCREW1_Y:.3f}")
    print()
    print(f"  {'Metric':<26} {'X (mm)':>10} {'Y (mm)':>10}")
    print(f"  {'─' * 46}")
    print(f"  {'Mean signed error':<26} {stats['mean_signed_err_x']:>+10.3f} {stats['mean_signed_err_y']:>+10.3f}")
    print(f"  {'Mean absolute error':<26} {stats['mean_abs_err_x']:>10.3f} {stats['mean_abs_err_y']:>10.3f}")
    print(f"  {'Max absolute error':<26} {stats['max_abs_err_x']:>10.3f} {stats['max_abs_err_y']:>10.3f}")
    print(f"  {'RMS error':<26} {stats['rms_err_x']:>10.3f} {stats['rms_err_y']:>10.3f}")
    print(f"  {'Std deviation':<26} {stats['std_err_x']:>10.3f} {stats['std_err_y']:>10.3f}")
    print()
    print(f"  Max combined vector error : {stats['max_combined']:.3f} mm  (trial #{worst['trial']})")
    print(f"  RMS combined vector error : {stats['rms_combined']:.3f} mm")
    print()
    print(f"  CSV saved → {csv_path}")
    print(f"{'=' * 55}")


def _skip_record(trial, note):
    return {
        "trial": trial, "detected_x": None, "detected_y": None,
        "err_x": None, "err_y": None, "combined_error": None,
        "note": note,
    }


if __name__ == "__main__":
    main()
