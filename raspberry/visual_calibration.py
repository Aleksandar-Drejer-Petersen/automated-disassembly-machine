"""
visual_calibration.py

Full visual calibration pipeline — run this whenever you move the machine,
swap the camera, or the detection accuracy degrades.

  Phase 1  Homography calibration  — moves over a 25-point grid, detects
           the fixed red square at each position, fits a new H matrix and
           saves it to output/homography.npz.

  Phase 2  Validation (10 trials, raw) — re-validates against the known
           component layout WITHOUT any perspective correction so the true
           residual error is measured.

  Phase 3  Linear correction fit — fits
               err_x = a_x * abs_x + b_x
               err_y = a_y * abs_y + b_y
           from the validation data and derives the correction:
               corrected_x = (1 − a_x) * abs_x − b_x
               corrected_y = (1 − a_y) * abs_y − b_y

  Phase 4  CSV output + apply prompt — writes a full report CSV then asks
           "Apply correction? (y/n)".  If yes, saves
           output/perspective_correction.npz (loaded automatically by
           transform.apply_perspective_correction).

Before running
--------------
  1. Update S1X, S1Y below with the SCREW1 machine position.
     Use the POSITION command on the machine while the bit tip is centred
     on SCREW1.
  2. Ensure the fixed red calibration square is on the work surface and
     accessible in the Y = 75 – 125 mm region.
  3. Ensure the PCB component is on the work surface.

Usage
-----
    python visual_calibration.py
    python visual_calibration.py --skip-phase1   # skip H recalibration
"""

import argparse
import csv
import math
import os
import time
from datetime import datetime

import cv2
import numpy as np

from config import (
    CALIBRATION_NPZ_PATH,
    VISION_PHOTOS_DIR,
    MM_PER_PIXEL_TXT_PATH,
    CAMERA_OFFSET_X_MM,
    CAMERA_OFFSET_Y_MM,
    VISION_CURRENT_RUN_DIR,
    CAM1_X,
    CAM1_Y,
)
from camera import capture_image, undistort_image, load_mm_per_pixel
from vision import find_red_square_offset, find_all_red_squares, detect_screws, reload_homography
from serial_comm import open_serial, send_command, wait_for_message
from transform import HOMOGRAPHY_NPZ_PATH, PERSPECTIVE_NPZ_PATH

os.makedirs(VISION_PHOTOS_DIR,      exist_ok=True)
os.makedirs(VISION_CURRENT_RUN_DIR, exist_ok=True)
os.makedirs(os.path.dirname(HOMOGRAPHY_NPZ_PATH),  exist_ok=True)
os.makedirs(os.path.dirname(PERSPECTIVE_NPZ_PATH), exist_ok=True)


# ══════════════════════════════════════════════════════════════════════════════
#  EDIT THESE when the PCB position changes
#  Measure SCREW1 with the POSITION command (bit tip centred on the screw).
# ══════════════════════════════════════════════════════════════════════════════
S1X = 75.763   # machine X of SCREW1 (bottom-right reference screw)
S1Y = 85.563   # machine Y of SCREW1

# ── Ground truth layout (20 mm grid) ──────────────────────────────────────────
#
#   As seen from the front of the machine (Y increases to the left):
#
#     SCREW2  PRESS1   SCREW1        X = S1X        (bottom) ← SCREW1 lower-right
#     SCREW4  ──────  SCREW3        X = S1X + 20   (middle — centre slot empty)
#     SCREW6  PRESS2   SCREW5        X = S1X + 40   (top)
#              Y: S1Y+40  S1Y+20   S1Y
#
GROUND_TRUTH = [
    ("SCREW1", "screw", S1X,       S1Y,       "bottom", "right"),
    ("PRESS1",  "press",  S1X,       S1Y + 20,  "bottom", "centre"),
    ("SCREW2", "screw", S1X,       S1Y + 40,  "bottom", "left"),
    ("SCREW3", "screw", S1X + 20,  S1Y,       "middle", "right"),
    ("SCREW4", "screw", S1X + 20,  S1Y + 40,  "middle", "left"),
    ("SCREW5", "screw", S1X + 40,  S1Y,       "top",    "right"),
    ("PRESS2",  "press",  S1X + 40,  S1Y + 20,  "top",    "centre"),
    ("SCREW6", "screw", S1X + 40,  S1Y + 40,  "top",    "left"),
]


# ── Phase 1: grid calibration settings ────────────────────────────────────────
X_FROM  = -20.0   # mm offset from CAM1_X
X_TO    =  20.0
Y_FROM  =  75.0   # mm offset from CAM1_Y  (covers the screw Y range)
Y_TO    = 125.0
STEPS   =   5     # 5 × 5 = 25 grid points

X_MIN, X_MAX = 5.0,  530.0   # hard machine limits
Y_MIN, Y_MAX = 5.0,  360.0

# ── Phase 2: validation settings ──────────────────────────────────────────────
N_TRIALS            = 10
MATCH_THRESHOLD_MM  = 12.0
CENTER_TOLERANCE_MM =  0.5
MAX_CENTER_ITERS    =   4


# ── Shared helpers ─────────────────────────────────────────────────────────────

def _find_red_center(image_path):
    """Return (cx, cy) pixel of the largest red blob, or None."""
    img = cv2.imread(image_path)
    if img is None:
        return None
    img = undistort_image(img, CALIBRATION_NPZ_PATH)
    hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0,   120, 70]), np.array([10,  255, 255])),
        cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255])),
    )
    k    = np.ones((15, 15), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    rx, ry, rw, rh = cv2.boundingRect(max(contours, key=cv2.contourArea))
    return float(rx + rw / 2), float(ry + rh / 2)


def _centre_on_square(ser, approx_x, approx_y, mpx, mpy, label):
    """Iteratively centre the camera on the red square. Returns (x, y)."""
    mx, my = approx_x, approx_y
    for i in range(MAX_CENTER_ITERS):
        img = os.path.join(VISION_CURRENT_RUN_DIR, f"vc_{label}_c{i}.jpg")
        capture_image(img)
        dx, dy = find_red_square_offset(img, mpx, mpy)
        print(f"    centre iter {i+1}: dx={dx:+.3f}  dy={dy:+.3f}")
        if abs(dx) <= CENTER_TOLERANCE_MM and abs(dy) <= CENTER_TOLERANCE_MM:
            print("    centred.")
            break
        send_command(ser, f"x {dx:.3f}")
        wait_for_message(ser, "SYSTEM IS AT X", timeout=60)
        send_command(ser, f"y {dy:.3f}")
        wait_for_message(ser, "SYSTEM IS AT Y", timeout=60)
        mx += dx
        my += dy
    else:
        print(f"    WARNING: not fully centred after {MAX_CENTER_ITERS} iters.")
    return mx, my


def _mean(v): return sum(v) / len(v) if v else 0.0
def _rms(v):  return math.sqrt(sum(x**2 for x in v) / len(v)) if v else 0.0
def _std(v):
    if not v: return 0.0
    m = _mean(v)
    return math.sqrt(sum((x - m)**2 for x in v) / len(v))


# ══════════════════════════════════════════════════════════════════════════════
#  PHASE 1 — Homography calibration
# ══════════════════════════════════════════════════════════════════════════════

def phase1_calibrate_homography(ser):
    """
    Traverse a 5×5 grid, detect the fixed red square pixel centre at each
    position, fit H, save to HOMOGRAPHY_NPZ_PATH.

    Returns (success: bool, h_stats: dict | None).
    """
    print()
    print("=" * 62)
    print("  PHASE 1 — Homography calibration (5×5 grid)")
    print("=" * 62)

    xs = np.linspace(X_FROM, X_TO, STEPS)
    ys = np.linspace(Y_FROM, Y_TO, STEPS)
    offsets = [(float(dx), float(dy)) for dy in ys for dx in xs]

    valid = []
    for dx, dy in offsets:
        mx = CAM1_X + dx
        my = CAM1_Y + dy
        if X_MIN <= mx <= X_MAX and Y_MIN <= my <= Y_MAX:
            valid.append((dx, dy, mx, my))
        else:
            print(f"  Skipping ({dx:+.0f}, {dy:+.0f}) — machine pos "
                  f"({mx:.0f}, {my:.0f}) out of limits")

    print(f"  {len(valid)}/{len(offsets)} positions within machine limits")

    print("\n  Homing …")
    send_command(ser, "h")
    wait_for_message(ser, "SYSTEM IS HOMED", timeout=120)

    print("  Moving to CAM1 …")
    send_command(ser, "cam1")
    wait_for_message(ser, "SYSTEM IS AT CAM1", timeout=60)

    pixel_pts   = []
    machine_pts = []
    failed      = []
    cur_x, cur_y = CAM1_X, CAM1_Y

    for i, (dx, dy, mx, my) in enumerate(valid):
        print(f"  [{i+1:2d}/{len(valid)}]  machine ({mx:.1f}, {my:.1f}) …",
              end="  ", flush=True)

        rel_x = mx - cur_x
        rel_y = my - cur_y

        if abs(rel_x) > 0.01:
            send_command(ser, f"x {rel_x:.3f}")
            wait_for_message(ser, "SYSTEM IS AT X", timeout=60)
        if abs(rel_y) > 0.01:
            send_command(ser, f"y {rel_y:.3f}")
            wait_for_message(ser, "SYSTEM IS AT Y", timeout=60)

        cur_x, cur_y = mx, my
        time.sleep(0.3)   # let vibration settle

        photo = os.path.join(
            VISION_PHOTOS_DIR,
            f"vc_grid_{i+1:02d}_x{mx:.0f}_y{my:.0f}.jpg"
        )
        capture_image(photo)

        centre = _find_red_center(photo)
        if centre is None:
            print("NO RED SQUARE")
            failed.append(i + 1)
        else:
            print(f"pixel ({centre[0]:.0f}, {centre[1]:.0f})")
            pixel_pts.append(centre)
            machine_pts.append((mx, my))

    print(f"\n  Collected {len(pixel_pts)} valid point(s), {len(failed)} failed.")

    if len(pixel_pts) < 4:
        print("  ERROR: need at least 4 valid points — "
              "check that the red square is visible across the grid.")
        return False, None

    print("  Fitting homography …")
    H, mask = cv2.findHomography(
        np.array(pixel_pts,   dtype=np.float32),
        np.array(machine_pts, dtype=np.float32),
        cv2.RANSAC, 1.0,
    )

    if H is None:
        print("  ERROR: cv2.findHomography() returned None.")
        return False, None

    inliers   = int(mask.sum())
    proj      = cv2.perspectiveTransform(
        np.array(pixel_pts, dtype=np.float32).reshape(-1, 1, 2), H
    ).reshape(-1, 2)
    residuals = np.linalg.norm(proj - np.array(machine_pts), axis=1)
    rms_err   = float(residuals.mean())
    max_err   = float(residuals.max())

    print(f"  {inliers}/{len(pixel_pts)} inliers  "
          f"RMS={rms_err:.3f} mm  Max={max_err:.3f} mm")

    if rms_err > 2.0:
        print("  WARNING: high reprojection error — "
              "check that the red square was clearly visible at all positions.")

    np.savez(HOMOGRAPHY_NPZ_PATH, H=H,
             rms_error=np.array([rms_err]),
             max_error=np.array([max_err]),
             n_points=np.array([len(pixel_pts)]))
    print(f"  Homography saved → {HOMOGRAPHY_NPZ_PATH}")

    if failed:
        print(f"  Skipped grid points: {failed} — "
              "check photos in vision_photos/ to diagnose.")

    return True, {"rms": rms_err, "max": max_err, "n": len(pixel_pts)}


# ══════════════════════════════════════════════════════════════════════════════
#  PHASE 2 — Validation (raw — no perspective correction)
# ══════════════════════════════════════════════════════════════════════════════

def phase2_validate(ser, mpx, mpy):
    """
    Run N_TRIALS detection cycles over the known component layout.
    abs_x / abs_y are computed directly (no apply_perspective_correction call)
    so the raw residual is measured.

    Returns records dict keyed by (op_name, trial_int).
    """
    print()
    print("=" * 62)
    print(f"  PHASE 2 — Validation  ({N_TRIALS} trials, RAW — no correction)")
    print("=" * 62)

    comp_cx = _mean([gt[2] for gt in GROUND_TRUTH])
    comp_cy = _mean([gt[3] for gt in GROUND_TRUTH])
    records = {}

    print("\n  Homing …")
    send_command(ser, "h")
    wait_for_message(ser, "SYSTEM IS HOMED", timeout=120)

    for trial in range(1, N_TRIALS + 1):
        print(f"\n  {'─' * 55}")
        print(f"  Trial {trial} / {N_TRIALS}")
        print(f"  {'─' * 55}")

        # ── Go to CAM1 ────────────────────────────────────────────────────
        send_command(ser, "cam1")
        wait_for_message(ser, "SYSTEM IS AT CAM1", timeout=60)

        # ── Sweep → find red square ───────────────────────────────────────
        sweep_img = os.path.join(VISION_CURRENT_RUN_DIR,
                                 f"vc_t{trial:02d}_sweep.jpg")
        capture_image(sweep_img)

        found = find_all_red_squares(sweep_img, mpx, mpy)
        abs_squares = [(CAM1_X + dx, CAM1_Y + dy) for dx, dy in found]

        if not abs_squares:
            print("  No red squares found — skipping trial.")
            for name, *_ in GROUND_TRUTH:
                records[(name, trial)] = None
            continue

        sq = min(abs_squares,
                 key=lambda s: math.hypot(s[0] - comp_cx, s[1] - comp_cy))
        print(f"  Red square approx  X={sq[0]:.2f}  Y={sq[1]:.2f}")

        send_command(ser, f"to {sq[0]:.3f} {sq[1]:.3f} 0.000")
        wait_for_message(ser, "SYSTEM IS AT POSITION", timeout=60)

        # ── Centre on square ──────────────────────────────────────────────
        rs_x, rs_y = _centre_on_square(
            ser, sq[0], sq[1], mpx, mpy, f"t{trial:02d}"
        )
        print(f"  Square centred  X={rs_x:.3f}  Y={rs_y:.3f}")

        # ── Detect ────────────────────────────────────────────────────────
        det_img   = os.path.join(VISION_CURRENT_RUN_DIR,
                                 f"vc_t{trial:02d}_detect.jpg")
        debug_img = os.path.join(VISION_CURRENT_RUN_DIR,
                                 f"vc_t{trial:02d}_debug.jpg")
        capture_image(det_img)
        screw_results, _ = detect_screws(det_img, debug_img, mpx, mpy)

        if not screw_results:
            print("  No operations detected — skipping trial.")
            for name, *_ in GROUND_TRUTH:
                records[(name, trial)] = None
            continue

        # Compute absolute position — RAW, no perspective correction
        for s in screw_results:
            s["abs_x"] = rs_x - s["machine_dx"] - CAMERA_OFFSET_X_MM
            s["abs_y"] = rs_y + s["machine_dy"] - CAMERA_OFFSET_Y_MM

        print(f"  {len(screw_results)} detection(s) (raw positions):")
        for s in screw_results:
            print(f"    abs=({s['abs_x']:.3f}, {s['abs_y']:.3f})  "
                  f"dx={s['machine_dx']:+.2f} dy={s['machine_dy']:+.2f}")

        # ── Match to ground truth ─────────────────────────────────────────
        unmatched = list(screw_results)
        for name, op_type, gt_x, gt_y, row_lbl, col_lbl in GROUND_TRUTH:
            candidates = [
                s for s in unmatched
                if math.hypot(s["abs_x"] - gt_x,
                              s["abs_y"] - gt_y) < MATCH_THRESHOLD_MM
            ]
            if not candidates:
                print(f"    {name:<7}: NOT DETECTED")
                records[(name, trial)] = None
                continue

            best = min(candidates,
                       key=lambda s: math.hypot(s["abs_x"] - gt_x,
                                                s["abs_y"] - gt_y))
            unmatched.remove(best)

            err_x    = best["abs_x"] - gt_x
            err_y    = best["abs_y"] - gt_y
            combined = math.hypot(err_x, err_y)

            print(f"    {name:<7}: det=({best['abs_x']:.3f}, {best['abs_y']:.3f})"
                  f"  err=({err_x:+.3f}, {err_y:+.3f})  |err|={combined:.3f} mm")

            records[(name, trial)] = {
                "op":         name,
                "trial":      trial,
                "gt_x":       gt_x,
                "gt_y":       gt_y,
                "detected_x": round(best["abs_x"], 4),
                "detected_y": round(best["abs_y"], 4),
                "err_x":      round(err_x,    4),
                "err_y":      round(err_y,    4),
                "combined":   round(combined, 4),
                "pixel_cx":   best["cx"],
                "pixel_cy":   best["cy"],
                "rs_x":       rs_x,
                "rs_y":       rs_y,
            }

    return records


# ══════════════════════════════════════════════════════════════════════════════
#  PHASE 3 — Fit linear perspective correction
# ══════════════════════════════════════════════════════════════════════════════

def phase3_fit_correction(records):
    """
    Fit:
        err_x = a_x * abs_x + b_x
        err_y = a_y * abs_y + b_y

    Correction to apply downstream:
        corrected_x = (1 − a_x) * abs_x − b_x   ≡   scale_x * abs_x + offset_x
        corrected_y = (1 − a_y) * abs_y − b_y   ≡   scale_y * abs_y + offset_y

    Returns a dict, or None if insufficient data.
    """
    print()
    print("=" * 62)
    print("  PHASE 3 — Linear perspective correction fit")
    print("=" * 62)

    all_abs_x, all_err_x = [], []
    all_abs_y, all_err_y = [], []

    for r in records.values():
        if r is None:
            continue
        all_abs_x.append(r["detected_x"])
        all_err_x.append(r["err_x"])
        all_abs_y.append(r["detected_y"])
        all_err_y.append(r["err_y"])

    n = len(all_abs_x)
    print(f"  Using {n} valid detection(s).")

    if n < 4:
        print(f"  ERROR: need ≥ 4 points to fit — only {n} available.")
        return None

    ax, bx = np.polyfit(all_abs_x, all_err_x, 1)
    ay, by = np.polyfit(all_abs_y, all_err_y, 1)

    scale_x  = 1.0 - ax
    offset_x = -bx
    scale_y  = 1.0 - ay
    offset_y = -by

    # R² quality check
    def r2(actual, predicted):
        ss_res = float(np.sum((np.array(actual) - np.array(predicted)) ** 2))
        ss_tot = float(np.sum((np.array(actual) - np.mean(actual)) ** 2))
        return 1.0 - ss_res / ss_tot if ss_tot > 1e-9 else 0.0

    ex_pred = [ax * x + bx for x in all_abs_x]
    ey_pred = [ay * y + by for y in all_abs_y]
    r2_x = r2(all_err_x, ex_pred)
    r2_y = r2(all_err_y, ey_pred)

    print()
    print(f"  Fit (X):  err_x = {ax:+.5f} × abs_x  {bx:+.5f}   R²={r2_x:.3f}")
    print(f"  Fit (Y):  err_y = {ay:+.5f} × abs_y  {by:+.5f}   R²={r2_y:.3f}")
    print()
    print("  Correction formula:")
    print(f"    corrected_x = {scale_x:.5f} × abs_x  {offset_x:+.5f}")
    print(f"    corrected_y = {scale_y:.5f} × abs_y  {offset_y:+.5f}")

    if r2_x < 0.5 or r2_y < 0.5:
        print()
        print("  NOTE: R² is low — the spatial gradient may not be the dominant")
        print("  error source.  Correction will still be saved if applied,")
        print("  but consider rechecking S1X / S1Y and re-running.")

    return {
        "scale_x":  scale_x,
        "offset_x": offset_x,
        "scale_y":  scale_y,
        "offset_y": offset_y,
        "a_x": ax, "b_x": bx, "r2_x": r2_x,
        "a_y": ay, "b_y": by, "r2_y": r2_y,
        "n_points": n,
    }


# ══════════════════════════════════════════════════════════════════════════════
#  PHASE 4 — CSV output
# ══════════════════════════════════════════════════════════════════════════════

def phase4_write_csv(records, correction, h_stats, ts, applied):
    """Write full report CSV and return the file path."""
    csv_path = f"visual_calibration_{ts}.csv"

    # Per-operation statistics
    op_stats = {}
    for name, *_ in GROUND_TRUTH:
        valid = [records[(name, t)]
                 for t in range(1, N_TRIALS + 1)
                 if records.get((name, t)) is not None]
        if not valid:
            op_stats[name] = None
            continue
        ex = [r["err_x"]    for r in valid]
        ey = [r["err_y"]    for r in valid]
        ev = [r["combined"] for r in valid]
        op_stats[name] = {
            "n":            len(valid),
            "mean_ex":      _mean(ex),
            "mean_ey":      _mean(ey),
            "rms_ex":       _rms(ex),
            "rms_ey":       _rms(ey),
            "std_ex":       _std(ex),
            "std_ey":       _std(ey),
            "max_combined": max(ev),
            "rms_combined": _rms(ev),
        }

    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)

        # ── Report header ────────────────────────────────────────────────
        w.writerow(["Visual Calibration Report"])
        w.writerow(["Timestamp",     ts])
        w.writerow(["Iterations",    N_TRIALS])
        w.writerow(["SCREW1 ref",    f"X={S1X}  Y={S1Y}"])
        w.writerow(["Grid spacing",  "20 mm"])
        if h_stats:
            w.writerow(["H calibration RMS (mm)", f"{h_stats['rms']:.4f}"])
            w.writerow(["H calibration Max (mm)", f"{h_stats['max']:.4f}"])
            w.writerow(["H calibration points",   h_stats["n"]])
        else:
            w.writerow(["H calibration stats", "not available"])
        w.writerow([])

        # ── Ground truth ─────────────────────────────────────────────────
        w.writerow(["GROUND TRUTH POSITIONS"])
        w.writerow(["name", "type", "gt_x", "gt_y", "position"])
        for name, typ, gx, gy, row, col in GROUND_TRUTH:
            w.writerow([name, typ, f"{gx:.3f}", f"{gy:.3f}", f"{row} {col}"])
        w.writerow([])

        # ── Raw data ─────────────────────────────────────────────────────
        w.writerow(["RAW DATA  (before perspective correction)"])
        w.writerow(["trial", "operation", "gt_x", "gt_y",
                    "detected_x", "detected_y",
                    "err_x", "err_y", "combined_error_mm",
                    "pixel_cx", "pixel_cy", "rs_x", "rs_y", "note"])
        gt_lookup = {name: (gx, gy) for name, _, gx, gy, *_ in GROUND_TRUTH}

        for trial in range(1, N_TRIALS + 1):
            for name, *_ in GROUND_TRUTH:
                r = records.get((name, trial))
                gx, gy = gt_lookup[name]
                if r is None:
                    w.writerow([trial, name, f"{gx:.3f}", f"{gy:.3f}",
                                "–", "–", "–", "–", "–",
                                "–", "–", "–", "–", "not detected"])
                else:
                    w.writerow([trial, name, f"{gx:.3f}", f"{gy:.3f}",
                                r["detected_x"], r["detected_y"],
                                r["err_x"],      r["err_y"],      r["combined"],
                                r["pixel_cx"],   r["pixel_cy"],
                                round(r["rs_x"], 4), round(r["rs_y"], 4), ""])
        w.writerow([])

        # ── Per-operation statistics ─────────────────────────────────────
        w.writerow(["PER-OPERATION STATISTICS"])
        w.writerow(["name", "position", "n",
                    "mean_err_x", "mean_err_y",
                    "rms_err_x",  "rms_err_y",
                    "std_err_x",  "std_err_y",
                    "max_combined_mm", "rms_combined_mm"])
        for name, typ, gx, gy, row, col in GROUND_TRUTH:
            s   = op_stats[name]
            pos = f"{row} {col}"
            if s is None:
                w.writerow([name, pos, 0] + ["–"] * 8)
            else:
                w.writerow([
                    name, pos, s["n"],
                    f"{s['mean_ex']:+.4f}", f"{s['mean_ey']:+.4f}",
                    f"{s['rms_ex']:.4f}",   f"{s['rms_ey']:.4f}",
                    f"{s['std_ex']:.4f}",   f"{s['std_ey']:.4f}",
                    f"{s['max_combined']:.4f}", f"{s['rms_combined']:.4f}",
                ])
        w.writerow([])

        # ── Perspective correction ────────────────────────────────────────
        w.writerow(["PERSPECTIVE CORRECTION FIT"])
        if correction:
            c = correction
            w.writerow(["n points used", c["n_points"]])
            w.writerow([])
            w.writerow(["axis", "fit formula", "R²"])
            w.writerow(["X",
                        f"err_x = {c['a_x']:+.5f} × abs_x  {c['b_x']:+.5f}",
                        f"{c['r2_x']:.3f}"])
            w.writerow(["Y",
                        f"err_y = {c['a_y']:+.5f} × abs_y  {c['b_y']:+.5f}",
                        f"{c['r2_y']:.3f}"])
            w.writerow([])
            w.writerow(["Suggested correction (saved in perspective_correction.npz):"])
            w.writerow(["scale_x",  f"{c['scale_x']:.6f}"])
            w.writerow(["offset_x", f"{c['offset_x']:+.6f}"])
            w.writerow(["scale_y",  f"{c['scale_y']:.6f}"])
            w.writerow(["offset_y", f"{c['offset_y']:+.6f}"])
            w.writerow([])
            if applied:
                w.writerow(["Applied automatically", "YES"])
                w.writerow(["Saved to", PERSPECTIVE_NPZ_PATH])
            else:
                w.writerow(["Applied automatically", "NO"])
                w.writerow(["Note",
                             "Re-run visual_calibration.py and answer 'y' to apply"])
        else:
            w.writerow(["fit failed — insufficient valid detections"])

    return csv_path


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Full visual calibration: H recalibration → validation → "
                    "perspective correction fit → CSV + apply prompt."
    )
    parser.add_argument("--skip-phase1", action="store_true",
                        help="Skip H recalibration and use the existing homography")
    args = parser.parse_args()

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    print()
    print("=" * 62)
    print("  Visual Calibration Pipeline")
    print("=" * 62)
    print()
    print(f"  SCREW1 reference: X={S1X}  Y={S1Y}")
    print()
    print("  Ground truth layout (20 mm grid):")
    for name, typ, gx, gy, row, col in GROUND_TRUTH:
        print(f"    {name:<7}  {typ:<5}  X={gx:.3f}  Y={gy:.3f}  [{row} {col}]")
    print()
    if args.skip_phase1:
        print("  Phase 1: SKIPPED (--skip-phase1)  — using existing H")
    else:
        print("  Phase 1: Homography calibration (5×5 grid, Y = 75–125 mm)")
    print(f"  Phase 2: {N_TRIALS} validation trials (raw — no perspective correction)")
    print("  Phase 3: Linear correction fit")
    print("  Phase 4: CSV output + apply prompt")
    print()
    input("  Press Enter to begin …\n")

    mpx, mpy = load_mm_per_pixel(MM_PER_PIXEL_TXT_PATH)
    ser      = open_serial()
    h_stats  = None

    try:
        # ── Phase 1 ──────────────────────────────────────────────────────
        if not args.skip_phase1:
            ok, h_stats = phase1_calibrate_homography(ser)
            if not ok:
                print("\nPhase 1 failed — aborting.")
                return
            # Reload H in the vision module so Phase 2 uses the new matrix
            reload_homography()
            print("  H matrix reloaded in vision module.")
        else:
            # Read existing H stats for the CSV header
            try:
                npz = np.load(HOMOGRAPHY_NPZ_PATH)
                if "rms_error" in npz:
                    h_stats = {
                        "rms": float(npz["rms_error"][0]),
                        "max": float(npz["max_error"][0]),
                        "n":   int(npz["n_points"][0]),
                    }
                    print(f"  Existing H: RMS={h_stats['rms']:.4f} mm  "
                          f"Max={h_stats['max']:.4f} mm  "
                          f"({h_stats['n']} points)")
            except Exception:
                pass

        # ── Phase 2 ──────────────────────────────────────────────────────
        records = phase2_validate(ser, mpx, mpy)

    finally:
        ser.close()
        print("\nSerial closed.")

    # ── Phase 3 ──────────────────────────────────────────────────────────
    correction = phase3_fit_correction(records)

    # ── Phase 4 prompt ────────────────────────────────────────────────────
    applied = False
    if correction is not None:
        print()
        ans = input("  Apply perspective correction? (y/n): ").strip().lower()
        if ans == "y":
            np.savez(
                PERSPECTIVE_NPZ_PATH,
                scale_x=np.float64(correction["scale_x"]),
                offset_x=np.float64(correction["offset_x"]),
                scale_y=np.float64(correction["scale_y"]),
                offset_y=np.float64(correction["offset_y"]),
            )
            print(f"  Correction saved → {PERSPECTIVE_NPZ_PATH}")
            applied = True
        else:
            print("  Correction NOT applied — suggested values in CSV only.")

    # ── Write CSV ─────────────────────────────────────────────────────────
    csv_path = phase4_write_csv(records, correction, h_stats, ts, applied)

    # ── Console summary ───────────────────────────────────────────────────
    print()
    print("=" * 62)
    print("  VISUAL CALIBRATION COMPLETE")
    print("=" * 62)
    print()
    if h_stats:
        print(f"  H calibration:  RMS={h_stats['rms']:.3f} mm  "
              f"Max={h_stats['max']:.3f} mm  ({h_stats['n']} points)")

    print()
    print(f"  {'Op':<8} {'Position':<16} {'Mean err X':>11} {'Mean err Y':>11}"
          f" {'RMS combined':>13}")
    print(f"  {'─' * 62}")
    for name, typ, gx, gy, row, col in GROUND_TRUTH:
        valid = [records[(name, t)]
                 for t in range(1, N_TRIALS + 1)
                 if records.get((name, t)) is not None]
        pos = f"{row} {col}"
        if not valid:
            print(f"  {name:<8} {pos:<16} {'–':>11} {'–':>11} {'–':>13}")
        else:
            ex = [r["err_x"]    for r in valid]
            ey = [r["err_y"]    for r in valid]
            ev = [r["combined"] for r in valid]
            print(f"  {name:<8} {pos:<16} "
                  f"{_mean(ex):>+11.3f} {_mean(ey):>+11.3f} "
                  f"{_rms(ev):>13.3f}")

    print()
    if correction:
        print("  Perspective correction fit:")
        print(f"    X:  scale={correction['scale_x']:.5f}  "
              f"offset={correction['offset_x']:+.5f}  R²={correction['r2_x']:.3f}")
        print(f"    Y:  scale={correction['scale_y']:.5f}  "
              f"offset={correction['offset_y']:+.5f}  R²={correction['r2_y']:.3f}")
        print()
        if applied:
            print("  Correction APPLIED → output/perspective_correction.npz")
            print("  All future detections will use it automatically.")
            print("  Run vision_validation_v2.py to confirm the improvement.")
        else:
            print("  Correction NOT applied (saved in CSV as suggestion).")
    else:
        print("  Correction fit failed — not enough valid detections.")

    print()
    print(f"  CSV report → {csv_path}")
    print("=" * 62)
    print()


if __name__ == "__main__":
    main()
