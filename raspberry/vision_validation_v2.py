"""
vision_validation_v2.py

Extended vision accuracy validation — tests all 8 operations on the item
(6 screws + 2 presses) in a single run.

Because all 8 operations live on the same red-square component, each trial
does ONE vision cycle (cam1 → centre on square → detect) and records a
detected position for all 8 operations at once.  This lets us compare
per-operation biases across the image and distinguish a uniform camera
offset from position-dependent lens warp.

Ground truth layout (20 mm grid, SCREW1 = lower-right reference)
as seen from the front of the machine (Y increases to the left):

    SCREW2  PRESS1   SCREW1        X ≈  75.763   (bottom) ← lower-right
    SCREW4  ──────  SCREW3        X ≈  95.763   (middle — centre empty)
    SCREW6  PRESS2   SCREW5        X ≈ 115.763   (top)
    col Y: 125.563  105.563  85.563

Note: machine X increases going UP the component (away from SCREW1 row).

SCREW1 ground truth from POSITION command run on 2026-05-20.
Middle-centre slot is physically empty on this component.

Usage:
    python vision_validation_v2.py
    python vision_validation_v2.py --iterations 20 --output v2.csv
"""

import argparse
import csv
import math
import os
import sys
from datetime import datetime

import cv2
import numpy as np

from config import (
    MM_PER_PIXEL_TXT_PATH,
    CAMERA_OFFSET_X_MM, CAMERA_OFFSET_Y_MM,
    VISION_CURRENT_RUN_DIR,
    CAM1_X, CAM1_Y,
)
from camera import capture_image, load_mm_per_pixel
from vision import find_red_square_offset, find_all_red_squares, detect_screws
from serial_comm import open_serial, send_command, wait_for_message
from transform import HOMOGRAPHY_NPZ_PATH, load_homography, apply_perspective_correction

os.makedirs(VISION_CURRENT_RUN_DIR, exist_ok=True)

# ── Ground truth positions ────────────────────────────────────────────────────
# SCREW1 measured via POSITION command (2026-05-20).
# All others derived with 20 mm grid spacing.
# Middle-centre slot (X≈95.763, Y≈105.563) is empty on this component.
#
# Machine X increases going UP the component (away from SCREW1 row).
# As seen from the front (Y increases to the left):
#   SCREW2  PRESS1   SCREW1        X = S1X        (bottom) ← SCREW1 lower-right
#   SCREW4  ──────  SCREW3        X = S1X + 20   (middle — centre empty)
#   SCREW6  PRESS2   SCREW5        X = S1X + 40   (top)
#            Y: S1Y+40  S1Y+20   S1Y

S1X = 75.763
S1Y = 85.563

# (name, op_type, abs_x, abs_y, row_label, col_label)
GROUND_TRUTH = [
    ("SCREW1", "screw", S1X,        S1Y,        "bottom", "right"),
    ("PRESS1",  "press",  S1X,        S1Y + 20,   "bottom", "centre"),
    ("SCREW2", "screw", S1X,        S1Y + 40,   "bottom", "left"),
    ("SCREW3", "screw", S1X + 20,   S1Y,        "middle", "right"),
    ("SCREW4", "screw", S1X + 20,   S1Y + 40,   "middle", "left"),
    ("SCREW5", "screw", S1X + 40,   S1Y,        "top",    "right"),
    ("PRESS2",  "press",  S1X + 40,   S1Y + 20,   "top",    "centre"),
    ("SCREW6", "screw", S1X + 40,   S1Y + 40,   "top",    "left"),
]

# Maximum distance (mm) from ground truth to accept a detection as a match
MATCH_THRESHOLD_MM  = 12.0

CENTER_TOLERANCE_MM = 0.5
MAX_CENTER_ITERS    = 4


# ── Vision helpers ────────────────────────────────────────────────────────────
def _centre_on_square(ser, approx_x, approx_y, mpx, mpy, label):
    """Iteratively centre camera on red square. Returns (final_x, final_y)."""
    mx, my = approx_x, approx_y
    for i in range(MAX_CENTER_ITERS):
        img = os.path.join(VISION_CURRENT_RUN_DIR, f"v2_{label}_c{i}.jpg")
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


# ── Stats ─────────────────────────────────────────────────────────────────────
def _mean(v): return sum(v) / len(v) if v else 0.0
def _rms(v):  return math.sqrt(sum(x**2 for x in v) / len(v)) if v else 0.0
def _std(v):
    if not v: return 0.0
    m = _mean(v)
    return math.sqrt(sum((x - m)**2 for x in v) / len(v))


# ── H refinement ─────────────────────────────────────────────────────────────

def _refine_homography(records, ground_truth, N, op_stats):
    """
    Use the mean per-operation errors from this validation run to compute
    a refined homography.

    For each GT operation we know:
      - mean pixel position (cx, cy) of the detected feature
      - mean rs_x, rs_y (red-square machine position for that trial)
      - gt_x, gt_y (true machine position)

    From the mapping used in vision.py:
      abs_x = rs_x + H(cx,cy).x - H(img_cx,img_cy).x - CAMERA_OFFSET_X_MM
      abs_y = rs_y + H(img_cx,img_cy).y - H(cx,cy).y - CAMERA_OFFSET_Y_MM

    Solving for H(cx,cy) that would give abs = gt:
      H_target_x = gt_x - rs_x + H(img_cx,img_cy).x + CAMERA_OFFSET_X_MM
      H_target_y = rs_y + H(img_cx,img_cy).y - CAMERA_OFFSET_Y_MM - gt_y

    We use these 8 (pixel, H_target) pairs to re-solve H.
    """
    H = load_homography()
    if H is None:
        print("\n  H refinement skipped — no homography file found.")
        return

    # Image centre — must match the resolution used by detect_screws()
    # (after undistortion the size can vary; use the value from op_stats records)
    # Grab from first valid record
    img_cx = img_cy = None
    for name, *_ in ground_truth:
        for t in range(1, N + 1):
            r = records.get((name, t))
            if r and "pixel_cx" in r:
                # Recover image centre from H: at image centre H gives the
                # "no-delta" machine position.  We don't store it directly,
                # but we know typical camera resolution.
                img_cx = 2304.0   # half of 4608 (native width after undistort ≈ this)
                img_cy = 1296.0   # half of 2592
                break
        if img_cx is not None:
            break

    if img_cx is None:
        print("\n  H refinement skipped — no pixel data found (were detections logged?).")
        return

    # H output at image centre
    pt_c = np.array([[[img_cx, img_cy]]], dtype=np.float32)
    hc   = cv2.perspectiveTransform(pt_c, H)[0, 0]
    hcx, hcy = float(hc[0]), float(hc[1])

    pixel_pts  = []
    target_pts = []

    for name, op_type, gt_x, gt_y, row_lbl, col_lbl in ground_truth:
        s = op_stats.get(name)
        if s is None:
            continue

        # Mean pixel position across all valid trials
        valid = [records[(name, t)] for t in range(1, N + 1)
                 if records.get((name, t)) is not None and "pixel_cx" in records[(name, t)]]
        if not valid:
            continue

        mean_cx  = _mean([r["pixel_cx"] for r in valid])
        mean_cy  = _mean([r["pixel_cy"] for r in valid])
        mean_rsx = _mean([r["rs_x"]     for r in valid])
        mean_rsy = _mean([r["rs_y"]     for r in valid])

        # Target H output at (mean_cx, mean_cy) to achieve gt position
        h_target_x = gt_x - mean_rsx + hcx + CAMERA_OFFSET_X_MM
        h_target_y = mean_rsy + hcy - CAMERA_OFFSET_Y_MM - gt_y

        pixel_pts.append([mean_cx, mean_cy])
        target_pts.append([h_target_x, h_target_y])

        print(f"  {name:<7}: pixel=({mean_cx:.0f},{mean_cy:.0f})  "
              f"H_target=({h_target_x:.2f},{h_target_y:.2f})")

    if len(pixel_pts) < 4:
        print(f"\n  H refinement skipped — only {len(pixel_pts)} valid points (need ≥ 4).")
        return

    print(f"\n  Solving refined H from {len(pixel_pts)} validation points …")
    H_new, mask = cv2.findHomography(
        np.array(pixel_pts,  dtype=np.float32),
        np.array(target_pts, dtype=np.float32),
        cv2.RANSAC, 0.5
    )

    if H_new is None:
        print("  H refinement failed — homography solve returned None.")
        return

    proj      = cv2.perspectiveTransform(
        np.array(pixel_pts, dtype=np.float32).reshape(-1, 1, 2), H_new
    ).reshape(-1, 2)
    residuals = np.linalg.norm(proj - np.array(target_pts), axis=1)
    inliers   = int(mask.sum())

    print(f"  Refined H: {inliers}/{len(pixel_pts)} inliers  "
          f"RMS={residuals.mean():.4f} mm  Max={residuals.max():.4f} mm")

    # Save — overwrite the existing homography
    np.savez(HOMOGRAPHY_NPZ_PATH, H=H_new,
             rms_error=np.array([residuals.mean()]),
             max_error=np.array([residuals.max()]),
             n_points=np.array([len(pixel_pts)]))
    print(f"  Refined H saved → {HOMOGRAPHY_NPZ_PATH}")
    print("  Re-run validation to confirm improvement.")


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Vision accuracy validation v2 — all 8 operations."
    )
    parser.add_argument("--iterations", type=int, default=10,
                        help="Number of vision cycles (default: 10)")
    parser.add_argument("--output", default=None,
                        help="CSV output path (default: vision_validation_v2_<ts>.csv)")
    args = parser.parse_args()

    ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = args.output or f"vision_validation_v2_{ts}.csv"
    N        = args.iterations

    # Component centre — used to choose the right red square from the sweep
    comp_cx = _mean([gt[2] for gt in GROUND_TRUTH])
    comp_cy = _mean([gt[3] for gt in GROUND_TRUTH])

    print()
    print("=" * 62)
    print("  Vision Accuracy Validation  v2  —  All 8 Operations")
    print("=" * 62)
    print()
    print("  Ground truth layout (20 mm grid)  — as seen from the front:")
    print()
    print("    SCREW2  PRESS1   SCREW1      X ≈  75.8   (bottom) ← lower-right")
    print("    SCREW4  ──────  SCREW3      X ≈  95.8   (middle — centre empty)")
    print("    SCREW6  PRESS2   SCREW5      X ≈ 115.8   (top)")
    print("    col Y: 125.6   105.6   85.6")
    print()
    for name, typ, gx, gy, row, col in GROUND_TRUTH:
        print(f"    {name:<7}  {typ:<5}  X={gx:.3f}  Y={gy:.3f}  [{row} {col}]")
    print()
    print(f"  Iterations : {N}")
    print(f"  Output CSV : {csv_path}")
    print()
    input("  Press Enter to home and begin …\n")

    # ── Load homography calibration stats ─────────────────────────────────────
    h_stats = {"rms": None, "max": None, "n": None}
    try:
        npz = np.load(HOMOGRAPHY_NPZ_PATH)
        if "rms_error" in npz:
            h_stats["rms"] = float(npz["rms_error"][0])
            h_stats["max"] = float(npz["max_error"][0])
            h_stats["n"]   = int(npz["n_points"][0])
            print(f"  H calibration: RMS={h_stats['rms']:.4f} mm  "
                  f"Max={h_stats['max']:.4f} mm  "
                  f"({h_stats['n']} points)")
        else:
            print("  H calibration stats not stored — re-run calibrate_grid.py to capture them.")
    except Exception:
        print("  No homography file found.")

    mpx, mpy = load_mm_per_pixel(MM_PER_PIXEL_TXT_PATH)
    ser = open_serial()

    # records keyed by (op_name, trial_number)
    records = {}   # (name, trial) → dict or None

    try:
        print("=== Homing machine ===")
        send_command(ser, "h")
        wait_for_message(ser, "SYSTEM IS HOMED", timeout=120)

        for trial in range(1, N + 1):
            print(f"\n{'─' * 55}")
            print(f"  Trial {trial} / {N}")
            print(f"{'─' * 55}")

            # ── CAM1 ─────────────────────────────────────────────────────
            send_command(ser, "cam1")
            wait_for_message(ser, "SYSTEM IS AT CAM1", timeout=60)

            # ── Sweep → find red square ───────────────────────────────────
            sweep_img = os.path.join(VISION_CURRENT_RUN_DIR, f"v2_t{trial:02d}_sweep.jpg")
            capture_image(sweep_img)

            found = find_all_red_squares(sweep_img, mpx, mpy)
            abs_squares = [(CAM1_X + dx, CAM1_Y + dy) for dx, dy in found]

            if not abs_squares:
                print(f"  No red squares found — skipping trial.")
                for name, *_ in GROUND_TRUTH:
                    records[(name, trial)] = None
                continue

            # Pick the square closest to the component centre
            sq = min(abs_squares,
                     key=lambda s: math.hypot(s[0] - comp_cx, s[1] - comp_cy))
            print(f"  Red square at approx  X={sq[0]:.2f}  Y={sq[1]:.2f}")

            send_command(ser, f"to {sq[0]:.3f} {sq[1]:.3f} 0.000")
            wait_for_message(ser, "SYSTEM IS AT POSITION", timeout=60)

            # ── Centre on square ──────────────────────────────────────────
            rs_x, rs_y = _centre_on_square(ser, sq[0], sq[1], mpx, mpy, f"t{trial:02d}")
            print(f"  Square centred at  X={rs_x:.3f}  Y={rs_y:.3f}")

            # ── Detect all operations ─────────────────────────────────────
            det_img   = os.path.join(VISION_CURRENT_RUN_DIR, f"v2_t{trial:02d}_detect.jpg")
            debug_img = os.path.join(VISION_CURRENT_RUN_DIR, f"v2_t{trial:02d}_debug.jpg")
            capture_image(det_img)
            screw_results, _ = detect_screws(det_img, debug_img, mpx, mpy)

            if not screw_results:
                print(f"  No operations detected — skipping trial.")
                for name, *_ in GROUND_TRUTH:
                    records[(name, trial)] = None
                continue

            # Compute absolute machine position + perspective correction.
            # Note: camera X axis is inverted vs machine X, so machine_dx is
            # subtracted (not added) when converting to absolute machine coords.
            for s in screw_results:
                raw_x = rs_x - s["machine_dx"] - CAMERA_OFFSET_X_MM
                raw_y = rs_y + s["machine_dy"] - CAMERA_OFFSET_Y_MM
                s["abs_x"], s["abs_y"] = apply_perspective_correction(raw_x, raw_y)

            # ── Debug: show ALL detections (after correction) before matching ─
            print(f"  detect_screws returned {len(screw_results)} detection(s):")
            for s in screw_results:
                print(f"    abs=({s['abs_x']:.3f}, {s['abs_y']:.3f})  "
                      f"dx={s['machine_dx']:+.2f} dy={s['machine_dy']:+.2f}  "
                      f"type={s['screw_type']}  press={s['has_press']}")

            # ── Match each ground truth to its nearest detected op ─────────
            unmatched = list(screw_results)
            for name, op_type, gt_x, gt_y, row_lbl, col_lbl in GROUND_TRUTH:
                candidates = [
                    s for s in unmatched
                    if math.hypot(s["abs_x"] - gt_x, s["abs_y"] - gt_y) < MATCH_THRESHOLD_MM
                ]
                if not candidates:
                    print(f"    {name:<7}: NOT DETECTED")
                    records[(name, trial)] = None
                    continue

                best = min(candidates,
                           key=lambda s: math.hypot(s["abs_x"] - gt_x, s["abs_y"] - gt_y))
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

    finally:
        ser.close()
        print("\nSerial closed.")

    # ── Per-operation statistics ──────────────────────────────────────────────
    op_stats = {}
    for name, op_type, gt_x, gt_y, row_lbl, col_lbl in GROUND_TRUTH:
        valid = [records[(name, t)] for t in range(1, N+1)
                 if records.get((name, t)) is not None]
        if not valid:
            op_stats[name] = None
            continue
        ex = [r["err_x"] for r in valid]
        ey = [r["err_y"] for r in valid]
        ev = [r["combined"] for r in valid]
        op_stats[name] = {
            "n":            len(valid),
            "mean_ex":      _mean(ex),
            "mean_ey":      _mean(ey),
            "mean_abs_ex":  _mean([abs(v) for v in ex]),
            "mean_abs_ey":  _mean([abs(v) for v in ey]),
            "max_abs_ex":   max(abs(v) for v in ex),
            "max_abs_ey":   max(abs(v) for v in ey),
            "rms_ex":       _rms(ex),
            "rms_ey":       _rms(ey),
            "std_ex":       _std(ex),
            "std_ey":       _std(ey),
            "max_combined": max(ev),
            "rms_combined": _rms(ev),
        }

    # ── Warp vs offset analysis ───────────────────────────────────────────────
    valid_op_stats = [(name, op_stats[name])
                      for name, *_ in GROUND_TRUTH if op_stats[name] is not None]
    warp_analysis = None
    if valid_op_stats:
        all_mex = [s["mean_ex"] for _, s in valid_op_stats]
        all_mey = [s["mean_ey"] for _, s in valid_op_stats]
        warp_analysis = {
            "overall_mean_ex": _mean(all_mex),
            "overall_mean_ey": _mean(all_mey),
            "range_ex":        max(all_mex) - min(all_mex),
            "range_ey":        max(all_mey) - min(all_mey),
            "min_mex":         min(all_mex),
            "max_mex":         max(all_mex),
            "min_mey":         min(all_mey),
            "max_mey":         max(all_mey),
        }
        warp_x = warp_analysis["range_ex"] >= 0.3
        warp_y = warp_analysis["range_ey"] >= 0.3
        warp_analysis["warp_x"] = warp_x
        warp_analysis["warp_y"] = warp_y

    # ── Write CSV ─────────────────────────────────────────────────────────────
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)

        # ── Header ───────────────────────────────────────────────────────
        w.writerow(["Vision Accuracy Validation v2 — All 8 Operations"])
        w.writerow(["Timestamp",    ts])
        w.writerow(["Iterations",   N])
        w.writerow(["SCREW1 ref",   f"X={S1X}  Y={S1Y}  (from POSITION command 2026-05-20)"])
        w.writerow(["Grid spacing", "20 mm"])
        if h_stats["rms"] is not None:
            w.writerow(["H calibration RMS (mm)", f"{h_stats['rms']:.4f}"])
            w.writerow(["H calibration Max (mm)", f"{h_stats['max']:.4f}"])
            w.writerow(["H calibration points",   h_stats["n"]])
        else:
            w.writerow(["H calibration stats", "not available — re-run calibrate_grid.py"])
        w.writerow([])

        w.writerow(["GROUND TRUTH POSITIONS"])
        w.writerow(["name", "type", "gt_x", "gt_y", "position"])
        for name, typ, gx, gy, row, col in GROUND_TRUTH:
            w.writerow([name, typ, f"{gx:.3f}", f"{gy:.3f}", f"{row} {col}"])
        w.writerow([])

        # ── Raw data ──────────────────────────────────────────────────────
        w.writerow(["RAW DATA"])
        w.writerow(["trial", "operation", "op_type", "position",
                    "gt_x", "gt_y",
                    "detected_x", "detected_y",
                    "err_x", "err_y", "combined_error_mm",
                    "pixel_cx", "pixel_cy", "rs_x", "rs_y", "note"])

        gt_lookup = {name: (typ, gx, gy, f"{row} {col}")
                     for name, typ, gx, gy, row, col in GROUND_TRUTH}

        for trial in range(1, N + 1):
            for name, *_ in GROUND_TRUTH:
                r = records.get((name, trial))
                typ, gx, gy, pos = gt_lookup[name]
                if r is None:
                    w.writerow([trial, name, typ, pos,
                                f"{gx:.3f}", f"{gy:.3f}",
                                "–", "–", "–", "–", "–",
                                "–", "–", "–", "–", "not detected"])
                else:
                    w.writerow([trial, name, typ, pos,
                                f"{gx:.3f}", f"{gy:.3f}",
                                r["detected_x"], r["detected_y"],
                                r["err_x"], r["err_y"], r["combined"],
                                r["pixel_cx"], r["pixel_cy"],
                                round(r["rs_x"], 4), round(r["rs_y"], 4), ""])
        w.writerow([])

        # ── Per-operation statistics ──────────────────────────────────────
        w.writerow(["PER-OPERATION STATISTICS"])
        w.writerow([])
        w.writerow(["name", "position", "n",
                    "mean_err_x", "mean_err_y",
                    "mean_abs_err_x", "mean_abs_err_y",
                    "max_abs_err_x", "max_abs_err_y",
                    "rms_err_x", "rms_err_y",
                    "std_err_x", "std_err_y",
                    "max_combined_mm", "rms_combined_mm"])

        for name, typ, gx, gy, row, col in GROUND_TRUTH:
            s = op_stats[name]
            pos = f"{row} {col}"
            if s is None:
                w.writerow([name, pos, 0] + ["–"] * 12)
                continue
            w.writerow([
                name, pos, s["n"],
                f"{s['mean_ex']:+.4f}",    f"{s['mean_ey']:+.4f}",
                f"{s['mean_abs_ex']:.4f}", f"{s['mean_abs_ey']:.4f}",
                f"{s['max_abs_ex']:.4f}",  f"{s['max_abs_ey']:.4f}",
                f"{s['rms_ex']:.4f}",      f"{s['rms_ey']:.4f}",
                f"{s['std_ex']:.4f}",      f"{s['std_ey']:.4f}",
                f"{s['max_combined']:.4f}", f"{s['rms_combined']:.4f}",
            ])
        w.writerow([])

        # ── Warp vs offset analysis ───────────────────────────────────────
        w.writerow(["WARP vs OFFSET ANALYSIS"])
        w.writerow(["If mean errors are consistent across positions → pure camera offset (easy fix)."])
        w.writerow(["If mean errors vary across positions → lens warp or perspective distortion."])
        w.writerow([])

        if warp_analysis:
            wa = warp_analysis
            w.writerow(["metric", "X (mm)", "Y (mm)"])
            w.writerow(["Overall mean bias",
                        f"{wa['overall_mean_ex']:+.4f}",
                        f"{wa['overall_mean_ey']:+.4f}"])
            w.writerow(["Min per-op mean error",
                        f"{wa['min_mex']:+.4f}",
                        f"{wa['min_mey']:+.4f}"])
            w.writerow(["Max per-op mean error",
                        f"{wa['max_mex']:+.4f}",
                        f"{wa['max_mey']:+.4f}"])
            w.writerow(["Range of mean errors  (warp indicator — >0.3 mm = suspicious)",
                        f"{wa['range_ex']:.4f}",
                        f"{wa['range_ey']:.4f}"])
            w.writerow(["Verdict",
                        "WARP" if wa["warp_x"] else "uniform offset",
                        "WARP" if wa["warp_y"] else "uniform offset"])
            w.writerow([])
            if not wa["warp_x"] and not wa["warp_y"]:
                w.writerow(["Suggested correction (subtract from CAMERA_OFFSET values):"])
                w.writerow(["CAMERA_OFFSET_X_MM", f"{CAMERA_OFFSET_X_MM} → {CAMERA_OFFSET_X_MM - wa['overall_mean_ex']:.4f}"])
                w.writerow(["CAMERA_OFFSET_Y_MM", f"{CAMERA_OFFSET_Y_MM} → {CAMERA_OFFSET_Y_MM + wa['overall_mean_ey']:.4f}"])

    # ── H matrix refinement from validation errors ───────────────────────────
    _refine_homography(records, GROUND_TRUTH, N, op_stats)

    # ── Console summary ───────────────────────────────────────────────────────
    print(f"\n{'=' * 65}")
    print(f"  VISION VALIDATION v2 — PER-OPERATION SUMMARY")
    print(f"{'=' * 65}")
    print(f"  {'Op':<8} {'Position':<16} {'Mean err X':>11} {'Mean err Y':>11} {'Max |err|':>10} {'Std X':>7} {'Std Y':>7}")
    print(f"  {'─' * 64}")
    for name, typ, gx, gy, row, col in GROUND_TRUTH:
        s = op_stats[name]
        pos = f"{row} {col}"
        if s is None:
            print(f"  {name:<8} {pos:<16} {'–':>11} {'–':>11} {'–':>10} {'–':>7} {'–':>7}")
        else:
            print(f"  {name:<8} {pos:<16} "
                  f"{s['mean_ex']:>+11.3f} {s['mean_ey']:>+11.3f} "
                  f"{s['max_combined']:>10.3f} "
                  f"{s['std_ex']:>7.3f} {s['std_ey']:>7.3f}")

    if warp_analysis:
        wa = warp_analysis
        print()
        print(f"  Overall bias  :  X={wa['overall_mean_ex']:+.3f} mm   Y={wa['overall_mean_ey']:+.3f} mm")
        print(f"  Range of biases: X={wa['range_ex']:.3f} mm   Y={wa['range_ey']:.3f} mm")
        print()
        if not wa["warp_x"] and not wa["warp_y"]:
            print("  → Errors are UNIFORM across the image — pure camera offset.")
            print(f"    Adjust CAMERA_OFFSET_X_MM: {CAMERA_OFFSET_X_MM} → {CAMERA_OFFSET_X_MM - wa['overall_mean_ex']:.4f}")
            print(f"    Adjust CAMERA_OFFSET_Y_MM: {CAMERA_OFFSET_Y_MM} → {CAMERA_OFFSET_Y_MM + wa['overall_mean_ey']:.4f}")
        elif wa["warp_x"] and wa["warp_y"]:
            print("  → Errors VARY in BOTH X and Y — lens warp / perspective distortion.")
            print("    Camera intrinsic calibration likely needs to be re-run.")
        else:
            ax = "X" if wa["warp_x"] else "Y"
            print(f"  → Errors vary in {ax} but are uniform in the other axis.")
            print("    Partial warp — consider re-running camera calibration.")

    print()
    print(f"  CSV saved → {csv_path}")
    print(f"{'=' * 65}")


if __name__ == "__main__":
    main()
