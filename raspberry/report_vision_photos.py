"""
report_vision_photos.py

Generates clear, report-quality annotated vision photos for CAM1.

If photos from a recent run already exist in VISION_CURRENT_RUN_DIR they are
used directly.  Otherwise the machine is homed, driven to CAM1, the first
red square is detected and centred on, and all images are captured live.

Output folder: raspberry/report_vision_photos/

  1_sweep_detection.jpg    -- initial sweep image: bounding box, crosshairs, arrow
  2_center_iter_N.jpg      -- one photo per centering iteration
  3_screws_detected.jpg    -- screw / press detection with enlarged annotations

Usage:
    python report_vision_photos.py
"""

import math
import os
import sys
import glob
import shutil
import time
import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from camera import undistort_image, load_mm_per_pixel, capture_image
from config import (
    CALIBRATION_NPZ_PATH,
    MM_PER_PIXEL_TXT_PATH,
    VISION_CURRENT_RUN_DIR,
)
from transform import (
    load_homography, load_affine,
    pixel_to_machine_H, pixel_to_machine_T, pixel_to_machine_simple,
)
from vision import detect_screws  # used in _run_live for the live-capture path

# ── Report visual constants (shared with vision.py via vision_draw) ──────────
from vision_draw import (
    MARKER_SIZE, MARKER_THICKNESS, RECT_THICKNESS,
    ARROW_THICKNESS, TIP_LENGTH, FONT, FONT_SCALE, FONT_THICKNESS,
    draw_text_bg, put_label, put_bottom_label,
)

# ── Machine / CAM1 constants ─────────────────────────────────────────────────
CAM1_NAME       = "cam1"
CAM1_X, CAM1_Y = 95.0, 0.0
CENTER_TOL_MM   = 0.5
MAX_ITERS       = 4

# ── Red detection kernel (same thresholds as vision.py) ─────────────────────
_KERNEL = np.ones((15, 15), np.uint8)

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "report_vision_photos")


# ── Transform helpers ────────────────────────────────────────────────────────
def _load_transforms():
    return load_homography(), load_affine()


def _px_to_machine(px, py, img_cx, img_cy, H, T, mm_x, mm_y):
    if H is not None:
        mx_f, my_f = pixel_to_machine_H(px, py, H)
        mx_c, my_c = pixel_to_machine_H(img_cx, img_cy, H)
        return -(mx_f - mx_c), -(my_f - my_c)
    if T is not None:
        return pixel_to_machine_T(px, py, img_cx, img_cy, T)
    return pixel_to_machine_simple(px, py, img_cx, img_cy, mm_x, mm_y)


# ── Red square detection ─────────────────────────────────────────────────────
def _detect_largest_red_square(img):
    """Returns (rx, ry, rw, rh, sq_cx, sq_cy) or None."""
    hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0,   120, 70]), np.array([10,  255, 255])),
        cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255])),
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  _KERNEL)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    cnt = max(cnts, key=cv2.contourArea)
    rx, ry, rw, rh = cv2.boundingRect(cnt)
    return rx, ry, rw, rh, rx + rw / 2.0, ry + rh / 2.0


# ── Annotate one image (sweep or centering) ──────────────────────────────────
def _annotate(image_path, out_path, H, T, mm_x, mm_y, iter_label=""):
    img = cv2.imread(image_path)
    if img is None:
        print(f"  [SKIP] Cannot read {image_path}")
        return False

    img = undistort_image(img, CALIBRATION_NPZ_PATH)
    h, w = img.shape[:2]
    img_cx, img_cy = w / 2.0, h / 2.0

    det = _detect_largest_red_square(img)
    if det is None:
        print(f"  [SKIP] No red square detected in {image_path}")
        return False

    rx, ry, rw, rh, sq_cx, sq_cy = det
    dx, dy = _px_to_machine(sq_cx, sq_cy, img_cx, img_cy, H, T, mm_x, mm_y)
    vec_mm = math.sqrt(dx**2 + dy**2)

    out = img.copy()
    cxf,  cyf  = int(img_cx), int(img_cy)
    sqxf, sqyf = int(sq_cx),  int(sq_cy)

    # Bounding box around detected square
    cv2.rectangle(out, (rx, ry), (rx + rw, ry + rh), (0, 120, 255), RECT_THICKNESS,
                  lineType=cv2.LINE_AA)

    # Image-centre crosshair (cyan) — label goes away from square centre
    cv2.drawMarker(out, (cxf, cyf), (0, 255, 255),
                   cv2.MARKER_CROSS, MARKER_SIZE, MARKER_THICKNESS, cv2.LINE_AA)
    put_label(out, "image centre", cxf, cyf, sqxf, sqyf, (0, 255, 255))

    # Square-centre crosshair (magenta) — label goes away from image centre
    cv2.drawMarker(out, (sqxf, sqyf), (255, 0, 255),
                   cv2.MARKER_CROSS, MARKER_SIZE, MARKER_THICKNESS, cv2.LINE_AA)
    put_label(out, "square centre", sqxf, sqyf, cxf, cyf, (255, 0, 255))

    # Arrow from image centre → square centre
    cv2.arrowedLine(out, (cxf, cyf), (sqxf, sqyf),
                    (0, 165, 255), ARROW_THICKNESS, tipLength=TIP_LENGTH,
                    line_type=cv2.LINE_AA)

    # Offset measurement label — centered at 80 % height like a subtitle
    prefix = f"{iter_label}   " if iter_label else ""
    offset_text = f"{prefix}dx={dx:+.2f}  dy={dy:+.2f}  |d|={vec_mm:.2f} mm"
    put_bottom_label(out, offset_text, (0, 165, 255), y_frac=0.82)

    cv2.imwrite(out_path, out)
    print(f"  Saved: {out_path}")
    return True


# ── Screw/press detection photo at full report scale ──────────────────────────
def _draw_screws_report(image_path, out_path, H, T, mm_x, mm_y):
    """
    Detect screws/press-buttons and draw ALL annotations at the same report-scale
    constants used by the centering photos (MARKER_SIZE, FONT_SCALE, etc.).
    Never falls back to a pre-annotated image — always generates fresh output.
    """
    from vision import classify_screw, is_screw_present

    img = cv2.imread(image_path)
    if img is None:
        print(f"  [SKIP] Cannot read {image_path}")
        return False

    img = undistort_image(img, CALIBRATION_NPZ_PATH)
    h, w = img.shape[:2]
    img_cx, img_cy = w / 2.0, h / 2.0

    # ── Detect red square ────────────────────────────────────────────────────
    det = _detect_largest_red_square(img)
    if det is None:
        print(f"  [SKIP] No red square found in {image_path}")
        return False
    rx, ry, rw, rh, red_cx, red_cy = det
    red_dx, red_dy = _px_to_machine(red_cx, red_cy, img_cx, img_cy, H, T, mm_x, mm_y)

    # ── Detect screws inside the red square ──────────────────────────────────
    margin = 90
    x1 = max(0, rx + margin)
    y1 = max(0, ry + margin)
    x2 = min(w, rx + rw - margin)
    y2 = min(h, ry + rh - margin)
    roi = img[y1:y2, x1:x2]

    roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    roi_red = cv2.bitwise_or(
        cv2.inRange(roi_hsv, np.array([0,   120, 70]), np.array([10,  255, 255])),
        cv2.inRange(roi_hsv, np.array([170, 120, 70]), np.array([180, 255, 255])),
    )
    thresh = cv2.bitwise_not(roi_red)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN,   np.ones((20, 20), np.uint8))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE,  np.ones((10, 10), np.uint8))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_ERODE,  np.ones((15, 15), np.uint8))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_DILATE, np.ones((15, 15), np.uint8))
    all_cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    MIN_AREA, MAX_AREA, MIN_CIRC = 2000, 150000, 0.75
    screw_cnts = []
    for cnt in all_cnts:
        area = cv2.contourArea(cnt)
        if not (MIN_AREA <= area <= MAX_AREA):
            continue
        perim = cv2.arcLength(cnt, True)
        if perim == 0:
            continue
        if (4 * np.pi * area) / (perim ** 2) >= MIN_CIRC:
            screw_cnts.append(cnt)

    # Find centre mounting post (circle closest to centroid of all circles)
    cx_all, cy_all = [], []
    for cnt in screw_cnts:
        (ccx, ccy), _ = cv2.minEnclosingCircle(cnt)
        cx_all.append(x1 + int(ccx))
        cy_all.append(y1 + int(ccy))
    pattern_cx = float(np.mean(cx_all)) if cx_all else red_cx
    pattern_cy = float(np.mean(cy_all)) if cy_all else red_cy

    ignore_idx, ignore_dist = None, float("inf")
    for i, cnt in enumerate(screw_cnts):
        (ccx, ccy), _ = cv2.minEnclosingCircle(cnt)
        d = math.sqrt((x1 + ccx - pattern_cx)**2 + (y1 + ccy - pattern_cy)**2)
        if d < ignore_dist:
            ignore_dist, ignore_idx = d, i
    if ignore_idx is not None and ignore_dist > 100:
        ignore_idx = None

    # ── Compute mm/px scale for diameter ────────────────────────────────────
    if H is not None:
        import cv2 as _cv2
        pt1 = np.array([[[float(img_cx),     float(img_cy)]]], dtype=np.float32)
        pt2 = np.array([[[float(img_cx) + 1, float(img_cy)]]], dtype=np.float32)
        m1  = _cv2.perspectiveTransform(pt1, H)[0, 0]
        m2  = _cv2.perspectiveTransform(pt2, H)[0, 0]
        avg_mm_per_px = float(np.linalg.norm(m2 - m1))
    else:
        avg_mm_per_px = ((mm_x or 0) + (mm_y or 0)) / 2

    # ── Draw ─────────────────────────────────────────────────────────────────
    out = img.copy()
    cxf,  cyf  = int(img_cx), int(img_cy)
    rcxf, rcyf = int(red_cx), int(red_cy)

    # ROI box
    cv2.rectangle(out, (x1, y1), (x2, y2), (255, 60, 60), RECT_THICKNESS, cv2.LINE_AA)

    # Image-centre crosshair (cyan) — label away from red centre
    cv2.drawMarker(out, (cxf, cyf), (0, 255, 255),
                   cv2.MARKER_CROSS, MARKER_SIZE, MARKER_THICKNESS, cv2.LINE_AA)
    put_label(out, "image centre", cxf, cyf, rcxf, rcyf, (0, 255, 255))

    # Red-square centre crosshair (magenta) — label away from image centre
    cv2.drawMarker(out, (rcxf, rcyf), (255, 0, 255),
                   cv2.MARKER_CROSS, MARKER_SIZE, MARKER_THICKNESS, cv2.LINE_AA)
    put_label(out, "red centre", rcxf, rcyf, cxf, cyf, (255, 0, 255))

    # Arrow image centre → red centre
    cv2.arrowedLine(out, (cxf, cyf), (rcxf, rcyf),
                    (255, 0, 255), ARROW_THICKNESS, tipLength=TIP_LENGTH,
                    line_type=cv2.LINE_AA)

    # Offset measurement — bottom centred
    vec_mm = math.sqrt(red_dx**2 + red_dy**2)
    put_bottom_label(out,
                     f"dx={red_dx:+.2f}  dy={red_dy:+.2f}  |d|={vec_mm:.2f} mm",
                     (255, 0, 255), y_frac=0.82)

    # Each detected hole
    screw_num = 0
    for i, cnt in enumerate(screw_cnts):
        (ccx, ccy), crad = cv2.minEnclosingCircle(cnt)
        cx  = x1 + int(ccx)
        cy  = y1 + int(ccy)
        rad = int(crad)

        if i == ignore_idx:
            cv2.circle(out, (cx, cy), rad, (128, 128, 128), RECT_THICKNESS, cv2.LINE_AA)
            draw_text_bg(out, "ignored", cx + rad + 20, cy, (128, 128, 128))
            continue

        screw_num += 1
        diameter_mm = rad * 2 * avg_mm_per_px
        screw_type  = classify_screw(diameter_mm)
        has_press   = is_screw_present(roi, int(ccx), int(ccy), rad)
        s_dx, s_dy  = _px_to_machine(cx, cy, img_cx, img_cy, H, T, mm_x, mm_y)
        color = (0, 255, 0) if has_press else (255, 100, 0)

        # Line from image centre to hole
        cv2.line(out, (cxf, cyf), (cx, cy), (180, 180, 180),
                 max(1, ARROW_THICKNESS // 4), cv2.LINE_AA)

        # Hole circle + centre dot
        cv2.circle(out, (cx, cy), rad, color, RECT_THICKNESS, cv2.LINE_AA)
        cv2.circle(out, (cx, cy), MARKER_THICKNESS, (0, 0, 220), cv2.FILLED, cv2.LINE_AA)

        if has_press:
            # ── Press operation ──────────────────────────────────────────────
            # No screw type or diameter shown — it's a press, not a screw.
            # Stack EVERYTHING (PRESS + number + offset) above the upper press
            # button and below the lower press button. Nothing goes to the sides.
            info_text  = f"#{screw_num}   dx={s_dx:+.1f}  dy={s_dy:+.1f} mm"
            press_text = "PRESS"
            (iw, ih), _ = cv2.getTextSize(info_text,  FONT, FONT_SCALE, FONT_THICKNESS)
            (pw, ph), _ = cv2.getTextSize(press_text, FONT, FONT_SCALE, FONT_THICKNESS)
            gap = 24
            ix = max(8, min(w - iw - 8, cx - iw // 2))
            px = max(8, min(w - pw - 8, cx - pw // 2))
            if cy < img_cy:
                # Upper press: PRESS (top) → info → circle
                info_y  = cy - rad - 20
                press_y = info_y - ih - gap
            else:
                # Lower press: circle → info → PRESS (bottom)
                info_y  = cy + rad + ih + 20
                press_y = info_y + ph + gap
            info_y  = max(ih + 8, min(h - 8, info_y))
            press_y = max(ph + 8, min(h - 8, press_y))
            draw_text_bg(out, info_text,  ix, info_y,  color)
            draw_text_bg(out, press_text, px, press_y, (0, 255, 255))
        else:
            # ── Regular screw ────────────────────────────────────────────────
            # Left-side screws: text to the LEFT  (right edge at circle edge)
            # Right-side screws: text to the RIGHT (left  edge at circle edge)
            type_text = f"#{screw_num} {screw_type} ({diameter_mm:.1f}mm)"
            off_text  = f"dx={s_dx:+.1f}  dy={s_dy:+.1f} mm"
            (t1w, t1h), _ = cv2.getTextSize(type_text, FONT, FONT_SCALE, FONT_THICKNESS)
            (t2w, _),   _ = cv2.getTextSize(off_text,  FONT, FONT_SCALE, FONT_THICKNESS)
            label_w = max(t1w, t2w)
            if cx < img_cx:
                lx = cx - rad - 20 - label_w      # right edge of text at left of circle
            else:
                lx = cx + rad + 20                 # left  edge of text at right of circle
            lx = max(8, min(w - label_w - 8, lx))
            draw_text_bg(out, type_text, lx, cy,            color)
            draw_text_bg(out, off_text,  lx, cy + t1h + 30, (200, 200, 200))

    cv2.imwrite(out_path, out)
    print(f"  Saved: {out_path}")
    return True


# ── Live capture (machine movement) ─────────────────────────────────────────
def _run_live(ser, mm_x, mm_y):
    """Home → CAM1 → sweep → centre on first square → detect screws."""
    from serial_comm import send_command, wait_for_message
    from vision import find_all_red_squares, find_red_square_offset

    os.makedirs(VISION_CURRENT_RUN_DIR, exist_ok=True)

    print("\n-- Homing...")
    send_command(ser, "h")
    wait_for_message(ser, "SYSTEM IS HOMED", timeout=120)

    print("\n-- Moving to CAM1...")
    send_command(ser, CAM1_NAME)
    wait_for_message(ser, "SYSTEM IS AT CAM1", timeout=60)

    sweep_path = os.path.join(VISION_CURRENT_RUN_DIR, "sweep_cam1.jpg")
    capture_image(sweep_path)

    squares = find_all_red_squares(sweep_path, mm_x, mm_y)
    if not squares:
        print("  No red squares found during sweep.")
        return sweep_path, [], None, None

    dx0, dy0 = squares[0]
    approx_x, approx_y = CAM1_X + dx0, CAM1_Y + dy0
    print(f"\n-- Moving to first square at (~{approx_x:.2f}, ~{approx_y:.2f})...")
    send_command(ser, f"to {approx_x:.3f} {approx_y:.3f} 0.000")
    wait_for_message(ser, "SYSTEM IS AT POSITION", timeout=60)

    # Iterative centering
    iter_paths = []
    machine_x, machine_y = approx_x, approx_y
    for it in range(MAX_ITERS):
        iter_img = os.path.join(VISION_CURRENT_RUN_DIR, f"centre_sq1_iter{it}.jpg")
        capture_image(iter_img)
        iter_paths.append(iter_img)

        dx, dy = find_red_square_offset(iter_img, mm_x, mm_y)
        print(f"  Iter {it}: dx={dx:+.3f}  dy={dy:+.3f} mm")
        if abs(dx) <= CENTER_TOL_MM and abs(dy) <= CENTER_TOL_MM:
            print("  Centred.")
            break
        send_command(ser, f"x {dx:.3f}")
        wait_for_message(ser, "SYSTEM IS AT X", timeout=60)
        send_command(ser, f"y {dy:.3f}")
        wait_for_message(ser, "SYSTEM IS AT Y", timeout=60)
        time.sleep(0.15)
        machine_x += dx
        machine_y += dy

    # Final screw detection
    detect_path = os.path.join(VISION_CURRENT_RUN_DIR, "detect_sq1.jpg")
    debug_path  = os.path.join(VISION_CURRENT_RUN_DIR, "debug_sq1.jpg")
    capture_image(detect_path)
    detect_screws(detect_path, debug_path, mm_x, mm_y)

    return sweep_path, iter_paths, detect_path, debug_path


# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    print("=== Report Vision Photos ===")
    os.makedirs(OUT_DIR, exist_ok=True)

    H, T = _load_transforms()
    mm_x = mm_y = None
    if os.path.exists(MM_PER_PIXEL_TXT_PATH):
        try:
            mm_x, mm_y = load_mm_per_pixel(MM_PER_PIXEL_TXT_PATH)
        except Exception as exc:
            print(f"  Warning: could not load mm_per_pixel: {exc}")

    # ── Check for existing photos from a recent run ──────────────────────────
    sweep_path  = os.path.join(VISION_CURRENT_RUN_DIR, "sweep_cam1.jpg")
    iter_paths  = sorted(glob.glob(os.path.join(VISION_CURRENT_RUN_DIR, "centre_sq1_iter*.jpg")))
    detect_path = os.path.join(VISION_CURRENT_RUN_DIR, "detect_sq1.jpg")
    debug_path  = os.path.join(VISION_CURRENT_RUN_DIR, "debug_sq1.jpg")

    have_existing = os.path.exists(sweep_path)

    if not have_existing:
        print("\nNo existing vision photos found — connecting to machine for live capture...")
        try:
            from serial_comm import open_serial
            ser = open_serial()
        except Exception as exc:
            print(f"\n  ERROR: Could not open serial port: {exc}")
            print("  Either run main.py first to capture vision photos,")
            print("  or connect the machine and re-run this script.")
            return

        try:
            sweep_path, iter_paths, detect_path, debug_path = _run_live(ser, mm_x, mm_y)
        finally:
            ser.close()
    else:
        print(f"\nUsing existing photos from:\n  {VISION_CURRENT_RUN_DIR}")

    # ── 1. Sweep detection photo ─────────────────────────────────────────────
    if sweep_path and os.path.exists(sweep_path):
        print(f"\n-- [1/3] Sweep detection")
        _annotate(sweep_path,
                  os.path.join(OUT_DIR, "1_sweep_detection.jpg"),
                  H, T, mm_x, mm_y)

    # ── 2. Centering iteration photos ────────────────────────────────────────
    if not iter_paths:
        if sweep_path and os.path.exists(sweep_path):
            print("\n  No iteration images found — using sweep image as iter 0.")
            iter_paths = [sweep_path]

    for i, img_path in enumerate(iter_paths):
        if os.path.exists(img_path):
            print(f"\n-- [2/3] Centering iteration {i}")
            _annotate(img_path,
                      os.path.join(OUT_DIR, f"2_center_iter_{i}.jpg"),
                      H, T, mm_x, mm_y,
                      iter_label=f"iter {i}")

    # ── 3. Screw / press detection photo ─────────────────────────────────────
    # Use the raw detect image if available; otherwise fall back to the last
    # centering iteration image (also a raw capture of the centred square).
    screw_src = None
    if detect_path and os.path.exists(detect_path):
        screw_src = detect_path
    elif iter_paths:
        last_iter = iter_paths[-1]
        if os.path.exists(last_iter):
            screw_src = last_iter
            print(f"\n  detect_sq1.jpg not found — using last iter image for screw detection.")

    if screw_src:
        print(f"\n-- [3/3] Screw/press detection: {screw_src}")
        _draw_screws_report(screw_src,
                            os.path.join(OUT_DIR, "3_screws_detected.jpg"),
                            H, T, mm_x, mm_y)
    else:
        print("\n  [3/3] No raw image available for screw detection — skipping.")

    print(f"\nDone. Report photos saved to:\n  {OUT_DIR}")


if __name__ == "__main__":
    main()
