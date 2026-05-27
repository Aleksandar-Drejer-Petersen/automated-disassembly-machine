"""
calibrate_grid.py

Automatic homography calibration — no checkerboard, no live window.

Moves the machine to a grid of positions over the fixed red square, takes a
photo at each position, detects the red square centre in pixels, then fits a
homography from the (pixel, machine) point cloud.

More grid points → better coverage of lens distortion across the image.

Usage:
    python calibrate_grid.py
    python calibrate_grid.py --dry-run   # print grid positions, don't move
"""

import argparse
import os
import time

import cv2
import numpy as np

from config import (
    CALIBRATION_NPZ_PATH,
    VISION_PHOTOS_DIR,
    MM_PER_PIXEL_TXT_PATH,
)
from camera import capture_image, undistort_image
from serial_comm import open_serial, send_command, wait_for_message, read_all_lines
from transform import HOMOGRAPHY_NPZ_PATH

os.makedirs(VISION_PHOTOS_DIR, exist_ok=True)
os.makedirs(os.path.dirname(HOMOGRAPHY_NPZ_PATH), exist_ok=True)

# ── Grid settings ─────────────────────────────────────────────────────────────
# The machine moves to CAM1 + each (dx, dy) offset so the red square appears
# at a different part of the camera image.
# Use --preview first to check all positions have the red square in frame,
# then adjust ranges until they all pass.

X_FROM  = -20.0   # mm offset from CAM1_X  →  machine X = 75–115 (covers screw positions)
X_TO    =  20.0
Y_FROM  =  75.0   # mm offset from CAM1_Y=0  →  machine Y = 75–125
Y_TO    = 125.0   # must cover the Y range where screws are actually detected

STEPS   = 5       # grid points per axis (total = STEPS × STEPS = 25 points)

CAM1_X  = 95.0
CAM1_Y  = 0.0

# Hard machine limits
X_MIN, X_MAX = 5.0,  530.0
Y_MIN, Y_MAX = 5.0,  360.0


def _grid_offsets():
    xs = np.linspace(X_FROM, X_TO, STEPS)
    ys = np.linspace(Y_FROM, Y_TO, STEPS)
    return [(float(dx), float(dy)) for dy in ys for dx in xs]


def find_red_center(image_path):
    """Return (cx, cy) pixel of red square centre, or None."""
    img = cv2.imread(image_path)
    if img is None:
        return None
    img = undistort_image(img, CALIBRATION_NPZ_PATH)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0,   120, 70]), np.array([10,  255, 255])),
        cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255])),
    )
    kernel = np.ones((15, 15), np.uint8)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    rx, ry, rw, rh = cv2.boundingRect(max(contours, key=cv2.contourArea))
    return float(rx + rw / 2), float(ry + rh / 2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true",
                        help="Print grid positions without moving or capturing")
    parser.add_argument("--preview", action="store_true",
                        help="Visit every position and take a photo but skip homography fitting")
    args = parser.parse_args()

    offsets = _grid_offsets()

    # Filter to positions within machine limits
    valid = []
    for dx, dy in offsets:
        mx = CAM1_X + dx
        my = CAM1_Y + dy
        if X_MIN <= mx <= X_MAX and Y_MIN <= my <= Y_MAX:
            valid.append((dx, dy, mx, my))
        else:
            print(f"  Skipping offset ({dx:+.0f}, {dy:+.0f}) — machine position "
                  f"({mx:.0f}, {my:.0f}) out of limits")

    print(f"\nGrid: {STEPS}×{STEPS} = {len(offsets)} points, "
          f"{len(valid)} within machine limits")

    if args.dry_run:
        print("\nDry-run positions:")
        for i, (dx, dy, mx, my) in enumerate(valid):
            print(f"  {i+1:2d}. machine=({mx:6.1f}, {my:6.1f})  offset=({dx:+.0f}, {dy:+.0f})")
        return

    ser = open_serial()

    try:
        print("\n=== Homing ===")
        send_command(ser, "h")
        wait_for_message(ser, "SYSTEM IS HOMED", timeout=120)

        print("\n=== Moving to CAM1 ===")
        send_command(ser, "cam1")
        wait_for_message(ser, "SYSTEM IS AT CAM1", timeout=60)

        pixel_pts   = []
        machine_pts = []
        failed      = []
        cur_x, cur_y = CAM1_X, CAM1_Y   # track current machine position

        for i, (dx, dy, mx, my) in enumerate(valid):
            print(f"\n[{i+1}/{len(valid)}] Moving to machine ({mx:.1f}, {my:.1f}) …")

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

            photo_path = os.path.join(
                VISION_PHOTOS_DIR, f"grid_{i+1:02d}_x{mx:.0f}_y{my:.0f}.jpg"
            )
            capture_image(photo_path)

            center = find_red_center(photo_path)
            if center is None:
                print(f"  WARNING: red square not found in photo")
                failed.append(i + 1)
            else:
                print(f"  Red square at pixel ({center[0]:.1f}, {center[1]:.1f})")

            if args.preview:
                continue   # skip recording points, just take photos

            if center is None:
                continue

            pixel_pts.append(center)
            machine_pts.append((mx, my))

    finally:
        ser.close()
        print("\nSerial closed.")

    # ── Preview mode stops here ───────────────────────────────────────────────
    if args.preview:
        print(f"\nPreview done. {len(valid) - len(failed)}/{len(valid)} photos had the red square visible.")
        if failed:
            print(f"Missing at points: {failed} — consider reducing RANGE_X/RANGE_Y.")
        print(f"Photos saved to: {VISION_PHOTOS_DIR}")
        print("Adjust RANGE_X / RANGE_Y / STEPS then run without --preview to calibrate.")
        return

    # ── Fit homography ────────────────────────────────────────────────────────
    print(f"\n=== Fitting homography from {len(pixel_pts)} points ===")

    if len(pixel_pts) < 4:
        print("ERROR: need at least 4 valid points. "
              "Check that the red square is visible from all grid positions.")
        return

    H, mask = cv2.findHomography(
        np.array(pixel_pts,   dtype=np.float32),
        np.array(machine_pts, dtype=np.float32),
        cv2.RANSAC, 1.0
    )

    if H is None:
        print("ERROR: homography solve failed.")
        return

    inliers = int(mask.sum())
    proj    = cv2.perspectiveTransform(
        np.array(pixel_pts, dtype=np.float32).reshape(-1, 1, 2), H
    ).reshape(-1, 2)
    residuals = np.linalg.norm(proj - np.array(machine_pts), axis=1)
    print(f"  {inliers}/{len(pixel_pts)} inliers")
    print(f"  RMS error:  {residuals.mean():.3f} mm")
    print(f"  Max error:  {residuals.max():.3f} mm")

    if residuals.mean() > 2.0:
        print("WARNING: high reprojection error. "
              "Check that the red square was clearly visible in the photos.")

    np.savez(HOMOGRAPHY_NPZ_PATH, H=H,
             rms_error=np.array([residuals.mean()]),
             max_error=np.array([residuals.max()]),
             n_points=np.array([len(pixel_pts)]))
    print(f"\nHomography saved → {HOMOGRAPHY_NPZ_PATH}")

    # Compute mm_per_pixel by evaluating the H Jacobian at the image centre
    # (H[0,0] alone is not mm/pixel for a general homography)
    sample_img = cv2.imread(os.path.join(
        VISION_PHOTOS_DIR,
        f"grid_01_x{valid[0][2]:.0f}_y{valid[0][3]:.0f}.jpg"
    ))
    if sample_img is not None:
        ih, iw = sample_img.shape[:2]
    else:
        iw, ih = 4608, 2592   # camera native resolution fallback
    cx_img, cy_img = iw / 2.0, ih / 2.0
    pt_c = np.array([[[cx_img,     cy_img]]], dtype=np.float32)
    pt_x = np.array([[[cx_img + 1, cy_img]]], dtype=np.float32)
    pt_y = np.array([[[cx_img,     cy_img + 1]]], dtype=np.float32)
    mc   = cv2.perspectiveTransform(pt_c, H)[0, 0]
    mx_  = cv2.perspectiveTransform(pt_x, H)[0, 0]
    my_  = cv2.perspectiveTransform(pt_y, H)[0, 0]
    mm_per_pixel_x = float(np.linalg.norm(mx_ - mc))
    mm_per_pixel_y = float(np.linalg.norm(my_ - mc))
    os.makedirs(os.path.dirname(MM_PER_PIXEL_TXT_PATH), exist_ok=True)
    with open(MM_PER_PIXEL_TXT_PATH, "w") as f:
        f.write(f"mm_per_pixel_x: {mm_per_pixel_x:.6f}\n")
        f.write(f"mm_per_pixel_y: {mm_per_pixel_y:.6f}\n")
        f.write(f"source: calibrate_grid.py ({len(pixel_pts)} points)\n")
    print(f"Scale fallback saved → {MM_PER_PIXEL_TXT_PATH}")

    if failed:
        print(f"\nSkipped {len(failed)} points (red square not found): {failed}")
        print("Inspect the corresponding photos in vision_photos/ to diagnose.")

    print("\nDone. Run vision_test.py --no-move to verify detection accuracy.")


if __name__ == "__main__":
    main()
