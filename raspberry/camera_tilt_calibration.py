"""
camera_tilt_calibration.py

Calibrates the camera for lens distortion and perspective tilt by
capturing images of a checkerboard from multiple positions.

The machine homes, moves to the start position, then traverses a grid
pattern over the stationary checkerboard.  OpenCV calibrateCamera() is
run on every frame where all corners were detected.  The result is saved
to the path defined by CALIBRATION_NPZ_PATH in config.py — the same file
that camera.py's undistort_image() already loads automatically.

Checkerboard requirements
--------------------------
  • 11 × 18 inner corners, 5 mm squares (6 black squares high, 9 wide)
  • Placed FLAT on the workpiece surface (Z = 0)
  • Orient the board with its LONGER side (18 inner corners / 9 black squares)
    running LEFT-RIGHT (machine Y direction) and the SHORTER side (11 inner
    corners / 6 black squares) running FRONT-BACK (machine X direction).
  • Centre the board roughly under X = 300, Y = 100.

Usage
------
    python camera_tilt_calibration.py
    python camera_tilt_calibration.py --x 300 --y 100

Outputs
-------
    <CALIBRATION_NPZ_PATH>            — loaded automatically by undistort_image()
    camera_calibration.json           — human-readable copy; send to assistant
    camera_calibration_images/        — raw captured images
    camera_calibration_debug/         — annotated corner images (visual check)
"""

import argparse
import json
import os
import subprocess
import time

import cv2
import numpy as np

from config import CALIBRATION_NPZ_PATH
from serial_comm import open_serial, send_command, wait_for_message


# ── Checkerboard ─────────────────────────────────────────────────────────────
# Edit these if your printed board differs.
BOARD_COLS    = 17     # inner corners along the LONG  side (horizontal, 18 squares → 17 corners)
BOARD_ROWS    = 10     # inner corners along the SHORT side (vertical,   11 squares → 10 corners)
SQUARE_MM     = 5.0    # physical square size in mm

# ── Camera ────────────────────────────────────────────────────────────────────
# Must match the resolution used by capture_image() in camera.py
CAPTURE_W     = 4608
CAPTURE_H     = 2592

# ── Machine ───────────────────────────────────────────────────────────────────
DEFAULT_X     = 300.0   # start position — board should be centred here
DEFAULT_Y     = 100.0
Z_HEIGHT      = 0.0

# Grid offsets (mm) from the start position.
# Designed to move the board to different frame regions while keeping it
# fully visible.  Reduce if the board drifts out of frame at the extremes.
X_OFFSETS = [-10, -5, 0, 5, 10]   # 5 steps in X  (front-back)
Y_OFFSETS = [-20, -10, 0, 10, 20]  # 5 steps in Y  (left-right)
# Total: 5 × 5 = 25 positions → typically 18-22 good detections

SETTLE_TIME   = 1.0    # seconds to wait after each move before capturing

# ── Output paths ──────────────────────────────────────────────────────────────
IMAGE_DIR     = "camera_calibration_images"
DEBUG_DIR     = "camera_calibration_debug"
JSON_PATH     = "camera_calibration.json"


# ── Helpers ───────────────────────────────────────────────────────────────────

def capture(path):
    """Capture one still image using rpicam-still (matches camera.py)."""
    result = subprocess.run(
        [
            "rpicam-still",
            "-o", path,
            "--width",  str(CAPTURE_W),
            "--height", str(CAPTURE_H),
            "--nopreview",
            "--autofocus-mode", "auto",
            "--autofocus-on-capture",
        ],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"rpicam-still failed: {result.stderr.strip()}")
    return cv2.imread(path)


def find_corners(img, board_size, criteria):
    """Return (found, refined_corners, (width, height))."""
    gray  = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH |
             cv2.CALIB_CB_NORMALIZE_IMAGE  |
             cv2.CALIB_CB_FAST_CHECK)
    ok, corners = cv2.findChessboardCorners(gray, board_size, flags)
    if ok:
        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria
        )
    return ok, corners, (gray.shape[1], gray.shape[0])


def save_debug_image(img, corners, board_size, path):
    """Save a scaled debug image with corners drawn."""
    vis   = img.copy()
    cv2.drawChessboardCorners(vis, board_size, corners, True)
    scale = min(1.0, 1600 / vis.shape[1])
    vis   = cv2.resize(vis, (int(vis.shape[1] * scale),
                              int(vis.shape[0] * scale)))
    cv2.imwrite(path, vis)


def mean_reprojection_error(objpoints, imgpoints, rvecs, tvecs, K, dist):
    total = 0.0
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(
            objpoints[i], rvecs[i], tvecs[i], K, dist
        )
        total += cv2.norm(imgpoints[i], proj, cv2.NORM_L2) / len(proj)
    return total / len(objpoints)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Checkerboard camera calibration for tilt / distortion."
    )
    parser.add_argument("--x", type=float, default=DEFAULT_X,
                        help=f"Start X mm (default {DEFAULT_X})")
    parser.add_argument("--y", type=float, default=DEFAULT_Y,
                        help=f"Start Y mm (default {DEFAULT_Y})")
    args = parser.parse_args()

    os.makedirs(IMAGE_DIR, exist_ok=True)
    os.makedirs(DEBUG_DIR, exist_ok=True)
    os.makedirs(os.path.dirname(CALIBRATION_NPZ_PATH), exist_ok=True)

    board_size = (BOARD_COLS, BOARD_ROWS)
    criteria   = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                  30, 0.001)

    # 3-D object points for one checkerboard view — flat board (z = 0)
    objp = np.zeros((BOARD_COLS * BOARD_ROWS, 3), np.float32)
    objp[:, :2] = np.mgrid[0:BOARD_COLS, 0:BOARD_ROWS].T.reshape(-1, 2)
    objp *= SQUARE_MM

    # Build grid positions
    positions = [
        (args.x + dx, args.y + dy, Z_HEIGHT)
        for dx in X_OFFSETS
        for dy in Y_OFFSETS
    ]

    print()
    print("=" * 65)
    print("  Camera Calibration — Tilt & Distortion")
    print("=" * 65)
    print(f"  Checkerboard  : {BOARD_COLS} × {BOARD_ROWS} inner corners, "
          f"{SQUARE_MM:.0f} mm squares")
    print(f"  Capture res   : {CAPTURE_W} × {CAPTURE_H} px")
    print(f"  Start pos     : X={args.x:.0f}  Y={args.y:.0f}  Z={Z_HEIGHT:.0f}")
    print(f"  Grid          : {len(X_OFFSETS)} × {len(Y_OFFSETS)}"
          f" = {len(positions)} positions")
    print(f"  NPZ output    : {CALIBRATION_NPZ_PATH}")
    print()
    print("  Checkerboard placement:")
    print("    • Flat on the workpiece surface (Z = 0)")
    print(f"    • Centred roughly under X={args.x:.0f}, Y={args.y:.0f}")
    print("    • LONG side (18 inner corners / 9 black squares wide) running LEFT-RIGHT (machine Y)")
    print("    • SHORT side (11 inner corners / 6 black squares tall) running FRONT-BACK (machine X)")
    print()
    input("  Press Enter when the board is in place …\n")

    ser = open_serial()

    objpoints, imgpoints = [], []
    img_size             = None
    good = 0
    bad  = 0

    try:
        # ── Home ─────────────────────────────────────────────────────────────
        print("=== Homing machine ===")
        send_command(ser, "h")
        wait_for_message(ser, "SYSTEM IS HOMED", timeout=120)

        # ── Move to start ─────────────────────────────────────────────────────
        print(f"\n  Moving to start  X={args.x:.1f}  Y={args.y:.1f}  Z={Z_HEIGHT:.1f}")
        send_command(ser, f"to {args.x:.3f} {args.y:.3f} {Z_HEIGHT:.3f}")
        wait_for_message(ser, "SYSTEM IS AT POSITION", timeout=60)

        # ── Verification capture ──────────────────────────────────────────────
        print("\n  Verification capture at start position …")
        verify_path = os.path.join(IMAGE_DIR, "verify.jpg")
        vimg = capture(verify_path)
        vok, _, _ = find_corners(vimg, board_size, criteria)
        if vok:
            print("  ✓  Checkerboard detected at start position — board looks good.")
        else:
            print("  ✗  Checkerboard NOT detected at start position.")
            print("     Possible fixes:")
            print("       • Check that the board is fully inside the camera frame")
            print("       • Try rotating the board 90 degrees")
            print(f"       • Make sure board has {BOARD_COLS} inner corners (short side) × {BOARD_ROWS} inner corners (long side)")
            print()
            cont = input("  Continue anyway? (y/n): ").strip().lower()
            if cont != "y":
                print("  Aborted.")
                return

        # ── Grid traversal ────────────────────────────────────────────────────
        print(f"\n  Starting grid traversal  ({len(positions)} positions) …")
        print()

        for i, (px, py, pz) in enumerate(positions, 1):
            print(f"  [{i:02d}/{len(positions)}]  "
                  f"X={px:6.1f}  Y={py:6.1f}", end="  ", flush=True)

            send_command(ser, f"to {px:.3f} {py:.3f} {pz:.3f}")
            wait_for_message(ser, "SYSTEM IS AT POSITION", timeout=60)
            time.sleep(SETTLE_TIME)

            img_path   = os.path.join(IMAGE_DIR, f"cal_{i:02d}.jpg")
            debug_path = os.path.join(DEBUG_DIR,  f"cal_{i:02d}_corners.jpg")

            img = capture(img_path)

            ok, corners, size = find_corners(img, board_size, criteria)
            if ok:
                objpoints.append(objp.copy())
                imgpoints.append(corners)
                img_size = size
                save_debug_image(img, corners, board_size, debug_path)
                good += 1
                print(f"✓  ({good} good so far)")
            else:
                bad += 1
                print(f"✗  corners not found  ({bad} skipped)")

        # ── Return to start ───────────────────────────────────────────────────
        print(f"\n  Returning to start position …")
        send_command(ser, f"to {args.x:.3f} {args.y:.3f} {Z_HEIGHT:.3f}")
        wait_for_message(ser, "SYSTEM IS AT POSITION", timeout=60)

    finally:
        ser.close()
        print("  Serial closed.")

    # ── Check we have enough images ───────────────────────────────────────────
    print(f"\n  {good} usable images, {bad} skipped.")

    if good < 6:
        print(f"\n  ERROR: calibrateCamera() needs ≥ 6 images, only got {good}.")
        print("  Check board placement (must be fully in frame) and re-run.")
        return

    # ── Run OpenCV calibration ────────────────────────────────────────────────
    print(f"\n  Running cv2.calibrateCamera() on {good} images …", flush=True)

    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_size, None, None
    )

    reproj = mean_reprojection_error(
        objpoints, imgpoints, rvecs, tvecs, K, dist
    )

    # ── Save NPZ (loaded automatically by undistort_image in camera.py) ───────
    np.savez(
        CALIBRATION_NPZ_PATH,
        camera_matrix    = K,
        dist_coeffs      = dist,
        image_size       = np.array(img_size),
        reprojection_err = np.array([reproj]),
    )

    # ── Save human-readable JSON ──────────────────────────────────────────────
    cal_dict = {
        "camera_matrix":         K.tolist(),
        "dist_coeffs":           dist.ravel().tolist(),
        "image_size_wh":         list(img_size),
        "reprojection_error_px": round(reproj, 5),
        "images_used":           good,
        "board_cols":            BOARD_COLS,
        "board_rows":            BOARD_ROWS,
        "square_mm":             SQUARE_MM,
    }
    with open(JSON_PATH, "w") as f:
        json.dump(cal_dict, f, indent=2)

    # ── Save a test undistorted image ─────────────────────────────────────────
    # Pick the centre image from the grid (position 13 = index 12)
    test_idx  = min(good // 2, good - 1) + 1
    test_path = os.path.join(IMAGE_DIR, f"cal_{test_idx:02d}.jpg")
    test_img  = cv2.imread(test_path)
    if test_img is not None:
        K_opt, roi = cv2.getOptimalNewCameraMatrix(
            K, dist, img_size, 1, img_size
        )
        undist = cv2.undistort(test_img, K, dist, None, K_opt)
        x, y, rw, rh = roi
        undist = undist[y:y+rh, x:x+rw]
        scale  = min(1.0, 1600 / undist.shape[1])
        cv2.imwrite(
            os.path.join(DEBUG_DIR, "undistorted_test.jpg"),
            cv2.resize(undist, (int(undist.shape[1] * scale),
                                int(undist.shape[0] * scale)))
        )
        print(f"  Test undistorted image → {DEBUG_DIR}/undistorted_test.jpg")

    # ── Console summary ───────────────────────────────────────────────────────
    reproj_label = (
        "← excellent"   if reproj < 0.5 else
        "← good"        if reproj < 1.0 else
        "← acceptable"  if reproj < 2.0 else
        "← poor — check debug images and re-run"
    )

    print()
    print("=" * 65)
    print("  CALIBRATION COMPLETE")
    print("=" * 65)
    print(f"  Images used        : {good} / {len(positions)}")
    print(f"  Reprojection error : {reproj:.4f} px  {reproj_label}")
    print()
    print("  Camera matrix K  (fx, fy = focal length in px; cx, cy = principal point):")
    print(f"    fx = {K[0,0]:.4f}   fy = {K[1,1]:.4f}")
    print(f"    cx = {K[0,2]:.4f}   cy = {K[1,2]:.4f}")
    print()
    print("  Distortion coefficients  [k1, k2, p1, p2, k3]:")
    print(f"    {dist.ravel().tolist()}")
    print()
    print(f"  Saved (for undistortion pipeline)  → {CALIBRATION_NPZ_PATH}")
    print(f"  Saved (human-readable, share this) → {JSON_PATH}")
    print(f"  Debug images with corners drawn    → {DEBUG_DIR}/")
    print()
    print("  The calibration is now active.  undistort_image() in camera.py")
    print("  will automatically apply it to every captured image.")
    print()
    if reproj < 1.0:
        print("  Reprojection error looks good — you're done.")
    else:
        print("  Reprojection error is high.  Check the debug images for")
        print("  misdetected corners, then re-run with a cleaner board placement.")
    print("=" * 65)


if __name__ == "__main__":
    main()
