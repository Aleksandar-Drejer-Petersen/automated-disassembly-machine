"""
Calibrate the pixel-to-machine transform matrix T.

Procedure
---------
1. Home the machine, then move to CAM1 position.
2. Take a baseline photo and find the red square center in pixels.
3. For each calibration offset (dm_x, dm_y) in mm:
     a. Move machine by that offset.
     b. Take a photo and find red square center.
     c. Record the pixel change vs machine movement.
     d. Return to baseline.
4. Solve for the 2x2 matrix T such that T @ pixel_change ≈ -machine_offset.
   Equivalently: for any image pixel offset (px, py) from image center,
   machine_correction = T @ [px, py]
5. Save T to NPZ.

The machine must be homed and the red square must be visible throughout.
Run this script once after physically setting up the camera.
"""

import numpy as np
import time
import sys

from config import (
    CAM1_X, CAM1_Y,
    CALIBRATION_NPZ_PATH,
)
from transform import TRANSFORM_NPZ_PATH
from camera import capture_image, undistort_image
from serial_comm import open_serial, send_command, read_all_lines, wait_for_message
import cv2

# Temporary image path used during calibration
_CAL_IMAGE_PATH = "/home/nicksrasp/Desktop/Bachelor/cal_temp.jpg"

# Offsets (mm) to move machine during calibration.
# Cover X, Y, and diagonal directions for a well-conditioned solve.
CAL_OFFSETS_MM = [
    ( 15,   0),
    (-15,   0),
    (  0,  15),
    (  0, -15),
    ( 15,  15),
    (-15,  15),
    ( 15, -15),
    (-15, -15),
]

# Wait after each movement before capturing (seconds).
# Increase if machine vibration blurs the image.
SETTLE_TIME_S = 1.5


def _find_red_square_pixel(image_path):
    """Return (px, py) pixel offset from image center, or None if not found."""
    img = cv2.imread(image_path)
    if img is None:
        return None

    img = undistort_image(img, CALIBRATION_NPZ_PATH)
    h, w = img.shape[:2]
    cx, cy = w / 2, h / 2

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0,   120, 70]), np.array([10,  255, 255])),
        cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
    )
    kernel = np.ones((15, 15), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    rx, ry, rw, rh = cv2.boundingRect(max(contours, key=cv2.contourArea))
    red_cx = rx + rw / 2
    red_cy = ry + rh / 2
    return (red_cx - cx, red_cy - cy)


def _move_and_wait(ser, axis, dist_mm, timeout=30):
    send_command(ser, f"{axis} {dist_mm:.4f}")
    time.sleep(SETTLE_TIME_S)
    # Drain any buffered responses without blocking
    read_all_lines(ser, timeout=0.3)


def _move_xy_and_wait(ser, dm_x, dm_y):
    if abs(dm_x) > 0.001:
        _move_and_wait(ser, "x", dm_x)
    if abs(dm_y) > 0.001:
        _move_and_wait(ser, "y", dm_y)


def run_calibration():
    ser = open_serial()

    try:
        print("Homing machine...")
        send_command(ser, "h")
        wait_for_message(ser, "SYSTEM IS HOMED", timeout=120)

        print(f"Moving to CAM1 ({CAM1_X}, {CAM1_Y})...")
        send_command(ser, "cam1")
        time.sleep(5)
        read_all_lines(ser, timeout=0.5)

        # Baseline capture
        print("Capturing baseline image...")
        capture_image(_CAL_IMAGE_PATH)
        baseline = _find_red_square_pixel(_CAL_IMAGE_PATH)
        if baseline is None:
            print("ERROR: red square not found in baseline image. Aborting.")
            return
        bpx, bpy = baseline
        print(f"Baseline red square offset from center: ({bpx:.1f}, {bpy:.1f}) px")

        dpixels = []  # pixel change for each offset: after - before
        dms     = []  # machine offsets applied

        for i, (dm_x, dm_y) in enumerate(CAL_OFFSETS_MM):
            print(f"\n--- Calibration point {i+1}/{len(CAL_OFFSETS_MM)}: move ({dm_x:+.0f}, {dm_y:+.0f}) mm ---")

            _move_xy_and_wait(ser, dm_x, dm_y)

            capture_image(_CAL_IMAGE_PATH)
            result = _find_red_square_pixel(_CAL_IMAGE_PATH)
            if result is None:
                print(f"WARNING: red square not found after offset ({dm_x}, {dm_y}). Skipping.")
                _move_xy_and_wait(ser, -dm_x, -dm_y)
                continue

            apx, apy = result
            dpx = apx - bpx
            dpy = apy - bpy
            print(f"  Red square moved by ({dpx:+.1f}, {dpy:+.1f}) px in image")
            dpixels.append([dpx, dpy])
            dms.append([dm_x, dm_y])

            _move_xy_and_wait(ser, -dm_x, -dm_y)

        if len(dpixels) < 4:
            print(f"ERROR: only {len(dpixels)} valid points collected. Need at least 4. Aborting.")
            return

        print(f"\nComputing transform from {len(dpixels)} calibration points...")

        # Solve: T @ dpixel = -dm  →  T = -dms.T @ pinv(dpixels.T)
        # Derivation: moving machine by dm shifts feature by dpixel in image.
        # To center on a feature at pixel offset p: machine_correction = T @ p.
        dpixels_arr = np.array(dpixels).T  # 2×N
        dms_arr     = np.array(dms).T      # 2×N

        T = -dms_arr @ np.linalg.pinv(dpixels_arr)

        print(f"Transform matrix T (pixel offset → machine mm offset):")
        print(f"  {T}")

        # Verify: residuals
        predicted = T @ dpixels_arr
        residuals = (-dms_arr) - predicted
        rms = np.sqrt(np.mean(residuals ** 2))
        print(f"RMS residual: {rms:.4f} mm")
        if rms > 0.5:
            print("WARNING: residual is large. Check that the red square was detected correctly.")

        np.savez(TRANSFORM_NPZ_PATH, T=T)
        print(f"\nTransform saved to: {TRANSFORM_NPZ_PATH}")

    finally:
        ser.close()
        print("Serial closed.")


if __name__ == "__main__":
    run_calibration()
