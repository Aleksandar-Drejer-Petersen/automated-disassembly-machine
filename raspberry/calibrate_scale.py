"""
calibrate_scale.py

Computes mm_per_pixel_x and mm_per_pixel_y by moving the machine a known
distance and measuring how many pixels the red square shifts between two photos.

No display or checkerboard needed — just takes photos and saves them to disk.

Usage:
    python calibrate_scale.py
"""

import os
import time
import cv2
import numpy as np

from config import (
    SERIAL_PORT, BAUD_RATE,
    CAM1_X, CAM1_Y,
    MM_PER_PIXEL_TXT_PATH,
    VISION_PHOTOS_DIR,
    CALIBRATION_NPZ_PATH,
)
from camera import capture_image, undistort_image
from serial_comm import open_serial, send_command, wait_for_message

os.makedirs(VISION_PHOTOS_DIR, exist_ok=True)

MOVE_MM     = 50.0   # how far to move for each axis calibration step
PHOTO_A_X   = os.path.join(VISION_PHOTOS_DIR, "cal_x_before.jpg")
PHOTO_B_X   = os.path.join(VISION_PHOTOS_DIR, "cal_x_after.jpg")
PHOTO_A_Y   = os.path.join(VISION_PHOTOS_DIR, "cal_y_before.jpg")
PHOTO_B_Y   = os.path.join(VISION_PHOTOS_DIR, "cal_y_after.jpg")


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
        print(f"  WARNING: no red square found in {image_path}")
        return None
    rx, ry, rw, rh = cv2.boundingRect(max(contours, key=cv2.contourArea))
    return rx + rw / 2, ry + rh / 2


def main():
    ser = open_serial()

    try:
        print("\n=== Homing ===")
        send_command(ser, "h")
        wait_for_message(ser, "SYSTEM IS HOMED", timeout=120)

        print("\n=== Moving to CAM1 ===")
        send_command(ser, "cam1")
        wait_for_message(ser, "SYSTEM IS AT CAM1", timeout=60)

        # ── X calibration ─────────────────────────────────────────────────────
        print(f"\n=== X calibration: move {MOVE_MM} mm in X ===")

        print("Taking photo A (before X move) …")
        capture_image(PHOTO_A_X)
        print(f"  Saved: {PHOTO_A_X}")

        send_command(ser, f"x {MOVE_MM:.1f}")
        wait_for_message(ser, "SYSTEM IS AT X", timeout=60)

        print("Taking photo B (after X move) …")
        capture_image(PHOTO_B_X)
        print(f"  Saved: {PHOTO_B_X}")

        # Return to start
        send_command(ser, f"x -{MOVE_MM:.1f}")
        wait_for_message(ser, "SYSTEM IS AT X", timeout=60)

        # ── Y calibration ─────────────────────────────────────────────────────
        print(f"\n=== Y calibration: move {MOVE_MM} mm in Y ===")

        print("Taking photo A (before Y move) …")
        capture_image(PHOTO_A_Y)
        print(f"  Saved: {PHOTO_A_Y}")

        send_command(ser, f"y {MOVE_MM:.1f}")
        wait_for_message(ser, "SYSTEM IS AT Y", timeout=60)

        print("Taking photo B (after Y move) …")
        capture_image(PHOTO_B_Y)
        print(f"  Saved: {PHOTO_B_Y}")

        # Return to start
        send_command(ser, f"y -{MOVE_MM:.1f}")
        wait_for_message(ser, "SYSTEM IS AT Y", timeout=60)

    finally:
        ser.close()
        print("\nSerial closed.")

    # ── Compute scale ─────────────────────────────────────────────────────────
    print("\n=== Computing mm_per_pixel ===")

    cA_x = find_red_center(PHOTO_A_X)
    cB_x = find_red_center(PHOTO_B_X)
    cA_y = find_red_center(PHOTO_A_Y)
    cB_y = find_red_center(PHOTO_B_Y)

    if cA_x is None or cB_x is None:
        print("ERROR: could not detect red square in X calibration photos.")
        print(f"  Check {PHOTO_A_X} and {PHOTO_B_X} manually.")
        return

    if cA_y is None or cB_y is None:
        print("ERROR: could not detect red square in Y calibration photos.")
        print(f"  Check {PHOTO_A_Y} and {PHOTO_B_Y} manually.")
        return

    px_shift_x = abs(cB_x[0] - cA_x[0])
    px_shift_y = abs(cB_y[1] - cA_y[1])

    print(f"  X: red square moved {px_shift_x:.1f} px for {MOVE_MM} mm")
    print(f"  Y: red square moved {px_shift_y:.1f} px for {MOVE_MM} mm")

    if px_shift_x < 5:
        print("WARNING: X pixel shift is very small — check that the machine actually moved.")
    if px_shift_y < 5:
        print("WARNING: Y pixel shift is very small — check that the machine actually moved.")

    mm_per_pixel_x = MOVE_MM / px_shift_x
    mm_per_pixel_y = MOVE_MM / px_shift_y

    print(f"\n  mm_per_pixel_x = {mm_per_pixel_x:.6f}")
    print(f"  mm_per_pixel_y = {mm_per_pixel_y:.6f}")

    os.makedirs(os.path.dirname(MM_PER_PIXEL_TXT_PATH), exist_ok=True)
    with open(MM_PER_PIXEL_TXT_PATH, "w") as f:
        f.write(f"mm_per_pixel_x: {mm_per_pixel_x:.6f}\n")
        f.write(f"mm_per_pixel_y: {mm_per_pixel_y:.6f}\n")
        f.write(f"move_mm: {MOVE_MM}\n")
        f.write(f"px_shift_x: {px_shift_x:.2f}\n")
        f.write(f"px_shift_y: {px_shift_y:.2f}\n")

    print(f"\nSaved → {MM_PER_PIXEL_TXT_PATH}")
    print("\nDone. Run vision_test.py to verify.")
    print(f"Calibration photos saved to {VISION_PHOTOS_DIR} — inspect them to confirm the red square was detected.")


if __name__ == "__main__":
    main()
