"""
Vision test: detect all screws and move the machine to the first detected one.

Usage:
    python vision_test.py
    python vision_test.py --no-move   # detect only, skip the axis move

Saves timestamped images to the same output folder as main.py:
    <ts>_raw.jpg          raw capture at CAM1
    <ts>_adjusted.jpg     capture after red-square correction
    <ts>_annotated.jpg    debug image with circles and labels
    <ts>_thresh.jpg       binary threshold used for hole detection
"""

import argparse
import os
import shutil
from datetime import datetime

from config import (
    DEBUG_IMAGE_PATH,
    MM_PER_PIXEL_TXT_PATH,
    CAM1_RED_SQUARE_IMAGE_PATH,
    CAM1_ADJUSTED_RED_SQUARE_IMAGE_PATH,
    CAM1_X, CAM1_Y,
    CAMERA_OFFSET_X_MM, CAMERA_OFFSET_Y_MM,
    VISION_PHOTOS_DIR,
)

os.makedirs(VISION_PHOTOS_DIR, exist_ok=True)
from camera import capture_image, load_mm_per_pixel
from vision import find_red_square_offset, detect_screws
from serial_comm import open_serial, send_command, wait_for_message, wait_for_any_message

OUTPUT_DIR = os.path.dirname(DEBUG_IMAGE_PATH)


def save_timestamped(src, label, ts):
    ext = os.path.splitext(src)[1]
    dst = os.path.join(OUTPUT_DIR, f"{ts}_{label}{ext}")
    if os.path.exists(src):
        shutil.copy2(src, dst)
        print(f"Saved: {dst}")
    return dst


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-move", action="store_true",
                        help="Detect screws but skip the axis move at the end")
    args = parser.parse_args()

    mm_per_pixel_x, mm_per_pixel_y = load_mm_per_pixel(MM_PER_PIXEL_TXT_PATH)
    ser = open_serial()
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    try:
        print("\n=== Homing machine ===")
        send_command(ser, "h")
        wait_for_message(ser, "SYSTEM IS HOMED", timeout=120)

        print("\n=== Moving to CAM1 ===")
        send_command(ser, "cam1")
        wait_for_message(ser, "SYSTEM IS AT CAM1", timeout=60)

        print("\n=== Centering on red square (iterative) ===")
        CENTER_TOLERANCE_MM = 0.5
        MAX_CENTER_ITERS    = 4

        red_square_x = CAM1_X
        red_square_y = CAM1_Y

        for iteration in range(MAX_CENTER_ITERS):
            img_path = CAM1_RED_SQUARE_IMAGE_PATH if iteration == 0 else CAM1_ADJUSTED_RED_SQUARE_IMAGE_PATH
            capture_image(img_path)
            if iteration == 0:
                save_timestamped(img_path, "raw", ts)

            dx, dy = find_red_square_offset(img_path, mm_per_pixel_x, mm_per_pixel_y)
            print(f"  Iter {iteration + 1}: residual dx={dx:+.3f} mm, dy={dy:+.3f} mm")

            if abs(dx) <= CENTER_TOLERANCE_MM and abs(dy) <= CENTER_TOLERANCE_MM:
                print(f"  Within {CENTER_TOLERANCE_MM} mm — centred.")
                break

            send_command(ser, f"x {dx:.3f}")
            wait_for_message(ser, "SYSTEM IS AT X", timeout=60)
            send_command(ser, f"y {dy:.3f}")
            wait_for_message(ser, "SYSTEM IS AT Y", timeout=60)
            red_square_x += dx
            red_square_y += dy
        else:
            print(f"  WARNING: still not centred after {MAX_CENTER_ITERS} iterations.")

        print(f"Red square machine position: X={red_square_x:.3f}, Y={red_square_y:.3f}")

        print("\n=== Capture adjusted image ===")
        capture_image(CAM1_ADJUSTED_RED_SQUARE_IMAGE_PATH)
        save_timestamped(CAM1_ADJUSTED_RED_SQUARE_IMAGE_PATH, "adjusted", ts)

        screw_results, _ = detect_screws(
            CAM1_ADJUSTED_RED_SQUARE_IMAGE_PATH,
            DEBUG_IMAGE_PATH,
            mm_per_pixel_x,
            mm_per_pixel_y,
        )

        save_timestamped(DEBUG_IMAGE_PATH, "annotated", ts)
        save_timestamped(DEBUG_IMAGE_PATH.replace(".jpg", "_thresh3.jpg"), "thresh", ts)

        print("\n=== RESULTS ===")
        print(f"Total screws found: {len(screw_results)}")
        for s in screw_results:
            screw_abs_x = red_square_x + s["machine_dx"] - CAMERA_OFFSET_X_MM
            screw_abs_y = red_square_y + s["machine_dy"] - CAMERA_OFFSET_Y_MM
            print(
                f"  Hole #{s['index']}: "
                f"offset=({s['machine_dx']:+.2f}, {s['machine_dy']:+.2f}) mm  "
                f"abs=(X={screw_abs_x:.3f}, Y={screw_abs_y:.3f})  "
                f"type={s['screw_type']}  diam={s['diameter_mm']:.1f}mm  "
                f"press={'YES' if s['has_press'] else 'no'}"
            )

        if not screw_results:
            print("No screws detected — nothing to move to.")
            return

        first = screw_results[0]
        print(
            f"\n=== First screw: #{first['index']}  "
            f"offset=({first['machine_dx']:+.2f}, {first['machine_dy']:+.2f}) mm ==="
        )

        if args.no_move:
            print("--no-move set: skipping axis move.")
        else:
            # Move camera to screw, then apply shaft offset so the BIT is over it
            dx_bit = first['machine_dx'] - CAMERA_OFFSET_X_MM
            dy_bit = first['machine_dy'] - CAMERA_OFFSET_Y_MM
            print(f"Moving to first screw (camera delta + shaft offset) …")
            send_command(ser, f"x {dx_bit:.3f}")
            wait_for_message(ser, "SYSTEM IS AT X", timeout=60)
            send_command(ser, f"y {dy_bit:.3f}")
            wait_for_message(ser, "SYSTEM IS AT Y", timeout=60)
            print("Done — bit is centred on first detected screw.")

            answer = input("\nPress Enter to unscrew, or type 'skip' to abort: ").strip().lower()
            if answer != "skip":
                print("\n=== Unscrewing ===")
                send_command(ser, "UNSCREW")
                result = wait_for_any_message(
                    ser,
                    ["UNSCREW: done", "control released", "UNSCREW ERROR"],
                    timeout=120,
                )
                if result is None:
                    print("Unscrew timed out.")
                elif "ERROR" in result:
                    print(f"Unscrew failed: {result}")
                else:
                    print("Unscrew complete.")

                print("\n=== Homing Z ===")
                send_command(ser, "H Z")
                wait_for_message(ser, "Axis Z safe", timeout=30)
            else:
                print("Unscrew skipped.")

    finally:
        ser.close()
        print("\nSerial closed.")


if __name__ == "__main__":
    main()
