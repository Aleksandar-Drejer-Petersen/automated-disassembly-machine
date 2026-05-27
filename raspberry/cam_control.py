"""
cam_control.py

Control the machine with normal commands and take photos with crosshair.

Commands
--------
  h              home all axes
  x <mm>         move X by mm  (e.g.  x 10  or  x -5)
  y <mm>         move Y
  z <mm>         move Z
  to <x> <y>     move to absolute XY position
  p              take photo, draw crosshair, save to output/photo.jpg
  q              quit
"""

import cv2
import time
import os
from picamera2 import Picamera2
from serial_comm import open_serial, send_command, read_all_lines, wait_for_message

PHOTO_PATH = "/home/alekpi/bachelor/automated-disassembly-machine/raspberry/output/photo.jpg"
PREVIEW_W  = 1280
PREVIEW_H  = 720

os.makedirs(os.path.dirname(PHOTO_PATH), exist_ok=True)

# Start camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"size": (PREVIEW_W, PREVIEW_H), "format": "BGR888"}
))
picam2.start()
picam2.set_controls({"AfMode": 0, "LensPosition": 4.5})
time.sleep(1)
print("Camera ready.")


def take_photo():
    raw   = picam2.capture_array()
    frame = cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR) if raw.shape[2] == 4 else raw.copy()
    frame = cv2.resize(frame, (PREVIEW_W, PREVIEW_H))

    # Draw crosshair at centre
    cx, cy = PREVIEW_W // 2, PREVIEW_H // 2
    cv2.line(frame,   (cx - 80, cy), (cx + 80, cy), (0, 255, 255), 2)
    cv2.line(frame,   (cx, cy - 80), (cx, cy + 80), (0, 255, 255), 2)
    cv2.circle(frame, (cx, cy), 6,   (0, 255, 255), -1)

    cv2.imwrite(PHOTO_PATH, frame)
    print(f"Photo saved: {PHOTO_PATH}")


def run():
    ser = open_serial()
    print("Machine connected.")
    print("Commands: h | x <mm> | y <mm> | z <mm> | to <x> <y> | p | q")

    try:
        while True:
            try:
                raw = input("> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if not raw:
                continue

            if raw == "q":
                break

            elif raw == "p":
                take_photo()

            elif raw == "h":
                send_command(ser, "h")
                wait_for_message(ser, "SYSTEM IS HOMED", timeout=120)

            elif raw.startswith("to "):
                parts = raw.split()
                if len(parts) == 3:
                    send_command(ser, f"to {float(parts[1]):.3f} {float(parts[2]):.3f} 0.000")
                    time.sleep(0.2)
                    read_all_lines(ser, timeout=0.5)
                else:
                    print("  usage: to <x> <y>")

            elif len(raw.split()) == 2 and raw.split()[0] in ("x", "y", "z"):
                axis, val = raw.split()
                try:
                    send_command(ser, f"{axis} {float(val):.3f}")
                    time.sleep(0.2)
                    read_all_lines(ser, timeout=0.5)
                except ValueError:
                    print(f"  usage: {axis} <mm>")

            else:
                # Pass anything else directly to Arduino
                send_command(ser, raw)
                time.sleep(0.2)
                read_all_lines(ser, timeout=0.5)

    finally:
        picam2.stop()
        ser.close()
        print("Done.")


if __name__ == "__main__":
    run()
