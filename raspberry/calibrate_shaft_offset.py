"""
calibrate_camera_to_screw1_offset.py

Moves the machine to SCREW1.
Then you jog X, Y and Z until the camera crosshair is centered above SCREW1.
Press c to save an image with a crosshair.
Type done when the camera is centered.
The script then calculates the camera XY offset from SCREW1.

Commands
  x <mm>      jog X by mm
  y <mm>      jog Y by mm
  z <mm>      jog Z by mm
  c           save image with crosshair
  done        calculate offset
  q           quit
"""

import cv2
import time
import os
import threading
from datetime import datetime
from flask import Flask, Response
from picamera2 import Picamera2

from serial_comm import open_serial, send_command, read_all_lines, wait_for_message


# ============================================================
# SCREW1 POSITION FROM ARDUINO CODE
# ============================================================
SCREW1_X_MM = 83.213
SCREW1_Y_MM = 86.988
START_Z_MM = 0.000

PREVIEW_W = 1280
PREVIEW_H = 720

SAVE_DIR = "screw1_camera_offset_images"


# ============================================================
# CAMERA SETUP
# ============================================================
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"size": (PREVIEW_W, PREVIEW_H), "format": "BGR888"}
))
picam2.start()
picam2.set_controls({"AfMode": 0, "LensPosition": 4.5})
time.sleep(1)

app = Flask(__name__)

latest_frame = None
frame_lock = threading.Lock()


def draw_crosshair(frame):
    cx = PREVIEW_W // 2
    cy = PREVIEW_H // 2

    cv2.line(frame, (cx - 90, cy), (cx + 90, cy), (0, 255, 255), 2)
    cv2.line(frame, (cx, cy - 90), (cx, cy + 90), (0, 255, 255), 2)
    cv2.circle(frame, (cx, cy), 7, (0, 255, 255), -1)

    return frame


def get_camera_frame_with_crosshair():
    raw = picam2.capture_array()

    if len(raw.shape) == 3 and raw.shape[2] == 4:
        frame = cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR)
    else:
        frame = raw.copy()

    frame = cv2.resize(frame, (PREVIEW_W, PREVIEW_H))
    frame = draw_crosshair(frame)

    with frame_lock:
        global latest_frame
        latest_frame = frame.copy()

    return frame


def save_crosshair_image(x=None, y=None, z=None):
    os.makedirs(SAVE_DIR, exist_ok=True)

    frame = get_camera_frame_with_crosshair()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"screw1_crosshair_{timestamp}.jpg"
    path = os.path.join(SAVE_DIR, filename)

    if x is not None and y is not None:
        text = f"X={x:.3f} mm  Y={y:.3f} mm"

        if z is not None:
            text += f"  Z={z:.3f} mm"

        cv2.putText(
            frame,
            text,
            (30, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 255),
            2,
            cv2.LINE_AA
        )

    cv2.imwrite(path, frame)
    return path


def generate_frames():
    while True:
        frame = get_camera_frame_with_crosshair()

        ret, buf = cv2.imencode(".jpg", frame)
        if not ret:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            buf.tobytes() +
            b"\r\n"
        )


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/")
def index():
    return """
    <html>
        <body style="margin:0;background:#000">
            <img src="/video_feed" style="width:100%">
        </body>
    </html>
    """


def start_stream():
    app.run(host="0.0.0.0", port=5000, threaded=True)


# ============================================================
# SERIAL HELPERS
# ============================================================
def read_position(ser, timeout=6.0):
    deadline = time.time() + timeout

    while time.time() < deadline:
        if ser.in_waiting:
            try:
                line = ser.readline().decode(errors="replace").strip()
            except Exception:
                continue

            if not line.startswith("DATA"):
                continue

            x = None
            y = None
            z = None

            for part in line.split("|"):
                p = part.strip()

                if p.startswith("X:"):
                    x = float(p.split(":")[1].strip().replace(" mm", ""))

                elif p.startswith("Y:"):
                    y = float(p.split(":")[1].strip().replace(" mm", ""))

                elif p.startswith("Z:"):
                    z = float(p.split(":")[1].strip().replace(" mm", ""))

            if x is not None and y is not None:
                return x, y, z

        time.sleep(0.02)

    return None


def jog(ser, axis, dist_mm):
    send_command(ser, f"{axis} {dist_mm:.3f}")
    time.sleep(0.05)
    read_all_lines(ser, timeout=0.3)


# ============================================================
# MAIN
# ============================================================
def run():
    ser = open_serial()

    try:
        print("Homing machine...")
        send_command(ser, "h")
        wait_for_message(ser, "SYSTEM IS HOMED", timeout=120)

        print()
        print("Moving to SCREW1 position:")
        print(f"  X = {SCREW1_X_MM:.3f} mm")
        print(f"  Y = {SCREW1_Y_MM:.3f} mm")
        print(f"  Z = {START_Z_MM:.3f} mm")

        send_command(
            ser,
            f"to {SCREW1_X_MM:.3f} {SCREW1_Y_MM:.3f} {START_Z_MM:.3f}"
        )

        time.sleep(5)
        read_all_lines(ser, timeout=0.5)

        print()
        print("Machine is now at SCREW1.")
        print("Now jog the machine until the camera crosshair is centered over SCREW1.")

        stream_thread = threading.Thread(target=start_stream, daemon=True)
        stream_thread.start()

        print()
        print("Live camera feed:")
        print("  http://172.20.10.2:5000")
        print()
        print("Commands:")
        print("  x <mm>      jog X")
        print("  y <mm>      jog Y")
        print("  z <mm>      jog Z")
        print("  c           save image with crosshair")
        print("  done        calculate offset")
        print("  q           quit")

        while True:
            raw = input("CMD> ").strip().lower()

            if raw == "q":
                print("Aborted.")
                return

            if raw == "c":
                pos = read_position(ser, timeout=8.0)

                if pos is None:
                    path = save_crosshair_image()
                    print()
                    print("Image saved without position data:")
                    print(f"  {path}")
                    continue

                x_now, y_now, z_now = pos
                path = save_crosshair_image(x_now, y_now, z_now)

                print()
                print("Image saved:")
                print(f"  {path}")
                print(f"Current position: X={x_now:.3f}  Y={y_now:.3f}  Z={z_now:.3f}")
                continue

            if raw == "done":
                pos = read_position(ser, timeout=8.0)

                if pos is None:
                    print("ERROR: no DATA line received. Try again when the machine is idle.")
                    continue

                x_final, y_final, z_final = pos

                camera_offset_x = x_final - SCREW1_X_MM
                camera_offset_y = y_final - SCREW1_Y_MM

                path = save_crosshair_image(x_final, y_final, z_final)

                print()
                print("=" * 60)
                print("CAMERA OFFSET RESULT")
                print("=" * 60)
                print("SCREW1 position:")
                print(f"  X = {SCREW1_X_MM:.3f} mm")
                print(f"  Y = {SCREW1_Y_MM:.3f} mm")
                print()
                print("Final camera centered position:")
                print(f"  X = {x_final:.3f} mm")
                print(f"  Y = {y_final:.3f} mm")
                print(f"  Z = {z_final:.3f} mm")
                print()
                print("Camera offset:")
                print(f"  CAMERA_OFFSET_X_MM = {camera_offset_x:.3f}")
                print(f"  CAMERA_OFFSET_Y_MM = {camera_offset_y:.3f}")
                print()
                print("Final confirmation image:")
                print(f"  {path}")
                print("=" * 60)
                return

            parts = raw.split()

            if len(parts) == 2 and parts[0] in ("x", "y", "z"):
                try:
                    dist = float(parts[1])
                    jog(ser, parts[0], dist)
                    print(f"Jogged {parts[0].upper()} {dist:+.3f} mm")

                except ValueError:
                    print("Use format: x 1 or y -0.5 or z 2")

            else:
                print("Commands: x <mm>, y <mm>, z <mm>, c, done, q")

    finally:
        picam2.stop()
        ser.close()
        print("Done.")


if __name__ == "__main__":
    run()