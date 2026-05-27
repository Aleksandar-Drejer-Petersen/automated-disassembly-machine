"""
calibrate_homography.py

Computes the pixel-to-machine homography H using a printed checkerboard.
Board: 11×18 inner corners, 5 mm per square (19×12 squares total).

How it works
------------
For each of 3 anchor corners the script opens a live camera feed.
You jog the machine (WASD / arrow keys) until the highlighted corner
sits exactly on the crosshair, then press C to confirm.
The machine position at that moment IS the machine coordinate of that corner.
No bit-tip alignment needed — the camera centre is the measurement point.

From 3 anchor (pixel, machine) pairs + known 20 mm grid spacing the script
computes machine coordinates for all 40 corners, then calls
cv2.findHomography to produce H and saves it.

Controls in the live window
---------------------------
  A / ←    move machine −X (left)
  D / →    move machine +X (right)
  W / ↑    move machine −Y (away)
  S / ↓    move machine +Y (toward)
  [ / ]    halve / double jog step  (0.1 → 0.2 → 0.5 → 1 → 2 → 5 mm)
  C        confirm — record current machine position
  ESC      abort calibration

Dependencies
------------
  pip install picamera2   (pre-installed on RPi OS Bullseye+)
"""

import cv2
import http.server
import numpy as np
import select
import sys
import termios
import threading
import time
import tty
from config import CALIBRATION_NPZ_PATH
from camera import undistort_image
from serial_comm import open_serial, send_command, read_all_lines, wait_for_message

# ── Board parameters ──────────────────────────────────────────────────────────
BOARD_COLS   = 17     # inner corners per row  (18 squares wide → 17)  [long side, horizontal]
BOARD_ROWS   = 10     # inner corner rows       (11 squares tall → 10) [short side, vertical]
SQUARE_MM    = 5.0

# ── Output ────────────────────────────────────────────────────────────────────
HOMOGRAPHY_NPZ = "/home/alekpi/bachelor/automated-disassembly-machine/raspberry/output/homography.npz"

# ── Live preview resolution (lower = faster detection) ────────────────────────
PREVIEW_W = 1280
PREVIEW_H = 720

# ── Jog step ladder (mm) ─────────────────────────────────────────────────────
JOG_STEPS = [0.1, 0.2, 0.5, 1.0, 2.0, 5.0]

# ── Arrow-key codes returned by cv2.waitKey on Linux ─────────────────────────
KEY_LEFT  = 81
KEY_RIGHT = 83
KEY_UP    = 82
KEY_DOWN  = 84

# ── Anchors: 3 corners the user must centre under the crosshair ───────────────
#   #0               → board origin      (0,   0  ) mm
#   #COLS-1          → end of first row  (80,  0  ) mm
#   #(ROWS-1)*COLS   → start of last row (0,   140) mm
ANCHOR_INDICES = [0, BOARD_COLS - 1, (BOARD_ROWS - 1) * BOARD_COLS]


def _board_mm(flat_index):
    col = flat_index % BOARD_COLS
    row = flat_index // BOARD_COLS
    return np.array([col * SQUARE_MM, row * SQUARE_MM], dtype=float)


# ── Serial helpers ────────────────────────────────────────────────────────────
def _read_position(ser, timeout=6.0):
    """Parse X, Y from the next DATA line the Arduino sends."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ser.in_waiting:
            try:
                line = ser.readline().decode(errors="replace").strip()
            except Exception:
                continue
            if not line.startswith("DATA"):
                continue
            x = y = None
            for part in line.split("|"):
                p = part.strip()
                if p.startswith("X:"):
                    x = float(p.split(":")[1].strip().replace(" mm", ""))
                elif p.startswith("Y:"):
                    y = float(p.split(":")[1].strip().replace(" mm", ""))
            if x is not None and y is not None:
                return x, y
        time.sleep(0.02)
    return None


def _jog(ser, axis, dist_mm):
    send_command(ser, f"{axis} {dist_mm:.3f}")
    # Drain responses without waiting
    time.sleep(0.05)
    read_all_lines(ser, timeout=0.1)


# ── Corner detection on a single frame ───────────────────────────────────────
def _detect_corners(frame_bgr):
    """Return (N,2) float32 array of inner corner pixel positions, or None."""
    gray  = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH |
             cv2.CALIB_CB_NORMALIZE_IMAGE  |
             cv2.CALIB_CB_FAST_CHECK)
    found, corners = cv2.findChessboardCorners(gray, (BOARD_COLS, BOARD_ROWS), flags)
    if not found:
        return None
    crit    = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 15, 0.1)
    corners = cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1), crit)
    return corners.reshape(-1, 2)


# ── Overlay drawing ───────────────────────────────────────────────────────────
def _draw_overlay(frame, corners, target_idx, step_mm, status_msg=""):
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2

    # Crosshair
    cv2.line(frame, (cx - 60, cy), (cx + 60, cy), (0, 255, 255), 2)
    cv2.line(frame, (cx, cy - 60), (cx, cy + 60), (0, 255, 255), 2)
    cv2.circle(frame, (cx, cy), 6, (0, 255, 255), -1)

    if corners is not None:
        # All corners — small white dots
        for i, (px, py) in enumerate(corners.astype(int)):
            cv2.circle(frame, (px, py), 4, (200, 200, 200), -1)

        # Target corner — red ring + line to crosshair
        tpx, tpy = corners[target_idx].astype(int)
        cv2.circle(frame, (tpx, tpy), 20, (0, 0, 255), 3)
        cv2.circle(frame, (tpx, tpy), 5,  (0, 0, 255), -1)
        cv2.line(frame, (cx, cy), (tpx, tpy), (0, 0, 255), 1)

        # Distance readout
        dist_px = float(np.hypot(tpx - cx, tpy - cy))
        cv2.putText(frame, f"dist: {dist_px:.0f} px",
                    (tpx + 25, tpy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Green circle when close
        if dist_px < 20:
            cv2.circle(frame, (cx, cy), 25, (0, 255, 0), 3)

    # HUD
    lines = [
        f"Target corner: #{target_idx}",
        f"Jog step: {step_mm} mm",
        "A/D=X  W/S=Y  [/]=step  C=confirm  ESC=abort",
    ]
    if status_msg:
        lines.append(status_msg)
    for i, txt in enumerate(lines):
        cv2.putText(frame, txt, (15, 35 + i * 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

    return frame


# ── Headless MJPEG preview server ────────────────────────────────────────────
MJPEG_PORT  = 5001
_frame_jpg  = b''
_frame_lock = threading.Lock()


class _MJPEGHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _frame_lock:
                    jpg = _frame_jpg
                if jpg:
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                        + jpg + b"\r\n"
                    )
                time.sleep(0.05)
        except Exception:
            pass

    def log_message(self, *_):
        pass


def _start_mjpeg_server():
    srv = http.server.HTTPServer(("0.0.0.0", MJPEG_PORT), _MJPEGHandler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"\nLive preview → open in browser:  http://<pi-ip>:{MJPEG_PORT}\n")
    return srv


def _press_frame(bgr_img):
    _, buf = cv2.imencode(".jpg", bgr_img, [cv2.IMWRITE_JPEG_QUALITY, 75])
    with _frame_lock:
        global _frame_jpg
        _frame_jpg = buf.tobytes()


def _getch(timeout=0.05):
    """Read one keypress without Enter (Linux terminal). Returns '' on timeout."""
    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if ready else ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ── Live alignment loop ───────────────────────────────────────────────────────
def live_align(ser, picam2, target_idx, corner_label):
    """
    Show live feed until user confirms (C) or aborts (ESC).
    Returns (machine_x, machine_y) or None on abort.
    """
    step_idx     = JOG_STEPS.index(1.0)
    corners      = None
    status       = ""
    DETECT_EVERY = 3
    frame_n      = 0

    print(f"\nAlign corner #{target_idx} ({corner_label}) to the crosshair.")
    print("Controls: a/d=X  w/s=Y  [/]=step  c=confirm  q=abort")

    while True:
        raw = picam2.capture_array()
        frame = cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR) if raw.shape[2] == 4 else raw.copy()
        frame = cv2.resize(frame, (PREVIEW_W, PREVIEW_H))
        frame = undistort_image(frame, CALIBRATION_NPZ_PATH)

        if frame_n % DETECT_EVERY == 0:
            corners = _detect_corners(frame)
        frame_n += 1

        step_mm = JOG_STEPS[step_idx]
        vis     = _draw_overlay(frame.copy(), corners, target_idx, step_mm, status)
        _press_frame(vis)

        key = _getch()
        if not key:
            continue

        if key == 'q':
            return None
        elif key == 'a':
            _jog(ser, "x", -step_mm);  status = f"Jogged X -{step_mm} mm"
        elif key == 'd':
            _jog(ser, "x", +step_mm);  status = f"Jogged X +{step_mm} mm"
        elif key == 'w':
            _jog(ser, "y", -step_mm);  status = f"Jogged Y -{step_mm} mm"
        elif key == 's':
            _jog(ser, "y", +step_mm);  status = f"Jogged Y +{step_mm} mm"
        elif key == '[':
            step_idx = max(0, step_idx - 1)
            status = f"Step → {JOG_STEPS[step_idx]} mm"
        elif key == ']':
            step_idx = min(len(JOG_STEPS) - 1, step_idx + 1)
            status = f"Step → {JOG_STEPS[step_idx]} mm"
        elif key == 'c':
            status = "Reading position..."
            pos = _read_position(ser, timeout=8.0)
            if pos is None:
                status = "ERROR: no DATA line — is machine idle?"
                continue
            print(f"  Confirmed: machine X={pos[0]:.3f}  Y={pos[1]:.3f}")
            return pos


# ── Homography solve ──────────────────────────────────────────────────────────
def _compute_machine_coords_all(anchor_machine):
    """
    Given 3 anchor machine positions, solve the affine board-mm → machine
    transform and apply it to all BOARD_COLS × BOARD_ROWS corners.
    Returns (N, 2) array of machine coordinates.
    """
    src = np.array([_board_mm(i) for i in ANCHOR_INDICES], dtype=float)
    dst = np.array(anchor_machine, dtype=float)

    src_aug = np.hstack([src, np.ones((3, 1))])
    A, _, _, _ = np.linalg.lstsq(src_aug, dst, rcond=None)   # (3, 2)

    all_board = np.array(
        [[col * SQUARE_MM, row * SQUARE_MM]
         for row in range(BOARD_ROWS)
         for col in range(BOARD_COLS)],
        dtype=float
    )
    all_aug = np.hstack([all_board, np.ones((len(all_board), 1))])
    return all_aug @ A   # (N, 2)


def _solve_homography(pixel_pts, machine_pts):
    H, mask = cv2.findHomography(
        np.array(pixel_pts,  dtype=np.float32),
        np.array(machine_pts, dtype=np.float32),
        cv2.RANSAC, 1.0
    )
    inliers  = int(mask.sum())
    proj     = cv2.perspectiveTransform(
        np.array(pixel_pts, dtype=np.float32).reshape(-1, 1, 2), H
    ).reshape(-1, 2)
    residuals = np.linalg.norm(proj - np.array(machine_pts), axis=1)
    print(f"Homography: {inliers}/{len(mask)} inliers, "
          f"RMS={residuals.mean():.3f} mm, max={residuals.max():.3f} mm")
    if residuals.mean() > 1.5:
        print("WARNING: reprojection error is high — check corner alignment.")
    return H


# ── Main ──────────────────────────────────────────────────────────────────────
def run():
    try:
        from picamera2 import Picamera2
    except ImportError:
        print("ERROR: picamera2 not found.  Install with:  pip install picamera2")
        return

    ser = open_serial()

    try:
        print("Homing machine...")
        send_command(ser, "h")
        wait_for_message(ser, "SYSTEM IS HOMED", timeout=120)

        print("Moving to CAM1...")
        send_command(ser, "cam1")
        wait_for_message(ser, "SYSTEM IS AT CAM1", timeout=60)

        # Start MJPEG preview server
        _start_mjpeg_server()

        # Start camera
        picam2 = Picamera2()
        cfg = picam2.create_preview_configuration(
            main={"size": (PREVIEW_W, PREVIEW_H), "format": "BGR888"}
        )
        picam2.configure(cfg)
        picam2.start()
        time.sleep(1)   # let exposure settle

        # ── Initial detection to sanity-check the board is visible ────────────
        print("Checking board is visible...")
        test_raw   = picam2.capture_array()
        test_frame = cv2.resize(test_raw, (PREVIEW_W, PREVIEW_H))
        test_frame = undistort_image(test_frame, CALIBRATION_NPZ_PATH)
        test_corners = _detect_corners(test_frame)

        if test_corners is None:
            print("WARNING: checkerboard not detected in initial frame.")
            print("Make sure the board is fully visible and well-lit.")
            print("The live view will still open — proceed if you can see the board.")
        else:
            print(f"Board detected: {len(test_corners)} corners found.")

        # ── Collect 3 anchor machine positions via live alignment ─────────────
        anchor_labels = [
            "top-LEFT  inner corner  (board origin)",
            f"top-RIGHT inner corner  ({(BOARD_COLS-1)*SQUARE_MM:.0f} mm along first row)",
            f"bottom-LEFT inner corner ({(BOARD_ROWS-1)*SQUARE_MM:.0f} mm down first column)",
        ]

        anchor_pixel_positions   = []
        anchor_machine_positions = []

        for anchor_idx, label in zip(ANCHOR_INDICES, anchor_labels):
            print(f"\n{'─'*60}")
            print(f"Step {len(anchor_machine_positions)+1}/3: align corner #{anchor_idx}")
            print(f"  {label}")
            print(f"{'─'*60}")

            result = live_align(ser, picam2, anchor_idx, label)
            if result is None:
                print("Calibration aborted.")
                picam2.stop()
                return

            machine_x, machine_y = result
            anchor_machine_positions.append([machine_x, machine_y])

            # Also record the pixel position at confirmation
            # (re-detect on a fresh frame at the confirmed position)
            time.sleep(0.3)
            frame = picam2.capture_array()
            frame = cv2.resize(frame, (PREVIEW_W, PREVIEW_H))
            frame = undistort_image(frame, CALIBRATION_NPZ_PATH)
            corners = _detect_corners(frame)
            if corners is not None:
                px, py = corners[anchor_idx]
                anchor_pixel_positions.append([float(px), float(py)])
                print(f"  Corner pixel at confirmation: ({px:.1f}, {py:.1f})")
            else:
                # Board lost — use image centre as fallback (should be near centre)
                anchor_pixel_positions.append([PREVIEW_W / 2.0, PREVIEW_H / 2.0])
                print("  WARNING: board not detected at confirmation frame — using image centre.")

        picam2.stop()

        # ── Solve homography ──────────────────────────────────────────────────
        print("\nComputing machine coordinates for all corners...")
        all_machine = _compute_machine_coords_all(anchor_machine_positions)

        # For pixel coordinates: we have 3 anchor pixels; the other 37 are
        # estimated by running detection on the last confirmed frame.
        # Reuse the test corners from the initial frame if they exist,
        # otherwise build from anchors only (3-point solve still produces H).
        if test_corners is not None and len(test_corners) == BOARD_COLS * BOARD_ROWS:
            print(f"Using all {len(test_corners)} detected corners for homography solve.")
            pixel_pts  = test_corners.tolist()
            machine_pts = all_machine.tolist()
        else:
            print("Using 3 anchor corners only (fewer points = less accurate H).")
            pixel_pts  = anchor_pixel_positions
            machine_pts = [all_machine[i].tolist() for i in ANCHOR_INDICES]

        print("Solving homography...")
        H = _solve_homography(pixel_pts, machine_pts)

        np.savez(HOMOGRAPHY_NPZ, H=H)
        print(f"\nH saved → {HOMOGRAPHY_NPZ}")
        print("H matrix:")
        print(H)

    finally:
        ser.close()
        print("Serial closed.")


if __name__ == "__main__":
    run()
