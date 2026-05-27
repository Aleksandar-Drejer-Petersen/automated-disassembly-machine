"""
calib_diagnose.py

Tries every plausible board size on the already-captured calibration images
and reports which (cols, rows) combination finds corners.

Run on the Pi AFTER camera_tilt_calibration.py has captured images:
    python calib_diagnose.py
"""

import os
import cv2

IMAGE_DIR = "camera_calibration_images"

# All (cols, rows) combos to try — covers every plausible count for an 11x18 board
CANDIDATES = [
    (cols, rows)
    for cols in range(5, 22)
    for rows in range(5, 22)
    if cols != rows  # square boards are ambiguous, skip
]

def try_detect(img_path):
    img  = cv2.imread(img_path)
    if img is None:
        print(f"  Could not load {img_path}")
        return
    # Downscale to ~1200px wide — fast_check runs in ms at this resolution
    scale = 1200 / img.shape[1]
    img   = cv2.resize(img, (1200, int(img.shape[0] * scale)))
    gray  = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH |
             cv2.CALIB_CB_NORMALIZE_IMAGE  |
             cv2.CALIB_CB_FAST_CHECK)

    hits = []
    for cols, rows in CANDIDATES:
        ok, _ = cv2.findChessboardCorners(gray, (cols, rows), flags)
        if ok:
            hits.append((cols, rows))

    if hits:
        print(f"\n  FOUND corners in {os.path.basename(img_path)}:")
        for c, r in hits:
            print(f"    BOARD_COLS={c}  BOARD_ROWS={r}")
    else:
        print(f"\n  No corners found in {os.path.basename(img_path)} with any size tried.")

# Try on the first few saved images
images = sorted([
    os.path.join(IMAGE_DIR, f)
    for f in os.listdir(IMAGE_DIR)
    if f.endswith(".jpg") and f.startswith("cal_")
])[:5]

if not images:
    print(f"No images found in {IMAGE_DIR}/ — run camera_tilt_calibration.py first.")
else:
    print(f"Trying {len(CANDIDATES)} board-size combinations on {len(images)} image(s)...\n")
    for path in images:
        try_detect(path)
