import cv2
import numpy as np
import os
import subprocess
from config import CALIBRATION_NPZ_PATH


def capture_image(output_path):
    print("Capturing image...")

    result = subprocess.run(
        [
            "rpicam-still",
            "-o", output_path,
            "--width",  "4608",
            "--height", "2592",
        ],
        capture_output=True,
        text=True
    )

    if result.returncode != 0:
        raise RuntimeError(f"rpicam-still failed:\n{result.stderr}")

    if not os.path.exists(output_path):
        raise RuntimeError(f"Image not saved at: {output_path}")

    print(f"Image captured and saved to: {output_path}")


def load_mm_per_pixel(txt_path):
    mm_x, mm_y = None, None

    with open(txt_path, "r") as f:
        for line in f:
            if "mm_per_pixel_x" in line:
                mm_x = float(line.split(":")[1].strip())
            elif "mm_per_pixel_y" in line:
                mm_y = float(line.split(":")[1].strip())

    if mm_x is None or mm_y is None:
        raise RuntimeError("Could not parse mm_per_pixel values from txt file.")

    print(f"Scale loaded: mm_per_pixel_x={mm_x}, mm_per_pixel_y={mm_y}")
    return mm_x, mm_y


def undistort_image(img, npz_path=CALIBRATION_NPZ_PATH):
    if not os.path.exists(npz_path):
        return img

    data = np.load(npz_path)

    camera_matrix = data["camera_matrix"]
    dist_coeffs   = data["dist_coeffs"]

    h, w = img.shape[:2]

    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix,
        dist_coeffs,
        (w, h),
        1,
        (w, h)
    )

    undistorted = cv2.undistort(
        img,
        camera_matrix,
        dist_coeffs,
        None,
        new_camera_matrix
    )

    x, y, rw, rh = roi
    undistorted = undistorted[y:y+rh, x:x+rw]

    print(f"Image undistorted. New size: {undistorted.shape[1]} x {undistorted.shape[0]} px")
    return undistorted
