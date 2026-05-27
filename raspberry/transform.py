import cv2
import numpy as np
import os

HOMOGRAPHY_NPZ_PATH    = "/home/alekpi/bachelor/automated-disassembly-machine/raspberry/output/homography.npz"
AFFINE_NPZ_PATH        = "/home/alekpi/bachelor/automated-disassembly-machine/raspberry/output/pixel_to_machine_transform.npz"
PERSPECTIVE_NPZ_PATH   = "/home/alekpi/bachelor/automated-disassembly-machine/raspberry/output/perspective_correction.npz"


def load_homography(path=HOMOGRAPHY_NPZ_PATH):
    if not os.path.exists(path):
        return None
    H = np.load(path)["H"]
    print(f"Homography H loaded from {path}")
    return H


def load_affine(path=AFFINE_NPZ_PATH):
    if not os.path.exists(path):
        return None
    T = np.load(path)["T"]
    print(f"Affine T loaded from {path}")
    return T


def pixel_to_machine_H(px_abs, py_abs, H):
    """
    Map an absolute image pixel position → absolute machine position using H.

    H accounts for perspective: pixels near the edge of the image are
    correctly mapped with a different effective mm/px than pixels at centre.

    px_abs, py_abs : absolute pixel coordinates in the (undistorted) image
    H              : 3×3 homography (from calibrate_homography.py)
    returns        : (machine_x_mm, machine_y_mm)
    """
    pt  = np.array([[[px_abs, py_abs]]], dtype=np.float32)
    out = cv2.perspectiveTransform(pt, H)
    return float(out[0, 0, 0]), float(out[0, 0, 1])


def pixel_to_machine_T(px_abs, py_abs, img_cx, img_cy, T):
    """
    Map pixel → machine using the 2×2 affine T (fallback if H not available).
    Operates on pixel offset from image centre, not absolute pixel.
    """
    vec = np.array([px_abs - img_cx, py_abs - img_cy], dtype=float)
    out = T @ vec
    return float(out[0]), float(out[1])


def pixel_to_machine_simple(px_abs, py_abs, img_cx, img_cy,
                             mm_per_pixel_x, mm_per_pixel_y):
    """
    Original fallback: independent scale factors + 90-degree axis swap.
    """
    machine_dx = -(py_abs - img_cy) * mm_per_pixel_y
    machine_dy = -(px_abs - img_cx) * mm_per_pixel_x
    return machine_dx, machine_dy


def apply_perspective_correction(abs_x, abs_y):
    """
    Apply the fitted linear perspective correction to absolute machine coords.

    If perspective_correction.npz exists (written by visual_calibration.py),
    applies:
        corrected_x = scale_x * abs_x + offset_x
        corrected_y = scale_y * abs_y + offset_y

    If the file does not exist or cannot be loaded, returns abs_x, abs_y
    unchanged (safe no-op).
    """
    if not os.path.exists(PERSPECTIVE_NPZ_PATH):
        return abs_x, abs_y
    try:
        npz      = np.load(PERSPECTIVE_NPZ_PATH)
        scale_x  = float(npz["scale_x"])
        offset_x = float(npz["offset_x"])
        scale_y  = float(npz["scale_y"])
        offset_y = float(npz["offset_y"])
        return scale_x * abs_x + offset_x, scale_y * abs_y + offset_y
    except Exception as e:
        print(f"  Warning: perspective_correction.npz load failed: {e}")
        return abs_x, abs_y
