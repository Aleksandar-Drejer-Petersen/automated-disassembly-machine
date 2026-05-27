import cv2
import numpy as np
from camera import undistort_image
from config import CALIBRATION_NPZ_PATH, CAM1_ADJUSTED_RED_SQUARE_IMAGE_PATH, SCREW_CLASSES, PRESS_VERIFY_DARK_RATIO
from transform import (load_homography, load_affine,
                       pixel_to_machine_H, pixel_to_machine_T,
                       pixel_to_machine_simple)
from vision_draw import (MARKER_SIZE, MARKER_THICKNESS, RECT_THICKNESS,
                         ARROW_THICKNESS, TIP_LENGTH, FONT, FONT_SCALE, FONT_THICKNESS,
                         draw_text_bg, put_label, put_bottom_label)

_H = load_homography()   # best — perspective-correct, position-aware
_T = load_affine()       # fallback — linear, offset-from-centre


def reload_homography():
    """Reload H from disk (call after visual_calibration.py rewrites the file)."""
    global _H
    _H = load_homography()
    return _H


def _px_to_machine(px_abs, py_abs, img_cx, img_cy,
                   mm_per_pixel_x=None, mm_per_pixel_y=None):
    """
    Return (machine_dx, machine_dy): how far the machine must move from
    its current position to centre on the feature at pixel (px_abs, py_abs).

    With H: delta = H(feature_pixel) − H(image_centre_pixel).
      This correctly accounts for perspective — a pixel near the corner
      of the image maps to a different mm offset than the same pixel
      distance at the centre.  The subtraction cancels the absolute
      machine origin so the result is always a relative delta.

    With T or simple fallback: linear approximation using offset from centre.
    """
    if _H is not None:
        mx_feat,   my_feat   = pixel_to_machine_H(px_abs,  py_abs,  _H)
        mx_centre, my_centre = pixel_to_machine_H(img_cx,  img_cy,  _H)
        # Both axes negated: the homography maps pixel→machine in the
        # calibration direction (higher machine → lower pixel), so the
        # raw delta is always opposite to the move needed.
        return -(mx_feat - mx_centre), -(my_feat - my_centre)
    if _T is not None:
        return pixel_to_machine_T(px_abs, py_abs, img_cx, img_cy, _T)
    return pixel_to_machine_simple(px_abs, py_abs, img_cx, img_cy,
                                   mm_per_pixel_x, mm_per_pixel_y)


def classify_screw(diameter_mm):
    for name, (lo, hi) in SCREW_CLASSES.items():
        if lo <= diameter_mm < hi:
            return name
    return "Unknown"


def find_all_red_squares(image_path, mm_per_pixel_x, mm_per_pixel_y,
                         min_area_px=8000, debug_path=None):
    """
    Find ALL red squares visible in the image.
    Returns list of (machine_dx, machine_dy) — how far the machine must move
    from its current position to centre the camera on each square.
    If debug_path is given, saves an annotated image showing the camera centre
    crosshair and a bounding rectangle + marker for every detected square.
    """
    img = cv2.imread(image_path)
    if img is None:
        raise RuntimeError(f"Could not load image: {image_path}")

    img = undistort_image(img, CALIBRATION_NPZ_PATH)
    h, w = img.shape[:2]
    img_cx = w / 2
    img_cy = h / 2

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask1    = cv2.inRange(hsv, np.array([0,   120, 70]), np.array([10,  255, 255]))
    mask2    = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
    red_mask = cv2.bitwise_or(mask1, mask2)

    kernel   = np.ones((15, 15), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN,  kernel)

    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    squares   = []
    sq_pixels = []   # (rx, ry, rw, rh, sq_cx, sq_cy) kept for debug drawing
    for cnt in contours:
        if cv2.contourArea(cnt) < min_area_px:
            continue
        rx, ry, rw, rh = cv2.boundingRect(cnt)
        sq_cx = rx + rw / 2
        sq_cy = ry + rh / 2
        dx, dy = _px_to_machine(sq_cx, sq_cy, img_cx, img_cy,
                                mm_per_pixel_x, mm_per_pixel_y)
        squares.append((dx, dy))
        sq_pixels.append((rx, ry, rw, rh, sq_cx, sq_cy))

    print(f"  find_all_red_squares: {len(squares)} square(s) in {image_path}")

    if debug_path is not None:
        debug = img.copy()
        cxf, cyf = int(img_cx), int(img_cy)

        # Image-centre crosshair (cyan)
        cv2.drawMarker(debug, (cxf, cyf), (0, 255, 255),
                       cv2.MARKER_CROSS, MARKER_SIZE, MARKER_THICKNESS, cv2.LINE_AA)

        # Each detected square
        sq_color = (0, 120, 255)
        for i, (rx, ry, rw, rh, sq_cx, sq_cy) in enumerate(sq_pixels):
            scxf, scyf = int(sq_cx), int(sq_cy)
            # Bounding box
            cv2.rectangle(debug, (rx, ry), (rx + rw, ry + rh),
                          sq_color, RECT_THICKNESS, cv2.LINE_AA)
            # Square-centre crosshair
            cv2.drawMarker(debug, (scxf, scyf), sq_color,
                           cv2.MARKER_CROSS, MARKER_SIZE, MARKER_THICKNESS, cv2.LINE_AA)
            # Arrow from image centre to this square
            cv2.arrowedLine(debug, (cxf, cyf), (scxf, scyf),
                            (0, 165, 255), ARROW_THICKNESS,
                            tipLength=TIP_LENGTH, line_type=cv2.LINE_AA)
            # Square label pressed away from image centre
            put_label(debug, f"sq{i + 1}", scxf, scyf, cxf, cyf, sq_color)

        # Image-centre label pressed away from the first square
        if sq_pixels:
            put_label(debug, "image centre",
                      cxf, cyf, int(sq_pixels[0][4]), int(sq_pixels[0][5]),
                      (0, 255, 255))
        else:
            draw_text_bg(debug, "image centre",
                         cxf + MARKER_SIZE // 2 + 20, cyf, (0, 255, 255))

        n = len(sq_pixels)
        put_bottom_label(debug,
                         f"{n} square{'s' if n != 1 else ''} detected",
                         (0, 165, 255))

        cv2.imwrite(debug_path, debug)
        print(f"  Sweep debug saved: {debug_path}")

    return squares


def find_red_square_offset(image_path, mm_per_pixel_x, mm_per_pixel_y,
                           debug_path=None):
    """
    Locate the largest red square in image_path and return the machine correction
    (machine_dx, machine_dy) needed to centre the camera on it.
    If debug_path is given, saves an annotated image showing:
      - yellow crosshair at the camera/image centre
      - magenta crosshair at the detected square centre
      - orange arrow from camera centre to square centre
      - text label with dx, dy and the Euclidean correction distance in mm
    """
    img = cv2.imread(image_path)

    if img is None:
        raise RuntimeError(f"Could not load image from: {image_path}")

    img = undistort_image(img, CALIBRATION_NPZ_PATH)

    h, w = img.shape[:2]
    img_cx = w / 2
    img_cy = h / 2

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask1    = cv2.inRange(hsv, np.array([0,   120, 70]), np.array([10,  255, 255]))
    mask2    = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
    red_mask = cv2.bitwise_or(mask1, mask2)

    kernel   = np.ones((15, 15), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN,  kernel)

    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        raise RuntimeError("No red square found in image.")

    red_contour = max(contours, key=cv2.contourArea)
    rx, ry, rw, rh = cv2.boundingRect(red_contour)

    red_cx = rx + rw / 2
    red_cy = ry + rh / 2

    machine_dx, machine_dy = _px_to_machine(
        red_cx, red_cy, img_cx, img_cy, mm_per_pixel_x, mm_per_pixel_y
    )

    print(f"Red square center: ({red_cx:.1f}, {red_cy:.1f}) px")
    print(f"Red square machine correction: dx={machine_dx:+.2f}mm, dy={machine_dy:+.2f}mm")

    if debug_path is not None:
        debug  = img.copy()
        cxf    = int(img_cx)
        cyf    = int(img_cy)
        sqxf   = int(red_cx)
        sqyf   = int(red_cy)
        vec_mm = (machine_dx ** 2 + machine_dy ** 2) ** 0.5

        # Image-centre crosshair (cyan)
        cv2.drawMarker(debug, (cxf, cyf), (0, 255, 255),
                       cv2.MARKER_CROSS, MARKER_SIZE, MARKER_THICKNESS, cv2.LINE_AA)
        put_label(debug, "image centre", cxf, cyf, sqxf, sqyf, (0, 255, 255))

        # Square-centre crosshair (magenta)
        cv2.drawMarker(debug, (sqxf, sqyf), (255, 0, 255),
                       cv2.MARKER_CROSS, MARKER_SIZE, MARKER_THICKNESS, cv2.LINE_AA)
        put_label(debug, "square centre", sqxf, sqyf, cxf, cyf, (255, 0, 255))

        # Arrow from image centre to square centre
        cv2.arrowedLine(debug, (cxf, cyf), (sqxf, sqyf),
                        (0, 165, 255), ARROW_THICKNESS,
                        tipLength=TIP_LENGTH, line_type=cv2.LINE_AA)

        # Offset label — centred at 80 % height (subtitle style)
        put_bottom_label(debug,
                         f"dx={machine_dx:+.2f}  dy={machine_dy:+.2f}  |d|={vec_mm:.2f} mm",
                         (0, 165, 255))

        cv2.imwrite(debug_path, debug)
        print(f"  Centre debug saved: {debug_path}")

    return machine_dx, machine_dy


def check_press_success(image_path, debug_path=None):
    """
    Verify press success using the same yellow-detection logic as is_screw_present.

    The machine positions the camera directly above the press target, so the
    button/pin is at the image centre.  If is_screw_present finds no yellow
    (button gone) → success.  If it still detects yellow → failure.

    Returns (success: bool, yellow_fraction: float, threshold: float).
    """
    img = cv2.imread(image_path)
    if img is None:
        raise RuntimeError(f"check_press_success: cannot load {image_path}")

    img = undistort_image(img, CALIBRATION_NPZ_PATH)
    h, w = img.shape[:2]

    cx, cy = w // 2, h // 2
    radius = min(h, w) // 12

    # Crop ROI around centre so is_screw_present gets relative coords
    margin = radius * 3
    x1 = max(0, cx - margin)
    y1 = max(0, cy - margin)
    x2 = min(w, cx + margin)
    y2 = min(h, cy + margin)
    roi    = img[y1:y2, x1:x2]
    roi_cx = cx - x1
    roi_cy = cy - y1

    # Compute yellow fraction directly (mirror of is_screw_present)
    hsv          = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    yellow_mask  = cv2.inRange(hsv, np.array([15, 60, 100]), np.array([35, 255, 255]))
    inner_r      = int(radius * 0.5)
    inner_mask   = np.zeros(roi.shape[:2], dtype=np.uint8)
    cv2.circle(inner_mask, (roi_cx, roi_cy), inner_r, 255, -1)
    total_px     = int(np.count_nonzero(inner_mask))
    yellow_px    = int(np.count_nonzero(cv2.bitwise_and(yellow_mask, yellow_mask, mask=inner_mask)))
    fraction     = yellow_px / total_px if total_px > 0 else 0.0
    THRESHOLD    = 0.15   # same as is_screw_present default
    press_found  = fraction >= THRESHOLD
    success      = not press_found   # no button = press succeeded

    print(f"  check_press_success: yellow_fraction={fraction:.3f}  threshold={THRESHOLD}  "
          f"→ {'FAILED (button still present)' if press_found else 'SUCCESS (button gone)'}")

    if debug_path is not None:
        debug = img.copy()
        color = (0, 200, 0) if success else (0, 0, 220)
        cv2.circle(debug, (cx, cy), radius,  color,         RECT_THICKNESS,      cv2.LINE_AA)
        cv2.circle(debug, (cx, cy), inner_r, (200, 200, 0), RECT_THICKNESS // 2, cv2.LINE_AA)
        cv2.drawMarker(debug, (cx, cy), color,
                       cv2.MARKER_CROSS, MARKER_SIZE, MARKER_THICKNESS, cv2.LINE_AA)
        put_bottom_label(debug,
                         f"{'PASS' if success else 'FAIL'}  "
                         f"yellow={fraction:.2f}  threshold={THRESHOLD:.2f}",
                         color)
        cv2.imwrite(debug_path, debug)
        print(f"  Press verify debug saved: {debug_path}")

    return success, fraction, THRESHOLD


def check_press_at_pixel(image_path, px, py, radius, debug_path=None):
    """
    Check whether a press button is still present at pixel (px, py) in image_path.
    Uses the same yellow-detection logic as is_screw_present.
    Called during verification using the original detection image coordinates.

    Returns (success: bool, yellow_fraction: float, threshold: float).
    """
    img = cv2.imread(image_path)
    if img is None:
        raise RuntimeError(f"check_press_at_pixel: cannot load {image_path}")

    img = undistort_image(img, CALIBRATION_NPZ_PATH)
    h, w = img.shape[:2]

    margin  = radius * 3
    x1      = max(0, px - margin)
    y1      = max(0, py - margin)
    x2      = min(w, px + margin)
    y2      = min(h, py + margin)
    roi     = img[y1:y2, x1:x2]
    roi_cx  = px - x1
    roi_cy  = py - y1

    hsv         = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(hsv, np.array([15, 60, 100]), np.array([35, 255, 255]))
    inner_r     = int(radius * 0.5)
    inner_mask  = np.zeros(roi.shape[:2], dtype=np.uint8)
    cv2.circle(inner_mask, (roi_cx, roi_cy), inner_r, 255, -1)
    total_px    = int(np.count_nonzero(inner_mask))
    yellow_px   = int(np.count_nonzero(cv2.bitwise_and(yellow_mask, yellow_mask, mask=inner_mask)))
    fraction    = yellow_px / total_px if total_px > 0 else 0.0
    THRESHOLD   = 0.15
    success     = fraction < THRESHOLD

    print(f"  check_press_at_pixel ({px},{py}): yellow={fraction:.3f}  "
          f"→ {'SUCCESS (button gone)' if success else 'FAILED (button present)'}")

    if debug_path is not None:
        debug = img.copy()
        color = (0, 200, 0) if success else (0, 0, 220)
        cv2.circle(debug, (px, py), radius,  color,         RECT_THICKNESS,      cv2.LINE_AA)
        cv2.circle(debug, (px, py), inner_r, (200, 200, 0), RECT_THICKNESS // 2, cv2.LINE_AA)
        cv2.drawMarker(debug, (px, py), color,
                       cv2.MARKER_CROSS, MARKER_SIZE, MARKER_THICKNESS, cv2.LINE_AA)
        put_bottom_label(debug,
                         f"{'PASS' if success else 'FAIL'}  "
                         f"yellow={fraction:.2f}  threshold={THRESHOLD:.2f}",
                         color)
        cv2.imwrite(debug_path, debug)

    return success, fraction, THRESHOLD


def is_screw_present(roi_color, cx, cy, radius, fraction_threshold=0.15):
    hsv = cv2.cvtColor(roi_color, cv2.COLOR_BGR2HSV)

    yellow_mask = cv2.inRange(
        hsv,
        np.array([15,  60, 100]),
        np.array([35, 255, 255])
    )

    inner_radius = int(radius * 0.5)
    inner_mask   = np.zeros(roi_color.shape[:2], dtype=np.uint8)
    cv2.circle(inner_mask, (cx, cy), inner_radius, 255, -1)

    inside = cv2.bitwise_and(yellow_mask, yellow_mask, mask=inner_mask)

    total_pixels = np.count_nonzero(inner_mask)
    hit_pixels   = np.count_nonzero(inside)

    if total_pixels == 0:
        return False

    fraction = hit_pixels / total_pixels
    print(f"    Inner yellow fraction: {fraction:.2f}")

    return fraction >= fraction_threshold


def detect_screws(image_path, debug_path, mm_per_pixel_x, mm_per_pixel_y):
    img = cv2.imread(image_path)

    if img is None:
        raise RuntimeError(f"Could not load image from: {image_path}")

    img = undistort_image(img, CALIBRATION_NPZ_PATH)

    cv2.imwrite(CAM1_ADJUSTED_RED_SQUARE_IMAGE_PATH, img)
    print("Adjusted image saved.")

    h, w = img.shape[:2]
    img_cx = w / 2
    img_cy = h / 2

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask1    = cv2.inRange(hsv, np.array([0,   120, 70]), np.array([10,  255, 255]))
    mask2    = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
    red_mask = cv2.bitwise_or(mask1, mask2)

    kernel   = np.ones((15, 15), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN,  kernel)

    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        raise RuntimeError("No red square found in image.")

    red_contour = max(contours, key=cv2.contourArea)
    rx, ry, rw, rh = cv2.boundingRect(red_contour)

    red_cx = rx + rw / 2
    red_cy = ry + rh / 2

    red_offset_x_mm = (red_cx - img_cx) * mm_per_pixel_x
    red_offset_y_mm = (red_cy - img_cy) * mm_per_pixel_y

    print(f"Red square offset: dx={red_offset_x_mm:+.2f}mm, dy={red_offset_y_mm:+.2f}mm")

    margin = 90
    x1 = max(0, rx + margin)
    y1 = max(0, ry + margin)
    x2 = min(w, rx + rw - margin)
    y2 = min(h, ry + rh - margin)

    roi     = img[y1:y2, x1:x2]
    roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    roi_red_mask = cv2.bitwise_or(
        cv2.inRange(roi_hsv, np.array([0,   120, 70]), np.array([10,  255, 255])),
        cv2.inRange(roi_hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
    )

    thresh = cv2.bitwise_not(roi_red_mask)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN,   np.ones((20, 20), np.uint8))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE,  np.ones((10, 10), np.uint8))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_ERODE,  np.ones((15, 15), np.uint8))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_DILATE, np.ones((15, 15), np.uint8))

    all_contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    MIN_AREA        = 2000
    MAX_AREA        = 150000
    MIN_CIRCULARITY = 0.75

    screw_contours = []
    for cnt in all_contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA or area > MAX_AREA:
            continue
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue
        circularity = (4 * np.pi * area) / (perimeter ** 2)
        if circularity >= MIN_CIRCULARITY:
            screw_contours.append((cnt, area, circularity))

    # Find the centre mounting point: the circle closest to the centroid of all
    # detected circles.  This is the central pin/mounting post of the red-square
    # bracket that holds the PCB down — it should not be operated on.
    if screw_contours:
        cx_all = []
        cy_all = []
        for cnt, _, _ in screw_contours:
            (cx_roi, cy_roi), _ = cv2.minEnclosingCircle(cnt)
            cx_all.append(x1 + int(cx_roi))
            cy_all.append(y1 + int(cy_roi))
        pattern_cx = float(np.mean(cx_all))
        pattern_cy = float(np.mean(cy_all))
    else:
        pattern_cx, pattern_cy = red_cx, red_cy

    closest_idx  = None
    closest_dist = float("inf")
    for i, (cnt, area, circularity) in enumerate(screw_contours):
        (cx_roi, cy_roi), _ = cv2.minEnclosingCircle(cnt)
        cx = x1 + int(cx_roi)
        cy = y1 + int(cy_roi)
        dist = np.sqrt((cx - pattern_cx) ** 2 + (cy - pattern_cy) ** 2)
        if dist < closest_dist:
            closest_dist = dist
            closest_idx  = i

    # Only ignore the circle if it is genuinely close to the pattern centroid.
    # If nothing is close, the centre mounting post wasn't detected — don't
    # accidentally ignore a real screw.
    CENTRE_IGNORE_MAX_PX = 100
    if closest_idx is not None and closest_dist > CENTRE_IGNORE_MAX_PX:
        print(f"Centre post not found (nearest circle is {closest_dist:.1f}px from centroid) — not ignoring any circle.")
        closest_idx = None
    elif closest_idx is not None:
        print(f"Ignoring circle #{closest_idx + 1} (centre mounting point, dist from pattern centroid={closest_dist:.1f}px)")

    debug    = img.copy()
    img_cx_i = int(img_cx)
    img_cy_i = int(img_cy)
    red_cx_i = int(red_cx)
    red_cy_i = int(red_cy)

    # ROI bounding box
    cv2.rectangle(debug, (x1, y1), (x2, y2), (255, 60, 60), RECT_THICKNESS, cv2.LINE_AA)

    # Image-centre crosshair (cyan)
    cv2.drawMarker(debug, (img_cx_i, img_cy_i), (0, 255, 255),
                   cv2.MARKER_CROSS, MARKER_SIZE, MARKER_THICKNESS, cv2.LINE_AA)
    # Red-square centre crosshair (magenta)
    cv2.drawMarker(debug, (red_cx_i, red_cy_i), (255, 0, 255),
                   cv2.MARKER_CROSS, MARKER_SIZE, MARKER_THICKNESS, cv2.LINE_AA)
    # Arrow image centre → red centre
    cv2.arrowedLine(debug, (img_cx_i, img_cy_i), (red_cx_i, red_cy_i),
                    (255, 0, 255), ARROW_THICKNESS,
                    tipLength=TIP_LENGTH, line_type=cv2.LINE_AA)
    put_label(debug, "image centre", img_cx_i, img_cy_i, red_cx_i, red_cy_i, (0, 255, 255))
    put_label(debug, "red centre",   red_cx_i, red_cy_i, img_cx_i, img_cy_i, (255, 0, 255))

    # Red-square offset — bottom-centred subtitle
    put_bottom_label(debug,
                     f"dx={red_offset_x_mm:+.1f}  dy={red_offset_y_mm:+.1f} mm",
                     (255, 0, 255))

    screw_results = []
    screw_num = 0
    print(f"Screws detected: {len(screw_contours)} ({len(screw_contours) - 1} after ignoring closest to center)")

    for i, (cnt, area, circularity) in enumerate(screw_contours):
        if i == closest_idx:
            (cx_roi, cy_roi), radius = cv2.minEnclosingCircle(cnt)
            cx = x1 + int(cx_roi)
            cy = y1 + int(cy_roi)
            cv2.circle(debug, (cx, cy), int(radius), (128, 128, 128), RECT_THICKNESS, cv2.LINE_AA)
            draw_text_bg(debug, "ignored", cx + int(radius) + 20, cy, (128, 128, 128))
            continue

        (cx_roi, cy_roi), radius = cv2.minEnclosingCircle(cnt)
        cx_roi = int(cx_roi)
        cy_roi = int(cy_roi)
        radius = int(radius)
        cx = x1 + cx_roi
        cy = y1 + cy_roi

        machine_dx, machine_dy = _px_to_machine(
            cx, cy, img_cx, img_cy, mm_per_pixel_x, mm_per_pixel_y
        )
        if _H is not None:
            # Evaluate H Jacobian at the screw centre: 1-pixel step in X
            import cv2 as _cv2
            pt1 = np.array([[[float(cx),   float(cy)]]], dtype=np.float32)
            pt2 = np.array([[[float(cx)+1, float(cy)]]], dtype=np.float32)
            m1  = _cv2.perspectiveTransform(pt1, _H)[0, 0]
            m2  = _cv2.perspectiveTransform(pt2, _H)[0, 0]
            avg_mm_per_px = float(np.linalg.norm(m2 - m1))
        elif _T is not None:
            avg_mm_per_px = float(np.linalg.norm(_T)) / np.sqrt(2)
        else:
            avg_mm_per_px = ((mm_per_pixel_x or 0) + (mm_per_pixel_y or 0)) / 2
        diameter_mm = radius * 2 * avg_mm_per_px
        screw_type  = classify_screw(diameter_mm)
        has_press   = is_screw_present(roi, cx_roi, cy_roi, radius)

        screw_num += 1
        screw_results.append({
            "index":       screw_num,
            "cx":          cx,
            "cy":          cy,
            "radius":      radius,
            "machine_dx":  machine_dx,
            "machine_dy":  machine_dy,
            "diameter_mm": diameter_mm,
            "screw_type":  screw_type,
            "has_press":   has_press,
        })

        circle_color = (0, 255, 0) if has_press else (255, 100, 0)

        # Line from image centre to hole
        cv2.line(debug, (img_cx_i, img_cy_i), (cx, cy),
                 (180, 180, 180), max(1, ARROW_THICKNESS // 4), cv2.LINE_AA)
        # Hole circle + centre dot
        cv2.circle(debug, (cx, cy), radius, circle_color, RECT_THICKNESS, cv2.LINE_AA)
        cv2.circle(debug, (cx, cy), MARKER_THICKNESS, (0, 0, 220), cv2.FILLED, cv2.LINE_AA)

        if has_press:
            # ── Press operation ──────────────────────────────────────────────
            # No screw classification (M5/M6/diameter) — it's a press, not a screw.
            # Stack PRESS + number + offset ABOVE the upper button, BELOW the lower.
            info_text  = f"#{screw_num}   dx={machine_dx:+.1f}  dy={machine_dy:+.1f} mm"
            press_text = "PRESS"
            (iw, ih), _ = cv2.getTextSize(info_text,  FONT, FONT_SCALE, FONT_THICKNESS)
            (pw, ph), _ = cv2.getTextSize(press_text, FONT, FONT_SCALE, FONT_THICKNESS)
            gap = 24
            ix = max(8, min(w - iw - 8, cx - iw // 2))
            px = max(8, min(w - pw - 8, cx - pw // 2))
            if cy < img_cy:
                # Upper press: PRESS (top) → info → circle
                info_y  = cy - radius - 20
                press_y = info_y - ih - gap
            else:
                # Lower press: circle → info → PRESS (bottom)
                info_y  = cy + radius + ih + 20
                press_y = info_y + ph + gap
            info_y  = max(ih + 8, min(h - 8, info_y))
            press_y = max(ph + 8, min(h - 8, press_y))
            draw_text_bg(debug, info_text,  ix, info_y,  circle_color)
            draw_text_bg(debug, press_text, px, press_y, (0, 255, 255))
        else:
            # ── Regular screw ────────────────────────────────────────────────
            # Left-side screws: text to the LEFT  (right edge at circle edge)
            # Right-side screws: text to the RIGHT (left  edge at circle edge)
            type_text = f"#{screw_num} {screw_type} ({diameter_mm:.1f}mm)"
            off_text  = f"dx={machine_dx:+.1f}  dy={machine_dy:+.1f} mm"
            (t1w, t1h), _ = cv2.getTextSize(type_text, FONT, FONT_SCALE, FONT_THICKNESS)
            (t2w, _),   _ = cv2.getTextSize(off_text,  FONT, FONT_SCALE, FONT_THICKNESS)
            label_w = max(t1w, t2w)
            if cx < img_cx:
                lx = cx - radius - 20 - label_w   # right edge of text at left of circle
            else:
                lx = cx + radius + 20              # left  edge of text at right of circle
            lx = max(8, min(w - label_w - 8, lx))
            draw_text_bg(debug, type_text, lx, cy,            circle_color)
            draw_text_bg(debug, off_text,  lx, cy + t1h + 30, (200, 200, 200))

        print(
            f"  Hole {screw_num}: center=({cx},{cy})px, "
            f"machine_offset=({machine_dx:+.2f},{machine_dy:+.2f})mm, "
            f"diameter={diameter_mm:.2f}mm, type={screw_type}, "
            f"press={'YES' if has_press else 'no'}"
        )

    thresh_full = np.zeros((h, w), dtype=np.uint8)
    thresh_full[y1:y2, x1:x2] = thresh

    cv2.imwrite(debug_path.replace(".jpg", "_thresh3.jpg"), thresh_full)
    cv2.imwrite(debug_path, debug)
    print(f"Debug image saved to: {debug_path}")

    return screw_results, (red_offset_x_mm, red_offset_y_mm)
