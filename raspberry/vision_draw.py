"""
vision_draw.py

Shared report-quality drawing constants and helpers used by both vision.py
(normal run debug photos) and report_vision_photos.py.

All annotations across the codebase use these constants so every saved image
looks identical — thick lines, large crosshairs, dark-background text labels,
and bottom-centred offset measurements.
"""
import math
import cv2

# ── Report visual constants ──────────────────────────────────────────────────
MARKER_SIZE      = 220
MARKER_THICKNESS = 18
RECT_THICKNESS   = 16
ARROW_THICKNESS  = 18
TIP_LENGTH       = 0.025
FONT_SCALE       = 2.8
FONT_THICKNESS   = 8
FONT             = cv2.FONT_HERSHEY_SIMPLEX


def draw_text_bg(img, text, x, y, color):
    """Draw *text* at baseline (x, y) with a solid dark background rectangle."""
    (tw, th), baseline = cv2.getTextSize(text, FONT, FONT_SCALE, FONT_THICKNESS)
    pad = 14
    cv2.rectangle(img,
                  (x - pad, y - th - pad),
                  (x + tw + pad, y + baseline + pad),
                  (15, 15, 15), cv2.FILLED)
    cv2.putText(img, text, (x, y), FONT, FONT_SCALE, color, FONT_THICKNESS, cv2.LINE_AA)


def put_label(img, text, anchor_x, anchor_y, away_x, away_y, color):
    """
    Draw *text* near (anchor_x, anchor_y) pressed away from (away_x, away_y).

    The label is placed past the end of the crosshair arm that points away from
    the opposing point.  A dark background rectangle ensures readability over
    any pre-existing drawing.

    Coincident-point fallback: when the two points are (nearly) identical the
    label is pressed in a fixed direction based on which image quadrant the
    anchor sits in.
    """
    h_img, w_img = img.shape[:2]
    (tw, th), _ = cv2.getTextSize(text, FONT, FONT_SCALE, FONT_THICKNESS)

    dx = anchor_x - away_x
    dy = anchor_y - away_y
    dist = math.sqrt(dx ** 2 + dy ** 2)

    if dist < 1.0:
        # Coincident points — use quadrant-based fallback direction
        w2, h2 = w_img // 2, h_img // 2
        dx = 1.0 if anchor_x >= w2 else -1.0
        dy = 1.0 if anchor_y >= h2 else -1.0
        dist = math.sqrt(2)

    nx, ny = dx / dist, dy / dist

    step = MARKER_SIZE // 2 + 25
    lx = int(anchor_x + nx * step)
    ly = int(anchor_y + ny * step)

    # Align text so it doesn't overlap the crosshair arm
    if nx < 0:
        lx -= tw
    if ny < 0:
        ly -= th // 2
    else:
        ly += th + 10

    lx = max(8, min(w_img - tw - 8, lx))
    ly = max(th + 8, min(h_img - 8, ly))

    draw_text_bg(img, text, lx, ly, color)


def put_bottom_label(img, text, color, y_frac=0.82):
    """
    Draw *text* centred horizontally at *y_frac* of the image height.
    Uses a dark background rectangle — subtitle style.
    """
    h_img, w_img = img.shape[:2]
    (tw, th), baseline = cv2.getTextSize(text, FONT, FONT_SCALE, FONT_THICKNESS)
    x = max(8, (w_img - tw) // 2)
    y = int(h_img * y_frac)
    pad = 14
    cv2.rectangle(img,
                  (x - pad, y - th - pad),
                  (x + tw + pad, y + baseline + pad),
                  (15, 15, 15), cv2.FILLED)
    cv2.putText(img, text, (x, y), FONT, FONT_SCALE, color, FONT_THICKNESS, cv2.LINE_AA)
