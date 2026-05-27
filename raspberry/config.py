VISION_PHOTOS_DIR     = "/home/alekpi/bachelor/automated-disassembly-machine/raspberry/vision_photos"
VISION_CURRENT_RUN_DIR = "/home/alekpi/bachelor/automated-disassembly-machine/raspberry/vision_current_run"
UNSCREW_PHOTOS_DIR = "/home/alekpi/bachelor/automated-disassembly-machine/raspberry/unscrew_photos"
MAIN_ANALYSIS_DIR  = "/home/alekpi/bachelor/automated-disassembly-machine/raspberry/main_analysis_photos"

DEBUG_IMAGE_PATH      = "/home/alekpi/bachelor/automated-disassembly-machine/raspberry/vision_photos/red_square_contour.jpg"
CALIBRATION_NPZ_PATH  = "/home/alekpi/bachelor/automated-disassembly-machine/raspberry/output/camera_calibration.npz"
MM_PER_PIXEL_TXT_PATH = "/home/alekpi/bachelor/automated-disassembly-machine/raspberry/output/top_height_result.txt"

CAM1_RED_SQUARE_IMAGE_PATH          = "/home/alekpi/bachelor/automated-disassembly-machine/raspberry/vision_photos/cam1_red_square_img.jpg"
CAM1_ADJUSTED_RED_SQUARE_IMAGE_PATH = "/home/alekpi/bachelor/automated-disassembly-machine/raspberry/vision_photos/cam1_adjusted_red_square_img.jpg"

SERIAL_PORT    = "/dev/ttyACM0"
BAUD_RATE      = 115200
SERIAL_TIMEOUT = 0.1

CAM1_X = 95.0
CAM1_Y = 0.0

CAMERA_OFFSET_X_MM = -2.137
CAMERA_OFFSET_Y_MM = -60.789

AUTO_START_UNSCREW_ALL = True

# A press operation is successful when the surface moved at least this far
# from the point of contact. The force limit is a safety stop only.
PRESS_SUCCESS_SURFACE_MM = 5.0

PRESS_VERIFY_DARK_RATIO = 0.65   # inner/outer brightness ratio — below = hole is dark = press succeeded

SCREW_CLASSES = {
    "M1": (1.5,  2.5),
    "M2": (2.5,  4.0),
    "M3": (4.0,  6.2),
    "M4": (6.2,  7.8),
    "M5": (7.8,  10.2),
    "M6": (10.2, 12.8),
}
