from serial_comm import send_command, read_all_lines
from config import CAMERA_OFFSET_X_MM, CAMERA_OFFSET_Y_MM


def send_detected_screws_to_arduino(ser, screw_results, red_square_x, red_square_y):
    print("\n=== SENDING DETECTED SCREWS TO ARDUINO ===")

    send_command(ser, "CLEAR SCREWS")
    read_all_lines(ser, timeout=1.0)

    sent_count = 0

    for i, s in enumerate(screw_results, start=1):
        if i > 6:
            print(f"Skipping screw #{i}: Arduino only supports 6 screw positions")
            continue

        if s["screw_type"] == "Unknown":
            print(f"Skipping screw #{i}: unknown screw type")
            continue

        # machine_dx/dy are already in machine coordinate frame (transform applied in vision.py)
        camera_screw_x = red_square_x + s["machine_dx"]
        camera_screw_y = red_square_y + s["machine_dy"]

        screw_abs_x = camera_screw_x - CAMERA_OFFSET_X_MM
        screw_abs_y = camera_screw_y - CAMERA_OFFSET_Y_MM

        cmd = (
            f"SET SCREW {i} {s['screw_type']} "
            f"{screw_abs_x:.3f} {screw_abs_y:.3f}"
        )

        send_command(ser, cmd)
        read_all_lines(ser, timeout=1.0)

        print(
            f"Sent screw #{i}: type={s['screw_type']}, "
            f"camera_position=(X={camera_screw_x:.3f}, Y={camera_screw_y:.3f}), "
            f"shaft_corrected=(X={screw_abs_x:.3f}, Y={screw_abs_y:.3f})"
        )

        sent_count += 1

    print(f"Detected screws sent to Arduino: {sent_count}")
    return sent_count
