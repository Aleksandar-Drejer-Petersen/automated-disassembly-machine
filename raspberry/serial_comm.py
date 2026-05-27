import serial
import time
from difflib import SequenceMatcher
from config import SERIAL_PORT, BAUD_RATE, SERIAL_TIMEOUT

# Minimum similarity ratio to fuzzy-accept a received line as a known message.
# At 0.85 a 1-2 byte dropout/corruption is still accepted, while distinct
# messages (e.g. "GRAB: complete." vs "PLACE: complete.") score ~0.71 and
# are correctly rejected.
_FUZZY_THRESHOLD = 0.85


def _is_fuzzy_match(candidate, line, threshold=_FUZZY_THRESHOLD):
    """Return True if candidate is an exact substring of line, or the two
    strings are similar enough to be the same message with 1-2 corrupted
    or dropped bytes."""
    cu, lu = candidate.upper(), line.upper()
    if cu in lu:
        return True
    if not cu or not lu:
        return False
    return SequenceMatcher(None, cu, lu).ratio() >= threshold


def _is_unknown_command(line):
    return "UNKNOWN COMMAND RECEIVED" in line.upper()


def _is_laser_stream(line):
    """Return True for high-frequency laser telemetry lines (L:<ts>:<mm>).
    These are suppressed from the console to avoid flooding the output."""
    return line.startswith("L:") and line.count(":") == 2


def open_serial():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)
    time.sleep(2)
    ser.reset_input_buffer()
    print(f"Serial opened: {SERIAL_PORT} @ {BAUD_RATE}")
    return ser


def send_command(ser, cmd):
    # Sleep 60ms then drain the Pi-side buffer so the 16U2 TX path is clear before we write.
    # The 16U2 USB bridge drops incoming (Pi→Arduino) bytes when its TX endpoint is stalled,
    # which happens when the Pi OS buffer is full and hasn't been read yet.
    time.sleep(0.06)
    ser.reset_input_buffer()
    full = cmd.strip() + "\n"
    ser.write(full.encode())
    print(f"[PI → ARD] {cmd.strip()}")


def read_all_lines(ser, timeout=0.5):
    lines = []
    deadline = time.time() + timeout

    while time.time() < deadline:
        if ser.in_waiting:
            try:
                line = ser.readline().decode(errors="replace").strip()
                if line:
                    if not _is_laser_stream(line):
                        print(f"[ARD → PI] {line}")
                    lines.append(line)
            except Exception:
                pass
        else:
            time.sleep(0.05)

    return lines


def wait_for_message(ser, expected, timeout=120):
    print(f"Waiting for: '{expected}'...")
    deadline = time.time() + timeout

    while time.time() < deadline:
        if ser.in_waiting:
            try:
                line = ser.readline().decode(errors="replace").strip()
                if line:
                    if not _is_laser_stream(line):
                        print(f"[ARD → PI] {line}")
                    if _is_unknown_command(line):
                        raise TimeoutError(
                            f"Arduino rejected the command (corrupted TX). "
                            f"Expected '{expected}', got: {line}"
                        )
                    if _is_fuzzy_match(expected, line):
                        print(f"Got: '{expected}'")
                        return True
            except TimeoutError:
                raise
            except Exception:
                pass
        else:
            time.sleep(0.05)

    raise TimeoutError(f"Did not receive '{expected}' within {timeout}s")


def wait_for_any_message(ser, candidates, timeout=120):
    """Wait until any of the candidate strings appears in a serial line.
    Returns the matched candidate string, 'UNKNOWN COMMAND' if Arduino
    rejected the command due to a corrupted transmission, or None on timeout."""
    print(f"Waiting for any of: {candidates} ...")
    deadline = time.time() + timeout

    while time.time() < deadline:
        if ser.in_waiting:
            try:
                line = ser.readline().decode(errors="replace").strip()
                if line:
                    if not _is_laser_stream(line):
                        print(f"[ARD → PI] {line}")
                    if _is_unknown_command(line):
                        print("  WARNING: Arduino rejected command (corrupted transmission).")
                        return "UNKNOWN COMMAND"
                    for original in candidates:
                        if _is_fuzzy_match(original, line):
                            print(f"Got: '{original}'")
                            return original
            except Exception:
                pass
        else:
            time.sleep(0.05)

    return None


def send_command_and_get_response(ser, cmd, timeout=3.0, stop_keywords=None):
    if stop_keywords is None:
        stop_keywords = []

    ser.reset_input_buffer()

    full = cmd.strip() + "\n"
    ser.write(full.encode())
    print(f"[PI → ARD] {cmd.strip()}")

    lines = []
    deadline = time.time() + timeout

    while time.time() < deadline:
        if ser.in_waiting:
            try:
                line = ser.readline().decode(errors="replace").strip()
                if line:
                    if not _is_laser_stream(line):
                        print(f"[ARD → PI] {line}")
                    lines.append(line)
                    for keyword in stop_keywords:
                        if keyword.upper() in line.upper():
                            return lines
            except Exception as e:
                print(f"Serial read error: {e}")
        else:
            time.sleep(0.03)

    return lines


def manual_command_mode(ser):
    print("\n=== MANUAL COMMAND MODE ===")
    print("Type any Arduino command exactly like in Serial Monitor.")
    print("Examples: h, cam1, x 10, y -5, LASER, FORCE, GET POSITION")
    print("Type 'exit' or 'q' to quit.\n")

    while True:
        cmd = input("Arduino command > ").strip()

        if cmd.lower() in ["exit", "q", "quit"]:
            print("Leaving manual command mode.")
            break

        if cmd == "":
            continue

        response = send_command_and_get_response(ser, cmd, timeout=5.0)

        print("\n--- RESPONSE RETURNED TO PYTHON ---")
        for line in response:
            print(line)
        print("-----------------------------------\n")
