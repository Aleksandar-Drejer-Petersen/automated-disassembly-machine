"""
Unscrew reliability tester.

Flow:
  1. Home all axes.
  2. Press Enter to begin a cycle.
  3. Machine unscrews.
  4. Z homes, X moves +100 mm so you can inspect the result.
  5. Press Enter = success,  type c + Enter = failure.
  6. X moves back -100 mm, repeat from step 3.
  7. Ctrl-C to quit — prints final score.
"""

import threading
import time
import sys
from serial_comm import open_serial

# ── serial helpers ─────────────────────────────────────────────────────────────
def send(ser, cmd):
    ser.write((cmd.strip() + "\n").encode())
    print(f"  → {cmd.strip()}")


def wait_for(ser, keyword, timeout=60.0):
    """Block until a line containing keyword arrives or timeout."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ser.in_waiting:
            try:
                line = ser.readline().decode(errors="replace").strip()
            except Exception:
                continue
            if line:
                print(f"  ← {line}")
                if keyword.upper() in line.upper():
                    return True
        else:
            time.sleep(0.02)
    print(f"  [TIMEOUT waiting for '{keyword}']")
    return False


def wait_for_any(ser, keywords, timeout=60.0):
    """Block until any of the keywords appear. Returns the matched keyword."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ser.in_waiting:
            try:
                line = ser.readline().decode(errors="replace").strip()
            except Exception:
                continue
            if line:
                print(f"  ← {line}")
                for kw in keywords:
                    if kw.upper() in line.upper():
                        return kw
        else:
            time.sleep(0.02)
    print(f"  [TIMEOUT waiting for any of {keywords}]")
    return None


def wait_for_idle(ser, timeout=60.0):
    """Poll STATUS until machine reports IDLE."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        time.sleep(0.5)
        send(ser, "STATUS")
        if wait_for(ser, "STATUS IDLE", timeout=2.0):
            return True
    return False


# ── main ───────────────────────────────────────────────────────────────────────
ser = open_serial()
successes = 0
failures  = 0
cycle     = 0

try:
    # ── home all axes ──────────────────────────────────────────────────────────
    print("\n=== Homing all axes ===")
    send(ser, "H")
    wait_for(ser, "HOME: all axes homed", timeout=120)
    print("Homed.\n")

    print("=== Moving to SCREW1 ===")
    send(ser, "SCREW1")
    wait_for(ser, "SYSTEM IS AT SCREW1", timeout=60)
    print("At SCREW1.\n")

    input("Press Enter when ready to start the first unscrew cycle …\n")

    while True:
        cycle += 1
        print(f"\n{'='*50}")
        print(f"  CYCLE {cycle}   |   successes: {successes}   failures: {failures}")
        print(f"{'='*50}\n")

        # ── unscrew ────────────────────────────────────────────────────────────
        print("--- Unscrewing ---")
        send(ser, "UNSCREW")
        result = wait_for_any(
            ser,
            ["UNSCREW: done", "control released", "UNSCREW ERROR"],
            timeout=120
        )
        if result is None:
            print("  [unscrew timed out]")

        # ── home Z ─────────────────────────────────────────────────────────────
        print("\n--- Homing Z ---")
        send(ser, "H Z")
        wait_for(ser, "Axis Z safe", timeout=30)

        # ── move X +100 ────────────────────────────────────────────────────────
        print("\n--- Moving X +100 mm for inspection ---")
        send(ser, "x 100")
        wait_for_idle(ser, timeout=30)

        # ── user judges result ─────────────────────────────────────────────────
        print("\nInspect the screw.")
        print("  Enter = success     c + Enter = failure")
        verdict = input("> ").strip().lower()

        if verdict == "c":
            failures += 1
            print(f"  Logged as FAILURE  (total: {successes} ok / {failures} fail)")
        else:
            successes += 1
            print(f"  Logged as SUCCESS  (total: {successes} ok / {failures} fail)")

        # ── move X -100 back ───────────────────────────────────────────────────
        print("\n--- Moving X back -100 mm ---")
        send(ser, "x -100")
        wait_for_idle(ser, timeout=30)

except KeyboardInterrupt:
    pass

finally:
    total = successes + failures
    rate  = (successes / total * 100) if total > 0 else 0.0
    print(f"\n{'='*50}")
    print(f"  FINAL SCORE after {total} cycle(s)")
    print(f"  Successes : {successes}")
    print(f"  Failures  : {failures}")
    print(f"  Success rate: {rate:.1f}%")
    print(f"{'='*50}\n")
    ser.close()
