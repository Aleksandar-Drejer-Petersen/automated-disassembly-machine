"""
Interactive Arduino terminal.

- A background thread prints everything the Arduino sends as it arrives.
- The main thread accepts commands immediately — no waiting for the previous
  command to finish.
- Type 'exit' or 'q' to quit.
"""

import threading
import sys
from serial_comm import open_serial


def reader(ser, stop_event):
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                line = ser.readline().decode(errors="replace").strip()
                if line:
                    print(f"\r[ARD] {line}\n> ", end="", flush=True)
            else:
                stop_event.wait(timeout=0.02)
        except Exception:
            break


ser = open_serial()
stop_event = threading.Event()
t = threading.Thread(target=reader, args=(ser, stop_event), daemon=True)
t.start()

print("Type commands below. Output appears as it arrives. 'q' to quit.\n")

try:
    while True:
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd.lower() in ("exit", "q", "quit"):
            break

        if cmd:
            ser.write((cmd + "\n").encode())
            print(f"[PI ] {cmd}")

finally:
    stop_event.set()
    ser.close()
    print("Disconnected.")
