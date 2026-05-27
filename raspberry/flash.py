import subprocess
import os

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
arduino_dir  = os.path.join(project_root, "arduino")

print("=== Pulling latest code ===")
r = subprocess.run(["git", "pull"], cwd=project_root)
if r.returncode != 0:
    print("=== git pull FAILED — aborting ===")
    exit(1)

print("\n=== Compiling & uploading with PlatformIO ===")
result = subprocess.run(["pio", "run", "-t", "upload"], cwd=arduino_dir)

if result.returncode == 0:
    print("\n=== Done ===")
else:
    print("\n=== FAILED ===")
