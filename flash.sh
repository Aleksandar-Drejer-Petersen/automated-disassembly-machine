#!/bin/bash
set -e

echo "=== Compiling & uploading with PlatformIO ==="
cd arduino
pio run -t upload

echo ""
echo "=== Done ==="
