#!/usr/bin/env bash
# Build (if needed) and load the LifeCam VX-6000 driver for the running WSL kernel.
# arm.launch.py does this automatically; this is the manual entry point.
#   tools/wsl_camera_driver.sh [--rebuild]
exec "$(dirname "$(realpath "$0")")/../src/arm_start/scripts/camera_driver.sh" "$@"