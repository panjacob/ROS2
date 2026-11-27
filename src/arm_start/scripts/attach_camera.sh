#!/usr/bin/env bash
# Make the USB camera available in WSL: load its driver (camera_driver.sh), then attach it from
# Windows with usbipd (attach_board.sh). camera_node runs this whenever its device is missing:
# at start, after a replug and after a WSL restart.
#   attach_camera.sh [device] [usb_vid:pid]   defaults: LifeCam VX-6000
DEV=${1:-/dev/v4l/by-id/usb-045e_USB20_Camera-video-index0}
HWID=${2:-045e:00f4}
DIR=$(dirname "$(realpath "$0")")
USBIPD="/mnt/c/Program Files/usbipd-win/usbipd.exe"

wait_dev() {
  for _ in $(seq "$1"); do [ -e "$DEV" ] && return 0; sleep 0.2; done
  return 1
}

[ -e "$DEV" ] && exit 0
if ! grep -qi microsoft /proc/version; then
  echo "attach_camera: $DEV missing - is the camera plugged in?"; exit 1
fi
if [ "${HWID,,}" = 045e:00f4 ]; then
  "$DIR/camera_driver.sh" || exit 1
  wait_dev 15 && { echo "attach_camera: $DEV ready"; exit 0; }
fi
# Attached without a device node (e.g. the driver was missing then): re-attach so the driver binds.
if [ -x "$USBIPD" ] && "$USBIPD" list 2>/dev/null | tr -d '\r' \
    | grep -qiE "^[0-9-]+ +$HWID .*Attached *$"; then
  echo "attach_camera: USB $HWID attached but $DEV missing, re-attaching..."
  "$USBIPD" detach --hardware-id "$HWID" >/dev/null 2>&1
  sleep 1
fi
exec "$DIR/attach_board.sh" "$DEV" "$HWID"
