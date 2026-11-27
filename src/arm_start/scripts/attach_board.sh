#!/usr/bin/env bash
# Make sure a USB device (NanoBee board, camera, ...) is available in WSL.
# If the device node is missing (unplugged / PC rebooted), attach it from Windows with usbipd-win.
#   attach_board.sh [device] [usb_vid:pid]    defaults: /dev/ttyUSB0 1a86:7523 (CH340 on MKS TinyBee)
#   (the camera uses attach_camera.sh, which also loads its driver)
PORT=${1:-/dev/ttyUSB0}
HWID=${2:-1a86:7523}
USBIPD="/mnt/c/Program Files/usbipd-win/usbipd.exe"

[ -e "$PORT" ] && exit 0
if ! grep -qi microsoft /proc/version; then
  echo "attach_board: $PORT missing - is the board plugged in?"; exit 1
fi
if [ ! -x "$USBIPD" ]; then
  echo "attach_board: $PORT missing and usbipd-win not found (winget install usbipd)"; exit 1
fi
echo "attach_board: $PORT missing, attaching USB $HWID from Windows..."
attach() { "$USBIPD" attach --wsl --hardware-id "$HWID"; }
if ! attach; then
  # Plugged in but not shared (never bound, or moved to another USB port): allow usbipd to share
  # this vid:pid on any port without admin rights from now on. One UAC prompt, only once.
  if "$USBIPD" list 2>/dev/null | tr -d '\r' | grep -qiE "^[0-9-]+ +$HWID .*Not shared *$" \
      && ! "$USBIPD" policy list 2>/dev/null | grep -qi "$HWID"; then
    echo "attach_board: allowing usbipd to share USB $HWID (one-time; confirm the Windows admin prompt)..."
    powershell.exe -NoProfile -Command "Start-Process -Wait -Verb RunAs usbipd -ArgumentList \
'policy','add','--effect','Allow','--operation','AutoBind','--hardware-id','$HWID'" </dev/null
  fi
  attach || {
    echo "attach_board: attach failed - is USB $HWID plugged in? (or as admin: usbipd bind --hardware-id $HWID)"
    exit 1
  }
fi
for _ in $(seq 50); do
  [ -e "$PORT" ] && { echo "attach_board: $PORT ready"; exit 0; }
  sleep 0.2
done
echo "attach_board: $PORT did not appear (missing kernel driver? camera: camera_driver.sh)"; exit 1
