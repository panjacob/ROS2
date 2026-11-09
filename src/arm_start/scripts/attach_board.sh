#!/usr/bin/env bash
# Make sure the FluidNC board's USB serial is available in WSL.
# If the port is missing (board was unplugged / PC rebooted), attach it from Windows with usbipd-win.
#   attach_board.sh [port] [usb_vid:pid]      defaults: /dev/ttyUSB0 1a86:7523 (CH340 on MKS TinyBee)
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
"$USBIPD" attach --wsl --hardware-id "$HWID" || {
  echo "attach_board: attach failed - board plugged in? (first time only, as admin: usbipd bind --hardware-id $HWID)"
  exit 1
}
for _ in $(seq 50); do
  [ -e "$PORT" ] && { echo "attach_board: $PORT ready"; exit 0; }
  sleep 0.2
done
echo "attach_board: $PORT did not appear"; exit 1
