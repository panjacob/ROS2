#!/usr/bin/env bash
# Attach the TinyBee (CH340) to WSL via usbipd and flash it with PlatformIO.
# Usage: ./upload.sh [-m]      -m = open the serial monitor after uploading
set -euo pipefail

VIDPID="1a86:7523"   # CH340
cd "$(dirname "$0")"

USBIPD="$(command -v usbipd.exe || echo /mnt/c/Program\ Files/usbipd-win/usbipd.exe)"
PIO="$(command -v pio || echo "$HOME/.local/bin/pio")"
[ -x "$PIO" ] || PIO="$HOME/.platformio/penv/bin/pio"

find_tty() {
    for d in /sys/class/tty/ttyUSB*; do
        [ -e "$d" ] || continue
        local dev; dev="$(readlink -f "$d/device")"
        while [ "$dev" != "/" ]; do
            if [ -f "$dev/idVendor" ]; then
                [ "$(cat "$dev/idVendor"):$(cat "$dev/idProduct")" = "$VIDPID" ] && { echo "/dev/$(basename "$d")"; return 0; }
                break
            fi
            dev="$(dirname "$dev")"
        done
    done
    return 1
}

PORT="$(find_tty || true)"
if [ -z "$PORT" ]; then
    line="$("$USBIPD" list 2>/dev/null | tr -d '\r' | sed -n '/^Connected:/,/^Persisted:/p' | grep -i "$VIDPID" | head -n1 || true)"
    [ -n "$line" ] || { echo "Board ($VIDPID) not found - is it plugged in?" >&2; exit 1; }
    busid="$(awk '{print $1}' <<<"$line")"
    echo "Found board on busid $busid: $line"

    if grep -q "Not shared" <<<"$line"; then
        echo "Binding $busid (one-time, needs admin - accept the UAC prompt)..."
        powershell.exe -NoProfile -Command "Start-Process usbipd -Verb RunAs -Wait -ArgumentList 'bind','--busid','$busid'" >/dev/null
    fi
    if ! grep -q "Attached" <<<"$line"; then
        "$USBIPD" attach --wsl --busid "$busid"
    fi

    for _ in $(seq 20); do PORT="$(find_tty || true)"; [ -n "$PORT" ] && break; sleep 0.5; done
    [ -n "$PORT" ] || { echo "Attached, but no /dev/ttyUSB* appeared." >&2; exit 1; }
fi

[ -r "$PORT" ] && [ -w "$PORT" ] || echo "No access to $PORT - run: sudo usermod -aG dialout \$USER (then re-login)" >&2

echo "Uploading to $PORT"
"$PIO" run -t upload --upload-port "$PORT"

if [ "${1:-}" = "-m" ]; then
    "$PIO" device monitor -p "$PORT"
fi
