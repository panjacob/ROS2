# NanoBee – build & flash in VS Code

Minimal robotic-arm firmware for the MKS TinyBee (ESP32).

## 1. One-time setup

1. Install [VS Code](https://code.visualstudio.com/).
2. In VS Code open **Extensions** (`Ctrl+Shift+X`), install **PlatformIO IDE**, restart VS Code when asked.
3. Install [Git](https://git-scm.com/download/win) and make sure `git` is on PATH (the ESP32 platform needs it during build).
4. If Windows does not show a COM port when the board is plugged in, install the board's USB-serial driver (CH340).

## 2. Open the project

`File > Open Folder...` and select the `NanoBee` folder (the one containing `platformio.ini`).
The first open downloads the ESP32 toolchain; wait until PlatformIO finishes (status bar).

## 3. Build / upload / monitor

Use the PlatformIO buttons in the bottom status bar, or the PlatformIO sidebar (alien icon) under `tinybee`:

| Action  | Status bar | Terminal command |
|---------|-----------|------------------|
| Build   | ✓ (check mark) | `pio run` |
| Upload  | → (arrow) | `pio run -t upload` |
| Serial monitor | 🔌 (plug) | `pio device monitor` |
| Clean   | 🗑 (trash) | `pio run -t clean` |

Upload picks the COM port automatically. If there are several ports, add `upload_port = COM5` (your port) to `platformio.ini`.

If `pio` is not found in a normal terminal, use the PlatformIO terminal (sidebar > *New Terminal*) or the full path:
`& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run`

> Flashing replaces whatever firmware is on the board. Keep the motor power off or the arm clear for the first start.

## 4. First start

- Serial monitor (115200) should print `NanoBee ready` and the Wi-Fi address.
- Default Wi-Fi: network `NanoBee`, password `nanobee123`, page at http://192.168.4.1/ (or http://nanobee.local/).
- Send `?` in the serial monitor to get the status; command list is at the top of `src/main.cpp`.

## 5. Settings

Runtime settings (speeds, accelerations, steps per degree, Wi-Fi, homing) live in
`../NanoBeeDriver/config.yaml` and are uploaded without recompiling – via the web page (*Settings > Save & apply*)
or from Python: `NanoBee("192.168.4.1", config="config.yaml").push_config()`.
Pin wiring is compile-time only, in `src/config.h`.
