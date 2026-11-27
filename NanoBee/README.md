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

Upload picks the serial port automatically. If there are several ports, pass it explicitly: `pio run -t upload --upload-port COM5` (Windows) or `--upload-port /dev/ttyUSB0` (WSL).

If `pio` is not found in a normal terminal, use the PlatformIO terminal (sidebar > *New Terminal*) or the full path:
`& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run`

### Flashing from WSL

WSL has no `COMx` ports – the board must be passed through with [usbipd-win](https://github.com/dorssel/usbipd-win)
and then shows up as `/dev/ttyUSB0`.

**Easiest:** `./upload.sh` (or `./upload.sh -m` to open the serial monitor afterwards) finds the CH340 board,
binds/attaches it with usbipd if needed and uploads to the right `/dev/ttyUSB*`.

Manual steps:

1. Windows PowerShell (admin): `usbipd list` and note the CH340 bus id (e.g. `2-1`).
   Once: `usbipd bind --busid 2-1`. After every replug: `usbipd attach --wsl --busid 2-1`.
2. WSL: check `ls /dev/ttyUSB*`. If you get *permission denied*: `sudo usermod -aG dialout $USER`, then log in again.
3. WSL:
   ```
   cd ~/ROS2/NanoBee
   pio run -t upload --upload-port /dev/ttyUSB0
   pio device monitor -p /dev/ttyUSB0
   ```

While attached to WSL the port is not available on Windows (and vice versa: `usbipd detach --busid 2-1`).

> Flashing replaces whatever firmware is on the board. Keep the motor power off or the arm clear for the first start.

## 4. First start

- Serial monitor (115200) should print `# NanoBee ready` and the Wi-Fi address.
- Default Wi-Fi: network `NanoBee`, password `nanobee123`, page at http://192.168.4.1/ (or http://nanobee.local/).
- Send `?` in the serial monitor to get the status; command list is at the top of `src/main.cpp`.

## 5. Settings

Runtime settings (speeds, accelerations, steps per degree, homing, soft limits, Wi-Fi) live in
`NanoBeeDriver/config.yaml` and are uploaded without recompiling – via the web page (*Settings > Save & apply*)
or from Python: `NanoBee(port="COM6", config="config.yaml").push_config()`.
Pin wiring is compile-time only, in `src/config.h`. The values the board actually uses (file + defaults)
are printed by `SETTINGS?` (serial) or `GET /settings`; the ROS bridge reads them from there at startup.

Key settings for the arm (joint settings live in per-joint blocks `joints.joint1`..`joint5`; the older
one-list-per-key form is still accepted):

| Key | Meaning |
|-----|---------|
| `motion.blend` | `true`: consecutive `T` segments run back to back without stopping (smooth ROS trajectories) |
| `joints.jointN.idle_disable`, `motion.idle_timeout_ms` | switch the driver off after this long at rest (cooler, less power); wakes on the next move, keeps the home reference (status `O:`) |
| `joints.jointN.homing` | `none`, `edge` (switch = `home_offset_deg`) or `center` (switch hit from both sides, middle = `home_offset_deg`) |
| `joints.jointN.homing_dir` | direction of the first approach to the switch |
| `joints.jointN.homing_speed`, `homing_max_travel` | optional per-joint homing seek speed (steps/s) and search distance (steps); `0` = `homing.speed` / `homing.max_travel` |
| `joints.soft_limits`, `jointN.min_deg`, `jointN.max_deg` | moves outside the range are rejected (`err limit`) and jogs stop at the edge, once the joint is homed or set with `Z`. `arm.launch.py` also uses min/max as the ROS (URDF) joint limits – relaunch after changing them |
| `homing.enabled` | `false` rejects `H` |

## 6. Protocol (USB serial 115200 or `GET /cmd?c=...`)

One command per line, positions in steps. Every command is answered with `ok` or `err <reason>`;
lines starting with `# ` are asynchronous messages (`# homed 0`, `# err home 0: ...`, `# halted`,
`# warn ...`). The full list is at the top of `src/main.cpp`.

| Command | Action |
|---------|--------|
| `M p0..p4 [speed [accel]]` / `R d0..d4 ...` | queued absolute / relative move, stops at the end |
| `T p0..p4 dt_ms` | queued timed segment, constant speed, blends with the next `T`; `err full` = queue full, retry |
| `J v0..v4 [accel]` | jog (steps/s), repeat within `jog_timeout_ms` |
| `S` / `!` | smooth stop / instant halt (both drop the queue) |
| `E 0\|1`, `Z [p0..p4]`, `H [j]`, `V`, `G us` | drivers (enabled at boot), set position, home (all or one), defaults, servo |
| `?` | `<Idle\|Run\|Jog\|Home P:p0,..,p4 Q:queued F:free E:enabled L:endstops H:referenced O:powered>` |
| `VER` | `NanoBee 2 joints=5` |
| `CONFIG` ... `END`, `CONFIG?`, `SETTINGS?`, `REBOOT` | serial only: upload / dump stored / dump effective settings, reboot |
