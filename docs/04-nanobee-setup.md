# 4. NanoBee setup

The TinyBee runs **NanoBee**, this project's own minimal arm firmware
([NanoBee/](../NanoBee)). Build details and the full protocol: [NanoBee/README.md](../NanoBee/README.md).

## 4.1 Flash the firmware

1. Install VS Code + the **PlatformIO IDE** extension (and Git), see [NanoBee/README.md](../NanoBee/README.md#1-one-time-setup).
2. The board must be visible to **Windows**: if it is attached to WSL, run `usbipd detach --busid <id>`
   (or unplug/replug it).
3. Open the `NanoBee` folder in VS Code and press *Upload*, or in a terminal:

   ```powershell
   cd NanoBee
   pio run -t upload        # upload_port in platformio.ini (default COM6)
   pio device monitor       # 115200, expect "# NanoBee ready" and the Wi-Fi address
   ```

Keep motor power off or the arm clear for the first start after flashing.

## 4.2 Pins (compile time, [NanoBee/src/config.h](../NanoBee/src/config.h))

Step/dir/enable of the 5 driver sockets go through the TinyBee's 74HC595 shift registers
(GPIO 25 clock, 26 latch, 27 data). Changing pins needs a rebuild and reflash.

| ROS joint | Firmware joint | Socket | Endstop |
|-----------|----------------|--------|---------|
| joint1 | J0 | X | X-MIN, GPIO 33 |
| joint2 | J1 | Y | Y-MIN, GPIO 32 |
| joint3 | J2 | Z | Z-MIN, GPIO 22 |
| joint4 | J3 | E0 | – |
| joint5 | J4 | E1 | – |
| gripper servo | `G us` command | servo header | GPIO 2 |

## 4.3 Connect

* **USB serial**, 115200 baud: `pio device monitor` or any terminal. One command per line, every
  command answers `ok` or `err <reason>`; lines starting with `# ` are asynchronous messages.
* **Wi-Fi (setup and testing only, ROS uses USB serial)**: join network `NanoBee` (password
  `nanobee123`), open <http://192.168.4.1/> or <http://nanobee.local/>. The page has STOP, enable,
  home, jog, move, set zero, servo and the settings editor. To join your own network set
  `wifi.mode: sta`, `ssid`, `password` in config.yaml and reboot the board.
* **Python** ([NanoBeeDriver/nanobee.py](../NanoBee/NanoBeeDriver/nanobee.py), `pip install pyserial`),
  angles in degrees:

  ```python
  from nanobee import NanoBee
  arm = NanoBee(port="COM6", config="config.yaml")   # or NanoBee("192.168.4.1") over Wi-Fi
  print(arm.status())
  ```

Only one program can hold the serial port: close the serial monitor / NanoBeeDriver before
starting ROS.

## 4.4 Settings – `config.yaml`

[NanoBee/NanoBeeDriver/config.yaml](../NanoBee/NanoBeeDriver/config.yaml) is the single source of
truth for the arm's tunables; the ROS bridge reads them from the board at startup (`SETTINGS?`).
Firmware units are **steps** (`steps/s`, `steps/s²`); degrees × `steps_per_deg` = steps.
Each joint has its own block under `joints:` (`joint1`..`joint5` = driver slots X, Y, Z, E0, E1):

```yaml
joints:
  soft_limits: true
  joint1:
    steps_per_deg: 17.7778
    max_speed: 6400
    homing: center
    min_deg: -175
    max_deg: 175
    # ...
```

The older form with one list per key (`steps_per_deg: [a, b, c, d, e]`) is still accepted.

| Key | Current value / meaning |
|-----|-------------------------|
| `joints.jointN.steps_per_deg` | 17.7778 for every joint (motor steps × microsteps × gear ratio / 360) |
| `joints.jointN.max_speed`, `max_accel` | in steps/s, steps/s² (6400 = 360°/s) |
| `joints.jointN.dir_invert` | flip a joint's direction |
| `joints.jointN.idle_disable` | `true`: the joint's driver switches off after `motion.idle_timeout_ms` at rest (cooler motors/drivers, less power) and wakes on the next move. Position and home reference are kept, so use it only for joints that hold their pose unpowered (now joint1 + joint5) |
| `joints.jointN.homing` | `none`, `edge` (switch = `home_offset_deg`) or `center` (switch hit from both sides, middle = `home_offset_deg`); joint1 = `center`, joint2 = `edge`, the others `none` |
| `joints.jointN.homing_dir`, `home_offset_deg` | direction of the first approach; angle given to the home reference |
| `joints.jointN.homing_speed`, `homing_max_travel` | optional per-joint seek speed (steps/s, slow touch = speed / 5) and search distance (steps); `0` = use `homing.speed` / `homing.max_travel` |
| `joints.soft_limits`, `jointN.min_deg`, `jointN.max_deg` | global switch; range per joint, enforced once the joint is referenced (homed or set with `Z`) |
| `motion.blend` | `true`: timed `T` segments run back to back without stopping (needed for smooth ROS trajectories) |
| `motion.default_speed`, `default_accel`, `jog_timeout_ms` | `M` defaults, jog watchdog |
| `motion.idle_timeout_ms` | rest time before `idle_disable` joints switch off (5000; `0` = never) |
| `homing.enabled`, `speed`, `backoff`, `max_travel` | `false` rejects `H`; homing speeds/distances in steps |
| `endstops.active_low` | switch polarity (all switches) |

Upload (the board must be idle; applied at once, Wi-Fi changes need a reboot):

* web page → *Settings* → *Save & apply*, or
* `NanoBee(port="COM6", config="config.yaml").push_config()` from `NanoBee/NanoBeeDriver`, or
* with ROS running (`use_hardware:=true`): the ROS web page (http://localhost:8080) → *Board settings
  (config.yaml)* → edit → **Send to board**. It saves `NanoBee/NanoBeeDriver/config.yaml` (launch
  argument `board_config:=...` for another file), uploads it through the bridge over USB and reloads
  the bridge settings; if the board rejects it the file is left unchanged. Same without the page:
  `ros2 service call /nanobee_bridge/push_config std_srvs/srv/Trigger`.

Check with `SETTINGS?` (effective values, defaults filled in). With ROS running, call
`ros2 service call /nanobee_bridge/reload_settings std_srvs/srv/Trigger` afterwards (the bridge
must be stopped while another program uses the serial port, so usually just restart it).

## 4.5 Essential commands

Positions are absolute steps, speeds steps/s.

| Command | Meaning |
|---------|---------|
| `?` | Status `<Idle\|Run\|Jog\|Home P:p0,..,p4 Q:queued F:free E:enabled L:endstops H:referenced O:powered>` (`L` = J0..J2 switches, `1` = pressed; `H` = joint homed/zeroed; `O` = driver on, `0` while switched off by `idle_disable`) |
| `M p0 p1 p2 p3 p4 [speed [accel]]` | Synchronized absolute move (all joints arrive together) |
| `R d0 .. d4 [speed [accel]]` | Relative move |
| `T p0 .. p4 dt_ms` | Timed segment (used by ROS trajectories); `err full` = queue full, retry |
| `J v0 .. v4 [accel]` | Jog velocities, repeat within `jog_timeout_ms` |
| `S` | Decelerate to a stop, drop the queue, cancel homing |
| `!` | Instant halt (real-time, no newline needed) |
| `E 0` / `E 1` | Disable / enable the drivers (disable drops the home reference) |
| `H` / `H j` | Home all joints with a homing mode / joint `j` |
| `Z` / `Z p0 .. p4` | Current position = 0 / given steps; the joints count as referenced |
| `VER`, `SETTINGS?`, `CONFIG?`, `REBOOT` | Version (`NanoBee 2 joints=5`), effective / stored settings, reboot |

## 4.6 Calibrate each joint (one at a time!)

1. **Direction** – jog the joint in + direction (web page or `J`). If it turns the wrong way, flip
   its `dir_invert`. Positive direction must match the URDF axis (right-hand rule around the
   `<axis>` vector).
2. **Steps per degree** – command a 90° move (e.g. `arm.move(0, 90, speed=30, relative=True)`) and
   measure the real angle. Correct: `new = old × commanded / measured`.
3. **Speed & acceleration** – raise `max_speed` / `max_accel` until steps are lost, then use
   ~50–60 % of that. Test with the arm **fully extended and loaded**.
4. **Endstop polarity** – press each switch by hand and watch `L:` in `?` (or the endstop lamp on
   the ROS web page). It must read `1` only while pressed; otherwise flip `endstops.active_low`.
   Joint1's switch is normally-closed: C + NC legs to **S + G** of the X-MIN connector (5V unused);
   pressed = pin high → `active_low: false`, and a broken wire reads as pressed (fail-safe).
5. **Homing joint1** (`center`) – hand on the E-stop, then `H 0` (or *Home* on either web page).
   The joint seeks the switch in `homing_dir`, backs off, touches it slowly, then finds it from
   the other side; the middle becomes `home_offset_deg`. Watch for `# homed 0` or
   `# err home 0: ...` (e.g. switch not found within `max_travel`). Adjust `home_offset_deg` so
   the reported angle equals the real one (0° = URDF zero pose).
   **Homing joint2** (`edge`, Y-MIN, GPIO 32) – the switch sits on one side only. Start with the
   arm at the top (0°), then `H 1`: it creeps toward +90° (`homing_dir: 1`, `homing_speed: 400`
   = 3°/s), backs off, touches the switch slowly and sets that point to `home_offset_deg` (90).
   `homing_max_travel` (12667 steps = 95°) stops it with `# err home 1: endstop not found`.
   If it moves away from the switch, flip `homing_dir`.
6. **Soft limits** – `min_deg` / `max_deg` apply once a joint is referenced: moves outside are
   rejected (`err limit`), jogs stop at the edge. `arm.launch.py` also uses them as the URDF joint
   limits (web page, MoveIt, simulators; read at launch, so relaunch after changing them). They
   must stay inside the mechanical range.

## 4.7 Joints without endstops (joint3…joint5)

`homing: none` – `H` skips them and their soft limits stay off until they are zeroed. Before
using them:

1. Move them to their mechanical zero (marks / alignment pins on the parts), motors enabled.
2. Zero them: **Home set → 0** in the joint's row on the ROS web page (ROS service
   `~/set_zero_<joint>`: only that joint becomes 0, from any position), ROS service `~/set_zero`
   (zeroes only joints that are not referenced, so a homed joint1 keeps its position), or `Z` /
   *Set all to 0* on the NanoBee web page **before** homing joint1 (it sets every joint to 0).

A board reset, power loss or `E 0` loses the position and the references: home and zero again.

## 4.8 Smoke test before ROS

With the joints at their zero pose, in the serial monitor (20° = 356 steps, 30° = 533 steps):

| Command | Expected |
|---------|----------|
| `VER` | `NanoBee 2 joints=5` |
| `Z` | `ok`, all joints referenced (`H:11111` in `?`) |
| `H 0` | `ok`, then `# homed 0` |
| `M 0 0 356 0 0` | elbow to 20° |
| `M 0 178 356 0 0` | shoulder to 10° |
| `M 533 178 356 0 0` | base to 30° |
| `M 0 0 0 0 0` | coordinated return – all joints start and stop together |
| `?` | `<Idle P:0,0,0,0,0 ...>` |

## 4.9 Start ROS

1. Close the serial monitor / NanoBeeDriver.
2. `ros2 launch arm_start arm.launch.py use_hardware:=true` – on WSL
   [attach_board.sh](../src/arm_start/scripts/attach_board.sh) attaches the board with usbipd if
   `/dev/ttyUSB0` is missing (see [02](02-dev-environment.md#usb-serial-via-usbipd-win)).
3. The bridge log shows `Connected: NanoBee 2 joints=5` and the board settings it read. Then
   continue with [06-ros2-software.md](06-ros2-software.md#62-nanobee_bridge-node).

## 4.10 Backup

Keep `NanoBee/NanoBeeDriver/config.yaml` in sync with the board and commit it after every change.
If you edited settings on the web page, copy them back (*Settings* text, or `arm.pull_config()`).
