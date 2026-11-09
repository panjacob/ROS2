# 4. FluidNC setup

Reference: <http://wiki.fluidnc.com/>.

## 4.1 Connect to FluidNC

* **Wi-Fi AP (factory default)**: join network `FluidNC`, open <http://192.168.0.1>.
* **Join your home Wi-Fi** (from any console – WebUI terminal or serial):

  ```
  $Sta/SSID=YourNetwork
  $Sta/Password=YourPassword
  $WiFi/Mode=STA>AP
  $Bye
  ```

  After reboot use <http://fluidnc.local> or the IP printed in the startup log (`$SS`).
* **Serial**: 115200 baud, e.g. Windows: FluidTerm from the FluidNC release, or any terminal.

## 4.2 Upload the config

1. Edit [firmware/fluidnc/config.yaml](../firmware/fluidnc/config.yaml) (gear ratios, ranges).
2. WebUI → *Flash* (LocalFS) → upload as `config.yaml` (replace existing).
3. Make sure it is the active config: `$Config/Filename=config.yaml`
4. Restart: `$Bye`, then check `$SS` for errors and `$CD` to dump the parsed config.

## 4.3 Essential commands

| Command | Meaning |
|---------|---------|
| `?` | Status report `<Idle|MPos:...|FS:...|Pn:...>` (`Pn` = triggered inputs) |
| `$H` | Home all axes with `cycle > 0` |
| `$HZ`, `$HY`… | Home single axis |
| `$X` | Clear alarm (unlock) |
| `!` / `~` | Feed hold / resume |
| `Ctrl-X` (0x18) | Soft reset |
| `$MD` / `$ME` | Disable / enable motors |
| `G90` / `G91` | Absolute / relative moves |
| `G1 X10 F600` | Move X (base) to 10° at 600°/min |
| `$J=G91 X5 F600` | Jog base +5° |
| `G10 L20 P0 A0` | Declare current position of A as 0 (sets work offset) |
| `$SS`, `$CD`, `$Settings/List` | Startup log, config dump, settings |

## 4.4 Calibrate each joint (one at a time!)

1. **Direction** – jog `+5°`. If the joint turns the wrong way, add `:low` to `direction_pin`
   (e.g. `I2SO.2:low`) or swap one motor coil pair. Positive direction must match the
   URDF axis (right-hand rule around the `<axis>` vector).
2. **Steps per degree** – command `G91 G1 X90 F600` and measure the real angle
   (protractor / phone inclinometer). Correct:
   `new_steps = old_steps × commanded / measured`.
3. **Speed & acceleration** – increase `max_rate_mm_per_min` and `acceleration_mm_per_sec2`
   until steps are lost, then use ~50–60 % of that. Test with the arm **fully extended and loaded**.
4. **Homing** – press the switch by hand, verify `Pn:` shows it. Then `$HZ` with a hand on the
   E-stop. Adjust `mpos_mm` so that after homing the reported angle equals the real angle
   (0° = the URDF zero pose).
5. **Soft limits** – `max_travel_mm` and `mpos_mm` define the reachable range
   `[mpos_mm, mpos_mm + max_travel_mm]` for negative-direction homing. Keep them inside the
   URDF `<limit>` values.

## 4.5 Joints without endstops (A, B)

`homing: cycle: 0` means `$H` skips them and soft limits are disabled for them. At startup:

1. Move the wrist to its mechanical zero (mark/alignment pin printed into the parts).
2. Set FluidNC machine position for those axes. Use a work-coordinate offset
   (`G10 L20 P0 A0 B0`) and switch the status report to WPos (`$10=0` / `$Report/Status=0`
   depending on version), or add endstops later – recommended.

> The bridge reads `MPos` (or `WPos` if MPos is absent). Keep this consistent with how you
> zero the axes.

## 4.6 Smoke test from the console before ROS

```
$X
G90
G1 Z20 F600      ; elbow
G1 Y10 F400      ; shoulder
G1 X30 F1200     ; base
G1 X0 Y0 Z0 F800 ; coordinated return
?
```

All joints should start and stop together on the last line. Only after this works, move on to
the ROS 2 bridge ([06-ros2-software.md](06-ros2-software.md)).

## 4.7 Backup

Download the active `config.yaml` from the WebUI after every change and commit it to this repo.
