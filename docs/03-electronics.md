# 3. Electronics

## 3.1 MKS TinyBee summary

* ESP32 (Wi-Fi), USB via CH340.
* **5 stepper driver sockets** (X, Y, Z, E0, E1) for StepStick-style drivers. Step/dir/enable
  are driven through **74HC595 shift registers (I2S output)** – in FluidNC these are `I2SO.n` pins
  and require `stepping: engine: I2S_STATIC` (or `I2S_STREAM`).
* Endstop inputs: X-MIN `gpio.33`, Y-MIN `gpio.32`, Z-MIN `gpio.22`.
* MOSFET outputs (heaters/fans) on `I2SO.16..20` – can drive a gripper solenoid, LEDs, a fan.
* EXP1/EXP2 headers expose free GPIOs if no LCD is attached (RC servo for a gripper, extra
  endstops). Pin list in the bottom of [config.yaml](../firmware/fluidnc/config.yaml).
* Supply: 12–24 V. Verify the exact input range and driver jumper settings for your board
  revision in the MKS documentation (<https://github.com/makerbase-mks/MKS-TinyBee>).

## 3.2 Bill of materials (starting point)

| Item | Qty | Notes |
|------|-----|-------|
| MKS TinyBee + FluidNC | 1 | already owned |
| Stepper drivers | 5 | **TMC2209** (quiet, standalone/STEP-DIR mode) or A4988/DRV8825 |
| NEMA17 steppers | 3 | base, shoulder, elbow – 40–48 mm length, ≥0.45 Nm, 1.5–2 A |
| NEMA17 pancake / NEMA14 | 2 | wrist pitch & roll – lighter is better |
| Power supply 24 V | 1 | ≥ 5 A (120 W+) for 5 motors |
| Endstops | 3–5 | Mechanical micro switches (wired NC) or hall sensors (A3144) |
| Emergency stop | 1 | Latching mushroom button that cuts **motor power** (24 V) |
| Gripper servo | 1 | MG996R / DS3218 (or SG90 for a light gripper), separate 5–6 V BEC |
| Bearings | – | 6807/6808 thin-section or 625/608 + 3D printed races, see [05](05-mechanical-design.md) |
| GT2 belts/pulleys or planetary/cycloidal parts | – | depends on gearbox choice |
| Wire, JST-XH connectors, drag chain, ferrules | – | – |

## 3.3 Driver configuration

* Set microstepping with the jumpers under each socket (16× is a good default; TMC2209
  standalone often interpolates to 256). The value enters `steps_per_mm`.
* **Set the current (Vref)** for each driver *before* connecting motors. Start at ~70 % of the
  motor rated current; raise only if steps are missed.
  * TMC2209 (typical StepStick, Rsense 0.11 Ω): `Irms ≈ Vref × 0.71` – check your driver's datasheet.
  * A4988: `I = Vref / (8 × Rsense)`.
* Never plug/unplug a motor while powered – it destroys drivers.
* Heatsinks on drivers; a small fan over the board.

## 3.4 Wiring

1. Motors → driver sockets according to the table in [01-architecture.md](01-architecture.md).
2. Endstops → X/Y/Z-MIN headers (signal + GND). Prefer **normally closed** switches:
   a broken wire then reads as "triggered" (safe). Invert in FluidNC with `:low` if needed.
3. 24 V PSU → E-stop (NC contact) → TinyBee power input.
   USB-powered logic keeps the ESP32 alive while motors are off – FluidNC will then lose
   position; re-home after an E-stop.
4. Gripper servo: signal → free GPIO (e.g. EXP1 `gpio.13`), power from a **separate**
   5–6 V BEC, common GND.
5. Use a drag chain / spiral wrap and leave slack at every joint for full rotation range.
   Wrist roll beyond ±180° needs a slip ring.

## 3.5 Bring-up checklist

- [ ] Board powers from USB, FluidNC console answers `$SS`
- [ ] Drivers inserted in the right orientation, Vref set
- [ ] 24 V on, no motor connected, nothing gets hot
- [ ] One motor connected, jog it from WebUI, direction checked
- [ ] Endstop states visible with `?` (`Pn:X` etc.) when pressed
- [ ] E-stop cuts motor power
