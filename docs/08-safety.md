# 8. Safety

Even a small stepper arm with 20:1 gearboxes can pinch fingers, break parts and throw objects.

## Rules

1. **Hardware E-stop** that cuts motor supply (24 V), reachable at all times. Software stop
   (`~/stop`, `!`) is not an E-stop.
2. First motion of any new config/code: **reduced speed** (`max_feed` low in
   `fluidnc_bridge.yaml`, low `max_rate_mm_per_min`), hand on E-stop, nobody in the workspace.
3. **Home after every power loss, E-stop, alarm or crash.** Positions are open-loop; after
   lost steps the reported position is wrong.
4. Keep FluidNC soft limits enabled for every joint that has an endstop. Set them slightly
   inside the mechanical range.
5. The arm may **drop when motors are disabled** (`$MD`, idle timeout, E-stop). Keep
   `idle_ms: 255`, add a counterbalance, and support the arm before cutting power.
6. Never hot-plug motors or drivers. Check driver orientation before powering.
7. Fuse the 24 V line; secure the PSU mains wiring in a closed enclosure.
8. Clamp the base to the table – a fast base rotation tips an unclamped arm.
9. Watch for heat: steppers at full current get hot enough to soften PLA mounts (use PETG/ASA).
10. Wi-Fi control: anyone on the network can reach the FluidNC telnet/WebUI. Use a trusted
    network, set a WebUI password if your FluidNC build supports it, and do not expose it to
    the internet.
