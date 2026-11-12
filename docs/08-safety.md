# 8. Safety

Even a small stepper arm with 20:1 gearboxes can pinch fingers, break parts and throw objects.

## Rules

1. **Hardware E-stop** that cuts motor supply (24 V), reachable at all times. Software stop
   (`~/stop`, `!`) is not an E-stop.
2. First motion of any new config/code: **reduced speed** (`max_feed` low in
   `fluidnc_bridge.yaml`, low `max_rate_mm_per_min`), hand on E-stop, nobody in the workspace.
3. **Home after every power loss, E-stop, alarm or crash.** Positions are open-loop; after
   lost steps the reported position is wrong. An Alarm caused by a reset during motion (abort
   during cycle, e.g. Alarm 3) or by a hard/soft limit always means the position may be lost:
   verify it (or re-home) **before** pressing **Unlock / re-arm**. `~/unlock` only sends `$X`
   and re-arms after FluidNC confirms; it never homes automatically.
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
11. MoveIt **Execute plan** on hardware (bridge trajectory action) has only been tested offline.
    For the first runs keep the panel's 10 % planning speed, review the RViz preview, stand at
    the E-stop and start with short moves. STOP/cancel/faults (feed hold, soft reset once the
    arm is reported stopped) disarm execution until **Unlock / re-arm**; check the arm and
    re-home if position may be lost before re-arming. If FluidNC does not confirm the stop
    within 5 s the reset is sent anyway and may cause an alarm with lost position (rule 3).
    MoveIt's execution-time monitoring usually cancels a too-slow trajectory, which also
    disarms. The bridge checks only the final position (open-loop steps), not the
    path. Software STOP does not cut motor power; if drives are disabled afterwards the arm may
    drop (rule 5).
