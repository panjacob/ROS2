# 8. Safety

Even a small stepper arm with 20:1 gearboxes can pinch fingers, break parts and throw objects.

## Rules

1. **Hardware E-stop** that cuts motor supply (24 V), reachable at all times. Software stop
   (`~/stop`, `!`) is not an E-stop.
2. First motion of any new config/code: **reduced speed** (low `joints.jointN.max_speed` /
   `motion.default_speed` in NanoBee's `config.yaml`, low `move_speed` / `max_jog_speed` in
   `nanobee_bridge.yaml`), hand on E-stop, nobody in the workspace.
3. **Home after every power loss, E-stop, board restart, motor disable or crash.** Positions are
   open-loop; after lost steps the reported position is wrong. A realtime halt (`!`), a
   trajectory abort or a hit endstop/soft limit always means the position may be lost: verify
   it (or re-home / `~/set_zero`) **before** pressing **Unlock / re-arm**. `~/unlock` only sends
   `E 1` (motors on) and re-arms the bridge; it never homes automatically.
4. Keep NanoBee soft limits (`joints.soft_limits`, `joints.jointN.min_deg`/`max_deg`) enabled. They
   are enforced only for joints referenced by homing or `Z`; set them slightly inside the
   mechanical range.
5. The arm may **drop when motors are disabled** (`E 0`, `~/motors` false, board reset, E-stop).
   Drivers start disabled at boot and are enabled by the first move/jog/home. Joints listed in
   `joints.jointN.idle_disable` also switch off by themselves after `motion.idle_timeout_ms` at rest
   (status `O:` bit `0`) and wake on the next command, keeping their home reference: enable it
   only for joints that do not sag or get pushed – a back-driven joint silently loses position
   (re-home it). Gravity-loaded joints (shoulder/elbow) stay powered. Add a counterbalance and
   support the arm before cutting power.
6. Never hot-plug motors or drivers. Check driver orientation before powering.
7. Fuse the 24 V line; secure the PSU mains wiring in a closed enclosure.
8. Clamp the base to the table – a fast base rotation tips an unclamped arm.
9. Watch for heat: steppers at full current get hot enough to soften PLA mounts (use PETG/ASA).
10. Wi-Fi: the NanoBee access point/web page has no login – anyone who joins the network can move
    the arm. Change the default AP password (`wifi.password`, default `nanobee123`), use it only
    for setup/testing on a trusted network and do not expose it to the internet.
11. MoveIt **Execute plan** on hardware (bridge trajectory action) has only been tested offline.
    For the first runs keep the panel's 10 % planning speed, review the RViz preview, stand at
    the E-stop and start with short moves. STOP/cancel/faults (`S` smooth stop; realtime `!`
    halt if `S` is not acknowledged) disarm execution until **Unlock / re-arm**; check the arm
    and re-home if position may be lost before re-arming (rule 3). MoveIt's execution-time
    monitoring usually cancels a too-slow trajectory, which also disarms. The bridge checks
    only the final position (open-loop steps), not the path. Software STOP does not cut motor
    power; if drives are disabled afterwards the arm may drop (rule 5).
