ID: 20261112T092702Z-designer-shared-targets

## Problem description

No implementation. Inspect the duplicated direct/planned joint target controls and propose one compact target workflow without hiding the distinction between direct commands and collision-checked planning.

## How did you manage to tackle the problem?

Read `AGENTS.md`, `src/arm_web/web/index.html`, relevant sections of `src/arm_web/arm_web/web_viewer.py`, `src/arm_web/package.xml`, `src/arm_sim/arm_sim/motion.py`, and `src/arm_fluidnc_bridge/arm_fluidnc_bridge/fluidnc_bridge.py`. Inspected relevant documentation in `docs/01-architecture.md`, `docs/06-ros2-software.md`, `docs/07-roadmap.md`, and `docs/08-safety.md`.

Recommended one initially blank Target column; panel-level Use current angles, Preview plan and simulation-only Execute plan; and a separate direct-movement disclosure explicitly stating no collision checking, using the same targets in simulation and hardware. Recommended making joints the primary operation card, keeping hardware status/endstops visible, and putting TCP definition fields and gamepad mapping into disclosures. Noted that the fixed position bar is a gauge rather than a mechanical range, that direct API success means sent rather than completion, and that preview invalidation must survive target changes and in-flight responses. Sent actionable hierarchy, labels, API mapping and state requirements to root and implementer. Only created this contribution record.

## Did it solve the problem? If not what was wrong?

Design assignment completed by source inspection; implementation remains assigned to another worker. Confirmed that the direct API accepts a multi-joint dictionary, planning requires all joint targets, and planned execution is simulation-only. No UI/source edits, builds, runtime tests, browser checks or hardware actions were performed. Source inspection and design recommendations do not validate physical motion, collision geometry or controller safety.
