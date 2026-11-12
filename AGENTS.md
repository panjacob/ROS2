Ignore this if you are copilot. This is for codex users(n000bs)!

# Robotyczna ręka

ROS 2 control for a five-DOF stepper arm using MKS TinyBee/ESP32 + FluidNC;
the gripper is planned. Read relevant source and package metadata before changes.
References: [architecture](docs/01-architecture.md), [software](docs/06-ros2-software.md),
[safety](docs/08-safety.md); track progress in [roadmap](docs/07-roadmap.md).
Verify documentation against code; examples and mocks are not hardware specifications.

## Team

- Root/orchestrator owns scope, interfaces, assignments, integration and final reporting.
  Delegate substantive analysis, implementation, validation and **all file edits**,
  including tiny documentation/configuration changes; no small-task exception.
- Implementer: ordinary design, changes and tests. Reviewer: independent inspection.
  Specialist: difficult kinematics, execution/fault semantics or unresolved disputes.
  Helper: bounded lookup/mechanical documentation, never control/safety decisions.
- At most three active agents including root, and one specialist, unless user requests more.
  Give objective, allowed paths, constraints, acceptance checks and edit/read-only status.
  Disjoint write ownership; no nested delegation without root assignment.
- Model/reasoning defaults belong in `.codex`, not here. Disclose unavailable capabilities
  and choose a supported worker fallback; if delegation is unavailable, stop affected work.
- Independent review is mandatory for behavior, runtime configuration, joint/coordinate
  definitions and control interfaces. Inspect actual changes, callers and tests;
  author fixes findings, reviewer rechecks material fixes. Never call self-review independent.
- Return paths, actual checks/results and limits. Findings need severity, trigger,
  consequence, location and correction; distinguish questions from confirmed defects.

## Contribution records

- Every subagent creates one short new `docs/agent-log/<UTC-YYYYMMDDTHHMMSSZ>-<role>-<unique>.md`
  per contribution. Start with a visible `ID: <filename stem>`, followed by exactly
  these three section headings, in order:
  - `Problem description`: the problem or assigned objective.
  - `How did you manage to tackle the problem?`: actual work and changed/inspected paths.
  - `Did it solve the problem? If not what was wrong?`: outcome, actual checks/results
    and unresolved limits.
  Read-only/no-change workers may create **only their own record** and label it
  “No implementation”; this is the sole exception to their read-only assignment.
- Records are append-only by policy; this is not
  filesystem-enforced immutability. Agents must never edit, overwrite, rename or delete
  existing records. Correct mistakes through a new record referencing the original.

## Control rules

- ROS uses radians/metres/seconds; degree conversion belongs at the FluidNC bridge.
  Verify axis order, signs, gearing, zeros, offsets and G-code modes. Keep selected
  Xacro, TF, planning, simulation, firmware calibration and mechanical limits consistent.
- Controller boundary validates names, sizes, finite values, limits, fresh state and
  readiness; GUI/gamepad paths cannot bypass it. Arbitrate manual/planned/homing motion.
  STOP requires deliberate rearming; faults invalidate execution and prevent stale replay.
- Define acceptance, feedback, completion, cancellation and failure across backends.
  `ok`/sending is not completion; reject unsupported timing, derivatives or tolerances.
  Track valid joint references; firmware owns limit stopping, not host polling.
- Software STOP is not emergency stop; account for gravity on torque loss. No automatic
  unlock/homing without an agreed procedure. Five DOF cannot satisfy arbitrary 6D poses;
  define task constraints and consistent TCP/tool collision geometry before planning.
- Default to offline/simulation. Connecting hardware, hardware launch, flashing, unlocking,
  homing, enabling drives or moving motors requires explicit authorization for that scope.

## Work and checks

- Preserve user changes and bounded scope. Never edit `README_FOR_HUMANS.md` without
  explicit override. Update relevant docs for behavior changes; verify touched limitations.
- Source the installed ROS distribution; claim only compatibility actually tested.
  For code/config/model changes, run focused checks, then affected packages/consumers:
  `colcon build --symlink-install --packages-up-to <packages>`,
  `colcon test --packages-select <packages>`, `colcon test-result --verbose`.
- Control checks cover malformed input, stale state, cancellation, concurrency, timeouts
  and fault recovery as relevant. Report skipped/unavailable checks; helper tests and
  mocks do not validate transport, dynamics or physical safety. Documentation-only edits
  need no full-stack build. Keep protocol, simulation and physical evidence distinct.
