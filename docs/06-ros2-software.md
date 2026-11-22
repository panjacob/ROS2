# 6. ROS 2 software

## 6.1 Packages

| Package | Type | Purpose |
|---------|------|---------|
| `arm_description` | ament_cmake | `urdf/arm.urdf.xacro`, `meshes/`, `launch/display.launch.py` |
| `arm_sim` | ament_python | Joint simulator for panel direct/jog/gamepad commands; publishes `/joint_states` |
| `arm_nanobee_bridge` | ament_python | `nanobee_bridge` node; `nanobee_protocol.py` and `trajectory.py` (pure, unit tested) |
| `arm_start` | ament_cmake | `launch/arm.launch.py`, `config/nanobee_bridge.yaml`, `scripts/attach_board.sh` |
| `arm_web` | ament_python | `web_viewer` node: web page (default http://localhost:8080) with live state, STOP/unlock, hold-to-jog, go-to, endstop homing and browser gamepad forwarding |
| `arm_teleop` | ament_python | `joy_teleop` node: `/joy` → `/jog` velocities (always started) |
| `arm_camera` | ament_python | `camera_node`: USB camera (OpenCV/V4L2) → `/camera/image_raw` (bgr8) and `/camera/image_raw/compressed` (JPEG); started unless `camera:=false`. A second instance `camera2` reads the phone camera from a stream URL (`camera2_device`, default `http://{windows_host}:8081/stream.mjpg`) → `/camera2/image_raw[/compressed]` (`camera2_link`); started unless `camera2:=false` |
| `arm_moveit_config` | ament_cmake | SRDF arm group, self-collision exclusions and OMPL planning configuration |

Planned:

| Package | Purpose |
|---------|---------|
| `arm_hardware` | C++ `ros2_control` `SystemInterface` for NanoBee (replaces the topic bridge) |
| `arm_gripper` | Gripper driver/action server |
| `arm_apps` | Task-level nodes (pick & place, teleop, vision) |

## 6.2 `nanobee_bridge` node

Source: [nanobee_bridge.py](../src/arm_nanobee_bridge/arm_nanobee_bridge/nanobee_bridge.py)

| Interface | Type | Description |
|-----------|------|-------------|
| `joint_states` (pub) | `sensor_msgs/JointState` | Positions [rad] = board steps (`P:` in the status report) / `steps_per_deg`, at `status_rate` Hz (20) |
| `~/machine_state` (pub) | `std_msgs/String` | NanoBee state: `Idle`/`Run`/`Jog`/`Home`, with ` motors off` appended while the drivers are disabled |
| `~/limit_switches` (pub) | `std_msgs/String` | Comma-separated joints whose endstop is pressed (e.g. `joint1`); empty when none |
| `joint_command` (sub) | `sensor_msgs/JointState` | Absolute target [rad] for named joints, one `M` move at `move_speed` (0 = board `motion.default_speed`); ignored while a trajectory goal is active or homing runs |
| `joint_trajectory` (sub) | `trajectory_msgs/JointTrajectory` | Unconfirmed legacy path: one `T` segment per point (+ final deceleration ramp), no URDF limit checks or completion (ignored while a trajectory goal is active or a previous one still streams). Prefer the action |
| `jog` (sub) | `sensor_msgs/JointState` | Velocity jog: `velocity` [rad/s] per named joint (others 0), sent as `J` at `jog_rate`, clamped to `max_jog_speed` and the board's `max_speed`. Must be re-sent within `jog_timeout`; zero/stale → jog stops. Ignored while a trajectory goal is active |
| `/arm_controller/follow_joint_trajectory` (action) | `control_msgs/FollowJointTrajectory` | MoveIt execution on the real arm, see [Trajectory execution](#trajectory-execution-on-hardware) |
| `~/trajectory_status` (pub, latched) | `std_msgs/String` | Armed / executing / result / disarm reason of trajectory execution |
| `~/home` (srv) | `std_srvs/Trigger` | `H`: home every joint with `joints.jointN.homing` ≠ `none` (board must be `Idle`; refused while jogging, streaming or a trajectory goal is active) |
| `~/home_<joint>` (srv) | `std_srvs/Trigger` | `H j`: home one joint; only created for joints with a homing mode (joint1, `center`: switch found from both sides, the middle becomes `home_offset_deg`) |
| `~/homing_status` (pub, latched) | `std_msgs/String` | Homing progress / result text |
| `~/stop` (srv) | `std_srvs/Trigger` | `S`: decelerate to a stop, drop queued motion, cancel homing; aborts the trajectory goal and jog. If `S` is not acknowledged the instant halt `!` is sent. Disarms trajectory execution until `~/unlock` |
| `~/unlock` (srv) | `std_srvs/Trigger` | `E 1` (enable the motors) and re-arm trajectory execution after STOP or a fault; fails (stays disarmed) if the board does not answer `ok`. Refused while a trajectory goal is active. Never homes |
| `~/motors` (srv) | `std_srvs/SetBool` | `E 1` / `E 0`: enable / disable the drivers. Disabled joints can be moved by hand; the board then forgets the home reference |
| `~/set_zero` (srv) | `std_srvs/Trigger` | `Z`: joints that are not referenced (homed or zeroed) get position 0; referenced joints keep theirs. Board must be `Idle` |
| `~/reload_settings` (srv) | `std_srvs/Trigger` | Read the board settings again (`SETTINGS?`) after changing its config.yaml |

Parameters: see [nanobee_bridge.yaml](../src/arm_start/config/nanobee_bridge.yaml) (serial port,
joint names in firmware order, rates, timeouts, trajectory tolerances, jog limits). Steps per
degree, speed/acceleration limits, homing modes and soft limits are **not** bridge parameters:
they are read from the board at startup (`SETTINGS?`), so the board's
[config.yaml](../NanoBee/NanoBeeDriver/config.yaml) is the single source of truth
(see [04](04-nanobee-setup.md#44-settings--configyaml)).
`arm.launch.py` also passes `robot_description`; joint position and velocity limits for
trajectory goals come from it (without it every goal is rejected).

Flow control: one command line at a time, wait for `ok`/`err <reason>` (`command_timeout`).
Status requests (`?`) and the instant halt (`!`) are written directly; lines starting with `# `
are asynchronous board messages (`# homed 0`, `# err home 0: ...`, `# warn ...`,
`# NanoBee ready` after a board restart, which disarms execution and re-reads the settings).
The bridge runs a multi-threaded executor: trajectory execution waits in its own callback
while serial reading, status and STOP keep running.

### Trajectory execution on hardware

> Implemented and tested only offline against a fake NanoBee (unit tests); **not yet run on
> the physical arm.** First runs: reduced speed (panel plans at 10 %), area clear, power
> cut-off in reach — software STOP is not an emergency stop (see [08](08-safety.md)).

* **Acceptance.** One goal at a time (a second is rejected). The goal then aborts with
  `INVALID_GOAL`/`INVALID_JOINTS` without motion unless: execution is armed; the connection is
  up; joint limits are known; no homing, jog (active or freshly requested) or streaming
  `joint_trajectory`; a fresh status report (within `status_timeout`) says `Idle` with an empty
  queue. The trajectory must name exactly joint1…joint5 (any order, no duplicates), have ≥ 1
  point, finite positions within URDF limits, `time_from_start` finite and strictly increasing,
  a point at t = 0 within `trajectory_start_tolerance` of the measured position, per-joint speed
  ≤ URDF velocity × 1.05 and ≤ the board's `joints.jointN.max_speed`, segments ≤ 60 s, and — for joints
  that are referenced while `soft_limits` is on — targets inside the board's `min_deg`/`max_deg`.
  Too fast or out-of-range goals are **rejected, not clamped**, so timing never silently deviates.
* **Unsupported (rejected):** `multi_dof_trajectory`, `path_tolerance`, component tolerances,
  efforts, a `header.stamp` more than 0.5 s in the future. Velocities/accelerations are only
  checked for size/finiteness (each segment runs at constant speed). Honoured: `goal_tolerance`
  positions (> 0, else `trajectory_goal_tolerance`), `goal_time_tolerance` (> 0 replaces the
  default time allowance).
* **Execution.** Radians → steps only here (board `steps_per_deg`). Each trajectory point
  becomes one `T p0..p4 dt_ms` segment (absolute steps, `dt` from `time_from_start`); with
  `motion.blend` the board runs them back to back without stopping. A final deceleration ramp
  at the joints' `max_accel` is appended so the arm stops on the last point instead of
  overshooting (ends at most `max_speed / (2 × max_accel)` late). The bridge keeps the board's
  32-entry queue topped up; `err full` just means "retry later". If `motion.blend` is off the
  bridge warns at startup: the arm then stops at every point.
* **Feedback** at `status_rate`: desired (linear interpolation of the points), actual (board
  position) and error.
* **Completion.** `ok` only means "queued". Success requires every segment accepted, two
  later status reports, board `Idle` with an empty queue and every joint within its goal
  tolerance of the last point. A final position mismatch aborts with `GOAL_TOLERANCE_VIOLATED`
  and disarms.
* **Failure** (abort, then STOP and disarm): refused segment, the board's queue ran dry before
  all segments were sent (`# warn ...`, serial link too slow), connection lost, stale status,
  any state other than `Run`/`Idle`, motors disabled, or no finish within
  `duration × trajectory_time_scaling + trajectory_time_margin` (defaults × 1.5 + 2 s).
  MoveIt's own duration monitoring (`moveit_controllers.yaml`: × 1.5 + 1 s) normally fires
  first: a too-slow trajectory then ends as a MoveIt **cancel** (goal canceled, STOP,
  execution disarmed) rather than a bridge timeout abort.
* **Cancel / STOP.** Cancel or `~/stop` (also during jog / go-to / homing) → `S`: the board
  decelerates to a stop, drops its queue and cancels homing; the bridge stops
  sending segments. If `S` is not acknowledged, `!` (instant halt, no deceleration – steps may be
  lost) is sent. The goal ends canceled (cancel) or aborted (`~/stop`). Execution stays
  **disarmed** until the operator presses **Unlock / re-arm** (`~/unlock`); the bridge starts
  armed. STOP drops the current jog velocity: jog stays inhibited until a zero-velocity jog
  message (gamepad release) or a pause of at least `jog_timeout` in jog messages, so a stick or
  button held through STOP does not restart motion. The same latch applies to a non-zero jog
  received while a trajectory goal is active or homing runs (refused messages still count as the
  jog stream), so a stick held through a goal cancel/fault does not jog afterwards.
  Jog/go-to/homing need no Unlock afterwards.
* **Limits.** Open loop: the board reports commanded steps; intermediate positions are not
  checked against a path tolerance. The simulator's STOP does not latch/disarm.

### Examples

```bash
ros2 launch arm_start arm.launch.py use_hardware:=true

# ...or run just the bridge against the board:
ros2 run arm_nanobee_bridge nanobee_bridge \
  --ros-args -p serial_port:=/dev/ttyUSB0 -p baud:=115200

# Read realtime status from the board:
ros2 topic list
ros2 topic echo /joint_states
ros2 topic echo /nanobee_bridge/machine_state
ros2 topic echo /nanobee_bridge/limit_switches

ros2 service call /nanobee_bridge/unlock std_srvs/srv/Trigger
ros2 service call /nanobee_bridge/home std_srvs/srv/Trigger
ros2 service call /nanobee_bridge/set_zero std_srvs/srv/Trigger
ros2 service call /nanobee_bridge/motors std_srvs/srv/SetBool "{data: false}"
ros2 service call /nanobee_bridge/reload_settings std_srvs/srv/Trigger

# Single move (radians)
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState \
  "{name: [joint1, joint2, joint3], position: [0.5, 0.3, -0.6]}"

# Timed trajectory
ros2 topic pub --once /joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [joint1, joint3],
  points: [
    {positions: [0.5, -0.5], time_from_start: {sec: 2}},
    {positions: [0.0,  0.0], time_from_start: {sec: 4}}
  ]}"

ros2 service call /nanobee_bridge/stop std_srvs/srv/Trigger
```

### Endstop homing (one switch per joint, hit from both sides)

```bash
ros2 service call /nanobee_bridge/home_joint1 std_srvs/srv/Trigger   # or "Home" on the web page
ros2 topic echo /nanobee_bridge/homing_status
```

Homing is configured on the board (`joints.jointN.homing`, `homing_dir`, `home_offset_deg`,
`homing.*` in [config.yaml](../NanoBee/NanoBeeDriver/config.yaml)); joint1 uses `center`.
Endstop pins are fixed in [NanoBee/src/config.h](../NanoBee/src/config.h): joint1 uses X-MIN
(GPIO 33), connector pins **S G 5V** (left to right): wire the switch's C + NC legs to **S + G**
(5V unused). Normally-closed reads low when released, so the pin is active-high
(`endstops.active_low: false`); a broken wire then reads as pressed (fail-safe). If you wire C + NO
instead, set `endstops.active_low: true`, push the config (see [04](04-nanobee-setup.md)) and call
`~/reload_settings`. Check the switch on the web page: the "Endstops" card shows a red **PRESSED**
lamp only while it is pressed (joints listed in the `home_joints` param of `web_viewer`,
default `['joint1']`).

## 6.3 Web page and gamepad

`ros2 launch arm_start arm.launch.py use_hardware:=true`, then open
http://localhost:8080 (from Windows too).

- STOP / Unlock / re-arm, hold − / + to jog a joint, one shared "Target" column for planning or direct movement; **Joint units** selects degrees (default) or radians for joint positions, targets, gauges and jog speed, "Home" for `home_joints`. On hardware the Bridge card also shows the latched trajectory execution status (armed / executing / result / disarmed); **Unlock / re-arm** enables the motors and re-arms plan execution after STOP or a fault.
- Changing joint units preserves physical targets, jog speed and an unchanged ready plan. TCP angles remain in degrees; the HTTP API still uses degrees and degrees/s.
- "Position gauge" bar per joint: −180° … 0 … +180° (−π … 0 … +π rad), the dot shows the current position (red and pinned to the end if outside).
- TCP position: the panel reads the `world` → `tool0` transform from TF and displays
  the origin of `tool0` in world coordinates (X/Y/Z in millimetres). It marks the
  position unavailable if joint states or the transform become stale. The web node's
  `reference_frame` parameter changes the reference frame.
- Named TCPs: use the TCP card to add a name and X/Y/Z offsets in millimetres plus
  roll/pitch/yaw in degrees relative to `tool0`. Each saved TCP has a
  `tcp_<name>` frame under `tool0`; selecting one displays its pose in `world`.
  Definitions and the selection persist in `~/.config/ros2_arm/tcps.json`; override
  the path with the `tcp_storage_path` launch argument. The built-in `tool0` is the
  zero-offset TCP and cannot be edited or deleted.
- Cameras card: live MJPEG streams (`GET /camera.mjpg`, snapshot `GET /camera.jpg`) of
  `/camera/image_raw/compressed` (`camera_topic` parameter) and of the phone camera
  (`GET /camera2.mjpg`, `GET /camera2.jpg`, `camera2_topic`). See
  [2.4.1](02-dev-environment.md#241-usb-camera-microsoft-lifecam-vx-6000) and
  [2.4.2](02-dev-environment.md#242-phone-camera-telescope) for camera setup in WSL.
- Gamepad: the browser reads it (Gamepad API, so Windows drivers work – WSL has no gamepad driver)
  and POSTs it to `/api/joy` → `/joy` → `joy_teleop` → `/jog`. Always on when a pad is connected
  (a real pad with the standard layout is preferred over virtual ones like vJoy),
  but only while the page has focus. Default mapping: left stick X → joint1, left stick Y → joint2,
  right stick Y → joint3, right stick X → joint4, LB/RB → joint5, B → STOP.
  Change it with `joy_teleop` parameters (`axis_index`, `axis_scale`, `button_minus`, `button_plus`,
  `max_speed`, `deadzone`, `expo`).

HTTP API (JSON body, `Content-Type: application/json` required): `/api/jog` `{"joint1": deg_per_s}`,
`/api/goto` `{"joint1": deg}`, `/api/plan` `{"joint1": deg, ..., "joint5": deg}`,
`/api/execute` `{"plan_id": "..."}`,
`/api/home` `{"joint": "joint1"}`, `/api/stop`, `/api/unlock`,
`/api/joy` `{"axes": [...], "buttons": [...]}`; `GET /state` returns everything shown on the page;
`GET /camera.mjpg` / `GET /camera.jpg` serve the USB camera, `GET /camera2.mjpg` / `GET /camera2.jpg` the phone camera.

## 6.4 MoveIt 2 joint planning

From the repository root, source the installed ROS distribution and run the
following commands.
Install `python3-rosdep` and `python3-colcon-common-extensions` first if either
command is missing; initialize rosdep once with `sudo rosdep init` if needed.
If the dependencies are already installed, omit the `rosdep` commands.

```bash
source /opt/ros/lyrical/setup.bash  # use jazzy if that is what you installed
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
ros2 launch arm_start arm.launch.py
```

Open http://localhost:8080, enter all five angles in **Target** (degrees by default; choose radians with **Joint units**), then
click **Preview plan**. After checking the RViz animation, click **Execute plan**. The panel shows the execution result. STOP interrupts the
movement. Start a new terminal with the ROS and workspace setup sourced
before using `ros2 topic` or `ros2 service` commands.

The launch starts `move_group` with the `arm` chain (`base_link` → `tool0`), OMPL,
and the current URDF. The panel sends all five target joint angles to `/move_action`
with `plan_only=true`. The start state comes from `/joint_states`; successful
trajectories are published to `/display_planned_path` and animated in RViz.
The panel reports missing/stale joint states, out-of-limit goals, collisions, and
planning failures. Planning itself does not move the arm. **Execute plan**
passes the stored trajectory to MoveIt's `/execute_trajectory` action. MoveIt
uses the `/arm_controller/follow_joint_trajectory` action — served by the simulator,
or by the NanoBee bridge with `use_hardware:=true` — to follow
its timed points and reports completion or failure to the panel. A plan can be
executed once within 120 seconds; execution is rejected if the current joints
have moved from the plan's start. STOP cancels the action and stops the controller.
The panel plans at 10 % velocity/acceleration scaling on hardware and 30 % in simulation
(`web_viewer` parameters `max_velocity_scaling`/`max_acceleration_scaling`, set by the launch).
The disclosed **Direct movement — no collision checking** action uses the same complete
target pose as planning and sends it directly; a sent command does not confirm arrival.
Use **Use current angles** before changing individual joints. A blank target field
defaults to that joint's current angle (filled into the field) only when joint state
is fresh (connected, at most 2 s old, all joints finite) and within the joint limits;
otherwise the request is rejected. Direct movement, jog,
homing, STOP and target edits invalidate the local preview; late planning replies
cannot re-enable execution after those changes.

The self-collision exclusions in `src/arm_moveit_config/config/arm.srdf.xacro` cover
adjacent links only. The URDF collision shapes are approximations, so revalidate
them against the final CAD before relying on collision checks with hardware.
Execution on hardware goes through the bridge's trajectory action (see
[Trajectory execution on hardware](#trajectory-execution-on-hardware)); it has been
tested only offline, not on the physical arm. After STOP or a fault the bridge rejects
goals until **Unlock / re-arm**. Cartesian/TCP goal planning is not configured yet.

## 6.5 `ros2_control` hardware interface (recommended upgrade)

To connect MoveIt to the real arm cleanly, implement `arm_hardware` (C++):

* Class derived from `hardware_interface::SystemInterface`.
* `on_init`: read params (serial port, joint order) from the `<ros2_control>` URDF tag.
* `on_activate`: open the serial port, check `VER`, read `SETTINGS?` (steps per degree), `E 1`,
  optionally `H`.
* `read()`: parse the latest `<... P:p0,..,p4 ...>` (requested with `?` from a background thread)
  → steps / `steps_per_deg` → state interfaces `position`.
* `write()`: stream the controller's commands as timed `T` segments (one per update period,
  blended by `motion.blend`), or as `J` velocity jogs that stop by themselves when commands
  cease (`jog_timeout_ms`); `S` / `!` to stop – suits `joint_trajectory_controller` streaming.
* Controllers: `joint_state_broadcaster` + `joint_trajectory_controller`.
* Then MoveIt's `moveit_simple_controller_manager` points at
  `joint_trajectory_controller/follow_joint_trajectory`.

Reference implementations: `ros2_control_demos` (example_1 RRBot), and the Thor/PAROL6
community ROS 2 drivers.

## 6.6 Joint simulation

Run `ros2 launch arm_start arm.launch.py` and open http://localhost:8080. The `arm_sim`
node accepts `/joint_command`, `/jog`, and the
`/arm_controller/follow_joint_trajectory` action; it publishes `/joint_states`.
The panel's direct movement, hold-to-jog, gamepad, Execute, and STOP
controls work without the board; RViz shows the same positions. The simulation has no
endstops or physical dynamics. `use_hardware:=true` selects the NanoBee bridge instead;
it serves the same action name, so MoveIt's controller configuration is shared.

The default model is `arm_concept.urdf.xacro`, which uses the meshes exported from
Blender. Pass `model:=arm.urdf.xacro` to either launch file to inspect the original
primitive model. Both models retain the same joint names, axes and limits.

For physics and controller testing later, add `<ros2_control>` and Gazebo (`ros_gz`)
plugins to the xacro. That requires realistic inertias (see [05](05-mechanical-design.md)).

## 6.7 Coding conventions

* ROS units: radians, meters, seconds. NanoBee units: motor steps (`steps_per_deg` from the
  board). Convert only in the bridge.
* Keep protocol logic free of ROS imports (testable with plain `pytest`).
* `colcon build --symlink-install` + `colcon test` before each commit.
* One source of truth per limit: URDF for planning; the board's config.yaml for steps, speeds
  and soft limits (NanoBee is the last guard; the bridge rejects goals that violate either).
