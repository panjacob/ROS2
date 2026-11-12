# 6. ROS 2 software

## 6.1 Packages

| Package | Type | Purpose |
|---------|------|---------|
| `arm_description` | ament_cmake | `urdf/arm.urdf.xacro`, `meshes/`, `launch/display.launch.py` |
| `arm_sim` | ament_python | Joint simulator for panel direct/jog/gamepad commands; publishes `/joint_states` |
| `arm_fluidnc_bridge` | ament_python | `fluidnc_bridge` node; `fluidnc_protocol.py` (pure, unit tested) |
| `arm_start` | ament_cmake | `launch/arm.launch.py`, `config/fluidnc_bridge.yaml` |
| `arm_web` | ament_python | `web_viewer` node: web page (default http://localhost:8080) with live state, STOP/unlock, hold-to-jog, go-to, endstop homing and browser gamepad forwarding |
| `arm_teleop` | ament_python | `joy_teleop` node: `/joy` → `/jog` velocities (always started) |
| `arm_moveit_config` | ament_cmake | SRDF arm group, self-collision exclusions and OMPL planning configuration |

Planned:

| Package | Purpose |
|---------|---------|
| `arm_hardware` | C++ `ros2_control` `SystemInterface` for FluidNC (replaces the topic bridge) |
| `arm_gripper` | Gripper driver/action server |
| `arm_apps` | Task-level nodes (pick & place, teleop, vision) |

## 6.2 `fluidnc_bridge` node

Source: [fluidnc_bridge.py](../src/arm_fluidnc_bridge/arm_fluidnc_bridge/fluidnc_bridge.py)

| Interface | Type | Description |
|-----------|------|-------------|
| `joint_states` (pub) | `sensor_msgs/JointState` | Positions [rad] from FluidNC work position (`WPos` = `MPos` − `WCO`) at `status_rate` Hz |
| `~/machine_state` (pub) | `std_msgs/String` | FluidNC state: `Idle`/`Run`/`Jog`/`Hold`/`Home`/`Alarm`/`Door` |
| `~/feed_rate` (pub) | `std_msgs/Float64` | Current feed rate [deg/min] from the `FS`/`F` status field |
| `~/spindle_speed` (pub) | `std_msgs/Float64` | Current spindle speed from the `FS` status field |
| `~/limit_switches` (pub) | `std_msgs/String` | Triggered pins from `Pn` (e.g. `XYZ`, probe `P`); empty when none |
| `joint_command` (sub) | `sensor_msgs/JointState` | Absolute target [rad] for named joints, moved at `max_feed` |
| `joint_trajectory` (sub) | `trajectory_msgs/JointTrajectory` | Points sent as `G1` segments, feed derived from `time_from_start` |
| `jog` (sub) | `sensor_msgs/JointState` | Velocity jog: `velocity` [rad/s] per named joint (others 0). Must be re-sent within `jog_timeout`; zero/stale → jog cancel |
| `~/home` (srv) | `std_srvs/Trigger` | `$H` |
| `~/home_<joint>` (srv) | `std_srvs/Trigger` | Endstop homing of one joint (`homing_joints`): seek left to the switch, then right to the switch (each with slow re-touch); the middle becomes 0° |
| `~/homing_status` (pub, latched) | `std_msgs/String` | Homing progress / result text |
| `~/unlock` (srv) | `std_srvs/Trigger` | `$X` |
| `~/stop` (srv) | `std_srvs/Trigger` | Abort homing, cancel jog, clear queue, feed hold, then soft reset when stopped |

Parameters: see [fluidnc_bridge.yaml](../src/arm_start/config/fluidnc_bridge.yaml).

Flow control: one G-code line at a time, wait for `ok`/`error` (simple send-response protocol).
Real-time characters (`?`, `!`, `0x18`) bypass the queue.

### Examples

```bash
ros2 launch arm_start arm.launch.py use_hardware:=true

# ...or run just the bridge against the board:
ros2 run arm_fluidnc_bridge fluidnc_bridge \
  --ros-args -p serial_port:=/dev/ttyUSB0 -p baud:=115200

# Read realtime status from the board:
ros2 topic list
ros2 topic echo /joint_states
ros2 topic echo /fluidnc_bridge/machine_state
ros2 topic echo /fluidnc_bridge/feed_rate
ros2 topic echo /fluidnc_bridge/limit_switches

ros2 service call /fluidnc_bridge/unlock std_srvs/srv/Trigger
ros2 service call /fluidnc_bridge/home std_srvs/srv/Trigger

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

ros2 service call /fluidnc_bridge/stop std_srvs/srv/Trigger
```

### Endstop homing (one switch per joint, hit from both sides)

```bash
ros2 service call /fluidnc_bridge/home_joint1 std_srvs/srv/Trigger   # or "Home" on the web page
ros2 topic echo /fluidnc_bridge/homing_status
```

The switch pin must be in the FluidNC config (`limit_all_pin` of that axis) and the joint's pin letter
set in `homing_pins`. Joint 1 uses X_STOP (`gpio.33`), connector pins **S G 5V** (left to right):
wire the switch's C + NC legs to **S + G** (5V unused). Normally-closed reads low when released, so the
pin is active-high (`gpio.33`); a broken wire then reads as pressed (fail-safe). If you wire C + NO
instead, use `gpio.33:low`. Pin changes need a config upload + restart:
`python3 tools/fluidnc_upload_config.py firmware/fluidnc/config.xyzab.yaml --restart` (stop ROS first).
Check the switch on the web page: the "Endstops" card shows a red **PRESSED** lamp only while it is pressed
(`home_pins` param of `web_viewer`, default `['X']`, pairs with `home_joints`).

## 6.3 Web page and gamepad

`ros2 launch arm_start arm.launch.py use_hardware:=true`, then open
http://localhost:8080 (from Windows too).

- STOP / Unlock alarm, hold − / + to jog a joint, one shared "Target [°]" column for planning or direct movement, "Home" for `homing_joints`.
- "Position gauge" bar per joint: −180° … 0 … +180°, the dot shows the current position (red and pinned to the end if outside).
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
`/api/joy` `{"axes": [...], "buttons": [...]}`; `GET /state` returns everything shown on the page.

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

Open http://localhost:8080, enter all five angles in **Target [°]**, then
click **Preview plan**. After checking the RViz animation, click **Execute plan**. The panel shows the execution result. STOP interrupts the
simulated movement. Start a new terminal with the ROS and workspace setup sourced
before using `ros2 topic` or `ros2 service` commands.

The launch starts `move_group` with the `arm` chain (`base_link` → `tool0`), OMPL,
and the current URDF. The panel sends all five target joint angles to `/move_action`
with `plan_only=true`. The start state comes from `/joint_states`; successful
trajectories are published to `/display_planned_path` and animated in RViz.
The panel reports missing/stale joint states, out-of-limit goals, collisions, and
planning failures. Planning itself does not move the arm. **Execute plan**
passes the stored trajectory to MoveIt's `/execute_trajectory` action. MoveIt
uses the simulator's `/arm_controller/follow_joint_trajectory` action to follow
its timed points and reports completion or failure to the panel. A plan can be
executed once within 120 seconds; execution is rejected if the current joints
have moved from the plan's start. STOP cancels the action and stops the simulator.
The disclosed **Direct movement — no collision checking** action uses the same complete
target pose as planning and sends it directly; a sent command does not confirm arrival.
Use **Use current angles** before changing individual joints. Direct movement, jog,
homing, STOP and target edits invalidate the local preview; late planning replies
cannot re-enable execution after those changes.

The self-collision exclusions in `src/arm_moveit_config/config/arm.srdf.xacro` cover
adjacent links only. The URDF collision shapes are approximations, so revalidate
them against the final CAD before relying on collision checks with hardware.
Execution is enabled only in simulation. The FluidNC bridge currently accepts a
`/joint_trajectory` topic but has no action controller that confirms completion;
the panel therefore disables Execute with `use_hardware:=true`. Cartesian/TCP
goal planning is also not configured yet.

## 6.5 `ros2_control` hardware interface (recommended upgrade)

To connect MoveIt to the real arm cleanly, implement `arm_hardware` (C++):

* Class derived from `hardware_interface::SystemInterface`.
* `on_init`: read params (port/host, axis letters) from the `<ros2_control>` URDF tag.
* `on_activate`: open connection, `$X`, `G90`, optionally `$H`.
* `read()`: parse latest `<...|MPos:...>` (requested with `?` from a background thread) → state interfaces `position`.
* `write()`: when the command differs from the last sent target by more than a threshold,
  send `G1` with a feed rate matching the controller's update period; or stream with
  FluidNC's jog command `$J=` which can be cancelled instantly (`0x85`) – suits
  `joint_trajectory_controller` streaming.
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
endstops or physical dynamics. `use_hardware:=true` selects the FluidNC bridge instead.

The default model is `arm_concept.urdf.xacro`, which uses the meshes exported from
Blender. Pass `model:=arm.urdf.xacro` to either launch file to inspect the original
primitive model. Both models retain the same joint names, axes and limits.

For physics and controller testing later, add `<ros2_control>` and Gazebo (`ros_gz`)
plugins to the xacro. That requires realistic inertias (see [05](05-mechanical-design.md)).

## 6.7 Coding conventions

* ROS units: radians, meters, seconds. FluidNC units: degrees. Convert only in the bridge.
* Keep protocol logic free of ROS imports (testable with plain `pytest`).
* `colcon build --symlink-install` + `colcon test` before each commit.
* One source of truth for joint limits: URDF ≥ FluidNC soft limits (FluidNC is the last guard).
