# ROS 2 3D-Printed Robotic Arm (MKS TinyBee + NanoBee)

A 5-DOF 3D-printed robotic arm controlled from **ROS 2**. Joints use stepper motors driven by an
**MKS TinyBee** (ESP32) running **NanoBee**, this project's own minimal firmware. A ROS 2 node
translates joint commands into NanoBee step commands and reads joint positions back from
NanoBee status reports.

```
 ROS 2 (RViz, MoveIt 2, your nodes)
        │  sensor_msgs/JointState, trajectory_msgs/JointTrajectory, FollowJointTrajectory
        ▼
 arm_nanobee_bridge  ──USB serial (M / T / J / S)──►  NanoBee on MKS TinyBee  ──►  5 x stepper drivers  ──►  arm
        ▲                                                    │
        └──────────── status "<Idle P:...>" ◄────────────────┘
```

## Repository layout

| Path | Contents |
|------|----------|
| [src/arm_description](src/arm_description) | URDF/xacro model, meshes, RViz preview launch |
| [src/arm_nanobee_bridge](src/arm_nanobee_bridge) | Python node: ROS 2 ⇄ NanoBee (line protocol over USB serial), MoveIt trajectory action |
| [src/arm_start](src/arm_start) | Launch files and parameters for sim / real hardware |
| [src/arm_moveit_config](src/arm_moveit_config) | MoveIt 2 arm group, collision settings and OMPL planner |
| [src/arm_gazebo](src/arm_gazebo) | Gazebo physics backend, table/banana world and matching MoveIt collision scene |
| [src/arm_web](src/arm_web) | Web page: live state, camera streams, joint/TCP planning and execution, jog, go-to, TCPs, gamepad |
| [src/arm_teleop](src/arm_teleop) | Gamepad (`/joy`) → joint jog |
| [src/arm_camera](src/arm_camera) | USB camera → `/camera/image_raw`, phone camera (Telescope via [windows_camera_bridge.py](src/arm_camera/windows/windows_camera_bridge.py), started by the launch) → `/camera2/image_raw` (+ `/compressed`) |
| [NanoBee](NanoBee) | NanoBee firmware for the TinyBee (PlatformIO), [NanoBeeDriver](NanoBee/NanoBeeDriver) Python driver and board settings [config.yaml](NanoBee/NanoBeeDriver/config.yaml) (steps/deg, speeds, homing, soft limits) |
| [docs](docs) | Step-by-step development guides |
| [hardware](hardware) | CAD sources, printable parts and hardware notes |

## Quick start

Requires Ubuntu with ROS 2 and MoveIt 2 installed. This workspace was tested with
ROS 2 Lyrical on Ubuntu 26.04; for Ubuntu 24.04 use Jazzy. See the
[development environment guide](docs/02-dev-environment.md) for native Ubuntu and WSL2 setup.

Run these commands from the repository root. In a new terminal, source the ROS
installation first (`lyrical` below; use `jazzy` if that is the version you installed).
You also need `colcon` and `rosdep` (`sudo apt install python3-colcon-common-extensions
python3-rosdep`). Run `sudo rosdep init` once if rosdep has not been initialized.
If the dependencies are already installed, you can skip the two `rosdep` commands.

```bash
cd ~/ros2_arm            # replace with the path where you cloned this repository
source /opt/ros/lyrical/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

In each later terminal, repeat both `source` commands before running a launch or
ROS CLI command.

Choose one launch command below. Stop it with Ctrl+C before trying another:

```bash
# 1. Preview the mesh model with joint sliders
ros2 launch arm_description display.launch.py

# 2. Joint simulation, MoveIt planning and execution
ros2 launch arm_start arm.launch.py

# Optional: select the original primitive model instead
ros2 launch arm_start arm.launch.py model:=arm.urdf.xacro

# Gazebo physics: table + banana, with MoveIt and the same web panel
ros2 launch arm_start arm.launch.py simulator:=gazebo

# 3. Real hardware (flash NanoBee and push its config.yaml first, see docs/04-nanobee-setup.md)
#    control it at http://localhost:8080 (a connected gamepad works too)
ros2 launch arm_start arm.launch.py use_hardware:=true
```

Hardware service and topic commands need a second terminal with ROS and this
workspace sourced; see [ROS 2 software](docs/06-ros2-software.md#62-nanobee_bridge-node).

For step 2, open **http://localhost:8080** after the launch starts. In the Joints
table, enter target angles under **Target** (degrees by default; choose radians with **Joint units**; blank fields hold current angles with fresh state), then click **Preview plan**. Review the animation in RViz. Click **Execute plan** to move
the simulated robot; the panel reports completion or failure. **STOP** interrupts
execution. A plan can be executed once within 120 seconds. Moving the arm with
direct movement or jog invalidates the plan, so plan again before executing.

With `use_hardware:=true`, Execute runs the plan on the arm through the NanoBee bridge's
trajectory action (planned at 10 % speed; not yet validated on the physical arm).
Use **Use current angles** to fill the shared targets before adjusting individual joints.
The secondary **Direct movement — no collision checking** section sends those targets
directly, including on hardware; sending does not confirm arrival.
To position a tool, select its **Active TCP**, enter all three **Target X/Y/Z [mm]**
coordinates in the displayed reference frame (normally `world`), then click
**Preview TCP plan**, review RViz, and **Execute TCP plan**. **Use current TCP position**
fills the fields from fresh state. The goal is the selected TCP origin including its
saved offset; orientation is free for this 5-DOF arm, and the planned path need not
be straight. Joint/TCP previews share one stored plan; target or tool changes require
a new preview. TCP offsets describe geometry, not additional tool collision bodies.
For a headless run, use `rviz:=false`; change the panel port with `web_port:=8081`.

In Gazebo, click **Use target above banana**, then **Preview TCP plan** and
**Execute TCP plan**. The panel waits for the collision scene to be ready. The
Gazebo demo supports planned moves, joint Jog and STOP; direct target moves and
gamepad motion are unavailable. Gazebo Jog is limited to 120 degrees/s,
without collision checking. The banana is static. Use `gazebo_gui:=false`
to disable the Gazebo window. See [Gazebo setup and first task](docs/09-gazebo.md)
for scene configuration and the remaining desktop validation.

Run tests: `colcon test --packages-select arm_sim arm_web arm_gazebo arm_nanobee_bridge && colcon test-result --verbose`

## Documentation

1. [Architecture](docs/01-architecture.md) – how the pieces fit, why this design, future upgrades
2. [Development environment](docs/02-dev-environment.md) – ROS 2 on Windows (WSL2), USB passthrough, VS Code
3. [Electronics](docs/03-electronics.md) – TinyBee, drivers, motors, power, endstops, wiring
4. [NanoBee setup](docs/04-nanobee-setup.md) – flashing, config upload, calibration, manual testing
5. [Mechanical design](docs/05-mechanical-design.md) – 3D printing, gearboxes, CAD → URDF
6. [ROS 2 software](docs/06-ros2-software.md) – packages, topics, parameters, MoveIt 2 integration
7. [Roadmap](docs/07-roadmap.md) – phased plan with checklists
8. [Safety](docs/08-safety.md) – read before powering motors
9. [Gazebo](docs/09-gazebo.md) – table/banana scene, TCP approach task and configuration

## Status

Joint simulation, MoveIt 2 joint/TCP position planning and simulation execution are available.
See [docs/07-roadmap.md](docs/07-roadmap.md) for future work.
