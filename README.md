# ROS 2 3D-Printed Robotic Arm (MKS TinyBee + FluidNC)

A 5-DOF 3D-printed robotic arm controlled from **ROS 2**. Joints use stepper motors driven by an
**MKS TinyBee** (ESP32) running **FluidNC**. A ROS 2 node translates joint commands into G-code and
reads joint positions back from FluidNC status reports.

```
 ROS 2 (RViz, MoveIt 2, your nodes)
        │  sensor_msgs/JointState, trajectory_msgs/JointTrajectory
        ▼
 arm_fluidnc_bridge  ──USB serial / Wi-Fi telnet──►  FluidNC on MKS TinyBee  ──►  5 x stepper drivers  ──►  arm
        ▲                                                   │
        └──────────── status "<Idle|MPos:...>" ◄────────────┘
```

## Repository layout

| Path | Contents |
|------|----------|
| [src/arm_description](src/arm_description) | URDF/xacro model, meshes, RViz preview launch |
| [src/arm_fluidnc_bridge](src/arm_fluidnc_bridge) | Python node: ROS 2 ⇄ FluidNC (G-code over serial/TCP) |
| [src/arm_start](src/arm_start) | Launch files and parameters for sim / real hardware |
| [src/arm_moveit_config](src/arm_moveit_config) | MoveIt 2 arm group, collision settings and OMPL planner |
| [src/arm_web](src/arm_web) | Web page: live state, joint planning and execution, jog, go-to, TCPs, gamepad |
| [src/arm_teleop](src/arm_teleop) | Gamepad (`/joy`) → joint jog |
| [firmware/fluidnc/config.yaml](firmware/fluidnc/config.yaml) | FluidNC machine config for the TinyBee (axes = joints, units = degrees) |
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

# 3. Real hardware (edit src/arm_start/config/fluidnc_bridge.yaml first)
#    control it at http://localhost:8080 (a connected gamepad works too)
ros2 launch arm_start arm.launch.py use_hardware:=true
```

Hardware service and topic commands need a second terminal with ROS and this
workspace sourced; see [ROS 2 software](docs/06-ros2-software.md#62-fluidnc_bridge-node).

For step 2, open **http://localhost:8080** after the launch starts. In the Joints
table, enter all five angles under **Target** (degrees by default; choose radians with **Joint units**), then click **Preview plan**. Review the animation in RViz. Click **Execute plan** to move
the simulated robot; the panel reports completion or failure. **STOP** interrupts
execution. A plan can be executed once within 120 seconds. Moving the arm with
direct movement or jog invalidates the plan, so plan again before executing.

With `use_hardware:=true`, planning and RViz preview still work, but Execute is
disabled because the FluidNC bridge has no trajectory action controller yet.
Use **Use current angles** to fill the shared targets before adjusting individual joints.
The secondary **Direct movement — no collision checking** section sends those targets
directly, including on hardware; sending does not confirm arrival.
For a headless run, use `rviz:=false`; change the panel port with `web_port:=8081`.

Run tests: `colcon test --packages-select arm_sim arm_web arm_fluidnc_bridge && colcon test-result --verbose`

## Documentation

1. [Architecture](docs/01-architecture.md) – how the pieces fit, why this design, future upgrades
2. [Development environment](docs/02-dev-environment.md) – ROS 2 on Windows (WSL2), USB passthrough, VS Code
3. [Electronics](docs/03-electronics.md) – TinyBee, drivers, motors, power, endstops, wiring
4. [FluidNC setup](docs/04-fluidnc-setup.md) – config upload, calibration, manual testing
5. [Mechanical design](docs/05-mechanical-design.md) – 3D printing, gearboxes, CAD → URDF
6. [ROS 2 software](docs/06-ros2-software.md) – packages, topics, parameters, MoveIt 2 integration
7. [Roadmap](docs/07-roadmap.md) – phased plan with checklists
8. [Safety](docs/08-safety.md) – read before powering motors

## Status

Joint simulation, MoveIt 2 joint planning and simulation execution are available.
See [docs/07-roadmap.md](docs/07-roadmap.md) for future work.
