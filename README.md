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
| [src/arm_web](src/arm_web) | Web page: live state, jog, go-to, endstop homing, browser gamepad |
| [src/arm_teleop](src/arm_teleop) | Gamepad (`/joy`) → joint jog |
| [firmware/fluidnc/config.yaml](firmware/fluidnc/config.yaml) | FluidNC machine config for the TinyBee (axes = joints, units = degrees) |
| [docs](docs) | Step-by-step development guides |
| `hardware/` (create it) | CAD sources, STEP, STL, BOM, wiring diagrams |

## Quick start

Requires Ubuntu 24.04 + ROS 2 Jazzy (on Windows use WSL2 – see [docs/02-dev-environment.md](docs/02-dev-environment.md)).

```bash
cd ~/ros2_arm            # this repository
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash

# 1. Preview the mesh model with joint sliders
ros2 launch arm_description display.launch.py

# 2. Joint simulation; use Go/jog/gamepad and manage TCPs at http://localhost:8080
ros2 launch arm_start arm.launch.py

# Optional: select the original primitive model instead
ros2 launch arm_start arm.launch.py model:=arm.urdf.xacro

# 3. Real hardware (edit src/arm_start/config/fluidnc_bridge.yaml first)
#    control it at http://localhost:8080 (a connected gamepad works too)
ros2 launch arm_start arm.launch.py use_hardware:=true
ros2 service call /fluidnc_bridge/home std_srvs/srv/Trigger
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState \
  "{name: [joint1, joint2], position: [0.5, 0.2]}"
```

Run unit tests: `colcon test --packages-select arm_fluidnc_bridge && colcon test-result --verbose`

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

Phase 0 – project skeleton. See [docs/07-roadmap.md](docs/07-roadmap.md).
