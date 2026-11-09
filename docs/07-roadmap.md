# 7. Roadmap

Work in small, testable phases. Do not move on until the exit criteria pass.

## Phase 0 – Environment & skeleton ✔ (this commit)
- [x] Workspace with `arm_description`, `arm_fluidnc_bridge`, `arm_start`
- [x] FluidNC config template for TinyBee
- [ ] WSL2 + ROS 2 Jazzy installed, `colcon build` passes ([02](02-dev-environment.md))
- [ ] `ros2 launch arm_description display.launch.py` shows the placeholder arm

## Phase 1 – Electronics test bench (no arm yet)
- [ ] Drivers + Vref set, 5 motors on the desk with tape flags ([03](03-electronics.md))
- [ ] Config uploaded, each motor jogs from WebUI ([04](04-fluidnc-setup.md))
- [ ] Endstops read correctly in `?` reports, `$H` works on bench
- [ ] Bridge connected: `ros2 topic echo /joint_states` changes when jogging from WebUI
- [ ] `joint_command` moves motors; `stop` service halts them

**Exit:** ROS 2 can move all 5 motors and read positions back.

## Phase 2 – Mechanical prototype
- [ ] Requirements & torque budget written ([05](05-mechanical-design.md))
- [ ] Gearbox test print for the shoulder – measure backlash & torque
- [ ] Base + shoulder assembled, calibrated (direction, steps/deg, homing)
- [ ] Elbow, wrist pitch, wrist roll assembled and calibrated
- [ ] Cable management, E-stop installed ([08](08-safety.md))

**Exit:** each joint homes, reaches its limits and returns to 0° within ±1°.

## Phase 3 – Accurate model
- [ ] Link meshes exported from CAD into `arm_description/meshes`
- [ ] URDF dimensions/limits match the real arm and FluidNC config
- [ ] Real arm and RViz model agree visually in several poses

## Phase 4 – Motion planning
- [ ] `arm_moveit_config` generated ([06](06-ros2-software.md#63-adding-moveit-2))
- [ ] Plans executed in simulation (mock hardware)
- [ ] `arm_hardware` ros2_control interface implemented
- [ ] MoveIt executes plans on the real arm at reduced speed

## Phase 5 – Gripper & applications
- [ ] Gripper designed (servo on FluidNC C axis or separate MCU)
- [ ] Gripper exposed as a ROS 2 action (`control_msgs/GripperCommand`)
- [ ] Pick & place demo with taught poses
- [ ] Optional: camera + ArUco/AprilTag for object poses, teleop with a gamepad

## Phase 6 – Improvements
- [ ] Joint encoders (AS5600/AS5048) for closed-loop position check
- [ ] Stall/missed-step detection (compare encoder vs commanded)
- [ ] Gravity compensation / counterbalance tuning
- [ ] CI: GitHub Actions running `colcon build` + tests in `ros:jazzy` container
