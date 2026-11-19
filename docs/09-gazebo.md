# Gazebo: table and banana

The Gazebo backend extends the existing MoveIt/web workflow with a physical arm
model, a table, floor and a curved banana. It uses the same
`/arm_controller/follow_joint_trajectory` action as the simple simulator and FluidNC.
The default simple simulator remains available without launching Gazebo.

## Install and launch

Use the Gazebo version paired with your ROS distribution: Lyrical + Jetty
(the current development environment), or Jazzy + Harmonic.

```bash
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
ros2 launch arm_start arm.launch.py simulator:=gazebo
```

Gazebo, RViz and the panel at http://localhost:8080 start together. The launch
spawns the arm, activates the joint state broadcaster and trajectory controller,
then installs the environment into MoveIt. The panel waits for confirmation of
the collision scene before allowing planning or execution.

For a server without GUI windows:

```bash
ros2 launch arm_start arm.launch.py simulator:=gazebo gazebo_gui:=false rviz:=false
```

`web_port:=8081` changes the panel port. `model:=arm.urdf.xacro` selects the primitive
arm model. `use_hardware:=true` and `simulator:=gazebo` are rejected together.
Do not run another arm backend in the same ROS domain at the same time.

## First task: approach the banana

1. Open the panel and wait for **Gazebo tabletop ready** and fresh joint/TCP state.
2. Choose **tool0**, or an existing TCP offset appropriate to your tool.
3. Click **Use target above banana**. The default target is approximately
   **X=280, Y=35, Z=139 mm** in `world`, 100 mm above the banana's highest collision
   sphere. This target describes the selected TCP origin.
4. Click **Preview TCP plan** and review the animation and environment in RViz.
5. Click **Execute TCP plan**. Observe the physical model move in Gazebo and check
   the TCP readout against the requested target. Repeat the preview and execution
   to test repeatability.
6. **STOP** cancels the trajectory controller's action goals; the controller holds
   position. A new preview is required after STOP.

The tool0 frame is the flange endpoint; this version adds no physical gripper.
Saved TCP offsets select a point relative to the flange and do not add tool
collision geometry. Orientation remains free for the 5-DOF arm, and the planned
path is not necessarily straight.

The Gazebo backend supports planned joint/TCP motion, joint Jog and STOP. Gamepad,
direct target moves and homing are unavailable in this mode. The simple simulator
and FluidNC keep their controls.

## Joint Jog

Hold the minus or plus button in the **Jog** column. Release the pointer/key or
leave the browser window to stop. The speed slider and the simulation-only URDF
velocity limits are 120 degrees/s in Gazebo. Physical-arm URDF velocity limits
remain unchanged. Each joint also obeys its position limits. Jog moves
directly without MoveIt collision checking.

Jog commands are refreshed while held. After 0.3 seconds without a command, the
adapter stops jogging. Every command has a bounded 0.12-second trajectory horizon,
so even a stopped adapter cannot leave an unbounded velocity command running.
Stale joint state also stops command generation. Pausing Gazebo does not accumulate
movement against wall time.

Jog invalidates an existing preview. Release Jog and wait briefly for the arm to
stop before previewing another plan. While a plan executes, Jog is rejected.
STOP clears jogging and cancels the planned trajectory, including a plan whose
controller acceptance is still pending. A held external `/jog` publisher must send
a neutral command before resuming after STOP; the web panel does this automatically.

The motion adapter owns the public `/arm_controller/follow_joint_trajectory`
action. It forwards plans to the internal `gazebo_arm_controller` and sends short
jog segments to that controller only while no plan owns it. Avoid commanding the
internal controller directly: that bypasses this arbitration.

## Scene and coordinate conventions

`src/arm_gazebo/config/scene.yaml` is the geometry source for both Gazebo and MoveIt.
There are no Fuel downloads or external model assets. The banana visual is a
curved chain of tapered spheres joined by cylinders, embedded in the SDF;
its collision model is a chain of spheres. The initial camera shows the tabletop
from close range rather than Gazebo's default distant view.
The table and floor use boxes. RViz shows the planner's collision approximation.

- `world` is at the robot mounting plane, with the tabletop at **z=0**.
- The floor is at **z=-0.75 m**; the robot base is fixed to the world.
- The banana model origin defaults to **(0.28, 0, 0.021) m**; its curved centreline
  bends toward positive Y.
- Only the base/table mounting contact is allowed in MoveIt. The other arm links
  must avoid the table. Existing robot self-collision rules are preserved.
- All scene geometry and ROS positions use metres; the panel accepts millimetres.

To try another banana position, copy the YAML, edit `banana.position` and restart:

```bash
ros2 launch arm_start arm.launch.py simulator:=gazebo scene_config:=/absolute/path/scene.yaml
```

The table, banana and floor are deliberately **static** in this first version.
Their poses are loaded at startup; do not reposition them through the Gazebo GUI,
because GUI edits are not synchronized back into MoveIt. Restart after scene
changes. The scene manager reapplies the static collision scene periodically,
including after MoveIt restarts. Moving objects, perception and physical grasping
are future extensions.

The server and GUI run as separate processes managed directly by launch. Each run
uses a unique Gazebo transport partition so it does not connect to an old world.
The clock bridge reads `/world/tabletop/clock` and publishes ROS `/clock`.
This transport partition does not isolate the ROS graph: stop all previous arm
launches and their Gazebo servers before starting another instance in the same
ROS domain. If logs report `Another world of the same name is running` or duplicate
controller managers, a previous simulation has not finished shutting down.

Gazebo supplies `/clock`; MoveIt, TF, controllers, panel and RViz use simulation
time in this mode. Pausing Gazebo also pauses motion. Restart the launch to reset
the demo; resetting time through the GUI during execution is not supported.

## Validation

```bash
python3 -m pytest src/arm_gazebo/test src/arm_web/test src/arm_sim/test src/arm_teleop/test -q
ros2 control list_controllers
ros2 topic echo /arm_gazebo/scene_ready --once
```

The controller list should contain active `joint_state_broadcaster` and
`gazebo_arm_controller`; scene readiness should be `true`. Offline tests check shared
geometry, approach clearance, preservation of collision rules, readiness gating
and panel behavior, as well as jogging limits, release/watchdog stops, pause,
STOP, action forwarding and plan/Jog arbitration. The SDF world can also be
validated with `gz sdf -k`.

At implementation time, package builds, offline tests and SDF/URDF conversion were
checked. An additional offline check with MoveIt's collision engine found 24
collision-free tool0 solutions for the default approach target, including sampled
joint-space paths from home. Full Gazebo motion remains to be validated on the local desktop: the
execution sandbox blocks the network sockets used by Gazebo transport and ROS DDS.

Official references:
- [Gazebo / ROS compatibility](https://gazebosim.org/docs/latest/ros_installation/)
- [gz_ros2_control](https://control.ros.org/lyrical/doc/gz_ros2_control/doc/index.html)
- [MoveIt collision objects](https://moveit.picknik.ai/main/doc/tutorials/planning_around_objects/planning_around_objects.html)
