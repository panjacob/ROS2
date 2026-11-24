"""Install the static MoveIt world and host Gazebo motion, jogging and STOP."""

from collections import defaultdict
from copy import deepcopy

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (AllowedCollisionEntry, CollisionObject, ObjectColor,
                             PlanningScene, PlanningSceneComponents)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from arm_gazebo.scene import approach_target, collision_shapes, load_scene
from arm_gazebo.motion_bridge import MotionBridge


def planning_scene(scene):
    result = PlanningScene(is_diff=True)
    objects = defaultdict(list)
    for name, kind, dimensions, position in collision_shapes(scene):
        objects[name].append((kind, dimensions, position))
    for name, shapes in objects.items():
        obj = CollisionObject(id=name, operation=CollisionObject.ADD)
        obj.header.frame_id = 'world'
        obj.pose.orientation.w = 1.0
        for kind, dimensions, position in shapes:
            obj.primitives.append(SolidPrimitive(
                type=SolidPrimitive.BOX if kind == 'box' else SolidPrimitive.SPHERE,
                dimensions=[float(v) for v in dimensions]))
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, position)
            pose.orientation.w = 1.0
            obj.primitive_poses.append(pose)
        result.world.collision_objects.append(obj)
        color = ObjectColor(id=name)
        color.color.r, color.color.g, color.color.b, color.color.a = {
            'table': (0.46, 0.29, 0.14, 1.0), 'floor': (0.55, 0.58, 0.62, 1.0),
            'banana': (0.98, 0.78, 0.05, 1.0)}[name]
        result.object_colors.append(color)
    return result


def allow_mount_contact(matrix):
    """Permit only the mounted base/table contact, preserving SRDF collision rules."""
    matrix = deepcopy(matrix)
    for name in ('base_link', 'table'):
        if name not in matrix.entry_names:
            matrix.entry_names.append(name)
            for row in matrix.entry_values:
                row.enabled.append(False)
            matrix.entry_values.append(AllowedCollisionEntry(
                enabled=[False] * len(matrix.entry_names)))
    base, table = (matrix.entry_names.index(name) for name in ('base_link', 'table'))
    matrix.entry_values[base].enabled[table] = True
    matrix.entry_values[table].enabled[base] = True
    return matrix


class SceneManager(Node):
    def __init__(self):
        super().__init__('arm_gazebo')
        scene = load_scene(self.declare_parameter('scene_config', '').value)
        self._scene = planning_scene(scene)
        group = ReentrantCallbackGroup()
        self._apply = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self._get = self.create_client(GetPlanningScene, 'get_planning_scene')
        self._cancel = self.create_client(
            CancelGoal, '/arm_controller/follow_joint_trajectory/_action/cancel_goal',
            callback_group=group)
        self.create_service(Trigger, '~/stop', self._stop, callback_group=group)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._ready = self.create_publisher(Bool, '~/scene_ready', latched)
        self._status = self.create_publisher(String, '~/machine_state', latched)
        self._homing = self.create_publisher(String, '~/homing_status', latched)
        self._homing.publish(String(data='Gazebo: no homing required'))
        self._ready.publish(Bool(data=False))
        self._pending = None
        # Wall clock also works while simulation is paused or before /clock starts.
        self._wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._motion = MotionBridge(self)
        self.create_timer(2.0, self._sync, clock=self._wall_clock)
        self.get_logger().info(
            'Banana approach target in world [mm]: ' +
            ', '.join(f'{v*1000:.1f}' for v in approach_target(scene)))

    def _sync(self):
        if self._pending is not None and not self._pending.done():
            return
        if not self._apply.service_is_ready() or not self._get.service_is_ready():
            self._ready.publish(Bool(data=False))
            self._status.publish(String(data='Waiting for MoveIt planning scene'))
            return
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        self._pending = self._get.call_async(request)
        self._pending.add_done_callback(self._got_matrix)

    def _got_matrix(self, future):
        try:
            self._scene.allowed_collision_matrix = allow_mount_contact(
                future.result().scene.allowed_collision_matrix)
            self._pending = self._apply.call_async(ApplyPlanningScene.Request(scene=self._scene))
            self._pending.add_done_callback(self._applied)
        except Exception as exc:
            self.get_logger().error(f'Cannot retrieve collision rules: {exc}')
            self._ready.publish(Bool(data=False))

    def _applied(self, future):
        try:
            success = future.result().success
        except Exception as exc:
            self.get_logger().error(f'Cannot apply tabletop scene: {exc}')
            success = False
        self._ready.publish(Bool(data=success))
        self._status.publish(String(data='Gazebo tabletop ready' if success else 'Scene unavailable'))

    async def _stop(self, _request, response):
        if hasattr(self, '_motion'):
            self._motion.stop()
        if not self._cancel.service_is_ready():
            response.message = 'Trajectory controller cancellation service unavailable'
            return response
        # A zero goal id and timestamp cancel all goals. The controller then holds position.
        request = self._cancel.call_async(CancelGoal.Request())
        completed = Future()

        def finish(value):
            if not completed.done():
                completed.set_result(value)

        timer = self.create_timer(2.0, lambda: finish(None), clock=self._wall_clock,
                                  callback_group=ReentrantCallbackGroup())
        request.add_done_callback(lambda value: finish(value))
        try:
            result = await completed
            if result is None:
                response.message = 'Timed out waiting for trajectory cancellation'
            else:
                cancellation = result.result()
                response.success = cancellation.return_code in (
                    CancelGoal.Response.ERROR_NONE,
                    CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID,
                    CancelGoal.Response.ERROR_GOAL_TERMINATED,
                )
                response.message = ('Gazebo trajectory cancellation accepted' if response.success
                                    else 'Trajectory controller rejected cancellation')
        except Exception as exc:
            response.message = f'Trajectory cancellation failed: {exc}'
        finally:
            self.destroy_timer(timer)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SceneManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
