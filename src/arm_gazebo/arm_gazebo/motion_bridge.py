"""Arbitrate web jogging and MoveIt plans before the Gazebo trajectory controller."""

import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from arm_gazebo.jog import JointJog


class MotionBridge:
    def __init__(self, node):
        self.node = node
        self.jog = None
        self.reserved = False
        self.generation = 0
        self.reserved_generation = 0
        self.internal_goal = None
        self.cancel_pending = False
        group = ReentrantCallbackGroup()
        self.client = ActionClient(node, FollowJointTrajectory,
                                   '/gazebo_arm_controller/follow_joint_trajectory',
                                   callback_group=group)
        self.server = ActionServer(
            node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory',
            execute_callback=self._execute, goal_callback=self._goal,
            cancel_callback=self._cancel, callback_group=group)
        self.publisher = node.create_publisher(
            JointTrajectory, '/gazebo_arm_controller/joint_trajectory', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = node.create_publisher(String, '~/trajectory_status', latched)
        node.create_subscription(String, 'robot_description', self._description, latched)
        node.create_subscription(JointState, 'joint_states', self._state, 10)
        node.create_subscription(JointState, 'jog', self._jog, 10)
        node.create_timer(0.05, self._tick, clock=node._wall_clock)
        self._report('Gazebo motion ready; waiting for joint state')

    def _report(self, text):
        self.status.publish(String(data=text))

    def _description(self, message):
        if self.jog is None:
            self.jog = JointJog(message.data)

    def _state(self, message):
        if self.jog is not None:
            self.jog.update(message.name, message.position, time.monotonic())

    def _jog(self, message):
        if self.jog is None:
            return
        if self.reserved:
            # Held jog inputs must be released before they can resume after a plan.
            self.jog.blocked = any(message.velocity)
            return
        try:
            if not self.jog.command(list(message.name), list(message.velocity), time.monotonic()):
                self._report('Jog refused: release the control and wait for fresh joint state')
        except ValueError as exc:
            self._hold(self.jog.stop(time.monotonic()))
            self._report(str(exc))

    def _hold(self, positions):
        if positions is not None:
            self._publish(positions, hold=True)

    def _publish(self, positions, hold=False):
        trajectory = JointTrajectory(joint_names=self.jog.names)
        if hold:
            trajectory.points = [JointTrajectoryPoint(
                positions=positions, velocities=[0.0] * len(positions))]
        else:
            # Anchor each replacement segment to measured state. Interpolating
            # from the previous desired point can add a step to a jog command.
            target = JointTrajectoryPoint(positions=positions)
            target.time_from_start.nanosec = int(self.jog.horizon * 1e9)
            trajectory.points = [JointTrajectoryPoint(positions=self.jog.positions), target]
        # A zero header timestamp starts the bounded segment on receipt in sim time.
        self.publisher.publish(trajectory)

    def _tick(self):
        if self.jog is None or self.reserved:
            return
        command = self.jog.tick(time.monotonic(), self.node.get_clock().now().nanoseconds/1e9)
        if command is not None:
            positions, hold = command
            self._publish(positions, hold)
            self._report('Jog stopped' if hold else 'Jogging (without collision checking)')

    def stop(self):
        self.generation += 1
        if self.jog is not None:
            self._hold(self.jog.stop(time.monotonic(), disarm=True))
        if self.internal_goal is not None:
            self.internal_goal.cancel_goal_async()
        self._report('STOP requested')

    def _goal(self, _request):
        now = time.monotonic()
        if (self.reserved or self.jog is None or not self.jog.fresh(now) or
                self.jog.busy(now) or not self.client.server_is_ready()):
            self._report('Plan refused: release Jog and wait for the controller to stop')
            return GoalResponse.REJECT
        self.reserved = True
        self.reserved_generation = self.generation
        self.cancel_pending = False
        return GoalResponse.ACCEPT

    def _cancel(self, _handle):
        self.cancel_pending = True
        if self.internal_goal is not None:
            self.internal_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def _execute(self, handle):
        generation = self.reserved_generation
        result = FollowJointTrajectory.Result()
        result.error_code = result.INVALID_GOAL
        try:
            if generation != self.generation or handle.is_cancel_requested or self.cancel_pending:
                if handle.is_cancel_requested:
                    handle.canceled()
                else:
                    handle.abort()
                return result

            def feedback(message):
                if handle.is_active:
                    handle.publish_feedback(message.feedback)

            self.internal_goal = await self.client.send_goal_async(
                handle.request, feedback_callback=feedback)
            if not self.internal_goal.accepted:
                result.error_string = 'Gazebo controller rejected the trajectory'
                handle.abort()
                return result
            if generation != self.generation or handle.is_cancel_requested or self.cancel_pending:
                self.internal_goal.cancel_goal_async()
            self._report('Executing planned trajectory')
            response = await self.internal_goal.get_result_async()
            result = response.result
            if generation != self.generation or handle.is_cancel_requested or self.cancel_pending:
                result.error_code = result.INVALID_GOAL
                result.error_string = 'Trajectory stopped'
                if handle.is_cancel_requested:
                    handle.canceled()
                else:
                    handle.abort()
            elif response.status == GoalStatus.STATUS_SUCCEEDED and result.error_code == result.SUCCESSFUL:
                handle.succeed()
            else:
                if result.error_code == result.SUCCESSFUL:
                    result.error_code = result.INVALID_GOAL
                    result.error_string = 'Controller ended without a successful action result'
                handle.abort()
            self._report('Planned trajectory finished: ' + result.error_string)
            return result
        except Exception as exc:
            if self.internal_goal is not None and self.internal_goal.accepted:
                self.internal_goal.cancel_goal_async()
            result.error_string = str(exc)
            if handle.is_active:
                handle.abort()
            return result
        finally:
            self.internal_goal = None
            self.reserved = False
