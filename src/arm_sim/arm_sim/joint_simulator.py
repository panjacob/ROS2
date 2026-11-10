"""ROS interface for a simple joint simulator, using the FluidNC bridge topics."""

import math
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

from arm_sim.motion import JointMotion, joints_from_urdf


class JointSimulator(Node):
    def __init__(self):
        super().__init__('arm_sim')
        description = self.declare_parameter('robot_description', '').value
        if not description:
            raise ValueError('robot_description is required')
        timeout = float(self.declare_parameter('jog_timeout', 0.3).value)
        rate = float(self.declare_parameter('publish_rate', 50.0).value)
        if timeout <= 0 or rate <= 0:
            raise ValueError('jog_timeout and publish_rate must be positive')
        self.motion = JointMotion(joints_from_urdf(description), timeout)
        self._motion_lock = threading.RLock()
        self._goal_reserved = False
        self._last_tick = time.monotonic()
        self._trajectory_action = ActionServer(
            self, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_trajectory,
            goal_callback=self._on_trajectory_goal,
            cancel_callback=self._on_trajectory_cancel,
            callback_group=ReentrantCallbackGroup())
        self._joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._state_pub = self.create_publisher(String, '~/machine_state', 10)
        self._pins_pub = self.create_publisher(String, '~/limit_switches', 10)
        self._feed_pub = self.create_publisher(Float64, '~/feed_rate', 10)
        self._spindle_pub = self.create_publisher(Float64, '~/spindle_speed', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._homing_pub = self.create_publisher(String, '~/homing_status', latched)
        self.create_subscription(JointState, 'joint_command', self._on_command, 10)
        self.create_subscription(JointState, 'jog', self._on_jog, 10)
        self.create_service(Trigger, '~/stop', self._on_stop)
        self.create_service(Trigger, '~/unlock', self._on_unlock)
        self.create_timer(1.0 / rate, self._tick)
        self._homing_pub.publish(String(data='Simulation: no endstops'))
        self.get_logger().info('Joint simulator ready')

    def _on_command(self, msg):
        with self._motion_lock:
            self.motion.command(msg.name, msg.position)

    def _on_jog(self, msg):
        with self._motion_lock:
            self.motion.set_jog(msg.name, msg.velocity, time.monotonic())

    def _on_stop(self, _request, response):
        with self._motion_lock:
            self.motion.stop()
        response.success = True
        response.message = 'Simulation stopped'
        return response

    def _on_trajectory_goal(self, goal):
        with self._motion_lock:
            if self._goal_reserved or not goal.trajectory.points:
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _on_trajectory_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _execute_trajectory(self, goal_handle):
        result = FollowJointTrajectory.Result()
        try:
            trajectory = goal_handle.request.trajectory
            points = [
                (p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
                 list(p.positions)) for p in trajectory.points]
            try:
                with self._motion_lock:
                    self.motion.start_trajectory(
                        trajectory.joint_names, points, time.monotonic())
            except ValueError as exc:
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = str(exc)
                goal_handle.abort()
                return result

            deadline = time.monotonic() + points[-1][0] + 3.0
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    with self._motion_lock:
                        self.motion.stop()
                    goal_handle.canceled()
                    result.error_string = 'Trajectory canceled'
                    return result
                with self._motion_lock:
                    status = self.motion.trajectory_result
                if status == 'completed':
                    goal_handle.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    return result
                if status == 'interrupted':
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    result.error_string = 'Trajectory interrupted by another command or STOP'
                    return result
                time.sleep(0.02)
            with self._motion_lock:
                self.motion.stop()
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = 'Trajectory did not finish in time'
            return result
        finally:
            with self._motion_lock:
                self._goal_reserved = False

    def _on_unlock(self, _request, response):
        response.success = True
        response.message = 'Simulation has no alarm'
        return response

    def _tick(self):
        now = time.monotonic()
        dt = min(now - self._last_tick, 0.1)
        self._last_tick = now
        with self._motion_lock:
            previous = self.motion.position.copy()
            moving = self.motion.step(dt, now)
            positions = self.motion.position.copy()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(positions)
        msg.position = list(positions.values())
        self._joint_pub.publish(msg)
        self._state_pub.publish(String(data='Run' if moving else 'Idle'))
        self._pins_pub.publish(String(data=''))
        distance = math.sqrt(sum((positions[name] - value) ** 2
                                 for name, value in previous.items()))
        self._feed_pub.publish(Float64(data=math.degrees(distance / dt) * 60 if dt > 0 else 0.0))
        self._spindle_pub.publish(Float64(data=0.0))


def main(args=None):
    rclpy.init(args=args)
    node = JointSimulator()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node._trajectory_action.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
