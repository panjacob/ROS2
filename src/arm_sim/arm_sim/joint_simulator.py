"""ROS interface for a simple joint simulator, using the FluidNC bridge topics."""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
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
        self._last_tick = time.monotonic()
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
        self.motion.command(msg.name, msg.position)

    def _on_jog(self, msg):
        self.motion.set_jog(msg.name, msg.velocity, time.monotonic())

    def _on_stop(self, _request, response):
        self.motion.stop()
        response.success = True
        response.message = 'Simulation stopped'
        return response

    def _on_unlock(self, _request, response):
        response.success = True
        response.message = 'Simulation has no alarm'
        return response

    def _tick(self):
        now = time.monotonic()
        dt = min(now - self._last_tick, 0.1)
        self._last_tick = now
        previous = self.motion.position.copy()
        moving = self.motion.step(dt, now)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.motion.position)
        msg.position = list(self.motion.position.values())
        self._joint_pub.publish(msg)
        self._state_pub.publish(String(data='Run' if moving else 'Idle'))
        self._pins_pub.publish(String(data=''))
        distance = math.sqrt(sum((self.motion.position[name] - value) ** 2
                                 for name, value in previous.items()))
        self._feed_pub.publish(Float64(data=math.degrees(distance / dt) * 60 if dt > 0 else 0.0))
        self._spindle_pub.publish(Float64(data=0.0))


def main(args=None):
    rclpy.init(args=args)
    node = JointSimulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
