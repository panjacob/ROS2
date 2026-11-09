"""Gamepad -> joint velocity jog.

  sub  joy  sensor_msgs/Joy        (from the web page, browser "standard" gamepad layout)
  pub  jog  sensor_msgs/JointState (velocity [rad/s] per joint, consumed by fluidnc_bridge)

Default layout (Xbox naming): left stick X -> joint1, left stick Y -> joint2,
right stick Y -> joint3, right stick X -> joint4, LB/RB -> joint5, B -> stop.
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_srvs.srv import Trigger


def shape(raw, deadzone, expo):
    """Deadzone + expo curve; returns -1..1."""
    raw = max(-1.0, min(1.0, raw))
    if abs(raw) <= deadzone:
        return 0.0
    return math.copysign(((abs(raw) - deadzone) / (1.0 - deadzone)) ** expo, raw)


def joy_to_velocities(axes, buttons, cfg):
    """Map one Joy reading to joint velocities [deg/s].

    cfg keys: axis_index, axis_scale, button_minus, button_plus (per joint, -1 = unused),
    deadzone, expo, max_speed, enable_button (-1 = not required).
    """
    def button(idx):
        return 0 <= idx < len(buttons) and buttons[idx] > 0

    n = len(cfg['axis_index'])
    if cfg['enable_button'] >= 0 and not button(cfg['enable_button']):
        return [0.0] * n
    vel = []
    for j in range(n):
        value = 0.0
        a = cfg['axis_index'][j]
        if 0 <= a < len(axes):
            value = shape(axes[a], cfg['deadzone'], cfg['expo']) * cfg['axis_scale'][j]
        if button(cfg['button_plus'][j]):
            value += 1.0
        if button(cfg['button_minus'][j]):
            value -= 1.0
        vel.append(max(-1.0, min(1.0, value)) * cfg['max_speed'])
    return vel


class JoyTeleop(Node):

    def __init__(self):
        super().__init__('joy_teleop')
        joints = self.declare_parameter(
            'joints', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']).value
        self._joints = list(joints)
        n = len(self._joints)
        self._cfg = {
            'axis_index': list(self.declare_parameter('axis_index', [0, 1, 3, 2, -1]).value),
            'axis_scale': list(self.declare_parameter(
                'axis_scale', [1.0, -1.0, -1.0, 1.0, 0.0]).value),
            'button_minus': list(self.declare_parameter('button_minus', [-1, -1, -1, -1, 4]).value),
            'button_plus': list(self.declare_parameter('button_plus', [-1, -1, -1, -1, 5]).value),
            'deadzone': float(self.declare_parameter('deadzone', 0.1).value),
            'expo': float(self.declare_parameter('expo', 1.8).value),
            'max_speed': float(self.declare_parameter('max_speed', 180.0).value),
            'enable_button': int(self.declare_parameter('enable_button', -1).value),
        }
        for key in ('axis_index', 'axis_scale', 'button_minus', 'button_plus'):
            if len(self._cfg[key]) != n:
                raise ValueError(f'{key} must have one entry per joint ({n})')
        self._stop_button = int(self.declare_parameter('stop_button', 1).value)
        self._joy_timeout = float(self.declare_parameter('joy_timeout', 0.5).value)
        rate = float(self.declare_parameter('rate', 20.0).value)
        stop_service = self.declare_parameter('stop_service', '/fluidnc_bridge/stop').value

        self._jog_pub = self.create_publisher(JointState, 'jog', 10)
        self._stop_client = self.create_client(Trigger, stop_service)
        self.create_subscription(Joy, 'joy', self._on_joy, 10)
        self.create_timer(1.0 / rate, self._tick)
        self._vel = [0.0] * n
        self._stamp = 0.0
        self._active = False
        self._stop_was_down = False
        self.get_logger().info('Gamepad teleop ready (waiting for /joy)')

    def _on_joy(self, msg):
        self._vel = joy_to_velocities(list(msg.axes), list(msg.buttons), self._cfg)
        self._stamp = time.monotonic()
        stop_down = 0 <= self._stop_button < len(msg.buttons) and msg.buttons[self._stop_button] > 0
        if stop_down and not self._stop_was_down:
            self.get_logger().warn('Stop button pressed')
            self._vel = [0.0] * len(self._joints)
            if self._stop_client.service_is_ready():
                self._stop_client.call_async(Trigger.Request())
        self._stop_was_down = stop_down

    def _tick(self):
        fresh = time.monotonic() - self._stamp < self._joy_timeout
        vel = self._vel if fresh else [0.0] * len(self._joints)
        moving = any(vel)
        # Publish continuously while moving; a single zero when released so web jog buttons
        # (same topic) are not overridden by an idle gamepad.
        if moving or self._active:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self._joints
            msg.velocity = [math.radians(v) for v in vel]
            self._jog_pub.publish(msg)
        self._active = moving


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
