"""ROS 2 <-> FluidNC bridge.

FluidNC axes are configured so that 1 "mm" == 1 degree of joint rotation.
ROS side uses radians (sensor_msgs/JointState convention).
Joint angles are FluidNC work coordinates (WPos = MPos - WCO); endstop homing sets them.

Topics:
  pub  joint_states      sensor_msgs/JointState      (from FluidNC status reports)
  pub  ~/homing_status   std_msgs/String             (latched progress/result of endstop homing)
  sub  joint_command     sensor_msgs/JointState      (move to absolute positions, max feed)
  sub  joint_trajectory  trajectory_msgs/JointTrajectory (timed sequence of points)
  sub  jog               sensor_msgs/JointState      (velocity [rad/s] per joint; keep publishing,
                                                      motion stops after jog_timeout without messages)
  pub  ~/trajectory_status std_msgs/String           (latched: armed/executing/result/disarm)
Actions:
  /arm_controller/follow_joint_trajectory  control_msgs/FollowJointTrajectory (MoveIt)
Services:
  ~/home          $H  (FluidNC homing cycle, needs homing configured in FluidNC)
  ~/home_<joint>  endstop homing of one joint (homing_* parameters)
  ~/unlock        $X  (clear alarm) and deliberately re-arm trajectory execution; arms only after
                  $X is acknowledged ok and a later status report is not Alarm
  ~/stop          feed hold, then soft reset once a fresh status report confirms the arm stopped;
                  clears queued moves, aborts homing and trajectory goals,
                  disarms trajectory execution until ~/unlock

joint_command, joint_trajectory, jog and homing are refused while a trajectory goal is active;
a trajectory goal is refused while any of them is active.
While a STOP is finishing (feed hold sent, soft reset not yet done) every motion source is
refused (joint_command, joint_trajectory, jog, homing, unlock, trajectory goals) and the writer
sends no queued line. After a STOP, jog stays inhibited until a zero-velocity jog message (or a
pause of at least jog_timeout without jog messages): a held stick/button never resumes motion.
"""

import math
import queue
import socket
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

from arm_sim.motion import joints_from_urdf

from arm_fluidnc_bridge.fluidnc_protocol import (
    build_jog, build_move, compute_feed, jog_segment, parse_status)
from arm_fluidnc_bridge.trajectory import (
    TrajectoryError, allowed_duration, goal_tolerances, plan_trajectory, within_tolerance)

# Explicit modal state for trajectory segments: mm (= degrees here), absolute, units/min feed.
TRAJECTORY_MODES = 'G21 G90 G94'
# Largest accepted header.stamp in the future; delayed starts are not supported.
MAX_START_DELAY = 0.5
# Wait this long for a confirmed stop after feed hold before soft-resetting anyway.
STOP_CONFIRM_TIMEOUT = 5.0
# Consecutive post-hold Idle/Hold:0 reports needed: the first one may answer a '?' sent before '!'.
STOP_CONFIRM_REPORTS = 2
STOP_POLL_PERIOD = 0.1
UNLOCK_ACK_TIMEOUT = 2.0

SOFT_RESET = b'\x18'
FEED_HOLD = b'!'
STATUS_QUERY = b'?'
JOG_CANCEL = b'\x85'


class SerialTransport:

    def __init__(self, port, baud):
        import serial
        self._s = serial.Serial(port, baud, timeout=0.1)

    def write(self, data):
        self._s.write(data)

    def read(self):
        return self._s.read(self._s.in_waiting or 1)

    def close(self):
        self._s.close()


class TcpTransport:
    """FluidNC telnet server (Wi-Fi), default port 23."""

    def __init__(self, host, port):
        self._s = socket.create_connection((host, port), timeout=5.0)
        self._s.settimeout(0.1)

    def write(self, data):
        self._s.sendall(data)

    def read(self):
        try:
            data = self._s.recv(1024)
        except socket.timeout:
            return b''
        if not data:
            raise ConnectionError('FluidNC closed the connection')
        return data

    def close(self):
        self._s.close()


class _Pending:
    """Completion of one queued line (set by the writer thread)."""

    def __init__(self):
        self.event = threading.Event()
        self.ok = False


class HomingError(Exception):
    pass


class FluidNCBridge(Node):

    def __init__(self, transport=None, **kwargs):
        super().__init__('fluidnc_bridge', **kwargs)
        self.declare_parameter('transport', 'serial')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('host', '192.168.0.1')
        self.declare_parameter('tcp_port', 23)
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'])
        self.declare_parameter('axis_letters', ['X', 'Y', 'Z', 'A', 'B'])
        self.declare_parameter('max_feed', 3600.0)
        self.declare_parameter('status_rate', 10.0)
        self.declare_parameter('ack_timeout', 10.0)
        self.declare_parameter('homing_timeout', 120.0)
        self.declare_parameter('jog_rate', 20.0)
        self.declare_parameter('jog_timeout', 0.3)
        self.declare_parameter('jog_lookahead', 0.25)
        self.declare_parameter('max_jog_speed', 360.0)
        self.declare_parameter('homing_joints', ['joint1'])
        self.declare_parameter('homing_pins', ['X'])
        self.declare_parameter('homing_seek_speed', 20.0)
        self.declare_parameter('homing_locate_speed', 4.0)
        self.declare_parameter('homing_backoff', 5.0)
        self.declare_parameter('homing_max_travel', 400.0)
        self.declare_parameter('robot_description', '')
        self.declare_parameter('status_timeout', 0.5)
        self.declare_parameter('trajectory_start_tolerance', 0.02)
        self.declare_parameter('trajectory_goal_tolerance', 0.01)
        self.declare_parameter('trajectory_time_scaling', 1.5)
        self.declare_parameter('trajectory_time_margin', 2.0)

        def param(name):
            return self.get_parameter(name).value

        self._joints = list(param('joint_names'))
        self._axes = list(param('axis_letters'))
        if len(self._joints) != len(self._axes):
            raise ValueError('joint_names and axis_letters must have the same length')
        self._index = {name: i for i, name in enumerate(self._joints)}
        self._max_feed = float(param('max_feed'))
        self._ack_timeout = float(param('ack_timeout'))
        self._homing_timeout = float(param('homing_timeout'))
        self._jog_period = 1.0 / float(param('jog_rate'))
        self._jog_timeout = float(param('jog_timeout'))
        self._jog_lookahead = float(param('jog_lookahead'))
        self._max_jog_speed = float(param('max_jog_speed'))
        self._seek_speed = float(param('homing_seek_speed'))
        self._locate_speed = float(param('homing_locate_speed'))
        self._backoff = float(param('homing_backoff'))
        self._max_travel = float(param('homing_max_travel'))
        homing_joints = list(param('homing_joints'))
        homing_pins = list(param('homing_pins'))
        if len(homing_joints) != len(homing_pins):
            raise ValueError('homing_joints and homing_pins must have the same length')
        self._homing_pins = {}
        for joint, pin in zip(homing_joints, homing_pins):
            if joint not in self._index:
                raise ValueError(f'homing joint "{joint}" is not in joint_names')
            self._homing_pins[joint] = pin
        self._status_timeout = float(param('status_timeout'))
        self._start_tolerance = float(param('trajectory_start_tolerance'))
        self._goal_tolerance = float(param('trajectory_goal_tolerance'))
        self._time_scaling = float(param('trajectory_time_scaling'))
        self._time_margin = float(param('trajectory_time_margin'))
        if min(self._status_timeout, self._start_tolerance, self._goal_tolerance) <= 0.0 or \
                self._time_scaling < 1.0 or self._time_margin < 0.0:
            raise ValueError('invalid trajectory/status parameters')
        self._limits = None
        description = param('robot_description')
        if description:
            limits = joints_from_urdf(description)
            missing = [j for j in self._joints if j not in limits]
            if missing:
                raise ValueError(f'robot_description has no limits for {missing}')
            self._limits = {j: limits[j] for j in self._joints}
        else:
            self.get_logger().warn(
                'robot_description not set: trajectory goals will be rejected (no joint limits)')

        if transport is not None:
            self._io = transport
        elif param('transport') == 'tcp':
            self._io = TcpTransport(param('host'), int(param('tcp_port')))
            self.get_logger().info(f"Connected to FluidNC at {param('host')}:{param('tcp_port')}")
        else:
            self._io = SerialTransport(param('serial_port'), int(param('baud')))
            self.get_logger().info(f"Opened {param('serial_port')} @ {param('baud')}")

        n = len(self._joints)
        self._write_lock = threading.Lock()
        self._ack = threading.Event()
        self._ack_ok = False
        self._inflight = None
        self._queue = queue.Queue()
        self._state = 'Unknown'
        self._position_deg = None
        self._mpos = None
        self._wco = [0.0] * n
        self._planner_free = None
        self._last_cmd_deg = None
        self._running = True
        self._status_cond = threading.Condition()
        self._status_seq = 0
        self._last_status = None
        self._jog_lock = threading.Lock()
        self._jog_vel = [0.0] * n
        self._jog_stamp = 0.0
        self._jogging = False
        self._jog_ahead_until = 0.0
        self._jog_inhibit = False
        self._jog_msg_time = 0.0
        self._homing = False
        self._homing_generation = 0
        self._homing_abort = threading.Event()
        self._last_status_time = None
        self._generation = 0
        self._stops_pending = 0
        self._arb_lock = threading.Lock()
        self._trajectory_active = False
        self._armed = True

        self._js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._state_pub = self.create_publisher(String, '~/machine_state', 10)
        self._feed_pub = self.create_publisher(Float64, '~/feed_rate', 10)
        self._spindle_pub = self.create_publisher(Float64, '~/spindle_speed', 10)
        self._pins_pub = self.create_publisher(String, '~/limit_switches', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._homing_pub = self.create_publisher(String, '~/homing_status', latched)
        self._traj_status_pub = self.create_publisher(String, '~/trajectory_status', latched)
        self.create_subscription(JointState, 'joint_command', self._on_joint_command, 10)
        self.create_subscription(JointTrajectory, 'joint_trajectory', self._on_trajectory, 10)
        self.create_subscription(JointState, 'jog', self._on_jog, 10)
        self.create_service(Trigger, '~/home', self._on_home)
        # Unlock blocks until $X is confirmed; its own group keeps ~/stop and timers responsive.
        self.create_service(Trigger, '~/unlock', self._on_unlock,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.create_service(Trigger, '~/stop', self._on_stop)
        for joint in self._homing_pins:
            self.create_service(
                Trigger, f'~/home_{joint}',
                lambda req, res, j=joint: self._on_home_joint(j, req, res))
        self.create_timer(1.0 / float(param('status_rate')), self._request_status)
        self.create_timer(self._jog_period, self._jog_tick)
        self._trajectory_action = ActionServer(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_trajectory,
            goal_callback=self._on_trajectory_goal,
            cancel_callback=self._on_trajectory_cancel,
            callback_group=ReentrantCallbackGroup())

        threading.Thread(target=self._read_loop, daemon=True).start()
        threading.Thread(target=self._write_loop, daemon=True).start()
        self._enqueue('G90')
        self._report('not homed' if self._homing_pins else 'no endstops configured')
        self._traj_status('armed: waiting for a trajectory goal')

    # ---------------------------------------------------------------- I/O

    def _raw_write(self, data):
        with self._write_lock:
            self._io.write(data)

    def _enqueue(self, line, timeout=None, pending=None, generation=None):
        if generation is None:
            generation = self._generation
        self._queue.put((line, timeout or self._ack_timeout, pending, generation))

    def _send_wait(self, line, timeout=None, generation=None):
        """Queue a line and block until FluidNC answers ok (True) or error/timeout (False)."""
        pending = _Pending()
        self._enqueue(line, timeout, pending, generation)
        pending.event.wait((timeout or self._ack_timeout) + 1.0)
        return pending.ok

    def _clear_queue(self):
        while True:
            try:
                _line, _timeout, pending, _gen = self._queue.get_nowait()
            except queue.Empty:
                return
            if pending is not None:
                pending.event.set()

    def _request_status(self):
        try:
            self._raw_write(STATUS_QUERY)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Status request failed: {exc}', throttle_duration_sec=5.0)

    def _write_loop(self):
        # Send-response protocol: one line at a time, wait for ok/error.
        while self._running:
            try:
                line, timeout, pending, generation = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            self._ack.clear()
            with self._write_lock:
                # A STOP after this line was queued bumps the generation: never send stale lines,
                # and send nothing at all until the STOP sequence (soft reset) has finished.
                stale = generation != self._generation or self._stops_pending > 0
                if not stale:
                    self._inflight = line
                    try:
                        self._io.write((line + '\n').encode('ascii'))
                    except Exception as exc:  # noqa: BLE001
                        self.get_logger().error(f'Write failed: {exc}')
                        stale = True
                        self._inflight = None
            if stale:
                if pending is not None:
                    pending.event.set()
                continue
            self.get_logger().debug(f'>> {line}')
            ok = False
            acked = self._ack.wait(timeout)
            self._inflight = None
            if not acked:
                self.get_logger().warn(f'No ack for "{line}" within {timeout:.1f}s')
            elif not self._ack_ok:
                self.get_logger().error(f'FluidNC rejected "{line}"')
            else:
                ok = True
            if pending is not None:
                pending.ok = ok
                pending.event.set()

    def _read_loop(self):
        buf = b''
        while self._running:
            try:
                buf += self._io.read()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().fatal(f'Connection lost: {exc}')
                self._running = False
                return
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                line = raw.decode('ascii', errors='replace').strip()
                if line:
                    self._handle_line(line)

    def _handle_line(self, line):
        if line.startswith('<'):
            parsed = parse_status(line)
            if parsed:
                self._on_status(parsed)
        elif line == 'ok':
            self._ack_ok = True
            self._ack.set()
        elif line.startswith('error'):
            self.get_logger().error(f'FluidNC {line}')
            self._ack_ok = False
            self._ack.set()
        elif line.startswith('ALARM'):
            self.get_logger().error(f'FluidNC {line} - use ~/unlock or ~/home')
        else:
            self.get_logger().info(f'FluidNC: {line}')

    def _on_status(self, status):
        state = status['state']
        if state != self._state:
            self.get_logger().info(f'State: {self._state} -> {state}')
            self._state = state
        self._state_pub.publish(String(data=state))

        if status['feed'] is not None:
            self._feed_pub.publish(Float64(data=status['feed']))
        if status['spindle'] is not None:
            self._spindle_pub.publish(Float64(data=status['spindle']))
        self._pins_pub.publish(String(data=status['pins']))
        if status['planner_free'] is not None:
            self._planner_free = status['planner_free']

        n = len(self._joints)
        if status['wco'] is not None:
            wco = status['wco'][:n]
            self._wco = wco + [0.0] * (n - len(wco))

        positions = status['positions']
        status['mpos'] = None
        status['wpos'] = None
        if positions is not None and len(positions) >= n:
            if status['pos_type'] == 'MPos':
                mpos = positions[:n]
                wpos = [m - o for m, o in zip(mpos, self._wco)]
            else:
                wpos = positions[:n]
                mpos = [w + o for w, o in zip(wpos, self._wco)]
            self._mpos = mpos
            self._position_deg = wpos
            status['mpos'] = mpos
            status['wpos'] = wpos
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self._joints
            msg.position = [math.radians(v) for v in wpos]
            self._js_pub.publish(msg)

        with self._status_cond:
            self._last_status = status
            self._last_status_time = time.monotonic()
            self._status_seq += 1
            self._status_cond.notify_all()

    # ----------------------------------------------------------- commands

    def _start_deg(self):
        if self._last_cmd_deg is not None:
            return list(self._last_cmd_deg)
        if self._position_deg is not None:
            return list(self._position_deg)
        return None

    def _apply(self, target, names, positions_rad):
        for name, value in zip(names, positions_rad):
            idx = self._index.get(name)
            if idx is None:
                self.get_logger().warn(f'Unknown joint "{name}" ignored', throttle_duration_sec=5.0)
                continue
            target[idx] = math.degrees(value)

    def _busy(self, what):
        if self._stops_pending:
            self.get_logger().warn(f'{what} ignored while STOP is finishing',
                                   throttle_duration_sec=2.0)
            return True
        if self._homing:
            self.get_logger().warn(f'{what} ignored while homing', throttle_duration_sec=2.0)
            return True
        if self._trajectory_active:
            self.get_logger().warn(
                f'{what} ignored while a trajectory goal is active', throttle_duration_sec=2.0)
            return True
        return False

    def _on_joint_command(self, msg):
        with self._arb_lock:
            self._joint_command_locked(msg)

    def _joint_command_locked(self, msg):
        if self._busy('joint_command'):
            return
        target = self._start_deg()
        if target is None:
            self.get_logger().warn('No position from FluidNC yet, command dropped')
            return
        self._apply(target, msg.name, msg.position)
        self._enqueue(build_move(self._axes, target, self._max_feed))
        self._last_cmd_deg = target

    def _on_trajectory(self, msg):
        with self._arb_lock:
            self._trajectory_topic_locked(msg)

    def _trajectory_topic_locked(self, msg):
        if self._busy('joint_trajectory'):
            return
        start = self._start_deg()
        if start is None:
            self.get_logger().warn('No position from FluidNC yet, trajectory dropped')
            return
        prev_t = 0.0
        for point in msg.points:
            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            target = list(start)
            self._apply(target, msg.joint_names, point.positions)
            feed = compute_feed(start, target, t - prev_t, self._max_feed)
            self._enqueue(build_move(self._axes, target, feed))
            start, prev_t = target, t
        self._last_cmd_deg = start

    # ---------------------------------------------------------------- jog

    def _on_jog(self, msg):
        vel = [0.0] * len(self._joints)
        for name, value in zip(msg.name, msg.velocity):
            idx = self._index.get(name)
            if idx is None:
                self.get_logger().warn(f'Unknown joint "{name}" ignored', throttle_duration_sec=5.0)
                continue
            deg = math.degrees(value)
            vel[idx] = max(-self._max_jog_speed, min(self._max_jog_speed, deg))
        with self._jog_lock:
            now = time.monotonic()
            gap = now - self._jog_msg_time
            # Refreshed for every message, refused ones included, so the release/pause rule
            # below measures the real jog stream.
            self._jog_msg_time = now
            refused = None
            if self._trajectory_active:
                refused = 'a trajectory goal is active'
            elif self._homing:
                refused = 'homing'
            if refused is not None:
                self._jog_vel = [0.0] * len(self._joints)
                self._jog_stamp = 0.0
                if any(vel):
                    # A stick/button held while jog is refused must be released (or paused)
                    # once jogging is allowed again; it never starts motion by itself.
                    self._jog_inhibit = True
                    self.get_logger().warn(f'jog ignored while {refused}',
                                           throttle_duration_sec=2.0)
                return
            if self._jog_inhibit:
                # After STOP: require a release (zero velocity) or a pause in the jog stream, so a
                # stick/button still held during the STOP never restarts motion by itself.
                if self._stops_pending or (any(vel) and gap < self._jog_timeout):
                    if any(vel):
                        self.get_logger().warn(
                            'jog ignored after STOP: release the jog first',
                            throttle_duration_sec=2.0)
                    return
                self._jog_inhibit = False
            self._jog_vel = vel
            self._jog_stamp = now

    def _jog_tick(self):
        with self._arb_lock:
            self._jog_tick_locked()

    def _jog_tick_locked(self):
        # Velocity jog: stream short $J segments, keeping only jog_lookahead seconds queued
        # so a stop (jog cancel) or a speed change takes effect quickly.
        if self._stops_pending or self._homing or self._trajectory_active:
            return
        now = time.monotonic()
        with self._jog_lock:
            vel = list(self._jog_vel)
            fresh = now - self._jog_stamp < self._jog_timeout
        if not fresh or not any(vel):
            if self._jogging:
                self._cancel_jog()
            return
        if self._state not in ('Idle', 'Jog'):
            self.get_logger().warn(f'Jog ignored in state {self._state}', throttle_duration_sec=2.0)
            return
        if not self._jogging:
            self._jogging = True
            self._last_cmd_deg = None
            self._jog_ahead_until = now
        if not self._queue.empty() or self._jog_ahead_until - now > self._jog_lookahead:
            return
        if self._planner_free is not None and self._planner_free < 8:
            return
        deltas, feed = jog_segment(vel, self._jog_period, self._max_feed)
        line = build_jog(self._axes, deltas, feed)
        if line:
            # Short ack timeout: a jog line can be silently dropped by a jog cancel.
            self._enqueue(line, timeout=1.0)
            self._jog_ahead_until = max(self._jog_ahead_until, now) + self._jog_period

    def _cancel_jog(self):
        self._jogging = False
        self._clear_queue()
        self._raw_write(JOG_CANCEL)
        # FluidNC discards a $J line still in its input buffer on jog cancel and never acks it;
        # release the writer so it doesn't block waiting for that ok.
        inflight = self._inflight
        if inflight is not None and inflight.startswith('$J'):
            self._ack_ok = True
            self._ack.set()
        self._last_cmd_deg = None

    # ------------------------------------------------------------- homing

    def _report(self, text, error=False):
        # rclpy forbids one call site logging at different severities, so keep two calls.
        if error:
            self.get_logger().error(f'Homing: {text}')
        else:
            self.get_logger().info(f'Homing: {text}')
        self._homing_pub.publish(String(data=text))

    def _on_home_joint(self, joint, _req, res):
        reason = None
        if self._stops_pending:
            reason = 'STOP still in progress'
        elif self._homing:
            reason = 'homing already running'
        elif self._trajectory_active:
            reason = 'a trajectory goal is active'
        elif self._jogging:
            reason = 'stop jogging first'
        elif self._state != 'Idle':
            reason = f'FluidNC must be Idle (now {self._state}); unlock or stop first'
        else:
            status = self._wait_status()
            if status is None or status['mpos'] is None:
                reason = 'no status from FluidNC'
            elif self._homing_pins[joint] in status['pins']:
                reason = 'endstop is pressed: jog the joint off the switch first'
        if reason:
            res.success, res.message = False, reason
            return res
        with self._arb_lock:
            if self._stops_pending:
                res.success, res.message = False, 'STOP still in progress'
                return res
            if self._trajectory_active or self._homing:
                res.success, res.message = False, 'a trajectory goal or homing is active'
                return res
            self._homing = True
            # Homing lines carry this generation, so a STOP makes all later ones stale.
            self._homing_generation = self._generation
        self._homing_abort.clear()
        threading.Thread(target=self._home_joint, args=(joint,), daemon=True).start()
        res.success, res.message = True, f'Homing {joint} started'
        return res

    def _home_joint(self, joint):
        # One switch, hit from both ends of the joint's travel:
        # find the left hit, then the right hit, and call the middle 0 deg.
        i, pin = self._index[joint], self._homing_pins[joint]
        try:
            self._report(f'{joint}: seeking left endstop')
            self._jog_until(i, pin, -1, self._seek_speed, pressed=True)
            left = self._locate(i, pin, -1)
            self._report(f'{joint}: left endstop at {left:.2f}, seeking right endstop')
            self._jog_until(i, pin, +1, self._seek_speed, pressed=False)
            self._jog_until(i, pin, +1, self._seek_speed, pressed=True)
            right = self._locate(i, pin, +1)
            self._move_rel(i, -self._backoff, self._seek_speed)
            span = right - left
            if span <= self._backoff:
                raise HomingError(f'travel between endstop hits is only {span:.2f} deg')
            center = (left + right) / 2.0
            status = self._wait_idle()
            here = status['mpos'][i] - center
            if not self._send_wait(f'G10 L20 P1 {self._axes[i]}{here:.4f}',
                                   generation=self._homing_generation):
                raise HomingError('FluidNC rejected the new work position')
            self._last_cmd_deg = None
            self._report(f'{joint}: homed, travel {span:.2f} deg (+/-{span / 2:.2f}), '
                         f'now at {here:.2f} deg')
        except HomingError as exc:
            self._raw_write(JOG_CANCEL)
            self._report(f'{joint}: homing failed: {exc}', error=True)
        finally:
            self._homing = False

    def _locate(self, i, pin, direction):
        """Back off the switch, then approach it again slowly; return the precise trigger MPos."""
        self._move_rel(i, -direction * self._backoff, self._seek_speed)
        if self._pin_pressed(pin):
            self._jog_until(i, pin, -direction, self._locate_speed, pressed=False)
        return self._jog_until(i, pin, direction, self._locate_speed, pressed=True)

    def _check_homing(self):
        if self._homing_abort.is_set():
            raise HomingError('aborted')
        if not self._running:
            raise HomingError('connection lost')

    def _wait_status(self, timeout=0.3):
        """Request a fresh status report and return it (None on timeout)."""
        with self._status_cond:
            seq = self._status_seq
        self._raw_write(STATUS_QUERY)
        with self._status_cond:
            if not self._status_cond.wait_for(lambda: self._status_seq > seq, timeout):
                return None
            return self._last_status

    def _pin_pressed(self, pin):
        status = self._wait_status()
        if status is None:
            raise HomingError('no status from FluidNC')
        return pin in status['pins']

    def _start_jog(self, i, delta, speed):
        line = build_jog([self._axes[i]], [delta], speed * 60.0)
        if not self._send_wait(line, generation=self._homing_generation):
            raise HomingError(f'FluidNC rejected "{line}"')

    def _watch(self, timeout):
        """Yield fresh status reports until timeout, raising on abort/alarm."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._check_homing()
            status = self._wait_status()
            if status is None:
                continue
            if status['state'].startswith('Alarm'):
                raise HomingError('FluidNC alarm')
            yield status
        raise HomingError('timeout')

    def _jog_until(self, i, pin, direction, speed, pressed):
        """Jog one axis until the switch reads `pressed`; return the MPos where it changed."""
        self._start_jog(i, direction * self._max_travel, speed)
        moving = False
        for status in self._watch(self._max_travel / speed + 5.0):
            if (pin in status['pins']) == pressed and status['mpos'] is not None:
                self._raw_write(JOG_CANCEL)
                self._wait_idle()
                return status['mpos'][i]
            if status['state'] == 'Jog':
                moving = True
            elif status['state'] == 'Idle' and moving:
                raise HomingError(f'endstop not found within {self._max_travel:.0f} deg')

    def _move_rel(self, i, delta, speed):
        self._start_jog(i, delta, speed)
        start = time.monotonic()
        for status in self._watch(abs(delta) / speed + 5.0):
            if status['state'] == 'Idle' and time.monotonic() - start > 0.2:
                return

    def _wait_idle(self):
        for status in self._watch(10.0):
            if status['state'] == 'Idle':
                return status

    # ----------------------------------------------------------- services

    def _on_home(self, _req, res):
        with self._arb_lock:
            if self._stops_pending:
                res.success, res.message = False, 'STOP still in progress'
                return res
            if self._homing or self._trajectory_active:
                res.success, res.message = False, 'endstop homing or a trajectory goal is active'
                return res
            self._last_cmd_deg = None
            self._enqueue('$H', self._homing_timeout)
        res.success, res.message = True, 'Homing queued'
        return res

    def _on_unlock(self, _req, res):
        # Unlock is the deliberate operator action that re-arms trajectory execution. Arm only
        # once FluidNC acknowledged $X and a status report requested afterwards is not Alarm.
        with self._arb_lock:
            if self._trajectory_active:
                res.success, res.message = False, 'a trajectory goal is active'
                return res
            if self._stops_pending:
                res.success, res.message = False, 'STOP still in progress; try again when stopped'
                return res
            self._armed = False
            self._last_cmd_deg = None
            generation = self._generation
            pending = _Pending()
            self._enqueue('$X', UNLOCK_ACK_TIMEOUT, pending, generation)
        error = None
        state = None
        if not pending.event.wait(UNLOCK_ACK_TIMEOUT + 1.0) or not pending.ok:
            error = 'FluidNC did not acknowledge $X with ok (error, timeout or busy queue)'
        else:
            status = self._wait_status(max(self._status_timeout, 0.3))
            if status is None:
                error = 'no FluidNC status report after $X'
            else:
                state = status['state']
                if state.startswith('Alarm'):
                    error = f'FluidNC still in {state} after $X'
        with self._arb_lock:
            if error is None and generation != self._generation:
                error = 'interrupted by STOP'
            if error is None:
                self._armed = True
        if error is not None:
            self._disarm(f'unlock failed: {error}')
            res.success, res.message = False, f'Unlock failed: {error}; execution stays disarmed'
            return res
        self._traj_status(f'armed: re-armed by ~/unlock (FluidNC {state})')
        res.success, res.message = True, (f'Unlocked ($X ok, FluidNC {state}); '
                                          'trajectory execution armed')
        return res

    def _on_stop(self, _req, res):
        self._halt('STOP requested')
        res.success, res.message = True, ('Feed hold sent, reset follows when stopped; '
                                          'trajectory execution disarmed until unlock')
        return res

    def _halt(self, reason):
        """Feed hold, flush queued lines, soft reset once stopped; disarm trajectory execution.

        Returns an Event set when the reset has been sent.
        """
        # One atomic step under _arb_lock (also used by unlock's final arming check and every
        # motion source): new generation, STOP pending, disarmed, jog dropped and inhibited.
        with self._arb_lock:
            with self._write_lock:
                self._generation += 1
                self._stops_pending += 1
            self._armed = False
            self._homing_abort.set()
            self._jogging = False
            with self._jog_lock:
                self._jog_vel = [0.0] * len(self._joints)
                self._jog_stamp = 0.0
                self._jog_inhibit = True
        self._clear_queue()
        done = threading.Event()
        # Only status reports newer than this may confirm the stop (cached state can predate
        # motion that the hold interrupts).
        with self._status_cond:
            hold_seq = self._status_seq
        try:
            self._raw_write(JOG_CANCEL)
            self._raw_write(FEED_HOLD)
            self._raw_write(STATUS_QUERY)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'STOP could not be sent: {exc}')
            self._end_stop(reset_sent=False)
            self._traj_status(self._disarm_text(reason))
            done.set()
            return done
        self._traj_status(self._disarm_text(reason))
        threading.Thread(target=self._finish_stop, args=(hold_seq, done), daemon=True).start()
        return done

    def _end_stop(self, reset_sent):
        with self._arb_lock:
            with self._write_lock:
                self._stops_pending -= 1
            if reset_sent and not self._stops_pending:
                # Queued while still holding _arb_lock: no motion line can get ahead of it.
                self._enqueue('G90')

    def _wait_stopped(self, hold_seq, timeout):
        """Wait for FluidNC to report a stop in status reports received after the feed hold.

        Accepts Alarm (motion already aborted) or STOP_CONFIRM_REPORTS consecutive Idle/Hold:0
        reports. Returns False on timeout or lost connection.
        """
        deadline = time.monotonic() + timeout
        next_query = time.monotonic() + STOP_POLL_PERIOD
        seen = hold_seq
        stopped = 0
        while self._running:
            now = time.monotonic()
            if now >= deadline:
                return False
            if now >= next_query:
                next_query = now + STOP_POLL_PERIOD
                try:
                    self._raw_write(STATUS_QUERY)
                except Exception:  # noqa: BLE001
                    pass
            with self._status_cond:
                self._status_cond.wait_for(lambda: self._status_seq != seen, 0.05)
                seq, status = self._status_seq, self._last_status
            if seq == seen:
                continue
            seen = seq
            state = status['state']
            if state.startswith('Alarm'):
                return True
            stopped = stopped + 1 if state in ('Hold:0', 'Idle') else 0
            if stopped >= STOP_CONFIRM_REPORTS:
                return True
        return False

    def _finish_stop(self, hold_seq, done=None):
        # Reset only after decelerating to Hold:0/Idle, otherwise FluidNC aborts the cycle
        # (Alarm, position lost).
        reset_sent = False
        try:
            if not self._wait_stopped(hold_seq, STOP_CONFIRM_TIMEOUT):
                self.get_logger().error(
                    f'Stop not confirmed by FluidNC within {STOP_CONFIRM_TIMEOUT:.0f} s; '
                    'sending soft reset anyway (may abort motion: Alarm, position lost)')
                self._traj_status(
                    'disarmed: stop not confirmed, soft reset sent anyway; position may be lost - '
                    'verify or re-home before Unlock / re-arm')
            self._raw_write(SOFT_RESET)
            reset_sent = True
            self._ack_ok = False
            self._ack.set()
            self._last_cmd_deg = None
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Soft reset could not be sent: {exc}')
        finally:
            self._end_stop(reset_sent)
            if done is not None:
                done.set()

    # ------------------------------------------------- trajectory action

    def _traj_status(self, text):
        self.get_logger().info(f'Trajectory: {text}')
        self._traj_status_pub.publish(String(data=text))

    @staticmethod
    def _disarm_text(reason):
        return f'disarmed: {reason}; check the arm, then Unlock / re-arm (~/unlock)'

    def _disarm(self, reason):
        self._armed = False
        self._traj_status(self._disarm_text(reason))

    def _jog_requested(self):
        with self._jog_lock:
            return time.monotonic() - self._jog_stamp < self._jog_timeout and any(self._jog_vel)

    def _trajectory_blocker(self):
        if not self._running:
            return 'FluidNC connection lost'
        if self._limits is None:
            return 'robot_description not set (joint limits unknown)'
        if self._stops_pending:
            return 'STOP is still finishing'
        if not self._armed:
            return 'execution disarmed after STOP or a fault; Unlock / re-arm (~/unlock) first'
        if self._homing:
            return 'homing in progress'
        if self._jogging or self._jog_requested():
            return 'jog active'
        if not self._queue.empty() or self._inflight is not None:
            return 'other commands are still being sent'
        return None

    def _on_trajectory_goal(self, _goal_request):
        with self._arb_lock:
            if self._stops_pending:
                self.get_logger().warn('Trajectory goal rejected: STOP is still finishing')
                return GoalResponse.REJECT
            if self._trajectory_active:
                self.get_logger().warn('Trajectory goal rejected: another goal is active')
                return GoalResponse.REJECT
            self._trajectory_active = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _on_trajectory_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _execute_trajectory(self, goal_handle):
        result = FollowJointTrajectory.Result()
        try:
            try:
                outcome, code, message = self._run_trajectory(goal_handle)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'Trajectory execution failed: {exc!r}')
                outcome, code, message = self._fault(f'internal error: {exc}')
            result.error_code = getattr(FollowJointTrajectory.Result, code)
            result.error_string = message
            if outcome == 'succeed':
                goal_handle.succeed()
            elif outcome == 'cancel':
                goal_handle.canceled()
            else:
                goal_handle.abort()
            note = '' if self._armed else '; execution disarmed until Unlock / re-arm'
            self._traj_status(f'{outcome}: {message}{note}')
            return result
        finally:
            self._last_cmd_deg = None
            with self._arb_lock:
                self._trajectory_active = False

    def _fault(self, reason, code='PATH_TOLERANCE_VIOLATED'):
        self._halt(reason).wait(6.0)
        return 'abort', code, reason

    def _check_goal_extras(self, goal):
        multi = goal.multi_dof_trajectory
        if multi.joint_names or multi.points:
            raise TrajectoryError('INVALID_GOAL', 'multi_dof_trajectory is not supported')
        if goal.path_tolerance:
            raise TrajectoryError(
                'INVALID_GOAL', 'path_tolerance is not supported (FluidNC timing is approximate)')
        for field in ('component_path_tolerance', 'component_goal_tolerance'):
            if getattr(goal, field, None):
                raise TrajectoryError('INVALID_GOAL', f'{field} is not supported')
        stamp = goal.trajectory.header.stamp
        if stamp.sec or stamp.nanosec:
            delay = (Time.from_msg(stamp) - self.get_clock().now()).nanoseconds * 1e-9
            if delay > MAX_START_DELAY:
                raise TrajectoryError(
                    'INVALID_GOAL', 'delayed start (future header.stamp) is not supported')

    def _run_trajectory(self, goal_handle):
        goal = goal_handle.request
        generation = self._generation
        reason = self._trajectory_blocker()
        if reason is None:
            status = self._wait_status(self._status_timeout)
            if status is None:
                reason = 'no fresh FluidNC status'
            elif status['state'] != 'Idle':
                reason = f"FluidNC must be Idle (now {status['state']})"
            elif status['wpos'] is None:
                reason = 'no position from FluidNC'
        if reason is not None:
            return 'abort', 'INVALID_GOAL', f'Not ready: {reason}'

        traj = goal.trajectory
        try:
            self._check_goal_extras(goal)
            tolerances = goal_tolerances(
                [{'name': t.name, 'position': t.position, 'velocity': t.velocity,
                  'acceleration': t.acceleration} for t in goal.goal_tolerance],
                self._joints, self._goal_tolerance)
            points = [{
                'time': p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
                'positions': list(p.positions),
                'velocities': list(p.velocities),
                'accelerations': list(p.accelerations),
                'effort': list(p.effort),
            } for p in traj.points]
            current = [math.radians(v) for v in status['wpos']]
            plan = plan_trajectory(traj.joint_names, points, self._joints, self._axes,
                                   self._limits, current, self._start_tolerance, self._max_feed)
            gtt = goal.goal_time_tolerance.sec + goal.goal_time_tolerance.nanosec * 1e-9
            allowed = allowed_duration(plan.duration, gtt, self._time_scaling, self._time_margin)
        except TrajectoryError as exc:
            return 'abort', exc.code, str(exc)

        self._traj_status(
            f'executing: {len(plan.lines)} segments, {plan.duration:.2f} s (allowed {allowed:.1f} s)')
        pendings = []
        # Lines carry the generation read before validation: a STOP in between drops them all.
        for line in [TRAJECTORY_MODES] + plan.lines:
            pending = _Pending()
            self._enqueue(line, pending=pending, generation=generation)
            pendings.append(pending)

        start = time.monotonic()
        deadline = start + allowed
        seen = None
        final_seq = None
        while True:
            if goal_handle.is_cancel_requested:
                self._halt('trajectory goal canceled').wait(6.0)
                return 'cancel', 'SUCCESSFUL', 'Trajectory canceled; motion halted'
            if self._generation != generation:
                return 'abort', 'PATH_TOLERANCE_VIOLATED', 'Trajectory interrupted by STOP'
            if not self._running:
                return self._fault('FluidNC connection lost')
            if any(p.event.is_set() and not p.ok for p in pendings):
                return self._fault('FluidNC rejected or did not acknowledge a trajectory line')
            now = time.monotonic()
            if self._last_status_time is None or \
                    now - self._last_status_time > self._status_timeout:
                return self._fault('FluidNC status reports are stale')
            if now > deadline:
                return self._fault(f'trajectory did not finish within {allowed:.1f} s',
                                   'GOAL_TOLERANCE_VIOLATED')
            with self._status_cond:
                self._status_cond.wait_for(lambda: self._status_seq != seen, 0.05)
                seq, status = self._status_seq, self._last_status
            if seq == seen:
                continue
            seen = seq
            if self._generation != generation:
                return 'abort', 'PATH_TOLERANCE_VIOLATED', 'Trajectory interrupted by STOP'
            state = status['state']
            if state not in ('Idle', 'Run'):
                return self._fault(f'FluidNC entered {state} during the trajectory')
            if status['wpos'] is None:
                continue
            actual = [math.radians(v) for v in status['wpos']]
            self._publish_feedback(goal_handle, plan, time.monotonic() - start, actual)
            if final_seq is None:
                if all(p.event.is_set() for p in pendings):
                    final_seq = seq
                continue
            # Two reports after the last ok: Idle then means the planner really finished.
            if seq >= final_seq + 2 and state == 'Idle':
                off = within_tolerance(actual, plan.final, tolerances)
                if off is None:
                    return 'succeed', 'SUCCESSFUL', (
                        f'Trajectory completed: {len(plan.lines)} segments, FluidNC Idle '
                        'within goal tolerance')
                i, error = off
                reason = (f'{self._joints[i]} stopped {error:+.4f} rad from the final point '
                          f'(tolerance {tolerances[i]:.4f})')
                self._disarm(reason)
                return 'abort', 'GOAL_TOLERANCE_VIOLATED', reason

    def _publish_feedback(self, goal_handle, plan, elapsed, actual):
        desired, velocity = plan.sample(elapsed)
        fb = FollowJointTrajectory.Feedback()
        fb.header.stamp = self.get_clock().now().to_msg()
        fb.joint_names = list(self._joints)
        stamp = Duration(seconds=max(0.0, elapsed)).to_msg()
        fb.desired.positions = desired
        fb.desired.velocities = velocity
        fb.desired.time_from_start = stamp
        fb.actual.positions = actual
        fb.actual.time_from_start = stamp
        fb.error.positions = [d - a for d, a in zip(desired, actual)]
        fb.error.time_from_start = stamp
        goal_handle.publish_feedback(fb)

    def destroy_node(self):
        self._running = False
        try:
            self._trajectory_action.destroy()
            self._io.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = FluidNCBridge()
    except OSError as exc:
        rclpy.logging.get_logger('fluidnc_bridge').fatal(
            f'Cannot connect to FluidNC: {exc}. After unplugging the board, reattach it to WSL '
            '(Windows: usbipd attach --wsl --busid 2-1) and close other programs using the port.')
        rclpy.try_shutdown()
        raise SystemExit(1)
    # Action execution blocks its own thread; status, services and timers keep running.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
