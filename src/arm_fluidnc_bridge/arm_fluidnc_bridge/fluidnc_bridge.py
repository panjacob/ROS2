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
Services:
  ~/home          $H  (FluidNC homing cycle, needs homing configured in FluidNC)
  ~/home_<joint>  endstop homing of one joint (homing_* parameters)
  ~/unlock        $X  (clear alarm)
  ~/stop          feed hold + soft reset, clears queued moves, aborts homing
"""

import math
import queue
import socket
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

from arm_fluidnc_bridge.fluidnc_protocol import (
    build_jog, build_move, compute_feed, jog_segment, parse_status)

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

    def __init__(self):
        super().__init__('fluidnc_bridge')
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

        if param('transport') == 'tcp':
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
        self._homing = False
        self._homing_abort = threading.Event()

        self._js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._state_pub = self.create_publisher(String, '~/machine_state', 10)
        self._feed_pub = self.create_publisher(Float64, '~/feed_rate', 10)
        self._spindle_pub = self.create_publisher(Float64, '~/spindle_speed', 10)
        self._pins_pub = self.create_publisher(String, '~/limit_switches', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._homing_pub = self.create_publisher(String, '~/homing_status', latched)
        self.create_subscription(JointState, 'joint_command', self._on_joint_command, 10)
        self.create_subscription(JointTrajectory, 'joint_trajectory', self._on_trajectory, 10)
        self.create_subscription(JointState, 'jog', self._on_jog, 10)
        self.create_service(Trigger, '~/home', self._on_home)
        self.create_service(Trigger, '~/unlock', self._on_unlock)
        self.create_service(Trigger, '~/stop', self._on_stop)
        for joint in self._homing_pins:
            self.create_service(
                Trigger, f'~/home_{joint}',
                lambda req, res, j=joint: self._on_home_joint(j, req, res))
        self.create_timer(1.0 / float(param('status_rate')), self._request_status)
        self.create_timer(self._jog_period, self._jog_tick)

        threading.Thread(target=self._read_loop, daemon=True).start()
        threading.Thread(target=self._write_loop, daemon=True).start()
        self._enqueue('G90')
        self._report('not homed' if self._homing_pins else 'no endstops configured')

    # ---------------------------------------------------------------- I/O

    def _raw_write(self, data):
        with self._write_lock:
            self._io.write(data)

    def _enqueue(self, line, timeout=None, pending=None):
        self._queue.put((line, timeout or self._ack_timeout, pending))

    def _send_wait(self, line, timeout=None):
        """Queue a line and block until FluidNC answers ok (True) or error/timeout (False)."""
        pending = _Pending()
        self._enqueue(line, timeout, pending)
        pending.event.wait((timeout or self._ack_timeout) + 1.0)
        return pending.ok

    def _clear_queue(self):
        while True:
            try:
                _line, _timeout, pending = self._queue.get_nowait()
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
                line, timeout, pending = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            self._ack.clear()
            self._inflight = line
            self._raw_write((line + '\n').encode('ascii'))
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
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self._joints
            msg.position = [math.radians(v) for v in wpos]
            self._js_pub.publish(msg)

        with self._status_cond:
            self._last_status = status
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
        if self._homing:
            self.get_logger().warn(f'{what} ignored while homing', throttle_duration_sec=2.0)
            return True
        return False

    def _on_joint_command(self, msg):
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
            self._jog_vel = vel
            self._jog_stamp = time.monotonic()

    def _jog_tick(self):
        # Velocity jog: stream short $J segments, keeping only jog_lookahead seconds queued
        # so a stop (jog cancel) or a speed change takes effect quickly.
        if self._homing:
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
        if self._homing:
            reason = 'homing already running'
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
        self._homing = True
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
            if not self._send_wait(f'G10 L20 P1 {self._axes[i]}{here:.4f}'):
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
        if not self._send_wait(line):
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
        if self._homing:
            res.success, res.message = False, 'endstop homing running'
            return res
        self._last_cmd_deg = None
        self._enqueue('$H', self._homing_timeout)
        res.success, res.message = True, 'Homing queued'
        return res

    def _on_unlock(self, _req, res):
        self._last_cmd_deg = None
        self._enqueue('$X')
        res.success, res.message = True, 'Unlock queued'
        return res

    def _on_stop(self, _req, res):
        self._homing_abort.set()
        with self._jog_lock:
            self._jog_stamp = 0.0
        self._jogging = False
        self._clear_queue()
        self._raw_write(JOG_CANCEL)
        self._raw_write(FEED_HOLD)
        threading.Thread(target=self._finish_stop, daemon=True).start()
        res.success, res.message = True, 'Feed hold sent, reset follows when stopped'
        return res

    def _finish_stop(self):
        # Reset only after decelerating to Hold:0, otherwise FluidNC loses position (alarm).
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and self._state not in ('Hold:0', 'Idle'):
            time.sleep(0.05)
        self._raw_write(SOFT_RESET)
        self._ack_ok = False
        self._ack.set()
        self._last_cmd_deg = None
        self._enqueue('G90')

    def destroy_node(self):
        self._running = False
        try:
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
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
