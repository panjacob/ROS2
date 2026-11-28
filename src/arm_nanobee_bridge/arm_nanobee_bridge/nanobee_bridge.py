"""ROS 2 <-> NanoBee bridge (MKS TinyBee running NanoBee/, USB serial).

NanoBee works in motor steps; ROS uses radians. Steps per degree, speed/acceleration limits,
homing modes and soft limits are read from the board at startup ("SETTINGS?"), so the board's
config.yaml is the single source of truth (edit it on the NanoBee web page or with
NanoBeeDriver, then call ~/reload_settings or restart the bridge; or upload config_path with
~/push_config, which also reloads them).

Topics:
  pub  joint_states        sensor_msgs/JointState  (from status reports, status_rate Hz)
  pub  ~/machine_state     std_msgs/String         Idle | Run | Jog | Home (+ " motors off")
  pub  ~/limit_switches    std_msgs/String         comma-separated joints whose endstop is pressed
  pub  ~/motors_enabled    std_msgs/Bool           motor drivers enabled (board E 1)
  pub  ~/motors_powered    std_msgs/String         comma-separated joints whose driver is powered
                                                   (an enabled joint may be idle-disabled)
  pub  ~/homing_status     std_msgs/String         (latched) homing progress/result
  pub  ~/homed             std_msgs/String         comma-separated referenced joints (homed or
                                                   zeroed; board H:)
  pub  ~/joint_settings    std_msgs/String         (latched) JSON per joint from the board settings:
                                                   {"joint1": {"homing": "center",
                                                   "max_speed_deg_s": 45.0}, ...}; homing is
                                                   "none" while homing.enabled is false
  pub  ~/trajectory_status std_msgs/String         (latched) armed/executing/result/disarmed
  pub  ~/armed             std_msgs/Bool           (latched) false after STOP or a fault: all
                                                   motion is refused until ~/unlock
  sub  joint_command       sensor_msgs/JointState  absolute positions (firmware M); optional
                                                   velocity is a positive per-joint limit [rad/s]
  sub  joint_trajectory    trajectory_msgs/JointTrajectory  timed points (firmware T segments)
  sub  jog                 sensor_msgs/JointState  velocity [rad/s] per joint; keep publishing,
                                                   motion stops after jog_timeout without messages
Actions:
  /arm_controller/follow_joint_trajectory  control_msgs/FollowJointTrajectory (MoveIt)
Services:
  ~/home             home every joint with joints.jointN.homing != none (firmware H)
  ~/home_<joint>     home one joint (refused with a reason if the board has no homing for it)
  ~/stop             decelerate to a stop, drop queued motion, abort homing and trajectory goals;
                     the bridge stays disarmed until ~/unlock: jog, joint_command,
                     joint_trajectory, homing and trajectory goals are all refused
  ~/unlock           enable the motors and re-arm all motion (START)
  ~/motors           std_srvs/SetBool: enable / disable the motor drivers (disable = free to move
                     by hand; the board then forgets its home reference)
  ~/set_zero         joints that are not homed: current position becomes 0
  ~/set_zero_<joint> manual homing: the joint's current position becomes 0 (homed or not, any
                     position); other joints keep their positions (Idle only)
  ~/reload_settings  read the board settings again
  ~/push_config      upload the file at parameter config_path (NanoBeeDriver/config.yaml) to the
                     board (Idle only), then reload the settings
joint_command, joint_trajectory, jog and homing are refused while a trajectory goal is active;
a trajectory goal is refused while any of them is active. After ~/unlock, jog stays inhibited until
a zero-velocity jog message (or a pause of at least jog_timeout): a held stick never resumes motion.
With enable_on_start (default true) the bridge enables the motor drivers on connect, so a restart
(or a board reset when the serial port opens) leaves the arm held instead of limp.
"""

import json
import math
import os
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory

from arm_sim.motion import joints_from_urdf

from arm_nanobee_bridge.nanobee_protocol import (
    NUM_JOINTS, ProtocolError, config_upload, format_steps, jog_line, move_line, parse_settings,
    parse_status, parse_version, segment_line)
from arm_nanobee_bridge.trajectory import (
    TrajectoryError, allowed_duration, goal_tolerances, plan_trajectory, stop_ramp,
    within_tolerance)

# Largest accepted header.stamp in the future; delayed starts are not supported.
MAX_START_DELAY = 0.5
# Pause before re-sending a T segment the board refused with "err full".
QUEUE_FULL_RETRY = 0.02
# The board parses an uploaded config and writes it to flash before answering.
CONFIG_TIMEOUT = 5.0


class SerialTransport:

    def __init__(self, port, baud):
        import serial
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = port, baud, 0.1
        # Opening the port must not reset the ESP32. Linux raises DTR and RTS together on open;
        # the auto-reset circuit pulls EN low only when exactly one of them is asserted, so
        # leave both asserted (pyserial would drop DTR before RTS: a reset pulse).
        s.dtr = s.rts = True
        s.open()
        self._s = s

    def write(self, data):
        self._s.write(data)

    def read(self):
        return self._s.read(self._s.in_waiting or 1)

    def close(self):
        self._s.close()


class CommandError(Exception):
    pass


class NanoBeeBridge(Node):

    def __init__(self, transport=None, **kwargs):
        super().__init__('nanobee_bridge', **kwargs)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'])
        self.declare_parameter('status_rate', 20.0)
        self.declare_parameter('command_timeout', 1.0)
        self.declare_parameter('connect_timeout', 8.0)
        self.declare_parameter('move_speed', 0.0)
        self.declare_parameter('jog_rate', 20.0)
        self.declare_parameter('jog_timeout', 0.3)
        self.declare_parameter('max_jog_speed', 360.0)
        self.declare_parameter('robot_description', '')
        self.declare_parameter('status_timeout', 0.5)
        self.declare_parameter('trajectory_start_tolerance', 0.02)
        self.declare_parameter('trajectory_goal_tolerance', 0.01)
        self.declare_parameter('trajectory_time_scaling', 1.5)
        self.declare_parameter('trajectory_time_margin', 2.0)
        self.declare_parameter('config_path', '')
        self.declare_parameter('enable_on_start', True)

        def param(name):
            return self.get_parameter(name).value

        self._joints = list(param('joint_names'))
        if len(self._joints) != NUM_JOINTS:
            raise ValueError(f'joint_names needs {NUM_JOINTS} names (firmware joints J0..J4)')
        self._index = {name: i for i, name in enumerate(self._joints)}
        self._cmd_timeout = float(param('command_timeout'))
        self._move_speed = float(param('move_speed'))
        self._jog_period = 1.0 / float(param('jog_rate'))
        self._jog_timeout = float(param('jog_timeout'))
        self._max_jog_speed = float(param('max_jog_speed'))
        self._status_timeout = float(param('status_timeout'))
        self._start_tolerance = float(param('trajectory_start_tolerance'))
        self._goal_tolerance = float(param('trajectory_goal_tolerance'))
        self._time_scaling = float(param('trajectory_time_scaling'))
        self._time_margin = float(param('trajectory_time_margin'))
        self._config_path = os.path.expanduser(str(param('config_path') or ''))
        self._enable_on_start = bool(param('enable_on_start'))
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

        # I/O state
        self._write_lock = threading.Lock()
        self._cmd_lock = threading.Lock()
        self._reply_cond = threading.Condition()
        self._waiting = False
        self._block = None      # lines of a multi-line reply (SETTINGS?) while collecting
        self._reply = None
        self._running = True
        self._status_cond = threading.Condition()
        self._status_seq = 0
        self._status = None
        self._status_time = None
        self._state = 'Unknown'
        self._settings = None
        self._underruns = 0

        # arbitration state
        self._arb_lock = threading.Lock()
        self._generation = 0
        self._trajectory_active = False
        self._streaming = False
        self._armed = True
        self._last_cmd_steps = None
        self._homing_requested = False
        self._jog_lock = threading.Lock()
        self._jog_vel = [0.0] * NUM_JOINTS
        self._jog_stamp = 0.0
        self._jogging = False
        self._jog_inhibit = False
        self._jog_msg_time = 0.0

        if transport is not None:
            self._io = transport
        else:
            self._io = SerialTransport(param('serial_port'), int(param('baud')))
            self.get_logger().info(f"Opened {param('serial_port')} @ {param('baud')}")
        threading.Thread(target=self._read_loop, daemon=True).start()
        try:
            self._connect(float(param('connect_timeout')))
            if self._enable_on_start:
                self._enable_motors()
        except Exception:
            self._running = False
            self._io.close()
            raise

        self._js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._state_pub = self.create_publisher(String, '~/machine_state', 10)
        self._pins_pub = self.create_publisher(String, '~/limit_switches', 10)
        self._enabled_pub = self.create_publisher(Bool, '~/motors_enabled', 10)
        self._powered_pub = self.create_publisher(String, '~/motors_powered', 10)
        self._homed_pub = self.create_publisher(String, '~/homed', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._joint_settings_pub = self.create_publisher(String, '~/joint_settings', latched)
        self._publish_joint_settings()
        self._homing_pub = self.create_publisher(String, '~/homing_status', latched)
        self._traj_status_pub = self.create_publisher(String, '~/trajectory_status', latched)
        self._armed_pub = self.create_publisher(Bool, '~/armed', latched)
        self._set_armed(self._armed)
        self.create_subscription(JointState, 'joint_command', self._on_joint_command, 10)
        self.create_subscription(JointTrajectory, 'joint_trajectory', self._on_trajectory, 10)
        self.create_subscription(JointState, 'jog', self._on_jog, 10)
        self.create_service(Trigger, '~/home', lambda req, res: self._on_home(None, res))
        # One service per joint; _on_home checks the board's current homing mode, so a pushed
        # config takes effect without restarting the bridge.
        for j in range(len(self._joints)):
            self.create_service(Trigger, f'~/home_{self._joints[j]}',
                                lambda req, res, j=j: self._on_home(j, res))
            self.create_service(Trigger, f'~/set_zero_{self._joints[j]}',
                                lambda req, res, j=j: self._on_set_zero_joint(j, res))
        self.create_service(Trigger, '~/stop', self._on_stop)
        self.create_service(Trigger, '~/unlock', self._on_unlock)
        self.create_service(SetBool, '~/motors', self._on_motors)
        self.create_service(Trigger, '~/set_zero', self._on_set_zero)
        self.create_service(Trigger, '~/reload_settings', self._on_reload_settings)
        self.create_service(Trigger, '~/push_config', self._on_push_config)
        self.create_timer(1.0 / float(param('status_rate')), self._request_status)
        self.create_timer(self._jog_period, self._jog_tick)
        self._trajectory_action = ActionServer(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_trajectory,
            goal_callback=self._on_trajectory_goal,
            cancel_callback=self._on_trajectory_cancel,
            callback_group=ReentrantCallbackGroup())

        homed = [self._joints[j] for j, m in enumerate(self._settings.homing) if m != 'none']
        if not homed:
            self._report('no homing configured on the board (joints.jointN.homing)')
        elif not self._settings.homing_enabled:
            self._report('homing disabled on the board (homing.enabled: false)')
        else:
            self._report(f"not homed ({', '.join(homed)} can be homed)")
        self._traj_status('armed: waiting for a trajectory goal')

    # ---------------------------------------------------------------- I/O

    def _connect(self, timeout):
        # The ESP32 may still be booting: poll '?' until a status report arrives.
        deadline = time.monotonic() + timeout
        while self._wait_status(0.5) is None:
            if time.monotonic() > deadline or not self._running:
                raise ConnectionError(
                    f'no status report from the board within {timeout:.0f} s; is the NanoBee '
                    'firmware flashed and the right serial port selected?')
        version = self._command('VER')
        parse_version(version)
        self._load_settings()
        self.get_logger().info(f'Connected: {version}')

    def _enable_motors(self):
        status = self._wait_status(self._status_timeout)
        if status is not None and status['enabled']:
            return
        reply = self._command('E 1')
        if reply != 'ok':
            raise CommandError(f'enabling the motors failed: {reply}')
        self.get_logger().info('Motors enabled on start (enable_on_start)')

    def _load_settings(self):
        settings = parse_settings(self._command('SETTINGS?', block=True))
        self._settings = settings
        spd = ', '.join(f'{v:g}' for v in settings.steps_per_deg)
        self.get_logger().info(
            f'Board settings: steps_per_deg [{spd}], homing {settings.homing}, '
            f"soft limits {'on' if settings.soft_limits else 'off'}, "
            f"blend {'on' if settings.blend else 'off'}")
        if not settings.blend:
            self.get_logger().warn(
                'motion.blend is off on the board: trajectories stop at every point')
        if hasattr(self, '_joint_settings_pub'):
            self._publish_joint_settings()

    def _publish_joint_settings(self):
        s = self._settings
        data = {name: {'homing': s.homing[j] if s.homing_enabled else 'none',
                       'max_speed_deg_s': s.max_speed[j] / s.steps_per_deg[j]}
                for j, name in enumerate(self._joints)}
        self._joint_settings_pub.publish(String(data=json.dumps(data)))

    def _raw_write(self, data):
        with self._write_lock:
            self._io.write(data)

    def _command(self, line, timeout=None, block=False):
        """Send one command line and return the board's reply (ok / err ... / data)."""
        timeout = timeout or self._cmd_timeout
        with self._cmd_lock:
            with self._reply_cond:
                self._reply = None
                self._block = [] if block else None
                self._waiting = True
            try:
                self._raw_write((line + '\n').encode('ascii'))
                with self._reply_cond:
                    if not self._reply_cond.wait_for(lambda: self._reply is not None, timeout):
                        raise CommandError(f'no reply to "{line}" within {timeout:.1f} s')
                    return self._reply
            finally:
                with self._reply_cond:
                    self._waiting = False
                    self._block = None

    def _send(self, line, timeout=None):
        """Send a command; return (ok, reply) without raising."""
        try:
            reply = self._command(line, timeout)
        except Exception as exc:  # noqa: BLE001
            return False, str(exc)
        return reply == 'ok', reply

    def _read_loop(self):
        buf = b''
        while self._running:
            try:
                buf += self._io.read()
            except Exception as exc:  # noqa: BLE001
                if self._running:
                    self.get_logger().fatal(f'Connection lost: {exc}')
                self._running = False
                return
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                raw_line = raw.decode('ascii', errors='replace').rstrip()
                if raw_line.strip():
                    self._handle_line(raw_line)

    def _handle_line(self, raw_line):
        # raw_line keeps its indentation: the SETTINGS? YAML dump needs it.
        line = raw_line.strip()
        if line.startswith('<'):
            status = parse_status(line)
            if status is not None:
                self._on_status(status)
            return
        with self._reply_cond:
            if self._waiting and self._block is not None:
                if line == 'END':
                    self._reply = '\n'.join(self._block) + '\n'
                    self._reply_cond.notify_all()
                else:
                    self._block.append(raw_line)
                return
            if not line.startswith('#'):
                if self._waiting and self._reply is None:
                    self._reply = line
                    self._reply_cond.notify_all()
                else:
                    self.get_logger().warn(f'Unexpected line from board: {line}')
                return
        self._on_event(line.lstrip('#').strip())

    def _on_event(self, text):
        if text.startswith('homed '):
            self._report(f'{self._joint_label(text[6:])}: homed')
        elif text.startswith('err home '):
            joint, _, why = text[9:].partition(':')
            self._report(f'{self._joint_label(joint)}: homing failed: {why.strip()}', error=True)
        elif text.startswith('warn'):
            self._underruns += 1
            self.get_logger().warn(f'Board: {text}')
        elif text.startswith('err'):
            self.get_logger().error(f'Board: {text}')
        elif text.startswith('NanoBee ready'):
            self.get_logger().error('Board restarted: position and home reference are lost')
            with self._arb_lock:
                self._last_cmd_steps = None
            self._disarm('board restarted (position lost)')
            threading.Thread(target=self._reload_after_reboot, daemon=True).start()
        else:
            self.get_logger().info(f'Board: {text}')

    def _reload_after_reboot(self):
        time.sleep(0.5)
        try:
            self._load_settings()
            if self._enable_on_start:
                self._enable_motors()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Could not reload settings after restart: {exc}')

    def _joint_label(self, text):
        try:
            return self._joints[int(text.strip())]
        except (ValueError, IndexError):
            return f'joint {text.strip()}'

    def _request_status(self):
        if not self._running:
            return
        try:
            self._raw_write(b'?\n')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Status request failed: {exc}', throttle_duration_sec=5.0)

    def _wait_status(self, timeout=0.3):
        """Request a fresh status report and return it (None on timeout)."""
        with self._status_cond:
            seq = self._status_seq
        try:
            self._raw_write(b'?\n')
        except Exception:  # noqa: BLE001
            return None
        with self._status_cond:
            if not self._status_cond.wait_for(lambda: self._status_seq > seq, timeout):
                return None
            return self._status

    def _on_status(self, status):
        if len(status['steps']) < NUM_JOINTS:
            return
        state = status['state']
        if state != self._state:
            if self._state == 'Home' and self._homing_requested:
                self._homing_requested = False
            self.get_logger().info(f'State: {self._state} -> {state}')
            self._state = state
        settings = self._settings
        with self._status_cond:
            self._status = status
            self._status_time = time.monotonic()
            self._status_seq += 1
            self._status_cond.notify_all()
        if settings is None or not hasattr(self, '_homed_pub'):
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joints)
        msg.position = [settings.to_rad(j, s) for j, s in enumerate(status['steps'][:NUM_JOINTS])]
        self._js_pub.publish(msg)
        self._state_pub.publish(String(data=state if status['enabled'] else f'{state} motors off'))
        pressed = [self._joints[j] for j, on in enumerate(status['endstops'][:NUM_JOINTS]) if on]
        self._pins_pub.publish(String(data=','.join(pressed)))
        self._enabled_pub.publish(Bool(data=bool(status['enabled'])))
        powered = [self._joints[j] for j, on in enumerate(status['powered'][:NUM_JOINTS]) if on]
        self._powered_pub.publish(String(data=','.join(powered)))
        homed = [self._joints[j] for j, on in enumerate(status['referenced'][:NUM_JOINTS]) if on]
        self._homed_pub.publish(String(data=','.join(homed)))

    def _fresh_status(self):
        with self._status_cond:
            if self._status_time is None or \
                    time.monotonic() - self._status_time > self._status_timeout:
                return None
            return self._status

    # ----------------------------------------------------------- commands

    def _busy(self, what):
        reason = None
        if not self._armed:
            reason = 'stopped (press START / ~/unlock)'
        elif self._trajectory_active:
            reason = 'a trajectory goal is active'
        elif self._state == 'Home':
            reason = 'homing'
        if reason:
            self.get_logger().warn(f'{what} ignored while {reason}', throttle_duration_sec=2.0)
        return reason is not None

    def _start_steps(self):
        if self._last_cmd_steps is not None:
            return list(self._last_cmd_steps)
        status = self._fresh_status()
        return list(status['steps'][:NUM_JOINTS]) if status else None

    def _apply(self, target, names, positions_rad):
        for name, value in zip(names, positions_rad):
            j = self._index.get(name)
            if j is None:
                self.get_logger().warn(f'Unknown joint "{name}" ignored', throttle_duration_sec=5.0)
                continue
            target[j] = self._settings.to_steps(j, value)

    def _on_joint_command(self, msg):
        with self._arb_lock:
            if self._busy('joint_command'):
                return
            target = self._start_steps()
            if target is None:
                self.get_logger().warn('No position from the board yet, command dropped')
                return
            start = list(target)
            self._apply(target, msg.name, msg.position)
            speed = None
            if msg.velocity:
                if (len(msg.velocity) != len(msg.name) or len(msg.position) != len(msg.name)
                        or len(set(msg.name)) != len(msg.name)
                        or any(name not in self._index for name in msg.name)
                        or any(not math.isfinite(v) or v <= 0 for v in msg.velocity)):
                    self.get_logger().warn('joint_command requires one positive finite velocity per known joint')
                    return
                distances = [abs(end - begin) for begin, end in zip(start, target)]
                dmax = max(distances)
                if dmax:
                    speed = min(
                        min(math.degrees(v), self._max_jog_speed) * self._settings.steps_per_deg[j]
                        * dmax / distances[j]
                        for name, v in zip(msg.name, msg.velocity)
                        for j in [self._index[name]] if distances[j])
            elif self._move_speed > 0.0:
                speed = self._move_speed * max(self._settings.steps_per_deg)
            ok, reply = self._send(move_line(target, speed))
            if ok:
                self._last_cmd_steps = target
            else:
                self.get_logger().error(f'joint_command refused by the board: {reply}')

    def _on_trajectory(self, msg):
        with self._arb_lock:
            if self._busy('joint_trajectory'):
                return
            if self._streaming:
                self.get_logger().warn('joint_trajectory ignored: previous one still streaming')
                return
            start = self._start_steps()
            if start is None:
                self.get_logger().warn('No position from the board yet, trajectory dropped')
                return
            segments, prev_t, previous = [], 0.0, start
            for point in msg.points:
                t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                target = list(previous)
                self._apply(target, msg.joint_names, point.positions)
                if t <= prev_t:
                    self.get_logger().error('joint_trajectory: time_from_start must increase')
                    return
                segments.append((previous, target, t - prev_t))
                previous, prev_t = target, t
            if not segments:
                return
            lines = [segment_line(b, dt * 1000.0) for _a, b, dt in segments[:-1]]
            a, b, dt = segments[-1]
            ramp, _extra = stop_ramp(a, b, dt, self._settings) if self._settings.blend \
                else ([(b, dt * 1000.0)], 0.0)
            lines += [segment_line(s, ms) for s, ms in ramp]
            self._last_cmd_steps = previous
            self._streaming = True
            generation = self._generation
        threading.Thread(target=self._stream_topic, args=(lines, generation), daemon=True).start()

    def _stream_topic(self, lines, generation):
        try:
            for line in lines:
                ok, reply = self._send_segment(line, generation)
                if not ok:
                    if reply:
                        self.get_logger().error(f'joint_trajectory refused by the board: {reply}')
                    return
        finally:
            self._streaming = False

    def _send_segment(self, line, generation, deadline=None):
        """Send a T segment, retrying while the board queue is full. (ok, reply); reply is None
        when a STOP (new generation) or the deadline interrupted the stream."""
        while self._running and generation == self._generation:
            ok, reply = self._send(line)
            if ok:
                return True, reply
            if reply != 'err full':
                return False, reply
            if deadline is not None and time.monotonic() > deadline:
                return False, None
            time.sleep(QUEUE_FULL_RETRY)
        return False, None

    # ---------------------------------------------------------------- jog

    def _on_jog(self, msg):
        vel = [0.0] * NUM_JOINTS
        for name, value in zip(msg.name, msg.velocity):
            j = self._index.get(name)
            if j is None:
                self.get_logger().warn(f'Unknown joint "{name}" ignored', throttle_duration_sec=5.0)
                continue
            if math.isfinite(value):
                deg = math.degrees(value)
                vel[j] = max(-self._max_jog_speed, min(self._max_jog_speed, deg))
        with self._jog_lock:
            now = time.monotonic()
            gap = now - self._jog_msg_time
            self._jog_msg_time = now
            refused = None
            if not self._armed:
                refused = 'stopped (press START / ~/unlock)'
            elif self._trajectory_active:
                refused = 'a trajectory goal is active'
            elif self._state == 'Home':
                refused = 'homing'
            if refused is not None:
                self._jog_vel = [0.0] * NUM_JOINTS
                self._jog_stamp = 0.0
                if any(vel):
                    # A stick/button held while jog is refused must be released (or paused)
                    # once jogging is allowed again; it never starts motion by itself.
                    self._jog_inhibit = True
                    self.get_logger().warn(f'jog ignored while {refused}',
                                           throttle_duration_sec=2.0)
                return
            if self._jog_inhibit:
                if any(vel) and gap < self._jog_timeout:
                    self.get_logger().warn('jog ignored after STOP: release the jog first',
                                           throttle_duration_sec=2.0)
                    return
                self._jog_inhibit = False
            self._jog_vel = vel
            self._jog_stamp = now

    def _jog_tick(self):
        with self._arb_lock:
            if self._trajectory_active or self._state == 'Home' or self._settings is None:
                return
            now = time.monotonic()
            with self._jog_lock:
                vel = list(self._jog_vel)
                fresh = now - self._jog_stamp < self._jog_timeout
            if not fresh or not any(vel) or not self._armed:
                if self._jogging:
                    self._jogging = False
                    ok, reply = self._send(jog_line([0.0] * NUM_JOINTS))
                    if not ok:
                        self._send('S')
                return
            if self._streaming or self._state not in ('Idle', 'Jog'):
                self.get_logger().warn(f'Jog ignored in state {self._state}',
                                       throttle_duration_sec=2.0)
                return
            s = self._settings
            steps = [max(-s.max_speed[j], min(s.max_speed[j], v * s.steps_per_deg[j]))
                     for j, v in enumerate(vel)]
            ok, reply = self._send(jog_line(steps))
            if ok:
                self._jogging = True
                self._last_cmd_steps = None
            else:
                self.get_logger().warn(f'Jog refused by the board: {reply}',
                                       throttle_duration_sec=2.0)

    # ------------------------------------------------------------- homing

    def _report(self, text, error=False):
        # rclpy forbids one call site logging at different severities, so keep two calls.
        if error:
            self.get_logger().error(f'Homing: {text}')
        else:
            self.get_logger().info(f'Homing: {text}')
        self._homing_pub.publish(String(data=text))

    def _on_home(self, joint, res):
        name = 'all joints' if joint is None else self._joints[joint]
        with self._arb_lock:
            reason = None
            if not self._armed:
                reason = 'the arm is stopped; press START (~/unlock) first'
            elif joint is not None and self._settings.homing[joint] == 'none':
                reason = (f'{name} has no homing on the board (joints.joint{joint + 1}.homing: '
                          'none); send the config to the board first')
            elif self._trajectory_active:
                reason = 'a trajectory goal is active'
            elif self._jogging or self._streaming:
                reason = 'stop jogging / streaming first'
            elif self._state != 'Idle':
                reason = f'the board must be Idle (now {self._state}); stop first'
            if reason is None:
                ok, reply = self._send('H' if joint is None else f'H {joint}')
                if not ok:
                    reason = f'board refused: {reply}'
                else:
                    self._homing_requested = True
                    self._state = 'Home'  # until the next status report confirms it
                    self._last_cmd_steps = None
        if reason:
            res.success, res.message = False, reason
            return res
        self._report(f'{name}: homing started')
        res.success, res.message = True, f'Homing {name} started'
        return res

    # ----------------------------------------------------------- services

    def _on_stop(self, _req, res):
        ok, reply = self._halt('STOP requested')
        res.success = ok
        res.message = ('Stopped; all motion disarmed until START' if ok
                       else f'Stop command failed ({reply}); sent emergency halt')
        return res

    def _halt(self, reason):
        """Smooth stop on the board, abort everything, disarm trajectory execution."""
        with self._arb_lock:
            self._generation += 1
            self._set_armed(False)
            self._jogging = False
            self._last_cmd_steps = None
            with self._jog_lock:
                self._jog_vel = [0.0] * NUM_JOINTS
                self._jog_stamp = 0.0
                self._jog_inhibit = True
        ok, reply = self._send('S')
        if not ok:
            try:
                self._raw_write(b'!')  # real-time halt, needs no reply
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'STOP could not be sent: {exc}')
        self._traj_status(self._disarm_text(reason))
        return ok, reply

    def _on_unlock(self, _req, res):
        with self._arb_lock:
            if self._trajectory_active:
                res.success, res.message = False, 'a trajectory goal is active'
                return res
            ok, reply = self._send('E 1')
            if ok:
                self._set_armed(True)
        if not ok:
            res.success, res.message = False, f'Enable failed: {reply}; execution stays disarmed'
            return res
        self._traj_status('armed: re-armed by ~/unlock')
        res.success, res.message = True, 'Started: motors enabled, motion armed'
        return res

    def _on_motors(self, req, res):
        with self._arb_lock:
            if self._trajectory_active:
                res.success, res.message = False, 'a trajectory goal is active'
                return res
            ok, reply = self._send('E 1' if req.data else 'E 0')
            self._last_cmd_steps = None
        res.success = ok
        res.message = ('Motors enabled' if req.data else
                       'Motors disabled (home reference cleared)') if ok else reply
        return res

    def _on_set_zero(self, _req, res):
        with self._arb_lock:
            status = self._wait_status(self._status_timeout)
            if status is None or status['state'] != 'Idle' or self._trajectory_active:
                res.success, res.message = False, 'the board must be Idle'
                return res
            steps = [p if ref else 0 for p, ref in
                     zip(status['steps'][:NUM_JOINTS], status['referenced'][:NUM_JOINTS])]
            ok, reply = self._send('Z ' + format_steps(steps))
            self._last_cmd_steps = None
        zeroed = [n for n, ref in zip(self._joints, status['referenced']) if not ref]
        res.success = ok
        res.message = f"Zeroed {', '.join(zeroed) or 'nothing'}" if ok else reply
        return res

    def _on_set_zero_joint(self, joint, res):
        name = self._joints[joint]
        with self._arb_lock:
            reason = None
            status = None
            if self._trajectory_active:
                reason = 'a trajectory goal is active'
            elif self._jogging or self._streaming:
                reason = 'stop jogging / streaming first'
            else:
                status = self._wait_status(self._status_timeout)
                if status is None or status['state'] != 'Idle':
                    reason = 'the board must be Idle; stop first'
            if reason is None:
                steps = list(status['steps'][:NUM_JOINTS])
                steps[joint] = 0
                ok, reply = self._send('Z ' + format_steps(steps))
                self._last_cmd_steps = None
                if not ok:
                    reason = f'board refused: {reply}'
        if reason:
            res.success, res.message = False, reason
            return res
        self._report(f'{name}: position set to 0 (manual home)')
        res.success, res.message = True, f'{name} position set to 0'
        return res

    def _on_reload_settings(self, _req, res):
        try:
            self._load_settings()
        except Exception as exc:  # noqa: BLE001
            res.success, res.message = False, str(exc)
            return res
        res.success, res.message = True, 'Settings reloaded'
        return res

    def _on_push_config(self, _req, res):
        if not self._config_path:
            res.success, res.message = False, 'parameter config_path is not set'
            return res
        try:
            with open(self._config_path, encoding='utf-8') as f:
                upload = config_upload(f.read())
        except (OSError, UnicodeDecodeError, ProtocolError) as exc:
            res.success, res.message = False, f'{self._config_path}: {exc}'
            return res
        with self._arb_lock:
            status = self._wait_status(self._status_timeout)
            reason = None
            if self._trajectory_active:
                reason = 'a trajectory goal is active'
            elif self._jogging or self._streaming:
                reason = 'stop jogging / streaming first'
            elif status is None or status['state'] != 'Idle':
                reason = 'the board must be Idle; stop first'
            else:
                ok, reply = self._send(upload, CONFIG_TIMEOUT)
                if ok:
                    self._last_cmd_steps = None
                else:
                    reason = f'board refused the config: {reply}'
        if reason:
            res.success, res.message = False, reason
            return res
        try:
            self._load_settings()
        except Exception as exc:  # noqa: BLE001
            res.success, res.message = False, f'config saved on the board, but reloading failed: {exc}'
            return res
        message = 'Config saved on the board and applied (Wi-Fi changes after a board reboot)'
        self.get_logger().info(f'Pushed {self._config_path} to the board')
        res.success, res.message = True, message
        return res

    # ------------------------------------------------- trajectory action

    def _traj_status(self, text):
        self.get_logger().info(f'Trajectory: {text}')
        self._traj_status_pub.publish(String(data=text))

    @staticmethod
    def _disarm_text(reason):
        return f'disarmed: {reason}; check the arm, then START (~/unlock)'

    def _set_armed(self, armed):
        self._armed = armed
        if hasattr(self, '_armed_pub'):
            self._armed_pub.publish(Bool(data=armed))

    def _disarm(self, reason):
        self._set_armed(False)
        if hasattr(self, '_traj_status_pub'):
            self._traj_status(self._disarm_text(reason))

    def _jog_requested(self):
        with self._jog_lock:
            return time.monotonic() - self._jog_stamp < self._jog_timeout and any(self._jog_vel)

    def _trajectory_blocker(self):
        if not self._running:
            return 'board connection lost'
        if self._limits is None:
            return 'robot_description not set (joint limits unknown)'
        if not self._armed:
            return 'disarmed after STOP or a fault; press START (~/unlock) first'
        if self._state == 'Home':
            return 'homing in progress'
        if self._jogging or self._jog_requested():
            return 'jog active'
        if self._streaming:
            return 'a joint_trajectory is still streaming'
        return None

    def _on_trajectory_goal(self, _goal_request):
        with self._arb_lock:
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
            note = '' if self._armed else '; disarmed until START'
            self._traj_status(f'{outcome}: {message}{note}')
            return result
        finally:
            with self._arb_lock:
                self._last_cmd_steps = None
                self._trajectory_active = False

    def _fault(self, reason, code='PATH_TOLERANCE_VIOLATED'):
        self._halt(reason)
        return 'abort', code, reason

    def _check_goal_extras(self, goal):
        multi = goal.multi_dof_trajectory
        if multi.joint_names or multi.points:
            raise TrajectoryError('INVALID_GOAL', 'multi_dof_trajectory is not supported')
        if goal.path_tolerance:
            raise TrajectoryError('INVALID_GOAL', 'path_tolerance is not supported')
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
                reason = 'no fresh status from the board'
            elif status['state'] != 'Idle' or status['queued']:
                reason = f"the board must be Idle (now {status['state']})"
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
            plan = plan_trajectory(traj.joint_names, points, self._joints, self._limits,
                                   self._settings, status['steps'], status['referenced'],
                                   self._start_tolerance)
            gtt = goal.goal_time_tolerance.sec + goal.goal_time_tolerance.nanosec * 1e-9
            allowed = allowed_duration(plan.execution_time, gtt, self._time_scaling,
                                       self._time_margin)
        except TrajectoryError as exc:
            return 'abort', exc.code, str(exc)

        self._traj_status(f'executing: {len(plan.segments)} segments, {plan.duration:.2f} s '
                          f'(allowed {allowed:.1f} s)')
        lines = [segment_line(steps, dt_ms) for steps, dt_ms in plan.segments]
        sent = 0
        start = time.monotonic()
        deadline = start + allowed
        seen = None
        final_seq = None
        underruns = self._underruns
        while True:
            if goal_handle.is_cancel_requested:
                self._halt('trajectory goal canceled')
                return 'cancel', 'SUCCESSFUL', 'Trajectory canceled; motion stopped'
            if self._generation != generation:
                return 'abort', 'PATH_TOLERANCE_VIOLATED', 'Trajectory interrupted by STOP'
            if not self._running:
                return self._fault('board connection lost')
            now = time.monotonic()
            if now > deadline:
                return self._fault(f'trajectory did not finish within {allowed:.1f} s',
                                   'GOAL_TOLERANCE_VIOLATED')
            # Keep the board queue topped up; "err full" just means try again later.
            while sent < len(lines):
                ok, reply = self._send(lines[sent])
                if ok:
                    sent += 1
                    continue
                if reply == 'err full':
                    break
                if self._generation != generation:
                    return 'abort', 'PATH_TOLERANCE_VIOLATED', 'Trajectory interrupted by STOP'
                return self._fault(f'board refused segment {sent + 1}/{len(lines)}: {reply}')
            if self._underruns != underruns and sent < len(lines):
                return self._fault('the board ran out of segments mid-trajectory '
                                   '(serial link too slow?)')
            if self._status_time is None or now - self._status_time > self._status_timeout:
                return self._fault('board status reports are stale')
            with self._status_cond:
                self._status_cond.wait_for(lambda: self._status_seq != seen, 0.05)
                seq, status = self._status_seq, self._status
            if seq == seen:
                continue
            seen = seq
            if self._generation != generation:
                return 'abort', 'PATH_TOLERANCE_VIOLATED', 'Trajectory interrupted by STOP'
            state = status['state']
            if state not in ('Idle', 'Run'):
                return self._fault(f'the board entered {state} during the trajectory')
            if not status['enabled']:
                return self._fault('motors were disabled during the trajectory')
            actual = [self._settings.to_rad(j, s)
                      for j, s in enumerate(status['steps'][:NUM_JOINTS])]
            self._publish_feedback(goal_handle, plan, time.monotonic() - start, actual)
            if final_seq is None:
                if sent == len(lines):
                    final_seq = seq
                continue
            # A report requested after the last segment was accepted: Idle means really done.
            if seq >= final_seq + 2 and state == 'Idle' and status['queued'] == 0:
                off = within_tolerance(actual, plan.final, tolerances)
                if off is None:
                    return 'succeed', 'SUCCESSFUL', (
                        f'Trajectory completed: {len(lines)} segments, board Idle '
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
        node = NanoBeeBridge()
    except ProtocolError as exc:
        rclpy.logging.get_logger('nanobee_bridge').fatal(f'Cannot use the board: {exc}')
        rclpy.try_shutdown()
        raise SystemExit(1)
    except (OSError, ConnectionError, CommandError) as exc:
        rclpy.logging.get_logger('nanobee_bridge').fatal(
            f'Cannot connect to NanoBee: {exc}. After unplugging the board, reattach it to WSL '
            '(Windows: usbipd attach --wsl --busid <id>) and close other programs using the port '
            '(serial monitor, NanoBeeDriver).')
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
