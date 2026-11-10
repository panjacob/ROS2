"""Web page for the arm: live state + control.

GET  /              page (web/index.html)
GET  /state         JSON snapshot (joints, TCP transform, bridge status, config)
POST /api/jog       {"joint1": deg_per_s, ...}   velocity jog (send repeatedly, ~10 Hz)
POST /api/goto      {"joint1": deg, ...}         absolute move -> joint_command
POST /api/home      {"joint": "joint1"}          endstop homing of one joint
POST /api/stop      {}                           abort everything
POST /api/unlock    {}                           clear FluidNC alarm
POST /api/joy       {"axes": [...], "buttons": [...]}  browser gamepad -> /joy
POST /api/tcp/save  {"name": "gripper", "x_mm": 0, ..., "yaw_deg": 0}
POST /api/tcp/select {"name": "gripper"}
POST /api/tcp/delete {"name": "gripper"}
POST /api/plan      {"joint1": deg, ..., "joint5": deg}  plan only, preview in RViz
POST /api/execute   {"plan_id": "..."}      execute previewed plan in simulation
"""

import json
import math
import os
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (Constraints, DisplayTrajectory, JointConstraint,
                             MoveItErrorCodes)
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from arm_web.tcp_registry import (
    TcpRegistry, quaternion_from_rpy_degrees, rpy_degrees_from_quaternion)
from arm_web.joint_planning import (check_plan_start, joint_limits_from_urdf,
                                    validate_joint_goal)

STATUS_TOPICS = {
    'machine_state': String,
    'limit_switches': String,
    'feed_rate': Float64,
    'spindle_speed': Float64,
    'homing_status': String,
}

PLAN_ERRORS = {
    MoveItErrorCodes.FAILURE: 'Planning failed; inspect MoveIt logs for details.',
    MoveItErrorCodes.PLANNING_FAILED: 'No collision-free path found within the planning time.',
    MoveItErrorCodes.INVALID_MOTION_PLAN: 'MoveIt rejected the planned path.',
    MoveItErrorCodes.TIMED_OUT: 'Planning timed out; try another target.',
    MoveItErrorCodes.START_STATE_IN_COLLISION: 'Current robot state is in collision.',
    MoveItErrorCodes.GOAL_IN_COLLISION: 'Target pose is in collision.',
    MoveItErrorCodes.START_STATE_INVALID: 'Current robot state is invalid.',
    MoveItErrorCodes.GOAL_STATE_INVALID: 'Target robot state is invalid.',
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: 'Target joint constraints are invalid.',
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: 'Planned path does not reach the target.',
    MoveItErrorCodes.ROBOT_STATE_STALE: 'Robot joint state is stale.',
    MoveItErrorCodes.INVALID_GROUP_NAME: 'MoveIt arm group is missing.',
    MoveItErrorCodes.CONTROL_FAILED: 'Controller could not complete the trajectory.',
}

PLAN_TTL = 120.0


class StateStore:
    """Thread-safe latest values, shared between ROS callbacks and HTTP threads."""

    def __init__(self):
        self._lock = threading.Lock()
        self._joints = {}
        self._joints_stamp = None
        self._status = {}
        self._tcp = None
        self._tcp_stamp = None
        self._tcp_received = None
        self._tcp_error = 'Waiting for TF'

    def update_joints(self, msg):
        with self._lock:
            for i, name in enumerate(msg.name):
                pos = msg.position[i] if i < len(msg.position) else None
                vel = msg.velocity[i] if i < len(msg.velocity) else None
                self._joints[name] = {
                    'position_rad': pos,
                    'position_deg': math.degrees(pos) if pos is not None else None,
                    'velocity': vel,
                }
            self._joints_stamp = time.monotonic()

    def update_status(self, key, value):
        with self._lock:
            self._status[key] = {'value': value, 'stamp': time.monotonic()}

    def update_tcp(self, transform):
        stamp = (transform.header.stamp.sec, transform.header.stamp.nanosec)
        with self._lock:
            if stamp != self._tcp_stamp:
                translation = transform.transform.translation
                self._tcp = {
                    'reference_frame': transform.header.frame_id,
                    'frame': transform.child_frame_id,
                    'position_m': {
                        'x': translation.x,
                        'y': translation.y,
                        'z': translation.z,
                    },
                    'rpy_deg': rpy_degrees_from_quaternion(transform.transform.rotation),
                }
                self._tcp_stamp = stamp
                self._tcp_received = time.monotonic()
            self._tcp_error = None

    def mark_tcp_unavailable(self, error):
        with self._lock:
            self._tcp_error = str(error)

    def reset_tcp(self):
        with self._lock:
            self._tcp = None
            self._tcp_stamp = None
            self._tcp_received = None
            self._tcp_error = 'Waiting for TF'

    def snapshot(self):
        now = time.monotonic()
        with self._lock:
            joints_age = None if self._joints_stamp is None else now - self._joints_stamp
            return {
                'joints': {k: dict(v) for k, v in self._joints.items()},
                'joints_age': joints_age,
                'tcp': None if self._tcp is None else {
                    **self._tcp,
                    'position_m': dict(self._tcp['position_m']),
                    'rpy_deg': list(self._tcp['rpy_deg']),
                    'age': now - self._tcp_received,
                },
                'tcp_error': self._tcp_error,
                'status': {
                    k: {'value': v['value'], 'age': now - v['stamp']}
                    for k, v in self._status.items()
                },
            }


class WebViewer(Node):

    def __init__(self):
        super().__init__('web_viewer')
        self.host = self.declare_parameter('host', '0.0.0.0').value
        self.port = int(self.declare_parameter('port', 8080).value)
        bridge_ns = self.declare_parameter('bridge_ns', '/fluidnc_bridge').value.rstrip('/')
        self.simulation = bool(self.declare_parameter('simulation', False).value)
        configured_home_joints = self.declare_parameter('home_joints', ['joint1']).value
        self.home_joints = [] if self.simulation else list(configured_home_joints)
        # FluidNC Pn letter of each home joint's endstop (same order as home_joints)
        self.home_pins = list(self.declare_parameter('home_pins', ['X']).value)
        self.max_jog_speed = float(self.declare_parameter('max_jog_speed', 360.0).value)
        self.reference_frame = self.declare_parameter('reference_frame', 'world').value
        tcp_storage_path = self.declare_parameter(
            'tcp_storage_path', '~/.config/ros2_arm/tcps.json').value
        self.tcps = TcpRegistry(tcp_storage_path)
        self.store = StateStore()
        urdf = self.declare_parameter('robot_description', '').value
        self.robot_name, self.joint_limits = joint_limits_from_urdf(urdf)
        self._plan_lock = threading.Lock()
        self._plan_state_lock = threading.Lock()
        self._plan_record = None
        self._plan_status = {'state': 'none', 'message': 'No plan yet.'}
        self._execution_goal_handle = None
        self._plan_client = ActionClient(self, MoveGroup, 'move_action')
        self._execute_client = ActionClient(
            self, ExecuteTrajectory, 'execute_trajectory')
        preview_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._preview_pub = self.create_publisher(
            DisplayTrajectory, 'display_planned_path', preview_qos)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.1, self._update_tcp)

        self.create_subscription(JointState, 'joint_states', self.store.update_joints, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for key, msg_type in STATUS_TOPICS.items():
            qos = latched if key == 'homing_status' else 10
            self.create_subscription(
                msg_type, f'{bridge_ns}/{key}',
                lambda m, k=key: self.store.update_status(k, m.data), qos)

        self._jog_pub = self.create_publisher(JointState, 'jog', 10)
        self._cmd_pub = self.create_publisher(JointState, 'joint_command', 10)
        self._joy_pub = self.create_publisher(Joy, 'joy', 10)
        self._trigger_clients = {
            'stop': self.create_client(Trigger, f'{bridge_ns}/stop'),
            'unlock': self.create_client(Trigger, f'{bridge_ns}/unlock'),
        }
        for joint in self.home_joints:
            self._trigger_clients[f'home_{joint}'] = self.create_client(
                Trigger, f'{bridge_ns}/home_{joint}')

    def config(self):
        tcp_config = self.tcps.snapshot()
        return {
            'home_joints': self.home_joints,
            'endstops': dict(zip(self.home_joints, self.home_pins)),
            'max_jog_speed': self.max_jog_speed,
            'simulation': self.simulation,
            'execution_available': self.simulation,
            'reference_frame': self.reference_frame,
            'joint_limits_deg': {
                name: [math.degrees(lo), math.degrees(hi)]
                for name, (lo, hi) in self.joint_limits.items()},
            'tcp_frame': next(tcp['frame'] for tcp in tcp_config['tcps']
                              if tcp['name'] == tcp_config['selected_tcp']),
            **tcp_config,
        }

    def plan_snapshot(self):
        with self._plan_state_lock:
            status = dict(self._plan_status)
            record = self._plan_record
            if record is not None:
                status['plan_id'] = record['id']
                status['age'] = time.monotonic() - record['created']
                if status['age'] > PLAN_TTL:
                    status['state'] = 'expired'
                    status['message'] = 'Plan expired; plan again.'
            return status

    def _invalidate_plan(self, message='Plan invalidated by another command.'):
        with self._plan_state_lock:
            had_plan = self._plan_record is not None
            self._plan_record = None
            if self._plan_status['state'] != 'executing':
                self._plan_status = {'state': 'none', 'message': message}
        if had_plan:
            self._preview_pub.publish(DisplayTrajectory())

    def _update_tcp(self):
        tcp_config = self.tcps.snapshot()
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for tcp in tcp_config['tcps']:
            if tcp['name'] == 'tool0':
                continue
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = 'tool0'
            transform.child_frame_id = tcp['frame']
            transform.transform.translation.x, transform.transform.translation.y, \
                transform.transform.translation.z = tcp['xyz_m']
            rotation = quaternion_from_rpy_degrees(tcp['rpy_deg'])
            transform.transform.rotation.x, transform.transform.rotation.y, \
                transform.transform.rotation.z, transform.transform.rotation.w = rotation
            transforms.append(transform)
        if transforms:
            self._tf_broadcaster.sendTransform(transforms)
        selected = tcp_config['selected_tcp']
        frame = next(tcp['frame'] for tcp in tcp_config['tcps'] if tcp['name'] == selected)
        try:
            transform = self._tf_buffer.lookup_transform(
                self.reference_frame, frame, Time())
        except TransformException as exc:
            self.store.mark_tcp_unavailable(exc)
            return
        self.store.update_tcp(transform)

    def select_tcp(self, body):
        name = body.get('name')
        self.tcps.select(name)
        self.store.reset_tcp()
        return True, f'TCP {name} selected'

    def save_tcp(self, body):
        name = body.get('name')
        xyz_m = [float(body[f'{axis}_mm']) / 1000 for axis in 'xyz']
        rpy_deg = [float(body[f'{axis}_deg']) for axis in ('roll', 'pitch', 'yaw')]
        self.tcps.save(name, xyz_m, rpy_deg)
        self.store.reset_tcp()
        return True, f'TCP {name} saved and selected'

    def delete_tcp(self, body):
        name = body.get('name')
        self.tcps.delete(name)
        self.store.reset_tcp()
        return True, f'TCP {name} deleted'

    # --- actions (called from HTTP threads; the main thread spins) ---

    def _joint_state(self, values, field):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(values.keys())
        setattr(msg, field, [math.radians(float(v)) for v in values.values()])
        return msg

    def jog(self, values):
        limit = self.max_jog_speed
        values = {k: max(-limit, min(limit, float(v))) for k, v in values.items()}
        if any(values.values()):
            self._invalidate_plan()
        self._jog_pub.publish(self._joint_state(values, 'velocity'))
        return True, 'ok'

    def goto(self, values):
        if not values:
            return False, 'no joints given'
        self._invalidate_plan()
        self._cmd_pub.publish(self._joint_state(values, 'position'))
        return True, 'sent'

    def plan(self, values):
        if not self._plan_lock.acquire(blocking=False):
            return False, 'Planning is already in progress.'
        try:
            with self._plan_state_lock:
                if self._plan_status['state'] == 'executing':
                    return False, 'Wait for the current execution to finish.'
                self._plan_record = None
                self._plan_status = {'state': 'planning', 'message': 'Planning…'}
            ok, message = self._plan_locked(values)
            with self._plan_state_lock:
                if not ok:
                    self._plan_status = {'state': 'failed', 'message': message}
                    return False, message
                plan_id = self._plan_record['id']
                self._plan_status = {'state': 'ready', 'message': message}
                return True, message, {'plan_id': plan_id}
        finally:
            self._plan_lock.release()

    def _plan_locked(self, values):
        # A previous preview must not be mistaken for the answer to this request.
        self._preview_pub.publish(DisplayTrajectory())
        try:
            target = validate_joint_goal(values, self.joint_limits)
        except ValueError as exc:
            return False, str(exc)
        state = self.store.snapshot()
        if state['joints_age'] is None or state['joints_age'] > 2.0 or any(
                name not in state['joints'] or
                state['joints'][name]['position_rad'] is None
                for name in self.joint_limits):
            return False, 'No fresh joint state. Wait for /joint_states and try again.'
        if not self._plan_client.wait_for_server(timeout_sec=1.0):
            return False, 'MoveIt planner is unavailable (/move_action).'

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.pipeline_id = 'ompl'
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 1
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        goal.request.start_state.is_diff = True
        constraints = Constraints()
        for name, position in target.items():
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = 0.001
            constraint.tolerance_below = 0.001
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True

        try:
            accepted = self._wait_future(self._plan_client.send_goal_async(goal), 3.0)
            if accepted is None:
                return False, 'MoveIt did not accept the request in time.'
            if not accepted.accepted:
                return False, 'MoveIt rejected the planning request.'
            response = self._wait_future(accepted.get_result_async(), 12.0)
            if response is None:
                accepted.cancel_goal_async()
                return False, 'Planning timed out.'
        except Exception as exc:
            self.get_logger().error(f'MoveIt planning request failed: {exc}')
            return False, 'Planner communication failed; see ROS logs.'

        result = response.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            reason = PLAN_ERRORS.get(result.error_code.val,
                                     f'MoveIt error code {result.error_code.val}.')
            return False, reason
        points = result.planned_trajectory.joint_trajectory.points
        if not points:
            return False, 'MoveIt returned an empty trajectory.'
        start_state = result.trajectory_start.joint_state
        start_positions = dict(zip(start_state.name, start_state.position))
        if any(name not in start_positions for name in self.joint_limits):
            return False, 'MoveIt returned a plan without a complete start state.'
        seconds = (points[-1].time_from_start.sec +
                   points[-1].time_from_start.nanosec * 1e-9)
        preview = DisplayTrajectory()
        preview.model_id = self.robot_name
        preview.trajectory_start = result.trajectory_start
        preview.trajectory = [result.planned_trajectory]
        self._preview_pub.publish(preview)
        with self._plan_state_lock:
            self._plan_record = {
                'id': secrets.token_urlsafe(12),
                'created': time.monotonic(),
                'trajectory': result.planned_trajectory,
                'start': {name: start_positions[name] for name in self.joint_limits},
                'duration': seconds,
            }
        return True, (f'Plan ready: {len(points)} points, {seconds:.1f} s. '
                      'Preview in RViz; ready to execute in simulation.')

    def execute(self, body):
        if not self.simulation:
            return False, 'Execution is available only in simulation; the hardware bridge has no trajectory action controller.'
        plan_id = body.get('plan_id')
        with self._plan_state_lock:
            record = self._plan_record
            if record is None or record['id'] != plan_id or self._plan_status['state'] != 'ready':
                return False, 'No matching ready plan. Plan again before executing.'
            if time.monotonic() - record['created'] > PLAN_TTL:
                self._plan_record = None
                self._plan_status = {'state': 'expired', 'message': 'Plan expired; plan again.'}
                return False, 'Plan expired; plan again.'
            state = self.store.snapshot()
            if state['joints_age'] is None or state['joints_age'] > 2.0:
                return False, 'No fresh joint state; plan again.'
            try:
                check_plan_start(
                    {name: entry['position_rad'] for name, entry in state['joints'].items()},
                    record['start'])
            except ValueError as exc:
                return False, str(exc)
            self._plan_record = None  # single use
            self._plan_status = {'state': 'executing', 'message': 'Executing plan…'}
        self._preview_pub.publish(DisplayTrajectory())

        ok, message = self._execute_trajectory(record)
        with self._plan_state_lock:
            stopped = self._plan_status['state'] == 'stopped'
            if not stopped:
                self._plan_status = {
                    'state': 'completed' if ok else 'failed', 'message': message}
            self._execution_goal_handle = None
        return (False, 'Execution stopped.') if stopped else (ok, message)

    def _execute_trajectory(self, record):
        if not self._execute_client.wait_for_server(timeout_sec=2.0):
            return False, 'MoveIt execution action is unavailable.'
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = record['trajectory']
        try:
            accepted = self._wait_future(self._execute_client.send_goal_async(goal), 3.0)
            if accepted is None or not accepted.accepted:
                return False, 'MoveIt rejected trajectory execution.'
            with self._plan_state_lock:
                self._execution_goal_handle = accepted
                stopped = self._plan_status['state'] == 'stopped'
            if stopped:
                accepted.cancel_goal_async()
            response = self._wait_future(
                accepted.get_result_async(), max(10.0, record['duration'] * 1.5 + 5.0))
            if response is None:
                accepted.cancel_goal_async()
                self.call('stop')
                return False, 'Execution timed out; simulator stopped.'
        except Exception as exc:
            self.get_logger().error(f'MoveIt execution failed: {exc}')
            self.call('stop')
            return False, 'Execution communication failed; simulator stopped.'
        code = response.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            return False, PLAN_ERRORS.get(code, f'MoveIt execution failed (code {code}).')
        return True, 'Planned trajectory executed successfully.'

    def stop(self):
        with self._plan_state_lock:
            self._plan_record = None
            self._plan_status = {'state': 'stopped', 'message': 'Stopped.'}
            handle = self._execution_goal_handle
        self._preview_pub.publish(DisplayTrajectory())
        if handle is not None:
            handle.cancel_goal_async()
        return self.call('stop')

    @staticmethod
    def _wait_future(future, timeout):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            return None
        return future.result()

    def joy(self, axes, buttons):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [float(a) for a in axes]
        msg.buttons = [int(b) for b in buttons]
        if any(abs(value) > 0.1 for value in msg.axes) or any(msg.buttons):
            self._invalidate_plan()
        self._joy_pub.publish(msg)
        return True, 'ok'

    def call(self, name, timeout=5.0):
        client = self._trigger_clients.get(name)
        if client is None:
            return False, f'unknown action {name}'
        if not client.service_is_ready():
            return False, f'service {client.srv_name} not available (bridge running?)'
        done = threading.Event()
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            return False, 'timeout waiting for bridge'
        res = future.result()
        return bool(res.success), res.message


def make_handler(node, page_path):

    class Handler(BaseHTTPRequestHandler):

        def log_message(self, fmt, *args):
            pass

        def _send(self, code, body, ctype='application/json'):
            data = body.encode() if isinstance(body, str) else body
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                with open(page_path, 'rb') as f:
                    self._send(200, f.read(), 'text/html; charset=utf-8')
            elif self.path == '/state':
                snap = node.store.snapshot()
                snap['config'] = node.config()
                snap['plan'] = node.plan_snapshot()
                self._send(200, json.dumps(snap))
            else:
                self._send(404, '{"error": "not found"}')

        def do_POST(self):
            # Browsers can't send a cross-site application/json POST without a CORS preflight
            # (which we never answer), so other web pages cannot move the arm.
            if not (self.headers.get('Content-Type') or '').startswith('application/json'):
                self._send(415, '{"ok": false, "message": "Content-Type must be application/json"}')
                return
            try:
                length = int(self.headers.get('Content-Length') or 0)
                body = json.loads(self.rfile.read(length) or b'{}')
                if not isinstance(body, dict):
                    raise ValueError('body must be a JSON object')
                outcome = self._dispatch(body)
                ok, message = outcome[:2]
                extra = outcome[2] if len(outcome) > 2 else {}
            except (ValueError, TypeError, KeyError, OSError) as exc:
                ok, message = False, f'bad request: {exc}'
                extra = {}
            self._send(200 if ok else 400,
                       json.dumps({'ok': ok, 'message': message, **extra}))

        def _dispatch(self, body):
            path = self.path
            if path == '/api/jog':
                return node.jog(body)
            if path == '/api/goto':
                return node.goto(body)
            if path == '/api/plan':
                return node.plan(body)
            if path == '/api/execute':
                return node.execute(body)
            if path == '/api/joy':
                return node.joy(body.get('axes', []), body.get('buttons', []))
            if path == '/api/tcp/save':
                return node.save_tcp(body)
            if path == '/api/tcp/select':
                return node.select_tcp(body)
            if path == '/api/tcp/delete':
                return node.delete_tcp(body)
            if path == '/api/home':
                node._invalidate_plan()
                return node.call(f"home_{body.get('joint', '')}", timeout=5.0)
            if path == '/api/stop':
                return node.stop()
            if path == '/api/unlock':
                return node.call('unlock')
            return False, 'unknown endpoint'

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = WebViewer()
    page = os.path.join(get_package_share_directory('arm_web'), 'web', 'index.html')
    try:
        server = ThreadingHTTPServer((node.host, node.port), make_handler(node, page))
    except OSError as exc:
        node.get_logger().fatal(
            f'Cannot open port {node.port}: {exc}. Another web_viewer still running? '
            f'Use web_port:=<other> or stop it.')
        node.destroy_node()
        rclpy.try_shutdown()
        raise SystemExit(1)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f'Web viewer at http://localhost:{node.port}')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
