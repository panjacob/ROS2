"""Real ROS message boundaries with in-process mocks; never connects to hardware."""
import math
import threading
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from arm_web.joint_planning import joint_limits_from_urdf
from arm_web.tcp_planning import PointKinematics
from arm_web.tcp_registry import TcpRegistry
from arm_web.web_viewer import StateStore, WebViewer


def future(value):
    result = Future()
    result.set_result(value)
    return result


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


class Planner:
    def __init__(self, node):
        self.node = node
        self.requests = []
        self.code = MoveItErrorCodes.SUCCESS
        self.finish_hook = lambda: None
        self.endpoint_shift = 0.
        self.cancelled = False

    def wait_for_server(self, **_):
        return True

    def send_goal_async(self, goal):
        self.requests.append(goal)
        result = MoveGroup.Result()
        result.error_code.val = self.code
        result.trajectory_start.joint_state = JointState(
            name=list(self.node.joint_limits), position=[0.]*5)
        trajectory = result.planned_trajectory.joint_trajectory
        trajectory.joint_names = list(self.node.joint_limits)
        positions = {c.joint_name: c.position for c in goal.request.goal_constraints[0].joint_constraints}
        endpoint = JointTrajectoryPoint(positions=[positions[name] for name in trajectory.joint_names])
        endpoint.positions[1] += self.endpoint_shift
        endpoint.time_from_start.sec = 3
        trajectory.points = [JointTrajectoryPoint(positions=[0.]*5), endpoint]

        def finished():
            self.finish_hook()
            return future(SimpleNamespace(result=result))

        def cancel():
            self.cancelled = True

        return future(SimpleNamespace(accepted=True, get_result_async=finished, cancel_goal_async=cancel))


class Fk:
    def __init__(self, node):
        self.node = node
        self.requests = []
        self.shift = 0.
        self.available = True

    def wait_for_service(self, **_):
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        joints = dict(zip(request.robot_state.joint_state.name, request.robot_state.joint_state.position))
        xyz = self.node._point_kinematics.point(joints, [0., 0., 0.])
        yaw, shoulder, elbow, wrist, roll = [joints[f'joint{i}'] for i in range(1, 6)]
        pitch = shoulder+elbow+wrist
        rotation = multiply(multiply([0., 0., math.sin(yaw/2), math.cos(yaw/2)],
                                     [0., math.sin(pitch/2), 0., math.cos(pitch/2)]),
                            [0., 0., math.sin(roll/2), math.cos(roll/2)])
        pose = PoseStamped()
        pose.header.frame_id = request.header.frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, xyz)
        pose.pose.position.x += self.shift
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = rotation
        response = GetPositionFK.Response()
        response.error_code.val = MoveItErrorCodes.SUCCESS
        response.fk_link_names = ['tool0']
        response.pose_stamped = [pose]
        return future(response)


@pytest.fixture
def viewer(tmp_path):
    node = object.__new__(WebViewer)
    urdf = (Path(__file__).resolve().parents[2] / 'arm_description/urdf/arm_concept.urdf').read_text()
    node.robot_name, node.joint_limits = joint_limits_from_urdf(urdf)
    node._point_kinematics = PointKinematics(urdf, node.joint_limits)
    node.tcps = TcpRegistry(tmp_path/'tcp.json')
    node.tcps.save('bent_tool', [.04, -.025, .12], [37., -61., 83.])
    node.store = StateStore()
    node.store.update_joints(JointState(name=list(node.joint_limits), position=[0.]*5))
    node._plan_lock = threading.Lock()
    node._plan_state_lock = threading.Lock()
    node._plan_generation = 0
    node._plan_record = None
    node._plan_status = {'state': 'none', 'message': 'No plan'}
    node._execution_goal_handle = None
    node._execution_pending = False
    node.reference_frame = 'world'
    node.velocity_scaling = node.acceleration_scaling = .3
    node.previews = []
    node._preview_pub = SimpleNamespace(publish=node.previews.append)
    node._plan_client = Planner(node)
    node._fk_client = Fk(node)
    node.call = lambda *_: (True, 'mock bridge stopped')
    node.get_logger = lambda: SimpleNamespace(error=lambda _: None)
    return node


def goal(node):
    tcp = next(t for t in node.tcps.snapshot()['tcps'] if t['name'] == node.tcps.snapshot()['selected_tcp'])
    point = node._point_kinematics.point(dict(zip(node.joint_limits, [.3, .4, -.6, .7, 1.])), tcp['xyz_m'])
    return {'tcp_name': tcp['name'], **{f'{axis}_mm': float(value)*1000 for axis, value in zip('xyz', point)}}


def test_gazebo_waits_for_scene_before_planning_or_execution(viewer):
    viewer.scene_ready = False
    assert not viewer.plan_tcp(goal(viewer))[0]
    assert not viewer.plan({})[0]
    assert not viewer.execute({'plan_id': 'old'})[0]
    assert not viewer._plan_client.requests
    viewer._scene_readiness(SimpleNamespace(data=True))
    assert viewer.plan_tcp(goal(viewer))[0]
    viewer._scene_readiness(SimpleNamespace(data=False))
    assert viewer._plan_record is None


def test_gazebo_rejects_unsupported_direct_motion(viewer):
    viewer.direct_motion_available = False
    assert not viewer.jog({'joint1': 5})[0]
    assert not viewer.goto({'joint1': 5})[0]
    assert not viewer.joy([1.0], [1])[0]


def test_gazebo_jog_works_without_enabling_direct_moves(viewer):
    import time
    viewer.direct_motion_available = False
    viewer.jog_available = True
    viewer.require_scene_ready = viewer.scene_ready = True
    viewer.max_jog_speed = 120.
    viewer._joint_state = lambda values, field: (values, field)
    published = []
    viewer._jog_pub = SimpleNamespace(publish=published.append)
    assert viewer.plan_tcp(goal(viewer))[0]
    assert viewer.jog({'joint1': 200})[0]
    assert published[-1] == ({'joint1': 120.}, 'velocity')
    assert viewer._plan_record is None
    assert not viewer.plan_tcp(goal(viewer))[0]
    assert not viewer.execute({'plan_id': 'old'})[0]
    assert not viewer.goto({'joint1': 0})[0]
    assert viewer.jog({'joint1': 0})[0]
    assert viewer._jog_until > time.monotonic()


def test_gazebo_jog_does_not_preempt_planned_execution(viewer):
    viewer.jog_available = True
    viewer.require_scene_ready = viewer.scene_ready = True
    viewer.max_jog_speed = 30.
    viewer._execution_pending = True
    assert not viewer.jog({'joint1': 5})[0]
    viewer._execution_pending = False
    viewer._plan_status['state'] = 'executing'
    assert not viewer.jog({'joint1': 5})[0]


@pytest.mark.parametrize('values', [{'unknown': 1}, {'joint1': float('nan')}, {'joint1': float('inf')}])
def test_jog_rejects_unknown_joints_and_nonfinite_speed(viewer, values):
    viewer.jog_available = True
    viewer.max_jog_speed = 30.
    assert not viewer.jog(values)[0]


class MotorsClient:
    srv_name = '/nanobee_bridge/motors'

    def __init__(self):
        self.requests = []
        self.ready = True

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request.data)
        return future(SimpleNamespace(success=True, message='Motors ' + ('on' if request.data else 'off')))


def test_motors_endpoint_calls_bridge_and_invalidates_plan(viewer):
    viewer.simulation = False
    viewer._motors_client = MotorsClient()
    assert viewer.plan_tcp(goal(viewer))[0]
    assert viewer.motors({'enable': False}) == (True, 'Motors off')
    assert viewer._plan_record is None
    assert viewer.motors({'enable': True})[0]
    assert viewer._motors_client.requests == [False, True]
    for body in ({}, {'enable': 'no'}, {'enable': 1}):
        assert not viewer.motors(body)[0]
    viewer._motors_client.ready = False
    assert not viewer.motors({'enable': True})[0]


def test_motors_rejected_in_simulation_and_during_execution(viewer):
    viewer._motors_client = MotorsClient()
    viewer.simulation = True
    assert not viewer.motors({'enable': False})[0]
    viewer.simulation = False
    viewer._execution_pending = True
    assert not viewer.motors({'enable': False})[0]
    viewer._execution_pending = False
    viewer._plan_status['state'] = 'executing'
    assert not viewer.motors({'enable': False})[0]
    assert viewer._motors_client.requests == []


def test_stopped_bridge_refuses_motion_until_start(viewer):
    viewer.simulation = False
    viewer.jog_available = True
    viewer.max_jog_speed = 30.
    viewer._joint_state = lambda values, field: (values, field)
    published = []
    viewer._jog_pub = viewer._cmd_pub = viewer._joy_pub = SimpleNamespace(publish=published.append)
    calls = []
    viewer.call = lambda name, timeout=5.0: (calls.append(name), (True, 'ok'))[1]
    viewer.store.update_status('armed', False)
    for result in (viewer.jog({'joint1': 5}), viewer.goto({'joint1': 5}),
                   viewer.joy([1.0], [0]), viewer.home('joint1'),
                   viewer.execute({'plan_id': 'x'})):
        assert result == (False, 'The arm is stopped; press START first.')
    assert published == [] and calls == []
    assert viewer.jog({'joint1': 0})[0]  # releasing a jog is always allowed
    viewer.store.update_status('armed', True)
    assert viewer.goto({'joint1': 5})[0]
    assert viewer.home('joint1')[0] and calls == ['home_joint1']
    viewer.simulation = True
    viewer.store.update_status('armed', False)
    assert not viewer.stopped()


class PushConfigClient:
    srv_name = '/nanobee_bridge/push_config'

    def __init__(self, path, success=True):
        self.path, self.success, self.uploaded = path, success, []

    def service_is_ready(self):
        return True

    def call_async(self, _request):
        self.uploaded.append(Path(self.path).read_text())
        return future(SimpleNamespace(success=self.success, message='board says ' + str(self.success)))


def test_board_config_load_save_and_push(viewer, tmp_path):
    path = tmp_path / 'config.yaml'
    path.write_text('motion:\n  blend: true\n')
    viewer.simulation = False
    viewer.board_config_path = str(path)
    viewer._board_config_lock = threading.Lock()
    viewer._push_config_client = PushConfigClient(path)
    assert viewer.board_config() == (True, f'Loaded {path}', {'yaml': 'motion:\n  blend: true\n',
                                                              'path': str(path)})
    assert viewer.plan_tcp(goal(viewer))[0]
    ok, message = viewer.push_board_config({'yaml': 'motion:\n  blend: false\n'})
    assert ok and 'board says True' in message
    assert viewer._push_config_client.uploaded == ['motion:\n  blend: false\n']
    assert path.read_text() == 'motion:\n  blend: false\n'
    assert viewer._plan_record is None

    viewer._push_config_client.success = False
    ok, message = viewer.push_board_config({'yaml': 'bogus: 1\n'})
    assert not ok and 'board says False' in message
    assert viewer._push_config_client.uploaded[-1] == 'bogus: 1\n'
    assert path.read_text() == 'motion:\n  blend: false\n'  # restored


def test_board_config_rejected_without_writing(viewer, tmp_path):
    path = tmp_path / 'config.yaml'
    path.write_text('a: 1\n')
    viewer.board_config_path = str(path)
    viewer._board_config_lock = threading.Lock()
    viewer._push_config_client = PushConfigClient(path)
    viewer.simulation = True
    assert not viewer.push_board_config({'yaml': 'b: 2\n'})[0]
    viewer.simulation = False
    for body in ({}, {'yaml': ''}, {'yaml': 3}, {'yaml': 'x' * 40000}):
        assert not viewer.push_board_config(body)[0]
    viewer._execution_pending = True
    assert not viewer.push_board_config({'yaml': 'b: 2\n'})[0]
    assert path.read_text() == 'a: 1\n' and viewer._push_config_client.uploaded == []
    viewer.board_config_path = ''
    assert not viewer.board_config()[0]


@pytest.mark.parametrize('name', ['tool0', 'bent_tool'])
def test_position_plan_uses_collision_planner_and_verified_fk(viewer, name):
    viewer.tcps.select(name)
    outcome = viewer.plan_tcp(goal(viewer))
    assert outcome[0], outcome
    request = viewer._plan_client.requests[-1]
    assert request.planning_options.plan_only
    assert request.request.pipeline_id == 'ompl'
    assert 2 <= len(request.request.goal_constraints) <= 24
    assert all(len(c.joint_constraints) == 5 for c in request.request.goal_constraints)
    assert all(not c.orientation_constraints for c in request.request.goal_constraints)
    assert len(viewer._fk_client.requests) == 1
    assert viewer._plan_record['tcp_context']['endpoint_error_mm'] <= 1.
    assert outcome[2]['plan_id'] == viewer._plan_record['id']
    assert viewer.plan_snapshot()['state'] == 'ready'
    assert viewer.previews[-1].trajectory


@pytest.mark.parametrize('code', [MoveItErrorCodes.GOAL_IN_COLLISION, MoveItErrorCodes.PLANNING_FAILED,
                                 MoveItErrorCodes.START_STATE_IN_COLLISION])
def test_moveit_rejections_never_produce_executable_preview(viewer, code):
    viewer._plan_client.code = code
    assert not viewer.plan_tcp(goal(viewer))[0]
    assert viewer._plan_record is None
    assert viewer.plan_snapshot()['state'] == 'failed'
    assert all(not preview.trajectory for preview in viewer.previews)
    assert not viewer._fk_client.requests


def test_unreachable_goal_never_calls_planner(viewer):
    body = goal(viewer)
    body['x_mm'] = 100000.
    assert not viewer.plan_tcp(body)[0]
    assert not viewer._plan_client.requests
    assert viewer._plan_record is None


@pytest.mark.parametrize('fault', ['endpoint', 'model', 'unavailable_fk'])
def test_endpoint_validation_fails_closed(viewer, fault):
    if fault == 'endpoint':
        viewer._plan_client.endpoint_shift = .1
    elif fault == 'model':
        viewer._fk_client.shift = .01
    else:
        viewer._fk_client.available = False
    assert not viewer.plan_tcp(goal(viewer))[0]
    assert viewer._plan_record is None
    assert all(not preview.trajectory for preview in viewer.previews)


@pytest.mark.parametrize('change', ['stop', 'manual', 'select', 'edit', 'delete'])
def test_inflight_plan_cannot_resurrect_after_command(viewer, change):
    def mutate():
        if change == 'stop':
            viewer.stop()
        elif change == 'manual':
            viewer._invalidate_plan()
        elif change == 'select':
            viewer.select_tcp({'name': 'tool0'})
        elif change == 'delete':
            viewer.delete_tcp({'name': 'bent_tool'})
        else:
            viewer.save_tcp({'name': 'bent_tool', 'x_mm': 40., 'y_mm': 20., 'z_mm': 120.,
                             'roll_deg': 1., 'pitch_deg': 2., 'yaw_deg': 3.})
    viewer._plan_client.finish_hook = mutate
    outcome = viewer.plan_tcp(goal(viewer))
    assert not outcome[0]
    assert 'invalidated' in outcome[1]
    assert viewer._plan_record is None
    assert viewer.plan_snapshot()['state'] != 'ready'
    assert all(not preview.trajectory for preview in viewer.previews)


def test_ready_plan_invalidated_by_tcp_edit_and_execute_rechecks_context(viewer):
    result = viewer.plan_tcp(goal(viewer))
    assert result[0]
    viewer.tcps.save('bent_tool', [.1, .2, .3], [0., 0., 0.])  # simulate out-of-band registry change
    assert not viewer.execute({'plan_id': result[2]['plan_id']})[0]
    assert viewer._plan_record is None


def test_tcp_mutations_rejected_during_execution(viewer):
    viewer._plan_status['state'] = 'executing'
    with pytest.raises(ValueError, match='execution'):
        viewer.select_tcp({'name': 'tool0'})
    assert viewer.tcps.snapshot()['selected_tcp'] == 'bent_tool'


def test_tcp_plan_uses_existing_single_use_executor(viewer):
    result = viewer.plan_tcp(goal(viewer))
    assert result[0]
    executed = []
    viewer._execute_trajectory = lambda record: (executed.append(record) is None, 'Executed')
    assert viewer.execute(result[2])[0]
    assert len(executed) == 1
    assert executed[0]['tcp_context']['tcp']['name'] == 'bent_tool'
    assert not viewer.execute(result[2])[0]


def test_joint_plan_still_works_without_fk_and_manual_race_is_rejected(viewer):
    result = viewer.plan(dict.fromkeys(viewer.joint_limits, 0.))
    assert result[0]
    assert not viewer._fk_client.requests
    viewer._plan_client.finish_hook = viewer._invalidate_plan
    assert not viewer.plan(dict.fromkeys(viewer.joint_limits, 10.))[0]
    assert viewer._plan_record is None


def test_external_reference_frame_is_transformed(viewer):
    viewer.reference_frame = 'fixture'
    transform = TransformStamped()
    transform.transform.translation.x = .1
    transform.transform.translation.y = .2
    transform.transform.rotation.z = math.sqrt(.5)
    transform.transform.rotation.w = math.sqrt(.5)
    calls = []
    def lookup(target, source, stamp):
        calls.append((target, source))
        return transform
    viewer._tf_buffer = SimpleNamespace(lookup_transform=lookup)
    assert np.allclose(viewer._target_in_root([.2, .3, .4]), [-.2, .4, .4])
    assert calls == [('world', 'fixture')]


def test_missing_reference_transform_fails_before_planning(viewer):
    from tf2_ros import TransformException
    viewer.reference_frame = 'missing_fixture'
    def lookup(*_):
        raise TransformException('missing')
    viewer._tf_buffer = SimpleNamespace(lookup_transform=lookup)
    result = viewer.plan_tcp(goal(viewer))
    assert not result[0]
    assert 'Cannot transform target' in result[1]
    assert not viewer._plan_client.requests


def test_http_dispatch_exposes_tcp_planning(viewer):
    from arm_web.web_viewer import make_handler
    handler_type = make_handler(viewer, '/unused')
    handler = object.__new__(handler_type)
    handler.path = '/api/plan_tcp'
    result = handler._dispatch(goal(viewer))
    assert result[0]
    assert result[2]['plan_id']


@pytest.mark.parametrize('stop_phase', ['server_wait', 'acceptance'])
def test_stop_blocks_replacement_plan_until_old_execution_finishes(viewer, stop_phase):
    result = viewer.plan_tcp(goal(viewer))
    assert result[0]
    sent, cancelled = [], []

    def stop_and_try_replacement():
        viewer.stop()
        retry = viewer.plan_tcp(goal(viewer))
        assert not retry[0]
        assert 'current execution' in retry[1]
        with pytest.raises(ValueError, match='execution'):
            viewer.select_tcp({'name': 'tool0'})

    def wait_for_server(**_):
        if stop_phase == 'server_wait':
            stop_and_try_replacement()
        return True

    def send(request):
        sent.append(request)
        if stop_phase == 'acceptance':
            stop_and_try_replacement()
        response = SimpleNamespace(result=SimpleNamespace(error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS)))
        return future(SimpleNamespace(
            accepted=True, cancel_goal_async=lambda: cancelled.append(True),
            get_result_async=lambda: future(response)))

    viewer._execute_client = SimpleNamespace(wait_for_server=wait_for_server, send_goal_async=send)
    assert not viewer.execute(result[2])[0]
    assert viewer.plan_snapshot()['state'] == 'stopped'
    assert not viewer._execution_pending
    assert bool(sent) == (stop_phase == 'acceptance')
    assert bool(cancelled) == (stop_phase == 'acceptance')
    assert viewer.plan_tcp(goal(viewer))[0]
