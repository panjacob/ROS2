"""Offline FollowJointTrajectory tests of the bridge against a fake FluidNC (no hardware)."""

import math
import re
import threading
import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from arm_fluidnc_bridge import fluidnc_bridge as bridge_module
from arm_fluidnc_bridge.fluidnc_bridge import FluidNCBridge

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
URDF = '<robot name="test"><link name="base"/>' + ''.join(
    f'<link name="l{i}"/><joint name="joint{i}" type="revolute"><parent link="base"/>'
    f'<child link="l{i}"/><limit lower="-2.0" upper="2.0" velocity="1.0" effort="1"/></joint>'
    for i in range(1, 6)) + '</robot>'
AXIS_WORD = re.compile(r'([XYZAB])(-?\d+\.\d+)')
RESULT = FollowJointTrajectory.Result


class FakeFluidNC:
    """Minimal FluidNC: answers '?', acks lines, queues G1 targets.

    With auto=True one queued move completes per status query; otherwise moves only complete
    when the test calls complete(). Feed hold during Run decelerates over `decel_reports`
    status reports (Run -> Hold:1 ... -> Hold:0), jog cancel likewise (Jog ... -> Idle); hold
    while Idle gives Hold:0 (Grbl behaviour). Soft reset while Run/Jog/Hold:1 raises Alarm
    (abort during cycle). Nothing here models real timing or dynamics.
    """

    MOVING = ('Run', 'Jog', 'Hold:1')

    def __init__(self):
        self.cond = threading.Condition()
        self.out = []
        self.state = 'Idle'
        self.pos = [0.0] * 5
        self.moves = []
        self.lines = []
        self.realtime = []
        self.reset_states = []
        self.auto = True
        self.ack = True
        self.report = True
        self.closed = False
        self.decel_reports = 3
        self.decel_left = 0
        self.decel_to = None
        self.unlock_reply = 'ok'
        self.unlock_clears = True
        self.reject_g1_after = None

    def _emit(self, text):
        self.out.append(text.encode() + b'\n')
        self.cond.notify_all()

    def _status(self):
        if self.decel_to is not None:
            self.decel_left -= 1
            if self.decel_left <= 0:
                self.state, self.decel_to = self.decel_to, None
        elif self.auto and self.moves and self.state == 'Run':
            self.pos = self.moves.pop(0)
            if not self.moves:
                self.state = 'Idle'
        pos = ','.join(f'{v:.3f}' for v in self.pos)
        self._emit(f'<{self.state}|MPos:{pos}|FS:0,0>')

    def _decelerate(self, during, final):
        self.state = during
        self.decel_to = final
        self.decel_left = self.decel_reports

    def write(self, data):
        with self.cond:
            if data == b'?':
                if self.report:
                    self._status()
                return
            if data in (b'!', b'\x18', b'\x85'):
                self.realtime.append(data)
                if data == b'!':
                    if self.state == 'Run':
                        self._decelerate('Hold:1', 'Hold:0')
                    elif self.state == 'Idle':
                        self.state = 'Hold:0'
                elif data == b'\x85':
                    if self.state == 'Jog' and self.decel_to is None:
                        self._decelerate('Jog', 'Idle')
                else:
                    self.reset_states.append(self.state)
                    self.moves.clear()
                    self.decel_to = None
                    if self.state in self.MOVING:
                        self.state = 'Alarm'
                    elif not self.state.startswith('Alarm'):
                        self.state = 'Idle'
                return
            for line in data.decode().splitlines():
                self.lines.append(line)
                reply = 'ok'
                if line.startswith('G1'):
                    target = list(self.moves[-1] if self.moves else self.pos)
                    for axis, value in AXIS_WORD.findall(line):
                        target['XYZAB'.index(axis)] = float(value)
                    self.moves.append(target)
                    if self.state == 'Idle':
                        self.state = 'Run'
                    if self.reject_g1_after is not None and \
                            len(self.g1_lines_locked()) > self.reject_g1_after:
                        reply = 'error:20'
                elif line.startswith('$J=') and self.state == 'Idle':
                    self.state = 'Jog'
                elif line == '$X':
                    reply = self.unlock_reply
                    if reply == 'ok' and self.unlock_clears and self.state.startswith('Alarm'):
                        self.state = 'Idle'
                if self.ack:
                    self._emit(reply)

    def read(self):
        with self.cond:
            if not self.out:
                self.cond.wait(0.02)
            if self.closed:
                raise ConnectionError('closed')
            data, self.out = b''.join(self.out), []
            return data

    def complete(self):
        with self.cond:
            if self.moves:
                self.pos = self.moves[-1]
            self.moves.clear()
            self.decel_to = None
            self.state = 'Idle'

    def close(self):
        with self.cond:
            self.closed = True

    def g1_lines_locked(self):
        return [line for line in self.lines if line.startswith('G1')]

    def g1_lines(self):
        with self.cond:
            return self.g1_lines_locked()


@pytest.fixture
def rig():
    rclpy.init()
    fake = FakeFluidNC()
    bridge = FluidNCBridge(transport=fake, parameter_overrides=[
        Parameter('robot_description', value=URDF),
        Parameter('status_rate', value=50.0),
        Parameter('status_timeout', value=0.5),
        Parameter('ack_timeout', value=1.0),
        Parameter('trajectory_time_scaling', value=1.0),
        Parameter('trajectory_time_margin', value=2.0),
    ])
    client_node = rclpy.create_node('fjt_test_client')
    client = ActionClient(client_node, FollowJointTrajectory,
                          '/arm_controller/follow_joint_trajectory')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    assert client.wait_for_server(timeout_sec=5.0)
    wait_for(lambda: bridge._last_status is not None and bridge._queue.empty()
             and bridge._inflight is None)

    class Rig:
        pass
    r = Rig()
    r.fake, r.bridge, r.client = fake, bridge, client
    yield r
    executor.shutdown()
    client.destroy()
    client_node.destroy_node()
    bridge.destroy_node()
    rclpy.try_shutdown()
    thread.join(timeout=2.0)


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    raise AssertionError('condition not reached')


def wait_future(future, timeout=10.0):
    wait_for(future.done, timeout)
    return future.result()


def make_goal(points, names=JOINTS):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(names)
    for t, positions in points:
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.time_from_start.sec = int(t)
        point.time_from_start.nanosec = int(round((t - int(t)) * 1e9))
        goal.trajectory.points.append(point)
    return goal


SIMPLE = [(0.0, [0.0] * 5), (0.2, [0.1, 0.0, 0.0, 0.0, 0.0]), (0.4, [0.2, 0.05, 0.0, 0.0, 0.0])]


def send(rig, goal):
    handle = wait_future(rig.client.send_goal_async(goal))
    return handle


def run(rig, goal):
    handle = send(rig, goal)
    assert handle.accepted
    return wait_future(handle.get_result_async())


def stop(rig):
    return rig.bridge._on_stop(Trigger.Request(), Trigger.Response())


def unlock(rig):
    return rig.bridge._on_unlock(Trigger.Request(), Trigger.Response())


def test_success_reports_feedback_and_streams_degrees(rig):
    feedback = []
    handle = wait_future(rig.client.send_goal_async(
        make_goal(SIMPLE), feedback_callback=lambda fb: feedback.append(fb.feedback)))
    response = wait_future(handle.get_result_async())
    assert response.status == GoalStatus.STATUS_SUCCEEDED, response.result.error_string
    assert response.result.error_code == RESULT.SUCCESSFUL
    assert 'G21 G90 G94' in rig.fake.lines
    final = rig.fake.g1_lines()[-1]
    assert f'X{math.degrees(0.2):.3f}' in final and f'Y{math.degrees(0.05):.3f}' in final
    assert feedback and list(feedback[-1].joint_names) == JOINTS
    assert len(feedback[-1].error.positions) == 5


def test_reordered_joint_names_map_to_axes(rig):
    names = ['joint2', 'joint1', 'joint3', 'joint4', 'joint5']
    response = run(rig, make_goal([(0.3, [0.1, 0.0, 0.0, 0.0, 0.0])], names))
    assert response.status == GoalStatus.STATUS_SUCCEEDED
    assert f'Y{math.degrees(0.1):.3f}' in rig.fake.g1_lines()[-1]


def test_not_successful_after_ok_until_idle(rig):
    rig.fake.auto = False
    handle = send(rig, make_goal(SIMPLE))
    result = handle.get_result_async()
    wait_for(lambda: len(rig.fake.g1_lines()) == 2 and rig.bridge._queue.empty())
    time.sleep(0.3)  # every line is acked, FluidNC still reports Run
    assert not result.done()
    rig.fake.complete()
    response = wait_future(result)
    assert response.status == GoalStatus.STATUS_SUCCEEDED


def test_final_position_mismatch_aborts_and_disarms(rig):
    rig.fake.auto = False
    handle = send(rig, make_goal(SIMPLE))
    result = handle.get_result_async()
    wait_for(lambda: len(rig.fake.g1_lines()) == 2)
    with rig.fake.cond:
        rig.fake.moves.clear()
        rig.fake.state = 'Idle'  # stopped short at the start (e.g. lost steps)
    response = wait_future(result)
    assert response.status == GoalStatus.STATUS_ABORTED
    assert response.result.error_code == RESULT.GOAL_TOLERANCE_VIOLATED
    assert not rig.bridge._armed


@pytest.mark.parametrize('goal, code', [
    (make_goal([(0.2, [0.1] * 5)], JOINTS[:4] + ['joint9']), RESULT.INVALID_JOINTS),
    (make_goal([(0.2, [0.1] * 4)], JOINTS[:4]), RESULT.INVALID_JOINTS),
    (make_goal([(0.2, [math.nan, 0, 0, 0, 0])]), RESULT.INVALID_GOAL),
    (make_goal([(0.2, [0.1] * 4)]), RESULT.INVALID_GOAL),
    (make_goal([(0.4, [0.1] * 5), (0.2, [0.0] * 5)]), RESULT.INVALID_GOAL),
    (make_goal([(5.0, [3.0, 0, 0, 0, 0])]), RESULT.INVALID_GOAL),
    (make_goal([(0.0, [0.5, 0, 0, 0, 0]), (1.0, [0.6, 0, 0, 0, 0])]), RESULT.INVALID_GOAL),
    (make_goal([(0.1, [1.0, 0, 0, 0, 0])]), RESULT.INVALID_GOAL),
    (make_goal([]), RESULT.INVALID_GOAL),
])
def test_malformed_goals_abort_without_motion(rig, goal, code):
    response = run(rig, goal)
    assert response.status == GoalStatus.STATUS_ABORTED
    assert response.result.error_code == code
    assert rig.fake.g1_lines() == []
    assert rig.bridge._armed


def test_unsupported_tolerances_rejected(rig):
    goal = make_goal(SIMPLE)
    goal.path_tolerance = [JointTolerance(name='joint1', position=0.1)]
    response = run(rig, goal)
    assert response.result.error_code == RESULT.INVALID_GOAL
    assert 'path_tolerance' in response.result.error_string
    assert rig.fake.g1_lines() == []


def test_stale_state_rejected(rig):
    rig.fake.report = False
    time.sleep(0.6)
    response = run(rig, make_goal(SIMPLE))
    assert response.status == GoalStatus.STATUS_ABORTED
    assert 'status' in response.result.error_string
    assert rig.fake.g1_lines() == []


@pytest.mark.parametrize('state', ['Alarm', 'Run', 'Hold:0', 'Home'])
def test_not_idle_rejected(rig, state):
    with rig.fake.cond:
        rig.fake.state = state
    response = run(rig, make_goal(SIMPLE))
    assert response.status == GoalStatus.STATUS_ABORTED
    assert 'Idle' in response.result.error_string
    assert rig.fake.g1_lines() == []


def test_cancel_halts_and_requires_rearm(rig):
    rig.fake.auto = False
    handle = send(rig, make_goal(SIMPLE))
    result = handle.get_result_async()
    wait_for(lambda: rig.fake.g1_lines())
    wait_future(handle.cancel_goal_async())
    response = wait_future(result)
    assert response.status == GoalStatus.STATUS_CANCELED
    assert b'!' in rig.fake.realtime and b'\x18' in rig.fake.realtime
    assert rig.fake.reset_states == ['Hold:0']
    assert not rig.bridge._armed
    assert run(rig, make_goal(SIMPLE)).status == GoalStatus.STATUS_ABORTED


def test_stop_during_execution_flushes_and_rejects_until_rearm(rig):
    rig.fake.auto = False
    rig.fake.ack = False  # keep lines queued in the bridge
    handle = send(rig, make_goal(SIMPLE))
    result = handle.get_result_async()
    wait_for(lambda: rig.fake.lines and rig.fake.lines[-1] == 'G21 G90 G94')
    assert stop(rig).success
    response = wait_future(result)
    assert response.status == GoalStatus.STATUS_ABORTED
    assert 'STOP' in response.result.error_string
    rig.fake.ack = True
    time.sleep(0.3)
    assert rig.fake.g1_lines() == []  # queued segments were never replayed
    wait_for(lambda: rig.bridge._queue.empty() and rig.bridge._inflight is None)
    rejected = run(rig, make_goal(SIMPLE))
    assert rejected.status == GoalStatus.STATUS_ABORTED
    assert 'disarmed' in rejected.result.error_string
    wait_for(lambda: rig.bridge._stops_pending == 0)
    assert unlock(rig).success
    wait_for(lambda: rig.bridge._queue.empty() and rig.bridge._inflight is None)
    rig.fake.auto = True
    assert run(rig, make_goal(SIMPLE)).status == GoalStatus.STATUS_SUCCEEDED


def test_alarm_mid_execution_aborts(rig):
    rig.fake.auto = False
    handle = send(rig, make_goal(SIMPLE))
    result = handle.get_result_async()
    wait_for(lambda: rig.fake.g1_lines())
    with rig.fake.cond:
        rig.fake.state = 'Alarm'
    response = wait_future(result)
    assert response.status == GoalStatus.STATUS_ABORTED
    assert 'Alarm' in response.result.error_string
    assert not rig.bridge._armed


def test_status_lost_mid_execution_aborts(rig):
    rig.fake.auto = False
    handle = send(rig, make_goal(SIMPLE))
    result = handle.get_result_async()
    wait_for(lambda: rig.fake.g1_lines())
    rig.fake.report = False
    response = wait_future(result)
    assert response.status == GoalStatus.STATUS_ABORTED
    assert 'stale' in response.result.error_string


def test_timeout_aborts_and_halts(rig):
    rig.fake.auto = False  # never finishes
    response = run(rig, make_goal(SIMPLE))
    assert response.status == GoalStatus.STATUS_ABORTED
    assert response.result.error_code == RESULT.GOAL_TOLERANCE_VIOLATED
    assert b'!' in rig.fake.realtime


def test_concurrent_goal_rejected(rig):
    rig.fake.auto = False
    first = send(rig, make_goal(SIMPLE))
    assert first.accepted
    second = send(rig, make_goal(SIMPLE))
    assert not second.accepted
    rig.fake.complete()
    assert wait_future(first.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED


def jog_msg(velocity):
    msg = JointState()
    msg.name = ['joint1']
    msg.velocity = [velocity]
    return msg


def test_jog_and_goal_arbitration(rig):
    rig.bridge._on_jog(jog_msg(0.5))
    response = run(rig, make_goal(SIMPLE))
    assert response.status == GoalStatus.STATUS_ABORTED
    assert 'jog' in response.result.error_string
    rig.bridge._on_jog(jog_msg(0.0))
    wait_for(lambda: not rig.bridge._jogging and rig.bridge._queue.empty()
             and rig.bridge._inflight is None and rig.fake.state == 'Idle')
    time.sleep(0.1)

    rig.fake.auto = False
    handle = send(rig, make_goal(SIMPLE))
    wait_for(lambda: rig.fake.g1_lines())
    with rig.fake.cond:
        before = len(rig.fake.lines)
    rig.bridge._on_jog(jog_msg(0.5))
    command = JointState(name=['joint1'], position=[1.0])
    rig.bridge._on_joint_command(command)
    home = rig.bridge._on_home(Trigger.Request(), Trigger.Response())
    assert not home.success
    time.sleep(0.2)
    with rig.fake.cond:
        new = rig.fake.lines[before:]
    assert not any(line.startswith(('$J', '$H')) for line in new)
    assert all(line.startswith('G1') for line in new)
    assert len(rig.fake.g1_lines()) == 2
    rig.fake.complete()
    assert wait_future(handle.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED


def wait_reset(rig):
    wait_for(lambda: rig.fake.reset_states and rig.bridge._stops_pending == 0)
    return list(rig.fake.reset_states)


def start_goal_then(rig, action):
    """Start a goal and interrupt it as soon as FluidNC is moving.

    The bridge's cached state is forced back to 'Idle' to reproduce the window before the next
    status report, when the cache still shows the pre-motion Idle.
    """
    rig.fake.auto = False
    handle = send(rig, make_goal(SIMPLE))
    result = handle.get_result_async()
    wait_for(lambda: rig.fake.state == 'Run', timeout=5.0)
    rig.bridge._state = 'Idle'
    action(handle)
    return wait_future(result)


def test_stop_immediately_after_goal_start_resets_only_when_stopped(rig):
    response = start_goal_then(rig, lambda _h: stop(rig))
    assert response.status == GoalStatus.STATUS_ABORTED
    assert wait_reset(rig) == ['Hold:0']  # never while Run/Hold:1
    assert not rig.fake.state.startswith('Alarm')
    assert not rig.bridge._armed


def test_cancel_immediately_after_goal_start_resets_only_when_stopped(rig):
    response = start_goal_then(rig, lambda h: h.cancel_goal_async())
    assert response.status == GoalStatus.STATUS_CANCELED
    assert rig.fake.reset_states == ['Hold:0']  # cancel waits for the reset before finishing
    assert not rig.fake.state.startswith('Alarm')
    assert not rig.bridge._armed


def test_fault_soon_after_goal_start_resets_only_when_stopped(rig):
    rig.fake.reject_g1_after = 1  # second G1 rejected while the first one runs
    rig.fake.auto = False
    handle = send(rig, make_goal(SIMPLE))
    response = wait_future(handle.get_result_async())
    assert response.status == GoalStatus.STATUS_ABORTED
    assert rig.fake.reset_states == ['Hold:0']


def test_stop_during_goto_waits_for_hold_complete(rig):
    rig.fake.auto = False
    rig.bridge._on_joint_command(JointState(name=['joint1'], position=[1.0]))
    wait_for(lambda: rig.fake.state == 'Run')
    rig.bridge._state = 'Idle'
    assert stop(rig).success
    assert wait_reset(rig) == ['Hold:0']
    assert not rig.fake.state.startswith('Alarm')


def test_stop_during_jog_waits_for_jog_stop(rig):
    rig.bridge._on_jog(jog_msg(0.5))
    wait_for(lambda: rig.fake.state == 'Jog', timeout=1.0)
    rig.bridge._state = 'Idle'
    assert stop(rig).success
    assert wait_reset(rig) == ['Idle']  # after the jog-cancel deceleration, not during it
    assert not rig.fake.state.startswith('Alarm')


def test_stop_not_confirmed_falls_back_to_reset_and_blocks_unlock(rig, monkeypatch):
    monkeypatch.setattr(bridge_module, 'STOP_CONFIRM_TIMEOUT', 0.5)
    messages = []
    publish = rig.bridge._traj_status
    rig.bridge._traj_status = lambda text: (messages.append(text), publish(text))
    rig.fake.auto = False
    rig.fake.decel_reports = 10 ** 9  # never reaches Hold:0
    rig.bridge._on_joint_command(JointState(name=['joint1'], position=[1.0]))
    wait_for(lambda: rig.fake.state == 'Run')
    assert stop(rig).success
    refused = unlock(rig)
    assert not refused.success and 'STOP' in refused.message
    assert wait_reset(rig) == ['Hold:1']  # fallback: reset despite no confirmation
    assert any('not confirmed' in m for m in messages)
    assert not rig.bridge._armed


def test_unlock_arms_only_after_ok_and_non_alarm_status(rig):
    rig.bridge._armed = False
    with rig.fake.cond:
        rig.fake.state = 'Alarm'
    res = unlock(rig)
    assert res.success, res.message
    assert '$X' in rig.fake.lines and rig.bridge._armed
    assert rig.fake.state == 'Idle'


@pytest.mark.parametrize('setup, text', [
    ({'unlock_reply': 'error:9'}, 'acknowledge'),
    ({'ack': False}, 'acknowledge'),
    ({'unlock_clears': False}, 'Alarm'),
])
def test_unlock_failure_stays_disarmed(rig, setup, text):
    rig.bridge._armed = False
    with rig.fake.cond:
        rig.fake.state = 'Alarm'
        for key, value in setup.items():
            setattr(rig.fake, key, value)
    res = unlock(rig)
    assert not res.success
    assert text in res.message
    assert not rig.bridge._armed
    rig.fake.ack = True
    response = run(rig, make_goal(SIMPLE))
    assert response.status == GoalStatus.STATUS_ABORTED
    assert rig.fake.g1_lines() == []


def lines_since(rig, start):
    with rig.fake.cond:
        return list(rig.fake.lines[start:])


def test_jog_held_through_stop_sends_nothing_and_needs_release(rig):
    publishing = threading.Event()
    publishing.set()
    alive = True

    def gamepad():
        while alive:
            if publishing.is_set():
                rig.bridge._on_jog(jog_msg(0.5))
            time.sleep(0.02)

    thread = threading.Thread(target=gamepad, daemon=True)
    thread.start()
    try:
        wait_for(lambda: rig.fake.state == 'Jog', timeout=1.0)
        rig.bridge._state = 'Idle'
        assert stop(rig).success
        with rig.fake.cond:
            start = len(rig.fake.lines)  # lines sent after STOP returned
        assert wait_reset(rig) == ['Idle']  # never reset while jogging
        time.sleep(0.6)  # stick still held after the STOP finished
        assert not any(line.startswith('$J') for line in lines_since(rig, start))
        assert rig.fake.reset_states == ['Idle'] and rig.fake.state == 'Idle'
        assert not rig.bridge._jogging
        publishing.clear()
        rig.bridge._on_jog(jog_msg(0.0))  # release re-enables jogging
        publishing.set()
        wait_for(lambda: rig.fake.state == 'Jog', timeout=1.0)
    finally:
        alive = False
        thread.join(timeout=1.0)
    rig.bridge._on_jog(jog_msg(0.0))
    wait_for(lambda: rig.fake.state == 'Idle' and not rig.bridge._jogging)


def test_jog_after_pause_following_stop_is_accepted(rig):
    rig.bridge._on_jog(jog_msg(0.5))
    wait_for(lambda: rig.fake.state == 'Jog', timeout=1.0)
    assert stop(rig).success
    wait_reset(rig)
    time.sleep(rig.bridge._jog_timeout + 0.05)  # operator let go (web buttons send no zero)
    rig.bridge._on_jog(jog_msg(0.5))
    wait_for(lambda: rig.fake.state == 'Jog', timeout=1.0)
    rig.bridge._on_jog(jog_msg(0.0))
    wait_for(lambda: rig.fake.state == 'Idle' and not rig.bridge._jogging)


def test_motion_sources_refused_while_stop_pending(rig):
    rig.fake.auto = False
    rig.fake.decel_reports = 25  # keep the STOP pending for a while
    rig.bridge._on_joint_command(JointState(name=['joint1'], position=[1.0]))
    wait_for(lambda: rig.fake.state == 'Run')
    with rig.fake.cond:
        start = len(rig.fake.lines)
    assert stop(rig).success
    assert rig.bridge._stops_pending
    rig.bridge._on_joint_command(JointState(name=['joint1'], position=[-1.0]))
    trajectory = JointTrajectory(joint_names=['joint1'])
    point = JointTrajectoryPoint(positions=[0.5])
    point.time_from_start.sec = 1
    trajectory.points.append(point)
    rig.bridge._on_trajectory(trajectory)
    rig.bridge._on_jog(jog_msg(0.5))
    home = rig.bridge._on_home(Trigger.Request(), Trigger.Response())
    home_joint = rig.bridge._on_home_joint('joint1', Trigger.Request(), Trigger.Response())
    refused_unlock = unlock(rig)
    goal = send(rig, make_goal(SIMPLE))
    assert rig.bridge._stops_pending  # all of the above happened during the STOP
    assert not home.success and 'STOP' in home.message
    assert not home_joint.success and 'STOP' in home_joint.message
    assert not refused_unlock.success
    assert not goal.accepted
    assert wait_reset(rig) == ['Hold:0']
    time.sleep(0.3)
    sent = lines_since(rig, start)
    assert not any(line.startswith(('G1', '$J', '$H', '$X')) for line in sent), sent
    assert rig.fake.state == 'Idle'


def test_stop_disarms_atomically_with_generation(rig):
    """Unlock's final arming check (under _arb_lock) can never interleave with STOP's disarm."""
    bridge = rig.bridge
    assert bridge._armed
    generation = bridge._generation
    bridge._arb_lock.acquire()
    try:
        halt = threading.Thread(target=bridge._halt, args=('STOP requested',), daemon=True)
        halt.start()
        time.sleep(0.1)
        # Inside the lock nothing of the STOP is visible yet; an unlock finishing now arms...
        assert bridge._generation == generation and bridge._armed
        assert not bridge._stops_pending
        if bridge._generation == generation:
            bridge._armed = True
    finally:
        bridge._arb_lock.release()
    halt.join(timeout=2.0)
    # ...and the STOP then disarms it with the new generation and STOP pending together.
    assert bridge._generation == generation + 1 and not bridge._armed
    wait_reset(rig)
    assert not bridge._armed
def _held_jog_through_goal_end(rig, end_goal, recover=None):
    """Hold a jog during an active goal, end the goal (cancel/fault) and keep holding."""
    alive = True
    publishing = threading.Event()
    publishing.set()

    def gamepad():
        while alive:
            if publishing.is_set():
                rig.bridge._on_jog(jog_msg(0.5))
            time.sleep(0.02)

    rig.fake.auto = False
    handle = send(rig, make_goal(SIMPLE))
    result = handle.get_result_async()
    wait_for(lambda: rig.fake.g1_lines())
    thread = threading.Thread(target=gamepad, daemon=True)
    thread.start()
    try:
        time.sleep(0.2)  # held while the goal executes
        end_goal(handle)
        response = wait_future(result)
        with rig.fake.cond:
            start = len(rig.fake.lines)  # lines sent after the goal finished
        wait_reset(rig)
        if recover is not None:
            recover()
        time.sleep(0.6)  # stick still held after the goal ended and the STOP finished
        sent = lines_since(rig, start)
        assert not any(line.startswith('$J') for line in sent), sent
        assert not rig.bridge._jogging and rig.fake.state == 'Idle'
        publishing.clear()
        time.sleep(0.05)
        rig.bridge._on_jog(jog_msg(0.0))  # release re-enables jogging
        publishing.set()
        wait_for(lambda: rig.fake.state == 'Jog', timeout=1.0)
    finally:
        alive = False
        thread.join(timeout=1.0)
    rig.bridge._on_jog(jog_msg(0.0))
    wait_for(lambda: rig.fake.state == 'Idle' and not rig.bridge._jogging)
    return response


def test_jog_held_through_goal_cancel_needs_release(rig):
    response = _held_jog_through_goal_end(rig, lambda h: wait_future(h.cancel_goal_async()))
    assert response.status == GoalStatus.STATUS_CANCELED


def test_jog_held_through_goal_fault_needs_release(rig):
    def fault(_handle):
        with rig.fake.cond:
            rig.fake.state = 'Alarm'

    def rearm():
        assert unlock(rig).success  # deliberate unlock never releases a held jog

    response = _held_jog_through_goal_end(rig, fault, rearm)
    assert response.status == GoalStatus.STATUS_ABORTED