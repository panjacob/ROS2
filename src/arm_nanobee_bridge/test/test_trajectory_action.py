"""Offline tests of the bridge node against a fake NanoBee board (no hardware)."""

import math
import os
import sys
import threading
import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from arm_nanobee_bridge.nanobee_bridge import NanoBeeBridge

sys.path.insert(0, os.path.dirname(__file__))
from fake_nanobee import FakeNanoBee  # noqa: E402

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
URDF = '<robot name="test"><link name="base"/>' + ''.join(
    f'<link name="l{i}"/><joint name="joint{i}" type="revolute"><parent link="base"/>'
    f'<child link="l{i}"/><limit lower="-2.0" upper="2.0" velocity="1.0" effort="1"/></joint>'
    for i in range(1, 6)) + '</robot>'
RESULT = FollowJointTrajectory.Result
SIMPLE = [(0.0, [0.0] * 5), (0.2, [0.1, 0.0, 0.0, 0.0, 0.0]), (0.4, [0.2, 0.05, 0.0, 0.0, 0.0])]


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


@pytest.fixture
def rig():
    rclpy.init()
    fake = FakeNanoBee()
    bridge = NanoBeeBridge(transport=fake, parameter_overrides=[
        Parameter('robot_description', value=URDF),
        Parameter('status_rate', value=50.0),
        Parameter('status_timeout', value=0.5),
        Parameter('trajectory_time_scaling', value=1.0),
        Parameter('trajectory_time_margin', value=2.0),
        Parameter('jog_timeout', value=0.2),
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
    wait_for(lambda: bridge._fresh_status() is not None)

    class Rig:
        pass
    r = Rig()
    r.fake, r.bridge, r.client, r.node = fake, bridge, client, client_node
    yield r
    executor.shutdown()
    client.destroy()
    client_node.destroy_node()
    bridge.destroy_node()
    fake.close()
    rclpy.try_shutdown()
    thread.join(timeout=2.0)


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


def send(rig, goal, **kwargs):
    return wait_future(rig.client.send_goal_async(goal, **kwargs))


def run(rig, goal):
    handle = send(rig, goal)
    assert handle.accepted
    return wait_future(handle.get_result_async())


def call(method, request=None):
    return method(request or Trigger.Request(), Trigger.Response())


def steps(rad):
    return round(rad * 57.29577951308232 * 17.7778)


def test_connect_reads_board_settings(rig):
    assert rig.bridge._settings.homing[0] == 'center'
    assert 'VER' in rig.fake.lines and 'SETTINGS?' in rig.fake.lines
    services = dict(rig.node.get_service_names_and_types())
    assert '/nanobee_bridge/home_joint1' in services
    assert '/nanobee_bridge/home_joint2' in services


def test_direct_move_respects_per_joint_speed_limits(rig):
    rig.bridge._on_joint_command(JointState(
        name=['joint1', 'joint2'], position=[math.radians(10), math.radians(5)],
        velocity=[math.radians(3.6), math.radians(1.0)]))
    line = next(line for line in rig.fake.lines if line.startswith('M '))
    parts = line.split()
    distance1, distance2 = map(int, parts[1:3])
    # Joint2 is the bottleneck, despite moving half as far.
    expected = 17.7778 * distance1 / distance2
    assert float(parts[-1]) == pytest.approx(expected, abs=1e-5)
    assert float(parts[-1]) < 64


@pytest.mark.parametrize('percent', [1, 50, 100])
def test_direct_move_speed_percentage(rig, percent):
    rig.bridge._on_joint_command(JointState(
        name=['joint1'], position=[math.radians(10)],
        velocity=[math.radians(6400 / 17.7778 * percent / 100)]))
    line = next(line for line in rig.fake.lines if line.startswith('M '))
    assert float(line.split()[-1]) == pytest.approx(6400 * percent / 100)


def test_direct_move_rejects_invalid_velocity_limits(rig):
    count = len(rig.fake.lines)
    for velocities in ([0.0], [-1.0], [float('nan')], [1.0, 2.0]):
        rig.bridge._on_joint_command(JointState(
            name=['joint1'], position=[0.1], velocity=velocities))
    assert not any(line.startswith('M ') for line in rig.fake.lines[count:])


def test_success_streams_segments_and_reports_feedback(rig):
    feedback = []
    handle = send(rig, make_goal(SIMPLE), feedback_callback=lambda fb: feedback.append(fb))
    response = wait_future(handle.get_result_async())
    assert response.status == GoalStatus.STATUS_SUCCEEDED, response.result.error_string
    assert response.result.error_code == RESULT.SUCCESSFUL
    lines = rig.fake.t_lines()
    assert len(lines) >= 2
    assert lines[-1].split()[1:6] == [str(steps(0.2)), str(steps(0.05)), '0', '0', '0']
    assert rig.fake.pos[:2] == [steps(0.2), steps(0.05)]
    assert feedback and list(feedback[-1].feedback.joint_names) == JOINTS


def test_full_queue_is_retried(rig):
    rig.fake.capacity = 1
    points = [(0.0, [0.0] * 5)] + [(0.1 * k, [0.02 * k, 0, 0, 0, 0]) for k in range(1, 8)]
    response = run(rig, make_goal(points))
    assert response.status == GoalStatus.STATUS_SUCCEEDED, response.result.error_string
    assert len(rig.fake.t_lines()) >= 7


def test_board_refusal_aborts_and_disarms(rig):
    rig.fake.replies['T'] = 'err limit'
    response = run(rig, make_goal(SIMPLE))
    assert response.status == GoalStatus.STATUS_ABORTED
    assert 'err limit' in response.result.error_string
    assert 'S' in rig.fake.lines and not rig.bridge._armed
    del rig.fake.replies['T']
    response = run(rig, make_goal(SIMPLE))
    assert response.status == GoalStatus.STATUS_ABORTED
    assert 'disarmed' in response.result.error_string
    assert call(rig.bridge._on_unlock).success
    assert run(rig, make_goal(SIMPLE)).status == GoalStatus.STATUS_SUCCEEDED


def test_invalid_goal_sends_nothing(rig):
    response = run(rig, make_goal([(0.1, [1.5, 0, 0, 0, 0])]))  # 15 rad/s > URDF 1 rad/s
    assert response.status == GoalStatus.STATUS_ABORTED
    assert response.result.error_code == RESULT.INVALID_GOAL
    assert rig.fake.t_lines() == []
    assert rig.bridge._armed


def test_cancel_stops_and_disarms(rig):
    rig.fake.auto = False
    handle = send(rig, make_goal(SIMPLE))
    wait_for(lambda: rig.fake.t_lines())
    wait_future(handle.cancel_goal_async())
    response = wait_future(handle.get_result_async())
    assert response.status == GoalStatus.STATUS_CANCELED
    assert 'S' in rig.fake.lines and not rig.bridge._armed


def test_board_leaving_run_aborts(rig):
    rig.fake.auto = False
    handle = send(rig, make_goal(SIMPLE))
    wait_for(lambda: rig.fake.t_lines())
    with rig.fake.cond:
        rig.fake.state = 'Jog'  # firmware underrun: segment stream ran dry
    response = wait_future(handle.get_result_async())
    assert response.status == GoalStatus.STATUS_ABORTED
    assert 'Jog' in response.result.error_string


def test_timeout_aborts(rig):
    rig.fake.auto = False
    response = run(rig, make_goal(SIMPLE))
    assert response.status == GoalStatus.STATUS_ABORTED
    assert response.result.error_code == RESULT.GOAL_TOLERANCE_VIOLATED
    assert 'S' in rig.fake.lines


def test_not_idle_rejected(rig):
    with rig.fake.cond:
        rig.fake.state = 'Home'
    wait_for(lambda: rig.bridge._state == 'Home')
    response = run(rig, make_goal(SIMPLE))
    assert response.status == GoalStatus.STATUS_ABORTED
    assert rig.fake.t_lines() == []


def test_home_services(rig):
    res = rig.bridge._on_home(None, Trigger.Response())
    assert res.success, res.message
    assert 'H' in rig.fake.lines
    wait_for(lambda: rig.bridge._state == 'Home')
    assert not rig.bridge._on_home(0, Trigger.Response()).success
    with rig.fake.cond:
        rig.fake.state = 'Idle'
        rig.fake.referenced[0] = True
    rig.fake.emit('# homed 0')
    wait_for(lambda: rig.bridge._state == 'Idle')
    assert rig.bridge._on_home(0, Trigger.Response()).success
    assert 'H 0' in rig.fake.lines
    res = rig.bridge._on_home(1, Trigger.Response())
    assert not res.success and 'no homing' in res.message
    assert 'H 1' not in rig.fake.lines


def test_motors_and_set_zero(rig):
    res = rig.bridge._on_motors(SetBool.Request(data=False), SetBool.Response())
    assert res.success and 'E 0' in rig.fake.lines
    with rig.fake.cond:
        rig.fake.pos = [100, 200, 300, 400, 500]
        rig.fake.referenced = [True, False, False, False, False]
    wait_for(lambda: rig.bridge._fresh_status()['steps'][0] == 100)
    assert call(rig.bridge._on_set_zero).success
    assert 'Z 100 0 0 0 0' in rig.fake.lines


def test_set_zero_single_joint(rig):
    services = dict(rig.node.get_service_names_and_types())
    assert '/nanobee_bridge/set_zero_joint3' in services
    with rig.fake.cond:
        rig.fake.pos = [100, 200, 300, 400, 500]
        rig.fake.referenced = [True, False, False, False, False]
    wait_for(lambda: rig.bridge._fresh_status()['steps'][2] == 300)
    res = rig.bridge._on_set_zero_joint(0, Trigger.Response())
    assert res.success, res.message
    assert 'Z 0 200 300 400 500' in rig.fake.lines
    with rig.fake.cond:
        rig.fake.state = 'Run'
    wait_for(lambda: rig.bridge._fresh_status()['state'] == 'Run')
    count = len(rig.fake.lines)
    res = rig.bridge._on_set_zero_joint(2, Trigger.Response())
    assert not res.success and 'Idle' in res.message
    assert not any(line.startswith('Z') for line in rig.fake.lines[count:])
    with rig.fake.cond:
        rig.fake.state = 'Idle'
    wait_for(lambda: rig.bridge._fresh_status()['state'] == 'Idle')


def test_motor_status_topics(rig):
    from std_msgs.msg import Bool, String
    enabled, powered = [], []
    rig.node.create_subscription(Bool, '/nanobee_bridge/motors_enabled',
                                 lambda m: enabled.append(m.data), 10)
    rig.node.create_subscription(String, '/nanobee_bridge/motors_powered',
                                 lambda m: powered.append(m.data), 10)
    with rig.fake.cond:
        rig.fake.powered = [True, False, True, True, True]
    wait_for(lambda: enabled and enabled[-1] and powered
             and powered[-1] == 'joint1,joint3,joint4,joint5')
    assert rig.bridge._on_motors(SetBool.Request(data=False), SetBool.Response()).success
    wait_for(lambda: not enabled[-1] and powered[-1] == '')


@pytest.mark.parametrize('enable_on_start', [True, False])
def test_enable_on_start(enable_on_start):
    rclpy.init()
    fake = FakeNanoBee()
    fake.enabled = False  # board just rebooted: drivers off
    try:
        bridge = NanoBeeBridge(transport=fake, parameter_overrides=[
            Parameter('enable_on_start', value=enable_on_start)])
        assert ('E 1' in fake.lines) == enable_on_start
        assert fake.enabled == enable_on_start
        bridge.destroy_node()
    finally:
        fake.close()
        rclpy.try_shutdown()


def test_enable_on_start_skips_enabled_board():
    rclpy.init()
    fake = FakeNanoBee()
    try:
        NanoBeeBridge(transport=fake).destroy_node()
        assert 'E 1' not in fake.lines
    finally:
        fake.close()
        rclpy.try_shutdown()


def test_push_config_uploads_file_and_reloads(rig, tmp_path):
    assert not call(rig.bridge._on_push_config).success  # config_path not set
    path = tmp_path / 'config.yaml'
    path.write_text('# comment\r\nmotion:\r\n  blend: true   # keep\r\n\r\n')
    rig.bridge._config_path = str(path)
    reloads = rig.fake.lines.count('SETTINGS?')
    res = call(rig.bridge._on_push_config)
    assert res.success, res.message
    assert rig.fake.config_yaml == '# comment\nmotion:\n  blend: true   # keep\n'
    assert rig.fake.lines.count('SETTINGS?') == reloads + 1

    rig.fake.replies['CONFIG'] = 'err line 2: bad or unknown setting'
    res = call(rig.bridge._on_push_config)
    assert not res.success and 'bad or unknown setting' in res.message


def test_push_config_refused_without_sending(rig, tmp_path):
    path = tmp_path / 'config.yaml'
    rig.bridge._config_path = str(path)
    assert not call(rig.bridge._on_push_config).success  # missing file
    path.write_text('motion:\n\tblend: true\n')
    res = call(rig.bridge._on_push_config)
    assert not res.success and 'tabs' in res.message
    path.write_text('motion:\n  blend: true\n')
    with rig.fake.cond:
        rig.fake.state = 'Run'
    wait_for(lambda: rig.bridge._fresh_status()['state'] == 'Run')
    assert not call(rig.bridge._on_push_config).success
    assert 'CONFIG' not in rig.fake.lines


def test_jog_sends_steps_and_stops_when_stale(rig):
    msg = JointState(name=['joint2'], velocity=[0.5])
    rig.bridge._on_jog(msg)
    wait_for(lambda: any(line.startswith('J ') for line in rig.fake.lines))
    jog = next(line for line in rig.fake.lines if line.startswith('J '))
    assert float(jog.split()[2]) == pytest.approx(0.5 * 57.29578 * 17.7778, abs=0.2)
    wait_for(lambda: rig.fake.lines[-1] == 'J 0.0 0.0 0.0 0.0 0.0')
    assert not rig.bridge._jogging


def test_jog_held_through_stop_needs_release(rig):
    msg = JointState(name=['joint1'], velocity=[0.3])
    rig.bridge._on_jog(msg)
    wait_for(lambda: rig.bridge._jogging)
    assert call(rig.bridge._on_stop).success
    count = len(rig.fake.lines)
    for _ in range(10):
        rig.bridge._on_jog(msg)
        time.sleep(0.03)
    moving = [line for line in rig.fake.lines[count:]
              if line.startswith('J ') and line != 'J 0.0 0.0 0.0 0.0 0.0']
    assert moving == []
    # Stopped: even a released-and-pressed jog stays refused until START.
    rig.bridge._on_jog(JointState(name=['joint1'], velocity=[0.0]))
    rig.bridge._on_jog(msg)
    time.sleep(0.1)
    assert not rig.bridge._jogging
    assert call(rig.bridge._on_unlock).success
    # The stick held through START must be released first.
    rig.bridge._on_jog(msg)
    time.sleep(0.1)
    assert not rig.bridge._jogging
    rig.bridge._on_jog(JointState(name=['joint1'], velocity=[0.0]))
    rig.bridge._on_jog(msg)
    wait_for(lambda: rig.bridge._jogging)


def test_stop_refuses_all_motion_until_unlock(rig):
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import Bool
    armed = []
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    rig.node.create_subscription(Bool, '/nanobee_bridge/armed',
                                 lambda m: armed.append(m.data), latched)
    wait_for(lambda: armed and armed[-1])
    assert call(rig.bridge._on_stop).success
    wait_for(lambda: not armed[-1])
    count = len(rig.fake.lines)
    rig.bridge._on_joint_command(JointState(name=['joint1'], position=[0.5]))
    res = rig.bridge._on_home(None, Trigger.Response())
    assert not res.success and 'START' in res.message
    rig.bridge._on_jog(JointState(name=['joint1'], velocity=[0.3]))
    time.sleep(0.1)
    sent = rig.fake.lines[count:]
    assert not any(line.startswith(('M', 'H', 'T')) for line in sent)
    assert not any(line.startswith('J ') and line != 'J 0.0 0.0 0.0 0.0 0.0' for line in sent)
    # Non-motion services still work while stopped.
    assert rig.bridge._on_motors(SetBool.Request(data=False), SetBool.Response()).success
    assert call(rig.bridge._on_unlock).success
    wait_for(lambda: armed[-1])
    rig.bridge._on_joint_command(JointState(name=['joint1'], position=[0.5]))
    wait_for(lambda: any(line.startswith('M') for line in rig.fake.lines[count:]))


def test_jog_refused_during_goal(rig):
    rig.fake.auto = False
    handle = send(rig, make_goal(SIMPLE))
    wait_for(lambda: rig.fake.t_lines())
    rig.bridge._on_jog(JointState(name=['joint1'], velocity=[0.3]))
    time.sleep(0.1)
    assert not any(line.startswith('J ') for line in rig.fake.lines)
    rig.fake.complete()
    assert wait_future(handle.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED


def test_board_restart_disarms(rig):
    rig.fake.emit('# NanoBee ready')
    wait_for(lambda: not rig.bridge._armed)
    wait_for(lambda: rig.fake.lines.count('SETTINGS?') >= 2)
