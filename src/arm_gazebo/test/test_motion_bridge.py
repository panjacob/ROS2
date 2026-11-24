"""Action forwarding and arbitration with controllable ROS client futures."""

import asyncio
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import GoalResponse
from rclpy.task import Future
from sensor_msgs.msg import JointState

from arm_gazebo.jog import JointJog
from arm_gazebo.motion_bridge import MotionBridge
from test_jog import URDF


def ready(value):
    future = Future()
    future.set_result(value)
    return future


class Handle:
    def __init__(self):
        self.request = FollowJointTrajectory.Goal()
        self.is_cancel_requested = False
        self.is_active = True
        self.outcome = None
        self.feedback = []

    def finish(self, state):
        self.outcome = state
        self.is_active = False

    def canceled(self):
        assert self.is_cancel_requested
        self.finish('canceled')

    def abort(self):
        self.finish('aborted')

    def succeed(self):
        self.finish('succeeded')

    def publish_feedback(self, message):
        self.feedback.append(message)


@pytest.fixture
def bridge(monkeypatch):
    import arm_gazebo.motion_bridge as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: 0.)
    bridge = object.__new__(MotionBridge)
    bridge.jog = JointJog(URDF)
    bridge.jog.update(bridge.jog.names, [0., 0.], 0.)
    bridge.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=50_000_000)))
    bridge.reserved = False
    bridge.generation = bridge.reserved_generation = 0
    bridge.cancel_pending = False
    bridge.internal_goal = None
    bridge.published, bridge.reports = [], []
    bridge.publisher = SimpleNamespace(publish=bridge.published.append)
    bridge.status = SimpleNamespace(publish=bridge.reports.append)
    bridge.client = SimpleNamespace(server_is_ready=lambda: True)
    return bridge


def test_public_action_and_jog_cannot_own_controller_together(bridge):
    bridge.jog.command(['joint1'], [.1], 0.)
    assert bridge._goal(None) == GoalResponse.REJECT
    bridge.jog.stop(-1.)
    assert bridge._goal(None) == GoalResponse.ACCEPT
    assert bridge._goal(None) == GoalResponse.REJECT
    bridge._jog(JointState(name=['joint1'], velocity=[.1]))
    bridge._tick()
    assert not bridge.published
    assert bridge.jog.blocked
    bridge._jog(JointState(name=['joint1'], velocity=[0.]))
    assert not bridge.jog.blocked


def test_stop_flushes_jog_even_without_an_action_goal(bridge):
    bridge.jog.command(['joint1'], [.1], 0.)
    bridge.stop()
    assert not bridge.jog.active
    assert bridge.jog.blocked
    assert list(bridge.published[-1].points[0].positions) == [0., 0.]
    assert list(bridge.published[-1].points[0].velocities) == [0., 0.]


@pytest.mark.parametrize('code,status,outcome', [
    (0, GoalStatus.STATUS_SUCCEEDED, 'succeeded'),
    (-4, GoalStatus.STATUS_ABORTED, 'aborted'),
    (0, GoalStatus.STATUS_CANCELED, 'aborted'),
])
def test_plan_forwards_feedback_result_and_releases_reservation(bridge, code, status, outcome):
    handle = Handle()
    assert bridge._goal(handle.request) == GoalResponse.ACCEPT
    sent, cancelled = [], []
    result = FollowJointTrajectory.Result(error_code=code, error_string='test result')
    internal = SimpleNamespace(accepted=True,
        get_result_async=lambda: ready(SimpleNamespace(result=result, status=status)),
        cancel_goal_async=lambda: cancelled.append(True))

    def send(request, feedback_callback):
        sent.append(request)
        feedback_callback(SimpleNamespace(feedback=FollowJointTrajectory.Feedback()))
        return ready(internal)

    bridge.client.send_goal_async = send
    returned = asyncio.run(bridge._execute(handle))
    assert sent == [handle.request]
    assert len(handle.feedback) == 1
    assert returned.error_code == (code if status == GoalStatus.STATUS_SUCCEEDED or code else result.INVALID_GOAL)
    assert handle.outcome == outcome
    assert not bridge.reserved and bridge.internal_goal is None


def test_stop_between_acceptance_and_dispatch_never_sends_trajectory(bridge):
    handle = Handle()
    assert bridge._goal(handle.request) == GoalResponse.ACCEPT
    bridge.stop()
    asyncio.run(bridge._execute(handle))
    assert handle.outcome == 'aborted'
    assert not bridge.reserved


def test_cancel_while_internal_acceptance_is_pending(bridge):
    async def run():
        handle = Handle()
        assert bridge._goal(handle.request) == GoalResponse.ACCEPT
        accepted = asyncio.get_running_loop().create_future()
        bridge.client.send_goal_async = lambda *_, **__: accepted
        pending = asyncio.create_task(bridge._execute(handle))
        await asyncio.sleep(0)
        handle.is_cancel_requested = True
        bridge._cancel(handle)
        cancelled = []
        result = FollowJointTrajectory.Result()
        accepted.set_result(SimpleNamespace(accepted=True,
            cancel_goal_async=lambda: cancelled.append(True),
            get_result_async=lambda: ready(SimpleNamespace(result=result, status=GoalStatus.STATUS_CANCELED))))
        await pending
        assert cancelled == [True]
        assert handle.outcome == 'canceled'
        assert not bridge.reserved

    asyncio.run(run())
