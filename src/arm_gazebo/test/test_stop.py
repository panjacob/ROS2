"""STOP must report controller rejection, unavailability and timeouts honestly."""

import asyncio
from types import SimpleNamespace

import pytest
from action_msgs.srv import CancelGoal
from rclpy.task import Future
from std_srvs.srv import Trigger

from arm_gazebo.scene_manager import SceneManager


@pytest.mark.parametrize('code,success', [(0, True), (1, False), (2, True), (3, True)])
def test_stop_waits_for_cancellation_result(code, success):
    node = object.__new__(SceneManager)
    result = Future()
    result.set_result(CancelGoal.Response(return_code=code))
    calls, destroyed = [], []
    node._cancel = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: (calls.append(request), result)[1])
    node._wall_clock = object()
    node.create_timer = lambda *_, **__: 'timer'
    node.destroy_timer = destroyed.append
    response = asyncio.run(node._stop(Trigger.Request(), Trigger.Response()))
    assert response.success is success
    assert calls[0].goal_info.stamp.sec == 0
    assert not any(calls[0].goal_info.goal_id.uuid)
    assert destroyed == ['timer']


def test_stop_without_controller_is_not_success():
    node = object.__new__(SceneManager)
    node._cancel = SimpleNamespace(service_is_ready=lambda: False)
    response = asyncio.run(node._stop(Trigger.Request(), Trigger.Response()))
    assert not response.success


def test_stop_timeout_is_not_success():
    node = object.__new__(SceneManager)
    node._wall_clock = object()
    node._cancel = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda _: Future())

    def timer(_duration, callback, **_):
        callback()
        return 'timer'

    node.create_timer = timer
    node.destroy_timer = lambda _: None
    response = asyncio.run(node._stop(Trigger.Request(), Trigger.Response()))
    assert not response.success
    assert 'Timed out' in response.message
