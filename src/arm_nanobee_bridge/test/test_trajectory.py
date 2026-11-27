import math
import os
import sys

import pytest

from arm_nanobee_bridge.nanobee_protocol import SPEED_MARGIN, parse_settings
from arm_nanobee_bridge.trajectory import (
    TrajectoryError, allowed_duration, goal_tolerances, plan_trajectory, stop_ramp,
    within_tolerance)

sys.path.insert(0, os.path.dirname(__file__))
from fake_nanobee import SETTINGS_YAML  # noqa: E402

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
LIMITS = {j: (-2.0, 2.0, 1.0) for j in JOINTS}
SETTINGS = parse_settings(SETTINGS_YAML.format(blend='true'))
NOBLEND = parse_settings(SETTINGS_YAML.format(blend='false'))
ZERO = [0] * 5
UNREF = [False] * 5


def pts(*items):
    return [{'time': t, 'positions': list(p)} for t, p in items]


def plan(points, names=JOINTS, settings=SETTINGS, current=ZERO, referenced=UNREF, limits=LIMITS):
    return plan_trajectory(names, points, JOINTS, limits, settings, current, referenced, 0.02)


def test_plan_ends_at_final_point_and_ramps_down():
    p = plan(pts((0.0, [0] * 5), (0.5, [0.2, 0, 0, 0, 0]), (1.0, [0.4, 0.1, 0, 0, 0])))
    final = p.segments[-1][0]
    assert final == [SETTINGS.to_steps(0, 0.4), SETTINGS.to_steps(1, 0.1), 0, 0, 0]
    assert len(p.segments) > 2  # last segment split into cruise + deceleration pieces
    assert p.tail > 0.0 and p.execution_time == pytest.approx(1.0 + p.tail)
    total = sum(dt for _s, dt in p.segments) / 1000.0
    assert total == pytest.approx(p.execution_time)


def test_plan_without_blend_has_no_ramp():
    p = plan(pts((0.5, [0.2, 0, 0, 0, 0])), settings=NOBLEND)
    assert len(p.segments) == 1 and p.tail == 0.0


def test_slow_end_needs_no_ramp():
    p = plan(pts((10.0, [0.01, 0, 0, 0, 0])))
    assert len(p.segments) == 1 and p.tail == 0.0


def test_reordered_joint_names():
    names = list(reversed(JOINTS))
    p = plan(pts((1.0, [0.5, 0, 0, 0, 0.1])), names=names)
    assert p.final == [0.1, 0, 0, 0, 0.5]
    assert p.segments[-1][0][4] == SETTINGS.to_steps(4, 0.5)


def test_firmware_speed_limit_rejected():
    fast = {j: (-3.0, 3.0, 100.0) for j in JOINTS}
    with pytest.raises(TrajectoryError, match='max_speed'):
        plan(pts((0.1, [1.0, 0, 0, 0, 0])), limits=fast)  # 573 deg/s > 360 deg/s


def test_urdf_velocity_rejected():
    with pytest.raises(TrajectoryError, match='URDF limit'):
        plan(pts((0.1, [0.5, 0, 0, 0, 0])))


def test_soft_limits_only_when_referenced():
    limits = {j: (-4.0, 4.0, 10.0) for j in JOINTS}
    goal = pts((2.0, [0, 1.8, 0, 0, 0]))  # 103 deg > joint2 max 95 deg
    plan(goal, limits=limits)
    with pytest.raises(TrajectoryError, match='soft limits'):
        plan(goal, limits=limits, referenced=[False, True, False, False, False])


def test_start_tolerance_and_time_order():
    with pytest.raises(TrajectoryError, match='start differs'):
        plan(pts((0.0, [0.1, 0, 0, 0, 0]), (1.0, [0.2, 0, 0, 0, 0])))
    with pytest.raises(TrajectoryError, match='increasing'):
        plan(pts((1.0, [0.1, 0, 0, 0, 0]), (1.0, [0.2, 0, 0, 0, 0])))
    with pytest.raises(TrajectoryError, match='URDF limits'):
        plan(pts((10.0, [2.5, 0, 0, 0, 0])))
    with pytest.raises(TrajectoryError, match='exactly'):
        plan(pts((1.0, [0, 0, 0, 0])), names=JOINTS[:4])


def test_stop_ramp_reaches_target_within_limits():
    start, target = [0] * 5, [1600, -800, 0, 0, 0]
    segments, extra = stop_ramp(start, target, 0.5, SETTINGS)
    assert segments[-1][0] == target
    previous, total = start, 0.0
    speeds = []
    for steps, dt_ms in segments:
        assert dt_ms > 0
        speed = max(abs(b - a) * 1000.0 / dt_ms for a, b in zip(previous, steps))
        assert speed <= SETTINGS.max_speed[0] * SPEED_MARGIN + 1
        speeds.append(speed)
        previous, total = steps, total + dt_ms
    assert speeds[-1] < speeds[0]  # decelerates
    assert total / 1000.0 == pytest.approx(0.5 + extra)
    v = 1600 / 0.5
    assert extra == pytest.approx(v / SETTINGS.max_accel[0] / 2.0)


def test_stop_ramp_short_segment():
    segments, _extra = stop_ramp([0] * 5, [20, 0, 0, 0, 0], 0.004, SETTINGS)
    assert segments[-1][0] == [20, 0, 0, 0, 0]
    assert all(dt > 0 for _s, dt in segments)


def test_helpers():
    tol = goal_tolerances([{'name': 'joint2', 'position': 0.05, 'velocity': 0.0,
                            'acceleration': 0.0}], JOINTS, 0.01)
    assert tol == [0.01, 0.05, 0.01, 0.01, 0.01]
    with pytest.raises(TrajectoryError):
        goal_tolerances([{'name': 'x', 'position': 0, 'velocity': 0, 'acceleration': 0}],
                        JOINTS, 0.01)
    assert allowed_duration(2.0, 0.0, 1.5, 2.0) == 5.0
    assert allowed_duration(2.0, 1.0, 1.5, 2.0) == 3.0
    assert within_tolerance([0, 0.02], [0, 0], [0.01, 0.01]) == (1, 0.02)
    assert within_tolerance([0, math.nextafter(0.01, 0)], [0, 0], [0.01, 0.01]) is None
