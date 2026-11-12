import math

import pytest

from arm_fluidnc_bridge.trajectory import (
    INVALID_GOAL, INVALID_JOINTS, TrajectoryError, allowed_duration, goal_tolerances,
    plan_trajectory, within_tolerance)

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
AXES = ['X', 'Y', 'Z', 'A', 'B']
LIMITS = {name: (-2.0, 2.0, 1.0) for name in JOINTS}
ZERO = [0.0] * 5


def pt(t, positions, **extra):
    return {'time': t, 'positions': positions, **extra}


def plan(points, names=JOINTS, current=ZERO, max_feed=21600.0):
    return plan_trajectory(names, points, JOINTS, AXES, LIMITS, current, 0.02, max_feed)


def code_of(points, **kwargs):
    with pytest.raises(TrajectoryError) as err:
        plan(points, **kwargs)
    return err.value.code


def test_valid_plan_converts_to_degrees_in_axis_order():
    names = ['joint3', 'joint1', 'joint2', 'joint5', 'joint4']
    result = plan([pt(0.0, ZERO), pt(1.0, [0.5, 0.0, 0.0, 0.0, 0.0])], names=names)
    assert result.times == [0.0, 1.0]
    assert result.final == [0.0, 0.0, 0.5, 0.0, 0.0]
    deg = math.degrees(0.5)
    assert result.lines == [f'G1 X0.000 Y0.000 Z{deg:.3f} A0.000 B0.000 F{deg * 60:.1f}']


def test_feed_matches_segment_duration():
    result = plan([pt(0.5, [0.1, 0.1, 0.0, 0.0, 0.0])])
    dist = math.hypot(math.degrees(0.1), math.degrees(0.1))
    assert result.lines[0].endswith(f'F{dist / 0.5 * 60:.1f}')


def test_zero_motion_segment_is_a_dwell():
    result = plan([pt(0.0, ZERO), pt(0.25, ZERO)])
    assert result.lines == ['G4 P0.250']


def test_sample_interpolates():
    result = plan([pt(0.0, ZERO), pt(2.0, [1.0, 0, 0, 0, 0])])
    positions, velocities = result.sample(1.0)
    assert positions[0] == pytest.approx(0.5)
    assert velocities[0] == pytest.approx(0.5)
    assert result.sample(5.0)[0][0] == pytest.approx(1.0)


@pytest.mark.parametrize('names', [
    ['joint1', 'joint2', 'joint3', 'joint4'],
    ['joint1', 'joint2', 'joint3', 'joint4', 'joint6'],
    ['joint1', 'joint1', 'joint3', 'joint4', 'joint5'],
])
def test_wrong_joint_names(names):
    assert code_of([pt(1.0, [0.0] * len(names))], names=names) == INVALID_JOINTS


@pytest.mark.parametrize('points', [
    [],
    [pt(1.0, [0.0] * 4)],
    [pt(1.0, [math.nan, 0, 0, 0, 0])],
    [pt(1.0, [math.inf, 0, 0, 0, 0])],
    [pt(math.nan, ZERO)],
    [pt(-1.0, ZERO)],
    [pt(1.0, ZERO), pt(1.0, ZERO)],
    [pt(1.0, ZERO), pt(0.5, ZERO)],
    [pt(0.0, ZERO), pt(0.0, ZERO)],
    [pt(10.0, [2.5, 0, 0, 0, 0])],
    [pt(0.0, [0.1, 0, 0, 0, 0]), pt(1.0, ZERO)],
    [pt(0.1, [1.0, 0, 0, 0, 0])],
    [pt(1.0, ZERO, velocities=[0.0] * 3)],
    [pt(1.0, ZERO, accelerations=[math.nan] * 5)],
    [pt(1.0, ZERO, effort=[0.0] * 5)],
])
def test_malformed_points_rejected(points):
    assert code_of(points) == INVALID_GOAL


def test_start_tolerance_accepts_small_offset():
    result = plan([pt(0.0, [0.01, 0, 0, 0, 0]), pt(1.0, [0.2, 0, 0, 0, 0])])
    assert result.positions[0][0] == pytest.approx(0.01)


def test_feed_above_max_feed_rejected_not_clamped():
    assert code_of([pt(1.0, [0.5, 0, 0, 0, 0])], max_feed=600.0) == INVALID_GOAL


def test_goal_tolerances():
    tol = goal_tolerances(
        [{'name': 'joint2', 'position': 0.05, 'velocity': 0.0, 'acceleration': 0.0}],
        JOINTS, 0.01)
    assert tol == [0.01, 0.05, 0.01, 0.01, 0.01]
    with pytest.raises(TrajectoryError):
        goal_tolerances([{'name': 'joint9', 'position': 0.1, 'velocity': 0, 'acceleration': 0}],
                        JOINTS, 0.01)
    with pytest.raises(TrajectoryError):
        goal_tolerances([{'name': 'joint1', 'position': -1.0, 'velocity': 0, 'acceleration': 0}],
                        JOINTS, 0.01)


def test_allowed_duration_and_tolerance_check():
    assert allowed_duration(2.0, 0.0, 1.5, 2.0) == pytest.approx(5.0)
    assert allowed_duration(2.0, 0.5, 1.5, 2.0) == pytest.approx(2.5)
    with pytest.raises(TrajectoryError):
        allowed_duration(2.0, -1.0, 1.5, 2.0)
    assert within_tolerance([0.0, 0.1], [0.0, 0.105], [0.01, 0.01]) is None
    index, error = within_tolerance([0.0, 0.1], [0.0, 0.2], [0.01, 0.01])
    assert index == 1 and error == pytest.approx(-0.1)
