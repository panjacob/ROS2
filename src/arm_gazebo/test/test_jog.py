import math

import pytest

from arm_gazebo.jog import JointJog


URDF = '''<robot name="test">
<joint name="joint1" type="revolute"><limit lower="-1" upper="1" velocity="0.2"/></joint>
<joint name="joint2" type="revolute"><limit lower="-2" upper="2" velocity="0.8"/></joint>
</robot>'''


@pytest.fixture
def jog():
    jog = JointJog(URDF)
    jog.update(['joint1', 'joint2'], [0., 0.], 0.)
    jog.tick(0., 0.)
    return jog


def test_commands_respect_velocity_and_position_limits(jog):
    assert jog.command(['joint1', 'joint2'], [10., -10.], 0.)
    target, hold = jog.tick(.05, .05)
    assert not hold
    assert target == pytest.approx([.2*.12, -.8*.12])
    jog.update(jog.names, [.999, -1.999], .05)
    assert jog.tick(.1, .1)[0] == [1., -2.]


def test_jog_allows_120_degrees_per_second_and_caps_higher_requests():
    jog = JointJog(URDF.replace('velocity="0.2"', 'velocity="5.0"'))
    jog.update(jog.names, [0., 0.], 0.)
    jog.tick(0., 0.)
    assert jog.command(['joint1'], [math.radians(120)], 0.)
    assert jog.tick(.05, .05)[0][0] == pytest.approx(math.radians(120)*.12)
    assert jog.command(['joint1'], [math.radians(240)], .05)
    assert jog.tick(.1, .1)[0][0] == pytest.approx(math.radians(120)*.12)


def test_release_publishes_one_hold_then_leaves_controller_alone(jog):
    jog.command(['joint1'], [.1], 0.)
    jog.update(jog.names, [.01, 0.], .04)
    jog.command(['joint1'], [0.], .05)
    assert jog.tick(.05, .05) == ([.01, 0.], True)
    assert not jog.active
    assert jog.busy(.1)
    assert not jog.busy(.2)
    assert jog.tick(.1, .1) is None


def test_lost_commands_stop_after_watchdog_timeout(jog):
    jog.command(['joint1'], [.1], 0.)
    assert jog.tick(.05, .05)[1] is False
    assert jog.tick(.31, .31) == ([0., 0.], True)
    assert not jog.active
    assert jog.tick(.4, .4) is None


def test_stale_or_incomplete_joint_state_never_generates_motion(jog):
    jog.update(['joint1'], [.9], .5)
    assert not jog.command(['joint1'], [.1], .6)
    jog.update(jog.names, [float('nan'), 0.], .6)
    assert not jog.fresh(.6)
    jog.command(['joint1'], [.1], 0.)
    assert jog.tick(.6, .6) is None
    assert not jog.active


def test_pause_never_accumulates_wall_time_motion(jog):
    jog.command(['joint1'], [.1], 0.)
    assert jog.tick(.05, 0.) is None
    assert jog.tick(.1, 0.) is None
    assert jog.tick(.15, .01)[0][0] == pytest.approx(.012)
    assert jog.tick(.31, .01)[1] is True


def test_clock_reset_stops_an_active_jog(jog):
    jog.command(['joint1'], [.1], 0.)
    jog.tick(.05, 5.)
    assert jog.tick(.1, 0.) == ([0., 0.], True)


def test_stop_requires_release_before_more_jogging(jog):
    jog.command(['joint1'], [.1], 0.)
    assert jog.stop(.05, disarm=True) == [0., 0.]
    assert not jog.command(['joint1'], [.1], .1)
    assert jog.command(['joint1'], [0.], .11)
    assert jog.command(['joint1'], [.1], .12)


@pytest.mark.parametrize('names,values', [
    (['missing'], [.1]), (['joint1'], [float('nan')]),
    (['joint1'], [float('inf')]), (['joint1'], []),
    (['joint1', 'joint1'], [.1, .2]), ([], []),
])
def test_malformed_jog_is_rejected(jog, names, values):
    with pytest.raises(ValueError):
        jog.command(names, values, 0.)
