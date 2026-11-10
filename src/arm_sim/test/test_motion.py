import math

from arm_sim.motion import JointMotion, joints_from_urdf


URDF = '''<robot name="arm"><link name="base"/><link name="arm"/>
<joint name="joint1" type="revolute"><parent link="base"/><child link="arm"/>
<limit lower="-1" upper="1" velocity="0.5" effort="1"/></joint></robot>'''


def test_command_moves_smoothly_and_respects_urdf_limits():
    motion = JointMotion(joints_from_urdf(URDF))
    motion.command(['joint1'], [2.0])
    assert motion.step(1.0, 1.0)
    assert math.isclose(motion.position['joint1'], 0.5)
    assert motion.step(1.0, 2.0)
    assert math.isclose(motion.position['joint1'], 1.0)
    assert not motion.step(1.0, 3.0)


def test_jog_expires_and_stop_holds_position():
    motion = JointMotion(joints_from_urdf(URDF), jog_timeout=0.3)
    motion.set_jog(['joint1'], [-10.0], now=0.0)
    motion.step(0.2, 0.2)
    assert math.isclose(motion.position['joint1'], -0.1)
    motion.step(0.2, 0.4)
    assert math.isclose(motion.position['joint1'], -0.1)
    motion.command(['joint1'], [0.8])
    motion.step(0.2, 0.6)
    motion.stop()
    held = motion.position['joint1']
    motion.step(1.0, 1.6)
    assert math.isclose(motion.position['joint1'], held)


def test_timed_trajectory_interpolates_and_can_be_stopped():
    motion = JointMotion(joints_from_urdf(URDF))
    motion.start_trajectory(['joint1'], [(0.0, [0.0]), (2.0, [0.5])], now=10.0)
    motion.step(0.1, 11.0)
    assert math.isclose(motion.position['joint1'], 0.25)
    motion.step(0.1, 12.1)
    assert math.isclose(motion.position['joint1'], 0.5)
    assert motion.trajectory_result == 'completed'
    motion.start_trajectory(['joint1'], [(1.0, [-0.5])], now=13.0)
    motion.stop()
    motion.step(1.0, 14.0)
    assert motion.trajectory_result == 'interrupted'
    assert math.isclose(motion.position['joint1'], 0.5)


def test_trajectory_rejects_out_of_bounds_goal():
    motion = JointMotion(joints_from_urdf(URDF))
    try:
        motion.start_trajectory(['joint1'], [(1.0, [2.0])], now=0.0)
    except ValueError as exc:
        assert 'outside URDF limits' in str(exc)
    else:
        raise AssertionError('out-of-bounds trajectory accepted')
