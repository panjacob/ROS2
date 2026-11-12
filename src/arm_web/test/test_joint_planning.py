import math

import pytest

from arm_web.joint_planning import (check_plan_start, joint_limits_from_urdf,
                                    validate_joint_goal)


URDF = '''<robot name="test_arm">
  <joint name="joint1" type="revolute"><limit lower="-1" upper="1"/></joint>
  <joint name="fixed" type="fixed"/>
</robot>'''


def test_parses_movable_joint_limits_and_converts_goal():
    name, limits = joint_limits_from_urdf(URDF)
    assert name == 'test_arm'
    assert limits == {'joint1': (-1.0, 1.0)}
    assert math.isclose(validate_joint_goal({'joint1': 30}, limits)['joint1'],
                        math.pi / 6)


@pytest.mark.parametrize('goal, message', [
    ({'joint1': 90}, 'outside the URDF limits'),
    ({}, 'Missing joints'),
    ({'joint1': 0, 'joint2': 0}, 'Unknown joints'),
    ({'joint1': float('nan')}, 'finite'),
])
def test_invalid_goal_has_user_reason(goal, message):
    _, limits = joint_limits_from_urdf(URDF)
    with pytest.raises(ValueError, match=message):
        validate_joint_goal(goal, limits)


def test_execution_requires_same_start_state():
    check_plan_start({'joint1': 0.01}, {'joint1': 0.0})
    with pytest.raises(ValueError, match='moved since planning'):
        check_plan_start({'joint1': 0.1}, {'joint1': 0.0})
