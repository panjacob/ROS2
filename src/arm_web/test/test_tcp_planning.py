"""Offline geometry tests: no ROS graph, serial connections or motion."""
import math
from pathlib import Path

import numpy as np
import pytest

from arm_web.joint_planning import joint_limits_from_urdf
from arm_web.tcp_planning import (
    IK_TOLERANCE_M, PointKinematics, transformed_point, validate_tcp_goal)
from arm_web.tcp_registry import TcpRegistry


@pytest.fixture
def arm():
    urdf = (Path(__file__).resolve().parents[2] /
            'arm_description/urdf/arm_concept.urdf').read_text()
    _, limits = joint_limits_from_urdf(urdf)
    return PointKinematics(urdf, limits)


def analytic_concept_point(q, offset):
    """Independent closed-form oracle for the actual arm's Z/Y/Y/Y/Z axes."""
    yaw, shoulder, elbow, wrist, roll = q
    pitch = shoulder + elbow + wrist
    radial = 0.2*math.sin(shoulder) + 0.18*math.sin(shoulder+elbow) + 0.09*math.sin(pitch)
    height = 0.13 + 0.2*math.cos(shoulder) + 0.18*math.cos(shoulder+elbow) + 0.09*math.cos(pitch)
    ox = math.cos(roll)*offset[0] - math.sin(roll)*offset[1]
    oy = math.sin(roll)*offset[0] + math.cos(roll)*offset[1]
    oz = offset[2]
    tilted_x = math.cos(pitch)*ox + math.sin(pitch)*oz
    tilted_z = -math.sin(pitch)*ox + math.cos(pitch)*oz
    return np.array([math.cos(yaw)*(radial+tilted_x)-math.sin(yaw)*oy,
                     math.sin(yaw)*(radial+tilted_x)+math.cos(yaw)*oy,
                     height+tilted_z])


@pytest.mark.parametrize('offset', [[0., 0., 0.], [0.04, -0.025, 0.12], [-0.08, 0.1, 0.]])
def test_actual_urdf_fk_ik_matches_independent_geometry(arm, offset):
    q = [0.3, 0.4, -0.6, 0.7, 1.0]
    target = analytic_concept_point(q, offset)
    assert np.allclose(arm.point(dict(zip(arm.names, q)), offset), target, atol=1e-12)
    solutions = arm.solve(target, offset, dict.fromkeys(arm.names, 0.0), max_solutions=8)
    assert len(solutions) >= 2
    for solution in solutions:
        positions = [solution[name] for name in arm.names]
        assert np.all(np.asarray(positions) >= arm.lower)
        assert np.all(np.asarray(positions) <= arm.upper)
        assert np.linalg.norm(analytic_concept_point(positions, offset)-target) <= IK_TOLERANCE_M


def test_jacobian_includes_rotating_off_axis_point(arm):
    q, offset = np.array([.3, .4, -.6, .7, 1.]), [.04, -.025, .12]
    _, jacobian = arm.point_and_jacobian(q, offset)
    numeric = np.column_stack([
        (arm.point_and_jacobian(q+np.eye(5)[i]*1e-6, offset)[0] -
         arm.point_and_jacobian(q-np.eye(5)[i]*1e-6, offset)[0])/2e-6
        for i in range(5)])
    assert np.allclose(jacobian, numeric, atol=1e-9)
    assert np.linalg.norm(jacobian[:, -1]) > .04


def test_unreachable_and_cancelled_goals_have_no_solution(arm):
    assert arm.solve([10., 0., 0.], [.04, -.025, .12], dict.fromkeys(arm.names, 0.)) == []
    assert arm.solve([.2, .1, .3], [0., 0., 0.], dict.fromkeys(arm.names, 0.), cancelled=lambda: True) == []


def test_fixed_rotated_origin_and_prismatic_joint():
    urdf = '''<robot name="test"><link name="root"/><link name="tool0"/>
    <joint name="slide" type="prismatic"><parent link="root"/><child link="tool0"/>
    <origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/><axis xyz="1 0 0"/>
    <limit lower="-1" upper="1"/></joint></robot>'''
    chain = PointKinematics(urdf, {'slide': (-1., 1.)})
    assert np.allclose(chain.point({'slide': .3}, [.1, .2, .4]), [.8, 2.4, 3.4])


def test_rotated_tcp_axes_do_not_rotate_tcp_origin(tmp_path, arm):
    registry = TcpRegistry(tmp_path/'tcp.json')
    registry.save('bent_tool', [.04, -.025, .12], [37., -61., 83.])
    tcp, target = validate_tcp_goal({'tcp_name': 'bent_tool', 'x_mm': 100., 'y_mm': -50., 'z_mm': 200.}, registry.snapshot())
    assert target == [.1, -.05, .2]
    assert tcp['xyz_m'] == [.04, -.025, .12]
    assert tcp['rpy_deg'] == [37., -61., 83.]
    q = dict(zip(arm.names, [.3, .4, -.6, .7, 1.]))
    assert np.allclose(arm.point(q, tcp['xyz_m']), analytic_concept_point(list(q.values()), tcp['xyz_m']))


@pytest.mark.parametrize('value', [None, True, '12', float('nan'), float('inf'), 10**1000])
def test_invalid_coordinates_rejected(tmp_path, value):
    registry = TcpRegistry(tmp_path/'tcp.json')
    with pytest.raises(ValueError, match='finite number'):
        validate_tcp_goal({'tcp_name': 'tool0', 'x_mm': value, 'y_mm': 0., 'z_mm': 200.}, registry.snapshot())


def test_stale_selection_rejected(tmp_path):
    registry = TcpRegistry(tmp_path/'tcp.json')
    with pytest.raises(ValueError, match='selection changed'):
        validate_tcp_goal({'tcp_name': 'missing', 'x_mm': 0., 'y_mm': 0., 'z_mm': 200.}, registry.snapshot())


def test_reference_transform_rotates_and_translates_position():
    result = transformed_point([1., 2., 3.], [0., 0., math.sqrt(.5), math.sqrt(.5)], [.1, .2, .3])
    assert np.allclose(result, [.8, 2.1, 3.3])


def test_seeded_workspace_targets_with_arbitrary_offsets(arm):
    rng = np.random.default_rng(8429)
    for _ in range(12):
        q, offset = rng.uniform(arm.lower, arm.upper), rng.uniform(-.2, .2, 3)
        target = analytic_concept_point(q, offset)
        solutions = arm.solve(target, offset, dict.fromkeys(arm.names, 0.), max_solutions=1)
        assert solutions
        result = analytic_concept_point([solutions[0][name] for name in arm.names], offset)
        assert np.linalg.norm(result-target) <= IK_TOLERANCE_M
