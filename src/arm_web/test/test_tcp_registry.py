import math

import pytest

from arm_web.tcp_registry import (
    TcpRegistry, quaternion_from_rpy_degrees, rpy_degrees_from_quaternion)


def test_tcp_definitions_survive_restart_and_selection(tmp_path):
    path = tmp_path / 'tcps.json'
    registry = TcpRegistry(path)
    registry.save('gripper_tip', [0.0, 0.0, 0.12], [0.0, 0.0, 90.0])
    registry.save('probe', [0.03, 0.0, 0.08], [0.0, 0.0, 0.0])
    registry.select('gripper_tip')

    reloaded = TcpRegistry(path).snapshot()
    assert reloaded['selected_tcp'] == 'gripper_tip'
    assert {tcp['frame'] for tcp in reloaded['tcps']} == {
        'tool0', 'tcp_gripper_tip', 'tcp_probe'}
    assert next(tcp for tcp in reloaded['tcps'] if tcp['name'] == 'gripper_tip')['xyz_m'][2] == 0.12

    registry.delete('gripper_tip')
    assert TcpRegistry(path).snapshot()['selected_tcp'] == 'tool0'


def test_invalid_tcp_does_not_replace_saved_definition(tmp_path):
    registry = TcpRegistry(tmp_path / 'tcps.json')
    registry.save('probe', [0.0, 0.0, 0.1], [0.0, 0.0, 0.0])
    with pytest.raises(ValueError):
        registry.save('probe', [math.nan, 0.0, 0.2], [0.0, 0.0, 0.0])
    with pytest.raises(ValueError):
        registry.save('tool0', [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    assert TcpRegistry(registry.path).snapshot()['tcps'][1]['xyz_m'][2] == 0.1


def test_rpy_to_quaternion_for_tcp_axis():
    x, y, z, w = quaternion_from_rpy_degrees([0.0, 0.0, 90.0])
    assert math.isclose(x, 0.0, abs_tol=1e-12)
    assert math.isclose(y, 0.0, abs_tol=1e-12)
    assert math.isclose(z, math.sqrt(0.5))
    assert math.isclose(w, math.sqrt(0.5))
    rotation = type('Rotation', (), dict(zip('xyzw', (x, y, z, w))))()
    roll, pitch, yaw = rpy_degrees_from_quaternion(rotation)
    assert math.isclose(roll, 0.0, abs_tol=1e-9)
    assert math.isclose(pitch, 0.0, abs_tol=1e-9)
    assert math.isclose(yaw, 90.0)
