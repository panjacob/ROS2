import math
import os
import sys

import pytest

from arm_nanobee_bridge.nanobee_protocol import (
    ProtocolError, config_joint_limits, config_upload, jog_line, move_line, parse_settings,
    parse_status, parse_version, segment_line)

sys.path.insert(0, os.path.dirname(__file__))
from fake_nanobee import LEGACY_SETTINGS_YAML, SETTINGS_YAML  # noqa: E402


def test_parse_status_full():
    s = parse_status('<Run P:10,-20,0,5,7 Q:3 F:13 E:1 L:010 H:10000>')
    assert s['state'] == 'Run'
    assert s['steps'] == [10, -20, 0, 5, 7]
    assert s['queued'] == 3 and s['free'] == 13 and s['enabled']
    assert s['endstops'] == [False, True, False]
    assert s['referenced'] == [True, False, False, False, False]
    assert s['powered'] == [True] * 5  # no O: field: follows E


def test_parse_status_idle_powered_off_joints():
    s = parse_status('<Idle P:0,0,0,0,0 Q:0 F:16 E:1 L:000 H:11111 O:01110>')
    assert s['enabled']
    assert s['referenced'] == [True] * 5
    assert s['powered'] == [False, True, True, True, False]


def test_parse_status_without_optional_fields():
    s = parse_status('<Idle P:0,0,0,0,0 Q:0 E:0 L:000>')
    assert s['free'] is None and not s['enabled']
    assert s['referenced'] == [False] * 5
    assert s['powered'] == [False] * 5


@pytest.mark.parametrize('line', ['ok', '<Idle|MPos:0,0,0>', '# homed 0', ''])
def test_parse_status_rejects_other_lines(line):
    assert parse_status(line) is None


def test_parse_version():
    assert parse_version('NanoBee 2 joints=5') == 2
    with pytest.raises(ProtocolError):
        parse_version('NanoBee 1 joints=5')
    with pytest.raises(ProtocolError):
        parse_version('NanoBee 2 joints=4')
    with pytest.raises(ProtocolError):
        parse_version('[VER:3.7 FluidNC]')


@pytest.mark.parametrize('dump', [SETTINGS_YAML, LEGACY_SETTINGS_YAML], ids=['per_joint', 'lists'])
def test_parse_settings_from_firmware_dump(dump):
    s = parse_settings(dump.format(blend='true'))
    assert s.steps_per_deg == [17.7778] * 5
    assert s.max_speed == [6400.0] * 5 and s.min_speed == 100.0
    assert s.homing == ['center', 'none', 'none', 'none', 'none']
    assert s.min_deg == [-175.0, -95.0, -140.0, -105.0, -185.0]
    assert s.soft_limits and s.blend and s.homing_enabled
    assert s.limit_steps(0) == (-3111, 3111)
    assert s.to_steps(0, math.radians(90)) == 1600
    assert s.to_rad(0, 1600) == pytest.approx(math.radians(90), abs=1e-4)
    assert not parse_settings(dump.format(blend='false')).blend


def test_parse_settings_per_joint_missing_key():
    text = SETTINGS_YAML.format(blend='true').replace('    max_speed: 6400\n', '', 1)
    with pytest.raises(ProtocolError, match='joints.joint1.max_speed'):
        parse_settings(text)


@pytest.mark.parametrize('text', [
    '', 'joints: 3', 'joints:\n  steps_per_deg: [1, 2]\n', ': :',
    'joints:\n  joint1:\n    steps_per_deg: 1\n'])
def test_parse_settings_rejects_bad_dumps(text):
    with pytest.raises(ProtocolError):
        parse_settings(text)


def test_line_builders():
    assert move_line([1, 2, 3, 4, 5]) == 'M 1 2 3 4 5'
    assert move_line([1, 2, 3, 4, 5], 100) == 'M 1 2 3 4 5 100.0'
    assert segment_line([1, -2, 3, 4, 5], 12.5) == 'T 1 -2 3 4 5 12.500'
    assert jog_line([0, 1.25, 0, 0, -3]) == 'J 0.0 1.2 0.0 0.0 -3.0'


def test_config_upload():
    assert config_upload('a:\r\n  b: 1  # c\n\n') == 'CONFIG\na:\n  b: 1  # c\nEND'
    for bad in ('', '\n \n', 'a:\n\tb: 1', 'a: \u00b0', 'END', 'a: ' + 'x' * 300):
        with pytest.raises(ProtocolError):
            config_upload(bad)


def test_config_joint_limits_blocks_lists_and_defaults():
    blocks = 'joints:\n' + ''.join(
        f'  joint{j}:\n    min_deg: {-10 * j}\n    max_deg: {10 * j}\n' for j in range(1, 5))
    blocks += '  joint5:\n    steps_per_deg: 1\n'
    assert config_joint_limits(blocks) == [(-10., 10.), (-20., 20.), (-30., 30.), (-40., 40.),
                                           (-180., 180.)]
    lists = 'joints:\n  min_deg: [-1, -2, -3, -4, -5]\n  max_deg: [1, 2, 3, 4, 5.5]\n'
    assert config_joint_limits(lists)[4] == (-5., 5.5)
    assert config_joint_limits('motion: {}\n') == [(-180., 180.)] * 5


def test_config_joint_limits_rejects_bad_values():
    for text in ('joints: [\n', 'joints:\n  joint2:\n    min_deg: abc\n',
                 'joints:\n  joint3:\n    min_deg: 10\n    max_deg: 10\n',
                 'joints:\n  joint1:\n    max_deg: .inf\n'):
        with pytest.raises(ProtocolError):
            config_joint_limits(text)


def test_config_joint_limits_reads_workspace_config():
    path = os.path.join(os.path.dirname(__file__), '../../../NanoBee/NanoBeeDriver/config.yaml')
    if not os.path.isfile(path):
        pytest.skip('NanoBee/NanoBeeDriver/config.yaml not in this checkout')
    with open(path, encoding='utf-8') as f:
        limits = config_joint_limits(f.read())
    assert len(limits) == 5 and all(lo < hi for lo, hi in limits)