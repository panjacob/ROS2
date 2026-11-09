import math

from arm_fluidnc_bridge.fluidnc_protocol import (
    build_jog, build_move, compute_feed, jog_segment, parse_status)


def test_parse_status_mpos():
    status = parse_status('<Idle|MPos:1.000,-2.500,0.000,0.000,90.000|FS:0,0>')
    assert status['state'] == 'Idle'
    assert status['positions'] == [1.0, -2.5, 0.0, 0.0, 90.0]
    assert status['feed'] == 0.0
    assert status['spindle'] == 0.0
    assert status['pins'] == ''


def test_parse_status_with_substate():
    status = parse_status('<Hold:0|WPos:0.000,0.000,0.000|FS:0,0>')
    assert status['state'] == 'Hold:0'
    assert status['positions'] == [0.0, 0.0, 0.0]


def test_parse_status_feed_and_pins():
    status = parse_status('<Run|MPos:1.0,2.0,0.0|FS:600,120|Pn:XYZ>')
    assert status['state'] == 'Run'
    assert status['feed'] == 600.0
    assert status['spindle'] == 120.0
    assert status['pins'] == 'XYZ'


def test_parse_status_without_pins():
    status = parse_status('<Idle|MPos:0.0,0.0,0.0|FS:0,0>')
    assert status['pins'] == ''


def test_parse_status_feed_only():
    status = parse_status('<Jog|MPos:0.0,0.0,0.0|F:300>')
    assert status['feed'] == 300.0
    assert status['spindle'] is None


def test_parse_status_rejects_other_lines():
    assert parse_status('ok') is None
    assert parse_status('[MSG:INFO: something]') is None


def test_compute_feed():
    # 60 units in 1 s -> 3600 units/min
    assert compute_feed([0, 0], [60, 0], 1.0, 10000) == 3600
    assert compute_feed([0, 0], [60, 0], 1.0, 1000) == 1000
    assert compute_feed([0, 0], [0, 0], 1.0, 500) == 500


def test_build_move():
    assert build_move(['X', 'A'], [1.0, -2.0], 600) == 'G1 X1.000 A-2.000 F600.0'


def test_parse_status_wco_and_planner():
    status = parse_status('<Idle|MPos:10.0,2.0|Bf:13,128|FS:0,0|WCO:4.0,0.0>')
    assert status['pos_type'] == 'MPos'
    assert status['wco'] == [4.0, 0.0]
    assert status['planner_free'] == 13
    assert parse_status('<Idle|WPos:1.0,2.0|FS:0,0>')['pos_type'] == 'WPos'
    assert parse_status('<Idle|MPos:1.0|FS:0,0>')['wco'] is None


def test_build_jog():
    assert build_jog(['X', 'Y', 'Z'], [0.5, 0.0, -0.25], 600) == '$J=G91 X0.5000 Z-0.2500 F600.0'
    assert build_jog(['X', 'Y'], [0.0, 0.00001], 600) is None


def test_jog_segment():
    deltas, feed = jog_segment([10.0, 0.0], 0.05, 3600)
    assert deltas == [0.5, 0.0] and feed == 600.0
    deltas, feed = jog_segment([30.0, 40.0], 0.1, 1200)  # 50 u/s = 3000 u/min -> clamped
    assert feed == 1200
    assert math.isclose(deltas[0], 1.2) and math.isclose(deltas[1], 1.6)
    assert jog_segment([0.0, 0.0], 0.05, 600) == ([0.0, 0.0], 0.0)
